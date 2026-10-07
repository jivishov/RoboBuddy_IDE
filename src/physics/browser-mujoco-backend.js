import { assertPhysicalScene, assertPhysicsBackend, MAX_ADVANCE_STEPS_PER_REQUEST, MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE, PHYSICS_BACKEND_API_VERSION, PhysicsBackendState, sampledObservationCount } from './backend-contract.js';
import { requireModelPackage } from './model-registry.js';
import './model-packages.js';

const MAX_TRACE_EVENTS = 512;
const MAX_STEP_BATCH = MAX_ADVANCE_STEPS_PER_REQUEST;
const STEP_TIME_TOLERANCE_SECONDS = 1e-9;
const WORLD_FRAME = Object.freeze({ id: 'mujoco_world', handedness: 'right-handed', upAxis: '+Z', linearUnit: 'm', angularUnit: 'rad' });

function exactStringArray(actual, expected) {
  return Array.isArray(actual) && actual.length === expected.length && actual.every((value, index) => value === expected[index]);
}
function objectIds(items) { return Array.isArray(items) ? items.map((item) => item?.id) : []; }
function exactOptionalNumber(actual, expected) { return (actual ?? null) === (expected ?? null); }
function assertSceneMatchesPackage(scene, modelPackage) {
  if (scene.robotId !== modelPackage.robotId) throw new Error(`Scene robotId ${scene.robotId} does not match model package ${modelPackage.robotId}`);
  if (Math.abs(Number(scene.physics.timestepSeconds) - Number(modelPackage.physics.timestepSeconds)) > 1e-12) throw new Error(`Scene timestep ${scene.physics.timestepSeconds} does not match model package ${modelPackage.physics.timestepSeconds}`);
  if (scene.physics.integrator !== modelPackage.physics.integrator) throw new Error(`Scene integrator ${scene.physics.integrator} does not match model package ${modelPackage.physics.integrator}`);
  if (!exactOptionalNumber(scene.physics.iterations, modelPackage.physics.iterations)) throw new Error(`Scene solver iterations ${scene.physics.iterations ?? 'default'} do not match model package ${modelPackage.physics.iterations ?? 'default'}`);
  if (!exactOptionalNumber(scene.physics.lsIterations, modelPackage.physics.lsIterations)) throw new Error(`Scene solver line-search iterations ${scene.physics.lsIterations ?? 'default'} do not match model package ${modelPackage.physics.lsIterations ?? 'default'}`);
  if (!exactStringArray(scene.controllers || [], modelPackage.controllers)) throw new Error('Scene controller set does not match the registered model package');
  if (modelPackage.sceneConstraints?.fixtures && !exactStringArray(objectIds(scene.fixtures), modelPackage.sceneConstraints.fixtures)) throw new Error('Scene fixture identities do not match the registered model package');
  if (modelPackage.sceneConstraints?.objects && !exactStringArray(objectIds(scene.objects), modelPackage.sceneConstraints.objects)) throw new Error('Scene object identities do not match the registered model package');
}

export class BrowserMuJoCoBackend {
  // `setupOperations` opts a backend into the declared setup path: a small, explicitly
  // allowlisted set of operations that may write physical state directly. It exists so a
  // pre-trial perturbation - start the robot face-down, place an object, cut motor torque -
  // is a separate, logged operation rather than something smuggled through ordinary control.
  // Backends that do not declare it reject setup entirely, so no robot gains a state-writing
  // path by accident.
  constructor({ workerUrl = new URL('./mujoco-worker.js', import.meta.url), setupOperations = [] } = {}) {
    this.setupOperations = new Set(setupOperations);
    this.workerUrl = workerUrl; this.worker = null; this.sequence = 0; this.pending = new Map(); this.loaded = false; this.disposed = false;
    this.sceneRevision = null; this.robotId = null; this.modelPackageId = null; this.lastObservation = null; this.lastScene = null;
    this.sessionId = null; this.epoch = 0; this.state = PhysicsBackendState.IDLE; this.trace = []; this.generation = 0; this.commandBudget = null;
  }

  async loadScene(scene = {}, context = {}) {
    this.#assertNotDisposed(); assertPhysicalScene(scene); this.#validateNewEpoch(context);
    this.sessionId = String(context.sessionId); this.epoch = context.epoch;
    const generation = ++this.generation;
    this.#invalidateLoadedScene(PhysicsBackendState.LOADING);
    try {
      const modelPackage = requireModelPackage(scene.modelPackage); assertSceneMatchesPackage(scene, modelPackage);
      if (!this.worker) this.#spawn();
      const raw = await this.#call('load', { modelPackage }); this.#assertGeneration(generation, context);
      this.loaded = true; this.sceneRevision = String(scene.revision); this.robotId = String(scene.robotId); this.modelPackageId = modelPackage.id;
      const observation = this.#decorateObservation(raw);
      if (observation.model.id !== (modelPackage.modelId || modelPackage.id) || observation.model.sha256 !== modelPackage.sha256) throw new Error('Loaded MuJoCo model identity does not match the registered model package');
      this.lastScene = { schemaVersion: scene.schemaVersion, id: scene.id, revision: this.sceneRevision, robotId: this.robotId, modelPackage: scene.modelPackage, physics: structuredClone(scene.physics), controllers: structuredClone(scene.controllers || []), model: structuredClone(observation.model) };
      this.lastObservation = observation; this.state = PhysicsBackendState.READY; this.#record('loadScene', { observation });
      return { sceneRevision: this.sceneRevision, robotId: this.robotId, observation };
    } catch (error) {
      if (generation === this.generation) this.#failLoadedScene('load failed');
      throw error;
    }
  }

  async reset(context = {}) {
    this.#assertLoaded(); this.#adoptNewEpoch(context); this.commandBudget = null; const generation = ++this.generation;
    try {
      const raw = await this.#call('reset'); this.#assertGeneration(generation, context); const observation = this.#decorateObservation(raw);
      this.lastObservation = observation; this.state = PhysicsBackendState.READY; this.#record('reset', { observation }); return observation;
    } catch (error) {
      if (generation === this.generation && this.loaded) this.#failLoadedScene('reset failed');
      throw error;
    }
  }

  async acceptCommand(envelope) {
    this.#assertLoaded(); if (envelope?.schemaVersion !== PHYSICS_BACKEND_API_VERSION) throw new Error('Unsupported physics command schema');
    this.#assertContext(envelope); if (envelope.sceneRevision !== this.sceneRevision) throw new Error('Stale scene revision');
    if (envelope.robotId !== this.robotId) throw new Error('Command robot does not match loaded scene');
    if (envelope.command?.type === 'set_joint_target' && !Number.isInteger(envelope.maxSteps)) throw new RangeError('set_joint_target requires a positive maxSteps budget');
    const wasPaused = this.state === PhysicsBackendState.PAUSED; const generation = this.generation; const raw = await this.#call('command', envelope.command);
    this.#assertGeneration(generation, envelope);
    let observation;
    try {
      observation = this.#decorateObservation(raw);
    } catch (error) {
      if (generation === this.generation && this.loaded) this.#failLoadedScene('invalid command observation');
      throw error;
    }
    this.lastObservation = observation;
    this.commandBudget = Number.isInteger(envelope.maxSteps) ? { commandId: envelope.commandId, remainingSteps: envelope.maxSteps } : null;
    this.state = wasPaused ? PhysicsBackendState.PAUSED : PhysicsBackendState.READY;
    this.#record('command', { commandId: envelope.commandId, command: structuredClone(envelope.command), maxSteps: envelope.maxSteps, remainingSteps: this.commandBudget?.remainingSteps ?? null, observation });
    return { status: 'accepted', commandId: envelope.commandId, remainingSteps: this.commandBudget?.remainingSteps ?? null, observation };
  }

  // The declared setup path. It is deliberately not part of the command envelope: setup
  // establishes a new initial condition, so it takes no command budget, is refused while a
  // command budget is outstanding, and is recorded in the trace under its own event type.
  // The worker validates every operation and stamps it into the observation's setup log, so
  // a run that began from a perturbation can never present itself as a nominal one.
  async applySetup(payload = {}, context = {}) {
    this.#assertLoaded(); this.#assertContext(context);
    const type = String(payload?.type || '');
    if (!this.setupOperations.has(type)) throw new Error(`Setup operation ${type || '<missing>'} is not declared by this backend`);
    // Refused only while a bounded command still has steps left to run. A budget that has
    // been fully spent is finished, not active - treating it as active would make every
    // declared perturbation after the first advance impossible.
    if (this.commandBudget && this.commandBudget.remainingSteps > 0) throw new Error('Setup may not run while a bounded command is still executing');
    const generation = this.generation;
    try {
      const raw = await this.#call('setup', payload); this.#assertGeneration(generation, context);
      const observation = this.#decorateObservation(raw);
      this.lastObservation = observation;
      this.#record('setup', { setup: structuredClone(payload), observation });
      return observation;
    } catch (error) {
      if (generation === this.generation && this.loaded) this.#failLoadedScene('setup failed');
      throw error;
    }
  }

  async advanceSteps(stepCount = 1, context = {}) {
    this.#assertLoaded(); this.#assertContext(context);
    if (!Number.isInteger(stepCount) || stepCount < 1 || stepCount > MAX_STEP_BATCH) throw new RangeError(`stepCount must be an integer from 1 to ${MAX_STEP_BATCH}`);
    if (this.commandBudget && stepCount > this.commandBudget.remainingSteps) throw new RangeError(`Step request ${stepCount} exceeds remaining command budget ${this.commandBudget.remainingSteps}`);
    const generation = this.generation; const previousTime = Number(this.lastObservation?.simulationTimeSeconds ?? 0); const wasPaused = this.state === PhysicsBackendState.PAUSED;
    if (!wasPaused) this.state = PhysicsBackendState.RUNNING;
    try {
      const raw = await this.#call('step', { count: stepCount }); this.#assertGeneration(generation, context); const observation = this.#decorateObservation(raw);
      const executedSteps = this.#executedSteps(previousTime, observation.simulationTimeSeconds, observation.engine.timestepSeconds);
      if (wasPaused && executedSteps !== 0) throw new Error(`Paused MuJoCo advanced ${executedSteps} steps`);
      if (!wasPaused && executedSteps !== stepCount) throw new Error(`MuJoCo advanced ${executedSteps} steps for a request of ${stepCount}`);
      this.lastObservation = observation; if (this.commandBudget) this.commandBudget.remainingSteps -= executedSteps;
      this.state = wasPaused ? PhysicsBackendState.PAUSED : PhysicsBackendState.READY;
      this.#record('advanceSteps', { requestedSteps: stepCount, executedSteps, commandId: this.commandBudget?.commandId ?? null, remainingSteps: this.commandBudget?.remainingSteps ?? null, observation });
      return observation;
    } catch (error) {
      if (generation === this.generation && this.loaded) this.#failLoadedScene('runtime step failed');
      throw error;
    }
  }

  // Optional sampled-advance capability. The physics authority stays in the worker: one
  // request executes every requested MuJoCo step and returns the ground-truth observations
  // captured at the declared step cadence, so a fine observation resolution no longer costs
  // one cross-thread round trip per sample. Executed steps are still derived from the
  // actual simulation-time delta and charged to the command budget exactly once.
  async advanceStepsObserved(stepCount = 1, context = {}) {
    this.#assertLoaded(); this.#assertContext(context);
    if (!Number.isInteger(stepCount) || stepCount < 1 || stepCount > MAX_STEP_BATCH) throw new RangeError(`stepCount must be an integer from 1 to ${MAX_STEP_BATCH}`);
    const sampleEverySteps = context.sampleEverySteps;
    if (!Number.isInteger(sampleEverySteps) || sampleEverySteps < 1) throw new RangeError('sampleEverySteps must be a positive integer');
    const expectedSamples = sampledObservationCount(stepCount, sampleEverySteps);
    if (expectedSamples > MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE) throw new RangeError(`A ${stepCount}-step advance sampled every ${sampleEverySteps} steps needs ${expectedSamples} observations; the bounded response carries at most ${MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE}`);
    if (this.commandBudget && stepCount > this.commandBudget.remainingSteps) throw new RangeError(`Step request ${stepCount} exceeds remaining command budget ${this.commandBudget.remainingSteps}`);
    const generation = this.generation; const previousTime = Number(this.lastObservation?.simulationTimeSeconds ?? 0); const wasPaused = this.state === PhysicsBackendState.PAUSED;
    if (!wasPaused) this.state = PhysicsBackendState.RUNNING;
    try {
      const raw = await this.#call('stepSampled', { count: stepCount, sampleEverySteps }); this.#assertGeneration(generation, context);
      const rawObservations = Array.isArray(raw?.observations) ? raw.observations : null;
      if (!rawObservations?.length) throw new Error('MuJoCo sampled advance returned no authoritative observations');
      const observations = rawObservations.map((sample) => this.#decorateObservation(sample));
      let cursor = previousTime; let executedSteps = 0;
      for (const [index, observation] of observations.entries()) {
        const steps = this.#executedSteps(cursor, observation.simulationTimeSeconds, observation.engine.timestepSeconds);
        if (wasPaused && steps !== 0) throw new Error(`Paused MuJoCo advanced ${steps} steps`);
        if (!wasPaused && steps < 1) throw new Error('MuJoCo sampled observations are not strictly ordered in simulation time');
        executedSteps += steps;
        cursor = observation.simulationTimeSeconds;
        const cadencePosition = index === observations.length - 1 ? stepCount : (index + 1) * sampleEverySteps;
        if (!wasPaused && executedSteps !== cadencePosition) throw new Error(`MuJoCo sample ${index + 1} lands ${executedSteps} steps into the advance instead of the declared ${cadencePosition}`);
      }
      if (wasPaused && observations.length !== 1) throw new Error('A paused sampled advance must return exactly one unchanged observation');
      if (!wasPaused && observations.length !== expectedSamples) throw new Error(`MuJoCo returned ${observations.length} samples for a declared ${expectedSamples}-sample cadence`);
      if (executedSteps !== (wasPaused ? 0 : stepCount)) throw new Error(`MuJoCo advanced ${executedSteps} steps for a request of ${stepCount}`);
      const finalObservation = observations[observations.length - 1];
      this.lastObservation = finalObservation; if (this.commandBudget) this.commandBudget.remainingSteps -= executedSteps;
      this.state = wasPaused ? PhysicsBackendState.PAUSED : PhysicsBackendState.READY;
      // Bounded evidence: one trace event per sampled advance records the cadence and how
      // many authoritative samples it produced, not every intermediate observation.
      this.#record('advanceStepsObserved', { requestedSteps: stepCount, executedSteps, sampleEverySteps, sampleCount: observations.length, initialSimulationTimeSeconds: previousTime, finalSimulationTimeSeconds: finalObservation.simulationTimeSeconds, commandId: this.commandBudget?.commandId ?? null, remainingSteps: this.commandBudget?.remainingSteps ?? null, observation: finalObservation });
      return { observations, finalObservation, executedSteps, sampleEverySteps };
    } catch (error) {
      if (generation === this.generation && this.loaded) this.#failLoadedScene('runtime sampled step failed');
      throw error;
    }
  }

  async getObservation(context = {}) {
    this.#assertLoaded(); this.#assertContext(context); const requestedView = context.view ?? 'ground_truth';
    if (requestedView !== 'ground_truth') throw new Error(`Observation view ${requestedView} is unsupported by the physical preview`);
    const generation = this.generation;
    try {
      const raw = await this.#call('observe'); this.#assertGeneration(generation, context);
      const observation = this.#decorateObservation(raw, requestedView); this.lastObservation = observation; return observation;
    } catch (error) {
      if (generation === this.generation && this.loaded) this.#failLoadedScene('observation failed');
      throw error;
    }
  }

  async getDiagnostics(context = {}) {
    if (this.sessionId && context?.sessionId) this.#assertSession(context);
    return { backend: 'browser-mujoco', state: this.state, loaded: this.loaded, sceneRevision: this.sceneRevision, robotId: this.robotId, modelPackageId: this.modelPackageId, sessionId: this.sessionId, epoch: this.epoch, pendingRequests: this.pending.size, workerActive: Boolean(this.worker), model: this.lastObservation?.model ? structuredClone(this.lastObservation.model) : null, engineVersion: this.lastObservation?.engine?.version || null, engineVersionEvidence: this.lastObservation?.engine?.versionEvidence || null, timestepSeconds: this.lastObservation?.engine?.timestepSeconds ?? null, worldFrame: structuredClone(WORLD_FRAME), activeCommandId: this.commandBudget?.commandId ?? null, remainingCommandSteps: this.commandBudget?.remainingSteps ?? null, traceEvents: this.trace.length };
  }

  async pause(context = {}) {
    this.#assertLoaded(); this.#assertContext(context); const generation = this.generation;
    try {
      const raw = await this.#call('pause'); this.#assertGeneration(generation, context); const observation = this.#decorateObservation(raw);
      this.lastObservation = observation; this.state = PhysicsBackendState.PAUSED; this.#record('pause', { observation }); return observation;
    } catch (error) {
      if (generation === this.generation && this.loaded) this.#failLoadedScene('pause failed');
      throw error;
    }
  }

  async resume(context = {}) {
    this.#assertLoaded(); this.#assertContext(context); const generation = this.generation;
    try {
      const raw = await this.#call('resume'); this.#assertGeneration(generation, context); const observation = this.#decorateObservation(raw);
      this.lastObservation = observation; this.state = PhysicsBackendState.READY; this.#record('resume', { observation }); return observation;
    } catch (error) {
      if (generation === this.generation && this.loaded) this.#failLoadedScene('resume failed');
      throw error;
    }
  }

  async cancelRun(context = {}) {
    this.#assertLoaded(); this.#adoptNewEpoch(context); ++this.generation; this.#record('cancelRun', { reason: context.reason || 'cancelled' }); this.#terminate('cancelled');
    this.#invalidateLoadedScene(PhysicsBackendState.IDLE);
    return { cancelled: true, reloadRequired: true };
  }

  async exportTrace(context = {}) { if (this.sessionId && context?.sessionId) this.#assertSession(context); return { backend: 'browser-mujoco', phase: 'physics-preview', apiVersion: PHYSICS_BACKEND_API_VERSION, scene: this.lastScene ? structuredClone(this.lastScene) : null, sessionId: this.sessionId, epoch: this.epoch, events: structuredClone(this.trace), lastObservation: structuredClone(this.lastObservation) }; }
  dispose() { if (this.disposed) return; this.disposed = true; ++this.generation; this.#terminate('disposed'); this.#invalidateLoadedScene(PhysicsBackendState.DISPOSED); }

  #decorateObservation(raw = {}, view = 'ground_truth') {
    const simulationTimeSeconds = Number(raw.simulationTime); const timestepSeconds = Number(raw.engine?.timestepSeconds); const modelSha256 = String(raw.model?.sha256 || '').toLowerCase();
    if (!Number.isFinite(simulationTimeSeconds) || simulationTimeSeconds < 0) throw new Error('MuJoCo returned invalid simulation time');
    if (!Number.isFinite(timestepSeconds) || timestepSeconds <= 0) throw new Error('MuJoCo returned invalid timestep');
    if (!/^[0-9a-f]{64}$/.test(modelSha256)) throw new Error('MuJoCo worker did not provide a valid model SHA-256');
    return { schemaVersion: PHYSICS_BACKEND_API_VERSION, sessionId: this.sessionId, epoch: this.epoch, simulationTimeSeconds, view, robotId: this.robotId, frames: { world: structuredClone(WORLD_FRAME) }, model: { id: String(raw.model?.id || ''), asset: String(raw.model?.asset || ''), sha256: modelSha256 }, engine: { name: 'MuJoCo', version: raw.engine?.version == null ? null : String(raw.engine.version), versionEvidence: String(raw.engine?.versionEvidence || 'unknown'), timestepSeconds,
        // Published when the worker reports them. Gravity is here because "the robot fell because
        // of gravity" is only checkable if the acting gravity is observable; the control interval
        // is here because a physical claim depends on the cadence that produced it.
        gravity: Array.isArray(raw.engine?.gravity) ? raw.engine.gravity.map(Number) : null,
        controlIntervalSeconds: raw.engine?.controlIntervalSeconds == null ? null : Number(raw.engine.controlIntervalSeconds) }, joints: structuredClone(raw.joints || {}), bodies: structuredClone(raw.bodies || {}), contactCount: Math.max(0, Number(raw.contactCount) || 0), contactsReadable: Boolean(raw.contactsReadable), contacts: structuredClone(Array.isArray(raw.contacts) ? raw.contacts : []), sensors: raw.imu ? { imu: structuredClone(raw.imu) } : {},
      // Present only on backends whose worker reports them. They carry the physical facts a
      // free-base locomotion controller needs and the honesty flags a reviewer needs: which
      // named foot geoms are actually touching the floor or the ball, whether the actuators
      // are producing force at all, and the log of every declared setup intervention.
      footContacts: raw.footContacts ? structuredClone(raw.footContacts) : null,
      // Free-base robots additionally publish their root as its own record, the contact set
      // classified by named geometry pair, and the controller that is actually engaged. A
      // support evaluation must be able to tell a foot from a torso, and a capability label
      // must name the controller that earned it, so neither may be inferred downstream.
      root: raw.root ? structuredClone(raw.root) : null,
      contactClasses: raw.contactClasses ? structuredClone(raw.contactClasses) : null,
      controller: raw.controller ? structuredClone(raw.controller) : null,
      actuationEnabled: raw.actuationEnabled === undefined ? null : Boolean(raw.actuationEnabled),
      // The servo gain the authority actually applied, and the force it produced. Published
      // because "actuation disabled" is only believable if the force can be read as zero.
      firmwareGain: raw.firmwareGain === undefined ? null : Number(raw.firmwareGain),
      appliedServoKp: raw.appliedServoKp === undefined ? null : Number(raw.appliedServoKp),
      actuatorForceTotalNm: raw.actuatorForceTotalNm === undefined ? null : Number(raw.actuatorForceTotalNm),
      ...(raw.actuatorModel ? {actuatorModel:structuredClone(raw.actuatorModel),sensorObservation:structuredClone(raw.sensorObservation),standingAssessment:structuredClone(raw.standingAssessment)} : {}),
      ...(raw.openarm ? { openarm: structuredClone(raw.openarm) } : {}),
      setupLog: Array.isArray(raw.setupLog) ? structuredClone(raw.setupLog) : [] };
  }

  #executedSteps(previousTime, nextTime, timestep) {
    const delta = nextTime - previousTime; if (!Number.isFinite(delta) || delta < -STEP_TIME_TOLERANCE_SECONDS) throw new Error('MuJoCo simulation time moved backwards');
    if (Math.abs(delta) <= STEP_TIME_TOLERANCE_SECONDS) return 0; const ratio = delta / timestep; const steps = Math.round(ratio);
    if (steps < 0 || Math.abs(delta - steps * timestep) > STEP_TIME_TOLERANCE_SECONDS) throw new Error(`MuJoCo simulation-time delta ${delta} is not an integer number of ${timestep}s steps`);
    return steps;
  }

  #spawn() {
    this.worker = new Worker(this.workerUrl, { type: 'module', name: 'robobuddy-mujoco-physics' });
    this.worker.onmessage = ({ data }) => { const request = this.pending.get(data?.id); if (!request) return; this.pending.delete(data.id); if (data.ok) request.resolve(data.payload); else request.reject(new Error(data.error || 'MuJoCo worker request failed')); };
    this.worker.onerror = (event) => {
      const reason = event.message || 'worker error';
      if (this.disposed) this.#terminate(reason);
      else this.#failLoadedScene(reason);
    };
  }
  #call(op, payload = {}) { if (!this.worker) throw new Error('MuJoCo worker is not running'); const id = ++this.sequence; return new Promise((resolve, reject) => { this.pending.set(id, { resolve, reject }); this.worker.postMessage({ id, op, payload }); }); }
  #record(type, details = {}) { this.trace.push({ type, sessionId: this.sessionId, epoch: this.epoch, sceneRevision: this.sceneRevision, robotId: this.robotId, simulationTimeSeconds: details.observation?.simulationTimeSeconds ?? this.lastObservation?.simulationTimeSeconds ?? null, ...details }); if (this.trace.length > MAX_TRACE_EVENTS) this.trace.splice(0, this.trace.length - MAX_TRACE_EVENTS); }
  #failLoadedScene(reason) { this.#record('backendFailure', { reason: String(reason) }); this.#invalidateLoadedScene(PhysicsBackendState.FAILED); this.#terminate(reason); }
  #invalidateLoadedScene(state) { this.loaded = false; this.state = state; this.sceneRevision = null; this.robotId = null; this.modelPackageId = null; this.lastScene = null; this.lastObservation = null; this.commandBudget = null; }
  #validateNewEpoch(context) { if (!context?.sessionId || !Number.isInteger(context.epoch) || context.epoch < 1) throw new Error('Physics load requires sessionId and positive epoch'); if (this.sessionId && context.sessionId !== this.sessionId) throw new Error('Physics backend is already owned by another session'); if (this.epoch && context.epoch <= this.epoch) throw new Error('Physics load requires a newer epoch'); }
  #adoptNewEpoch(context) { this.#assertSession(context); if (!Number.isInteger(context.epoch) || context.epoch <= this.epoch) throw new Error('Physics reset/cancel requires a newer epoch'); this.epoch = context.epoch; }
  #assertSession(context) { if (!context?.sessionId || context.sessionId !== this.sessionId) throw new Error('Stale or foreign physics session'); }
  #assertContext(context) { this.#assertSession(context); if (context.epoch !== this.epoch) throw new Error('Stale physics epoch'); }
  #assertGeneration(generation, context) { if (generation !== this.generation) throw new Error('Stale MuJoCo worker response'); this.#assertContext(context); }
  #assertLoaded() { this.#assertNotDisposed(); if (!this.loaded || !this.worker) throw new Error('No MuJoCo scene is loaded'); }
  #assertNotDisposed() { if (this.disposed) throw new Error('MuJoCo backend is disposed'); }
  #terminate(reason) { try { this.worker?.terminate(); } catch {} this.worker = null; for (const pending of this.pending.values()) pending.reject(new Error(`MuJoCo worker ${reason}`)); this.pending.clear(); }
}

assertPhysicsBackend(BrowserMuJoCoBackend.prototype);
