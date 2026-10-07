import { BrowserMuJoCoBackend } from './browser-mujoco-backend.js';
import { PhysicsSession } from './session.js';
import {
  MICRODUCK_ACTION_WIDTH,
  MICRODUCK_CONTROL_DECIMATION,
  MICRODUCK_CONTROL_INTERVAL_SECONDS,
  MICRODUCK_OBSERVATION_WIDTH,
  MICRODUCK_POLICY_JOINT_ORDER,
  MicroDuckController,
  makeCommand,
  policyIdForNet,
} from './microduck-controller.js';
import {
  MICRODUCK_BALL_BODY, MICRODUCK_TRUNK_BODY, MICRODUCK_WALK_PACKAGE,
} from './microduck-model-package.js';
import {
  MICRODUCK_CAPABILITY_AUDIT, MICRODUCK_PHYSICAL_CAPABILITY, MICRODUCK_PHYSICAL_PACKAGES,
  MICRODUCK_PHYSICAL_POLICIES, assertMicroDuckCompatibility, isPhysicallySupported,
  microduckCapability, microduckCompatibilityIdentity, microduckPackageSupportsCapability,
  microduckRequiredPackageKeys, MICRODUCK_PHYSICAL_SKILL_IDS,
} from './microduck-capabilities.js';
import { MICRODUCK_GAIT_ONSET_MS, MICRODUCK_SCENES, boundedMicroDuckCommand } from './microduck-scene.js';
import { MicroDuckPhysicalEvaluator, tiltDegrees } from './microduck-task-evaluator.js';

// Declared setup operations this workspace's backend may perform. Anything not on this list
// cannot write physical state at all.
const SETUP_OPERATIONS = Object.freeze(['set_trunk_orientation', 'set_object_pose', 'set_actuation_enabled']);

// Observation publication cadence, in authoritative MuJoCo steps.
//
// The narrowest causal event this workspace depends on is a sole/ball contact during a kick.
// Native evidence measures those contacts lasting one to a few milliseconds each, so the
// cadence has to resolve a single 5 ms physics step: every step is published. That is a
// software observation parameter, not a hardware sensor rate, and it is deliberately finer
// than the 20 ms controller interval - the renderer does not need every sample, but the kick
// evaluator does.
export const MICRODUCK_OBSERVATION_BATCH_STEPS = 1;

// A bounded WebMCP or Python advance. Long runs are made of many of these, so cancellation
// and stale-epoch checks land promptly.
export const MICRODUCK_MAX_ADVANCE_SECONDS = 2;

const DECLARED_PERTURBATIONS = Object.freeze({
  face_down: { quaternionWxyz: [Math.SQRT1_2, 0, -Math.SQRT1_2, 0], label: 'declared pre-trial perturbation: face down' },
  face_up: { quaternionWxyz: [Math.SQRT1_2, 0, Math.SQRT1_2, 0], label: 'declared pre-trial perturbation: face up' },
  on_side: { quaternionWxyz: [Math.SQRT1_2, Math.SQRT1_2, 0, 0], label: 'declared pre-trial perturbation: on its side' },
  upright: { quaternionWxyz: [1, 0, 0, 0], label: 'declared pre-trial perturbation: upright' },
});

function policyJointValues(observation, field) {
  return MICRODUCK_POLICY_JOINT_ORDER.map((id) => {
    const value = Number(observation?.joints?.[id]?.[field]);
    if (!Number.isFinite(value)) throw new Error(`Authoritative observation is missing ${field} for ${id}`);
    return value;
  });
}

/**
 * The MicroDuck Phase 5C physical workspace.
 *
 * One MuJoCo PhysicsSession is the sole physical authority. The controller turns
 * authoritative observations plus a bounded command into actuator targets; the plant decides
 * what happens. Nothing in this class writes root pose, root velocity, joint state or object
 * state outside the declared setup path, and nothing here produces a task outcome that is
 * not read back out of physics.
 */
export class MicroDuckPhysicalSimulator {
  constructor(canvas = null, { rig = null } = {}) {
    this.canvas = canvas;
    this.rig = rig;
    this.ready = false;
    this.highContrast = true;
    this.presentationDirty = false;
    this.threeScene = null;
    this.camera = null;
    this.renderer = null;
    this.controls = null;
    this.ballMesh = null;
    this.markers = [];
    this.THREE = null;
    // Presentation - three.js, the rig adapter and the ONNX runtime - is loaded lazily, only
    // when a canvas is present. That keeps the physics core importable and testable without a
    // renderer, and it is why setScenario is the thing that builds the viewport rather than
    // the constructor.
    this.session = null;
    this.policyRuntime = null;
    this.controller = null;
    this.modelPackage = null;
    this.scene = null;
    this.packageKey = null;
    this.identity = null;
    this.disposed = false;
    this.runEpoch = 0;
    this.cancelled = false;
    this.lastObservation = null;
    this.evaluator = null;
    this.requested = boundedMicroDuckCommand({}).command;
    this.requestedLimitedBy = [];
    this.observationListeners = new Set();
    this.unsubscribe = null;
  }

  // ---------------------------------------------------------------- lifecycle
  // Headless construction, for tests and for the reference/conformance paths. A canvas-backed
  // workspace is built by SimulatorHost through the constructor and setScenario instead.
  static async create({ packageKey = 'walk', canvas = null, rig = null, policyRuntime = null, backendFactory = null } = {}) {
    const simulator = new MicroDuckPhysicalSimulator(canvas, { rig });
    simulator.injectedPolicyRuntime = policyRuntime;
    await simulator.load(packageKey, { policyRuntime, backendFactory });
    return simulator;
  }

  /**
   * `backendFactory` and `policyRuntime` are injection seams for headless verification. The
   * browser path uses neither: it builds the real worker-backed backend and the real ONNX
   * runtime below.
   */
  async load(packageKey = 'walk', { policyRuntime = null, backendFactory = null } = {}) {
    this.#assertLive();
    backendFactory = backendFactory || this.backendFactory || null;
    const modelPackage = MICRODUCK_PHYSICAL_PACKAGES[packageKey];
    const scene = MICRODUCK_SCENES[packageKey];
    if (!modelPackage || !scene) throw new Error(`Unknown MicroDuck physical package: ${packageKey}`);

    this.#teardownSession();
    const backend = backendFactory
      ? backendFactory({ setupOperations: SETUP_OPERATIONS })
      : new BrowserMuJoCoBackend({
        workerUrl: new URL('./microduck-mujoco-worker.js', import.meta.url),
        setupOperations: SETUP_OPERATIONS,
      });
    this.backendFactory = backendFactory;
    this.session = new PhysicsSession(
      backend,
      { sessionId: `microduck-physical-${Date.now()}`, observationBatchSteps: MICRODUCK_OBSERVATION_BATCH_STEPS },
    );
    this.unsubscribe = this.session.subscribe((event) => this.#onObservation(event));

    const runtime = policyRuntime || this.injectedPolicyRuntime;
    if (runtime) this.policyRuntime = runtime;
    else {
      const { MicroDuckPolicyRuntime } = await import('../microduck/policy-runtime.js');
      this.policyRuntime = new MicroDuckPolicyRuntime();
      await this.policyRuntime.initialize();
    }

    this.modelPackage = modelPackage;
    this.scene = scene;
    this.packageKey = packageKey;

    // Compatibility is validated before anything can run, for every policy this workspace
    // will make available. A mismatch is a refusal here, not a fidelity note later.
    this.identity = {};
    for (const policyId of MICRODUCK_PHYSICAL_POLICIES) {
      const { identity } = assertMicroDuckCompatibility(modelPackage, policyId);
      this.identity[policyId] = identity;
    }

    this.controller = new MicroDuckController({ availablePolicies: MICRODUCK_PHYSICAL_POLICIES });
    const result = await this.session.loadScene(scene);
    this.runEpoch += 1;
    this.cancelled = false;
    this.requested = boundedMicroDuckCommand({}).command;
    this.requestedLimitedBy = [];
    this.evaluator = new MicroDuckPhysicalEvaluator({ commandedTwist: this.requested.twist });
    this.#applyPresentation(this.lastObservation);
    return result;
  }

  async reset() {
    this.#assertLoaded();
    this.controller.reset();
    this.runEpoch += 1;
    this.cancelled = false;
    this.requested = boundedMicroDuckCommand({}).command;
    this.requestedLimitedBy = [];
    const observation = await this.session.reset();
    this.evaluator = new MicroDuckPhysicalEvaluator({ commandedTwist: this.requested.twist });
    return observation;
  }

  cancel(reason = 'cancelled') {
    this.cancelled = true;
    this.cancelReason = String(reason);
    this.runEpoch += 1;
  }

  async pause() { this.#assertLoaded(); return this.session.pause(); }
  async resume() { this.#assertLoaded(); return this.session.resume(); }

  subscribe(listener) {
    if (typeof listener !== 'function') throw new TypeError('observation subscriber must be a function');
    this.observationListeners.add(listener);
    return () => this.observationListeners.delete(listener);
  }

  dispose() {
    if (this.disposed) return;
    this.disposed = true;
    this.ready = false;
    this.#teardownSession();
    this.#disposePresentation();
    void this.policyRuntime?.dispose?.();
    this.policyRuntime = null;
    this.observationListeners.clear();
  }

  // ------------------------------------------------------------ command surface
  /** Validate without mutating command, controller or evaluator state. */
  validateCommand(requested = {}) {
    this.#assertLoaded();
    const { command, limitedBy } = boundedMicroDuckCommand(requested);
    if (!['walk', 'lowTraction'].includes(this.packageKey) && command.twist.some((value) => Math.abs(value) > 1e-12)) {
      throw new Error(`MicroDuck collision-plant mismatch: non-zero velocity commands require the walk plant; active package is ${this.packageKey}`);
    }
    return { command, limitedBy };
  }

  /** Latch a bounded command. Accepting it is not achieving it. */
  setCommand(requested = {}) {
    const { command, limitedBy } = this.validateCommand(requested);
    this.requested = command;
    this.requestedLimitedBy = limitedBy;
    this.evaluator = new MicroDuckPhysicalEvaluator({ commandedTwist: command.twist });
    return { accepted: true, requested: structuredClone(command), limitedBy: [...limitedBy] };
  }

  /**
   * Ask for a skill. An unsupported capability is refused here and routed nowhere: the
   * physical workspace has no legacy dynamics to fall back to.
   */
  validateSkillRequest(skill) {
    this.#assertLoaded();
    const capability = microduckCapability(skill);
    if (capability && !capability.physicalPolicy) {
      return { accepted: false, status: 'unsupported', capability: capability.id, reason: capability.evidence };
    }
    if (capability && !microduckPackageSupportsCapability(this.packageKey, capability.id)) {
      const requiredPackageKeys = [...microduckRequiredPackageKeys(capability.id)];
      return {
        accepted: false, status: 'wrong-plant', capability: capability.id, requiredPackageKeys,
        reason: `${capability.id} requires MicroDuck collision plant ${requiredPackageKeys.join(' or ')}; active package is ${this.packageKey}`,
      };
    }
    if (!MICRODUCK_PHYSICAL_SKILL_IDS.includes(skill) || !this.controller.hasPolicy(capability?.physicalPolicy)) {
      return { accepted: false, status: 'unsupported', capability: skill, reason: `${skill} is not a physical skill command; use sit or stand_up for the sit_stand capability` };
    }
    return { accepted: true, status: 'available', capability: capability.id };
  }

  requestSkill(skill) {
    const result = this.validateSkillRequest(skill);
    if (!result.accepted) return result;
    if (!this.controller.requestSkill(skill)) return { accepted: false, status: 'unsupported', capability: skill, reason: 'The controller refused the skill' };
    return { ...result, status: 'running' };
  }

  // -------------------------------------------------------------- declared setup
  /**
   * A declared pre-trial perturbation. It is logged as setup by the authority, appears in
   * every subsequent observation, and never counts as task progress.
   */
  async applyPerturbation(name) {
    this.#assertLoaded();
    const perturbation = DECLARED_PERTURBATIONS[name];
    if (!perturbation) throw new Error(`Unknown declared perturbation: ${name}`);
    const observation = await this.session.applySetup({
      type: 'set_trunk_orientation',
      quaternionWxyz: [...perturbation.quaternionWxyz],
      label: perturbation.label,
    });
    // A perturbation begins a new trial, so the controller's feedback state must not carry
    // an action from the previous one into it.
    this.controller.reset();
    this.evaluator = new MicroDuckPhysicalEvaluator({ commandedTwist: this.requested.twist });
    return observation;
  }

  /** A declared torque-off condition, for the actuation-disabled negative control. */
  async setActuationEnabled(enabled) {
    this.#assertLoaded();
    return this.session.applySetup({
      type: 'set_actuation_enabled',
      enabled: enabled !== false,
      label: enabled === false ? 'declared torque-off condition' : 'actuators enabled',
    });
  }

  /** A declared object placement, for kick positive and miss controls. */
  async placeBall(positionM, label = 'declared pre-trial ball placement') {
    this.#assertLoaded();
    if (this.packageKey !== 'kick') throw new Error('Only the kick package carries a ball');
    return this.session.applySetup({ type: 'set_object_pose', bodyId: MICRODUCK_BALL_BODY, positionM, label });
  }

  /** Settle a perturbation with the servos holding the home pose. Logged, not evaluated. */
  async settle(seconds, { assertActive = null } = {}) {
    this.#assertLoaded();
    if (assertActive !== null && typeof assertActive !== 'function') throw new TypeError('assertActive must be a synchronous guard function');
    assertActive?.();
    const steps = Math.max(1, Math.round(Number(seconds) / this.modelPackage.physics.timestepSeconds));
    const targetsRad = Object.fromEntries(
      MICRODUCK_POLICY_JOINT_ORDER.map((id) => [id, this.modelPackage.initialJointPositionsRad[id]]),
    );
    const session = this.session;
    await session.sendCommand({ type: 'set_joint_targets', targetsRad }, { maxSteps: steps });
    assertActive?.();
    // A worker batch already submitted cannot be retracted. Guarded callers submit
    // at most one controller interval (20 ms) at a time, never the whole settle budget.
    // The target frame and every physics step are unchanged; no policy runs here.
    const batchSize = assertActive ? MICRODUCK_CONTROL_DECIMATION : steps;
    for (let remaining = steps; remaining > 0;) {
      assertActive?.();
      const batch = Math.min(batchSize, remaining);
      await session.advanceSteps(batch);
      remaining -= batch;
      assertActive?.();
    }
    this.controller.reset();
    this.evaluator = new MicroDuckPhysicalEvaluator({ commandedTwist: this.requested.twist });
    return this.lastObservation;
  }

  // ------------------------------------------------------------- the control loop
  /**
   * Advance the physical session by a bounded interval of SIMULATION time.
   *
   * Each controller tick reads the authoritative observation, assembles the 61-value policy
   * input, runs inference, and writes actuator targets; the plant then advances exactly the
   * declared number of physics steps. Nothing here is driven by a display frame, and no
   * required control or physics step is dropped to keep up with wall-clock time - a slow
   * machine reports a lower real-time factor, it does not get a different trajectory.
   */
  async advanceSeconds(seconds, { bodyActive = false, onTick = null, assertActive = null } = {}) {
    this.#assertLoaded();
    // Optional synchronous ownership/cancellation guard. It must run on BOTH sides
    // of awaited work, especially inference, before committing controller feedback
    // or submitting more commands. A rejected call does not poison the next one.
    if (assertActive !== null && typeof assertActive !== 'function') throw new TypeError('assertActive must be a synchronous guard function');
    assertActive?.();
    const duration = Number(seconds);
    if (!Number.isFinite(duration) || duration <= 0) throw new RangeError('advanceSeconds requires a positive duration');
    if (duration > MICRODUCK_MAX_ADVANCE_SECONDS) throw new RangeError(`advanceSeconds is bounded to ${MICRODUCK_MAX_ADVANCE_SECONDS} simulated seconds per call`);
    const epoch = this.runEpoch;
    const ticks = Math.max(1, Math.round(duration / MICRODUCK_CONTROL_INTERVAL_SECONDS));
    const command = makeCommand(this.requested);
    let executedTicks = 0;
    let lastStep = null;
    for (let tick = 0; tick < ticks; tick += 1) {
      assertActive?.();
      if (this.cancelled || epoch !== this.runEpoch) break;
      const observation = this.lastObservation || await this.session.getObservation();
      assertActive?.();
      if (this.cancelled || epoch !== this.runEpoch) break;
      const pending = this.controller.beginTick({
        observation: {
          gyroRadS: observation.sensors?.imu?.gyroRadS,
          projectedGravity: observation.sensors?.imu?.projectedGravity,
          jointPositionRad: policyJointValues(observation, 'positionRad'),
          jointVelocityRadS: policyJointValues(observation, 'velocityRadS'),
        },
        command,
        bodyActive,
      });
      const rawAction = await this.policyRuntime.infer(pending.policyId, pending.observation);
      assertActive?.();
      // A run that was cancelled or replaced while inference was in flight must not write
      // targets into the next workspace.
      if (this.cancelled || epoch !== this.runEpoch) break;
      if (!rawAction || rawAction.length !== MICRODUCK_ACTION_WIDTH) {
        throw new Error(`Policy ${pending.policyId} returned ${rawAction?.length ?? 0} values, expected ${MICRODUCK_ACTION_WIDTH}`);
      }
      lastStep = this.controller.completeTick(pending, rawAction, MICRODUCK_CONTROL_INTERVAL_SECONDS);
      await this.session.sendCommand(
        // The gain travels with the targets, exactly as the deployed daemon writes the servo
        // P-gain register alongside each target frame. Standing, kicks and the sit/rise cycle
        // run softer than walking, and that difference is physical, not cosmetic.
        { type: 'set_joint_targets', targetsRad: { ...lastStep.targetsRad }, firmwareGain: lastStep.gain },
        { maxSteps: MICRODUCK_CONTROL_DECIMATION },
      );
      assertActive?.();
      if (this.cancelled || epoch !== this.runEpoch) break;
      await this.session.advanceSteps(MICRODUCK_CONTROL_DECIMATION);
      executedTicks += 1;
      if (typeof onTick === 'function') onTick({ tick, step: lastStep, observation: this.lastObservation });
      assertActive?.();
    }
    return {
      requestedSeconds: duration,
      requestedTicks: ticks,
      executedTicks,
      completed: executedTicks === ticks,
      cancelled: this.cancelled || epoch !== this.runEpoch,
      simulatedSeconds: executedTicks * MICRODUCK_CONTROL_INTERVAL_SECONDS,
      controller: lastStep ? { net: lastStep.net, policyId: lastStep.policyId, actionScale: lastStep.scale, busy: lastStep.busy } : null,
      observation: this.lastObservation,
    };
  }

  // -------------------------------------------------------------------- state
  getState() {
    const observation = this.lastObservation;
    const trunk = observation?.bodies?.[MICRODUCK_TRUNK_BODY] || null;
    return {
      backend: 'browser-mujoco',
      workspace: 'physical',
      modelPackageId: this.modelPackage?.id ?? null,
      modelSha256: this.modelPackage?.sha256 ?? null,
      sceneId: this.scene?.id ?? null,
      sceneRevision: this.scene?.revision ?? null,
      sessionId: this.session?.sessionId ?? null,
      epoch: this.session?.epoch ?? null,
      runEpoch: this.runEpoch,
      cancelled: this.cancelled,
      capability: MICRODUCK_PHYSICAL_CAPABILITY,
      // Three deliberately separate views: what was asked for, what the controller decided,
      // and what the physics actually is.
      requested: { command: structuredClone(this.requested), limitedBy: [...this.requestedLimitedBy] },
      controller: this.controller?.lastStep
        ? {
          net: this.controller.lastStep.net,
          policyId: this.controller.lastStep.policyId,
          actionScale: this.controller.lastStep.scale,
          firmwareGain: this.controller.lastStep.gain,
          standingTuned: this.controller.lastStep.standingTuned,
          busy: this.controller.lastStep.busy,
          effectiveTwist: [...this.controller.lastStep.commandTwist],
          previousRawAction: [...this.controller.previousRawAction],
        }
        : null,
      actual: observation
        ? {
          simulationTimeSeconds: observation.simulationTimeSeconds,
          trunkPositionM: trunk ? [...trunk.positionM] : null,
          trunkQuaternionWxyz: trunk ? [...trunk.quaternionWxyz] : null,
          trunkTiltDeg: trunk ? Number(tiltDegrees(trunk.quaternionWxyz).toFixed(4)) : null,
          jointPositionRad: Object.fromEntries(MICRODUCK_POLICY_JOINT_ORDER.map((id) => [id, observation.joints?.[id]?.positionRad ?? null])),
          jointTargetRad: Object.fromEntries(MICRODUCK_POLICY_JOINT_ORDER.map((id) => [id, observation.joints?.[id]?.targetRad ?? null])),
          jointEffortNm: Object.fromEntries(MICRODUCK_POLICY_JOINT_ORDER.map((id) => [id, observation.joints?.[id]?.effortNm ?? null])),
          footContacts: observation.footContacts ? structuredClone(observation.footContacts) : null,
          ballPositionM: observation.bodies?.[MICRODUCK_BALL_BODY]?.positionM ?? null,
          actuationEnabled: observation.actuationEnabled,
          // The effective BAM firmware gain and actual actuator force come from the physical
          // authority. The controller view above reports the scheduled gain it ASKED for.
          // `appliedServoKp` is retained only as the source-XML equivalent compatibility
          // diagnostic; BAM torque physics does not use that fallback PD stiffness.
          firmwareGain: observation.firmwareGain ?? null,
          appliedServoKp: observation.appliedServoKp ?? null,
          actuatorForceTotalNm: observation.actuatorForceTotalNm ?? null,
          setupLog: Array.isArray(observation.setupLog) ? observation.setupLog.map((item) => ({ ...item })) : [],
        }
        : null,
      capabilities: MICRODUCK_CAPABILITY_AUDIT.map((item) => ({ id: item.id, label: item.label, status: item.status, physical: Boolean(item.physicalPolicy), availableInActivePlant: microduckPackageSupportsCapability(this.packageKey, item.id), requiredPackageKeys: [...microduckRequiredPackageKeys(item.id)] })),
      gaitOnsetMS: MICRODUCK_GAIT_ONSET_MS,
      hardwareValidated: false,
    };
  }

  report() { return this.evaluator ? this.evaluator.report() : null; }
  locomotionVerdict(options) { return this.evaluator ? this.evaluator.locomotionVerdict(options) : null; }

  compatibilityIdentity(policyId) {
    this.#assertLoaded();
    return microduckCompatibilityIdentity(this.modelPackage, policyId);
  }

  // -------------------------------------------------- SimulatorHost backend interface
  async setScenario(profileId, scenario) {
    if (profileId !== 'microduck') throw new Error('MicroDuck physical simulator only accepts the microduck profile');
    if (scenario?.simulationMode !== 'physical_mujoco') throw new Error('MicroDuck physical simulator requires a physical_mujoco scenario');
    const packageKey = Object.entries(MICRODUCK_SCENES).find(([, item]) => item.id === scenario.physicalSceneId)?.[0];
    if (!packageKey) throw new Error(`Unknown MicroDuck physical scene: ${scenario?.physicalSceneId}`);
    await this.#ensurePresentation();
    await this.load(packageKey);
    this.ready = true;
    if (this.canvas) {
      this.canvas.dataset.simulatorBackend = 'browser-mujoco';
      this.canvas.dataset.simulationAuthority = 'physics-session';
      this.canvas.dataset.physicalSceneId = this.scene.id;
      this.canvas.dataset.modelPackageId = this.scene.modelPackage;
      this.canvas.dataset.microduckWorkspace = 'physical';
      this.canvas.dataset.microduckControllerHz = String(Math.round(1 / MICRODUCK_CONTROL_INTERVAL_SECONDS));
      this.canvas.dataset.microduckRootAuthority = 'mujoco-free-body';
      this.canvas.dataset.microduckUnsupportedPhysicalSkills = MICRODUCK_CAPABILITY_AUDIT
        .filter((item) => !item.physicalPolicy).map((item) => item.id).join(',');
    }
    this.fit();
    return true;
  }

  renderFrame() {
    if (this.disposed || !this.renderer) return;
    if (this.presentationDirty && this.lastObservation) {
      this.#applyPresentation(this.lastObservation);
      this.presentationDirty = false;
    }
    this.controls?.update();
    this.renderer.render(this.threeScene, this.camera);
  }

  resize() {
    if (this.disposed || !this.renderer || !this.canvas) return false;
    const width = Math.max(1, this.canvas.clientWidth || this.canvas.width || 640);
    const height = Math.max(1, this.canvas.clientHeight || this.canvas.height || 480);
    const dpr = Math.min(globalThis.devicePixelRatio || 1, 2);
    if (this.canvas.width !== Math.floor(width * dpr) || this.canvas.height !== Math.floor(height * dpr)) this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    return true;
  }

  fit() {
    if (!this.camera || !this.controls) return false;
    this.controls.target.set(0, 60, 0);
    this.camera.position.set(420, 300, 420);
    this.camera.updateProjectionMatrix();
    this.controls.update();
    return true;
  }

  setHighContrastScene(value) {
    this.highContrast = Boolean(value);
    for (const marker of this.markers) marker.visible = this.highContrast;
    if (this.canvas) this.canvas.dataset.highContrastScene = String(this.highContrast);
    return this.highContrast;
  }

  isHighContrastSceneEnabled() { return this.highContrast; }
  isReady() { return Boolean(this.ready && this.session && this.lastObservation && this.session.sceneRevision && this.session.robotId); }
  getPhysicalSession() { return this.session; }

  getPhysicalAuthorityToken() {
    if (!this.isReady()) return null;
    return Object.freeze({
      sessionId: this.session.sessionId, epoch: this.session.epoch,
      sceneRevision: this.session.sceneRevision, robotId: this.session.robotId,
      simulationTimeSeconds: this.lastObservation.simulationTimeSeconds,
    });
  }

  getTaskEvaluation() { return this.report(); }

  getPresentationAudit() {
    return Object.freeze({
      physicalAuthority: 'MuJoCo PhysicsSession only',
      rootTransformSource: 'observed MuJoCo trunk_base free-body pose',
      bakedWorldRootApplied: false,
      configuredRollers: false,
      jointPresentationSource: 'same authoritative MuJoCo body-pose snapshot as trunk and contacts; no joint FK reconstruction',
      bodyPoseSnapshotSource: 'MuJoCo xpos/xquat solved kinematics',
      ballTransformSource: 'observed MuJoCo microduck_ball free-body pose',
      // The legacy demonstrator re-seats its visual on the floor every frame. The physical
      // workspace must not: trunk height is a physical result, so a crouch, a fall and the
      // airborne phase of a roulade all have to be visible.
      rendererFloorSnapping: false,
      rendererIntegratesRoot: false,
      observationBatchSteps: MICRODUCK_OBSERVATION_BATCH_STEPS,
      controlIntervalSeconds: MICRODUCK_CONTROL_INTERVAL_SECONDS,
      physicsTimestepSeconds: this.modelPackage?.physics?.timestepSeconds ?? null,
    });
  }

  // ----------------------------------------------------------------- internals
  async #ensurePresentation() {
    if (!this.canvas || this.renderer) return;
    const [THREE, controlsModule, rigModule] = await Promise.all([
      import('three'),
      import('https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/controls/OrbitControls.js'),
      import('../microduck/rig-adapter.js'),
    ]);
    this.THREE = THREE;
    this.threeScene = new THREE.Scene();
    this.threeScene.background = new THREE.Color(0xb4bcc0);
    this.camera = new THREE.PerspectiveCamera(45, 1, 1, 8000);
    this.camera.position.set(420, 300, 420);
    this.renderer = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(globalThis.devicePixelRatio || 1, 2));
    this.controls = new controlsModule.OrbitControls(this.camera, this.canvas);
    this.controls.enableDamping = true;
    this.controls.target.set(0, 60, 0);
    this.threeScene.add(new THREE.HemisphereLight(0xffffff, 0x30404a, 1.3));
    const key = new THREE.DirectionalLight(0xffffff, 1.9);
    key.position.set(400, 900, 400);
    this.threeScene.add(key);
    const grid = new THREE.GridHelper(4000, 40, 0x334155, 0x4b5563);
    grid.userData.presentationOnly = true;
    this.markers.push(grid);
    this.threeScene.add(grid);
    this.robotRoot = new THREE.Group();
    this.robotRoot.name = 'microduck-physical-presentation';
    this.threeScene.add(this.robotRoot);
    if (!this.rig) {
      this.rig = await rigModule.MicroDuckRigAdapter.load({ includeConfiguredRollers: false });
    }
    this.robotRoot.add(this.rig.root);
    this.rig.setVariant?.('walking');
    this.resize();
  }

  #ensureBallMesh() {
    if (!this.threeScene || !this.THREE || this.ballMesh || this.packageKey !== 'kick') return;
    const THREE = this.THREE;
    const radiusMm = 35;
    this.ballMesh = new THREE.Mesh(
      new THREE.SphereGeometry(radiusMm, 24, 16),
      new THREE.MeshStandardMaterial({ color: 0xff8c1a, roughness: 0.85, metalness: 0 }),
    );
    this.ballMesh.name = 'microduck-physical-ball-presentation';
    this.threeScene.add(this.ballMesh);
  }

  #onObservation(event) {
    const observation = event?.observation;
    if (!observation) return;
    this.lastObservation = observation;
    this.evaluator?.observe(observation);
    // The renderer refreshes on its own frame, not on every published physics sample: the
    // observation cadence is 5 ms and a display frame is ~16 ms, so applying the presentation
    // here would do the same work three times per frame for nothing.
    this.presentationDirty = true;
    for (const listener of this.observationListeners) {
      try { listener(event); } catch { /* presentation consumers are never simulation authorities */ }
    }
  }

  // Presentation only. It reads the authoritative pose; it never writes physical state, and
  // it never re-seats the robot on the floor - trunk height is a physical result here.
  #applyPresentation(observation) {
    if (!observation) return;
    const trunk = observation.bodies?.[MICRODUCK_TRUNK_BODY];
    if (this.rig && trunk?.positionM && trunk?.quaternionWxyz) {
      this.rig.applyPhysicalBodyPoses(observation.bodies);
    }
    const ball = observation.bodies?.[MICRODUCK_BALL_BODY];
    if (ball?.positionM) {
      this.#ensureBallMesh();
      // MuJoCo is z-up in metres; the viewport is y-up in millimetres.
      if (this.ballMesh) this.ballMesh.position.set(ball.positionM[0] * 1000, ball.positionM[2] * 1000, -ball.positionM[1] * 1000);
    }
  }

  #disposePresentation() {
    try { this.rig?.dispose?.(); } catch { /* a partially built rig still has to go */ }
    this.rig = null;
    if (this.ballMesh) {
      this.ballMesh.geometry?.dispose?.();
      this.ballMesh.material?.dispose?.();
      this.ballMesh.removeFromParent();
      this.ballMesh = null;
    }
    try { this.controls?.dispose?.(); } catch { /* controls may already be detached */ }
    try { this.renderer?.dispose?.(); } catch { /* the context may already be lost */ }
    this.controls = null;
    this.renderer = null;
    this.markers = [];
  }

  #teardownSession() {
    try { this.unsubscribe?.(); } catch { /* a disposed session has no subscribers left */ }
    this.unsubscribe = null;
    try { this.session?.dispose(); } catch { /* disposing an already-failed session is not an error */ }
    this.session = null;
    this.lastObservation = null;
    this.evaluator = null;
  }

  #assertLive() { if (this.disposed) throw new Error('MicroDuck physical simulator is disposed'); }
  #assertLoaded() { this.#assertLive(); if (!this.session || !this.modelPackage) throw new Error('No MicroDuck physical scene is loaded'); }
}

export {
  MICRODUCK_ACTION_WIDTH, MICRODUCK_CONTROL_DECIMATION, MICRODUCK_OBSERVATION_WIDTH,
  MICRODUCK_WALK_PACKAGE, isPhysicallySupported, policyIdForNet,
};
