import { assertPhysicalScene, assertPhysicsBackend, assertSampledObservations, makeCommandEnvelope, MAX_ADVANCE_STEPS_PER_REQUEST, MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE, PHYSICS_BACKEND_API_VERSION, supportsSampledAdvance } from './backend-contract.js';

function uid(prefix) {
  return `${prefix}-${crypto.randomUUID?.() || Math.random().toString(36).slice(2)}`;
}

export class PhysicsSession {
  constructor(backend, { sessionId = uid('physics'), observationBatchSteps = null } = {}) {
    this.backend = assertPhysicsBackend(backend);
    this.sessionId = String(sessionId);
    if (observationBatchSteps != null && (!Number.isInteger(observationBatchSteps) || observationBatchSteps < 1)) {
      throw new RangeError('observationBatchSteps must be a positive integer when configured');
    }
    this.observationBatchSteps = observationBatchSteps == null ? null : Number(observationBatchSteps);
    this.epoch = 0;
    this.sceneRevision = null;
    this.robotId = null;
    this.disposed = false;
    this.activeCommandId = null;
    this.observationListeners = new Set();
  }

  subscribe(listener) {
    this.#assertLive();
    if (typeof listener !== 'function') throw new TypeError('PhysicsSession subscriber must be a function');
    this.observationListeners.add(listener);
    return () => this.observationListeners.delete(listener);
  }

  async loadScene(scene) {
    this.#assertLive();
    assertPhysicalScene(scene);
    this.epoch += 1;
    this.activeCommandId = null;
    const context = this.#context();
    try {
      const result = await this.backend.loadScene(structuredClone(scene), context);
      this.#assertCurrent(context);
      this.sceneRevision = result?.sceneRevision ?? scene.revision;
      this.robotId = result?.robotId ?? scene.robotId;
      this.#publishObservation(result?.observation, 'loadScene', context);
      return { apiVersion: PHYSICS_BACKEND_API_VERSION, ...this.#context(), ...result };
    } catch (error) {
      if (this.epoch === context.epoch && !this.disposed) this.#clearLoadedState();
      throw error;
    }
  }

  async reset(options = {}) {
    this.#assertLoaded();
    this.epoch += 1;
    this.activeCommandId = null;
    const context = this.#context();
    const result = await this.#runLoadedOperation(() => this.backend.reset({ ...structuredClone(options), ...context }), context);
    this.#publishObservation(result, 'reset', context);
    return { ...this.#context(), result };
  }

  async sendCommand(command, { commandId = uid('cmd'), maxSteps = null } = {}) {
    this.#assertLoaded();
    const context = this.#context();
    const envelope = makeCommandEnvelope({
      ...context,
      commandId,
      sceneRevision: this.sceneRevision,
      robotId: this.robotId,
      command,
      maxSteps,
    });
    const result = await this.#runLoadedOperation(() => this.backend.acceptCommand(envelope), context);
    this.#assertCurrent(context);
    this.activeCommandId = envelope.commandId;
    this.#publishObservation(result?.observation, 'sendCommand', context);
    return result;
  }

  // A declared pre-trial intervention: an initial orientation, an object placement, a
  // torque-off condition. It establishes a new initial condition rather than advancing the
  // task, so it is a distinct session operation from sendCommand, it publishes its
  // observation tagged as setup, and the backend refuses it unless the operation is on that
  // backend's declared allowlist.
  async applySetup(payload) {
    this.#assertLoaded();
    if (typeof this.backend.applySetup !== 'function') throw new Error('This physics backend declares no setup path');
    const context = this.#context();
    const observation = await this.#runLoadedOperation(() => this.backend.applySetup(structuredClone(payload), context), context);
    this.#publishObservation(observation, 'applySetup', context);
    return observation;
  }

  async advanceSteps(steps) {
    this.#assertLoaded();
    if (!Number.isInteger(steps) || steps < 1) throw new RangeError('steps must be a positive integer');
    const context = this.#context();
    const batch = this.observationBatchSteps;
    if (!batch || steps <= batch) {
      const observation = await this.#runLoadedOperation(() => this.backend.advanceSteps(steps, context), context);
      this.#publishObservation(observation, 'advanceSteps', context);
      return observation;
    }

    // Long lockstep advances still execute exclusively in the authoritative backend,
    // but publish intermediate ground-truth observations at a declared physics-step
    // cadence. This prevents task evaluators/controllers from losing short-lived
    // physical contacts merely because a caller requested a long simulation interval.
    if (supportsSampledAdvance(this.backend)) return this.#advanceSampled(steps, batch, context);

    let remaining = steps;
    let observation = null;
    while (remaining > 0) {
      const chunk = Math.min(batch, remaining);
      observation = await this.#runLoadedOperation(() => this.backend.advanceSteps(chunk, context), context);
      this.#publishObservation(observation, 'advanceSteps', context);
      remaining -= chunk;
    }
    return observation;
  }

  // Sampling-capable backends execute the whole interval in the physics authority and
  // return its ordered ground-truth samples in one response, so the observation cadence
  // no longer costs one cross-thread round trip per sample. The session stays the sampling
  // policy owner: it declares the cadence, validates ordering, and publishes each real
  // observation through the ordinary subscriber mechanism.
  async #advanceSampled(steps, sampleEverySteps, context) {
    const maxStepsPerRequest = Math.min(sampleEverySteps * MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE, MAX_ADVANCE_STEPS_PER_REQUEST);
    let remaining = steps;
    let previousSimulationTimeSeconds = null;
    let finalObservation = null;
    while (remaining > 0) {
      const requested = Math.min(maxStepsPerRequest, remaining);
      const result = await this.#runLoadedOperation(() => this.backend.advanceStepsObserved(requested, { ...context, sampleEverySteps }), context);
      const { observations, finalObservation: last } = assertSampledObservations(result, { stepCount: requested, sampleEverySteps, previousSimulationTimeSeconds });
      for (const observation of observations) this.#publishObservation(observation, 'advanceSteps', context);
      previousSimulationTimeSeconds = Number(last.simulationTimeSeconds);
      finalObservation = last;
      remaining -= requested;
      // A paused authority executes no physics; further requests cannot change that.
      if (result.executedSteps === 0) break;
    }
    return finalObservation;
  }

  async getObservation(options = {}) {
    this.#assertLoaded();
    const context = this.#context();
    const observation = await this.#runLoadedOperation(() => this.backend.getObservation({ ...options, ...context }), context);
    this.#publishObservation(observation, 'getObservation', context);
    return observation;
  }

  async getDiagnostics() {
    this.#assertLive();
    const context = this.#context();
    const diagnostics = await this.backend.getDiagnostics(context);
    this.#assertCurrent(context);
    if (diagnostics?.loaded === false) this.#clearLoadedState();
    return diagnostics;
  }

  async pause() {
    this.#assertLoaded();
    const context = this.#context();
    const observation = await this.#runLoadedOperation(() => this.backend.pause(context), context);
    this.#publishObservation(observation, 'pause', context);
    return observation;
  }

  async resume() {
    this.#assertLoaded();
    const context = this.#context();
    const observation = await this.#runLoadedOperation(() => this.backend.resume(context), context);
    this.#publishObservation(observation, 'resume', context);
    return observation;
  }

  async cancelRun(reason = 'cancelled') {
    this.#assertLoaded();
    this.epoch += 1;
    const context = this.#context();
    const result = await this.#runLoadedOperation(() => this.backend.cancelRun({ reason, ...context }), context);
    this.#assertCurrent(context);
    this.activeCommandId = null;
    if (result?.reloadRequired) this.#clearLoadedState();
    return result;
  }

  exportTrace() {
    this.#assertLive();
    return this.backend.exportTrace(this.#context());
  }

  dispose() {
    if (this.disposed) return;
    this.disposed = true;
    this.epoch += 1;
    this.#clearLoadedState();
    this.observationListeners.clear();
    this.backend.dispose();
  }

  #context() {
    return { sessionId: this.sessionId, epoch: this.epoch };
  }

  #publishObservation(observation, source, context) {
    this.#assertCurrent(context);
    if (!observation || typeof observation !== 'object') return;
    const event = Object.freeze({
      source: String(source),
      sessionId: this.sessionId,
      epoch: this.epoch,
      sceneRevision: this.sceneRevision,
      robotId: this.robotId,
      observation: structuredClone(observation),
    });
    for (const listener of this.observationListeners) {
      this.#assertCurrent(context);
      try { listener(event); } catch { /* Observation consumers cannot become simulation authorities. */ }
    }
    this.#assertCurrent(context);
  }

  // An operation belongs to the epoch it started in, including every sample and every chunk.
  // Backends also reject stale RPCs, but that cannot protect publication after an RPC returns:
  // an observation subscriber may synchronously reset/dispose the session during delivery.
  #assertCurrent(context) {
    if (this.disposed || !context || context.epoch !== this.epoch || context.sessionId !== this.sessionId) {
      throw new Error('Stale physics session operation: its epoch was superseded');
    }
  }

  async #runLoadedOperation(operation, context) {
    this.#assertCurrent(context);
    try {
      const result = await operation();
      this.#assertCurrent(context);
      return result;
    } catch (error) {
      // A late failure is not evidence that the replacement scene has failed.
      if (this.disposed || context.epoch !== this.epoch) throw error;
      try {
        const diagnostics = await this.backend.getDiagnostics(context);
        if (!this.disposed && context.epoch === this.epoch && diagnostics?.loaded === false) this.#clearLoadedState();
      } catch {
        // Preserve the original operation failure. A diagnostic failure is not authority evidence.
      }
      throw error;
    }
  }

  #clearLoadedState() {
    this.activeCommandId = null;
    this.sceneRevision = null;
    this.robotId = null;
  }

  #assertLoaded() {
    this.#assertLive();
    if (!this.robotId || !this.sceneRevision) throw new Error('No physical scene loaded');
  }

  #assertLive() {
    if (this.disposed) throw new Error('Physics session is disposed');
  }
}
