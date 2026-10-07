import { bodyToWheelRadS, degreesPerSecondToRadians, MAX_WHEEL_RAD_S } from '../physics/lekiwi-kinematics.js';

export const LIVE_SIM_API_VERSION = 'robobuddy.sim.v1';

// LeKiwi keeps the public LeRobot chassis fields. They are a *request*: the bridge converts them
// through the pinned Kiwi body-to-wheel mapping into bounded wheel velocity targets, and MuJoCo
// decides the resulting base motion. get_observation() always returns achieved state.
const CHASSIS_FIELDS = Object.freeze(['x.vel', 'y.vel', 'theta.vel']);

const DEFAULT_TIMEOUT_MS = 15000;
const STEP_ALIGNMENT_TOLERANCE_SECONDS = 1e-9;
const DEFAULT_GOAL_TOLERANCE_RAD = 0.02;
const DEFAULT_CONTROLLER_PERIOD_SECONDS = 0.02;
const DEFAULT_GOAL_TIMEOUT_SECONDS = 2;
const DEFAULT_ACTION_STEP_BUDGET = 1000;

function liveError(code, message, details = {}) {
  const error = new Error(message);
  error.code = code;
  Object.assign(error, details);
  return error;
}

function finiteNonNegative(value, label) {
  const number = Number(value);
  if (!Number.isFinite(number) || number < 0) throw new RangeError(`${label} must be a finite non-negative number`);
  return number;
}

function finitePositive(value, label) {
  const number = Number(value);
  if (!Number.isFinite(number) || number <= 0) throw new RangeError(`${label} must be a finite positive number`);
  return number;
}

function boundedTargets(action) {
  if (!action || typeof action !== 'object' || Array.isArray(action)) throw new TypeError('action must be a joint-target object');
  const entries = Object.entries(action);
  if (!entries.length) throw new TypeError('action must contain at least one joint target');
  const targetsRad = {};
  const chassis = {};
  for (const [jointId, raw] of entries) {
    if (CHASSIS_FIELDS.includes(jointId)) {
      const value = Number(raw);
      if (!Number.isFinite(value)) throw new TypeError(`Chassis request ${jointId} must be finite`);
      chassis[jointId] = value;
      continue;
    }
    if (!jointId || jointId.endsWith('.pos')) throw new TypeError(`Physical API joint ${jointId || '<empty>'} must use the declared joint id, not legacy .pos fields`);
    const targetRad = Number(raw);
    if (!Number.isFinite(targetRad)) throw new TypeError(`Target for ${jointId} must be finite radians`);
    targetsRad[jointId] = targetRad;
  }
  return { targetsRad, chassis, hasChassis: Object.keys(chassis).length > 0 };
}

function chassisWheelTargets(chassis) {
  const body = {
    x: Number(chassis['x.vel'] ?? 0),
    y: Number(chassis['y.vel'] ?? 0),
    thetaRadS: degreesPerSecondToRadians(chassis['theta.vel'] ?? 0),
  };
  const { targetsRadS, saturated, scale } = bodyToWheelRadS(body, undefined, { maxWheelRadS: MAX_WHEEL_RAD_S });
  return { body, targetsRadS: { ...targetsRadS }, saturated, scale };
}

function ownerFromDiagnostics(diagnostics) {
  const timestepSeconds = Number(diagnostics?.timestepSeconds);
  if (!diagnostics?.loaded || !diagnostics?.sessionId || !Number.isInteger(diagnostics?.epoch)
    || diagnostics.epoch < 1 || !diagnostics?.sceneRevision || !diagnostics?.robotId
    || !Number.isFinite(timestepSeconds) || timestepSeconds <= 0) {
    throw liveError('SIMULATION_NOT_READY', 'Physical session is not loaded with an identified model timestep');
  }
  return Object.freeze({
    sessionId: String(diagnostics.sessionId),
    epoch: Number(diagnostics.epoch),
    sceneRevision: String(diagnostics.sceneRevision),
    robotId: String(diagnostics.robotId),
    backend: String(diagnostics.backend || 'unknown'),
    modelPackageId: diagnostics.modelPackageId == null ? null : String(diagnostics.modelPackageId),
    model: diagnostics.model ? structuredClone(diagnostics.model) : null,
    engineVersion: diagnostics.engineVersion == null ? null : String(diagnostics.engineVersion),
    timestepSeconds,
  });
}

function ownerMatches(owner, diagnostics) {
  return Boolean(owner && diagnostics?.loaded
    && owner.sessionId === String(diagnostics.sessionId)
    && owner.epoch === Number(diagnostics.epoch)
    && owner.sceneRevision === String(diagnostics.sceneRevision)
    && owner.robotId === String(diagnostics.robotId)
    && Math.abs(owner.timestepSeconds - Number(diagnostics.timestepSeconds)) <= 1e-12);
}

export class LivePythonBridge {
  constructor(session, { timeoutMs = DEFAULT_TIMEOUT_MS } = {}) {
    this.session = session;
    this.timeoutMs = timeoutMs;
    this.disposed = false;
    this.connected = false;
    this.owner = null;
    this.cancelled = false;
  }

  async connect(expectedRobotId = null) {
    this.#assertLive();
    const diagnostics = await this.#withTimeout(this.session.getDiagnostics());
    const owner = ownerFromDiagnostics(diagnostics);
    if (expectedRobotId && owner.robotId !== expectedRobotId) {
      throw liveError('PROFILE_MISMATCH', `Physical session robot ${owner.robotId} does not match requested ${expectedRobotId}`);
    }
    this.owner = owner;
    this.connected = true;
    this.cancelled = false;
    return this.#descriptor();
  }

  async sendAction(action, { commandId, maxSteps = DEFAULT_ACTION_STEP_BUDGET, durationSeconds = null } = {}) {
    await this.#assertOwner();
    if (!Number.isInteger(maxSteps) || maxSteps < 1) throw new RangeError('maxSteps must be a positive integer');
    const { targetsRad, chassis, hasChassis } = boundedTargets(action);
    const wheels = hasChassis ? chassisWheelTargets(chassis) : null;
    const command = hasChassis
      ? { type: 'set_mixed_targets', targetsRad, targetsRadS: wheels.targetsRadS }
      : { type: 'set_joint_targets', targetsRad };
    if (durationSeconds != null) {
      if (this.owner.robotId !== 'openarm_v2_bimanual' || hasChassis) throw new Error('duration_seconds is supported only by the OpenArm trajectory controller');
      if (typeof durationSeconds !== 'number' || !Number.isFinite(durationSeconds) || durationSeconds <= 0) throw new RangeError('duration_seconds must be positive finite seconds');
      command.type = 'move_joint_targets'; command.durationSeconds = durationSeconds;
    }
    const accepted = await this.#withTimeout(this.session.sendCommand(
      command,
      { ...(commandId ? { commandId: String(commandId) } : {}), maxSteps },
    ));
    return {
      apiVersion: LIVE_SIM_API_VERSION,
      status: String(accepted?.status || 'accepted'),
      commandId: accepted?.commandId || commandId || null,
      acceptedTargetsRad: structuredClone(targetsRad),
      ...(hasChassis ? {
        requestedChassisVelocity: structuredClone(chassis),
        acceptedWheelTargetsRadS: structuredClone(wheels.targetsRadS),
        wheelCommandSaturated: wheels.saturated,
        chassisNote: 'requestedChassisVelocity is a bounded command, not a measurement; read achieved motion from get_observation()',
      } : {}),
      remainingSteps: accepted?.remainingSteps ?? null,
      observation: accepted?.observation ? structuredClone(accepted.observation) : null,
      units: { jointPosition: 'rad', jointVelocity: 'rad/s', wheelVelocity: 'rad/s', chassisLinear: 'm/s', chassisAngular: 'deg/s', time: 's' },
    };
  }

  async advance(seconds) {
    await this.#assertOwner();
    const duration = finiteNonNegative(seconds, 'advance duration');
    if (duration === 0) return this.getObservation({ view: 'ground_truth' });
    const steps = this.#stepsForDuration(duration, 'advance duration');
    return this.#withTimeout(this.session.advanceSteps(steps));
  }

  async advanceControllerInterval(controllerPeriodSeconds = DEFAULT_CONTROLLER_PERIOD_SECONDS) {
    await this.#assertOwner();
    const duration = finitePositive(controllerPeriodSeconds, 'controllerPeriodSeconds');
    const steps = this.#stepsForDuration(duration, 'controllerPeriodSeconds');
    const observation = await this.#withTimeout(this.session.advanceSteps(steps));
    return { observation, controllerPeriodSeconds: duration, physicsSteps: steps, timestepSeconds: this.owner.timestepSeconds };
  }

  async getObservation({ view = 'ground_truth' } = {}) {
    await this.#assertOwner();
    if (view === 'hardware_like' && this.owner.robotId === 'asimov_1_23dof_physical') {
      const owner = this.owner;
      const observation = await this.#withTimeout(this.session.getObservation({ view:'ground_truth' }));
      await this.#assertOwner();
      if (this.owner !== owner || observation.sessionId !== owner.sessionId || observation.epoch !== owner.epoch) {
        throw liveError('STALE_LIVE_SESSION', 'The sensor read crossed a session reset or reconnect');
      }
      if (!observation.sensorObservation) throw liveError('UNSUPPORTED_OBSERVATION_PROFILE','This Asimov reference scene has no declared hardware-like sensitivity profile');
      return { ...structuredClone(observation.sensorObservation), sessionId:owner.sessionId, epoch:owner.epoch };
    }
    if (view !== 'ground_truth') {
      throw liveError('UNSUPPORTED_OBSERVATION_PROFILE', `Observation view ${view} is not calibrated or supported for this physical preview`);
    }
    return this.#withTimeout(this.session.getObservation({ view }));
  }

  async waitForGoal(targets, {
    toleranceRad = DEFAULT_GOAL_TOLERANCE_RAD,
    timeoutSeconds = DEFAULT_GOAL_TIMEOUT_SECONDS,
    controllerPeriodSeconds = DEFAULT_CONTROLLER_PERIOD_SECONDS,
  } = {}) {
    await this.#assertOwner();
    const { targetsRad: requested, hasChassis } = boundedTargets(targets);
    if (hasChassis) throw new TypeError('wait_for_goal accepts joint targets only; chassis velocity requests are not goal conditions');
    const tolerance = finitePositive(toleranceRad, 'toleranceRad');
    const timeout = finiteNonNegative(timeoutSeconds, 'timeoutSeconds');
    const controllerPeriod = finitePositive(controllerPeriodSeconds, 'controllerPeriodSeconds');
    const controllerSteps = this.#stepsForDuration(controllerPeriod, 'controllerPeriodSeconds');
    const maxIntervals = Math.ceil(timeout / controllerPeriod);
    const started = await this.getObservation({ view: 'ground_truth' });
    const startTime = Number(started.simulationTimeSeconds);

    const evaluate = (observation) => {
      const errorsRad = {};
      let complete = true;
      for (const [jointId, targetRad] of Object.entries(requested)) {
        const actual = Number(observation?.joints?.[jointId]?.positionRad);
        if (!Number.isFinite(actual)) {
          return { complete: false, unsupportedJoint: jointId, errorsRad };
        }
        errorsRad[jointId] = targetRad - actual;
        if (Math.abs(errorsRad[jointId]) > tolerance) complete = false;
      }
      return { complete, errorsRad };
    };

    let observation = started;
    let result = evaluate(observation);
    if (result.unsupportedJoint) {
      return { status: 'unsupported', reason: `Observation has no joint ${result.unsupportedJoint}`, observation, errorsRad: result.errorsRad };
    }
    if (result.complete) return { status: 'completed', observation, errorsRad: result.errorsRad, elapsedSimulationSeconds: 0 };

    for (let interval = 0; interval < maxIntervals; interval += 1) {
      if (this.cancelled || this.disposed) return { status: 'cancelled', observation, errorsRad: result.errorsRad };
      observation = await this.#withTimeout(this.session.advanceSteps(controllerSteps));
      result = evaluate(observation);
      if (result.unsupportedJoint) {
        return { status: 'unsupported', reason: `Observation has no joint ${result.unsupportedJoint}`, observation, errorsRad: result.errorsRad };
      }
      const elapsedSimulationSeconds = Number(observation.simulationTimeSeconds) - startTime;
      if (result.complete) return { status: 'completed', observation, errorsRad: result.errorsRad, elapsedSimulationSeconds };
      if (elapsedSimulationSeconds + STEP_ALIGNMENT_TOLERANCE_SECONDS >= timeout) break;
    }
    return {
      status: 'timeout',
      observation,
      errorsRad: result.errorsRad,
      elapsedSimulationSeconds: Number(observation.simulationTimeSeconds) - startTime,
    };
  }

  // --- free-base physical surface -------------------------------------------------------------
  // These are generic: the authority decides whether they are available. A scene that declares no
  // standing controller, or a backend that declares no setup operation, refuses them outright, so
  // no robot gains a capability here that its model package has not declared.

  /** Engage a declared standing controller. Acceptance is not achievement: read the state back. */
  async engageStand({ controllerId, commandId, maxSteps = 200000 } = {}) {
    await this.#assertOwner();
    if (!controllerId || typeof controllerId !== 'string') throw new TypeError('engage_stand requires a declared controller id');
    if (!Number.isInteger(maxSteps) || maxSteps < 1) throw new RangeError('maxSteps must be a positive integer');
    const accepted = await this.#withTimeout(this.session.sendCommand(
      { type: 'engage_stand', controllerId },
      { ...(commandId ? { commandId: String(commandId) } : {}), maxSteps },
    ));
    return {
      apiVersion: LIVE_SIM_API_VERSION,
      status: String(accepted?.status || 'accepted'),
      controllerId,
      commandId: accepted?.commandId || commandId || null,
      remainingSteps: accepted?.remainingSteps ?? null,
      note: 'the standing controller is engaged; it is a bounded posture controller and it can fail. Read achieved state from get_state().',
    };
  }

  async releaseStand() {
    await this.#assertOwner();
    const accepted = await this.#withTimeout(this.session.sendCommand({ type: 'release_stand' }, { maxSteps: 1 }));
    return { apiVersion: LIVE_SIM_API_VERSION, status: String(accepted?.status || 'accepted'), controllerId: null };
  }

  /** A declared pre-trial setup operation. It is logged by the authority and is never control. */
  async applyDeclaredSetup(payload = {}) {
    await this.#assertOwner();
    if (!payload || typeof payload !== 'object' || Array.isArray(payload) || !payload.type) throw new TypeError('setup requires a declared operation object');
    return this.#withTimeout(this.session.applySetup(structuredClone(payload)));
  }

  /**
   * Ground-truth state in the public snake_case shape. Every value is measured; a requested target
   * is reported beside the accepted command and the measured position, never in place of it.
   */
  async getState() {
    const observation = await this.getObservation({ view: 'ground_truth' });
    const joints = {};
    for (const [jointId, joint] of Object.entries(observation.joints || {})) {
      joints[jointId] = {
        requested_target_rad: joint.requestedTargetRad ?? joint.targetRad ?? null,
        accepted_target_rad: joint.acceptedTargetRad ?? joint.targetRad ?? null,
        position_rad: joint.positionRad,
        velocity_rad_s: joint.velocityRadS,
        effort_nm: joint.effortNm ?? null,
        effort_limit_nm: joint.effortLimitNm ?? null,
        velocity_limit_rad_s: joint.velocityLimitRadS ?? null,
        joint_range_rad: joint.jointRangeRad ?? null,
        command_bounded: joint.commandBounded ?? null,
        kp: joint.kp ?? null,
        kd: joint.kd ?? null,
      };
    }
    const classes = observation.contactClasses || null;
    return {
      apiVersion: LIVE_SIM_API_VERSION,
      simulation_time_s: observation.simulationTimeSeconds,
      model: observation.model ? structuredClone(observation.model) : null,
      root: observation.root ? {
        mode: observation.root.mode,
        position_m: observation.root.positionM,
        quaternion_wxyz: observation.root.quaternionWxyz,
        linear_velocity_m_s: observation.root.linearVelocityMS,
        angular_velocity_rad_s: observation.root.angularVelocityRadS,
        upright_z: observation.root.uprightZ,
        tilt_rad: observation.root.tiltRad,
      } : null,
      joints,
      foot_contacts: classes ? {
        left: (classes.leftFootFloor || []).map((entry) => entry.geoms),
        right: (classes.rightFootFloor || []).map((entry) => entry.geoms),
      } : null,
      contacts: classes ? {
        non_foot_ground: (classes.otherBodyFloor || []).map((entry) => entry.geoms),
        self: (classes.robotSelf || []).map((entry) => entry.geoms),
        external_object: (classes.robotExternalObject || []).map((entry) => entry.geoms),
        fixture: (classes.robotFixture || []).map((entry) => entry.geoms),
      } : null,
      controller_mode: observation.controller?.id ?? null,
      controller_claim: observation.controller?.claim ?? null,
      actuation_enabled: observation.actuationEnabled,
      ...(observation.actuatorModel ? {actuator_model:structuredClone(observation.actuatorModel),standing_assessment:structuredClone(observation.standingAssessment)} : {}),
      setup_log: Array.isArray(observation.setupLog) ? structuredClone(observation.setupLog) : [],
    };
  }

  async pause() {
    await this.#assertOwner();
    return this.#withTimeout(this.session.pause());
  }

  async resume() {
    await this.#assertOwner();
    return this.#withTimeout(this.session.resume());
  }

  async reset(options = {}) {
    await this.#assertOwner();
    const result = await this.#withTimeout(this.session.reset(structuredClone(options)));
    const diagnostics = await this.#withTimeout(this.session.getDiagnostics());
    this.owner = ownerFromDiagnostics(diagnostics);
    this.connected = true;
    this.cancelled = false;
    return { ...result, connection: this.#descriptor() };
  }

  async cancel(reason = 'python-cancelled') {
    if (this.disposed || !this.connected) return false;
    try {
      await this.#assertOwner();
    } catch (error) {
      if (error?.code === 'STALE_LIVE_SESSION' || error?.code === 'SIMULATION_NOT_READY') {
        this.connected = false;
        this.cancelled = true;
        return false;
      }
      throw error;
    }
    this.cancelled = true;
    const result = await this.#withTimeout(this.session.cancelRun(reason));
    this.connected = false;
    this.owner = null;
    return result;
  }

  disconnect() {
    this.connected = false;
    this.owner = null;
    return { disconnected: true };
  }

  dispose() {
    this.disposed = true;
    this.connected = false;
    this.owner = null;
  }

  #descriptor() {
    return Object.freeze({
      apiVersion: LIVE_SIM_API_VERSION,
      physical: true,
      ...structuredClone(this.owner),
      units: Object.freeze({ length: 'm', mass: 'kg', time: 's', angle: 'rad', angularVelocity: 'rad/s' }),
      observationProfiles: Object.freeze(this.owner.robotId === 'asimov_1_23dof_physical' && /(?:actuator|standing)/.test(this.owner.modelPackageId) ? ['ground_truth','hardware_like'] : ['ground_truth']),
    });
  }

  #stepsForDuration(duration, label) {
    const timestep = Number(this.owner?.timestepSeconds);
    if (!Number.isFinite(timestep) || timestep <= 0) throw liveError('SIMULATION_NOT_READY', 'Physical session did not report a valid MuJoCo timestep');
    const ratio = duration / timestep;
    const steps = Math.round(ratio);
    if (steps < 1 || Math.abs(duration - steps * timestep) > STEP_ALIGNMENT_TOLERANCE_SECONDS) {
      throw new RangeError(`${label} ${duration}s must be an integer number of ${timestep}s physics steps`);
    }
    return steps;
  }

  async #assertOwner() {
    this.#assertLive();
    if (!this.connected || !this.owner) throw liveError('SIMULATION_NOT_READY', 'Connect the live physical session before issuing commands');
    const diagnostics = await this.#withTimeout(this.session.getDiagnostics());
    if (!ownerMatches(this.owner, diagnostics)) {
      this.connected = false;
      throw liveError('STALE_LIVE_SESSION', 'The physical session epoch, scene revision, robot, or timestep changed; this Python run no longer owns it');
    }
    return diagnostics;
  }

  async #withTimeout(promise) {
    let timer;
    try {
      return await Promise.race([
        promise,
        new Promise((_, reject) => {
          timer = setTimeout(() => reject(liveError('SIMULATION_TIMEOUT', 'Live simulation operation timed out')), this.timeoutMs);
        }),
      ]);
    } finally {
      clearTimeout(timer);
    }
  }

  #assertLive() {
    if (this.disposed) throw liveError('OPERATION_CANCELLED', 'Live Python bridge is disposed');
  }
}
