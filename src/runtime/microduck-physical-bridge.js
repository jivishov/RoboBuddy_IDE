import { LIVE_SIM_API_VERSION } from './live-python-bridge.js';
import { MICRODUCK_MAX_ADVANCE_SECONDS } from '../physics/microduck-physical-simulator.js';
import { MICRODUCK_CONTROL_INTERVAL_SECONDS, MICRODUCK_POLICY_JOINT_ORDER } from '../physics/microduck-controller.js';
import { MICRODUCK_CAPABILITY_AUDIT, MICRODUCK_PHYSICAL_SKILL_IDS, MICRODUCK_UNSUPPORTED_CAPABILITIES, microduckCapability } from '../physics/microduck-capabilities.js';
import { MICRODUCK_COMMAND_LIMITS, MICRODUCK_GAIT_ONSET_MS } from '../physics/microduck-scene.js';

// The live-Python bridge for the MicroDuck physical workspace.
//
// It implements the same boundary methods PhysicalPythonRuntime already dispatches, so the
// worker, the run epoch, pause/step/cancel and the stale-bridge rejection are all reused
// unchanged. What differs is the command surface: a learner program does NOT write joint
// targets here. It issues the same bounded velocity/head/body command the deployed robot
// takes, and the policy plus the plant decide what happens.
//
// Nothing reachable from Python can write root pose, root velocity, ball state or a task
// outcome. `get_observation` deliberately separates the requested command, the controller's
// decision and the actual physical state, so a program cannot mistake one for another.

const COMMAND_FIELDS = Object.freeze(['vx', 'vy', 'vyaw', 'neckPitch', 'headPitch', 'headYaw', 'headRoll', 'bodyZ', 'bodyRoll', 'bodyPitch']);
const SKILL_IDS = Object.freeze([...MICRODUCK_PHYSICAL_SKILL_IDS, ...MICRODUCK_UNSUPPORTED_CAPABILITIES]);

function liveError(code, message, details = null) {
  const error = new Error(message);
  error.code = code;
  if (details) error.details = details;
  return error;
}

function finiteNonNegative(value, label) {
  const number = value;
  if (typeof number !== 'number' || !Number.isFinite(number) || number < 0) throw new RangeError(`${label} must be a finite non-negative number`);
  return number;
}

export class MicroDuckPhysicalBridge {
  constructor(simulator) {
    if (!simulator) throw new TypeError('MicroDuckPhysicalBridge requires the physical simulator');
    this.simulator = simulator;
    this.connected = false;
    this.disposed = false;
    this.cancelled = false;
    this.owner = null;
  }

  async connect(expectedRobotId = null) {
    this.#assertLive();
    const authority = this.simulator.getPhysicalAuthorityToken?.();
    if (!authority?.sessionId) throw liveError('SIMULATION_NOT_READY', 'MicroDuck PhysicsSession authority is unavailable');
    if (expectedRobotId && authority.robotId !== expectedRobotId) {
      throw liveError('PROFILE_MISMATCH', `Physical session robot ${authority.robotId} does not match requested ${expectedRobotId}`);
    }
    this.owner = { ...authority, runEpoch: this.simulator.runEpoch };
    this.connected = true;
    this.cancelled = false;
    return Object.freeze({
      apiVersion: LIVE_SIM_API_VERSION,
      physical: true,
      ...structuredClone(this.owner),
      backend: 'browser-mujoco',
      commandFields: [...COMMAND_FIELDS],
      skillCommands: [...MICRODUCK_PHYSICAL_SKILL_IDS],
      commandLimits: structuredClone(MICRODUCK_COMMAND_LIMITS),
      gaitOnsetMS: MICRODUCK_GAIT_ONSET_MS,
      controlIntervalSeconds: MICRODUCK_CONTROL_INTERVAL_SECONDS,
      policyJoints: [...MICRODUCK_POLICY_JOINT_ORDER],
      capabilities: MICRODUCK_CAPABILITY_AUDIT.map((item) => ({ id: item.id, status: item.status, physical: Boolean(item.physicalPolicy) })),
      units: Object.freeze({ length: 'm', time: 's', angle: 'rad', linearVelocity: 'm/s', angularVelocity: 'rad/s' }),
      observationProfiles: Object.freeze(['ground_truth']),
      note: 'A command is a request into a trained policy and a MuJoCo plant. Nothing here writes root pose, root velocity, ball state or task success.',
    });
  }

  /**
   * Latch a bounded command, optionally requesting a skill, and optionally advance.
   *
   * `action` is a plain object of command fields, with an optional `skill`. Unknown fields
   * and unsupported skills are rejected rather than ignored.
   */
  async sendAction(action, { maxSteps = null } = {}) {
    this.#assertOwner();
    if (!action || typeof action !== 'object' || Array.isArray(action)) throw liveError('INVALID_ARGUMENT', 'send_action requires a command object');
    const request = {};
    let skill = null;
    let advanceSeconds = 0;
    for (const [key, raw] of Object.entries(action)) {
      if (key === 'skill') {
        skill = raw;
        if (typeof skill !== 'string' || !SKILL_IDS.includes(skill)) throw liveError('INVALID_ARGUMENT', 'Unknown MicroDuck skill command; use sit or stand_up for sit/stand');
        continue;
      }
      if (key === 'advance_seconds') { advanceSeconds = finiteNonNegative(raw, 'advance_seconds'); continue; }
      if (!COMMAND_FIELDS.includes(key)) throw liveError('INVALID_ARGUMENT', `Unknown MicroDuck command field: ${key}`);
      const value = raw;
      if (typeof value !== 'number' || !Number.isFinite(value)) throw liveError('INVALID_ARGUMENT', `${key} must be a finite number`);
      request[key] = value;
    }
    if (advanceSeconds > MICRODUCK_MAX_ADVANCE_SECONDS) throw new RangeError(`advance_seconds is bounded to ${MICRODUCK_MAX_ADVANCE_SECONDS} simulated seconds per call`);
    void maxSteps;

    // Validate the entire combined action before latching either part. A rejected
    // skill must not leave its accompanying velocity command running.
    if (Object.keys(request).length) this.simulator.validateCommand(request);
    let skillResult = null;
    if (skill) {
      skillResult = this.simulator.validateSkillRequest(skill);
      if (!skillResult.accepted) {
        const capability = microduckCapability(skill);
        const plantMismatch = skillResult.status === 'wrong-plant';
        throw liveError(plantMismatch ? 'PLANT_MISMATCH' : 'CAPABILITY_UNSUPPORTED', `${skill} cannot run in the active MicroDuck physical plant: ${skillResult.reason}`, {
          capability: skill, status: plantMismatch ? skillResult.status : (capability?.status ?? 'unsupported in physical mode'), requiredPackageKeys: skillResult.requiredPackageKeys || [], routedToLegacy: false,
        });
      }
    }
    const accepted = Object.keys(request).length ? this.simulator.setCommand(request) : null;
    if (skill) skillResult = this.simulator.requestSkill(skill);
    let run = null;
    if (advanceSeconds > 0) run = await this.simulator.advanceSeconds(advanceSeconds);
    this.#assertOwner();
    return {
      apiVersion: LIVE_SIM_API_VERSION,
      status: 'accepted',
      requested: accepted ? accepted.requested : structuredClone(this.simulator.getState().requested.command),
      limitedBy: accepted ? accepted.limitedBy : [],
      skill: skill ? { id: skill, status: skillResult.status, capability: skillResult.capability } : null,
      advanceSeconds,
      executedTicks: run?.executedTicks ?? 0,
      completed: run ? run.completed : null,
      note: 'Accepting a command is not achieving it. Read achieved motion from get_observation().',
    };
  }

  /** Advance bounded simulation time through the fixed-cadence controller loop. */
  async advance(seconds) {
    this.#assertOwner();
    const duration = finiteNonNegative(seconds, 'advance duration');
    if (duration === 0) return this.getObservation();
    if (duration > MICRODUCK_MAX_ADVANCE_SECONDS) throw new RangeError(`advance duration is bounded to ${MICRODUCK_MAX_ADVANCE_SECONDS} simulated seconds per call`);
    const run = await this.simulator.advanceSeconds(duration);
    return { ...this.getObservation(), executedTicks: run.executedTicks, completed: run.completed, cancelled: run.cancelled };
  }

  /**
   * The three views, deliberately separate: what was requested, what the controller decided,
   * and what the physics actually is. Only `actual` is a measurement.
   */
  getObservation({ view = 'ground_truth' } = {}) {
    this.#assertOwner();
    if (view !== 'ground_truth') throw liveError('INVALID_ARGUMENT', `Observation view ${view} is unsupported by the physical preview`);
    const state = this.simulator.getState();
    return {
      apiVersion: LIVE_SIM_API_VERSION,
      view,
      requested: state.requested,
      controller: state.controller,
      actual: state.actual,
      taskEvaluation: this.simulator.report(),
      capabilities: state.capabilities,
      hardwareValidated: false,
    };
  }

  /**
   * Wait until the trunk settles, or until the bounded timeout. It reports what actually
   * happened; it never asserts success.
   */
  async waitForGoal(goal = {}, { timeoutSeconds = 2, controllerPeriodSeconds = MICRODUCK_CONTROL_INTERVAL_SECONDS } = {}) {
    this.#assertOwner();
    if (!goal || typeof goal !== 'object' || Array.isArray(goal) || Object.keys(goal).some((key) => key !== 'upright')
      || (goal.upright !== undefined && typeof goal.upright !== 'boolean')) {
      throw liveError('INVALID_ARGUMENT', 'MicroDuck wait_for_goal supports only an upright boolean goal');
    }
    const timeout = Math.min(finiteNonNegative(timeoutSeconds, 'timeout_seconds'), MICRODUCK_MAX_ADVANCE_SECONDS);
    const period = finiteNonNegative(controllerPeriodSeconds, 'controller_period_seconds');
    const dt = MICRODUCK_CONTROL_INTERVAL_SECONDS;
    // Use whole controller ticks, flooring the budget so a wait never exceeds its timeout.
    const budgetTicks = Math.floor(timeout / dt + 1e-10);
    const periodTicks = Math.max(1, Math.min(Math.floor(MICRODUCK_MAX_ADVANCE_SECONDS / dt), Math.floor(period / dt + 1e-10)));
    const desired = goal.upright !== false;
    const goalReached = () => this.simulator.report()?.upright === desired;
    let reached = goalReached();
    let elapsedTicks = 0;
    let interrupted = false;
    while (!reached && elapsedTicks < budgetTicks) {
      this.#assertOwner();
      const ticks = Math.min(periodTicks, budgetTicks - elapsedTicks);
      const run = await this.simulator.advanceSeconds(ticks * dt);
      this.#assertOwner();
      elapsedTicks += run.executedTicks;
      reached = goalReached();
      if (!run.completed) { interrupted = true; break; }
    }
    return { ...this.getObservation(), reached, timedOut: !reached && !interrupted,
      interrupted, timeoutSeconds: timeout, simulatedSeconds: elapsedTicks * dt };
  }

  async pause() { this.#assertOwner(); await this.simulator.pause(); return { paused: true }; }
  async resume() { this.#assertOwner(); await this.simulator.resume(); return { resumed: true }; }

  /** A reset establishes a new initial condition. It is never a recovery and never a success. */
  async reset() {
    this.#assertOwner();
    await this.simulator.reset();
    this.owner = { ...this.simulator.getPhysicalAuthorityToken(), runEpoch: this.simulator.runEpoch };
    return { ...this.getObservation(), reset: true, countsAsRecovery: false };
  }

  async cancel(reason = 'cancelled') {
    if (!this.connected) return false;
    this.cancelled = true;
    this.simulator.cancel(reason);
    this.connected = false;
    this.owner = null;
    return { cancelled: true, reason: String(reason) };
  }

  disconnect() { this.connected = false; this.owner = null; return { disconnected: true }; }
  dispose() { this.disposed = true; this.connected = false; this.owner = null; }

  #assertLive() { if (this.disposed) throw liveError('SIMULATION_NOT_READY', 'MicroDuck physical bridge is disposed'); }

  // A bridge from a replaced workspace or a cancelled run must never reach the new session.
  #assertOwner() {
    this.#assertLive();
    if (!this.connected || !this.owner) throw liveError('SIMULATION_NOT_READY', 'MicroDuck physical bridge is not connected');
    if (this.cancelled) throw liveError('OPERATION_CANCELLED', 'This MicroDuck run was cancelled');
    const authority = this.simulator.getPhysicalAuthorityToken?.();
    if (!authority?.sessionId) throw liveError('SIMULATION_NOT_READY', 'MicroDuck PhysicsSession authority is unavailable');
    if (authority.sessionId !== this.owner.sessionId || authority.sceneRevision !== this.owner.sceneRevision || authority.robotId !== this.owner.robotId
      || authority.epoch !== this.owner.epoch || this.simulator.runEpoch !== this.owner.runEpoch) {
      throw liveError('OPERATION_CANCELLED', 'The MicroDuck physical session was replaced during this run');
    }
  }
}
