import { PROFILES } from '../profiles.js';
import { LEKIWI_COURIER_PACKAGE } from '../physics/lekiwi-model-package.js';
import { MAX_WHEEL_RAD_S, maxBodySpeedMS, SOURCE_PARAMETERS, WHEEL_ORDER } from '../physics/lekiwi-kinematics.js';
import { basePoseFromObservation } from '../physics/lekiwi-task-evaluator.js';
import { WebMcpDomainError } from './agent-facade.js';

// The tool name stays `control_lekiwi_simulation` so existing agent workflows keep working. The
// schema is versioned because the semantics genuinely changed: a base command is now a bounded
// request that is translated into wheel actuation, and the arm is commanded in radians rather
// than the legacy normalized degree scale.
export const WEBMCP_LEKIWI_PHYSICAL_SCHEMA_VERSION = 'robobuddy.lekiwi.physical.v1';
const MAX_ADVANCE_SECONDS = 2;
const DEFAULT_COMMAND_STEPS = 2000;
const MAX_COMMAND_STEPS = 8000;

const MAX_FORWARD_MS = Number(maxBodySpeedMS([1, 0], SOURCE_PARAMETERS).toFixed(4));
const MAX_LATERAL_MS = Number(maxBodySpeedMS([0, 1], SOURCE_PARAMETERS).toFixed(4));
const MAX_LINEAR_MS = Math.min(MAX_FORWARD_MS, MAX_LATERAL_MS);
const MAX_THETA_DEG_S = Number((MAX_WHEEL_RAD_S * SOURCE_PARAMETERS.wheelRadiusM
  / Math.max(...SOURCE_PARAMETERS.rows.map((row) => row.momentArmM)) * 180 / Math.PI).toFixed(2));

const ARM_RANGES = Object.freeze(Object.fromEntries(
  LEKIWI_COURIER_PACKAGE.actuators
    .filter((actuator) => actuator.command === 'position-rad')
    .map((actuator) => {
      const joint = LEKIWI_COURIER_PACKAGE.joints.find((item) => item.id === actuator.jointId);
      const minimum = Math.max(Number(joint.rangeRad[0]), Number(actuator.controlRangeRad[0]));
      const maximum = Math.min(Number(joint.rangeRad[1]), Number(actuator.controlRangeRad[1]));
      if (!Number.isFinite(minimum) || !Number.isFinite(maximum) || minimum >= maximum) throw new Error(`LeKiwi physical WebMCP has no valid control intersection for ${actuator.jointId}`);
      return [actuator.jointId, Object.freeze([minimum, maximum])];
    }),
));

function invalid(message) { throw new WebMcpDomainError('INVALID_ARGUMENT', message, { retryable: false }); }
function plain(value, label = 'input') { if (!value || typeof value !== 'object' || Array.isArray(value)) invalid(`${label} must be an object.`); }
function onlyKeys(value, allowed, label = 'input') { for (const key of Object.keys(value)) if (!allowed.includes(key)) invalid(`Unexpected ${label} field: ${key}.`); }

export function createLeKiwiPhysicalControlSchema() {
  const armTargets = {
    type: 'object',
    properties: Object.fromEntries(Object.entries(ARM_RANGES).map(([jointId, range]) => [jointId, { type: 'number', minimum: range[0], maximum: range[1], description: 'radians; SO-ARM101 MuJoCo position target' }])),
    minProperties: 1,
    additionalProperties: false,
  };
  const base = {
    type: 'object',
    properties: {
      'x.vel': { type: 'number', minimum: -MAX_FORWARD_MS, maximum: MAX_FORWARD_MS, description: 'requested forward chassis velocity, m/s' },
      'y.vel': { type: 'number', minimum: -MAX_LATERAL_MS, maximum: MAX_LATERAL_MS, description: 'requested left chassis velocity, m/s' },
      'theta.vel': { type: 'number', minimum: -MAX_THETA_DEG_S, maximum: MAX_THETA_DEG_S, description: 'requested yaw rate, deg/s (LeRobot public unit)' },
    },
    minProperties: 1,
    additionalProperties: false,
  };
  return {
    type: 'object',
    oneOf: [
      {
        type: 'object',
        properties: {
          schema_version: { type: 'string', const: WEBMCP_LEKIWI_PHYSICAL_SCHEMA_VERSION },
          command: { type: 'string', const: 'set_chassis_velocity' },
          chassis_velocity: base,
          advance_seconds: { type: 'number', minimum: 0, maximum: MAX_ADVANCE_SECONDS, description: 'Optional bounded simulation-time advancement; must align to the 0.002 s LeKiwi timestep.' },
          max_steps: { type: 'integer', minimum: 1, maximum: MAX_COMMAND_STEPS, description: 'Hard physics-step budget for the latched wheel command.' },
        },
        required: ['schema_version', 'command', 'chassis_velocity'],
        additionalProperties: false,
      },
      {
        type: 'object',
        properties: {
          schema_version: { type: 'string', const: WEBMCP_LEKIWI_PHYSICAL_SCHEMA_VERSION },
          command: { type: 'string', const: 'set_arm_targets' },
          targets_rad: armTargets,
          advance_seconds: { type: 'number', minimum: 0, maximum: MAX_ADVANCE_SECONDS },
          max_steps: { type: 'integer', minimum: 1, maximum: MAX_COMMAND_STEPS },
        },
        required: ['schema_version', 'command', 'targets_rad'],
        additionalProperties: false,
      },
      {
        type: 'object',
        properties: { schema_version: { type: 'string', const: WEBMCP_LEKIWI_PHYSICAL_SCHEMA_VERSION }, command: { type: 'string', const: 'stop' } },
        required: ['schema_version', 'command'],
        additionalProperties: false,
      },
      {
        type: 'object',
        properties: { schema_version: { type: 'string', const: WEBMCP_LEKIWI_PHYSICAL_SCHEMA_VERSION }, command: { type: 'string', const: 'reset' } },
        required: ['schema_version', 'command'],
        additionalProperties: false,
      },
    ],
    additionalProperties: false,
  };
}

export function getLeKiwiPhysicalControlDefinition(facade) {
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || !context.simulationReady || context.profileId !== 'lekiwi' || context.simulationMode !== 'physical_mujoco') return null;
  return {
    name: 'control_lekiwi_simulation',
    title: 'Control LeKiwi physical MuJoCo simulation',
    description: `Control the active LeKiwi V1 physical mobile-manipulation workspace through ${WEBMCP_LEKIWI_PHYSICAL_SCHEMA_VERSION}. A chassis velocity is a bounded request: it is converted through the pinned LeRobot Kiwi mapping into bounded wheel velocity targets, and MuJoCo decides the actual base motion from wheel/ground contact. Arm targets are explicit radians. Every call routes to the same PhysicsSession used by live Python, rendering and task evaluation. No ZMQ, hardware transport, base-pose write, object transform, weld, snap, save, export or publish capability is exposed.`,
    inputSchema: createLeKiwiPhysicalControlSchema(),
  };
}

function parse(input) {
  plain(input);
  if (input.schema_version !== WEBMCP_LEKIWI_PHYSICAL_SCHEMA_VERSION) invalid(`schema_version must be ${WEBMCP_LEKIWI_PHYSICAL_SCHEMA_VERSION}.`);
  if (input.command === 'reset' || input.command === 'stop') {
    onlyKeys(input, ['schema_version', 'command']);
    return { command: input.command };
  }
  const advanceOf = (value) => {
    const seconds = value == null ? 0 : Number(value);
    if (!Number.isFinite(seconds) || seconds < 0 || seconds > MAX_ADVANCE_SECONDS) invalid(`advance_seconds must be between 0 and ${MAX_ADVANCE_SECONDS}.`);
    return seconds;
  };
  const stepsOf = (value) => {
    const steps = value == null ? DEFAULT_COMMAND_STEPS : value;
    if (!Number.isInteger(steps) || steps < 1 || steps > MAX_COMMAND_STEPS) invalid(`max_steps must be an integer from 1 to ${MAX_COMMAND_STEPS}.`);
    return steps;
  };
  if (input.command === 'set_chassis_velocity') {
    onlyKeys(input, ['schema_version', 'command', 'chassis_velocity', 'advance_seconds', 'max_steps']);
    plain(input.chassis_velocity, 'chassis_velocity');
    const entries = Object.entries(input.chassis_velocity);
    if (!entries.length) invalid('chassis_velocity must contain at least one field.');
    const action = { 'x.vel': 0, 'y.vel': 0, 'theta.vel': 0 };
    const bounds = { 'x.vel': MAX_FORWARD_MS, 'y.vel': MAX_LATERAL_MS, 'theta.vel': MAX_THETA_DEG_S };
    for (const [key, raw] of entries) {
      if (!(key in bounds)) invalid(`Unknown LeKiwi chassis field: ${key}.`);
      const value = Number(raw);
      if (!Number.isFinite(value) || Math.abs(value) > bounds[key]) invalid(`${key} must be within +/-${bounds[key]}.`);
      action[key] = value;
    }
    return { command: 'set_chassis_velocity', action, advanceSeconds: advanceOf(input.advance_seconds), maxSteps: stepsOf(input.max_steps) };
  }
  if (input.command !== 'set_arm_targets') invalid('LeKiwi physical command must be set_chassis_velocity, set_arm_targets, stop or reset.');
  onlyKeys(input, ['schema_version', 'command', 'targets_rad', 'advance_seconds', 'max_steps']);
  plain(input.targets_rad, 'targets_rad');
  const entries = Object.entries(input.targets_rad);
  if (!entries.length) invalid('targets_rad must contain at least one arm joint target.');
  const targetsRad = {};
  for (const [jointId, raw] of entries) {
    const range = ARM_RANGES[jointId];
    if (!range) invalid(`Unknown LeKiwi physical arm joint: ${jointId}.`);
    const value = Number(raw);
    if (!Number.isFinite(value) || value < range[0] || value > range[1]) invalid(`${jointId} must be within ${range[0]}..${range[1]} radians.`);
    targetsRad[jointId] = value;
  }
  return { command: 'set_arm_targets', targetsRad, advanceSeconds: advanceOf(input.advance_seconds), maxSteps: stepsOf(input.max_steps) };
}

function sameAuthority(a, b) { return Boolean(a && b && a.sessionId === b.sessionId && Number(a.epoch) === Number(b.epoch) && a.sceneRevision === b.sceneRevision && a.robotId === b.robotId); }
function capture(facade, expectedEpoch) {
  facade.assertActive(expectedEpoch);
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || context.profileId !== 'lekiwi' || context.simulationMode !== 'physical_mujoco' || !context.simulationReady) throw new WebMcpDomainError('PROFILE_MISMATCH', 'This tool requires the active ready LeKiwi physical workspace.', { retryable: true });
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'Finish or stop the active Python run before direct LeKiwi control.', { retryable: true });
  const authority = facade.app.sim.getPhysicalAuthorityToken?.();
  if (!authority?.sessionId) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'LeKiwi PhysicsSession authority is unavailable.', { retryable: true });
  return Object.freeze({ workspaceGeneration: context.workspaceGeneration, simulatorEpoch: context.simulatorEpoch, authority: Object.freeze({ ...authority }) });
}
function assertCurrent(facade, baseline, expectedEpoch, signal, { allowAuthorityChange = false } = {}) {
  if (signal?.aborted) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The LeKiwi control call was cancelled.', { retryable: true });
  facade.assertActive(expectedEpoch);
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || context.profileId !== 'lekiwi' || context.simulationMode !== 'physical_mujoco' || context.workspaceGeneration !== baseline.workspaceGeneration || context.simulatorEpoch !== baseline.simulatorEpoch) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The active LeKiwi workspace or simulator changed during control.', { retryable: true });
  if (!allowAuthorityChange && !sameAuthority(baseline.authority, facade.app.sim.getPhysicalAuthorityToken?.())) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The LeKiwi PhysicsSession epoch changed during control.', { retryable: true });
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'A Python run acquired the simulation during direct control.', { retryable: true });
}
function observed(observation) {
  const pose = basePoseFromObservation(observation);
  return {
    simulationTimeSeconds: Number(observation?.simulationTimeSeconds || 0),
    basePose: pose ? { xM: pose.xM, yM: pose.yM, yawRad: pose.yawRad, speedMS: pose.speedMS, yawRateRadS: pose.yawRateRadS } : null,
    wheelVelocityRadS: Object.fromEntries(WHEEL_ORDER.map((id) => [id, Number(observation?.joints?.[id]?.velocityRadS ?? Number.NaN)])),
    wheelTargetRadS: Object.fromEntries(WHEEL_ORDER.map((id) => [id, Number(observation?.joints?.[id]?.targetVelocityRadS ?? Number.NaN)])),
    wheelEffortNm: Object.fromEntries(WHEEL_ORDER.map((id) => [id, Number(observation?.joints?.[id]?.effortNm ?? Number.NaN)])),
    armRad: Object.fromEntries(Object.keys(ARM_RANGES).map((id) => [id, Number(observation?.joints?.[id]?.positionRad ?? Number.NaN)])),
    beakerPositionM: observation?.bodies?.empty_beaker?.positionM ? [...observation.bodies.empty_beaker.positionM] : null,
    contactCount: Number(observation?.contactCount || 0),
  };
}

export async function executeLeKiwiPhysicalControl(facade, input, signal, expectedEpoch) {
  const parsed = parse(input);
  const baseline = capture(facade, expectedEpoch);
  if (facade.activeControlId) throw new WebMcpDomainError('COMMAND_CONFLICT', 'Another bounded robot-control call is active.', { retryable: true });
  const controlId = `webmcp-lekiwi-${expectedEpoch}-${++facade.controlSequence}`;
  facade.activeControlId = controlId;
  const base = { ok: true, profileId: 'lekiwi', robot: PROFILES.lekiwi.label, schemaVersion: WEBMCP_LEKIWI_PHYSICAL_SCHEMA_VERSION, hardwareValidated: false };
  try {
    assertCurrent(facade, baseline, expectedEpoch, signal);
    if (parsed.command === 'reset') {
      if (!(await facade.app.resetSimulation())) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'LeKiwi physical simulation could not be reset.', { retryable: true });
      assertCurrent(facade, baseline, expectedEpoch, signal, { allowAuthorityChange: true });
      return { ...base, command: 'reset', reset: true, physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.() };
    }
    if (parsed.command === 'stop') {
      const result = await facade.app.sim.applyChassisVelocity({ 'x.vel': 0, 'y.vel': 0, 'theta.vel': 0 }, { maxSteps: 400 });
      assertCurrent(facade, baseline, expectedEpoch, signal);
      facade.app.setStatus?.('Agent commanded a bounded LeKiwi wheel stop');
      facade.app.renderPanels?.();
      return { ...base, command: 'stop', commandStatus: result.status, commandId: result.commandId, acceptedWheelTargetsRadS: result.acceptedWheelTargetsRadS, observedState: observed(result.observation), taskEvaluation: result.taskEvaluation, physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.() };
    }
    if (parsed.command === 'set_chassis_velocity') {
      const result = await facade.app.sim.applyChassisVelocity(parsed.action, { maxSteps: parsed.maxSteps, advanceSeconds: parsed.advanceSeconds });
      assertCurrent(facade, baseline, expectedEpoch, signal);
      facade.app.setStatus?.('Agent requested a bounded LeKiwi chassis velocity');
      facade.app.renderPanels?.();
      return {
        ...base,
        command: 'set_chassis_velocity',
        commandStatus: result.status,
        commandId: result.commandId,
        requestedChassisVelocity: result.requestedChassisVelocity,
        acceptedWheelTargetsRadS: result.acceptedWheelTargetsRadS,
        wheelCommandSaturated: result.saturated,
        advanceSeconds: parsed.advanceSeconds,
        observedState: observed(result.observation),
        taskEvaluation: result.taskEvaluation,
        physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.(),
        note: 'requestedChassisVelocity is a command, not a measurement; observedState.basePose is the achieved MuJoCo state.',
      };
    }
    const result = await facade.app.sim.applyArmTargets(parsed.targetsRad, { maxSteps: parsed.maxSteps, advanceSeconds: parsed.advanceSeconds });
    assertCurrent(facade, baseline, expectedEpoch, signal);
    facade.app.setStatus?.('Agent applied bounded LeKiwi arm targets');
    facade.app.renderPanels?.();
    return { ...base, command: 'set_arm_targets', commandStatus: result.status, commandId: result.commandId, acceptedTargetsRad: result.acceptedTargetsRad, advanceSeconds: parsed.advanceSeconds, observedState: observed(result.observation), taskEvaluation: result.taskEvaluation, physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.() };
  } catch (error) {
    if (error instanceof WebMcpDomainError) throw error;
    throw new WebMcpDomainError('SIMULATION_REJECTED', String(error?.message || error).slice(0, 280), { retryable: true, details: { profileId: 'lekiwi' } });
  } finally {
    if (facade.activeControlId === controlId) facade.activeControlId = null;
  }
}

export const LEKIWI_PHYSICAL_CONTROL_LIMITS = Object.freeze({
  maxForwardMS: MAX_FORWARD_MS,
  maxLateralMS: MAX_LATERAL_MS,
  maxLinearMS: MAX_LINEAR_MS,
  maxThetaDegS: MAX_THETA_DEG_S,
  maxWheelRadS: MAX_WHEEL_RAD_S,
  maxCommandSteps: MAX_COMMAND_STEPS,
  maxAdvanceSeconds: MAX_ADVANCE_SECONDS,
});
