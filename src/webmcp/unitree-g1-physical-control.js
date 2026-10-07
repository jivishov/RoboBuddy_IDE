import { PROFILES } from '../profiles.js';
import { G1_JOINT_ORDER, G1_JOINT_RANGE_RAD } from '../physics/unitree-g1-source-audit.js';
import { UNITREE_G1_FREEBASE_PACKAGE } from '../physics/unitree-g1-model-package.js';
import { UNITREE_G1_CAPABILITY_AUDIT } from '../physics/unitree-g1-capabilities.js';
import { G1_CONTROLLERS, G1_STAND_CONTROLLER_PROFILES } from '../physics/unitree-g1-controller.js';
import { WebMcpDomainError } from './agent-facade.js';

// Bounded agent access to the Unitree G1 physical workspace.
//
// Every command routes to the same PhysicsSession that live Python, the renderer and the task
// evaluator observe. The tool exposes no set_root_pose, no set_root_velocity, no force_upright,
// no external force, no object placement and no walk: those either do not exist in the authority
// or are not agent-reachable operations.
export const WEBMCP_UNITREE_G1_SCHEMA_VERSION = 'robobuddy.unitree-g1.physical.v1';
const MAX_ADVANCE_SECONDS = 2;
const DEFAULT_COMMAND_STEPS = 4000;
const MAX_COMMAND_STEPS = 40000;
const MAX_STAND_HOLD_SECONDS = 2;

// The single verified standing controller. The source FixStand profile exists in the authority as
// measured negative evidence and is deliberately not agent-reachable.
const AGENT_STAND_CONTROLLER = G1_CONTROLLERS.STAND;

const JOINT_RANGES = Object.freeze(Object.fromEntries(G1_JOINT_ORDER.map((jointId, index) => [jointId, Object.freeze([...G1_JOINT_RANGE_RAD[index]])])));

function invalid(message) { throw new WebMcpDomainError('INVALID_ARGUMENT', message, { retryable: false }); }
function plain(value, label = 'input') { if (!value || typeof value !== 'object' || Array.isArray(value)) invalid(`${label} must be an object.`); }
function onlyKeys(value, allowed, label = 'input') { for (const key of Object.keys(value)) if (!allowed.includes(key)) invalid(`Unexpected ${label} field: ${key}.`); }

export function createUnitreeG1ControlSchema() {
  const version = { type: 'string', const: WEBMCP_UNITREE_G1_SCHEMA_VERSION };
  const targets = {
    type: 'object',
    properties: Object.fromEntries(Object.entries(JOINT_RANGES).map(([jointId, range]) => [jointId, {
      type: 'number', minimum: range[0], maximum: range[1],
      description: 'radians; a bounded low-level position target, not an achieved position',
    }])),
    minProperties: 1,
    additionalProperties: false,
  };
  const bare = (command) => ({
    type: 'object',
    properties: { schema_version: version, command: { type: 'string', const: command } },
    required: ['schema_version', 'command'],
    additionalProperties: false,
  });
  return {
    type: 'object',
    oneOf: [
      bare('inspect_capability'),
      bare('read_state'),
      bare('stop'),
      bare('reset'),
      {
        type: 'object',
        properties: {
          schema_version: version,
          command: { type: 'string', const: 'set_joint_targets' },
          targets_rad: targets,
          advance_seconds: { type: 'number', minimum: 0, maximum: MAX_ADVANCE_SECONDS, description: 'Optional bounded simulation-time advancement.' },
          max_steps: { type: 'integer', minimum: 1, maximum: MAX_COMMAND_STEPS, description: 'Hard physics-step budget for the latched command.' },
        },
        required: ['schema_version', 'command', 'targets_rad'],
        additionalProperties: false,
      },
      {
        type: 'object',
        properties: {
          schema_version: version,
          command: { type: 'string', const: 'advance' },
          advance_seconds: { type: 'number', exclusiveMinimum: 0, maximum: MAX_ADVANCE_SECONDS },
        },
        required: ['schema_version', 'command', 'advance_seconds'],
        additionalProperties: false,
      },
      {
        type: 'object',
        properties: {
          schema_version: version,
          command: { type: 'string', const: 'stand' },
          hold_seconds: { type: 'number', minimum: 0, maximum: MAX_STAND_HOLD_SECONDS, description: 'Optional bounded simulation-time hold after engaging the controller.' },
        },
        required: ['schema_version', 'command'],
        additionalProperties: false,
      },
    ],
    additionalProperties: false,
  };
}

export function getUnitreeG1PhysicalControlDefinition(facade) {
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || !context.simulationReady || context.profileId !== 'unitree' || context.simulationMode !== 'physical_mujoco') return null;
  return {
    name: 'control_unitree_g1_physical_simulation',
    title: 'Control the Unitree G1 physical MuJoCo simulation',
    description: `Control the active Unitree G1 29-DoF free-base physical workspace through ${WEBMCP_UNITREE_G1_SCHEMA_VERSION}. A joint target is a bounded request: it is clamped to the source joint range, turned into a bounded actuator torque by the Unitree low-level motor law, and MuJoCo decides the resulting motion, so requested, accepted and measured joint values are reported separately. stand() engages the one verified posture controller; it is a posture hold, not dynamic balance or perturbation recovery, and it can fail. Walking is unsupported and no walk command exists. There is no root-pose write, no root-velocity write, no upright correction, no external force, no object placement, and no hardware transport.`,
    inputSchema: createUnitreeG1ControlSchema(),
  };
}

function parse(input) {
  plain(input);
  if (input.schema_version !== WEBMCP_UNITREE_G1_SCHEMA_VERSION) invalid(`schema_version must be ${WEBMCP_UNITREE_G1_SCHEMA_VERSION}.`);
  const command = String(input.command || '');
  if (['inspect_capability', 'read_state', 'stop', 'reset'].includes(command)) {
    onlyKeys(input, ['schema_version', 'command']);
    return { command };
  }
  const advanceOf = (value, { required = false } = {}) => {
    if (value == null) {
      if (required) invalid('advance_seconds is required for this command.');
      return 0;
    }
    const seconds = Number(value);
    if (!Number.isFinite(seconds) || seconds < 0 || seconds > MAX_ADVANCE_SECONDS) invalid(`advance_seconds must be between 0 and ${MAX_ADVANCE_SECONDS}.`);
    if (required && seconds <= 0) invalid('advance_seconds must be greater than 0.');
    return seconds;
  };
  if (command === 'advance') {
    onlyKeys(input, ['schema_version', 'command', 'advance_seconds']);
    return { command, advanceSeconds: advanceOf(input.advance_seconds, { required: true }) };
  }
  if (command === 'stand') {
    onlyKeys(input, ['schema_version', 'command', 'hold_seconds']);
    const hold = input.hold_seconds == null ? 0 : Number(input.hold_seconds);
    if (!Number.isFinite(hold) || hold < 0 || hold > MAX_STAND_HOLD_SECONDS) invalid(`hold_seconds must be between 0 and ${MAX_STAND_HOLD_SECONDS}.`);
    return { command, holdSeconds: hold };
  }
  if (command !== 'set_joint_targets') invalid('Unitree G1 physical command must be inspect_capability, read_state, set_joint_targets, advance, stand, stop or reset.');
  onlyKeys(input, ['schema_version', 'command', 'targets_rad', 'advance_seconds', 'max_steps']);
  plain(input.targets_rad, 'targets_rad');
  const entries = Object.entries(input.targets_rad);
  if (!entries.length) invalid('targets_rad must contain at least one joint target.');
  const targetsRad = {};
  for (const [jointId, raw] of entries) {
    const range = JOINT_RANGES[jointId];
    if (!range) invalid(`Unknown Unitree G1 joint: ${jointId}.`);
    const value = Number(raw);
    if (!Number.isFinite(value) || value < range[0] || value > range[1]) invalid(`${jointId} must be within ${range[0]}..${range[1]} radians.`);
    targetsRad[jointId] = value;
  }
  const steps = input.max_steps == null ? DEFAULT_COMMAND_STEPS : input.max_steps;
  if (!Number.isInteger(steps) || steps < 1 || steps > MAX_COMMAND_STEPS) invalid(`max_steps must be an integer from 1 to ${MAX_COMMAND_STEPS}.`);
  return { command, targetsRad, advanceSeconds: advanceOf(input.advance_seconds), maxSteps: steps };
}

function sameAuthority(a, b) { return Boolean(a && b && a.sessionId === b.sessionId && Number(a.epoch) === Number(b.epoch) && a.sceneRevision === b.sceneRevision && a.robotId === b.robotId); }

function capture(facade, expectedEpoch) {
  facade.assertActive(expectedEpoch);
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || context.profileId !== 'unitree' || context.simulationMode !== 'physical_mujoco' || !context.simulationReady) throw new WebMcpDomainError('PROFILE_MISMATCH', 'This tool requires the active ready Unitree G1 physical workspace.', { retryable: true });
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'Finish or stop the active Python run before direct Unitree G1 control.', { retryable: true });
  const authority = facade.app.sim.getPhysicalAuthorityToken?.();
  if (!authority?.sessionId) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'Unitree G1 PhysicsSession authority is unavailable.', { retryable: true });
  return Object.freeze({ workspaceGeneration: context.workspaceGeneration, simulatorEpoch: context.simulatorEpoch, authority: Object.freeze({ ...authority }) });
}

function assertCurrent(facade, baseline, expectedEpoch, signal, { allowAuthorityChange = false } = {}) {
  if (signal?.aborted) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The Unitree G1 control call was cancelled.', { retryable: true });
  facade.assertActive(expectedEpoch);
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || context.profileId !== 'unitree' || context.simulationMode !== 'physical_mujoco' || context.workspaceGeneration !== baseline.workspaceGeneration || context.simulatorEpoch !== baseline.simulatorEpoch) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The active Unitree G1 workspace or simulator changed during control.', { retryable: true });
  if (!allowAuthorityChange && !sameAuthority(baseline.authority, facade.app.sim.getPhysicalAuthorityToken?.())) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The Unitree G1 PhysicsSession epoch changed during control.', { retryable: true });
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'A Python run acquired the simulation during direct control.', { retryable: true });
}

function observedState(facade) {
  const state = facade.app.sim.getState?.();
  if (!state) return null;
  return {
    simulationTimeSeconds: state.simulation_time_s,
    root: state.root,
    footContacts: state.foot_contacts,
    contacts: state.contacts,
    controllerMode: state.controller_mode,
    actuationEnabled: state.actuation_enabled,
    joints: state.joints,
    walking: 'unsupported',
  };
}

export async function executeUnitreeG1PhysicalControl(facade, input, signal, expectedEpoch) {
  const parsed = parse(input);
  const baseline = capture(facade, expectedEpoch);
  if (facade.activeControlId) throw new WebMcpDomainError('COMMAND_CONFLICT', 'Another bounded robot-control call is active.', { retryable: true });
  const controlId = `webmcp-unitree-g1-${expectedEpoch}-${++facade.controlSequence}`;
  facade.activeControlId = controlId;
  const base = { ok: true, profileId: 'unitree', robot: PROFILES.unitree.label, schemaVersion: WEBMCP_UNITREE_G1_SCHEMA_VERSION, hardwareValidated: false };
  try {
    assertCurrent(facade, baseline, expectedEpoch, signal);

    if (parsed.command === 'inspect_capability') {
      // Read-only. It reports the capability table verbatim and starts nothing.
      return {
        ...base,
        command: 'inspect_capability',
        modelPackage: UNITREE_G1_FREEBASE_PACKAGE.id,
        modelId: UNITREE_G1_FREEBASE_PACKAGE.modelId,
        source: structuredClone(UNITREE_G1_FREEBASE_PACKAGE.source),
        rootMode: UNITREE_G1_FREEBASE_PACKAGE.rootMode,
        controllers: [...UNITREE_G1_FREEBASE_PACKAGE.controllers],
        agentStandController: { id: AGENT_STAND_CONTROLLER, ...structuredClone(G1_STAND_CONTROLLER_PROFILES[AGENT_STAND_CONTROLLER]) },
        capabilities: UNITREE_G1_CAPABILITY_AUDIT.map((item) => ({ id: item.id, label: item.label, capability: item.capability, evidence: item.evidence, backend: item.backend, limitations: [...item.limitations] })),
        limitations: [...UNITREE_G1_FREEBASE_PACKAGE.limitations],
        physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.(),
      };
    }

    if (parsed.command === 'read_state') {
      return { ...base, command: 'read_state', observedState: observedState(facade), taskEvaluation: facade.app.sim.getTaskEvaluation?.(), physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.() };
    }

    if (parsed.command === 'reset') {
      if (!(await facade.app.resetSimulation())) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'Unitree G1 physical simulation could not be reset.', { retryable: true });
      assertCurrent(facade, baseline, expectedEpoch, signal, { allowAuthorityChange: true });
      // Reset establishes a new initial condition. It is never a recovery and never a success.
      return { ...base, command: 'reset', reset: true, note: 'reset establishes a new initial condition; it is not a recovery and is not task progress', physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.() };
    }

    if (parsed.command === 'stop') {
      await facade.app.sim.stop();
      assertCurrent(facade, baseline, expectedEpoch, signal);
      facade.app.setStatus?.('Agent commanded a bounded Unitree G1 hold at the measured joint state');
      facade.app.renderPanels?.();
      return { ...base, command: 'stop', observedState: observedState(facade), note: 'stop holds the measured joint state through the bounded controller; it does not teleport velocities to zero', physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.() };
    }

    if (parsed.command === 'advance') {
      await facade.app.sim.advanceTime(parsed.advanceSeconds);
      assertCurrent(facade, baseline, expectedEpoch, signal);
      facade.app.renderPanels?.();
      return { ...base, command: 'advance', advanceSeconds: parsed.advanceSeconds, observedState: observedState(facade), taskEvaluation: facade.app.sim.getTaskEvaluation?.(), physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.() };
    }

    if (parsed.command === 'stand') {
      const result = await facade.app.sim.engageStand({ controllerId: AGENT_STAND_CONTROLLER, holdSeconds: parsed.holdSeconds });
      assertCurrent(facade, baseline, expectedEpoch, signal);
      facade.app.setStatus?.('Agent engaged the verified Unitree G1 standing posture controller');
      facade.app.renderPanels?.();
      return {
        ...base,
        command: 'stand',
        controllerId: AGENT_STAND_CONTROLLER,
        commandStatus: result?.status ?? 'accepted',
        commandId: result?.commandId ?? null,
        holdSeconds: parsed.holdSeconds,
        observedState: observedState(facade),
        taskEvaluation: facade.app.sim.getTaskEvaluation?.(),
        claim: G1_STAND_CONTROLLER_PROFILES[AGENT_STAND_CONTROLLER].claim,
        note: 'engaging the standing controller is a command, not an achievement; read observedState and taskEvaluation for what the plant actually did',
        physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.(),
      };
    }

    const result = await facade.app.sim.applyPhysicalTargets(parsed.targetsRad, { maxSteps: parsed.maxSteps, advanceSeconds: parsed.advanceSeconds });
    assertCurrent(facade, baseline, expectedEpoch, signal);
    facade.app.setStatus?.('Agent applied bounded Unitree G1 joint targets');
    facade.app.renderPanels?.();
    return {
      ...base,
      command: 'set_joint_targets',
      commandStatus: result?.status ?? 'accepted',
      commandId: result?.commandId ?? null,
      requestedTargetsRad: structuredClone(parsed.targetsRad),
      advanceSeconds: parsed.advanceSeconds,
      observedState: observedState(facade),
      taskEvaluation: facade.app.sim.getTaskEvaluation?.(),
      note: 'requestedTargetsRad is a command, not a measurement; observedState.joints reports the accepted command and the measured position separately. Setting joint targets releases any engaged standing controller.',
      physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.(),
    };
  } catch (error) {
    if (error instanceof WebMcpDomainError) throw error;
    throw new WebMcpDomainError('SIMULATION_REJECTED', String(error?.message || error).slice(0, 280), { retryable: true, details: { profileId: 'unitree' } });
  } finally {
    if (facade.activeControlId === controlId) facade.activeControlId = null;
  }
}

export const UNITREE_G1_CONTROL_LIMITS = Object.freeze({
  maxAdvanceSeconds: MAX_ADVANCE_SECONDS,
  maxStandHoldSeconds: MAX_STAND_HOLD_SECONDS,
  maxCommandSteps: MAX_COMMAND_STEPS,
  defaultCommandSteps: DEFAULT_COMMAND_STEPS,
  agentStandController: AGENT_STAND_CONTROLLER,
  exposedCommands: Object.freeze(['inspect_capability', 'read_state', 'set_joint_targets', 'advance', 'stand', 'stop', 'reset']),
  // Named so a reviewer can grep for them: none of these exists as an agent-reachable operation.
  neverExposed: Object.freeze(['set_root_pose', 'set_root_velocity', 'force_upright', 'walk', 'set_actuation', 'apply_external_force', 'place_object']),
});
