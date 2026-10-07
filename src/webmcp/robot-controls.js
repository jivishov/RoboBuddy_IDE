import { PROFILES, validateAction } from '../profiles.js';
import { SO101_MANIPULATION_MODEL_PACKAGE } from '../physics/model-packages.js';
import { WebMcpDomainError } from './agent-facade.js';

const DIRECT_CONTROL_PROFILES = Object.freeze(['so101', 'lekiwi', 'unitree']);
const DIRECT_CONTROL_SET = new Set(DIRECT_CONTROL_PROFILES);
const LEKIWI_BASE_FIELDS = Object.freeze(['x.vel', 'y.vel', 'theta.vel']);
const LEKIWI_STOP_ACTION = Object.freeze({ 'x.vel': 0, 'y.vel': 0, 'theta.vel': 0 });
const MAX_LEKIWI_DURATION_MS = 3000;
const SOURCE_ACTION_APPLY_MS = 20;
const SO101_PHYSICAL_WEBMCP_VERSION = 'robobuddy.so101.physical.v1';
const MAX_SO101_ADVANCE_SECONDS = 2;
const DEFAULT_SO101_COMMAND_STEPS = 1000;
const MAX_SO101_COMMAND_STEPS = 5000;

const SO101_PHYSICAL_RANGES = Object.freeze(Object.fromEntries(
  SO101_MANIPULATION_MODEL_PACKAGE.joints.map((joint) => {
    const actuator = SO101_MANIPULATION_MODEL_PACKAGE.actuators.find((item) => item.jointId === joint.id);
    if (!actuator) throw new Error(`SO-101 physical WebMCP has no actuator for ${joint.id}`);
    const minimum = Math.max(Number(joint.rangeRad[0]), Number(actuator.controlRangeRad[0]));
    const maximum = Math.min(Number(joint.rangeRad[1]), Number(actuator.controlRangeRad[1]));
    if (!Number.isFinite(minimum) || !Number.isFinite(maximum) || minimum >= maximum) throw new Error(`SO-101 physical WebMCP has no valid control intersection for ${joint.id}`);
    return [joint.id, Object.freeze([minimum, maximum])];
  }),
));

const TOOL_META = Object.freeze({
  so101: Object.freeze({
    name: 'control_so101_simulation',
    title: 'Control SO-101 physical MuJoCo simulation',
    description: `Control the active SO-101 rigid-body benchmark through ${SO101_PHYSICAL_WEBMCP_VERSION}. Joint targets are explicit radians and route to the same authoritative PhysicsSession used by live Python, rendering, and task evaluation. Command acceptance is not target achievement; returned state is observed MuJoCo state. No hardware, object-transform, snap, weld, save, export, or publish control is exposed.`,
  }),
  lekiwi: Object.freeze({
    name: 'control_lekiwi_simulation',
    title: 'Control LeKiwi browser simulation',
    description: 'Apply one partial LeKiwi arm/base action using only configured action fields and limits, or reset the active LeKiwi source-plant simulation. Any nonzero base velocity requires a bounded duration and is automatically stopped before the tool returns. This never accesses ZMQ, physical hardware, files, networks, save, export, or publish surfaces.',
  }),
  unitree: Object.freeze({
    name: 'control_unitree_g1_simulation',
    title: 'Pose Unitree G1 browser simulation',
    description: 'Apply one partial pose using only the configured Unitree G1 named-joint limits, or reset to the active neutral kinematic workspace. This is browser-only pose visualization: no gait, balance, contact, Unitree SDK, network, hardware, save, export, or publish control is exposed.',
  }),
});

function invalidInput(message) {
  throw new WebMcpDomainError('INVALID_ARGUMENT', message, { retryable: false });
}

function assertPlainObject(value, label = 'input') {
  if (!value || typeof value !== 'object' || Array.isArray(value)) invalidInput(`${label} must be an object.`);
}

function assertOnlyKeys(value, allowed, label = 'input') {
  for (const key of Object.keys(value)) if (!allowed.includes(key)) invalidInput(`Unexpected ${label} field: ${key}.`);
}

function profileActionSchema(profileId) {
  const profile = PROFILES[profileId];
  return {
    type: 'object',
    properties: Object.fromEntries(Object.entries(profile.limits).map(([key, range]) => [key, {
      type: 'number',
      minimum: Number(range[0]),
      maximum: Number(range[1]),
      description: `${profile.units?.[key] || 'deg'}; configured browser-simulation envelope`,
    }])),
    minProperties: 1,
    additionalProperties: false,
  };
}

function so101PhysicalTargetsSchema() {
  return {
    type: 'object',
    properties: Object.fromEntries(Object.entries(SO101_PHYSICAL_RANGES).map(([jointId, range]) => [jointId, {
      type: 'number',
      minimum: Number(range[0]),
      maximum: Number(range[1]),
      description: 'radians; physical MuJoCo joint target',
    }])),
    minProperties: 1,
    additionalProperties: false,
  };
}

function so101PhysicalControlSchema() {
  return {
    type: 'object',
    oneOf: [
      {
        type: 'object',
        properties: {
          schema_version: { type: 'string', const: SO101_PHYSICAL_WEBMCP_VERSION },
          command: { type: 'string', const: 'set_joint_targets' },
          targets_rad: so101PhysicalTargetsSchema(),
          advance_seconds: {
            type: 'number', minimum: 0, maximum: MAX_SO101_ADVANCE_SECONDS,
            description: 'Optional bounded simulation-time advancement after latching the target. Must align to the active MuJoCo timestep.',
          },
          max_steps: {
            type: 'integer', minimum: 1, maximum: MAX_SO101_COMMAND_STEPS,
            description: 'Hard maximum physics-step budget for the latched command.',
          },
        },
        required: ['schema_version', 'command', 'targets_rad'],
        additionalProperties: false,
      },
      {
        type: 'object',
        properties: {
          schema_version: { type: 'string', const: SO101_PHYSICAL_WEBMCP_VERSION },
          command: { type: 'string', const: 'reset' },
        },
        required: ['schema_version', 'command'],
        additionalProperties: false,
      },
    ],
  };
}

export function createProfileControlSchema(profileId) {
  if (!DIRECT_CONTROL_SET.has(profileId)) throw new Error(`Unsupported direct-control profile: ${profileId}`);
  if (profileId === 'so101') return so101PhysicalControlSchema();
  const setActionProperties = {
    command: { type: 'string', const: 'set_action' },
    action: profileActionSchema(profileId),
  };
  if (profileId === 'lekiwi') {
    setActionProperties.duration_ms = {
      type: 'integer', minimum: 20, maximum: MAX_LEKIWI_DURATION_MS,
      description: 'Required for any nonzero x.vel, y.vel, or theta.vel. The base is stopped automatically after this modeled duration.',
    };
  }
  return {
    type: 'object',
    oneOf: [
      { type: 'object', properties: setActionProperties, required: ['command', 'action'], additionalProperties: false },
      { type: 'object', properties: { command: { type: 'string', const: 'reset' } }, required: ['command'], additionalProperties: false },
    ],
  };
}

export function getProfileControlDefinition(facade) {
  const context = facade.getRegistrationContext();
  const profileId = context.profileId;
  if (context.workspaceStatus !== 'ready' || !context.simulationReady || !DIRECT_CONTROL_SET.has(profileId)) return null;
  if (profileId === 'so101' && context.simulationMode !== 'physical_mujoco') return null;
  // The physical LeKiwi workspace publishes control_lekiwi_simulation from
  // lekiwi-physical-control.js under a versioned physical schema; the legacy source-plant tool
  // must never shadow it with the same name.
  if (profileId === 'lekiwi' && context.simulationMode === 'physical_mujoco') return null;
  // The Unitree G1 physical workspace is a separate workspace with its own model package, its own
  // controller identity and its own versioned tool. This pose tool writes joint angles straight
  // into the canonical rig, so it must never be reachable while a physical plant is displayed:
  // a pose write is not a physical command and must never be offered as one.
  if (profileId === 'unitree' && context.simulationMode === 'physical_mujoco') return null;
  return { profileId, ...TOOL_META[profileId], inputSchema: createProfileControlSchema(profileId) };
}

function parseSo101PhysicalInput(input) {
  assertPlainObject(input);
  if (input.schema_version !== SO101_PHYSICAL_WEBMCP_VERSION) invalidInput(`schema_version must be ${SO101_PHYSICAL_WEBMCP_VERSION}.`);
  if (input.command === 'reset') {
    assertOnlyKeys(input, ['schema_version', 'command']);
    return { command: 'reset', schemaVersion: SO101_PHYSICAL_WEBMCP_VERSION };
  }
  if (input.command !== 'set_joint_targets') invalidInput('physical SO-101 command must be set_joint_targets or reset.');
  assertOnlyKeys(input, ['schema_version', 'command', 'targets_rad', 'advance_seconds', 'max_steps']);
  assertPlainObject(input.targets_rad, 'targets_rad');
  const entries = Object.entries(input.targets_rad);
  if (!entries.length) invalidInput('targets_rad must contain at least one physical joint target.');
  const targetsRad = {};
  for (const [jointId, raw] of entries) {
    const range = SO101_PHYSICAL_RANGES[jointId];
    if (!range) invalidInput(`Unknown physical SO-101 joint: ${jointId}.`);
    const value = Number(raw);
    if (!Number.isFinite(value)) invalidInput(`${jointId} must be a finite radian value.`);
    if (value < range[0] || value > range[1]) invalidInput(`${jointId}=${value} rad is outside ${range[0]}..${range[1]} rad.`);
    targetsRad[jointId] = value;
  }
  const advanceSeconds = input.advance_seconds == null ? 0 : Number(input.advance_seconds);
  if (!Number.isFinite(advanceSeconds) || advanceSeconds < 0 || advanceSeconds > MAX_SO101_ADVANCE_SECONDS) invalidInput(`advance_seconds must be between 0 and ${MAX_SO101_ADVANCE_SECONDS}.`);
  const maxSteps = input.max_steps == null ? DEFAULT_SO101_COMMAND_STEPS : input.max_steps;
  if (!Number.isInteger(maxSteps) || maxSteps < 1 || maxSteps > MAX_SO101_COMMAND_STEPS) invalidInput(`max_steps must be an integer from 1 to ${MAX_SO101_COMMAND_STEPS}.`);
  return { command: 'set_joint_targets', schemaVersion: SO101_PHYSICAL_WEBMCP_VERSION, targetsRad, advanceSeconds, maxSteps };
}

function parseInput(profileId, input) {
  if (profileId === 'so101') return parseSo101PhysicalInput(input);
  assertPlainObject(input);
  if (input.command === 'reset') {
    assertOnlyKeys(input, ['command']);
    return { command: 'reset' };
  }
  if (input.command !== 'set_action') invalidInput('command must be set_action or reset.');
  const topLevelKeys = profileId === 'lekiwi' ? ['command', 'action', 'duration_ms'] : ['command', 'action'];
  assertOnlyKeys(input, topLevelKeys);
  assertPlainObject(input.action, 'action');
  let action;
  try { action = validateAction(profileId, input.action); }
  catch (error) { invalidInput(error.message); }

  if (profileId !== 'lekiwi') return { command: 'set_action', action };
  const movingBase = LEKIWI_BASE_FIELDS.some((key) => Math.abs(Number(action[key] ?? 0)) > 1e-12);
  if (movingBase) {
    if (!Number.isInteger(input.duration_ms) || input.duration_ms < 20 || input.duration_ms > MAX_LEKIWI_DURATION_MS) invalidInput(`Nonzero LeKiwi base velocity requires duration_ms between 20 and ${MAX_LEKIWI_DURATION_MS}.`);
  } else if (input.duration_ms !== undefined) invalidInput('duration_ms is only accepted when the LeKiwi action includes a nonzero base velocity.');
  return { command: 'set_action', action, durationMs: movingBase ? input.duration_ms : null, movingBase };
}

function captureControlContext(facade, profileId, expectedEpoch) {
  facade.assertActive(expectedEpoch);
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready') throw new WebMcpDomainError('SIMULATION_NOT_READY', 'The RoboBuddy workspace is not ready.', { retryable: true });
  if (context.profileId !== profileId || !DIRECT_CONTROL_SET.has(context.profileId)) throw new WebMcpDomainError('PROFILE_MISMATCH', `This tool requires the active ${PROFILES[profileId].shortLabel} workspace.`);
  if (!context.simulationReady) throw new WebMcpDomainError('SIMULATION_NOT_READY', `${PROFILES[profileId].shortLabel} browser simulation is not ready.`, { retryable: true });
  if (profileId === 'so101' && context.simulationMode !== 'physical_mujoco') throw new WebMcpDomainError('PROFILE_MISMATCH', 'SO-101 direct control requires the physical MuJoCo workspace.');
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'Stop or finish the active Python run before direct WebMCP simulation control.', { retryable: true });
  const physicalAuthority = profileId === 'so101' ? facade.app.sim.getPhysicalAuthorityToken?.() : null;
  if (profileId === 'so101' && !physicalAuthority?.sessionId) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'SO-101 physical authority is unavailable.', { retryable: true });
  return Object.freeze({
    profileId,
    workspaceGeneration: context.workspaceGeneration,
    simulatorEpoch: context.simulatorEpoch,
    physicalAuthority: physicalAuthority ? Object.freeze({ ...physicalAuthority }) : null,
  });
}

function samePhysicalAuthority(a, b) {
  return Boolean(a && b
    && a.sessionId === b.sessionId
    && Number(a.epoch) === Number(b.epoch)
    && a.sceneRevision === b.sceneRevision
    && a.robotId === b.robotId);
}

function assertControlCurrent(facade, baseline, expectedEpoch, signal, { allowPhysicalAuthorityChange = false } = {}) {
  if (signal?.aborted) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The WebMCP robot-control call was cancelled.', { retryable: true });
  facade.assertActive(expectedEpoch);
  const current = facade.getRegistrationContext();
  if (current.workspaceStatus !== 'ready'
    || !current.simulationReady
    || current.profileId !== baseline.profileId
    || current.workspaceGeneration !== baseline.workspaceGeneration
    || current.simulatorEpoch !== baseline.simulatorEpoch) {
    throw new WebMcpDomainError('OPERATION_CANCELLED', 'The active robot, workspace, or simulator backend changed during the WebMCP control call.', { retryable: true });
  }
  if (baseline.profileId === 'so101' && !allowPhysicalAuthorityChange) {
    const authority = facade.app.sim.getPhysicalAuthorityToken?.();
    if (!samePhysicalAuthority(baseline.physicalAuthority, authority)) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The SO-101 PhysicsSession epoch changed during this WebMCP call.', { retryable: true });
  }
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'A Python run acquired the simulation during WebMCP control.', { retryable: true });
  return current;
}

function canSafelyStopLeKiwi(facade, baseline, expectedEpoch) {
  try {
    facade.assertActive(expectedEpoch);
    const current = facade.getRegistrationContext();
    return current.workspaceStatus === 'ready'
      && current.simulationReady
      && current.profileId === 'lekiwi'
      && current.workspaceGeneration === baseline.workspaceGeneration
      && current.simulatorEpoch === baseline.simulatorEpoch
      && facade.app.getExecutionState() === 'idle';
  } catch { return false; }
}

async function autoStopLeKiwi(facade, baseline, expectedEpoch) {
  if (!canSafelyStopLeKiwi(facade, baseline, expectedEpoch)) return false;
  try {
    const stopped = await facade.app.sim.applyAction(LEKIWI_STOP_ACTION);
    return stopped !== false;
  } catch { return false; }
}

function simulationResult(facade, profileId, command, extras = {}) {
  const snapshot = facade.app.getAgentSnapshot();
  return {
    ok: true,
    profileId,
    robot: PROFILES[profileId].label,
    command,
    hardwareValidated: false,
    ...extras,
    simulation: snapshot.workspaceStatus === 'ready' ? facade.inspectSimulation(snapshot) : { workspaceStatus: snapshot.workspaceStatus },
  };
}

function observedSo101State(observation) {
  return {
    simulationTimeSeconds: Number(observation?.simulationTimeSeconds ?? 0),
    jointsRad: Object.fromEntries(Object.entries(observation?.joints || {}).map(([jointId, state]) => [jointId, Number(state.positionRad)])),
    blockPositionM: Array.isArray(observation?.bodies?.benchmark_block?.positionM) ? [...observation.bodies.benchmark_block.positionM] : null,
    contactCount: Number(observation?.contactCount || 0),
  };
}

export async function executeProfileControl(facade, profileId, input, signal, expectedEpoch) {
  if (!DIRECT_CONTROL_SET.has(profileId)) throw new WebMcpDomainError('PROFILE_MISMATCH', 'This direct-control tool is not available for the requested profile.');
  const parsed = parseInput(profileId, input);
  const baseline = captureControlContext(facade, profileId, expectedEpoch);
  if (facade.activeControlId) throw new WebMcpDomainError('COMMAND_CONFLICT', 'Another bounded WebMCP robot-control call is still active.', { retryable: true });
  const controlId = `webmcp-${profileId}-${expectedEpoch}-${++facade.controlSequence}`;
  facade.activeControlId = controlId;
  const beforeTick = () => { assertControlCurrent(facade, baseline, expectedEpoch, signal); return true; };
  let lekiwiBaseStarted = false;

  try {
    assertControlCurrent(facade, baseline, expectedEpoch, signal);
    if (parsed.command === 'reset') {
      const reset = await facade.app.resetSimulation();
      if (!reset) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'The browser simulation could not be reset.', { retryable: true });
      assertControlCurrent(facade, baseline, expectedEpoch, signal, { allowPhysicalAuthorityChange: profileId === 'so101' });
      facade.app.setStatus?.(`Agent reset ${PROFILES[profileId].shortLabel} browser simulation`);
      facade.app.renderPanels?.();
      return simulationResult(facade, profileId, 'reset', {
        reset: true,
        schemaVersion: profileId === 'so101' ? SO101_PHYSICAL_WEBMCP_VERSION : undefined,
        physicalAuthority: profileId === 'so101' ? facade.app.sim.getPhysicalAuthorityToken?.() : undefined,
      });
    }

    if (profileId === 'so101') {
      const result = await facade.app.sim.applyPhysicalTargets(parsed.targetsRad, {
        maxSteps: parsed.maxSteps,
        advanceSeconds: parsed.advanceSeconds,
      });
      assertControlCurrent(facade, baseline, expectedEpoch, signal);
      facade.app.setStatus?.('Agent applied bounded SO-101 physical MuJoCo target');
      facade.app.renderPanels?.();
      return simulationResult(facade, profileId, parsed.command, {
        schemaVersion: SO101_PHYSICAL_WEBMCP_VERSION,
        commandStatus: result.status,
        commandId: result.commandId,
        acceptedTargetsRad: result.acceptedTargetsRad,
        advanceSeconds: parsed.advanceSeconds,
        observedState: observedSo101State(result.observation),
        taskEvaluation: result.taskEvaluation,
        physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.(),
      });
    }

    const applied = await facade.app.sim.applyAction(parsed.action, { beforeTick });
    if (applied === false) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The browser simulation stopped before the action completed.', { retryable: true });
    lekiwiBaseStarted = profileId === 'lekiwi' && parsed.movingBase;

    if (lekiwiBaseStarted) {
      const remainingSeconds = Math.max(0, parsed.durationMs - SOURCE_ACTION_APPLY_MS) / 1000;
      if (remainingSeconds > 0) {
        const advanced = await facade.app.sim.advanceTime(remainingSeconds, { realtime: true, beforeTick });
        if (advanced === false) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The LeKiwi modeled base motion stopped before its bounded duration completed.', { retryable: true });
      }
      await autoStopLeKiwi(facade, baseline, expectedEpoch);
      lekiwiBaseStarted = false;
    }

    assertControlCurrent(facade, baseline, expectedEpoch, signal);
    facade.app.setStatus?.(`Agent applied bounded ${PROFILES[profileId].shortLabel} browser-simulation action`);
    facade.app.renderPanels?.();
    return simulationResult(facade, profileId, 'set_action', {
      appliedAction: parsed.action,
      durationMs: parsed.durationMs,
      baseAutoStopped: profileId === 'lekiwi' ? Boolean(parsed.movingBase) : undefined,
    });
  } catch (error) {
    if (error instanceof WebMcpDomainError) throw error;
    throw new WebMcpDomainError(
      'SIMULATION_REJECTED',
      String(error?.message || 'The browser simulation rejected the bounded robot action.').slice(0, 280),
      { retryable: true, details: { profileId } },
    );
  } finally {
    if (lekiwiBaseStarted) await autoStopLeKiwi(facade, baseline, expectedEpoch);
    if (facade.activeControlId === controlId) facade.activeControlId = null;
  }
}

export const WEBMCP_DIRECT_CONTROL_PROFILES = DIRECT_CONTROL_PROFILES;
export const WEBMCP_SO101_PHYSICAL_SCHEMA_VERSION = SO101_PHYSICAL_WEBMCP_VERSION;
