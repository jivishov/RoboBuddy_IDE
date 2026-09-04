import { PROFILES, validateAction } from '../profiles.js';
import { WebMcpDomainError } from './agent-facade.js';

const DIRECT_CONTROL_PROFILES = Object.freeze(['so101', 'lekiwi', 'unitree']);
const DIRECT_CONTROL_SET = new Set(DIRECT_CONTROL_PROFILES);
const LEKIWI_BASE_FIELDS = Object.freeze(['x.vel', 'y.vel', 'theta.vel']);
const LEKIWI_STOP_ACTION = Object.freeze({ 'x.vel': 0, 'y.vel': 0, 'theta.vel': 0 });
const MAX_LEKIWI_DURATION_MS = 3000;
const SOURCE_ACTION_APPLY_MS = 20;

const TOOL_META = Object.freeze({
  so101: Object.freeze({
    name: 'control_so101_simulation',
    title: 'Control SO-101 browser simulation',
    description: 'Apply one partial SO-101 action using only the configured LeRobot-style joint/gripper fields and limits, or reset the active SO-101 source-plant simulation. This controls modeled browser state only; it does not access serial devices, physical hardware, files, networks, save, export, or publish surfaces.',
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
  if (!value || typeof value !== 'object' || Array.isArray(value)) {
    invalidInput(`${label} must be an object.`);
  }
}

function assertOnlyKeys(value, allowed, label = 'input') {
  for (const key of Object.keys(value)) {
    if (!allowed.includes(key)) invalidInput(`Unexpected ${label} field: ${key}.`);
  }
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

export function createProfileControlSchema(profileId) {
  if (!DIRECT_CONTROL_SET.has(profileId)) throw new Error(`Unsupported direct-control profile: ${profileId}`);
  const setActionProperties = {
    command: { type: 'string', const: 'set_action' },
    action: profileActionSchema(profileId),
  };
  if (profileId === 'lekiwi') {
    setActionProperties.duration_ms = {
      type: 'integer',
      minimum: 20,
      maximum: MAX_LEKIWI_DURATION_MS,
      description: 'Required for any nonzero x.vel, y.vel, or theta.vel. The base is stopped automatically after this modeled duration.',
    };
  }
  return {
    type: 'object',
    oneOf: [
      {
        type: 'object',
        properties: setActionProperties,
        required: ['command', 'action'],
        additionalProperties: false,
      },
      {
        type: 'object',
        properties: { command: { type: 'string', const: 'reset' } },
        required: ['command'],
        additionalProperties: false,
      },
    ],
  };
}

export function getProfileControlDefinition(facade) {
  const context = facade.getRegistrationContext();
  const profileId = context.profileId;
  if (context.workspaceStatus !== 'ready' || !context.simulationReady || !DIRECT_CONTROL_SET.has(profileId)) return null;
  return {
    profileId,
    ...TOOL_META[profileId],
    inputSchema: createProfileControlSchema(profileId),
  };
}

function parseInput(profileId, input) {
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
  try {
    action = validateAction(profileId, input.action);
  } catch (error) {
    invalidInput(error.message);
  }

  if (profileId !== 'lekiwi') return { command: 'set_action', action };

  const movingBase = LEKIWI_BASE_FIELDS.some((key) => Math.abs(Number(action[key] ?? 0)) > 1e-12);
  if (movingBase) {
    if (!Number.isInteger(input.duration_ms) || input.duration_ms < 20 || input.duration_ms > MAX_LEKIWI_DURATION_MS) {
      invalidInput(`Nonzero LeKiwi base velocity requires duration_ms between 20 and ${MAX_LEKIWI_DURATION_MS}.`);
    }
  } else if (input.duration_ms !== undefined) {
    invalidInput('duration_ms is only accepted when the LeKiwi action includes a nonzero base velocity.');
  }
  return { command: 'set_action', action, durationMs: movingBase ? input.duration_ms : null, movingBase };
}

function captureControlContext(facade, profileId, expectedEpoch) {
  facade.assertActive(expectedEpoch);
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready') {
    throw new WebMcpDomainError('SIMULATION_NOT_READY', 'The RoboBuddy workspace is not ready.', { retryable: true });
  }
  if (context.profileId !== profileId || !DIRECT_CONTROL_SET.has(context.profileId)) {
    throw new WebMcpDomainError('PROFILE_MISMATCH', `This tool requires the active ${PROFILES[profileId].shortLabel} workspace.`);
  }
  if (!context.simulationReady) {
    throw new WebMcpDomainError('SIMULATION_NOT_READY', `${PROFILES[profileId].shortLabel} browser simulation is not ready.`, { retryable: true });
  }
  if (facade.app.getExecutionState() !== 'idle') {
    throw new WebMcpDomainError('SIMULATION_BUSY', 'Stop or finish the active Python run before direct WebMCP simulation control.', { retryable: true });
  }
  return Object.freeze({
    profileId,
    workspaceGeneration: context.workspaceGeneration,
    simulatorEpoch: context.simulatorEpoch,
  });
}

function assertControlCurrent(facade, baseline, expectedEpoch, signal) {
  if (signal?.aborted) {
    throw new WebMcpDomainError('OPERATION_CANCELLED', 'The WebMCP robot-control call was cancelled.', { retryable: true });
  }
  facade.assertActive(expectedEpoch);
  const current = facade.getRegistrationContext();
  if (current.workspaceStatus !== 'ready'
    || !current.simulationReady
    || current.profileId !== baseline.profileId
    || current.workspaceGeneration !== baseline.workspaceGeneration
    || current.simulatorEpoch !== baseline.simulatorEpoch) {
    throw new WebMcpDomainError('OPERATION_CANCELLED', 'The active robot, workspace, or simulator backend changed during the WebMCP control call.', { retryable: true });
  }
  if (facade.app.getExecutionState() !== 'idle') {
    throw new WebMcpDomainError('SIMULATION_BUSY', 'A Python run acquired the simulation during WebMCP control.', { retryable: true });
  }
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
  } catch {
    return false;
  }
}

async function autoStopLeKiwi(facade, baseline, expectedEpoch) {
  if (!canSafelyStopLeKiwi(facade, baseline, expectedEpoch)) return false;
  try {
    const stopped = await facade.app.sim.applyAction(LEKIWI_STOP_ACTION);
    return stopped !== false;
  } catch {
    return false;
  }
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

export async function executeProfileControl(facade, profileId, input, signal, expectedEpoch) {
  if (!DIRECT_CONTROL_SET.has(profileId)) {
    throw new WebMcpDomainError('PROFILE_MISMATCH', 'This direct-control tool is not available for the requested profile.');
  }
  const parsed = parseInput(profileId, input);
  const baseline = captureControlContext(facade, profileId, expectedEpoch);
  if (facade.activeControlId) {
    throw new WebMcpDomainError('COMMAND_CONFLICT', 'Another bounded WebMCP robot-control call is still active.', { retryable: true });
  }
  const controlId = `webmcp-${profileId}-${expectedEpoch}-${++facade.controlSequence}`;
  facade.activeControlId = controlId;
  const beforeTick = () => {
    assertControlCurrent(facade, baseline, expectedEpoch, signal);
    return true;
  };
  let lekiwiBaseStarted = false;

  try {
    assertControlCurrent(facade, baseline, expectedEpoch, signal);
    if (parsed.command === 'reset') {
      const reset = await facade.app.resetSimulation();
      if (!reset) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'The browser simulation could not be reset.', { retryable: true });
      assertControlCurrent(facade, baseline, expectedEpoch, signal);
      facade.app.setStatus?.(`Agent reset ${PROFILES[profileId].shortLabel} browser simulation`);
      facade.app.renderPanels?.();
      return simulationResult(facade, profileId, 'reset', { reset: true });
    }

    const applied = await facade.app.sim.applyAction(parsed.action, { beforeTick });
    if (applied === false) {
      throw new WebMcpDomainError('OPERATION_CANCELLED', 'The browser simulation stopped before the action completed.', { retryable: true });
    }
    lekiwiBaseStarted = profileId === 'lekiwi' && parsed.movingBase;

    if (lekiwiBaseStarted) {
      const remainingSeconds = Math.max(0, parsed.durationMs - SOURCE_ACTION_APPLY_MS) / 1000;
      if (remainingSeconds > 0) {
        const advanced = await facade.app.sim.advanceTime(remainingSeconds, { realtime: true, beforeTick });
        if (advanced === false) {
          throw new WebMcpDomainError('OPERATION_CANCELLED', 'The LeKiwi modeled base motion stopped before its bounded duration completed.', { retryable: true });
        }
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
