import { PROFILES } from '../profiles.js';
import { MICRODUCK_CAPABILITY_AUDIT, MICRODUCK_CAPABILITY_STATUS, MICRODUCK_PHYSICAL_SKILL_IDS, microduckCapability } from '../physics/microduck-capabilities.js';
import { MICRODUCK_COMMAND_LIMITS, MICRODUCK_GAIT_ONSET_MS } from '../physics/microduck-scene.js';
import { MICRODUCK_MAX_ADVANCE_SECONDS } from '../physics/microduck-physical-simulator.js';
import { MICRODUCK_CONTROL_INTERVAL_SECONDS } from '../physics/microduck-controller.js';
import { WebMcpDomainError } from './agent-facade.js';

// The physical MicroDuck tool is a NEW name, not a re-skin of the legacy demonstrator's
// `control_microduck_simulation`. The semantics genuinely changed - a velocity is now a
// bounded request into a trained policy and a MuJoCo plant, not a root-motion instruction -
// and the legacy tool keeps its own name and its own truthful legacy labelling. Preserving
// the old name here would have put a physical badge on a synthetic command path.
export const WEBMCP_MICRODUCK_PHYSICAL_SCHEMA_VERSION = 'robobuddy.microduck.physical.v1';

const L = MICRODUCK_COMMAND_LIMITS;

// Only capabilities the physical package actually carries are commandable. Roller-mode
// locomotion and roller crouch are absent by construction, and are reported as unsupported
// rather than routed to the legacy demonstrator.
const PHYSICAL_SKILLS = MICRODUCK_PHYSICAL_SKILL_IDS;
const UNSUPPORTED_SKILLS = Object.freeze(
  MICRODUCK_CAPABILITY_AUDIT.filter((item) => !item.physicalPolicy).map((item) => item.id),
);
const DECLARED_PERTURBATIONS = Object.freeze(['face_down', 'face_up', 'on_side', 'upright']);

function invalid(message) { throw new WebMcpDomainError('INVALID_ARGUMENT', message, { retryable: false }); }
function plain(value, label = 'input') { if (!value || typeof value !== 'object' || Array.isArray(value)) invalid(`${label} must be an object.`); }
function onlyKeys(value, allowed, label = 'input') { for (const key of Object.keys(value)) if (!allowed.includes(key)) invalid(`Unexpected ${label} field: ${key}.`); }

export function createMicroDuckPhysicalControlSchema() {
  const number = (range, description) => ({ type: 'number', minimum: range[0], maximum: range[1], description });
  const command = {
    type: 'object',
    properties: {
      vx: number(L.vxMS, 'requested forward velocity, m/s; the policy decides the gait'),
      vy: number(L.vyMS, 'requested left velocity, m/s'),
      vyaw: number(L.vyawRadS, 'requested yaw rate, rad/s'),
      neckPitch: number(L.neckPitchRad, 'neck pitch command, rad'),
      headPitch: number(L.headPitchRad, 'head pitch command, rad'),
      headYaw: number(L.headYawRad, 'head yaw command, rad'),
      headRoll: number(L.headRollRad, 'head roll command, rad'),
      bodyZ: number(L.bodyZM, 'standing body height offset, m'),
      bodyRoll: number(L.bodyRollRad, 'standing body roll, rad'),
      bodyPitch: number(L.bodyPitchRad, 'standing body pitch, rad'),
    },
    minProperties: 1,
    additionalProperties: false,
  };
  const advance = { type: 'number', minimum: 0, maximum: MICRODUCK_MAX_ADVANCE_SECONDS, description: `bounded simulation-time advancement, seconds; the controller runs at ${MICRODUCK_CONTROL_INTERVAL_SECONDS * 1000} ms per tick` };
  const version = { type: 'string', const: WEBMCP_MICRODUCK_PHYSICAL_SCHEMA_VERSION };
  return {
    type: 'object',
    // Root closure sees only sibling properties, not properties inside oneOf.
    // Each command branch below still rejects fields from all other branches.
    properties: { schema_version: version, command: { type: 'string' }, request: {}, skill: {}, advance_seconds: {}, perturbation: {}, settle_seconds: {} },
    oneOf: [
      {
        type: 'object',
        properties: { schema_version: version, command: { type: 'string', const: 'set_command' }, request: command, advance_seconds: advance },
        required: ['schema_version', 'command', 'request'],
        additionalProperties: false,
      },
      {
        type: 'object',
        properties: {
          schema_version: version,
          command: { type: 'string', const: 'request_skill' },
          skill: { type: 'string', enum: [...PHYSICAL_SKILLS, ...UNSUPPORTED_SKILLS], description: 'an explicit controller command (sit or stand_up for sit/stand); unsupported roller ids are rejected without a legacy fallback' },
          advance_seconds: advance,
        },
        required: ['schema_version', 'command', 'skill'],
        additionalProperties: false,
      },
      {
        type: 'object',
        properties: { schema_version: version, command: { type: 'string', const: 'advance' }, advance_seconds: advance },
        required: ['schema_version', 'command', 'advance_seconds'],
        additionalProperties: false,
      },
      {
        type: 'object',
        properties: {
          schema_version: version,
          command: { type: 'string', const: 'setup_perturbation' },
          perturbation: { type: 'string', enum: [...DECLARED_PERTURBATIONS] },
          settle_seconds: { type: 'number', minimum: 0, maximum: MICRODUCK_MAX_ADVANCE_SECONDS, description: 'let the declared perturbation settle with the servos holding the home pose' },
        },
        required: ['schema_version', 'command', 'perturbation'],
        additionalProperties: false,
      },
      {
        type: 'object',
        properties: { schema_version: version, command: { type: 'string', const: 'stop' } },
        required: ['schema_version', 'command'],
        additionalProperties: false,
      },
      {
        type: 'object',
        properties: { schema_version: version, command: { type: 'string', const: 'reset' } },
        required: ['schema_version', 'command'],
        additionalProperties: false,
      },
    ],
    additionalProperties: false,
  };
}

export function getMicroDuckPhysicalControlDefinition(facade) {
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || !context.simulationReady || context.profileId !== 'microduck' || context.simulationMode !== 'physical_mujoco') return null;
  return {
    name: 'control_microduck_physical_simulation',
    title: 'Control the MicroDuck physical MuJoCo workspace',
    description: `Control the active MicroDuck alpha physical workspace through ${WEBMCP_MICRODUCK_PHYSICAL_SCHEMA_VERSION}. A velocity is a bounded REQUEST: it is encoded into the pinned 61-value observation, a deployed ONNX policy decides the joint targets, and MuJoCo decides what the robot actually does from foot-floor contact. Below about ${MICRODUCK_GAIT_ONSET_MS} m/s the matched policy holds a stand rather than initiating a gait. Every call routes to the same PhysicsSession used by live Python, the renderer and the task evaluator. Unsupported capabilities (${UNSUPPORTED_SKILLS.join(', ')}) are refused, never routed to the legacy demonstrator. No root-pose write, root-velocity write, ball-velocity write, kick impulse, boundary clamp, reset-as-recovery, task-success write, hardware transport, save, export or publish capability is exposed.`,
    inputSchema: createMicroDuckPhysicalControlSchema(),
  };
}

function parse(input) {
  plain(input);
  if (input.schema_version !== WEBMCP_MICRODUCK_PHYSICAL_SCHEMA_VERSION) invalid(`schema_version must be ${WEBMCP_MICRODUCK_PHYSICAL_SCHEMA_VERSION}.`);
  const advanceOf = (value, required = false) => {
    if (value === undefined) {
      if (required) invalid('advance_seconds is required for this command.');
      return 0;
    }
    const seconds = value;
    if (typeof seconds !== 'number' || !Number.isFinite(seconds) || seconds < 0 || seconds > MICRODUCK_MAX_ADVANCE_SECONDS) invalid(`advance_seconds must be between 0 and ${MICRODUCK_MAX_ADVANCE_SECONDS}.`);
    return seconds;
  };
  if (input.command === 'reset' || input.command === 'stop') {
    onlyKeys(input, ['schema_version', 'command']);
    return { command: input.command };
  }
  if (input.command === 'advance') {
    onlyKeys(input, ['schema_version', 'command', 'advance_seconds']);
    return { command: 'advance', advanceSeconds: advanceOf(input.advance_seconds, true) };
  }
  if (input.command === 'setup_perturbation') {
    onlyKeys(input, ['schema_version', 'command', 'perturbation', 'settle_seconds']);
    if (!DECLARED_PERTURBATIONS.includes(input.perturbation)) invalid(`perturbation must be one of ${DECLARED_PERTURBATIONS.join(', ')}.`);
    return { command: 'setup_perturbation', perturbation: input.perturbation, settleSeconds: advanceOf(input.settle_seconds) };
  }
  if (input.command === 'request_skill') {
    onlyKeys(input, ['schema_version', 'command', 'skill', 'advance_seconds']);
    if (![...PHYSICAL_SKILLS, ...UNSUPPORTED_SKILLS].includes(input.skill)) invalid('skill must be a supported command id; use sit or stand_up for sit/stand.');
    return { command: 'request_skill', skill: input.skill, advanceSeconds: advanceOf(input.advance_seconds) };
  }
  if (input.command !== 'set_command') invalid('MicroDuck physical command must be set_command, request_skill, advance, setup_perturbation, stop or reset.');
  onlyKeys(input, ['schema_version', 'command', 'request', 'advance_seconds']);
  plain(input.request, 'request');
  const allowed = new Set(['vx', 'vy', 'vyaw', 'neckPitch', 'headPitch', 'headYaw', 'headRoll', 'bodyZ', 'bodyRoll', 'bodyPitch']);
  const request = {};
  for (const [key, raw] of Object.entries(input.request)) {
    if (!allowed.has(key)) invalid(`Unknown MicroDuck command field: ${key}.`);
    const value = raw;
    if (typeof value !== 'number' || !Number.isFinite(value)) invalid(`${key} must be a finite number.`);
    const ranges = { vx: L.vxMS, vy: L.vyMS, vyaw: L.vyawRadS, neckPitch: L.neckPitchRad, headPitch: L.headPitchRad, headYaw: L.headYawRad, headRoll: L.headRollRad, bodyZ: L.bodyZM, bodyRoll: L.bodyRollRad, bodyPitch: L.bodyPitchRad };
    const [minimum, maximum] = ranges[key];
    if (value < minimum || value > maximum) invalid(`${key} must be between ${minimum} and ${maximum}.`);
    request[key] = value;
  }
  if (!Object.keys(request).length) invalid('request must contain at least one field.');
  return { command: 'set_command', request, advanceSeconds: advanceOf(input.advance_seconds) };
}

function sameAuthority(a, b) { return Boolean(a && b && a.sessionId === b.sessionId && Number(a.epoch) === Number(b.epoch) && a.sceneRevision === b.sceneRevision && a.robotId === b.robotId); }

function capture(facade, expectedEpoch) {
  facade.assertActive(expectedEpoch);
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || context.profileId !== 'microduck' || context.simulationMode !== 'physical_mujoco' || !context.simulationReady) {
    throw new WebMcpDomainError('PROFILE_MISMATCH', 'This tool requires the active ready MicroDuck physical workspace.', { retryable: true });
  }
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'Finish or stop the active Python run before direct MicroDuck control.', { retryable: true });
  const authority = facade.app.sim.getPhysicalAuthorityToken?.();
  if (!authority?.sessionId) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'MicroDuck PhysicsSession authority is unavailable.', { retryable: true });
  return Object.freeze({ workspaceGeneration: context.workspaceGeneration, simulatorEpoch: context.simulatorEpoch, authority: Object.freeze({ ...authority }) });
}

function assertCurrent(facade, baseline, expectedEpoch, signal, { allowAuthorityChange = false } = {}) {
  if (signal?.aborted) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The MicroDuck control call was cancelled.', { retryable: true });
  facade.assertActive(expectedEpoch);
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || context.profileId !== 'microduck' || context.simulationMode !== 'physical_mujoco'
    || context.workspaceGeneration !== baseline.workspaceGeneration || context.simulatorEpoch !== baseline.simulatorEpoch) {
    throw new WebMcpDomainError('OPERATION_CANCELLED', 'The active MicroDuck workspace or simulator changed during control.', { retryable: true });
  }
  if (!allowAuthorityChange && !sameAuthority(baseline.authority, facade.app.sim.getPhysicalAuthorityToken?.())) {
    throw new WebMcpDomainError('OPERATION_CANCELLED', 'The MicroDuck PhysicsSession epoch changed during control.', { retryable: true });
  }
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'A Python run acquired the simulation during direct control.', { retryable: true });
}

// The three views a caller must be able to tell apart: what was asked for, what the
// controller decided, and what the physics actually is.
function observedState(simulator) {
  const state = simulator.getState();
  return {
    requested: state.requested,
    controller: state.controller,
    actual: state.actual,
    taskEvaluation: simulator.report(),
    note: 'requested is a command, controller is the policy/scaling decision, actual is the achieved MuJoCo state. Only actual is a measurement.',
  };
}

export async function executeMicroDuckPhysicalControl(facade, input, signal, expectedEpoch) {
  const parsed = parse(input);
  const baseline = capture(facade, expectedEpoch);
  if (facade.activeControlId) throw new WebMcpDomainError('COMMAND_CONFLICT', 'Another bounded robot-control call is active.', { retryable: true });
  const controlId = `webmcp-microduck-${expectedEpoch}-${++facade.controlSequence}`;
  facade.activeControlId = controlId;
  const simulator = facade.app.sim.backend;
  // Check the original registration, workspace, physics authority and call signal
  // inside advancement, not just after the entire requested budget has executed.
  const advancementOptions = { assertActive: () => assertCurrent(facade, baseline, expectedEpoch, signal) };
  const base = {
    ok: true, profileId: 'microduck', robot: PROFILES.microduck?.label || 'MicroDuck',
    schemaVersion: WEBMCP_MICRODUCK_PHYSICAL_SCHEMA_VERSION, backend: 'browser-mujoco', hardwareValidated: false,
  };
  try {
    assertCurrent(facade, baseline, expectedEpoch, signal);
    if (typeof simulator?.setCommand !== 'function' || typeof simulator?.advanceSeconds !== 'function') {
      throw new WebMcpDomainError('SIMULATION_NOT_READY', 'The active MicroDuck physical command backend is unavailable.');
    }

    if (parsed.command === 'reset') {
      if (!(await facade.app.resetSimulation())) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'MicroDuck physical simulation could not be reset.', { retryable: true });
      assertCurrent(facade, baseline, expectedEpoch, signal, { allowAuthorityChange: true });
      // A reset establishes a new initial condition. It is never a recovery and never a success.
      return { ...base, command: 'reset', reset: true, countsAsRecovery: false, physicalAuthority: simulator.getPhysicalAuthorityToken?.() };
    }

    if (parsed.command === 'stop') {
      simulator.setCommand({});
      assertCurrent(facade, baseline, expectedEpoch, signal);
      facade.app.setStatus?.('Agent zeroed the MicroDuck physical velocity command');
      facade.app.renderPanels?.();
      // A zero command is a zero command: the plant decides how the robot comes to rest.
      return { ...base, command: 'stop', ...observedState(simulator), physicalAuthority: simulator.getPhysicalAuthorityToken?.() };
    }

    if (parsed.command === 'setup_perturbation') {
      await simulator.applyPerturbation(parsed.perturbation);
      assertCurrent(facade, baseline, expectedEpoch, signal);
      if (parsed.settleSeconds > 0) {
        await simulator.settle(parsed.settleSeconds, advancementOptions);
        assertCurrent(facade, baseline, expectedEpoch, signal);
      }
      facade.app.setStatus?.(`Agent applied the declared ${parsed.perturbation} setup perturbation`);
      facade.app.renderPanels?.();
      return {
        ...base, command: 'setup_perturbation', perturbation: parsed.perturbation,
        settleSeconds: parsed.settleSeconds,
        isDeclaredSetup: true, countsAsTaskProgress: false,
        ...observedState(simulator),
        physicalAuthority: simulator.getPhysicalAuthorityToken?.(),
      };
    }

    if (parsed.command === 'request_skill') {
      const capability = microduckCapability(parsed.skill);
      const result = simulator.requestSkill(parsed.skill);
      if (!result.accepted) {
        const plantMismatch = result.status === 'wrong-plant';
        throw new WebMcpDomainError(plantMismatch ? 'PLANT_MISMATCH' : 'CAPABILITY_UNSUPPORTED', `${parsed.skill} cannot run in the active MicroDuck physical plant: ${result.reason}`, {
          retryable: plantMismatch,
          details: { capability: parsed.skill, status: plantMismatch ? result.status : MICRODUCK_CAPABILITY_STATUS.UNSUPPORTED, requiredPackageKeys: result.requiredPackageKeys || [], routedToLegacy: false },
        });
      }
      assertCurrent(facade, baseline, expectedEpoch, signal);
      if (parsed.advanceSeconds > 0) {
        await simulator.advanceSeconds(parsed.advanceSeconds, advancementOptions);
        assertCurrent(facade, baseline, expectedEpoch, signal);
      }
      facade.app.setStatus?.(`Agent requested the physical ${parsed.skill} skill`);
      facade.app.renderPanels?.();
      return {
        ...base, command: 'request_skill', skill: parsed.skill,
        capabilityStatus: capability?.status ?? null,
        advanceSeconds: parsed.advanceSeconds,
        ...observedState(simulator),
        physicalAuthority: simulator.getPhysicalAuthorityToken?.(),
      };
    }

    if (parsed.command === 'advance') {
      const run = parsed.advanceSeconds === 0
        ? { executedTicks: 0, completed: true, cancelled: false, simulatedSeconds: 0 }
        : await simulator.advanceSeconds(parsed.advanceSeconds, advancementOptions);
      assertCurrent(facade, baseline, expectedEpoch, signal);
      facade.app.renderPanels?.();
      return {
        ...base, command: 'advance', advanceSeconds: parsed.advanceSeconds,
        executedTicks: run.executedTicks, completed: run.completed, cancelled: run.cancelled,
        simulatedSeconds: run.simulatedSeconds,
        ...observedState(simulator),
        physicalAuthority: simulator.getPhysicalAuthorityToken?.(),
      };
    }

    const accepted = simulator.setCommand(parsed.request);
    assertCurrent(facade, baseline, expectedEpoch, signal);
    let run = null;
    if (parsed.advanceSeconds > 0) {
      run = await simulator.advanceSeconds(parsed.advanceSeconds, advancementOptions);
      assertCurrent(facade, baseline, expectedEpoch, signal);
    }
    facade.app.setStatus?.('Agent requested a bounded MicroDuck physical command');
    facade.app.renderPanels?.();
    return {
      ...base, command: 'set_command',
      accepted: accepted.accepted, limitedBy: accepted.limitedBy,
      advanceSeconds: parsed.advanceSeconds,
      executedTicks: run?.executedTicks ?? 0,
      completed: run ? run.completed : null,
      gaitOnsetMS: MICRODUCK_GAIT_ONSET_MS,
      ...observedState(simulator),
      physicalAuthority: simulator.getPhysicalAuthorityToken?.(),
    };
  } catch (error) {
    if (error instanceof WebMcpDomainError) throw error;
    throw new WebMcpDomainError('SIMULATION_REJECTED', String(error?.message || error).slice(0, 280), { retryable: true, details: { profileId: 'microduck' } });
  } finally {
    if (facade.activeControlId === controlId) facade.activeControlId = null;
  }
}

export const MICRODUCK_PHYSICAL_CONTROL_LIMITS = Object.freeze({
  ...L,
  maxAdvanceSeconds: MICRODUCK_MAX_ADVANCE_SECONDS,
  controlIntervalSeconds: MICRODUCK_CONTROL_INTERVAL_SECONDS,
  gaitOnsetMS: MICRODUCK_GAIT_ONSET_MS,
  physicalSkills: PHYSICAL_SKILLS,
  unsupportedSkills: UNSUPPORTED_SKILLS,
  declaredPerturbations: DECLARED_PERTURBATIONS,
});
