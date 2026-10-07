import { asimovProgramSchemas,parseAsimovProgram,executeAsimovProgram,ASIMOV_PROGRAM_LIMITS,ASIMOV_PROGRAM_REVISION,ASIMOV_UPPER_BODY_JOINTS } from './asimov-program.js';
import { asimovWholeBodySchema,parseAsimovWholeBody,executeAsimovWholeBodyMotion,ASIMOV_WHOLE_BODY_LIMITS,ASIMOV_WHOLE_BODY_REVISION } from './asimov-whole-body.js';
import { ASIMOV_MASS_RECONCILIATION } from '../physics/asimov-sensitivity.js';
import { PROFILES } from '../profiles.js';
import { ASIMOV_SOURCE } from '../physics/asimov-generated.js';
import { ASIMOV_LIMITATIONS } from '../physics/asimov-model-package.js';
import { WebMcpDomainError } from './agent-facade.js';

// Opt-in bounded access to the same Asimov PhysicsSession used by Python and rendering.
export const WEBMCP_ASIMOV_SCHEMA_VERSION = 'robobuddy.asimov.physical.v1';
const MAX_ADVANCE_SECONDS = 2;
const DEFAULT_COMMAND_STEPS = 4000;
const MAX_COMMAND_STEPS = 40000;
const JOINT_RANGES=Object.freeze(Object.fromEntries(ASIMOV_SOURCE.joints.map(j=>[j.id,j.rangeRad])));

function invalid(message) { throw new WebMcpDomainError('INVALID_ARGUMENT', message, { retryable: false }); }
function plain(value, label = 'input') { if (!value || typeof value !== 'object' || Array.isArray(value)) invalid(`${label} must be an object.`); }
function onlyKeys(value, allowed, label = 'input') { for (const key of Object.keys(value)) if (!allowed.includes(key)) invalid(`Unexpected ${label} field: ${key}.`); }

export function createAsimovControlSchema() {
  const version = { type: 'string', const: WEBMCP_ASIMOV_SCHEMA_VERSION };
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
      ...asimovProgramSchemas(version,targets),
      asimovWholeBodySchema(version,targets),
      {type:'object',properties:{schema_version:version,command:{type:'string',const:'set_standing_targets'},targets_rad:{...targets,properties:Object.fromEntries(ASIMOV_UPPER_BODY_JOINTS.map(id=>[id,targets.properties[id]]))},advance_seconds:{type:'number',minimum:0,maximum:2},max_steps:{type:'integer',minimum:1,maximum:40000}},required:['schema_version','command','targets_rad'],additionalProperties:false},
      bare('inspect_capability'),
      bare('read_state'),
      bare('read_sensors'),
      bare('engage_stand'),
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
    ],
  };
}

export function getAsimovPhysicalControlDefinition(facade) {
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || !context.simulationReady || context.profileId !== 'asimov' || context.simulationMode !== 'physical_mujoco') return null;
  return {
    name: 'control_asimov_physical_simulation',
    title: 'Control the Asimov 1 physical MuJoCo simulation',
    description: `Control the active Asimov physical scene through ${WEBMCP_ASIMOV_SCHEMA_VERSION}. Bounded joint torques under the selected reference or experimental actuator profile, measured state, gravity and contacts. Estimated PD gains are not hardware calibration. Mounted scenes fix the pelvis explicitly. Experimental standing can be engaged only in its declared standing scene; acceptance is not achievement. Separate synthetic sensor reads are available only in actuator experiments. run_sequence accepts 1..32 prevalidated segments totaling at most 12 simulation seconds; wait_for_joint tests measured attainment and can time out. run_whole_body_motion accepts bounded agent-generated full-body keyframes in actuated free-base scenes for stepping, walking attempts, turning, squatting, reaching, gestures or other motions; achieved displacement and support come from MuJoCo contact physics, not the intent label. It has no stabilization assistance by default. Optional ground_truth stabilization is an explicit simulator-only ankle-target supervisor, not a trained gait or hardware controller. Passive Gravity Drop remains observation-only. Inspect capability for the 23 joint IDs, limits, selected controller and timestep. No root-pose write, root-velocity write, external-force, hidden reset, grasp or hardware operation is exposed.`,
    inputSchema: createAsimovControlSchema(),
  };
}

function parse(input) {
  plain(input);
  if (input.schema_version !== WEBMCP_ASIMOV_SCHEMA_VERSION) invalid(`schema_version must be ${WEBMCP_ASIMOV_SCHEMA_VERSION}.`);
  if(typeof input.command!=='string')invalid('command must be a string.');
  const command = input.command;
  if(['run_sequence','wait_for_joint'].includes(command)) {try{return parseAsimovProgram(input);}catch(e){invalid(e.message);}}
  if(command==='run_whole_body_motion') {try{return parseAsimovWholeBody(input);}catch(e){invalid(e.message);}}
  if (['inspect_capability', 'read_state', 'read_sensors', 'engage_stand', 'stop', 'reset'].includes(command)) {
    onlyKeys(input, ['schema_version', 'command']);
    return { command };
  }
  const advanceOf = (value, { required = false } = {}) => {
    if (value === undefined) {
      if (required) invalid('advance_seconds is required for this command.');
      return 0;
    }
    const seconds = value;
    if (!Number.isFinite(seconds) || seconds < 0 || seconds > MAX_ADVANCE_SECONDS) invalid(`advance_seconds must be between 0 and ${MAX_ADVANCE_SECONDS}.`);
    if (required && seconds <= 0) invalid('advance_seconds must be greater than 0.');
    return seconds;
  };
  if (command === 'advance') {
    onlyKeys(input, ['schema_version', 'command', 'advance_seconds']);
    return { command, advanceSeconds: advanceOf(input.advance_seconds, { required: true }) };
  }
  if (!['set_joint_targets','set_standing_targets'].includes(command)) invalid('Asimov 1 physical command must be inspect_capability, read_state, read_sensors, engage_stand, set_joint_targets, set_standing_targets, run_sequence, wait_for_joint, run_whole_body_motion, advance, stop or reset.');
  onlyKeys(input, ['schema_version', 'command', 'targets_rad', 'advance_seconds', 'max_steps']);
  plain(input.targets_rad, 'targets_rad');
  const entries = Object.entries(input.targets_rad);
  if (!entries.length) invalid('targets_rad must contain at least one joint target.');
  const targetsRad = {};
  for (const [jointId, raw] of entries) {
    const range = JOINT_RANGES[jointId];
    if (!Object.hasOwn(JOINT_RANGES,jointId)) invalid(`Unknown Asimov 1 joint: ${jointId}.`);
    const value = raw;
    if (!Number.isFinite(value) || value < range[0] || value > range[1]) invalid(`${jointId} must be within ${range[0]}..${range[1]} radians.`);
    if(command==='set_standing_targets'&&!ASIMOV_UPPER_BODY_JOINTS.includes(jointId))invalid('Standing targets accept waist/arm joints only.');
    targetsRad[jointId] = value;
  }
  const steps = input.max_steps === undefined ? DEFAULT_COMMAND_STEPS : input.max_steps;
  if (!Number.isInteger(steps) || steps < 1 || steps > MAX_COMMAND_STEPS) invalid(`max_steps must be an integer from 1 to ${MAX_COMMAND_STEPS}.`);
  return { command, targetsRad, advanceSeconds: advanceOf(input.advance_seconds), maxSteps: steps };
}

function sameAuthority(a, b) { return Boolean(a && b && a.sessionId === b.sessionId && Number(a.epoch) === Number(b.epoch) && a.sceneRevision === b.sceneRevision && a.robotId === b.robotId); }

function capture(facade, expectedEpoch) {
  facade.assertActive(expectedEpoch);
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || context.profileId !== 'asimov' || context.simulationMode !== 'physical_mujoco' || !context.simulationReady) throw new WebMcpDomainError('PROFILE_MISMATCH', 'This tool requires the active ready Asimov 1 physical workspace.', { retryable: true });
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'Finish or stop the active Python run before direct Asimov 1 control.', { retryable: true });
  const authority = facade.app.sim.getPhysicalAuthorityToken?.();
  if (!authority?.sessionId) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'Asimov 1 PhysicsSession authority is unavailable.', { retryable: true });
  return Object.freeze({ workspaceGeneration: context.workspaceGeneration, simulatorEpoch: context.simulatorEpoch, authority: Object.freeze({ ...authority }), simulator: facade.app.sim.backend ?? facade.app.sim });
}

function assertCurrent(facade, baseline, expectedEpoch, signal, { allowAuthorityChange = false } = {}) {
  if (signal?.aborted) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The Asimov 1 control call was cancelled.', { retryable: true });
  facade.assertActive(expectedEpoch);
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || context.profileId !== 'asimov' || context.simulationMode !== 'physical_mujoco' || context.workspaceGeneration !== baseline.workspaceGeneration || context.simulatorEpoch !== baseline.simulatorEpoch) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The active Asimov 1 workspace or simulator changed during control.', { retryable: true });
  if (!allowAuthorityChange && !sameAuthority(baseline.authority, facade.app.sim.getPhysicalAuthorityToken?.())) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The Asimov 1 PhysicsSession epoch changed during control.', { retryable: true });
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'A Python run acquired the simulation during direct control.', { retryable: true });
}

function wholeBodyAvailability(facade, state) {
  const sceneId=facade.app.scenario?.physicalSceneId??'';
  if (state?.root?.mode !== 'free-base') return 'requires-free-base';
  if (sceneId === 'asimov-drop' || state?.controller_mode === 'passive') return 'unavailable-passive-gravity-drop';
  if (state?.actuation_enabled === false) return 'requires-enabled-actuation';
  return 'experimental-agent-generated-keyframes';
}

function observedState(facade) {
  const state = facade.app.sim.getState?.();
  if (!state) return null;
  const wholeBody=wholeBodyAvailability(facade,state);
  return {
    simulationTimeSeconds: state.simulation_time_s,
    root: state.root,
    footContacts: state.foot_contacts,
    contacts: state.contacts,
    ...(state.actuator_model ? { actuator_model: structuredClone(state.actuator_model) } : {}),
    ...(state.standing_assessment ? { standing_assessment: structuredClone(state.standing_assessment) } : {}),
    controllerMode: state.controller_mode,
    actuationEnabled: state.actuation_enabled,
    joints: state.joints,
    wholeBodyMotion: wholeBody,
    walking: wholeBody==='experimental-agent-generated-keyframes'?'experimental-agent-generated-trajectory; not a validated gait policy':wholeBody,
  };
}

export async function executeAsimovPhysicalControl(facade, input, signal, expectedEpoch) {
  const parsed = parse(input);
  const baseline = capture(facade, expectedEpoch);
  if (facade.activeControlId) throw new WebMcpDomainError('COMMAND_CONFLICT', 'Another bounded robot-control call is active.', { retryable: true });
  const controlId = `webmcp-asimov-${expectedEpoch}-${++facade.controlSequence}`;
  facade.activeControlId = controlId;
  const sim=baseline.simulator;
  const deadline=performance.now()+ASIMOV_PROGRAM_LIMITS.maxWallSeconds*1000;
  const guard=()=>{assertCurrent(facade,baseline,expectedEpoch,signal);if(performance.now()>=deadline)throw new WebMcpDomainError('OPERATION_CANCELLED','Asimov call reached its fixed wall deadline.',{retryable:true});};
  const advance=async(seconds)=>{guard();await sim.advanceTime(seconds,{signal,assertActive:guard,deadlineMs:deadline});guard();};
  const base = { ok: true, profileId: 'asimov', robot: PROFILES.asimov.label, schemaVersion: WEBMCP_ASIMOV_SCHEMA_VERSION, hardwareValidated: false };
  try {
    guard();

    if (parsed.command === 'inspect_capability') {
      const state=facade.app.sim.getState?.();
      const rootMode=state?.root?.mode;
      const wholeBody=wholeBodyAvailability(facade,state);
      const wholeBodyReady=wholeBody==='experimental-agent-generated-keyframes';
      return {
        ...base,
        command: 'inspect_capability',
        sourceRevision: ASIMOV_SOURCE.revision,
        rootMode,
        capabilities: {
          jointTargets:'supported',standingUpperBodyTargets:'supported-in-active-standing-trial',sequence:'supported',measuredJointWait:'supported',
          wholeBodyMotion:wholeBody,
          stepping:wholeBodyReady?'experimental-physical-trajectory':wholeBody,
          walking:wholeBodyReady?'experimental-agent-generated-trajectory; not a trained or validated gait policy':wholeBody,
          turning:wholeBodyReady?'experimental-physical-trajectory':wholeBody,
          standing:/^asimov-(?:standing$|sensor-standing)/.test(facade.app.scenario?.physicalSceneId??'')?'experimental-flat-floor-trial':'unsupported',neck:'fixed in source'
        },
        programming:{
          revision:ASIMOV_PROGRAM_REVISION,limits:ASIMOV_PROGRAM_LIMITS,
          wholeBodyRevision:ASIMOV_WHOLE_BODY_REVISION,wholeBodyLimits:ASIMOV_WHOLE_BODY_LIMITS,
          wholeBodyDefaultStabilization:'none',
          wholeBodyStabilization:'optional bounded ground-truth ankle-target correction at 20 Hz; simulator-only, not hardware-like sensing or a trained gait; final targets remain source-velocity-limited',
          angleUnit:'rad',timeUnit:'simulation seconds',timestepSeconds:facade.app.scenario?.controller?.physicsTimestepSeconds??sim.selectedScene?.physics?.timestepSeconds,
          manualTargetsReleaseStanding:true,standingControllerId:sim.selectedScene?.controllers?.find(id=>['asimov-stance-feedback-v1','asimov-sensor-stance-v2'].includes(id))??null
        },
        jointDefinitions:ASIMOV_SOURCE.joints.map(j=>({id:j.id,rangeRad:j.rangeRad,velocityLimitRadS:j.velocityLimitRadS,sourceEffortLimitNm:j.effortLimitNm})),
        massReconciliation:ASIMOV_MASS_RECONCILIATION,
        actuatorModel:state?.actuator_model??null,
        limitations: [...(facade.app.scenario?.limitations??ASIMOV_LIMITATIONS), 'Agent-generated whole-body trajectories are simulator experiments. A completed trajectory is not proof of a stable step, walk, turn or hardware-feasible motion; inspect measured displacement, contacts, support transitions and final state. Passive Gravity Drop remains observation-only.'],
        physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.(),
      };
    }

    if (['run_sequence','wait_for_joint'].includes(parsed.command)) {
      const dt=facade.app.scenario?.controller?.physicsTimestepSeconds??sim.selectedScene?.physics?.timestepSeconds;
      const result=await executeAsimovProgram(sim,parsed,{dt,guard,advance});guard();facade.app.renderPanels?.();
      return {...base,command:parsed.command,...result,observedState:observedState(facade),taskEvaluation:sim.getTaskEvaluation?.(),physicalAuthority:baseline.authority};
    }

    if(parsed.command==='run_whole_body_motion') {
      const dt=facade.app.scenario?.controller?.physicsTimestepSeconds??sim.selectedScene?.physics?.timestepSeconds;
      const result=await executeAsimovWholeBodyMotion(sim,parsed,{dt,guard,advance});guard();facade.app.renderPanels?.();
      return {...base,command:parsed.command,...result,observedState:observedState(facade),taskEvaluation:sim.getTaskEvaluation?.(),physicalAuthority:baseline.authority};
    }

    if (parsed.command === 'read_state') {
      return { ...base, command: 'read_state', observedState: observedState(facade), taskEvaluation: facade.app.sim.getTaskEvaluation?.(), physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.() };
    }

    if (parsed.command === 'read_sensors') {
      return {...base,command:'read_sensors',sensorObservation:facade.app.sim.getSensorObservation(),physicalAuthority:facade.app.sim.getPhysicalAuthorityToken?.()};
    }
    if (parsed.command === 'engage_stand') {
      if(!/^asimov-(?:standing$|sensor-standing)/.test(facade.app.scenario?.physicalSceneId??'')) invalid('Standing is declared only in a matching experimental standing scene');
      await sim.engageStand();
      assertCurrent(facade,baseline,expectedEpoch,signal);
      facade.app.renderPanels?.();
      return {...base,command:'engage_stand',note:'Controller engaged without advancing time; read the continuous standing assessment after bounded advances.',observedState:observedState(facade),physicalAuthority:facade.app.sim.getPhysicalAuthorityToken?.()};
    }

    if (parsed.command === 'reset') {
      if (!(await facade.app.resetSimulation())) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'Asimov 1 physical simulation could not be reset.', { retryable: true });
      assertCurrent(facade, baseline, expectedEpoch, signal, { allowAuthorityChange: true });
      // Reset establishes a new initial condition. It is never a recovery and never a success.
      return { ...base, command: 'reset', reset: true, note: 'reset establishes a new initial condition; it is not a recovery and is not task progress', physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.() };
    }

    if (parsed.command === 'stop') {
      await sim.stop();
      assertCurrent(facade, baseline, expectedEpoch, signal);
      facade.app.setStatus?.('Agent commanded a bounded Asimov 1 hold at the measured joint state');
      facade.app.renderPanels?.();
      return { ...base, command: 'stop', observedState: observedState(facade), note: 'stop holds the measured joint state through the bounded controller; it does not teleport velocities to zero', physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.() };
    }

    if (parsed.command === 'advance') {
      await advance(parsed.advanceSeconds);
      assertCurrent(facade, baseline, expectedEpoch, signal);
      facade.app.renderPanels?.();
      return { ...base, command: 'advance', advanceSeconds: parsed.advanceSeconds, observedState: observedState(facade), taskEvaluation: facade.app.sim.getTaskEvaluation?.(), physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.() };
    }

    const result = await (parsed.command==='set_standing_targets'?sim.applyStandingTargets.bind(sim):sim.applyPhysicalTargets.bind(sim))(parsed.targetsRad, { maxSteps: parsed.maxSteps, advanceSeconds: parsed.advanceSeconds, signal, assertActive:guard, deadlineMs:deadline });
    assertCurrent(facade, baseline, expectedEpoch, signal);
    facade.app.setStatus?.('Agent applied bounded Asimov 1 joint targets');
    facade.app.renderPanels?.();
    return {
      ...base,
      command: parsed.command,
      commandStatus: result?.status ?? 'accepted',
      commandId: result?.commandId ?? null,
      requestedTargetsRad: structuredClone(parsed.targetsRad),
      advanceSeconds: parsed.advanceSeconds,
      observedState: observedState(facade),
      taskEvaluation: facade.app.sim.getTaskEvaluation?.(),
      note: 'requestedTargetsRad is a command, not a measurement; observedState.joints reports the accepted command and the measured position separately.',
      physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.(),
    };
  } catch (error) {
    if (error instanceof WebMcpDomainError) throw error;
    throw new WebMcpDomainError('SIMULATION_REJECTED', String(error?.message || error).slice(0, 280), { retryable: true, details: { profileId: 'asimov' } });
  } finally {
    if (facade.activeControlId === controlId) facade.activeControlId = null;
  }
}

export const ASIMOV_CONTROL_LIMITS = Object.freeze({
  maxAdvanceSeconds: MAX_ADVANCE_SECONDS,
  maxCommandSteps: MAX_COMMAND_STEPS,
  defaultCommandSteps: DEFAULT_COMMAND_STEPS,
  exposedCommands: Object.freeze(['inspect_capability', 'read_state', 'read_sensors', 'engage_stand', 'set_joint_targets', 'advance', 'stop', 'reset', 'run_sequence', 'wait_for_joint', 'set_standing_targets', 'run_whole_body_motion']),
  // Named so a reviewer can grep for them: none of these exists as an agent-reachable operation.
  neverExposed: Object.freeze(['set_root_pose', 'set_root_velocity', 'force_upright', 'set_actuation', 'apply_external_force', 'place_object', 'connect_hardware']),
});
