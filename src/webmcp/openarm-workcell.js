import { OPENARM_EQUIPMENT_VERSION, validateOpenArmEquipment, EQUIPMENT_KINDS } from '../physics/openarm-equipment.js';
import { conditionMet, objectGeometryIds } from '../physics/openarm-observation.js';
import { WebMcpDomainError } from './agent-facade.js';
import { plain, onlyKeys, finite, vec, vecSchema, invalid, targetsSchema, validateTargets, validateTool, captureOpenArm, assertOpenArmCurrent, sameAuthority, backendFor } from './openarm-physical-control.js';

import { OPENARM_SCENE_MODES, validateOpenArmSceneMode } from '../physics/openarm-workcell-scene.js';

export const OPENARM_PROGRAM_VERSION = 'robobuddy.openarm.program.v1';
const toolSchema = { type: 'object', properties: { side: { enum: ['left', 'right'] }, position_m: vecSchema(3), quaternion_wxyz: vecSchema(4) }, required: ['side', 'position_m'], additionalProperties: false };
export function createOpenArmWorkcellSchema() {
  const item = { type: 'object', properties: {
    id: { type: 'string', pattern: '^[a-z][a-z0-9_]{0,23}$' }, label: { type: 'string', maxLength: 64 }, kind: { enum: [...EQUIPMENT_KINDS] },
    position_m: vecSchema(3), quaternion_wxyz: vecSchema(4), yaw_rad: { type: 'number', minimum: -Math.PI, maximum: Math.PI }, dimensions_m: vecSchema(3), mass_kg: { type: 'number', minimum: .005, maximum: 2 }, dynamic: { type: 'boolean' },
    parts: { type: 'array', minItems: 1, maxItems: 12, items: { type: 'object', properties: { shape: { enum: ['box', 'sphere', 'cylinder'] }, position_m: vecSchema(3), dimensions_m: vecSchema(3), radius_m: { type: 'number', minimum: .001, maximum: .2 }, height_m: { type: 'number', minimum: .002, maximum: .4 } }, required: ['shape', 'position_m'], additionalProperties: false } },
  }, required: ['id', 'kind', 'position_m'], not: { required: ['yaw_rad', 'quaternion_wxyz'] }, additionalProperties: false };
  const version = { type: 'string', const: OPENARM_EQUIPMENT_VERSION };
  return { type: 'object', oneOf: [
    { type: 'object', properties: { schema_version: version, command: { const: 'stage' }, expected_scene_revision: { type: 'string' }, scene_mode: { enum: ['baseline', 'blank'] }, equipment: { type: 'array', maxItems: 12, items: item } }, required: ['schema_version', 'command', 'expected_scene_revision', 'equipment'], additionalProperties: false },
    { type: 'object', properties: { schema_version: version, command: { const: 'apply' }, stage_id: { type: 'string' }, acknowledge_reset: { const: true } }, required: ['schema_version', 'command', 'stage_id', 'acknowledge_reset'], additionalProperties: false },
    { type: 'object', properties: { schema_version: version, command: { const: 'discard' } }, required: ['schema_version', 'command'], additionalProperties: false },
  ] };
}
function conditionSchema() {
  const common = { dwell_seconds: { type: 'number', minimum: .02, maximum: 1 }, timeout_seconds: { type: 'number', minimum: 0, maximum: 5 } };
  const variants = [
    [{ type: { const: 'authored_task_complete' } }, ['type']],
    [{ type: { const: 'funnel_seated' }, object_id: { type: 'string' }, receiver_id: { type: 'string' } }, ['type', 'object_id', 'receiver_id']],
    [{ type: { enum: ['bilateral_grasp', 'released'] }, side: { enum: ['left', 'right'] }, object_id: { type: 'string' } }, ['type', 'side', 'object_id']],
    [{ type: { const: 'supported' }, object_id: { type: 'string' }, support_geom: { type: 'string' } }, ['type', 'object_id', 'support_geom']],
    [{ type: { const: 'in_region' }, object_id: { type: 'string' }, center_m: vecSchema(3), half_extents_m: vecSchema(3) }, ['type', 'object_id', 'center_m', 'half_extents_m']],
    [{ type: { const: 'equipment_joint' }, joint_id: { type: 'string' }, minimum: { type: 'number' }, maximum: { type: 'number' } }, ['type', 'joint_id', 'minimum', 'maximum']],
    [{ type: { const: 'tool_reached' }, side: { enum: ['left', 'right'] }, position_m: vecSchema(3), tolerance_m: { type: 'number', minimum: .001, maximum: .01 } }, ['type', 'side', 'position_m', 'tolerance_m']],
  ];
  return { type: 'object', oneOf: variants.map(([properties, required]) => ({ type: 'object', properties: { ...common, ...properties }, required, additionalProperties: false })) };
}
export function createOpenArmProgramSchema() {
  return { type: 'object', properties: {
    schema_version: { const: OPENARM_PROGRAM_VERSION }, expected_scene_revision: { type: 'string' },
    segments: { type: 'array', minItems: 1, maxItems: 32, items: { type: 'object', properties: {
      label: { type: 'string', maxLength: 80 }, duration_seconds: { type: 'number', minimum: .04, maximum: 12 }, targets_rad: targetsSchema(), tool: toolSchema, wait_for: conditionSchema(),
    }, required: ['duration_seconds'], additionalProperties: false } },
  }, required: ['schema_version', 'expected_scene_revision', 'segments'], additionalProperties: false };
}
export function getOpenArmWorkcellDefinitions(facade) {
  const c = facade.getRegistrationContext();
  if (c.profileId !== 'openarm' || c.simulationMode !== 'physical_mujoco' || c.workspaceStatus !== 'ready' || !c.simulationReady) return [];
  return [
    { name: 'inspect_openarm_workcell', title: 'Inspect OpenArm workcell', description: 'Read physical body poses, per-finger contacts/forces, observed pinch frames, equipment affordances, passive equipment joints, staged geometry and running-program progress. World metres, Z-up, radians. Simulator ground truth, not hardware sensor data.', inputSchema: { type: 'object', properties: {}, additionalProperties: false }, readOnly: true },
    { name: 'manage_openarm_workcell', title: 'Build OpenArm lab equipment', description: `For general unfamiliar equipment or an authored bench use manage_openarm_scene instead. This legacy tool stages bounded ${OPENARM_EQUIPMENT_VERSION} equipment, including hollow funnel/burette/beaker/erlenmeyer_flask, ring_stand, bottle, tile, and the existing trays/racks/vials/blocks/buttons/primitive assemblies. Inspect assetCatalog for dimensions and affordances. position_m is bottom-centre in Z-up world metres; dimensions_m are FULL widths and height. scene_mode:blank removes the baseline task fixtures/flask/beaker, preserving the robot, mount, table and floor. Stage an empty list with scene_mode:blank for a clean workcell; scene_mode:baseline restores the reference fixtures. Omitted mode preserves the current mode. Stage replaces the complete custom equipment list. Optional normalized quaternion_wxyz OR yaw_rad sets setup orientation; position_m remains the local bottom-centre origin. Dimensions/materials are illustrative, not recovered from image pixels. A wireframe preview is visible. Apply requires stage_id and acknowledge_reset:true, checks initial collisions, and explicitly resets into a new scene revision. Failed application preserves the old scene. Stage an empty list to remove custom equipment. No arbitrary code/XML/URLs or hidden object motion.`, inputSchema: createOpenArmWorkcellSchema(), readOnly: false },
    { name: 'run_openarm_program', title: 'Run bounded OpenArm manipulation program', description: `Execute ${OPENARM_PROGRAM_VERSION} against the CURRENT scene without an implicit reset. Up to 32 segments and 60 worst-case simulation seconds. Each segment may send speed-bounded joint targets OR a Cartesian pinch-reference tool target, or just advance. Optional wait_for predicates require sustained observed contacts, placement, release, tool position, passive equipment-joint travel or authored_task_complete (independent observed transfer history), or funnel_seated (actual bore fit, receiver contact, release and settled motion; >=0.2 s dwell); failure stops the sequence and reports actual progress. Reference durations must fit the servo speed limits. Unspecified tool orientation is not constrained. No collision-free planning guarantee, attachment, hardware or publishing. Human Stop and Agent Assist revocation cancel execution.`, inputSchema: createOpenArmProgramSchema(), readOnly: false },
  ];
}
export function inspectOpenArmWorkcell(facade, input, epoch) {
  plain(input); onlyKeys(input, []); facade.assertActive(epoch);
  const c = facade.getRegistrationContext(); if (c.profileId !== 'openarm' || c.workspaceStatus !== 'ready' || !c.simulationReady) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'OpenArm workcell is not ready', { retryable: true });
  const state = backendFor(facade).getWorkcellState();
  return { ok: true, ...state, contacts: state.contacts.slice(0, 64), contactsTruncated: state.contacts.length > 64 };
}
export async function manageOpenArmWorkcell(facade, input, signal, epoch) {
  plain(input); if (input.schema_version !== OPENARM_EQUIPMENT_VERSION) invalid('Unsupported equipment schema_version');
  if (input.command === 'stage') { onlyKeys(input, ['schema_version', 'command', 'expected_scene_revision', 'equipment', 'scene_mode']); validateOpenArmEquipment(input.equipment); if (input.scene_mode !== undefined) { validateOpenArmSceneMode(input.scene_mode); if (input.scene_mode === 'authored') invalid('Use manage_openarm_scene for authored scenes'); } }
  else if (input.command === 'apply') { onlyKeys(input, ['schema_version', 'command', 'stage_id', 'acknowledge_reset']); if (typeof input.stage_id !== 'string' || input.acknowledge_reset !== true) invalid('Applying requires stage_id and acknowledge_reset:true'); }
  else if (input.command === 'discard') onlyKeys(input, ['schema_version', 'command']);
  else invalid('Equipment command must be stage, apply or discard');
  const baseline = captureOpenArm(facade, epoch), app = facade.app, backend = backendFor(facade);
  if (input.command === 'stage' && input.expected_scene_revision !== baseline.authority.sceneRevision) invalid('Scene revision is stale; inspect the workcell again');
  const id = `webmcp-workcell-${epoch}-${++facade.controlSequence}`; facade.activeControlId = id;
  let token = null;
  try {
    if (input.command === 'apply' && app.beginExecution) { token = app.beginExecution(); if (token == null) throw new Error('Could not acquire workcell execution'); }
    const guard = () => assertOpenArmCurrent(facade, baseline, epoch, signal, token);
    guard();
    let result;
    if (input.command === 'stage') result = backend.stageEquipment(input.equipment, input.scene_mode);
    else if (input.command === 'discard') result = backend.discardStagedEquipment();
    else result = await backend.applyStagedEquipment(input.stage_id, true, guard);
    assertOpenArmCurrent(facade, baseline, epoch, signal, token, input.command === 'apply');
    app.setStatus?.(`OpenArm equipment ${input.command === 'stage' ? 'staged for review' : input.command === 'apply' ? 'applied; physical scene explicitly reset' : 'preview discarded'}`); app.renderPanels?.();
    return { ok: true, command: input.command, result, physicalAuthority: app.sim.getPhysicalAuthorityToken(), hardwareValidated: false };
  } finally { if (token != null) app.finishExecution?.(token); if (facade.activeControlId === id) facade.activeControlId = null; }
}
function validateCondition(raw, observation, geometryIds) {
  plain(raw, 'wait_for');
  const fields = { authored_task_complete: [], funnel_seated: ['object_id', 'receiver_id'], bilateral_grasp: ['side', 'object_id'], released: ['side', 'object_id'], supported: ['object_id', 'support_geom'], in_region: ['object_id', 'center_m', 'half_extents_m'], equipment_joint: ['joint_id', 'minimum', 'maximum'], tool_reached: ['side', 'position_m', 'tolerance_m'] }[raw.type];
  if (!fields) invalid('Unsupported wait_for predicate');
  onlyKeys(raw, ['type', 'timeout_seconds', 'dwell_seconds', ...fields], 'wait_for');
  for (const field of fields) if (raw[field] === undefined) invalid(`wait_for requires ${field}`);
  if (fields.includes('side') && !['left', 'right'].includes(raw.side)) invalid('Invalid wait_for side');
  if (fields.includes('object_id') && (typeof raw.object_id !== 'string' || !observation.bodies?.[raw.object_id] || !objectGeometryIds(observation, raw.object_id).length)) invalid('wait_for object_id must identify a vessel or declared equipment body');
  if (raw.type === 'funnel_seated') {
    const assets = observation.openarm?.equipment || [];
    const f = assets.find(e=>e.bodyId===raw.object_id), b = assets.find(e=>e.bodyId===raw.receiver_id);
    if (f?.kind !== 'funnel' || !f.dynamic || b?.kind !== 'burette' || b.dynamic) invalid('funnel_seated requires a dynamic funnel object_id and fixed burette receiver_id');
  }
  if (raw.type === 'supported' && !geometryIds.includes(raw.support_geom)) invalid('Unknown support_geom');
  if (raw.type === 'in_region') { vec(raw.center_m, 3, 'center_m'); vec(raw.half_extents_m, 3, 'half_extents_m').forEach(v => finite(v, .001, .5, 'half extent')); }
  if (raw.type === 'tool_reached') { validateTool({ side: raw.side, position_m: raw.position_m }); finite(raw.tolerance_m, .001, .01, 'tolerance_m'); }
  if (raw.type === 'equipment_joint') {
    const j = observation.openarm?.equipmentJoints?.find(j => j.id === raw.joint_id);
    if (!j) invalid('Unknown passive equipment joint');
    finite(raw.minimum, j.range[0], j.range[1], 'minimum'); finite(raw.maximum, raw.minimum, j.range[1], 'maximum');
  }
  return { ...structuredClone(raw), dwell_seconds: raw.dwell_seconds === undefined ? (raw.type === 'funnel_seated' ? .20 : .06) : finite(raw.dwell_seconds, raw.type === 'funnel_seated' ? .20 : .02, 1, 'dwell_seconds'), timeout_seconds: raw.timeout_seconds === undefined ? 3 : finite(raw.timeout_seconds, 0, 5, 'timeout_seconds') };
}
export function validateOpenArmProgram(input, workcell, observation) {
  plain(input); onlyKeys(input, ['schema_version', 'expected_scene_revision', 'segments']);
  if (input.schema_version !== OPENARM_PROGRAM_VERSION || input.expected_scene_revision !== workcell.authority.sceneRevision) invalid('Unsupported program version or stale scene revision');
  if (new TextEncoder().encode(JSON.stringify(input)).length > 48000) invalid('Program exceeds the 48 kB input budget');
  if (!Array.isArray(input.segments) || !input.segments.length || input.segments.length > 32) invalid('Program must have 1..32 segments');
  const segments = input.segments.map((s, i) => {
    plain(s, 'segment'); onlyKeys(s, ['label', 'duration_seconds', 'targets_rad', 'tool', 'wait_for'], 'segment');
    if (s.wait_for?.type === 'authored_task_complete' && workcell.taskEvaluation?.schemaVersion !== 'robobuddy.lab.task.v2') invalid('Define an authored task before waiting for its completion');
    const duration = finite(s.duration_seconds, .04, 12, 'duration_seconds');
    if (Math.abs(duration/.001-Math.round(duration/.001)) > 1e-7) invalid('Segment duration must align to 1 ms');
    if (s.targets_rad !== undefined && s.tool !== undefined) invalid('Use targets_rad OR tool in a segment, not both');
    if (s.label !== undefined && (typeof s.label !== 'string' || s.label.length > 80)) invalid('Segment label must be at most 80 characters');
    return { label: s.label ?? `Segment ${i+1}`, durationSeconds: duration, targets: s.targets_rad === undefined ? null : validateTargets(s.targets_rad), tool: s.tool === undefined ? null : validateTool(s.tool), condition: s.wait_for === undefined ? null : validateCondition(s.wait_for, observation, workcell.geometryIds) };
  });
  const worstCaseSeconds = segments.reduce((sum, s) => sum + s.durationSeconds + (s.condition?.timeout_seconds || 0), 0);
  if (worstCaseSeconds > 60) invalid('Program exceeds 60 worst-case simulation seconds');
  return { segments, worstCaseSeconds };
}
export async function runOpenArmProgram(facade, input, signal, epoch) {
  const baseline = captureOpenArm(facade, epoch), app = facade.app, backend = backendFor(facade);
  const program = validateOpenArmProgram(input, backend.getWorkcellState(), backend.getState().observation);
  const id = `webmcp-openarm-program-${epoch}-${++facade.controlSequence}`; facade.activeControlId = id;
  const session = app.sim.getPhysicalSession(); const deadline = performance.now() + 180000;
  let token = null, unsubscribe = null, abortReason = null; const completed = [];
  const abort = () => {
    abortReason = 'cancelled';
    if (sameAuthority(baseline.authority, app.sim.getPhysicalAuthorityToken())) void session.cancelRun('webmcp-program-abort').catch(() => {});
    if (token != null && app.runToken === token) app.cancelExecution?.('WEBMCP_ABORT');
  };
  try {
    token = app.beginExecution?.(); if (token == null) throw new Error('OpenArm program could not acquire the execution lease');
    app.openarmAgentProgramActive = true;
    signal?.addEventListener('abort', abort, { once: true });
    const guard = () => { assertOpenArmCurrent(facade, baseline, epoch, signal, token); if (performance.now() > deadline) throw new Error('Program exceeded its 180-second wall-time budget'); };
    const advance = async seconds => {
      let steps = Math.round(seconds/.001);
      while (steps > 0) {
        guard();
        // Pause freezes the lockstep scheduler, not motor velocities. Polling keeps
        // cancellation/deadline checks alive even while a human leaves it paused.
        if (app.getExecutionState() === 'paused') { await new Promise(r => setTimeout(r, 25)); continue; }
        const count = Math.min(40, steps);
        await backend.advanceTime(count * .001); steps -= count; guard();
      }
    };
    guard();
    for (let i=0; i<program.segments.length; i++) {
      guard(); const segment = program.segments[i]; const condition = segment.condition;
      let since = null, met = false, lastTime = null;
      if (condition) unsubscribe = session.subscribe(({observation}) => {
        const time = observation.simulationTimeSeconds;
        if (lastTime != null && time <= lastTime) return;
        if (lastTime != null && time-lastTime > .08) since = null;
        lastTime = time;
        if (condition.type === 'authored_task_complete' ? backend.getTaskEvaluation()?.success === true : conditionMet(observation, condition)) { since ??= time; met = time - since + 1e-9 >= condition.dwell_seconds; }
        else { since = null; met = false; }
      });
      backend.setProgramProgress?.({ status: 'running', id, index: i+1, segments: program.segments.length, label: segment.label, condition: condition?.type || null });
      app.setStatus?.(`OpenArm program ${i+1}/${program.segments.length}: ${segment.label}`);
      const maxSteps = Math.ceil((segment.durationSeconds + (condition?.timeout_seconds || 0))/.001) + 100;
      if (segment.targets) await backend.applyPhysicalTargets(segment.targets, { durationSeconds: segment.durationSeconds, maxSteps });
      else if (segment.tool) await backend.applyToolTarget({ ...segment.tool, durationSeconds: segment.durationSeconds }, { maxSteps });
      else {
        // A dwell receives its own explicit finite budget by re-latching the
        // current references; this neither resets state nor moves an object.
        const references = Object.fromEntries(Object.entries(backend.getState().observation.joints).filter(([key, j]) => j.targetRad != null && !key.endsWith('finger_joint2')).map(([key, j]) => [key, j.referenceRad]));
        await backend.applyPhysicalTargets(references, { durationSeconds: segment.durationSeconds, maxSteps });
      }
      await advance(segment.durationSeconds);
      const end = backend.getState().observation.simulationTimeSeconds;
      while (condition && !met && backend.getState().observation.simulationTimeSeconds < end + condition.timeout_seconds - 1e-9) await advance(Math.min(.04, Math.max(.001, Math.round((end + condition.timeout_seconds - backend.getState().observation.simulationTimeSeconds)/.001)*.001)));
      unsubscribe?.(); unsubscribe = null;
      const observation = backend.getState().observation;
      const record = { index: i+1, label: segment.label, simulationTimeSeconds: observation.simulationTimeSeconds, condition: condition?.type || null, conditionSatisfied: condition ? met : null };
      completed.push(record);
      if (condition && !met) {
        backend.setProgramProgress?.({ status: 'failed', id, ...record }); app.setStatus?.(`OpenArm program stopped: ${segment.label} did not satisfy ${condition.type}`);
        return { ok: false, executionStatus: 'failed', reason: 'condition-timeout', segments: completed, physicalAuthority: app.sim.getPhysicalAuthorityToken(), taskEvaluation: backend.getTaskEvaluation(), hardwareValidated: false };
      }
    }
    guard(); backend.setProgramProgress?.({ status: 'completed', id, segments: completed.length }); app.setStatus?.('OpenArm program complete; observed outcome checks recorded'); app.renderPanels?.();
    return { ok: true, executionStatus: 'completed', outcome: program.segments.some(s => s.condition) ? 'requested-observation-conditions-satisfied' : 'executed-without-custom-outcome-assertions', segments: completed, taskEvaluation: backend.getTaskEvaluation(), physicalAuthority: app.sim.getPhysicalAuthorityToken(), hardwareValidated: false };
  } catch (error) {
    if (sameAuthority(baseline.authority, app.sim.getPhysicalAuthorityToken())) backend.setProgramProgress?.({ status: abortReason || 'interrupted', id, completedSegments: completed.length, reason: String(error.message || error).slice(0, 240) });
    if (error instanceof WebMcpDomainError) throw error;
    throw new WebMcpDomainError('SIMULATION_REJECTED', String(error.message || error).slice(0, 360), { retryable: true, details: { completedSegments: completed.length } });
  } finally {
    unsubscribe?.(); signal?.removeEventListener('abort', abort);
    if (token != null && app.runToken === token) { app.openarmAgentProgramActive = false; app.finishExecution?.(token); }
    if (facade.activeControlId === id) facade.activeControlId = null;
  }
}
