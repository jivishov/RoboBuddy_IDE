import { OPENARM_V2_PHASE5A_MODEL_PACKAGE } from '../physics/openarm-model-package.js';
import { OPENARM_CONTROL_PROFILE } from '../physics/openarm-servo.js';
import { WebMcpDomainError } from './agent-facade.js';

export const WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION = 'robobuddy.openarm.physical.v1';
export const OPENARM_RANGES = Object.freeze(Object.fromEntries(OPENARM_V2_PHASE5A_MODEL_PACKAGE.joints.map(j => {
  const a = OPENARM_V2_PHASE5A_MODEL_PACKAGE.actuators.find(a => a.jointId === j.id);
  return [j.id, [Math.max(j.rangeRad[0], a.controlRangeRad[0]), Math.min(j.rangeRad[1], a.controlRangeRad[1])]];
})));
export function invalid(message) { throw new WebMcpDomainError('INVALID_ARGUMENT', message, { retryable: false }); }
export function plain(value, label = 'input') { if (!value || typeof value !== 'object' || Array.isArray(value) || ![Object.prototype, null].includes(Object.getPrototypeOf(value))) invalid(`${label} must be a plain object`); }
export function onlyKeys(value, keys, label = 'input') { for (const key of Object.keys(value)) if (!keys.includes(key)) invalid(`Unknown ${label} field: ${key}`); }
export function finite(value, minimum, maximum, label) { if (typeof value !== 'number' || !Number.isFinite(value) || value < minimum || value > maximum) invalid(`${label} must be a number in ${minimum}..${maximum}`); return value; }
export function vec(value, length, label) { if (!Array.isArray(value) || value.length !== length || value.some(v => typeof v !== 'number' || !Number.isFinite(v))) invalid(`${label} must contain ${length} finite numbers`); return [...value]; }
export const vecSchema = n => ({ type: 'array', items: { type: 'number' }, minItems: n, maxItems: n });
export function targetsSchema() { return { type: 'object', properties: Object.fromEntries(Object.entries(OPENARM_RANGES).map(([id, r]) => [id, { type: 'number', minimum: r[0], maximum: r[1] }])), minProperties: 1, additionalProperties: false }; }
export function validateTargets(raw) {
  plain(raw, 'targets_rad'); if (!Object.keys(raw).length) invalid('targets_rad must be nonempty');
  return Object.fromEntries(Object.entries(raw).map(([id, value]) => { const r = OPENARM_RANGES[id]; if (!r) invalid(`Unknown controllable OpenArm joint ${id}`); return [id, finite(value, r[0], r[1], id)]; }));
}
export function validateTool(raw) {
  plain(raw, 'tool'); onlyKeys(raw, ['side', 'position_m', 'quaternion_wxyz'], 'tool');
  if (!['left', 'right'].includes(raw.side)) invalid('side must be left or right');
  const positionM = vec(raw.position_m, 3, 'position_m');
  finite(positionM[0], -.3, .95, 'position X'); finite(positionM[1], -.75, .75, 'position Y'); finite(positionM[2], .8, 1.9, 'position Z');
  let quaternionWxyz;
  if (raw.quaternion_wxyz !== undefined) { quaternionWxyz = vec(raw.quaternion_wxyz, 4, 'quaternion_wxyz'); if (Math.abs(Math.hypot(...quaternionWxyz)-1) > .001) invalid('quaternion_wxyz must be normalized'); }
  return { side: raw.side, positionM, ...(quaternionWxyz ? { quaternionWxyz } : {}) };
}
export function createOpenArmPhysicalControlSchema() {
  const common = { schema_version: { const: WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION, type: 'string' }, advance_seconds: { type: 'number', minimum: 0, maximum: 2 }, max_steps: { type: 'integer', minimum: 1, maximum: 15000 }, duration_seconds: { type: 'number', minimum: .04, maximum: 12 } };
  // additionalProperties is inside each branch, not outside oneOf (JSON Schema scope).
  return { type: 'object', oneOf: [
    { type: 'object', properties: { ...common, command: { const: 'set_joint_targets' }, targets_rad: targetsSchema() }, required: ['schema_version', 'command', 'targets_rad'], additionalProperties: false },
    { type: 'object', properties: { ...common, command: { const: 'move_tool' }, side: { enum: ['left', 'right'] }, position_m: vecSchema(3), quaternion_wxyz: vecSchema(4) }, required: ['schema_version', 'command', 'side', 'position_m'], additionalProperties: false },
    { type: 'object', properties: { schema_version: common.schema_version, command: { const: 'advance' }, seconds: { type: 'number', minimum: .001, maximum: 2 } }, required: ['schema_version', 'command', 'seconds'], additionalProperties: false },
    { type: 'object', properties: { schema_version: common.schema_version, command: { const: 'reset' } }, required: ['schema_version', 'command'], additionalProperties: false },
  ] };
}
export function getOpenArmPhysicalControlDefinition(facade) {
  const c = facade.getRegistrationContext();
  if (c.workspaceStatus !== 'ready' || !c.simulationReady || c.profileId !== 'openarm' || c.simulationMode !== 'physical_mujoco') return null;
  return { name: 'control_openarm_simulation', title: 'Control OpenArm physical simulation', description: `Bounded ${WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION} joint targets, Cartesian pinch-reference IK, advance and explicit reset. SI world metres/Z-up and radians. Targets use speed-bounded quintic references and effort-limited actuators; acceptance is NOT achievement. IK does not guarantee a collision-free path. Inspect actual contacts and positions. No object attachments, hardware, save or publish.`, inputSchema: createOpenArmPhysicalControlSchema() };
}
export function sameAuthority(a, b) { return Boolean(a && b && a.sessionId === b.sessionId && a.epoch === b.epoch && a.sceneRevision === b.sceneRevision && a.robotId === b.robotId); }
export function captureOpenArm(facade, epoch) {
  facade.assertActive(epoch); const c = facade.getRegistrationContext();
  if (c.workspaceStatus !== 'ready' || c.profileId !== 'openarm' || c.simulationMode !== 'physical_mujoco' || !c.simulationReady) throw new WebMcpDomainError('PROFILE_MISMATCH', 'Open a ready OpenArm physical workspace', { retryable: true });
  if (facade.app.getExecutionState() !== 'idle' || facade.activeControlId) throw new WebMcpDomainError('SIMULATION_BUSY', 'Stop or finish the active operation first', { retryable: true });
  const authority = facade.app.sim.getPhysicalAuthorityToken?.();
  if (!authority) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'OpenArm authority unavailable', { retryable: true });
  return { context: c, authority };
}
export function assertOpenArmCurrent(facade, baseline, epoch, signal, token = null, allowAuthorityChange = false) {
  if (signal?.aborted) throw new WebMcpDomainError('OPERATION_CANCELLED', 'OpenArm operation cancelled', { retryable: true });
  facade.assertActive(epoch); const c = facade.getRegistrationContext();
  if (c.profileId !== 'openarm' || c.workspaceStatus !== 'ready' || c.workspaceGeneration !== baseline.context.workspaceGeneration || c.simulatorEpoch !== baseline.context.simulatorEpoch || (!allowAuthorityChange && !sameAuthority(baseline.authority, facade.app.sim.getPhysicalAuthorityToken?.()))) throw new WebMcpDomainError('OPERATION_CANCELLED', 'OpenArm workspace or session changed', { retryable: true });
  if (token != null ? facade.app.runToken !== token : facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('OPERATION_CANCELLED', 'OpenArm control ownership changed', { retryable: true });
}
export function backendFor(facade) { return facade.app.sim.backend || facade.app.sim; }
export function compactObserved(observation) { return { simulationTimeSeconds: observation?.simulationTimeSeconds, jointsRad: Object.fromEntries(Object.entries(observation?.joints || {}).map(([id, j]) => [id, j.positionRad])), flaskPositionM: observation?.bodies?.flask?.positionM, beakerPositionM: observation?.bodies?.beaker?.positionM, pinchReferences: observation?.openarm?.pinchReferences, motionPlan: observation?.openarm?.motionPlan, contactCount: observation?.contactCount }; }

export async function executeOpenArmPhysicalControl(facade, input, signal, epoch) {
  plain(input); if (input.schema_version !== WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION) invalid('Unsupported OpenArm schema_version');
  const command = input.command;
  let targets, tool, seconds = 0, duration = null, maxSteps = 2000;
  if (command === 'reset') onlyKeys(input, ['schema_version', 'command']);
  else if (command === 'advance') { onlyKeys(input, ['schema_version', 'command', 'seconds']); seconds = finite(input.seconds, .001, 2, 'seconds'); }
  else if (command === 'set_joint_targets' || command === 'move_tool') {
    onlyKeys(input, ['schema_version', 'command', 'advance_seconds', 'max_steps', 'duration_seconds', ...(command === 'move_tool' ? ['side', 'position_m', 'quaternion_wxyz'] : ['targets_rad'])]);
    if (command === 'set_joint_targets') targets = validateTargets(input.targets_rad);
    else tool = validateTool({ side: input.side, position_m: input.position_m, ...(input.quaternion_wxyz === undefined ? {} : { quaternion_wxyz: input.quaternion_wxyz }) });
    seconds = input.advance_seconds === undefined ? 0 : finite(input.advance_seconds, 0, 2, 'advance_seconds');
    if (input.duration_seconds !== undefined) duration = finite(input.duration_seconds, .04, 12, 'duration_seconds');
    maxSteps = input.max_steps === undefined ? 15000 : finite(input.max_steps, 1, 15000, 'max_steps');
    if (!Number.isInteger(maxSteps) || seconds/.001 > maxSteps + 1e-9) invalid('Advance must fit an integer step budget');
  } else invalid('command must be set_joint_targets, move_tool, advance or reset');
  if (Math.abs(seconds/.001 - Math.round(seconds/.001)) > 1e-7) invalid('Simulation time must align to 1 ms');
  const baseline = captureOpenArm(facade, epoch), app = facade.app, backend = backendFor(facade);
  const controlId = `webmcp-openarm-${epoch}-${++facade.controlSequence}`; facade.activeControlId = controlId;
  let token = null;
  const session = app.sim.getPhysicalSession?.();
  const abort = () => { if (sameAuthority(baseline.authority, app.sim.getPhysicalAuthorityToken?.())) void session?.cancelRun('webmcp-abort').catch(() => {}); };
  try {
    if (command !== 'reset' && typeof app.beginExecution === 'function') { token = app.beginExecution(); if (token == null) throw new Error('Cannot acquire OpenArm execution'); }
    signal?.addEventListener('abort', abort, { once: true });
    assertOpenArmCurrent(facade, baseline, epoch, signal, token);
    let result;
    if (command === 'reset') {
      if (!await app.resetSimulation()) throw new Error('OpenArm reset failed');
      assertOpenArmCurrent(facade, baseline, epoch, signal, null, true);
      return { ok: true, profileId: 'openarm', command, reset: true, schemaVersion: WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION, hardwareValidated: false, physicalAuthority: app.sim.getPhysicalAuthorityToken() };
    }
    if (command === 'advance') result = { observation: await backend.advanceTime(seconds), status: 'advanced' };
    else if (tool) result = await backend.applyToolTarget({ ...tool, ...(duration == null ? {} : { durationSeconds: duration }) }, { advanceSeconds: seconds, maxSteps });
    else result = await app.sim.applyPhysicalTargets(targets, { advanceSeconds: seconds, maxSteps, durationSeconds: duration });
    assertOpenArmCurrent(facade, baseline, epoch, signal, token);
    app.setStatus?.('Agent applied bounded OpenArm control; inspect achieved state'); app.renderPanels?.();
    return { ok: true, profileId: 'openarm', command, schemaVersion: WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION, hardwareValidated: false, commandStatus: result.status, commandId: result.commandId, acceptedTargetsRad: result.acceptedTargetsRad, observedState: compactObserved(result.observation), taskEvaluation: backend.getTaskEvaluation?.(), physicalAuthority: app.sim.getPhysicalAuthorityToken(), controlProfile: OPENARM_CONTROL_PROFILE };
  } catch (error) {
    if (error instanceof WebMcpDomainError) throw error;
    throw new WebMcpDomainError('SIMULATION_REJECTED', String(error.message || error).slice(0, 360), { retryable: true });
  } finally {
    signal?.removeEventListener('abort', abort); if (token != null) app.finishExecution?.(token);
    if (facade.activeControlId === controlId) facade.activeControlId = null;
  }
}
