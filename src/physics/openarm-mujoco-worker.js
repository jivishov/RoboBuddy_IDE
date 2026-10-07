import { OpenArmServo, OPENARM_CONTROL_PROFILE } from './openarm-servo.js';
import { validateOpenArmEquipment, appendEquipmentXml } from './openarm-equipment.js';
import { workcellBaseXml, validateOpenArmSceneMode } from './openarm-workcell-scene.js';
import { appendGeneralSceneXml } from './openarm-general-scene.js';
import { solveOpenArmIK } from './openarm-ik.js';
import loadMujoco from '../../assets/microduck/runtime/mujoco/mujoco.js';
import { MAX_ADVANCE_STEPS_PER_REQUEST, MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE, sampledObservationCount } from './backend-contract.js';

const MUJOCO_BASE_URL = new URL('../../assets/microduck/runtime/mujoco/', import.meta.url);
const EXPECTED_MUJOCO_VERSION = '3.11.0';
const MAX_STEP_BATCH = MAX_ADVANCE_STEPS_PER_REQUEST;
const MODEL_ASSET_RE = /^models\/[A-Za-z0-9._/-]+\.xml$/;
const MODEL_VALUE_TOLERANCE = 1e-9;
const INTEGRATOR_CODES = Object.freeze({ Euler: 0, RK4: 1, implicit: 2, implicitfast: 3 });

let mujoco = null;
let model = null;
let data = null;
let paused = false;
let descriptor = null;
let jointState = new Map();
let couplingState = new Map();
let actuatorState = new Map();
let bodyState = new Map();
let modelInfo = null;
let servo = null;
let equipment = null;
let motionPlan = null;

function reply(id, ok, payload = null, error = null) { postMessage({ id, ok, payload, error }); }
async function ensureMuJoCo() { if (mujoco) return mujoco; mujoco = await loadMujoco({ locateFile: (path) => new URL(path, MUJOCO_BASE_URL).href }); return mujoco; }
async function sha256Text(text) { const digest = await crypto.subtle.digest('SHA-256', new TextEncoder().encode(text)); return [...new Uint8Array(digest)].map((value) => value.toString(16).padStart(2, '0')).join(''); }
function enumValue(name) { const entry = mujoco?.mjtObj?.[name]; const value = entry && typeof entry === 'object' && 'value' in entry ? entry.value : entry; if (!Number.isInteger(Number(value))) throw new Error(`MuJoCo object enum ${name} is unavailable`); return Number(value); }
function idFor(typeName, name) { const id = Number(mujoco.mj_name2id(model, enumValue(typeName), name)); if (!Number.isInteger(id) || id < 0) throw new Error(`Required MuJoCo ${typeName} named ${name} is missing`); return id; }
function nameForGeom(id) { if (!Number.isInteger(id) || id < 0 || typeof mujoco?.mj_id2name !== 'function') return null; try { return mujoco.mj_id2name(model, enumValue('mjOBJ_GEOM'), id) || null; } catch { return null; } }
function runtimeVersionEvidence() { const version = typeof mujoco?.mj_versionString === 'function' ? String(mujoco.mj_versionString()) : EXPECTED_MUJOCO_VERSION; if (version !== EXPECTED_MUJOCO_VERSION) throw new Error(`Bundled MuJoCo runtime reports ${version}; expected ${EXPECTED_MUJOCO_VERSION}`); return { version, evidence: typeof mujoco?.mj_versionString === 'function' ? 'mj_versionString runtime introspection' : 'bundled asset manifest and repository hash gate' }; }
function validatedModelUrl(asset) { if (!MODEL_ASSET_RE.test(asset || '') || String(asset).includes('..')) throw new Error('Worker rejected non-registry model asset path'); const url = new URL(`../../${asset}`, import.meta.url); if (url.origin !== self.location.origin) throw new Error('Worker model asset must be same-origin'); return url.href; }
function assertNumericArrayClose(actual, expected, label, tolerance = MODEL_VALUE_TOLERANCE) { if (!Array.isArray(expected) || actual.length !== expected.length) throw new Error(`${label} descriptor shape mismatch`); for (let index = 0; index < actual.length; index += 1) { const delta = Math.abs(Number(actual[index]) - Number(expected[index])); if (!Number.isFinite(delta) || delta > tolerance) throw new Error(`${label} mismatch at index ${index}: compiled ${actual[index]}, descriptor ${expected[index]}`); } }

function resolveModelAddresses(modelSha256) {
  jointState = new Map(); couplingState = new Map(); actuatorState = new Map(); bodyState = new Map();
  for (const joint of descriptor.joints) {
    const id = idFor('mjOBJ_JOINT', joint.id);
    const qpos = Number(model.jnt_qposadr[id]);
    const dof = Number(model.jnt_dofadr[id]);
    const range = [Number(model.jnt_range[id * 2]), Number(model.jnt_range[id * 2 + 1])];
    const axis = [Number(model.jnt_axis[id * 3]), Number(model.jnt_axis[id * 3 + 1]), Number(model.jnt_axis[id * 3 + 2])];
    if (joint.rangeRad) assertNumericArrayClose(range, joint.rangeRad, `Joint ${joint.id} range`);
    if (joint.axis) assertNumericArrayClose(axis, joint.axis, `Joint ${joint.id} axis`);
    jointState.set(joint.id, { id, qpos, dof, range, axis });
  }
  for (const coupling of descriptor.mechanicalCouplings || []) {
    const driver = jointState.get(coupling.driverJointId);
    if (!driver) throw new Error(`Mechanical coupling ${coupling.id} references unknown driver ${coupling.driverJointId}`);
    if (jointState.has(coupling.followerJointId)) throw new Error(`Mechanical coupling follower ${coupling.followerJointId} collides with a declared command joint`);
    const id = idFor('mjOBJ_JOINT', coupling.followerJointId);
    const qpos = Number(model.jnt_qposadr[id]);
    const dof = Number(model.jnt_dofadr[id]);
    const range = [Number(model.jnt_range[id * 2]), Number(model.jnt_range[id * 2 + 1])];
    const axis = [Number(model.jnt_axis[id * 3]), Number(model.jnt_axis[id * 3 + 1]), Number(model.jnt_axis[id * 3 + 2])];
    const resolved = { ...coupling, driver, id, qpos, dof, range, axis, passiveCoupled: true };
    couplingState.set(coupling.id, resolved);
    jointState.set(coupling.followerJointId, { id, qpos, dof, range, axis, passiveCoupled: true });
  }
  for (const actuator of descriptor.actuators) {
    const id = idFor('mjOBJ_ACTUATOR', actuator.id);
    const joint = jointState.get(actuator.jointId);
    if (!joint) throw new Error(`Actuator ${actuator.id} references unknown descriptor joint ${actuator.jointId}`);
    if (model.actuator_trnid?.length && Number(model.actuator_trnid[id * 2]) !== joint.id) throw new Error(`Actuator ${actuator.id} is not mapped to joint ${actuator.jointId}`);
    const controlRange = [Number(model.actuator_ctrlrange[id * 2]), Number(model.actuator_ctrlrange[id * 2 + 1])];
    if (actuator.controlRangeRad) assertNumericArrayClose(controlRange, actuator.controlRangeRad, `Actuator ${actuator.id} control range`);
    actuatorState.set(actuator.id, { ...actuator, id, controlRange });
  }
  for (const body of descriptor.bodies || []) {
    const id = idFor('mjOBJ_BODY', body.id);
    let freeDof = null;
    if (body.freeJointId) {
      const freeJointId = idFor('mjOBJ_JOINT', body.freeJointId);
      freeDof = Number(model.jnt_dofadr[freeJointId]);
      if (!Number.isInteger(freeDof) || freeDof < 0) throw new Error(`Body ${body.id} free joint ${body.freeJointId} has no valid dof address`);
    }
    bodyState.set(body.id, { id, freeDof });
  }
  const timestepSeconds = Number(model.opt?.timestep);
  if (!Number.isFinite(timestepSeconds) || Math.abs(timestepSeconds - Number(descriptor.physics.timestepSeconds)) > 1e-12) throw new Error(`Unexpected timestep ${timestepSeconds}; expected ${descriptor.physics.timestepSeconds}`);
  const expectedIntegrator = INTEGRATOR_CODES[descriptor.physics.integrator];
  const compiledIntegrator = Number(model.opt?.integrator);
  if (!Number.isInteger(expectedIntegrator) || compiledIntegrator !== expectedIntegrator) throw new Error(`Unexpected integrator ${compiledIntegrator}; expected ${descriptor.physics.integrator} (${expectedIntegrator})`);
  const version = runtimeVersionEvidence();
  modelInfo = { modelSha256, engineVersion: version.version, engineVersionEvidence: version.evidence, timestepSeconds };
}

function readContacts() {
  const count = Number(data?.ncon || 0);
  if (count > 512) throw new RangeError('OpenArm contact budget exceeded');
  const contacts = [];
  let readable = true;
  const collection = data?.contact;
  if (!collection) return { count, readable: count === 0, contacts };
  const force = new mujoco.DoubleBuffer(6);
  try {
    const available = typeof collection.size === 'function' ? Number(collection.size()) : count;
    if (available < count) readable = false;
    for (let index = 0; index < Math.min(count, available); index += 1) {
      const contact = typeof collection.get === 'function' ? collection.get(index) : collection[index];
      if (!contact) { readable = false; continue; }
      try {
        const geom1 = Number(contact.geom?.[0] ?? contact.geom1 ?? -1);
        const geom2 = Number(contact.geom?.[1] ?? contact.geom2 ?? -1);
        mujoco.mj_contactForce(model, data, index, force);
        contacts.push({ geom1, geom2, geom1Name: nameForGeom(geom1), geom2Name: nameForGeom(geom2), distanceM: Number(contact.dist ?? 0), normalForceN: Math.max(0, Number(force.GetView()[0])), positionM: Array.from(contact.pos || []).slice(0, 3) });
      } finally { contact.delete?.(); }
    }
  } finally { force.delete(); collection.delete?.(); }
  return { count, readable, contacts };
}

function positionActuatorForJoint(jointId, { required = false } = {}) {
  const matches = [...actuatorState.values()].filter((actuator) => actuator.jointId === jointId && actuator.command === 'position-rad');
  if (matches.length > 1 || (required && matches.length !== 1)) throw new Error(`Joint ${jointId} does not have exactly one position actuator`);
  return matches[0] || null;
}
function actuatorForJoint(jointId) { return positionActuatorForJoint(jointId, { required: true }); }
function actuatorEffortNm(actuator) { const value = Number(data?.actuator_force?.[actuator?.id]); return Number.isFinite(value) ? value : null; }
function observation() {
  if (!model || !data || !descriptor || !modelInfo) return { simulationTime: 0, model: null, engine: null, joints: {}, bodies: {}, contactCount: 0, contactsReadable: false, contacts: [] };
  const joints = {};
  for (const [name, joint] of jointState) {
    const actuator = positionActuatorForJoint(name);
    joints[name] = { positionRad: Number(data.qpos[joint.qpos]), velocityRadS: Number(data.qvel[joint.dof]), targetRad: actuator ? servo?.target(name) ?? Number(data.ctrl[actuator.id]) : null, referenceRad: actuator ? servo?.reference(name) ?? Number(data.ctrl[actuator.id]) : null, actuatorTargetRad: actuator ? Number(data.ctrl[actuator.id]) : null, effortNm: actuator ? actuatorEffortNm(actuator) : null, controlRangeRad: actuator ? [...actuator.controlRange] : null, jointRangeRad: [...joint.range] };
  }
  const bodies = {};
  for (const [name, body] of bodyState) {
    const posOffset = body.id * 3;
    const quatOffset = body.id * 4;
    const record = {
      frame: 'mujoco_world',
      centerOfMassM: Array.from(data.xipos.slice(posOffset, posOffset + 3)),
      positionM: [Number(data.xpos[posOffset]), Number(data.xpos[posOffset + 1]), Number(data.xpos[posOffset + 2])],
      quaternionWxyz: [Number(data.xquat[quatOffset]), Number(data.xquat[quatOffset + 1]), Number(data.xquat[quatOffset + 2]), Number(data.xquat[quatOffset + 3])],
    };
    if (body.freeDof != null) {
      record.linearVelocityFrame = 'mujoco_world'; record.angularVelocityFrame = 'body_local';
      record.linearVelocityMS = [Number(data.qvel[body.freeDof]), Number(data.qvel[body.freeDof + 1]), Number(data.qvel[body.freeDof + 2])];
      record.angularVelocityRadS = [Number(data.qvel[body.freeDof + 3]), Number(data.qvel[body.freeDof + 4]), Number(data.qvel[body.freeDof + 5])];
    }
    bodies[name] = record;
  }
  const contactState = readContacts();
  return { simulationTime: Number(data.time || 0), model: { id: descriptor.modelId || descriptor.id, asset: descriptor.asset, sha256: modelInfo.modelSha256 }, engine: { version: modelInfo.engineVersion, versionEvidence: modelInfo.engineVersionEvidence, timestepSeconds: modelInfo.timestepSeconds }, joints, bodies, contactCount: contactState.count, contactsReadable: contactState.readable, contacts: contactState.contacts, openarm: { sceneMode: descriptor.sceneMode || 'baseline', pinchReferences: Object.fromEntries(['left', 'right'].map(side => { const site = idFor('mjOBJ_SITE', `${side}_pinch_reference`); const ee = bodies[`openarm_${side}_ee_base_link`]; return [side, { frame: 'mujoco_world', positionM: Array.from(data.site_xpos.slice(site * 3, site * 3 + 3)), quaternionWxyz: ee.quaternionWxyz }]; })), observationPhase: 'post-step forward dynamics; all quantities at simulationTime', controlProfile: OPENARM_CONTROL_PROFILE, motionPlan, equipment: equipment?.records || [], equipmentJoints: (equipment?.passiveJoints || []).map(j => { const id = idFor('mjOBJ_JOINT', j.id); const q = Number(data.qpos[model.jnt_qposadr[id]]); return { ...j, position: q, velocity: Number(data.qvel[model.jnt_dofadr[id]]), pressed: q >= j.pressedThresholdM }; }) }, setupLog: descriptor.equipment?.length || descriptor.sceneMode === 'blank' || descriptor.generalScene ? [{ type: 'explicit-workcell-compile-and-reset', simulationTimeSeconds: 0, sceneMode: descriptor.sceneMode || 'baseline', equipmentIds: (descriptor.generalScene?.objects || descriptor.equipment || []).map(e => e.id), modelSha256: modelInfo.modelSha256 }] : [] };
}

function applyDeclaredInitialState() {
  if (!model || !data || !descriptor) throw new Error('No MuJoCo model is loaded');
  mujoco.mj_resetData(model, data);
  for (const [jointId, raw] of Object.entries(descriptor.initialJointPositionsRad || {})) {
    const joint = jointState.get(jointId);
    if (!joint) throw new Error(`Initial state references unknown joint ${jointId}`);
    const value = Number(raw);
    if (!Number.isFinite(value)) throw new Error(`Initial state for ${jointId} must be finite radians`);
    if (joint.range.every(Number.isFinite) && (value < joint.range[0] || value > joint.range[1])) throw new RangeError(`Initial state ${value} is outside joint ${jointId} range ${joint.range[0]}..${joint.range[1]}`);
    const actuator = positionActuatorForJoint(jointId);
    if (actuator && (value < actuator.controlRange[0] || value > actuator.controlRange[1])) throw new RangeError(`Initial state ${value} is outside actuator ${actuator.id} control range ${actuator.controlRange[0]}..${actuator.controlRange[1]}`);
    data.qpos[joint.qpos] = value;
    if (actuator) data.ctrl[actuator.id] = value;
  }
  for (const coupling of couplingState.values()) {
    const driverValue = Number(data.qpos[coupling.driver.qpos]);
    const followerValue = driverValue * Number(coupling.multiplier ?? 1) + Number(coupling.offsetRad ?? 0);
    if (!Number.isFinite(followerValue)) throw new Error(`Mechanical coupling ${coupling.id} produced a non-finite setup value`);
    if (coupling.range.every(Number.isFinite) && (followerValue < coupling.range[0] || followerValue > coupling.range[1])) throw new RangeError(`Mechanical coupling ${coupling.id} setup value ${followerValue} is outside follower range ${coupling.range[0]}..${coupling.range[1]}`);
    data.qpos[coupling.qpos] = followerValue;
  }
  paused = false;
  mujoco.mj_forward(model, data);
  servo = new OpenArmServo(model, data, actuatorState, jointState);
  servo.apply();
  mujoco.mj_forward(model, data);
  motionPlan = null;
  return observation();
}
function disposeModel() { try { data?.delete?.(); } catch {} try { model?.delete?.(); } catch {} data = null; model = null; descriptor = null; jointState = new Map(); couplingState = new Map(); actuatorState = new Map(); bodyState = new Map(); modelInfo = null; paused = false; servo = null; equipment = null; motionPlan = null; }
async function load(modelPackage) {
  if (!modelPackage?.id || !modelPackage?.asset || !modelPackage?.sha256 || !Array.isArray(modelPackage.joints) || !Array.isArray(modelPackage.actuators)) throw new Error('Worker requires a validated registered model package descriptor');
  const modelUrl = validatedModelUrl(modelPackage.asset);
  const mj = await ensureMuJoCo();
  disposeModel();
  let xml = await fetch(modelUrl, { cache: 'no-store' }).then((response) => { if (!response.ok) throw new Error(`MuJoCo model returned HTTP ${response.status}`); return response.text(); });
  const baseHash = await sha256Text(xml);
  if (baseHash !== (modelPackage.baseSha256 || modelPackage.sha256)) throw new Error(`Base model SHA-256 mismatch for ${modelPackage.id}`);
  const sceneMode = validateOpenArmSceneMode(modelPackage.sceneMode);
  if (modelPackage.generalScene) {
    if (sceneMode !== 'authored') throw new Error('General SceneSpec requires authored mode');
    equipment = appendGeneralSceneXml(xml, modelPackage.generalScene);
    if (await sha256Text(JSON.stringify(equipment.spec)) !== modelPackage.sceneSpecSha256) throw new Error('Authored SceneSpec checksum mismatch');
    xml = equipment.xml;
  } else {
  if (sceneMode === 'authored') throw new Error('Authored mode requires a validated SceneSpec');
  xml = workcellBaseXml(xml, sceneMode);
  if (modelPackage.equipment?.length || sceneMode === 'blank') {
    const normalized = validateOpenArmEquipment(modelPackage.equipment || []);
    equipment = appendEquipmentXml(xml, normalized);
    xml = equipment.xml;
  }
  }
  const modelSha256 = await sha256Text(xml);
  if (modelSha256 !== modelPackage.sha256) throw new Error(`Model SHA-256 mismatch for ${modelPackage.id}`);
  model = mj.from_xml_string(xml);
  if (!model) throw new Error(`MuJoCo failed to compile ${modelPackage.id}`);
  data = new mj.MjData(model);
  descriptor = structuredClone(modelPackage);
  resolveModelAddresses(modelSha256);
  const initial = applyDeclaredInitialState();
  if (equipment) validateInitialEquipmentClearance();
  return initial;
}
function reset() { return applyDeclaredInitialState(); }
function integrateStep() {
  servo.apply();
  mujoco.mj_step(model, data);
  // Refresh on EVERY physics step, not only when sampled. Observation cadence
  // cannot change the controller/solver update sequence.
  mujoco.mj_forward(model, data);
  if (!Array.from(data.qpos).every(Number.isFinite) || !Array.from(data.qvel).every(Number.isFinite)) throw new Error('OpenArm numerical state is non-finite');
}
function step(count = 1) {
  if (!model || !data) throw new Error('No MuJoCo model is loaded');
  if (!Number.isInteger(count) || count < 1 || count > MAX_STEP_BATCH) throw new RangeError(`step count must be an integer from 1 to ${MAX_STEP_BATCH}`);
  if (!paused) for (let index = 0; index < count; index += 1) integrateStep();
  return observation();
}
function validateInitialEquipmentClearance() {
  const names = Array.from({ length: Number(model.ngeom) }, (_, i) => nameForGeom(i));
  const owners = new Map(equipment.records.flatMap(e => e.geometryIds.map(name => [name, e.id])));
  const owner = name => owners.get(name);
  const segment = new Float64Array(6);
  for (let a = 0; a < names.length; a++) {
    const ownA = owner(names[a]); if (!ownA) continue;
    for (let b = 0; b < names.length; b++) {
      if (a === b || owner(names[b]) === ownA || (owner(names[b]) && b < a)) continue;
      const distance = Number(mujoco.mj_geomDistance(model, data, a, b, .001, segment));
      if (distance < -.0005) throw new RangeError(`Initial equipment overlaps ${names[a]} / ${names[b]} by ${(-distance * 1000).toFixed(2)} mm. The existing workcell was not changed.`);
    }
  }
}
// One request advances many real MuJoCo steps and returns the ground-truth observations
// captured at a declared step cadence. Sampling is a pure, deterministic function of the
// requested step count: the worker never derives task state, and every returned entry is
// an ordinary observation() of the actual MuJoCo state at that step.
function stepSampled(count = 1, sampleEverySteps = 1) {
  if (!model || !data) throw new Error('No MuJoCo model is loaded');
  if (!Number.isInteger(count) || count < 1 || count > MAX_STEP_BATCH) throw new RangeError(`step count must be an integer from 1 to ${MAX_STEP_BATCH}`);
  if (!Number.isInteger(sampleEverySteps) || sampleEverySteps < 1) throw new RangeError('sampleEverySteps must be a positive integer');
  if (paused) return { observations: [observation()], executedSteps: 0, sampleEverySteps };
  const expected = sampledObservationCount(count, sampleEverySteps);
  if (expected > MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE) throw new RangeError(`A ${count}-step advance sampled every ${sampleEverySteps} steps needs ${expected} observations; this worker returns at most ${MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE}`);
  const observations = [];
  for (let index = 1; index <= count; index += 1) {
    integrateStep();
    if (index === count || index % sampleEverySteps === 0) observations.push(observation());
  }
  return { observations, executedSteps: count, sampleEverySteps };
}
function validatedJointTarget(jointId, targetValue) {
  if (!jointId || !jointState.has(jointId)) throw new Error(`Unknown declared joint ${jointId || '<missing>'}`);
  const actuator = actuatorForJoint(jointId);
  const target = targetValue;
  if (typeof target !== 'number' || !Number.isFinite(target)) throw new TypeError(`targetRad for ${jointId} must be finite`);
  const [minimum, maximum] = actuator.controlRange;
  if (target < minimum || target > maximum) throw new RangeError(`targetRad ${target} is outside the actuator control range ${minimum}..${maximum}`);
  const jointRange = jointState.get(jointId).range;
  if (jointRange.every(Number.isFinite) && (target < jointRange[0] || target > jointRange[1])) throw new RangeError(`targetRad ${target} is outside joint ${jointId} range ${jointRange[0]}..${jointRange[1]}`);
  return { actuator, target };
}
function command(payload = {}) {
  let targets;
  if (payload.type === 'set_joint_target') {
    const jointId = payload.jointId || (descriptor.joints.length === 1 ? descriptor.joints[0].id : null);
    if (!jointId) throw new Error('set_joint_target requires a declared jointId for this model');
    targets = { [jointId]: payload.targetRad };
  } else if (payload.type === 'set_joint_targets' || payload.type === 'move_joint_targets') {
    if (!payload.targetsRad || typeof payload.targetsRad !== 'object' || Array.isArray(payload.targetsRad) || !Object.keys(payload.targetsRad).length) throw new Error('set_joint_targets requires a non-empty targetsRad object');
    targets = payload.targetsRad;
  } else if (payload.type === 'set_tool_target') {
    const solution = solveOpenArmIK(mujoco, model, data, jointState, payload);
    targets = solution.targetsRad;
    const validated = Object.fromEntries(Object.entries(targets).map(([id, value]) => [id, validatedJointTarget(id, value).target]));
    const timing = servo.request(validated, payload.durationSeconds ?? null);
    motionPlan = { ...solution, ...timing };
    return observation();
  } else throw new Error(`Unsupported physical command: ${payload.type}`);
  const validated = Object.entries(targets).map(([jointId, targetRad]) => [jointId, validatedJointTarget(jointId, targetRad)]);
  const timing = servo.request(Object.fromEntries(validated.map(([id, { target }]) => [id, target])), payload.type === 'move_joint_targets' ? payload.durationSeconds : null);
  motionPlan = { type: payload.type, ...timing, collisionFreePath: false }; 
  return observation();
}

self.onmessage = async (event) => {
  const { id, op, payload } = event.data || {};
  try {
    let result;
    if (op === 'load') result = await load(payload?.modelPackage);
    else if (op === 'reset') result = reset();
    else if (op === 'step') result = step(payload?.count);
    else if (op === 'stepSampled') result = stepSampled(payload?.count, payload?.sampleEverySteps);
    else if (op === 'observe') result = observation();
    else if (op === 'command') result = command(payload);
    else if (op === 'pause') { paused = true; result = observation(); }
    else if (op === 'resume') { paused = false; result = observation(); }
    else if (op === 'dispose') { disposeModel(); result = true; }
    else throw new Error(`Unknown worker operation: ${op}`);
    reply(id, true, result);
  } catch (error) { reply(id, false, null, String(error?.stack || error?.message || error)); }
};
