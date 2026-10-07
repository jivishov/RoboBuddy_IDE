import loadMujoco from '../../assets/microduck/runtime/mujoco/mujoco.js';
import { MAX_ADVANCE_STEPS_PER_REQUEST, MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE, sampledObservationCount } from './backend-contract.js';

// LeKiwi Phase 5B authoritative MuJoCo worker.
//
// It extends the shared worker pattern with exactly what the selected LeKiwi design needs:
//   * bounded velocity actuators for the three Kiwi drive wheels, alongside the existing
//     bounded position actuators for the SO-ARM101 arm;
//   * free-body observation for the chassis and the free payload;
//   * passive roller joints reported as ordinary observed joints.
// It never writes base pose, base velocity, or object state outside the declared setup/reset
// path, and it derives no task state.

const MUJOCO_BASE_URL = new URL('../../assets/microduck/runtime/mujoco/', import.meta.url);
const EXPECTED_MUJOCO_VERSION = '3.11.0';
const MAX_STEP_BATCH = MAX_ADVANCE_STEPS_PER_REQUEST;
const MODEL_ASSET_RE = /^models\/[A-Za-z0-9._/-]+\.xml$/;
const MODEL_VALUE_TOLERANCE = 1e-9;
const INTEGRATOR_CODES = Object.freeze({ Euler: 0, RK4: 1, implicit: 2, implicitfast: 3 });
const POSITION_COMMAND = 'position-rad';
const VELOCITY_COMMAND = 'velocity-rad-s';

let mujoco = null;
let model = null;
let data = null;
let paused = false;
let descriptor = null;
let jointState = new Map();
let actuatorState = new Map();
let bodyState = new Map();
let modelInfo = null;

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
  jointState = new Map(); actuatorState = new Map(); bodyState = new Map();
  for (const joint of descriptor.joints) {
    const id = idFor('mjOBJ_JOINT', joint.id);
    const qpos = Number(model.jnt_qposadr[id]);
    const dof = Number(model.jnt_dofadr[id]);
    if (![qpos, dof].every(Number.isInteger)) throw new Error(`MuJoCo address table is invalid for joint ${joint.id}`);
    const range = [Number(model.jnt_range[id * 2]), Number(model.jnt_range[id * 2 + 1])];
    const axis = [Number(model.jnt_axis[id * 3]), Number(model.jnt_axis[id * 3 + 1]), Number(model.jnt_axis[id * 3 + 2])];
    if (joint.rangeRad) assertNumericArrayClose(range, joint.rangeRad, `Joint ${joint.id} range`);
    if (joint.axis) assertNumericArrayClose(axis, joint.axis, `Joint ${joint.id} axis`);
    jointState.set(joint.id, { id, qpos, dof, range, axis, passive: Boolean(joint.passive) });
  }
  for (const actuator of descriptor.actuators) {
    const id = idFor('mjOBJ_ACTUATOR', actuator.id);
    const joint = jointState.get(actuator.jointId);
    if (!joint) throw new Error(`Actuator ${actuator.id} references unknown descriptor joint ${actuator.jointId}`);
    if (joint.passive) throw new Error(`Actuator ${actuator.id} targets a joint declared passive`);
    if (model.actuator_trnid?.length && Number(model.actuator_trnid[id * 2]) !== joint.id) throw new Error(`Actuator ${actuator.id} is not mapped to joint ${actuator.jointId}`);
    const controlRange = [Number(model.actuator_ctrlrange[id * 2]), Number(model.actuator_ctrlrange[id * 2 + 1])];
    const forceRange = [Number(model.actuator_forcerange[id * 2]), Number(model.actuator_forcerange[id * 2 + 1])];
    if (actuator.command !== POSITION_COMMAND && actuator.command !== VELOCITY_COMMAND) throw new Error(`Actuator ${actuator.id} declares unsupported command ${actuator.command}`);
    if (actuator.controlRangeRad) assertNumericArrayClose(controlRange, actuator.controlRangeRad, `Actuator ${actuator.id} control range`);
    if (actuator.controlRangeRadS) assertNumericArrayClose(controlRange, actuator.controlRangeRadS, `Actuator ${actuator.id} velocity control range`);
    if (actuator.forceRangeNm) assertNumericArrayClose(forceRange, actuator.forceRangeNm, `Actuator ${actuator.id} force range`);
    // Every actuator must be bounded in both command and effort: an unlimited motor would let
    // requested motion appear without a defensible physical cost.
    if (!controlRange.every(Number.isFinite) || controlRange[0] >= controlRange[1]) throw new Error(`Actuator ${actuator.id} has no finite control range`);
    if (!forceRange.every(Number.isFinite) || forceRange[0] >= forceRange[1]) throw new Error(`Actuator ${actuator.id} has no finite force range`);
    actuatorState.set(actuator.id, { ...actuator, id, controlRange, forceRange });
  }
  for (const body of descriptor.bodies || []) {
    const id = idFor('mjOBJ_BODY', body.id);
    let freeDof = null;
    let freeQpos = null;
    if (body.freeJointId) {
      const freeJointId = idFor('mjOBJ_JOINT', body.freeJointId);
      if (Number(model.jnt_type[freeJointId]) !== 0) throw new Error(`Body ${body.id} joint ${body.freeJointId} is not a free joint`);
      freeDof = Number(model.jnt_dofadr[freeJointId]);
      freeQpos = Number(model.jnt_qposadr[freeJointId]);
      if (!Number.isInteger(freeDof) || freeDof < 0) throw new Error(`Body ${body.id} free joint ${body.freeJointId} has no valid dof address`);
    }
    bodyState.set(body.id, { id, freeDof, freeQpos });
  }
  // No equality constraint may exist in a LeKiwi package: there is no source mechanical
  // coupling to model, so any constraint would be a hidden attachment.
  const equalityCount = Number(model.neq ?? 0);
  if (equalityCount !== 0) throw new Error(`LeKiwi models must declare no equality constraints; compiled model has ${equalityCount}`);

  const timestepSeconds = Number(model.opt?.timestep);
  if (!Number.isFinite(timestepSeconds) || Math.abs(timestepSeconds - Number(descriptor.physics.timestepSeconds)) > 1e-12) throw new Error(`Unexpected timestep ${timestepSeconds}; expected ${descriptor.physics.timestepSeconds}`);
  const expectedIntegrator = INTEGRATOR_CODES[descriptor.physics.integrator];
  const compiledIntegrator = Number(model.opt?.integrator);
  if (!Number.isInteger(expectedIntegrator) || compiledIntegrator !== expectedIntegrator) throw new Error(`Unexpected integrator ${compiledIntegrator}; expected ${descriptor.physics.integrator} (${expectedIntegrator})`);
  const compiledIterations = Number(model.opt?.iterations);
  if (descriptor.physics.iterations != null && compiledIterations !== Number(descriptor.physics.iterations)) throw new Error(`Unexpected solver iterations ${compiledIterations}; expected ${descriptor.physics.iterations}`);
  const compiledLsIterations = Number(model.opt?.ls_iterations ?? model.opt?.lsIterations);
  if (descriptor.physics.lsIterations != null && compiledLsIterations !== Number(descriptor.physics.lsIterations)) throw new Error(`Unexpected solver line-search iterations ${compiledLsIterations}; expected ${descriptor.physics.lsIterations}`);
  const version = runtimeVersionEvidence();
  modelInfo = { modelSha256, engineVersion: version.version, engineVersionEvidence: version.evidence, timestepSeconds };
}

function readContacts() {
  const count = Number(data?.ncon || 0);
  const contacts = [];
  let readable = true;
  const collection = data?.contact;
  if (!collection) return { count, readable: count === 0, contacts };
  try {
    const available = typeof collection.size === 'function' ? Number(collection.size()) : count;
    if (available < count) readable = false;
    for (let index = 0; index < Math.min(count, available); index += 1) {
      const contact = typeof collection.get === 'function' ? collection.get(index) : collection[index];
      if (!contact) { readable = false; continue; }
      try {
        const geom1 = Number(contact.geom?.[0] ?? contact.geom1 ?? -1);
        const geom2 = Number(contact.geom?.[1] ?? contact.geom2 ?? -1);
        contacts.push({ geom1, geom2, geom1Name: nameForGeom(geom1), geom2Name: nameForGeom(geom2), distanceM: Number(contact.dist ?? 0) });
      } finally { contact.delete?.(); }
    }
  } finally { collection.delete?.(); }
  return { count, readable, contacts };
}

function actuatorForJoint(jointId, command) {
  const matches = [...actuatorState.values()].filter((actuator) => actuator.jointId === jointId && (!command || actuator.command === command));
  if (matches.length > 1) throw new Error(`Joint ${jointId} has more than one matching actuator`);
  return matches[0] || null;
}
function actuatorEffortNm(actuator) { const value = Number(data?.actuator_force?.[actuator?.id]); return Number.isFinite(value) ? value : null; }

function observation() {
  if (!model || !data || !descriptor || !modelInfo) return { simulationTime: 0, model: null, engine: null, joints: {}, bodies: {}, contactCount: 0, contactsReadable: false, contacts: [] };
  const joints = {};
  for (const [name, joint] of jointState) {
    const actuator = actuatorForJoint(name);
    const record = {
      positionRad: Number(data.qpos[joint.qpos]),
      velocityRadS: Number(data.qvel[joint.dof]),
      targetRad: null,
      targetVelocityRadS: null,
      effortNm: actuator ? actuatorEffortNm(actuator) : null,
      controlRangeRad: null,
      controlRangeRadS: null,
      forceRangeNm: actuator ? [...actuator.forceRange] : null,
      jointRangeRad: [...joint.range],
      passive: Boolean(joint.passive),
    };
    if (actuator?.command === POSITION_COMMAND) {
      record.targetRad = Number(data.ctrl[actuator.id]);
      record.controlRangeRad = [...actuator.controlRange];
    } else if (actuator?.command === VELOCITY_COMMAND) {
      record.targetVelocityRadS = Number(data.ctrl[actuator.id]);
      record.controlRangeRadS = [...actuator.controlRange];
    }
    joints[name] = record;
  }
  const bodies = {};
  for (const [name, body] of bodyState) {
    const posOffset = body.id * 3;
    const quatOffset = body.id * 4;
    const record = {
      frame: 'mujoco_world',
      positionM: [Number(data.xpos[posOffset]), Number(data.xpos[posOffset + 1]), Number(data.xpos[posOffset + 2])],
      quaternionWxyz: [Number(data.xquat[quatOffset]), Number(data.xquat[quatOffset + 1]), Number(data.xquat[quatOffset + 2]), Number(data.xquat[quatOffset + 3])],
    };
    if (body.freeDof != null) {
      record.linearVelocityMS = [Number(data.qvel[body.freeDof]), Number(data.qvel[body.freeDof + 1]), Number(data.qvel[body.freeDof + 2])];
      record.angularVelocityRadS = [Number(data.qvel[body.freeDof + 3]), Number(data.qvel[body.freeDof + 4]), Number(data.qvel[body.freeDof + 5])];
      record.freeBody = true;
    }
    bodies[name] = record;
  }
  const contactState = readContacts();
  return {
    simulationTime: Number(data.time || 0),
    model: { id: descriptor.modelId || descriptor.id, asset: descriptor.asset, sha256: modelInfo.modelSha256 },
    engine: { version: modelInfo.engineVersion, versionEvidence: modelInfo.engineVersionEvidence, timestepSeconds: modelInfo.timestepSeconds },
    joints, bodies,
    contactCount: contactState.count, contactsReadable: contactState.readable, contacts: contactState.contacts,
  };
}

function applyDeclaredInitialState() {
  if (!model || !data || !descriptor) throw new Error('No MuJoCo model is loaded');
  mujoco.mj_resetData(model, data);
  for (const [jointId, raw] of Object.entries(descriptor.initialJointPositionsRad || {})) {
    const joint = jointState.get(jointId);
    if (!joint) throw new Error(`Initial state references unknown joint ${jointId}`);
    if (joint.passive) throw new Error(`Initial state may not assign passive joint ${jointId}`);
    const value = Number(raw);
    if (!Number.isFinite(value)) throw new Error(`Initial state for ${jointId} must be finite radians`);
    if (joint.range.every(Number.isFinite) && (value < joint.range[0] || value > joint.range[1])) throw new RangeError(`Initial state ${value} is outside joint ${jointId} range ${joint.range[0]}..${joint.range[1]}`);
    const actuator = actuatorForJoint(jointId, POSITION_COMMAND);
    if (actuator && (value < actuator.controlRange[0] || value > actuator.controlRange[1])) throw new RangeError(`Initial state ${value} is outside actuator ${actuator.id} control range`);
    data.qpos[joint.qpos] = value;
    if (actuator) data.ctrl[actuator.id] = value;
  }
  // Wheel velocity servos always start commanded to a full stop.
  for (const actuator of actuatorState.values()) if (actuator.command === VELOCITY_COMMAND) data.ctrl[actuator.id] = 0;
  paused = false;
  mujoco.mj_forward(model, data);
  return observation();
}

function disposeModel() { try { data?.delete?.(); } catch {} try { model?.delete?.(); } catch {} data = null; model = null; descriptor = null; jointState = new Map(); actuatorState = new Map(); bodyState = new Map(); modelInfo = null; paused = false; }

async function load(modelPackage) {
  if (!modelPackage?.id || !modelPackage?.asset || !modelPackage?.sha256 || !Array.isArray(modelPackage.joints) || !Array.isArray(modelPackage.actuators)) throw new Error('Worker requires a validated registered model package descriptor');
  const modelUrl = validatedModelUrl(modelPackage.asset);
  const mj = await ensureMuJoCo();
  disposeModel();
  const xml = await fetch(modelUrl, { cache: 'no-store' }).then((response) => { if (!response.ok) throw new Error(`MuJoCo model returned HTTP ${response.status}`); return response.text(); });
  const modelSha256 = await sha256Text(xml);
  if (modelSha256 !== modelPackage.sha256) throw new Error(`Model SHA-256 mismatch for ${modelPackage.id}`);
  model = mj.from_xml_string(xml);
  if (!model) throw new Error(`MuJoCo failed to compile ${modelPackage.id}`);
  data = new mj.MjData(model);
  if (!data) throw new Error(`MuJoCo failed to allocate data for ${modelPackage.id}`);
  descriptor = structuredClone(modelPackage);
  resolveModelAddresses(modelSha256);
  return applyDeclaredInitialState();
}

function reset() { return applyDeclaredInitialState(); }
function step(count = 1) {
  if (!model || !data) throw new Error('No MuJoCo model is loaded');
  if (paused) return observation();
  if (!Number.isInteger(count) || count < 1 || count > MAX_STEP_BATCH) throw new RangeError(`step count must be an integer from 1 to ${MAX_STEP_BATCH}`);
  for (let index = 0; index < count; index += 1) mujoco.mj_step(model, data);
  return observation();
}
function stepSampled(count = 1, sampleEverySteps = 1) {
  if (!model || !data) throw new Error('No MuJoCo model is loaded');
  if (!Number.isInteger(count) || count < 1 || count > MAX_STEP_BATCH) throw new RangeError(`step count must be an integer from 1 to ${MAX_STEP_BATCH}`);
  if (!Number.isInteger(sampleEverySteps) || sampleEverySteps < 1) throw new RangeError('sampleEverySteps must be a positive integer');
  if (paused) return { observations: [observation()], executedSteps: 0, sampleEverySteps };
  const expected = sampledObservationCount(count, sampleEverySteps);
  if (expected > MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE) throw new RangeError(`A ${count}-step advance sampled every ${sampleEverySteps} steps needs ${expected} observations; this worker returns at most ${MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE}`);
  const observations = [];
  for (let index = 1; index <= count; index += 1) {
    mujoco.mj_step(model, data);
    if (index === count || index % sampleEverySteps === 0) observations.push(observation());
  }
  return { observations, executedSteps: count, sampleEverySteps };
}

function validatedTarget(jointId, rawValue, command) {
  const joint = jointState.get(jointId);
  if (!joint) throw new Error(`Unknown declared joint ${jointId || '<missing>'}`);
  if (joint.passive) throw new Error(`Joint ${jointId} is a passive source mechanism, not a command surface`);
  const actuator = actuatorForJoint(jointId, command);
  if (!actuator) throw new Error(`Joint ${jointId} has no ${command} actuator`);
  const target = Number(rawValue);
  if (!Number.isFinite(target)) throw new TypeError(`Target for ${jointId} must be finite`);
  const [minimum, maximum] = actuator.controlRange;
  if (target < minimum || target > maximum) throw new RangeError(`Target ${target} is outside the ${actuator.id} control range ${minimum}..${maximum}`);
  if (command === POSITION_COMMAND && joint.range.every(Number.isFinite) && (target < joint.range[0] || target > joint.range[1])) {
    throw new RangeError(`Target ${target} is outside joint ${jointId} range ${joint.range[0]}..${joint.range[1]}`);
  }
  return { actuator, target };
}

function command(payload = {}) {
  if (!model || !data || !descriptor || !modelInfo) throw new Error('No MuJoCo model is loaded');
  const applied = [];
  if (payload.type === 'set_joint_targets') {
    if (!payload.targetsRad || typeof payload.targetsRad !== 'object' || Array.isArray(payload.targetsRad) || !Object.keys(payload.targetsRad).length) throw new Error('set_joint_targets requires a non-empty targetsRad object');
    for (const [jointId, value] of Object.entries(payload.targetsRad)) applied.push(validatedTarget(jointId, value, POSITION_COMMAND));
  } else if (payload.type === 'set_wheel_velocity_targets') {
    if (!payload.targetsRadS || typeof payload.targetsRadS !== 'object' || Array.isArray(payload.targetsRadS) || !Object.keys(payload.targetsRadS).length) throw new Error('set_wheel_velocity_targets requires a non-empty targetsRadS object');
    for (const [jointId, value] of Object.entries(payload.targetsRadS)) applied.push(validatedTarget(jointId, value, VELOCITY_COMMAND));
  } else if (payload.type === 'set_mixed_targets') {
    const positions = payload.targetsRad || {};
    const velocities = payload.targetsRadS || {};
    if (!Object.keys(positions).length && !Object.keys(velocities).length) throw new Error('set_mixed_targets requires at least one target');
    for (const [jointId, value] of Object.entries(positions)) applied.push(validatedTarget(jointId, value, POSITION_COMMAND));
    for (const [jointId, value] of Object.entries(velocities)) applied.push(validatedTarget(jointId, value, VELOCITY_COMMAND));
  } else {
    throw new Error(`Unsupported physical command: ${payload.type}`);
  }
  for (const { actuator, target } of applied) data.ctrl[actuator.id] = target;
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
