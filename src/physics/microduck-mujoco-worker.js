import loadMujoco from '../../assets/microduck/runtime/mujoco/mujoco.js';
import { MAX_ADVANCE_STEPS_PER_REQUEST, MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE, sampledObservationCount } from './backend-contract.js';
import { projectedGravityFromQuaternion } from './microduck-controller.js';
import { MICRODUCK_NOMINAL_FIRMWARE_GAIN, microDuckKpForFirmwareGain } from './microduck-model-package.js';
import {
  MICRODUCK_BAM_M6,
  MICRODUCK_BAM_REVISION,
  MICRODUCK_BAM_VERSION,
  MICRODUCK_DEPLOYMENT_PLANT_PROFILE,
  MicroDuckDeploymentImuFilter,
  MicroDuckTargetDelay,
  clamp,
  microDuckBamForceCeilingNm,
  microDuckBamFrictionLossNm,
  microDuckBamMotorTorqueNm,
  microDuckBamSmallSignalKpNmRad,
} from './microduck-bam-plant.js';

// MicroDuck authoritative browser plant.
//
// The source XML carries a convenient identified position-actuator fallback, but the policies
// were not trained against that fallback. At load time this worker converts those actuators
// to torque motors and applies the public BAM M6 XL330 model used by microduck_rl. Ordinary
// control still accepts bounded joint-position targets; BAM turns them into voltage, motor
// torque, load-dependent/Stribeck friction and back-EMF at every 5 ms physics step.
//
// The default browser workspace is the deterministic deployment-reference path: deployed
// gain schedule + deployed median-of-three IMU preprocessing, with the BAM hardware model at
// the source CPU regression's nominal 7.4 V / 0.1 V-per-Nm sag condition. The stochastic
// training distributions live in microduck-bam-plant.js and are validated separately; they
// are not silently injected into an interactive deployment preview.

const MUJOCO_BASE_URL = new URL('../../assets/microduck/runtime/mujoco/', import.meta.url);
const EXPECTED_MUJOCO_VERSION = '3.11.0';
const MAX_STEP_BATCH = MAX_ADVANCE_STEPS_PER_REQUEST;
const MODEL_ASSET_RE = /^models\/[A-Za-z0-9._/-]+\.xml$/;
const MODEL_VALUE_TOLERANCE = 1e-9;
const INTEGRATOR_CODES = Object.freeze({ Euler: 0, RK4: 1, implicit: 2, implicitfast: 3 });
const POSITION_COMMAND = 'position-rad';
const TRUNK_BODY = 'trunk_base';
const GYRO_SENSOR = 'imu_ang_vel';
const FOOT_GEOMS = Object.freeze(['left_foot_collision', 'right_foot_collision']);
const FLOOR_GEOM = 'microduck_floor';
const BALL_GEOM = 'microduck_ball_geom';
const MAX_SETUP_EVENTS = 64;
const CONTROL_DECIMATION = 4;

let mujoco = null;
let model = null;
let data = null;
let paused = false;
let descriptor = null;
let jointState = new Map();
let actuatorState = new Map();
let bodyState = new Map();
let geomIds = new Map();
let modelInfo = null;
let actuationEnabled = true;
let setupLog = [];
let appliedFirmwareGain = MICRODUCK_NOMINAL_FIRMWARE_GAIN;

// BAM runtime state. Targets stay in radians here; data.ctrl is torque after conversion.
let requestedTargets = [];
let delayedTargets = [];
let targetDelay = null;
let previousMotorTorqueNm = [];
let lastMotorTorqueNm = [];
let lastFrictionLossNm = [];
let effectiveVinV = MICRODUCK_DEPLOYMENT_PLANT_PROFILE.nominalVinV;
let physicsStepsSinceImuSample = 0;
let imuFilter = new MicroDuckDeploymentImuFilter();

function reply(id, ok, payload = null, error = null) { postMessage({ id, ok, payload, error }); }
async function ensureMuJoCo() { if (mujoco) return mujoco; mujoco = await loadMujoco({ locateFile: (path) => new URL(path, MUJOCO_BASE_URL).href }); return mujoco; }
async function sha256Text(text) { const digest = await crypto.subtle.digest('SHA-256', new TextEncoder().encode(text)); return [...new Uint8Array(digest)].map((value) => value.toString(16).padStart(2, '0')).join(''); }
function enumValue(name) { const entry = mujoco?.mjtObj?.[name]; const value = entry && typeof entry === 'object' && 'value' in entry ? entry.value : entry; if (!Number.isInteger(Number(value))) throw new Error(`MuJoCo object enum ${name} is unavailable`); return Number(value); }
function constraintEnumValue(name) { const entry = mujoco?.mjtConstraint?.[name]; const value = entry && typeof entry === 'object' && 'value' in entry ? entry.value : entry; return Number.isInteger(Number(value)) ? Number(value) : null; }
function idFor(typeName, name) { const id = Number(mujoco.mj_name2id(model, enumValue(typeName), name)); if (!Number.isInteger(id) || id < 0) throw new Error(`Required MuJoCo ${typeName} named ${name} is missing`); return id; }
function optionalId(typeName, name) { try { const id = Number(mujoco.mj_name2id(model, enumValue(typeName), name)); return Number.isInteger(id) && id >= 0 ? id : null; } catch { return null; } }
function nameForGeom(id) { if (!Number.isInteger(id) || id < 0 || typeof mujoco?.mj_id2name !== 'function') return null; try { return mujoco.mj_id2name(model, enumValue('mjOBJ_GEOM'), id) || null; } catch { return null; } }
function runtimeVersionEvidence() { const version = typeof mujoco?.mj_versionString === 'function' ? String(mujoco.mj_versionString()) : EXPECTED_MUJOCO_VERSION; if (version !== EXPECTED_MUJOCO_VERSION) throw new Error(`Bundled MuJoCo runtime reports ${version}; expected ${EXPECTED_MUJOCO_VERSION}`); return { version, evidence: typeof mujoco?.mj_versionString === 'function' ? 'mj_versionString runtime introspection' : 'bundled asset manifest and repository hash gate' }; }
function validatedModelUrl(asset) { if (!MODEL_ASSET_RE.test(asset || '') || String(asset).includes('..')) throw new Error('Worker rejected non-registry model asset path'); const url = new URL(`../../${asset}`, import.meta.url); if (url.origin !== self.location.origin) throw new Error('Worker model asset must be same-origin'); return url.href; }
function assertNumericArrayClose(actual, expected, label, tolerance = MODEL_VALUE_TOLERANCE) { if (!Array.isArray(expected) || actual.length !== expected.length) throw new Error(`${label} descriptor shape mismatch`); for (let index = 0; index < actual.length; index += 1) { const delta = Math.abs(Number(actual[index]) - Number(expected[index])); if (!Number.isFinite(delta) || delta > tolerance) throw new Error(`${label} mismatch at index ${index}: compiled ${actual[index]}, descriptor ${expected[index]}`); } }
function logSetup(event) { setupLog.push(Object.freeze({ ...event, simulationTime: Number(data?.time || 0) })); if (setupLog.length > MAX_SETUP_EVENTS) setupLog.shift(); }

function actuatorForJoint(jointId) {
  const matches = [...actuatorState.values()].filter((actuator) => actuator.jointId === jointId);
  if (matches.length > 1) throw new Error(`Joint ${jointId} has more than one matching actuator`);
  return matches[0] || null;
}

function resolveModelAddresses(modelSha256) {
  jointState = new Map(); actuatorState = new Map(); bodyState = new Map(); geomIds = new Map();
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
    if (actuator.command !== POSITION_COMMAND) throw new Error(`Actuator ${actuator.id} declares unsupported command ${actuator.command}`);
    const controlRange = [Number(model.actuator_ctrlrange[id * 2]), Number(model.actuator_ctrlrange[id * 2 + 1])];
    const sourceForceRange = [Number(model.actuator_forcerange[id * 2]), Number(model.actuator_forcerange[id * 2 + 1])];
    if (actuator.controlRangeRad) assertNumericArrayClose(controlRange, actuator.controlRangeRad, `Actuator ${actuator.id} control range`);
    if (actuator.sourceForceRangeNm) assertNumericArrayClose(sourceForceRange, actuator.sourceForceRangeNm, `Actuator ${actuator.id} source force range`);
    else if (actuator.forceRangeNm) assertNumericArrayClose(sourceForceRange, actuator.forceRangeNm, `Actuator ${actuator.id} source force range`);
    if (!controlRange.every(Number.isFinite) || controlRange[0] >= controlRange[1]) throw new Error(`Actuator ${actuator.id} has no finite target range`);
    actuatorState.set(actuator.id, { ...actuator, id, joint, controlRange, sourceForceRange, forceRange: [...sourceForceRange] });
  }
  for (const body of descriptor.bodies || []) {
    const id = idFor('mjOBJ_BODY', body.id);
    let freeDof = null; let freeQpos = null;
    if (body.freeJointId) {
      const freeJointId = idFor('mjOBJ_JOINT', body.freeJointId);
      if (Number(model.jnt_type[freeJointId]) !== 0) throw new Error(`Body ${body.id} joint ${body.freeJointId} is not a free joint`);
      freeDof = Number(model.jnt_dofadr[freeJointId]); freeQpos = Number(model.jnt_qposadr[freeJointId]);
      if (!Number.isInteger(freeDof) || freeDof < 0) throw new Error(`Body ${body.id} free joint ${body.freeJointId} has no valid dof address`);
    }
    bodyState.set(body.id, { id, freeDof, freeQpos, freeJointId: body.freeJointId || null });
  }
  if (!bodyState.has(TRUNK_BODY) || bodyState.get(TRUNK_BODY).freeDof == null) throw new Error('MicroDuck packages must declare a free trunk body');
  for (const name of [...FOOT_GEOMS, FLOOR_GEOM]) geomIds.set(name, idFor('mjOBJ_GEOM', name));
  const ballGeom = optionalId('mjOBJ_GEOM', BALL_GEOM); if (ballGeom != null) geomIds.set(BALL_GEOM, ballGeom);
  if (Number(model.neq ?? 0) !== 0) throw new Error(`MicroDuck models must declare no equality constraints; compiled model has ${model.neq}`);
  const gyroSensor = idFor('mjOBJ_SENSOR', GYRO_SENSOR);
  const gyroAddress = Number(model.sensor_adr[gyroSensor]);
  if (!Number.isInteger(gyroAddress) || gyroAddress < 0) throw new Error('MicroDuck models must expose the source-named imu gyro sensor');
  const timestepSeconds = Number(model.opt?.timestep);
  if (!Number.isFinite(timestepSeconds) || Math.abs(timestepSeconds - Number(descriptor.physics.timestepSeconds)) > 1e-12) throw new Error(`Unexpected timestep ${timestepSeconds}; expected ${descriptor.physics.timestepSeconds}`);
  const expectedIntegrator = INTEGRATOR_CODES[descriptor.physics.integrator];
  if (!Number.isInteger(expectedIntegrator) || Number(model.opt?.integrator) !== expectedIntegrator) throw new Error(`Unexpected MuJoCo integrator; expected ${descriptor.physics.integrator}`);
  if (descriptor.physics.iterations != null && Number(model.opt?.iterations) !== Number(descriptor.physics.iterations)) throw new Error('Unexpected solver iterations');
  const compiledLsIterations = Number(model.opt?.ls_iterations ?? model.opt?.lsIterations);
  if (descriptor.physics.lsIterations != null && compiledLsIterations !== Number(descriptor.physics.lsIterations)) throw new Error('Unexpected solver line-search iterations');
  const version = runtimeVersionEvidence();
  modelInfo = { modelSha256, engineVersion: version.version, engineVersionEvidence: version.evidence, timestepSeconds, gyroAddress };
}

function configureBamMotorPlant() {
  if (!model || !data) throw new Error('No MuJoCo model is loaded');
  const gainWidth = Number(model.actuator_gainprm.length / Math.max(Number(model.nu || 1), 1));
  const biasWidth = Number(model.actuator_biasprm.length / Math.max(Number(model.nu || 1), 1));
  const forceCeiling = microDuckBamForceCeilingNm();
  for (const actuator of actuatorState.values()) {
    // Runtime equivalent of bam.mjlab.BamActuator.edit_spec: a position target is consumed by
    // BAM, while the compiled MuJoCo actuator itself becomes a unit-gain torque motor.
    for (let i = 0; i < gainWidth; i += 1) model.actuator_gainprm[actuator.id * gainWidth + i] = 0;
    for (let i = 0; i < biasWidth; i += 1) model.actuator_biasprm[actuator.id * biasWidth + i] = 0;
    model.actuator_gainprm[actuator.id * gainWidth] = 1;
    model.actuator_forcerange[actuator.id * 2] = -forceCeiling;
    model.actuator_forcerange[actuator.id * 2 + 1] = forceCeiling;
    model.dof_armature[actuator.joint.dof] = MICRODUCK_BAM_M6.armatureKgM2 * MICRODUCK_DEPLOYMENT_PLANT_PROFILE.armatureScale;
    // BAM edit_spec replaces the XML joint friction/damping. The dynamic BAM values are
    // written immediately before each mj_step; start from zero so the fallback servo plant
    // can never be double-counted during model setup.
    model.dof_frictionloss[actuator.joint.dof] = 0;
    model.dof_damping[actuator.joint.dof] = 0;
    actuator.forceRange = [-forceCeiling, forceCeiling];
  }
  if (typeof mujoco.mj_setConst === 'function') mujoco.mj_setConst(model, data);
}

function resetBamState() {
  const count = Number(model?.nu || 0);
  requestedTargets = new Array(count).fill(0);
  delayedTargets = new Array(count).fill(0);
  previousMotorTorqueNm = new Array(count).fill(0);
  lastMotorTorqueNm = new Array(count).fill(0);
  lastFrictionLossNm = new Array(count).fill(0);
  effectiveVinV = MICRODUCK_DEPLOYMENT_PLANT_PROFILE.nominalVinV;
  targetDelay = new MicroDuckTargetDelay({ minLag: 0, maxLag: 0 });
  appliedFirmwareGain = MICRODUCK_NOMINAL_FIRMWARE_GAIN;
  physicsStepsSinceImuSample = 0;
  imuFilter.reset();
}

function frictionConstraintForceForJoint(jointId) {
  const frictionCode = constraintEnumValue('mjCNSTR_FRICTION_DOF');
  if (frictionCode == null || !data?.efc_id || !data?.efc_type || !data?.efc_force) return 0;
  const count = Math.min(Number(data.nefc ?? data.efc_id.length ?? 0), data.efc_id.length || 0);
  let total = 0;
  for (let i = 0; i < count; i += 1) {
    if (Number(data.efc_type[i]) === frictionCode && Number(data.efc_id[i]) === Number(jointId)) total += Number(data.efc_force[i]) || 0;
  }
  return total;
}

function updateBamBeforePhysicsStep() {
  const delayed = targetDelay.push(requestedTargets);
  delayedTargets = delayed.target;
  const load = previousMotorTorqueNm.reduce((sum, value) => sum + Math.abs(Number(value) || 0), 0);
  effectiveVinV = Math.max(
    MICRODUCK_DEPLOYMENT_PLANT_PROFILE.vinMinV,
    MICRODUCK_DEPLOYMENT_PLANT_PROFILE.nominalVinV - MICRODUCK_DEPLOYMENT_PLANT_PROFILE.vinDropGainVPerNm * load,
  );
  const nextMotor = new Array(requestedTargets.length).fill(0);
  for (const actuator of actuatorState.values()) {
    const index = actuator.id;
    const dof = actuator.joint.dof;
    const q = Number(data.qpos[actuator.joint.qpos]);
    const dq = Number(data.qvel[dof]);
    const motorTorque = microDuckBamMotorTorqueNm({
      targetRad: delayedTargets[index], positionRad: q, velocityRadS: dq,
      firmwareGain: appliedFirmwareGain, vinV: effectiveVinV, enabled: actuationEnabled,
    });
    nextMotor[index] = motorTorque;

    // bam.mujoco.MujocoController computes friction from the previous solved actuator load
    // and the current bias/constraint forces, then lets MuJoCo's friction constraint apply it.
    const previousSolvedActuator = Number(data.qfrc_actuator?.[dof]) || 0;
    const external = -(Number(data.qfrc_bias?.[dof]) || 0)
      + (Number(data.qfrc_constraint?.[dof]) || 0)
      - frictionConstraintForceForJoint(actuator.joint.id);
    const frictionLoss = microDuckBamFrictionLossNm({
      motorTorqueNm: previousSolvedActuator,
      externalTorqueNm: external,
      velocityRadS: dq,
      frictionScale: MICRODUCK_DEPLOYMENT_PLANT_PROFILE.frictionScale,
      quadraticSignGate: MICRODUCK_DEPLOYMENT_PLANT_PROFILE.quadraticSignGate,
    });
    model.dof_frictionloss[dof] = frictionLoss;
    model.dof_damping[dof] = MICRODUCK_BAM_M6.frictionViscousNmPerRadS;
    lastFrictionLossNm[index] = frictionLoss;
    data.ctrl[index] = clamp(motorTorque, actuator.forceRange[0], actuator.forceRange[1]);
  }
  previousMotorTorqueNm = nextMotor;
  lastMotorTorqueNm = [...nextMotor];
}

function rawImu() {
  const trunk = bodyState.get(TRUNK_BODY);
  const quatOffset = trunk.id * 4;
  const quat = [Number(data.xquat[quatOffset]), Number(data.xquat[quatOffset + 1]), Number(data.xquat[quatOffset + 2]), Number(data.xquat[quatOffset + 3])];
  const a = modelInfo.gyroAddress;
  return {
    gyroRadS: [Number(data.sensordata[a]), Number(data.sensordata[a + 1]), Number(data.sensordata[a + 2])],
    projectedGravity: projectedGravityFromQuaternion(quat),
  };
}

function sampleDeploymentImu() {
  const raw = rawImu();
  return imuFilter.sample(raw.gyroRadS, raw.projectedGravity);
}

function afterPhysicsStep() {
  physicsStepsSinceImuSample += 1;
  if (physicsStepsSinceImuSample >= CONTROL_DECIMATION) {
    physicsStepsSinceImuSample = 0;
    sampleDeploymentImu();
  }
}

function actuatorForceTotalNm() {
  let total = 0;
  for (const actuator of actuatorState.values()) {
    const value = Number(data?.actuator_force?.[actuator.id]);
    if (Number.isFinite(value)) total += Math.abs(value);
  }
  return total;
}

function contactForceNormalN(index) {
  if (typeof mujoco?.mj_contactForce !== 'function') return null;
  try { const buffer = new Float64Array(6); mujoco.mj_contactForce(model, data, index, buffer); return Math.abs(Number(buffer[0])); } catch { return null; }
}

function readContacts() {
  const count = Number(data?.ncon || 0); const contacts = []; let readable = true;
  const collection = data?.contact; const floor = geomIds.get(FLOOR_GEOM); const ball = geomIds.get(BALL_GEOM) ?? null;
  const feet = FOOT_GEOMS.map((name) => geomIds.get(name));
  const footFloor = { left_foot_collision: false, right_foot_collision: false }; const footBall = [];
  if (!collection) return { count, readable: count === 0, contacts, footFloor, footBall };
  try {
    const available = typeof collection.size === 'function' ? Number(collection.size()) : count;
    if (available < count) readable = false;
    for (let index = 0; index < Math.min(count, available); index += 1) {
      const contact = typeof collection.get === 'function' ? collection.get(index) : collection[index];
      if (!contact) { readable = false; continue; }
      try {
        const geom1 = Number(contact.geom?.[0] ?? contact.geom1 ?? -1); const geom2 = Number(contact.geom?.[1] ?? contact.geom2 ?? -1);
        const name1 = nameForGeom(geom1); const name2 = nameForGeom(geom2);
        contacts.push({ geom1, geom2, geom1Name: name1, geom2Name: name2, distanceM: Number(contact.dist ?? 0) });
        const footIndex = feet.findIndex((id) => id === geom1 || id === geom2);
        if (footIndex >= 0) {
          const footName = FOOT_GEOMS[footIndex];
          if (geom1 === floor || geom2 === floor) footFloor[footName] = true;
          if (ball != null && (geom1 === ball || geom2 === ball)) footBall.push({ foot: footName, normalForceN: contactForceNormalN(index), geoms: [name1, name2] });
        }
      } finally { contact.delete?.(); }
    }
  } finally { collection.delete?.(); }
  return { count, readable, contacts, footFloor, footBall };
}

function observation() {
  if (!model || !data || !descriptor || !modelInfo) return { simulationTime: 0, model: null, engine: null, joints: {}, bodies: {}, imu: null, contactCount: 0, contactsReadable: false, contacts: [], footContacts: null, actuationEnabled: true, setupLog: [] };
  const joints = {};
  for (const [name, joint] of jointState) {
    const actuator = actuatorForJoint(name);
    joints[name] = {
      positionRad: Number(data.qpos[joint.qpos]), velocityRadS: Number(data.qvel[joint.dof]),
      // Preserve controller intent even though data.ctrl is now motor torque.
      targetRad: actuator ? Number(requestedTargets[actuator.id]) : null,
      delayedTargetRad: actuator ? Number(delayedTargets[actuator.id]) : null,
      effortNm: actuator ? (Number(data.actuator_force?.[actuator.id]) || 0) : null,
      motorTorqueNm: actuator ? Number(lastMotorTorqueNm[actuator.id] || 0) : null,
      frictionLossNm: actuator ? Number(lastFrictionLossNm[actuator.id] || 0) : null,
      controlRangeRad: actuator ? [...actuator.controlRange] : null,
      forceRangeNm: actuator ? [...actuator.forceRange] : null,
      jointRangeRad: [...joint.range], passive: Boolean(joint.passive),
    };
  }
  const bodies = {};
  for (const [name, body] of bodyState) {
    const posOffset = body.id * 3; const quatOffset = body.id * 4;
    const record = { frame: 'mujoco_world', positionM: [Number(data.xpos[posOffset]), Number(data.xpos[posOffset + 1]), Number(data.xpos[posOffset + 2])], quaternionWxyz: [Number(data.xquat[quatOffset]), Number(data.xquat[quatOffset + 1]), Number(data.xquat[quatOffset + 2]), Number(data.xquat[quatOffset + 3])] };
    if (body.freeDof != null) { record.linearVelocityMS = [Number(data.qvel[body.freeDof]), Number(data.qvel[body.freeDof + 1]), Number(data.qvel[body.freeDof + 2])]; record.angularVelocityRadS = [Number(data.qvel[body.freeDof + 3]), Number(data.qvel[body.freeDof + 4]), Number(data.qvel[body.freeDof + 5])]; record.freeBody = true; }
    bodies[name] = record;
  }
  const raw = rawImu(); const filtered = imuFilter.last;
  const contactState = readContacts();
  const imu = {
    site: 'imu',
    gyroRadS: [...filtered.gyroRadS], projectedGravity: [...filtered.projectedGravity],
    rawGyroRadS: [...raw.gyroRadS], rawProjectedGravity: [...raw.projectedGravity],
    pipeline: MICRODUCK_DEPLOYMENT_PLANT_PROFILE.imuPipeline,
    source: 'MuJoCo ground truth passed through the deployed MicroDuck median-of-three gyro/gravity preprocessing; SFLP gyro-bias estimation remains an on-sensor hardware function',
    bam: {
      model: 'xl330/m6', bamVersion: MICRODUCK_BAM_VERSION, bamRevision: MICRODUCK_BAM_REVISION,
      profile: MICRODUCK_DEPLOYMENT_PLANT_PROFILE.id,
      effectiveVinV, vinDropGainVPerNm: MICRODUCK_DEPLOYMENT_PLANT_PROFILE.vinDropGainVPerNm,
      smallSignalKpNmRad: microDuckBamSmallSignalKpNmRad(actuationEnabled ? appliedFirmwareGain : 0, effectiveVinV),
      forceCeilingNm: microDuckBamForceCeilingNm(), targetDelayPhysicsSteps: [0, 0],
    },
  };
  return {
    simulationTime: Number(data.time || 0),
    model: { id: descriptor.modelId || descriptor.id, asset: descriptor.asset, sha256: modelInfo.modelSha256 },
    engine: { version: modelInfo.engineVersion, versionEvidence: modelInfo.engineVersionEvidence, timestepSeconds: modelInfo.timestepSeconds },
    joints, bodies, imu,
    contactCount: contactState.count, contactsReadable: contactState.readable, contacts: contactState.contacts,
    footContacts: { floor: contactState.footFloor, ball: contactState.footBall },
    actuationEnabled, firmwareGain: actuationEnabled ? appliedFirmwareGain : 0,
    // Compatibility field retained for older consumers. Physics no longer uses this PD kp;
    // it is the source XML's equivalent identified stiffness at the current firmware gain.
    appliedServoKp: actuationEnabled ? microDuckKpForFirmwareGain(appliedFirmwareGain) : 0,
    actuatorForceTotalNm: actuatorForceTotalNm(), setupLog: [...setupLog],
  };
}

function applyDeclaredInitialState({ keepSetupLog = false } = {}) {
  if (!model || !data || !descriptor) throw new Error('No MuJoCo model is loaded');
  mujoco.mj_resetData(model, data); if (!keepSetupLog) setupLog = [];
  resetBamState();
  for (const [jointId, raw] of Object.entries(descriptor.initialJointPositionsRad || {})) {
    const joint = jointState.get(jointId); if (!joint) throw new Error(`Initial state references unknown joint ${jointId}`);
    const value = Number(raw); if (!Number.isFinite(value)) throw new Error(`Initial state for ${jointId} must be finite radians`);
    if (joint.range.every(Number.isFinite) && (value < joint.range[0] || value > joint.range[1])) throw new RangeError(`Initial state ${value} is outside joint ${jointId} range`);
    const actuator = actuatorForJoint(jointId); if (actuator && (value < actuator.controlRange[0] || value > actuator.controlRange[1])) throw new RangeError(`Initial state ${value} is outside actuator target range`);
    data.qpos[joint.qpos] = value;
    if (actuator) { requestedTargets[actuator.id] = value; delayedTargets[actuator.id] = value; data.ctrl[actuator.id] = 0; }
  }
  targetDelay.reset(requestedTargets);
  actuationEnabled = true; paused = false;
  for (const actuator of actuatorState.values()) {
    // BAM edit_spec replaces the XML joint friction/damping. The dynamic BAM values are
    // written immediately before each mj_step; start from zero so the fallback servo plant
    // can never be double-counted during model setup.
    model.dof_frictionloss[actuator.joint.dof] = 0;
    model.dof_damping[actuator.joint.dof] = 0;
  }
  mujoco.mj_forward(model, data);
  imuFilter.reset(); sampleDeploymentImu();
  logSetup({ event: 'reset', detail: 'source home pose; BAM M6 deployment-reference plant; deployed IMU median history reset', bamVersion: MICRODUCK_BAM_VERSION, plantProfile: MICRODUCK_DEPLOYMENT_PLANT_PROFILE.id });
  return observation();
}

function disposeModel() {
  try { data?.delete?.(); } catch {} try { model?.delete?.(); } catch {}
  data = null; model = null; descriptor = null; jointState = new Map(); actuatorState = new Map(); bodyState = new Map(); geomIds = new Map(); modelInfo = null;
  paused = false; actuationEnabled = true; setupLog = []; resetBamState();
}

function meshDependenciesFromXml(xml) {
  const compiler = xml.match(/<compiler\b[^>]*\bmeshdir="([^"]+)"/i);
  const meshDir = compiler?.[1] || '';
  if (meshDir !== 'assets') throw new Error(`MicroDuck source model must declare compiler meshdir="assets", got ${meshDir || '<none>'}`);
  const files = [...xml.matchAll(/<mesh\b[^>]*\bfile="([^"]+)"/gi)].map((match) => match[1]);
  if (!files.length) throw new Error('MicroDuck source model declares no external mesh files');
  const unique = [...new Set(files)];
  for (const file of unique) {
    if (!/^[A-Za-z0-9._-]+\.stl$/i.test(file) || file.includes('..') || file.includes('/') || file.includes('\\')) {
      throw new Error(`Worker rejected unsafe MicroDuck mesh dependency: ${file}`);
    }
  }
  return unique;
}

async function buildModelVfs(mj, xml, modelUrl) {
  if (typeof mj?.MjVFS !== 'function') throw new Error('Bundled MuJoCo runtime does not expose MjVFS');
  const files = meshDependenciesFromXml(xml);
  const modelDirectory = new URL('./', modelUrl);
  const assetDirectory = new URL('assets/', modelDirectory);
  if (assetDirectory.origin !== self.location.origin) throw new Error('MicroDuck mesh directory must be same-origin');
  const vfs = new mj.MjVFS();
  try {
    await Promise.all(files.map(async (file) => {
      const url = new URL(file, assetDirectory);
      if (url.origin !== self.location.origin || !url.pathname.startsWith(assetDirectory.pathname)) {
        throw new Error(`Worker rejected cross-origin or escaping MicroDuck mesh dependency: ${file}`);
      }
      const response = await fetch(url.href, { cache: 'no-store' });
      if (!response.ok) throw new Error(`MicroDuck mesh ${file} returned HTTP ${response.status}`);
      const bytes = new Uint8Array(await response.arrayBuffer());
      if (!bytes.byteLength) throw new Error(`MicroDuck mesh ${file} is empty`);
      // The source MJCF declares meshdir="assets". This is the exact VFS key
      // resolved by MuJoCo; no path rewriting or primitive substitution occurs.
      vfs.addBuffer(`assets/${file}`, bytes);
    }));
    return vfs;
  } catch (error) {
    try { vfs.delete?.(); } catch {}
    throw error;
  }
}

async function load(modelPackage) {
  if (!modelPackage?.id || !modelPackage?.asset || !modelPackage?.sha256 || !Array.isArray(modelPackage.joints) || !Array.isArray(modelPackage.actuators)) throw new Error('Worker requires a validated registered model package descriptor');
  const modelUrl = validatedModelUrl(modelPackage.asset); const mj = await ensureMuJoCo(); disposeModel();
  const xml = await fetch(modelUrl, { cache: 'no-store' }).then((response) => { if (!response.ok) throw new Error(`MuJoCo model returned HTTP ${response.status}`); return response.text(); });
  const modelSha256 = await sha256Text(xml); if (modelSha256 !== modelPackage.sha256) throw new Error(`Model SHA-256 mismatch for ${modelPackage.id}`);
  const vfs = await buildModelVfs(mj, xml, modelUrl);
  try { model = mj.MjModel.from_xml_string(xml, vfs); } finally { try { vfs.delete?.(); } catch {} }
  if (!model) throw new Error(`MuJoCo failed to compile ${modelPackage.id}`);
  data = new mj.MjData(model); if (!data) throw new Error(`MuJoCo failed to allocate data for ${modelPackage.id}`);
  descriptor = structuredClone(modelPackage); resolveModelAddresses(modelSha256); configureBamMotorPlant();
  return applyDeclaredInitialState();
}

function doPhysicsStep() { updateBamBeforePhysicsStep(); mujoco.mj_step(model, data); afterPhysicsStep(); }
function step(count = 1) {
  if (!model || !data) throw new Error('No MuJoCo model is loaded'); if (paused) return observation();
  if (!Number.isInteger(count) || count < 1 || count > MAX_STEP_BATCH) throw new RangeError(`step count must be an integer from 1 to ${MAX_STEP_BATCH}`);
  for (let index = 0; index < count; index += 1) doPhysicsStep(); return observation();
}
function stepSampled(count = 1, sampleEverySteps = 1) {
  if (!model || !data) throw new Error('No MuJoCo model is loaded');
  if (!Number.isInteger(count) || count < 1 || count > MAX_STEP_BATCH) throw new RangeError(`step count must be an integer from 1 to ${MAX_STEP_BATCH}`);
  if (!Number.isInteger(sampleEverySteps) || sampleEverySteps < 1) throw new RangeError('sampleEverySteps must be a positive integer');
  if (paused) return { observations: [observation()], executedSteps: 0, sampleEverySteps };
  const expected = sampledObservationCount(count, sampleEverySteps); if (expected > MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE) throw new RangeError('sampled advance exceeds bounded observation count');
  const observations = [];
  for (let index = 1; index <= count; index += 1) { doPhysicsStep(); if (index === count || index % sampleEverySteps === 0) observations.push(observation()); }
  return { observations, executedSteps: count, sampleEverySteps };
}

function validatedTarget(jointId, rawValue) {
  const joint = jointState.get(jointId); if (!joint) throw new Error(`Unknown declared joint ${jointId || '<missing>'}`); if (joint.passive) throw new Error(`Joint ${jointId} is passive`);
  const actuator = actuatorForJoint(jointId); if (!actuator) throw new Error(`Joint ${jointId} has no actuator`);
  const target = Number(rawValue); if (!Number.isFinite(target)) throw new TypeError(`Target for ${jointId} must be finite`);
  if (target < actuator.controlRange[0] || target > actuator.controlRange[1]) throw new RangeError(`Target ${target} is outside ${jointId} range`);
  return { actuator, target };
}
function command(payload = {}) {
  if (!model || !data || !descriptor || !modelInfo) throw new Error('No MuJoCo model is loaded'); if (payload.type !== 'set_joint_targets') throw new Error(`Unsupported physical command: ${payload.type}`);
  const targets = payload.targetsRad; if (!targets || typeof targets !== 'object' || Array.isArray(targets) || !Object.keys(targets).length) throw new Error('set_joint_targets requires a non-empty targetsRad object');
  const applied = Object.entries(targets).map(([jointId, value]) => validatedTarget(jointId, value));
  if (payload.firmwareGain != null && actuationEnabled) {
    const gain = Number(payload.firmwareGain); if (!Number.isFinite(gain) || gain < 0) throw new TypeError('firmwareGain must be finite and non-negative'); appliedFirmwareGain = gain;
  }
  for (const { actuator, target } of applied) requestedTargets[actuator.id] = target;
  return observation();
}

function setup(payload = {}) {
  if (!model || !data || !descriptor) throw new Error('No MuJoCo model is loaded');
  if (payload.type === 'set_trunk_orientation') {
    const quaternion = payload.quaternionWxyz; if (!Array.isArray(quaternion) || quaternion.length !== 4 || !quaternion.every((value) => Number.isFinite(Number(value)))) throw new TypeError('set_trunk_orientation requires a finite wxyz quaternion');
    const norm = Math.hypot(...quaternion.map(Number)); if (!(norm > 1e-6)) throw new RangeError('set_trunk_orientation quaternion must be non-degenerate');
    const trunk = bodyState.get(TRUNK_BODY); for (let i = 0; i < 4; i += 1) data.qpos[trunk.freeQpos + 3 + i] = Number(quaternion[i]) / norm; for (let i = 0; i < 6; i += 1) data.qvel[trunk.freeDof + i] = 0;
    mujoco.mj_forward(model, data); imuFilter.reset(); sampleDeploymentImu(); physicsStepsSinceImuSample = 0;
    logSetup({ event: 'setup_trunk_orientation', label: String(payload.label || 'declared pre-trial orientation'), quaternionWxyz: quaternion.map(Number), imuHistoryReset: true });
  } else if (payload.type === 'set_object_pose') {
    const body = bodyState.get(String(payload.bodyId || '')); if (!body || body.freeQpos == null) throw new Error(`set_object_pose requires a declared free body, not ${payload.bodyId}`); if (body.id === bodyState.get(TRUNK_BODY).id) throw new Error('set_object_pose may not move the robot trunk');
    const position = payload.positionM; if (!Array.isArray(position) || position.length !== 3 || !position.every((value) => Number.isFinite(Number(value)))) throw new TypeError('set_object_pose requires finite positionM');
    for (let i = 0; i < 3; i += 1) data.qpos[body.freeQpos + i] = Number(position[i]); data.qpos[body.freeQpos + 3] = 1; data.qpos[body.freeQpos + 4] = 0; data.qpos[body.freeQpos + 5] = 0; data.qpos[body.freeQpos + 6] = 0; for (let i = 0; i < 6; i += 1) data.qvel[body.freeDof + i] = 0;
    mujoco.mj_forward(model, data); logSetup({ event: 'setup_object_pose', label: String(payload.label || 'declared pre-trial object placement'), bodyId: String(payload.bodyId), positionM: position.map(Number) });
  } else if (payload.type === 'set_actuation_enabled') {
    actuationEnabled = payload.enabled !== false; if (actuationEnabled) appliedFirmwareGain = MICRODUCK_NOMINAL_FIRMWARE_GAIN;
    else { lastMotorTorqueNm.fill(0); previousMotorTorqueNm.fill(0); for (const actuator of actuatorState.values()) data.ctrl[actuator.id] = 0; }
    mujoco.mj_forward(model, data);
    logSetup({ event: 'setup_actuation', label: String(payload.label || (actuationEnabled ? 'actuators enabled' : 'declared motor-torque-off condition')), actuationEnabled, firmwareGain: actuationEnabled ? appliedFirmwareGain : 0, passiveBamFrictionRetained: true });
  } else throw new Error(`Unsupported setup operation: ${payload.type}`);
  return observation();
}

self.onmessage = async (event) => {
  const { id, op, payload } = event.data || {};
  try {
    let result;
    if (op === 'load') result = await load(payload?.modelPackage);
    else if (op === 'reset') result = applyDeclaredInitialState();
    else if (op === 'step') result = step(payload?.count);
    else if (op === 'stepSampled') result = stepSampled(payload?.count, payload?.sampleEverySteps);
    else if (op === 'observe') result = observation();
    else if (op === 'command') result = command(payload);
    else if (op === 'setup') result = setup(payload);
    else if (op === 'pause') { paused = true; result = observation(); }
    else if (op === 'resume') { paused = false; result = observation(); }
    else if (op === 'dispose') { disposeModel(); result = { disposed: true }; }
    else throw new Error(`Unsupported worker operation: ${op}`);
    reply(id, true, result);
  } catch (error) { reply(id, false, null, error?.stack || error?.message || String(error)); }
};
