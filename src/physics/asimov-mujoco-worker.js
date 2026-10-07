import { asimovSensitivity, ASIMOV_BODY_SENSOR_PROFILE, ASIMOV_SENSOR_STANDING_CONTROLLER } from './asimov-sensitivity.js';
import { AsimovBodySensors } from './asimov-body-sensors.js';
import { AsimovSensorStanding } from './asimov-sensor-standing.js';
import { ASIMOV_ACTUATOR_PROFILE, ASIMOV_SENSOR_PROFILE } from './asimov-actuator-profile.js';
import { AsimovActuatorPlant } from './asimov-actuator-plant.js';
import { AsimovSensorModel } from './asimov-sensor-model.js';
import { ASIMOV_STANDING_CONTROLLER, AsimovStandingEvaluator, stanceFeedback } from './asimov-standing.js';
import { ASIMOV_SOURCE } from './asimov-generated.js';
import { angularVelocityWorld } from './asimov-frames.js';
import loadMujoco from '../../assets/microduck/runtime/mujoco/mujoco.js';
import { MAX_ADVANCE_STEPS_PER_REQUEST, MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE, sampledObservationCount } from './backend-contract.js';
import { ASIMOV_JOINT_ORDER, ASIMOV_FOOT_CONTACT_GEOMS, ASIMOV_JOINT_HOLD_PROFILE, ASIMOV_LOWLEVEL_COMMAND_PROFILE, ASIMOV_LOWLEVEL_CONTROL_INTERVAL_SECONDS, boundLowLevelCommand, lowLevelTorqueNm } from './asimov-controller.js';

// Asimov authoritative worker. The audited fleet RPC/session contract is reused,
// but model identity and controller parameters are exclusively Asimov's. Control
// writes motor ctrl and declared passive joint friction, never root or joint state.
// Reset is the explicitly declared model keyframe.
// Visual STLs have no mass/contact in source and are rendered outside the engine.

const MUJOCO_BASE_URL = new URL('../../assets/microduck/runtime/mujoco/', import.meta.url);
const EXPECTED_MUJOCO_VERSION = '3.11.0';
const MAX_STEP_BATCH = MAX_ADVANCE_STEPS_PER_REQUEST;
const MODEL_ASSET_RE = /^models\/[A-Za-z0-9._/-]+\.xml$/;
const MODEL_VALUE_TOLERANCE = 1e-9;
const INTEGRATOR_CODES = Object.freeze({ Euler: 0, RK4: 1, implicit: 2, implicitfast: 3 });
const TORQUE_COMMAND = 'torque-nm';
const FOOT_GEOM_SIDE = new Map([
  ...ASIMOV_FOOT_CONTACT_GEOMS.left.map((name) => [name, 'left']),
  ...ASIMOV_FOOT_CONTACT_GEOMS.right.map((name) => [name, 'right']),
]);

let mujoco = null;
let model = null;
let data = null;
let paused = false;
let descriptor = null;
let jointState = new Map();
let actuatorState = new Map();
let bodyState = new Map();
let modelInfo = null;
let accepted = [];
let controller = null;
let actuationEnabled = true;
let setupLog = [];
// Distinguish the estimated hold controller from caller-specified low-level commands.
let commandProfile = ASIMOV_JOINT_HOLD_PROFILE;
let actuatorPlant = null;
let sensorModel = null;
let sensorStanding = null;
let standingEvaluator = new AsimovStandingEvaluator();

function reply(id, ok, payload = null, error = null) { postMessage({ id, ok, payload, error }); }
async function ensureMuJoCo() { if (mujoco) return mujoco; mujoco = await loadMujoco({ locateFile: (path) => new URL(path, MUJOCO_BASE_URL).href }); return mujoco; }
async function sha256Text(text) { const digest = await crypto.subtle.digest('SHA-256', new TextEncoder().encode(text)); return [...new Uint8Array(digest)].map((value) => value.toString(16).padStart(2, '0')).join(''); }
function enumValue(name) { const entry = mujoco?.mjtObj?.[name]; const value = entry && typeof entry === 'object' && 'value' in entry ? entry.value : entry; if (!Number.isInteger(Number(value))) throw new Error(`MuJoCo object enum ${name} is unavailable`); return Number(value); }
function idFor(typeName, name) { const id = Number(mujoco.mj_name2id(model, enumValue(typeName), name)); if (!Number.isInteger(id) || id < 0) throw new Error(`Required MuJoCo ${typeName} named ${name} is missing`); return id; }
function nameFor(typeName, id) { if (!Number.isInteger(id) || id < 0 || typeof mujoco?.mj_id2name !== 'function') return null; try { return mujoco.mj_id2name(model, enumValue(typeName), id) || null; } catch { return null; } }
function runtimeVersionEvidence() { const version = typeof mujoco?.mj_versionString === 'function' ? String(mujoco.mj_versionString()) : EXPECTED_MUJOCO_VERSION; if (version !== EXPECTED_MUJOCO_VERSION) throw new Error(`Bundled MuJoCo runtime reports ${version}; expected ${EXPECTED_MUJOCO_VERSION}`); return { version, evidence: typeof mujoco?.mj_versionString === 'function' ? 'mj_versionString runtime introspection' : 'bundled asset manifest and repository hash gate' }; }
function validatedModelUrl(asset) { if (!MODEL_ASSET_RE.test(asset || '') || String(asset).includes('..')) throw new Error('Worker rejected non-registry model asset path'); const url = new URL(`../../${asset}`, import.meta.url); if (url.origin !== self.location.origin) throw new Error('Worker model asset must be same-origin'); return url.href; }
function assertClose(actual, expected, label, tolerance = MODEL_VALUE_TOLERANCE) { if (!Array.isArray(expected) || actual.length !== expected.length) throw new Error(`${label} descriptor shape mismatch`); for (let index = 0; index < actual.length; index += 1) { const delta = Math.abs(Number(actual[index]) - Number(expected[index])); if (!Number.isFinite(delta) || delta > tolerance) throw new Error(`${label} mismatch at index ${index}: compiled ${actual[index]}, descriptor ${expected[index]}`); } }
function logSetup(entry) { setupLog = [...setupLog.slice(-31), Object.freeze({ simulationTimeSeconds: Number(data?.time || 0), ...entry })]; }

function resolveModelAddresses(modelSha256) {
  jointState = new Map(); actuatorState = new Map(); bodyState = new Map();
  const declared = descriptor.joints.map((joint) => joint.id);
  if (declared.join('|') !== ASIMOV_JOINT_ORDER.join('|')) {
    throw new Error('Model package joint order does not match the audited Asimov 1 23-DoF motor order');
  }
  for (const [index, joint] of descriptor.joints.entries()) {
    const id = idFor('mjOBJ_JOINT', joint.id);
    const qpos = Number(model.jnt_qposadr[id]);
    const dof = Number(model.jnt_dofadr[id]);
    if (![qpos, dof].every(Number.isInteger)) throw new Error(`MuJoCo address table is invalid for joint ${joint.id}`);
    const range = [Number(model.jnt_range[id * 2]), Number(model.jnt_range[id * 2 + 1])];
    const axis = [Number(model.jnt_axis[id * 3]), Number(model.jnt_axis[id * 3 + 1]), Number(model.jnt_axis[id * 3 + 2])];
    const forceRange = [Number(model.jnt_actfrcrange[id * 2]), Number(model.jnt_actfrcrange[id * 2 + 1])];
    assertClose(range, joint.rangeRad, `Joint ${joint.id} range`);
    assertClose(axis, joint.axis, `Joint ${joint.id} axis`);
    assertClose(forceRange, [-joint.effortLimitNm, joint.effortLimitNm], `Joint ${joint.id} actuator force range`);
    if (descriptor.actuatorProfileId) {
      assertClose([Number(model.dof_armature[dof]),Number(model.dof_damping[dof]),Number(model.dof_frictionloss[dof])], [ASIMOV_SOURCE.joints[index].armature,0,0], `Joint ${joint.id} inertia/dissipation`);
    }
    jointState.set(joint.id, { id, index, qpos, dof, range, axis });
  }
  for (const actuator of descriptor.actuators) {
    const id = idFor('mjOBJ_ACTUATOR', actuator.id);
    const joint = jointState.get(actuator.jointId);
    if (!joint) throw new Error(`Actuator ${actuator.id} references unknown descriptor joint ${actuator.jointId}`);
    if (actuator.command !== TORQUE_COMMAND) throw new Error(`Actuator ${actuator.id} declares unsupported command ${actuator.command}`);
    if (model.actuator_trnid?.length && Number(model.actuator_trnid[id * 2]) !== joint.id) throw new Error(`Actuator ${actuator.id} is not mapped to joint ${actuator.jointId}`);
    if (id !== joint.index) throw new Error(`Actuator ${actuator.id} is at model index ${id}, expected the audited motor index ${joint.index}`);
    const controlRange = [Number(model.actuator_ctrlrange[id * 2]), Number(model.actuator_ctrlrange[id * 2 + 1])];
    const forceRange = [Number(model.actuator_forcerange[id * 2]), Number(model.actuator_forcerange[id * 2 + 1])];
    assertClose(forceRange, actuator.forceRangeNm, `Actuator ${actuator.id} force range`);
    assertClose(controlRange, actuator.controlRangeNm, `Actuator ${actuator.id} control range`);
    // Every actuator must be bounded in command and effort. An unlimited motor would let requested
    // motion appear without a defensible physical cost.
    if (!controlRange.every(Number.isFinite) || controlRange[0] >= controlRange[1]) throw new Error(`Actuator ${actuator.id} has no finite control range`);
    actuatorState.set(actuator.id, { ...actuator, id, controlRange, forceRange, jointIndex: joint.index });
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
  const pelvisFree = Boolean(bodyState.get('pelvis_link')?.freeDof != null);
  if (pelvisFree !== (descriptor.rootMode === 'free-base')) {
    throw new Error(`Model ${descriptor.id} declares rootMode ${descriptor.rootMode} but the compiled pelvis is ${pelvisFree ? 'free' : 'welded'}`);
  }
  // The source plant has ideal independent joints (including ankles), no tendon or equality.
  // Adding a coupling or weld at runtime would silently change the declared model.
  const equalityCount = Number(model.neq ?? 0);
  if (equalityCount !== 0) throw new Error(`Asimov 1 models must declare no equality constraints; compiled model has ${equalityCount}`);
  const tendonCount = Number(model.ntendon ?? 0);
  if (tendonCount !== 0) throw new Error(`Asimov 1 models must declare no tendons; compiled model has ${tendonCount}`);
  for (const side of ['left', 'right']) {
    for (const geom of ASIMOV_FOOT_CONTACT_GEOMS[side]) idFor('mjOBJ_GEOM', geom);
  }

  const timestepSeconds = Number(model.opt?.timestep);
  if (!Number.isFinite(timestepSeconds) || Math.abs(timestepSeconds - Number(descriptor.physics.timestepSeconds)) > 1e-12) throw new Error(`Unexpected timestep ${timestepSeconds}; expected ${descriptor.physics.timestepSeconds}`);
  const expectedIntegrator = INTEGRATOR_CODES[descriptor.physics.integrator];
  if (!Number.isInteger(expectedIntegrator) || Number(model.opt?.integrator) !== expectedIntegrator) throw new Error(`Unexpected integrator ${model.opt?.integrator}; expected ${descriptor.physics.integrator}`);
  if (descriptor.physics.iterations != null && Number(model.opt?.iterations) !== Number(descriptor.physics.iterations)) throw new Error(`Unexpected solver iterations ${model.opt?.iterations}`);
  const compiledLs = Number(model.opt?.ls_iterations ?? model.opt?.lsIterations);
  if (descriptor.physics.lsIterations != null && compiledLs !== Number(descriptor.physics.lsIterations)) throw new Error(`Unexpected solver line-search iterations ${compiledLs}`);
  const gravity = [Number(model.opt?.gravity?.[0] ?? 0), Number(model.opt?.gravity?.[1] ?? 0), Number(model.opt?.gravity?.[2] ?? 0)];
  if (!(gravity[2] < -9)) throw new Error(`Asimov 1 physical scenes require real gravity; compiled model has ${gravity.join(', ')}`);
  const version = runtimeVersionEvidence();
  modelInfo = { modelSha256, engineVersion: version.version, engineVersionEvidence: version.evidence, timestepSeconds, gravity };
}

function measured(index) {
  const joint = jointState.get(ASIMOV_JOINT_ORDER[index]);
  return { positionRad: Number(data.qpos[joint.qpos]), velocityRadS: Number(data.qvel[joint.dof]) };
}

function neutralCommands() {
  return ASIMOV_JOINT_ORDER.map((jointId, index) => boundLowLevelCommand(jointId, {
    positionRad: measured(index).positionRad, velocityRadS: 0, feedforwardTorqueNm: 0, kp: 0, kd: 0,
  }));
}

function holdCommandsAt(positions, profile = ASIMOV_JOINT_HOLD_PROFILE) {
  return ASIMOV_JOINT_ORDER.map((jointId, index) => boundLowLevelCommand(jointId, {
    positionRad: positions[index], velocityRadS: 0, feedforwardTorqueNm: 0, kp: profile.kp[index], kd: profile.kd[index],
  }));
}

/**
 * One low-level controller update, run once per physics step.
 *
 * Its cadence follows the physics clock, never the rendering frame rate.
 */
function applyControlLaw() {
  if (actuatorPlant) {
    let feedback = null;
    if (controller?.kind === 'standing') {
      if (sensorStanding) {
        // The sensor regulator never receives rootObservation or joint ground truth.
        if (actuatorPlant.stepIndex % actuatorPlant.every === 0) sensorStanding.update(Number(data.time),sensorModel.observation(Number(data.time)));
        feedback = sensorStanding.torques;
      } else feedback = stanceFeedback(rootObservation());
    }
    const output = actuatorPlant.step(accepted, ASIMOV_JOINT_ORDER.map((_,i)=>measured(i)), actuationEnabled, feedback);
    for (let i=0;i<ASIMOV_JOINT_ORDER.length;i++) {
      const id=ASIMOV_JOINT_ORDER[i];
      data.ctrl[actuatorState.get(id).id]=output[i].motorNm;
      // Explicit passive joint friction; it persists when electric actuation is off.
      // No root force/velocity, target state or object state is changed.
      data.qfrc_applied[jointState.get(id).dof]=output[i].frictionNm;
    }
    return;
  }
  for (let index = 0; index < ASIMOV_JOINT_ORDER.length; index += 1) {
    const command = accepted[index];
    const actuator = actuatorState.get(command.jointId);
    data.ctrl[actuator.id] = actuationEnabled ? lowLevelTorqueNm(command, measured(index)) : 0;
  }
}

function readContacts() {
  const count = Number(data?.ncon || 0);
  const classified = { leftFootFloor: [], rightFootFloor: [], otherBodyFloor: [], robotExternalObject: [], robotSelf: [], robotFixture: [], objectFloor: [] };
  const contacts = [];
  let readable = true;
  const collection = data?.contact;
  if (!collection) return { count, readable: count === 0, contacts, classified };
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
        const name1 = nameFor('mjOBJ_GEOM', geom1);
        const name2 = nameFor('mjOBJ_GEOM', geom2);
        const body1 = nameFor('mjOBJ_BODY', Number(model.geom_bodyid[geom1]));
        const body2 = nameFor('mjOBJ_BODY', Number(model.geom_bodyid[geom2]));
        // MuJoCo 3.11 WASM out parameters need a heap-backed DoubleBuffer. A JS
        // Float64Array is copied into WASM and silently retains zeros on return.
        mujoco.mj_contactForce(model, data, index, force);
        const normalForceN = Number(force.GetView()[0]);
        if (!Number.isFinite(normalForceN)) readable = false;
        const entry = { geoms: [name1, name2], bodies: [body1, body2], distanceM: Number(contact.dist ?? 0), normalForceN };
        contacts.push(entry);
        const names = [name1, name2];
        if (names.includes('floor')) {
          const other = name1 === 'floor' ? name2 : name1;
          const side = FOOT_GEOM_SIDE.get(other);
          if (side === 'left') classified.leftFootFloor.push(entry);
          else if (side === 'right') classified.rightFootFloor.push(entry);
          else if (String(other).startsWith('contact_probe_block')) classified.objectFloor.push(entry);
          else classified.otherBodyFloor.push(entry);
        } else if (names.some((name) => String(name).startsWith('contact_probe_block'))) {
          classified.robotExternalObject.push(entry);
        } else if (names.some((name) => String(name).endsWith('_stop_wall'))) {
          classified.robotFixture.push(entry);
        } else if (body1 !== 'world' && body2 !== 'world') {
          classified.robotSelf.push(entry);
        }
      } finally { contact.delete?.(); }
    }
  } finally { force.delete(); collection.delete?.(); }
  return { count, readable, contacts, classified };
}

function rootObservation() {
  const pelvis = bodyState.get('pelvis_link');
  const posOffset = pelvis.id * 3;
  const quatOffset = pelvis.id * 4;
  const quaternion = [Number(data.xquat[quatOffset]), Number(data.xquat[quatOffset + 1]), Number(data.xquat[quatOffset + 2]), Number(data.xquat[quatOffset + 3])];
  const [w, x, y, z] = quaternion;
  const uprightZ = 1 - 2 * (x * x + y * y);
  const record = {
    mode: descriptor.rootMode,
    frame: 'mujoco_world',
    positionM: [Number(data.xpos[posOffset]), Number(data.xpos[posOffset + 1]), Number(data.xpos[posOffset + 2])],
    quaternionWxyz: quaternion,
    uprightZ,
    tiltRad: Math.acos(Math.min(1, Math.max(-1, uprightZ))),
    linearVelocityMS: [0, 0, 0],
    angularVelocityRadS: [0, 0, 0],
    free: pelvis.freeDof != null,
  };
  if (pelvis.freeDof != null) {
    record.linearVelocityMS = [Number(data.qvel[pelvis.freeDof]), Number(data.qvel[pelvis.freeDof + 1]), Number(data.qvel[pelvis.freeDof + 2])];
    record.angularVelocityRadS = angularVelocityWorld(quaternion, Array.from(data.qvel.slice(pelvis.freeDof + 3, pelvis.freeDof + 6)));
  }
  void w;
  return record;
}

function observation() {
  if (!model || !data || !descriptor || !modelInfo) {
    return { simulationTime: 0, model: null, engine: null, root: null, joints: {}, bodies: {}, contactCount: 0, contactsReadable: false, contacts: [], contactClasses: null, controller: null, actuationEnabled: true, setupLog: [] };
  }
  // mj_step integrates qpos/qvel but leaves the derived quantities - body poses, contacts - at the
  // state before the integration, so reading them straight after a step publishes body positions
  // that lag the joint angles and velocities beside them by one timestep. During a fall that is a
  // visible millimetres-scale disagreement between the rendered rig and the reported bodies. One
  // forward evaluation puts every published quantity at the same instant. The actuators are plain
  // torque motors, so the reported effort is unchanged by it, and the trajectory is untouched:
  // mj_step performs this same evaluation itself before integrating.
  mujoco.mj_forward(model, data);
  const joints = {};
  for (const [index, jointId] of ASIMOV_JOINT_ORDER.entries()) {
    const joint = jointState.get(jointId);
    const command = accepted[index];
    const actuator = actuatorState.get(jointId);
    const state = measured(index);
    joints[jointId] = {
      // Requested, accepted and measured are three different things and are always reported as three.
      requestedTargetRad: command.requested.positionRad,
      acceptedTargetRad: command.positionRad,
      requestedVelocityRadS: command.requested.velocityRadS,
      acceptedVelocityRadS: command.velocityRadS,
      commandBounded: command.bounded,
      positionRad: state.positionRad,
      velocityRadS: state.velocityRadS,
      effortNm: Number(data.actuator_force?.[actuator.id]),
      feedforwardTorqueNm: command.feedforwardTorqueNm,
      kp: command.kp,
      kd: command.kd,
      jointRangeRad: [...joint.range],
      velocityLimitRadS: command.limits.velocityLimitRadS,
      effortLimitNm: command.limits.effortLimitNm,
    };
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
      record.angularVelocityRadS = angularVelocityWorld(record.quaternionWxyz, Array.from(data.qvel.slice(body.freeDof + 3, body.freeDof + 6)));
      record.freeBody = true;
    }
    bodies[name] = record;
  }
  const contactState = readContacts();
  return {
    simulationTime: Number(data.time || 0),
    model: { id: descriptor.modelId || descriptor.id, asset: descriptor.asset, sha256: modelInfo.modelSha256, rootMode: descriptor.rootMode },
    engine: {
      version: modelInfo.engineVersion, versionEvidence: modelInfo.engineVersionEvidence,
      timestepSeconds: modelInfo.timestepSeconds, gravity: [...modelInfo.gravity],
      controlIntervalSeconds: ASIMOV_LOWLEVEL_CONTROL_INTERVAL_SECONDS,
    },
    root: rootObservation(),
    joints,
    bodies,
    contactCount: contactState.count,
    contactsReadable: contactState.readable,
    contacts: contactState.contacts,
    contactClasses: contactState.classified,
    controller: controller
      ? { id: controller.profile.id, label: controller.profile.label, mode: controller.kind, engagedAtSeconds: controller.startedAtSeconds, claim: controller.profile.claim }
      : { id: commandProfile.id, label: commandProfile.label, mode: commandProfile === ASIMOV_LOWLEVEL_COMMAND_PROFILE ? 'low-level' : 'joint-hold', engagedAtSeconds: null, claim: commandProfile.claim },
    actuationEnabled,
    setupLog: [...setupLog],
    ...(actuatorPlant ? {actuatorModel: {...actuatorPlant.snapshot(),configurationSha256:modelInfo.actuatorProfileSha256,sensorConfigurationSha256:modelInfo.sensorProfileSha256}, sensorObservation: sensorModel.observation(Number(data.time)), standingAssessment: {...standingEvaluator.snapshot(),...(sensorStanding?{controllerId:ASIMOV_SENSOR_STANDING_CONTROLLER.id,sensorFeedback:sensorStanding.snapshot(),sensitivityProfileId:descriptor.sensitivityProfileId,sensitivityConfigurationSha256:modelInfo.sensitivityProfileSha256}: {})}} : {}),
  };
}

function applyDeclaredInitialState() {
  if (!model || !data || !descriptor) throw new Error('No MuJoCo model is loaded');
  // The declared initial condition is the model's own keyframe: the generator writes it from the
  // source reference offsets and the declared fixture height, so no runtime state assignment
  // is needed here or anywhere else in this worker.
  if (Number(model.nkey ?? 0) < 1) throw new Error(`Model ${descriptor.id} declares no initial keyframe`);
  mujoco.mj_resetDataKeyframe(model, data, 0);
  paused = false;
  controller = null;
  commandProfile = ASIMOV_JOINT_HOLD_PROFILE;
  actuationEnabled = true;
  setupLog = [];
  const sensitivity=descriptor.sensitivityProfileId?asimovSensitivity(descriptor.sensitivityProfileId):null;
  actuatorPlant = descriptor.actuatorProfileId ? new AsimovActuatorPlant(modelInfo.timestepSeconds,sensitivity?.actuator??{}) : null;
  sensorModel = sensitivity ? new AsimovBodySensors(sensitivity.sensor) : actuatorPlant ? new AsimovSensorModel() : null;
  sensorStanding = sensitivity ? new AsimovSensorStanding() : null;
  standingEvaluator.reset();
  if (!['joint-hold', 'passive'].includes(descriptor.initialCommand)) {
    throw new Error(`Model ${descriptor.id} declares an unknown initialCommand ${descriptor.initialCommand}`);
  }
  mujoco.mj_forward(model, data);
  // The declared initial command. 'passive' is the declared zero-torque behaviour - zero torque, so
  // gravity acts on an unheld plant. 'joint-hold' issues a bounded hold at the pose the model
  // itself declares, through the repository-estimated PD gains. Either way this is a command: no
  // joint or root state is written here or anywhere else.
  const initialCommand = descriptor.initialCommand === 'passive' ? 'passive' : 'joint-hold';
  accepted = initialCommand === 'passive'
    ? neutralCommands()
    : holdCommandsAt(ASIMOV_JOINT_ORDER.map((_, index) => measured(index).positionRad));
  logSetup({ event: 'initialCommand', initialCommand, detail: initialCommand === 'passive' ? 'every actuator commanded to zero torque' : 'bounded joint hold at the declared initial joint positions, at the repository-estimated PD gains' });
  for (const actuator of actuatorState.values()) data.ctrl[actuator.id] = 0;
  mujoco.mj_forward(model, data);
  if (sensorModel) sampleSensors();
  return observation();
}

function disposeModel() {
  try { data?.delete?.(); } catch { /* disposal is best effort */ }
  try { model?.delete?.(); } catch { /* disposal is best effort */ }
  data = null; model = null; descriptor = null; modelInfo = null;
  actuatorPlant=null; sensorModel=null; sensorStanding=null; standingEvaluator.reset();
  jointState = new Map(); actuatorState = new Map(); bodyState = new Map();
  accepted = []; controller = null; commandProfile = ASIMOV_JOINT_HOLD_PROFILE; actuationEnabled = true; paused = false; setupLog = [];
}

function meshDependenciesFromXml(xml) {
  const compiler = xml.match(/<compiler\b[^>]*\bmeshdir="([^"]+)"/i);
  const meshDir = compiler?.[1] || '';
  if (meshDir !== 'assets') throw new Error(`Asimov 1 model must declare compiler meshdir="assets", got ${meshDir || '<none>'}`);
  const files = [...xml.matchAll(/<mesh\b[^>]*\bfile="([^"]+)"/gi)].map((match) => match[1]);
  if (!files.length) throw new Error('Asimov 1 model declares no external mesh files');
  const unique = [...new Set(files)];
  for (const file of unique) {
    if (!/^[A-Za-z0-9._-]+\.stl$/i.test(file) || file.includes('..') || file.includes('/') || file.includes('\\')) {
      throw new Error(`Worker rejected unsafe Asimov 1 mesh dependency: ${file}`);
    }
  }
  return unique;
}

async function load(modelPackage) {
  if (!modelPackage?.id || !modelPackage?.asset || !modelPackage?.sha256 || !Array.isArray(modelPackage.joints) || !Array.isArray(modelPackage.actuators)) {
    throw new Error('Worker requires a validated registered model package descriptor');
  }
  if (modelPackage.actuatorProfileId && modelPackage.actuatorProfileId !== ASIMOV_ACTUATOR_PROFILE.id) throw new Error('Unknown Asimov actuator profile');
  const sensitivity=modelPackage.sensitivityProfileId?asimovSensitivity(modelPackage.sensitivityProfileId):null;
  const sensorId=sensitivity?ASIMOV_BODY_SENSOR_PROFILE.id:ASIMOV_SENSOR_PROFILE.id;
  const standingId=sensitivity?ASIMOV_SENSOR_STANDING_CONTROLLER.id:ASIMOV_STANDING_CONTROLLER.id;
  if (modelPackage.sensorProfileId && modelPackage.sensorProfileId !== sensorId) throw new Error('Unknown Asimov sensor profile');
  if (sensitivity && (!modelPackage.actuatorProfileId || modelPackage.sensorProfileId!==sensorId || modelPackage.standingControllerId!==standingId)) throw new Error('Incomplete sensor-standing profile');
  if (modelPackage.standingControllerId && (!modelPackage.actuatorProfileId || modelPackage.standingControllerId !== standingId || modelPackage.rootMode !== 'free-base')) throw new Error('Invalid standing model/controller pair');
  const modelUrl = validatedModelUrl(modelPackage.asset);
  const mj = await ensureMuJoCo();
  disposeModel();
  const xml = await fetch(modelUrl, { cache: 'no-store' }).then((response) => { if (!response.ok) throw new Error(`MuJoCo model returned HTTP ${response.status}`); return response.text(); });
  const modelSha256 = await sha256Text(xml);
  if (modelSha256 !== modelPackage.sha256) throw new Error(`Model SHA-256 mismatch for ${modelPackage.id}`);
  const vfs = new mj.MjVFS();
  try { model = mj.MjModel.from_xml_string(xml, vfs); } finally { try { vfs.delete?.(); } catch { /* disposal is best effort */ } }
  if (!model) throw new Error(`MuJoCo failed to compile ${modelPackage.id}`);
  data = new mj.MjData(model);
  if (!data) throw new Error(`MuJoCo failed to allocate data for ${modelPackage.id}`);
  descriptor = structuredClone(modelPackage);
  resolveModelAddresses(modelSha256);
  if (descriptor.actuatorProfileId) {
    modelInfo.actuatorProfileSha256=await sha256Text(JSON.stringify(sensitivity?{base:ASIMOV_ACTUATOR_PROFILE,sensitivity:sensitivity.actuator}:ASIMOV_ACTUATOR_PROFILE));
    modelInfo.sensorProfileSha256=await sha256Text(JSON.stringify(sensitivity?.sensor??ASIMOV_SENSOR_PROFILE));
    if (sensitivity) modelInfo.sensitivityProfileSha256=await sha256Text(JSON.stringify(sensitivity));
  }
  return applyDeclaredInitialState();
}

function sampleSensors() {
  const joints=Object.fromEntries(ASIMOV_JOINT_ORDER.map((id,i)=>[id,measured(i)]));
  sensorModel.sample(Number(data.time), joints, rootObservation());
}
function doPhysicsStep() {
  applyControlLaw(); mujoco.mj_step(model, data);
  if (actuatorPlant) {
    // State-derived observations are synchronized, and their histories are updated
    // by physics time even when no UI or Python caller reads them.
    mujoco.mj_forward(model,data);
    if (![...data.qpos,...data.qvel].every(Number.isFinite)) throw new Error('Non-finite Asimov physical state');
    sampleSensors();
    if (standingEvaluator.startedAt != null) standingEvaluator.update(Number(data.time),rootObservation(),readContacts(),actuationEnabled,controller?.kind==='standing' && !sensorStanding?.fault);
  }
}

function step(count = 1) {
  if (!model || !data) throw new Error('No MuJoCo model is loaded');
  if (!Number.isInteger(count) || count < 1 || count > MAX_STEP_BATCH) throw new RangeError(`step count must be an integer from 1 to ${MAX_STEP_BATCH}`);
  if (paused) return observation();
  for (let index = 0; index < count; index += 1) doPhysicsStep();
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
    doPhysicsStep();
    if (index === count || index % sampleEverySteps === 0) observations.push(observation());
  }
  return { observations, executedSteps: count, sampleEverySteps };
}

function command(payload = {}) {
  if (!model || !data || !descriptor || !modelInfo) throw new Error('No MuJoCo model is loaded');
  if (payload.type === 'engage_stand') {
    if (!actuatorPlant || !descriptor.standingControllerId || payload.controllerId !== descriptor.standingControllerId) throw new Error('Standing controller is only declared in its matching experimental scene');
    if (Object.keys(payload).some(k=>!['type','controllerId'].includes(k))) throw new Error('Unexpected standing command field');
    if (standingEvaluator.startedAt != null) throw new Error('Reset explicitly before starting a new standing trial');
    if (!actuationEnabled) throw new Error('Standing cannot engage with actuation disabled');
    accepted=holdCommandsAt(ASIMOV_SOURCE.joints.map(j=>j.referenceRad));
    controller={kind:'standing',profile:sensorStanding?ASIMOV_SENSOR_STANDING_CONTROLLER:ASIMOV_STANDING_CONTROLLER,startedAtSeconds:Number(data.time)};
    sensorStanding?.reset(Number(data.time));
    standingEvaluator.start(Number(data.time),rootObservation());
  } else if (payload.type === 'release_stand') {
    if (!descriptor.standingControllerId) throw new Error('No standing controller in this scene');
    accepted=holdCommandsAt(ASIMOV_JOINT_ORDER.map((_,i)=>measured(i).positionRad));
    controller=null; commandProfile=ASIMOV_JOINT_HOLD_PROFILE;
  } else if (payload.type === 'set_standing_targets') {
    if (controller?.kind!=='standing' || standingEvaluator.failure || sensorStanding?.fault) throw new Error('Upper-body targets require an active, nonfailed standing trial');
    if(Object.keys(payload).some(k=>!['type','targetsRad'].includes(k)))throw new Error('Unexpected standing target field');
    const targets=payload.targetsRad;
    if(!targets||typeof targets!=='object'||Array.isArray(targets)||!Object.keys(targets).length)throw new Error('Nonempty upper-body targets required');
    const allowed=new Set(ASIMOV_JOINT_ORDER.slice(12));
    for(const [id,v] of Object.entries(targets))if(!allowed.has(id)||!Number.isFinite(v)||v<jointState.get(id).range[0]||v>jointState.get(id).range[1])throw new Error('Standing targets accept bounded waist/arm joints only');
    const next=holdCommandsAt(ASIMOV_JOINT_ORDER.map((id,i)=>Object.hasOwn(targets,id)?targets[id]:accepted[i].positionRad));
    accepted=next; // Keep balance regulator/evaluator; a subsequent fall remains a failure.
  } else if (payload.type === 'set_joint_targets') {
    const targets = payload.targetsRad;
    if (!targets || typeof targets !== 'object' || Array.isArray(targets) || !Object.keys(targets).length) throw new Error('set_joint_targets requires a non-empty targetsRad object');
    for (const jointId of Object.keys(targets)) if (!jointState.has(jointId)) throw new Error(`Unknown declared joint ${jointId}`);
    const positions = ASIMOV_JOINT_ORDER.map((jointId, index) => (jointId in targets ? targets[jointId] : accepted[index].positionRad));
    for (const value of positions) if (typeof value !== 'number' || !Number.isFinite(value)) throw new TypeError('Every joint target must be finite radians');
    const nextAccepted = holdCommandsAt(positions);
    // Validate the complete request before changing the active controller or command vector.
    // A rejected command must not change the active command vector or controller identity.
    controller = null;
    commandProfile = ASIMOV_JOINT_HOLD_PROFILE;
    accepted = nextAccepted;
  } else if (payload.type === 'set_lowlevel_targets') {
    const commands = payload.commands;
    if (!commands || typeof commands !== 'object' || Array.isArray(commands) || !Object.keys(commands).length) throw new Error('set_lowlevel_targets requires a non-empty commands object');
    for (const jointId of Object.keys(commands)) if (!jointState.has(jointId)) throw new Error(`Unknown declared joint ${jointId}`);
    const nextAccepted = ASIMOV_JOINT_ORDER.map((jointId, index) => {
      const request = commands[jointId];
      if (!(jointId in commands)) return accepted[index];
      if (!request || typeof request !== 'object' || Array.isArray(request)) throw new TypeError(`Low-level command ${jointId} must be an object`);
      for (const key of Object.keys(request)) if (!['positionRad','velocityRadS','feedforwardTorqueNm','kp','kd'].includes(key)) throw new RangeError(`Unknown low-level field ${key}`);
      return boundLowLevelCommand(jointId, {
        positionRad: request.positionRad ?? accepted[index].positionRad,
        velocityRadS: request.velocityRadS ?? 0,
        feedforwardTorqueNm: request.feedforwardTorqueNm ?? 0,
        kp: request.kp ?? ASIMOV_JOINT_HOLD_PROFILE.kp[index],
        kd: request.kd ?? ASIMOV_JOINT_HOLD_PROFILE.kd[index],
      });
    });
    controller = null;
    commandProfile = ASIMOV_LOWLEVEL_COMMAND_PROFILE;
    accepted = nextAccepted;
  } else {
    throw new Error(`Unsupported physical command: ${payload.type}`);
  }
  if (standingEvaluator.startedAt != null) standingEvaluator.update(Number(data.time),rootObservation(),readContacts(),actuationEnabled,controller?.kind==='standing' && !sensorStanding?.fault);
  return observation();
}

// The declared setup allowlist. It contains exactly one operation, and that operation removes
// capability rather than adding it. There is no setup path here that places the robot, moves the
// root, applies an external force, or attaches anything.
function applySetup(payload = {}) {
  if (!model || !data || !descriptor) throw new Error('No MuJoCo model is loaded');
  if (payload.type !== 'set_actuation') throw new Error(`Unsupported declared setup operation: ${payload.type}`);
  if (typeof payload.enabled !== 'boolean') throw new TypeError('enabled must be boolean');
  const enabled = payload.enabled;
  actuationEnabled = enabled;
  if (!enabled) actuatorPlant?.disable();
  for (const actuator of actuatorState.values()) if (!enabled) data.ctrl[actuator.id] = 0;
  logSetup({ event: 'set_actuation', enabled, detail: enabled ? 'actuator effort re-enabled' : 'every actuator produces zero effort; the controller keeps running and keeps issuing bounded commands' });
  mujoco.mj_forward(model, data);
  if (standingEvaluator.startedAt != null) standingEvaluator.update(Number(data.time),rootObservation(),readContacts(),actuationEnabled,controller?.kind==='standing' && !sensorStanding?.fault);
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
    else if (op === 'setup') result = applySetup(payload);
    else if (op === 'pause') { paused = true; result = observation(); }
    else if (op === 'resume') { paused = false; result = observation(); }
    else if (op === 'dispose') { disposeModel(); result = true; }
    else throw new Error(`Unknown worker operation: ${op}`);
    reply(id, true, result);
  } catch (error) { if(op==='load') disposeModel(); reply(id, false, null, String(error?.stack || error?.message || error)); }
};
