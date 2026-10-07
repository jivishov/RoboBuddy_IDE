import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

import {
  G1_EFFORT_LIMIT_NM, G1_FOOT_CONTACT_GEOMS, G1_JOINT_AXIS, G1_JOINT_ORDER, G1_JOINT_RANGE_RAD,
  G1_SELF_CONTACT_EXCLUSIONS, G1_TOTAL_MASS_KG, G1_VELOCITY_LIMIT_RAD_S, MENAGERIE_G1_REFERENCE,
  G1_URDF_JOINT_RANGE_RAD, G1_URDF_MJCF_MAX_RANGE_DELTA_RAD, G1_URDF_MJCF_RANGE_DISCREPANCIES,
  UNITREE_G1_RECONCILIATION, UNITREE_MUJOCO_SOURCE, UNITREE_RL_MJLAB_SOURCE, UNITREE_ROS_SOURCE,
  assertReconciliationCoverage, reconciliationByEvidence,
} from '../../src/physics/unitree-g1-source-audit.js';
import {
  G1_ANKLE_STABILITY_REQUIREMENT_NM_PER_RAD, G1_CONTROLLERS, G1_JOINT_HOLD_PROFILE, G1_MAX_KD, G1_MAX_KP,
  G1_ROBOBUDDY_ANKLE_KD, G1_ROBOBUDDY_ANKLE_KP,
  G1_LOWLEVEL_CONTROL_INTERVAL_SECONDS, G1_PHYSICS_TIMESTEP_SECONDS, G1_STAND_CONTROLLER_PROFILES,
  G1_STAND_POSE_RAD, ROBOBUDDY_STAND_KD, ROBOBUDDY_STAND_KP, UNITREE_FIXSTAND_KD, UNITREE_FIXSTAND_KP,
  UNITREE_FIXSTAND_RAMP_SECONDS, ankleStiffnessAudit, assertControllerTables, boundLowLevelCommand,
  lowLevelTorqueNm, standCommands, standTargetRad,
} from '../../src/physics/unitree-g1-controller.js';
import {
  UNITREE_G1_BLOCKED_JOINT, UNITREE_G1_EXTERNAL_OBJECT, UNITREE_G1_FREEBASE_DROP_PACKAGE,
  UNITREE_G1_FREEBASE_PACKAGE, UNITREE_G1_MODEL_PACKAGES, UNITREE_G1_MOUNTED_BLOCKED_PACKAGE,
  UNITREE_G1_MOUNTED_PACKAGE, UNITREE_G1_ROBOT_ID, UNITREE_G1_STAND_PELVIS_Z_M,
} from '../../src/physics/unitree-g1-model-package.js';
import { UNITREE_G1_SCENES, UNITREE_G1_STAND_GATE, UnitreeG1StandEvaluator } from '../../src/physics/unitree-g1-scene.js';
import { UNITREE_G1_CAPABILITY_AUDIT, assertCapabilityAudit, unitreeG1CapabilityById } from '../../src/physics/unitree-g1-capabilities.js';
import { UNITREE_G1_CONTROL_LIMITS, createUnitreeG1ControlSchema } from '../../src/webmcp/unitree-g1-physical-control.js';
import { tasksForProfile } from '../../src/task-catalog.js';
import { physicsCapabilityFor } from '../../src/physics/capabilities.js';
import { PARAMETER_EVIDENCE } from '../../src/physics/model-registry.js';

const REPO = resolve(dirname(fileURLToPath(import.meta.url)), '../..');
const read = (path) => readFileSync(resolve(REPO, path), 'utf8');
const sha256 = (path) => createHash('sha256').update(readFileSync(resolve(REPO, path))).digest('hex');

// --- 1. source identity and reconciliation -----------------------------------------------------
assert.equal(UNITREE_ROS_SOURCE.revision, 'dd4fa6866e523ad61324f658d63736e4eda3a6e4', 'the pinned Unitree model revision must not drift');
assert.match(UNITREE_ROS_SOURCE.variant, /29-DoF, fixed rubber hands/);
assert.equal(sha256('models/unitree_g1/source/g1_29dof.xml'), UNITREE_ROS_SOURCE.mjcfSha256, 'vendored source MJCF hash');
assert.equal(sha256('models/unitree_g1/source/g1_29dof.urdf'), UNITREE_ROS_SOURCE.urdfSha256, 'vendored source URDF hash');
assert.equal(sha256('models/unitree_g1/source/g1_29dof_unitree_mujoco.xml'), UNITREE_MUJOCO_SOURCE.mjcfSha256, 'vendored unitree_mujoco MJCF hash');
assert.equal(MENAGERIE_G1_REFERENCE.status, 'not-adopted', 'Menagerie is a different G1 revision and must stay unadopted');
assert.ok(assertReconciliationCoverage() >= 19);
assert.ok(reconciliationByEvidence('calibration-required').some((row) => row.parameter === 'hardware alignment'));
assert.ok(reconciliationByEvidence('estimated').some((row) => row.parameter === 'standing controller ankle gains'),
  'the repository-authored ankle gains must be declared estimated, never source-derived');

// The URDF and the MJCF must agree on all 29 joints, and the audit table must equal both.
{
  const urdf = read('models/unitree_g1/source/g1_29dof.urdf');
  const mjcf = read('models/unitree_g1/source/g1_29dof.xml');
  for (const [index, jointId] of G1_JOINT_ORDER.entries()) {
    const urdfJoint = new RegExp(`<joint name="${jointId}"[\\s\\S]*?</joint>`).exec(urdf);
    assert.ok(urdfJoint, `${jointId} must exist in the source URDF`);
    const limit = /<limit[^>]*effort="([^"]+)"[^>]*velocity="([^"]+)"[^>]*lower="([^"]+)"[^>]*upper="([^"]+)"/.exec(urdfJoint[0])
      || /<limit[^>]*lower="([^"]+)"[^>]*upper="([^"]+)"[^>]*effort="([^"]+)"[^>]*velocity="([^"]+)"/.exec(urdfJoint[0]);
    assert.ok(limit, `${jointId} must declare a URDF limit`);
    const [lower, upper, effort, velocity] = limit[0].includes('effort="') && limit[0].indexOf('effort=') < limit[0].indexOf('lower=')
      ? [Number(limit[3]), Number(limit[4]), Number(limit[1]), Number(limit[2])]
      : [Number(limit[1]), Number(limit[2]), Number(limit[3]), Number(limit[4])];
    assert.equal(lower, G1_URDF_JOINT_RANGE_RAD[index][0], `${jointId} URDF lower limit`);
    assert.equal(upper, G1_URDF_JOINT_RANGE_RAD[index][1], `${jointId} URDF upper limit`);
    // The audited URDF/MJCF rounding: it exists on exactly six wrist joints and nowhere else.
    const delta = Math.max(Math.abs(lower - G1_JOINT_RANGE_RAD[index][0]), Math.abs(upper - G1_JOINT_RANGE_RAD[index][1]));
    if (G1_URDF_MJCF_RANGE_DISCREPANCIES.includes(jointId)) {
      assert.ok(delta > 0 && delta <= G1_URDF_MJCF_MAX_RANGE_DELTA_RAD, `${jointId} URDF/MJCF rounding must stay within the audited bound`);
    } else {
      assert.equal(delta, 0, `${jointId} must agree exactly between URDF and MJCF`);
    }
    assert.equal(effort, G1_EFFORT_LIMIT_NM[index], `${jointId} URDF effort limit`);
    assert.equal(velocity, G1_VELOCITY_LIMIT_RAD_S[index], `${jointId} URDF velocity limit`);
    const mjcfJoint = new RegExp(`<joint name="${jointId}"[^/]*?/>`).exec(mjcf);
    assert.ok(mjcfJoint, `${jointId} must exist in the source MJCF`);
    assert.ok(mjcfJoint[0].includes(`range="${G1_JOINT_RANGE_RAD[index][0]} ${G1_JOINT_RANGE_RAD[index][1]}"`), `${jointId} MJCF range must equal the URDF range`);
    assert.ok(mjcfJoint[0].includes(`actuatorfrcrange="-${G1_EFFORT_LIMIT_NM[index]} ${G1_EFFORT_LIMIT_NM[index]}"`), `${jointId} MJCF actuator force range must equal the URDF effort limit`);
    assert.ok(mjcfJoint[0].includes(`axis="${G1_JOINT_AXIS[index].join(' ')}"`), `${jointId} MJCF axis must equal the audited axis`);
    // The audited discrepancy: the MJCF carries no velocity limiter at all.
    assert.ok(!mjcfJoint[0].includes('velocity'), `${jointId} MJCF must not carry a velocity limit; the command layer re-imposes the URDF bound`);
  }
  assert.ok(/left_rubber_hand/.test(mjcf) && /right_rubber_hand/.test(mjcf), 'the selected variant must be the fixed-rubber-hand G1');
  assert.ok(!/hand_index|hand_thumb|hand_middle/.test(mjcf), 'the selected variant must not be the dexterous-hand G1');
  // The hands must be visual-only in the source, which is what makes "dexterous hands unsupported" a fact.
  for (const hand of ['left_rubber_hand', 'right_rubber_hand']) {
    const geoms = [...mjcf.matchAll(new RegExp(`<geom[^>]*mesh="${hand}"[^>]*/>`, 'g'))].map((match) => match[0]);
    assert.ok(geoms.length > 0, `${hand} must appear in the source model`);
    for (const geom of geoms) assert.ok(geom.includes('contype="0"') && geom.includes('conaffinity="0"'), `${hand} must be visual-only in the source`);
  }
}

// --- 2. controller contract --------------------------------------------------------------------
assert.ok(assertControllerTables());
assert.equal(G1_JOINT_ORDER.length, 29);
assert.equal(UNITREE_FIXSTAND_RAMP_SECONDS, 2, 'the source FixStand ramp is ts = [0, 2]');
assert.deepEqual([...G1_STAND_POSE_RAD.slice(0, 6)], [-0.1, 0, 0, 0.3, -0.2, 0], 'the standing posture is the source FixStand qs');

// The repository standing profile may differ from source FixStand only at the four ankle gains.
{
  const differing = ROBOBUDDY_STAND_KP.map((value, index) => (value === UNITREE_FIXSTAND_KP[index] && ROBOBUDDY_STAND_KD[index] === UNITREE_FIXSTAND_KD[index] ? null : index)).filter((index) => index != null);
  assert.deepEqual(differing, [4, 5, 10, 11], 'only the ankle pitch/roll gains are repository-authored');
}

// The measured free-base failure of the source controller is a property of the gains, and the
// criterion that predicts it is asserted here so neither can drift silently.
{
  const source = ankleStiffnessAudit(G1_STAND_CONTROLLER_PROFILES[G1_CONTROLLERS.SOURCE_FIXSTAND]);
  const repository = ankleStiffnessAudit(G1_STAND_CONTROLLER_PROFILES[G1_CONTROLLERS.STAND]);
  assert.equal(source.totalAnkleKpNmPerRad, 80);
  assert.equal(repository.totalAnkleKpNmPerRad, 500);
  assert.ok(Math.abs(G1_ANKLE_STABILITY_REQUIREMENT_NM_PER_RAD - G1_TOTAL_MASS_KG * 9.81 * 0.6715) < 1e-9);
  assert.equal(source.passesRigidPendulumHeuristic, false, 'source FixStand stiffness falls below the rigid-pendulum heuristic, not a proof of failure');
  assert.equal(repository.passesRigidPendulumHeuristic, true);
  assert.ok(repository.marginRatio > 2);
  assert.match(G1_STAND_CONTROLLER_PROFILES[G1_CONTROLLERS.SOURCE_FIXSTAND].claim, /NOT to maintain free-base posture/);
  assert.match(G1_STAND_CONTROLLER_PROFILES[G1_CONTROLLERS.STAND].claim, /Not dynamic balance, not perturbation recovery/);
  assert.match(G1_STAND_CONTROLLER_PROFILES[G1_CONTROLLERS.STAND].gainProvenance, /ankle gains are repository-authored/);
}

// Bounded command: requested, accepted and the source bound are three separate reported facts.
{
  const knee = boundLowLevelCommand('left_knee_joint', { positionRad: 6, velocityRadS: 500, feedforwardTorqueNm: 1e4, kp: 1e5, kd: 1e5 });
  assert.equal(knee.requested.positionRad, 6);
  assert.equal(knee.positionRad, G1_JOINT_RANGE_RAD[3][1]);
  assert.equal(knee.velocityRadS, G1_VELOCITY_LIMIT_RAD_S[3]);
  assert.equal(knee.feedforwardTorqueNm, G1_EFFORT_LIMIT_NM[3]);
  assert.equal(knee.kp, G1_MAX_KP);
  assert.equal(knee.kd, G1_MAX_KD);
  assert.equal(knee.bounded, true);
  assert.equal(knee.limits.velocityLimitRadS, 20, 'the source URDF velocity limit is re-imposed at command level');
  assert.throws(() => boundLowLevelCommand('nope_joint', { positionRad: 0 }), /Unknown Unitree G1 joint/);
  assert.throws(() => boundLowLevelCommand('left_knee_joint', { positionRad: Number.NaN }), /must be finite/);
}

// The low-level law is exactly the Unitree bridge expression, clamped to the source effort limit.
{
  const command = boundLowLevelCommand('left_knee_joint', { positionRad: 1, velocityRadS: 0, feedforwardTorqueNm: 2, kp: 10, kd: 1 });
  assert.equal(lowLevelTorqueNm(command, { positionRad: 0.5, velocityRadS: 0.2 }), 2 + 10 * 0.5 + 1 * -0.2);
  const saturating = boundLowLevelCommand('left_wrist_pitch_joint', { positionRad: 1, kp: 400 });
  assert.equal(lowLevelTorqueNm(saturating, { positionRad: -1, velocityRadS: 0 }), G1_EFFORT_LIMIT_NM[20], 'torque is clamped to the source effort limit');
  assert.equal(lowLevelTorqueNm(saturating, { positionRad: 1.6, velocityRadS: 0 }), -G1_EFFORT_LIMIT_NM[20]);
}

// The standing ramp is exactly Unitree's linear interpolator over ts = [0, ramp].
{
  const profile = G1_STAND_CONTROLLER_PROFILES[G1_CONTROLLERS.STAND];
  const start = G1_JOINT_ORDER.map(() => 0);
  assert.deepEqual(standTargetRad(profile, start, 0), start);
  assert.deepEqual(standTargetRad(profile, start, 10), [...G1_STAND_POSE_RAD]);
  const half = standTargetRad(profile, start, UNITREE_FIXSTAND_RAMP_SECONDS / 2);
  assert.ok(Math.abs(half[3] - G1_STAND_POSE_RAD[3] / 2) < 1e-12);
  const commands = standCommands(profile, start, 10);
  assert.equal(commands.length, 29);
  assert.equal(commands[4].kp, 250);
  assert.equal(commands[4].kd, 10);
  assert.equal(commands[0].kp, UNITREE_FIXSTAND_KP[0], 'non-ankle gains stay at the Unitree source values');
  for (const command of commands) assert.equal(command.velocityRadS, 0, 'FixStand commands dq_target = 0');
  for (const command of commands) assert.equal(command.feedforwardTorqueNm, 0, 'FixStand commands tau_ff = 0');
}
assert.equal(G1_JOINT_HOLD_PROFILE.kp, UNITREE_FIXSTAND_KP, 'the default joint-hold gains are the unchanged Unitree source gains');
{
  // The provenance record must state the gains that actually ship, or it is a false record of the
  // one deviation this workspace declares.
  const row = UNITREE_G1_RECONCILIATION.find((item) => item.parameter === 'standing controller ankle gains');
  assert.ok(row, 'the reconciliation table must record the ankle gain deviation');
  assert.deepEqual([...row.value], [G1_ROBOBUDDY_ANKLE_KP, G1_ROBOBUDDY_ANKLE_KD],
    'the recorded ankle gains must equal the shipped ankle gains');
  assert.equal(row.evidence, PARAMETER_EVIDENCE.ESTIMATED, 'a repository-authored gain is never source-derived');
}

// --- 3. model packages and generated models -----------------------------------------------------
assert.equal(UNITREE_G1_MODEL_PACKAGES.length, 4);
for (const modelPackage of UNITREE_G1_MODEL_PACKAGES) {
  assert.equal(modelPackage.robotId, UNITREE_G1_ROBOT_ID);
  assert.equal(modelPackage.joints.length, 29);
  assert.equal(modelPackage.actuators.length, 29);
  assert.equal(sha256(modelPackage.asset), modelPackage.sha256, `${modelPackage.id} asset hash must match the registry`);
  assert.equal(modelPackage.physics.timestepSeconds, G1_PHYSICS_TIMESTEP_SECONDS);
  assert.equal(modelPackage.physics.integrator, 'Euler');
  for (const [index, joint] of modelPackage.joints.entries()) {
    assert.equal(joint.id, G1_JOINT_ORDER[index], 'model package joint order must be the audited motor order');
    assert.deepEqual(joint.rangeRad, [...G1_JOINT_RANGE_RAD[index]]);
    assert.equal(joint.effortLimitNm, G1_EFFORT_LIMIT_NM[index]);
    assert.equal(joint.velocityLimitRadS, G1_VELOCITY_LIMIT_RAD_S[index]);
  }
  for (const actuator of modelPackage.actuators) assert.equal(actuator.command, 'torque-nm', 'G1 actuators are direct-torque motors, never position servos');
  assert.ok(modelPackage.limitations.some((line) => /fixed rubber hands are visual-only/.test(line)));
  assert.ok(modelPackage.limitations.some((line) => /never a measurement|No measurement of an assembled Unitree G1/.test(line)));
  assert.equal(modelPackage.evidence.standingControllerAnkleGains, 'estimated');
  assert.equal(modelPackage.evidence.hardwareAlignment, 'calibration-required');
}
// The declared initial command is a command, and each scene declares the one it needs.
assert.equal(UNITREE_G1_MOUNTED_PACKAGE.initialCommand, 'joint-hold');
assert.equal(UNITREE_G1_MOUNTED_BLOCKED_PACKAGE.initialCommand, 'joint-hold');
assert.equal(UNITREE_G1_FREEBASE_PACKAGE.initialCommand, 'joint-hold');
assert.equal(UNITREE_G1_FREEBASE_DROP_PACKAGE.initialCommand, 'passive', 'the gravity-release fixture must hold nothing up');
for (const modelPackage of UNITREE_G1_MODEL_PACKAGES) assert.ok(['joint-hold', 'passive'].includes(modelPackage.initialCommand));
assert.ok(UNITREE_G1_FREEBASE_PACKAGE.limitations.some((line) => /source gains lose the posture within about half a second/.test(line)),
  'the free-base package must state that its default hold is not a standing controller');
assert.equal(UNITREE_G1_MOUNTED_PACKAGE.rootMode, 'fixed-mounted');
assert.equal(UNITREE_G1_MOUNTED_BLOCKED_PACKAGE.rootMode, 'fixed-mounted');
assert.equal(UNITREE_G1_FREEBASE_PACKAGE.rootMode, 'free-base');
assert.equal(UNITREE_G1_FREEBASE_DROP_PACKAGE.rootMode, 'free-base');
assert.deepEqual(UNITREE_G1_FREEBASE_PACKAGE.sceneConstraints.objects, [UNITREE_G1_EXTERNAL_OBJECT], 'the free-base scene declares an external physical object');
assert.ok(UNITREE_G1_FREEBASE_PACKAGE.controllers.includes(G1_CONTROLLERS.STAND));
assert.ok(!UNITREE_G1_MOUNTED_PACKAGE.controllers.includes(G1_CONTROLLERS.STAND), 'a mounted fixture must not offer a standing controller');
assert.equal(UNITREE_G1_FREEBASE_PACKAGE.bodies.find((body) => body.id === 'pelvis').freeJointId, 'floating_base_joint');
assert.equal(UNITREE_G1_MOUNTED_PACKAGE.bodies.find((body) => body.id === 'pelvis').freeJointId, undefined);

// The generated MJCF variants must carry exactly the structure the audit claims.
{
  const models = Object.fromEntries(['mounted', 'mounted_blocked', 'freebase', 'freebase_drop'].map((name) => [name, read(`models/unitree_g1/${name}.xml`)]));
  for (const [name, xml] of Object.entries(models)) {
    assert.ok(!/<equality>/.test(xml), `${name} must declare no equality constraint`);
    assert.ok(!/<tendon>/.test(xml), `${name} must declare no tendon`);
    assert.ok(!/<weld\b/.test(xml), `${name} must declare no weld`);
    assert.ok(/<option timestep="0.002" integrator="Euler" iterations="100" ls_iterations="50"/.test(xml), `${name} numerical settings`);
    assert.ok(/<keyframe>/.test(xml), `${name} must declare its initial condition as a model keyframe`);
    assert.ok(/geom name="floor"/.test(xml), `${name} must have a floor`);
    for (const side of ['left', 'right']) for (const geom of G1_FOOT_CONTACT_GEOMS[side]) assert.ok(xml.includes(`name="${geom}"`), `${name} must name ${geom}`);
    for (const [body1, body2] of G1_SELF_CONTACT_EXCLUSIONS.map((item) => item.bodies)) {
      assert.ok(xml.includes(`body1="${body1}" body2="${body2}"`), `${name} must carry the justified exclusion ${body1}/${body2}`);
    }
    for (const [index, jointId] of G1_JOINT_ORDER.entries()) {
      assert.ok(xml.includes(`<motor name="${jointId}" joint="${jointId}" ctrlrange="-${G1_EFFORT_LIMIT_NM[index]} ${G1_EFFORT_LIMIT_NM[index]}"`), `${name} motor ${jointId} must be bounded to the source effort limit`);
    }
    assert.ok(/armature="0.01" damping="0.05" frictionloss="0.2"/.test(xml), `${name} must carry the unitree_mujoco dynamic augmentation`);
    assert.ok(/armature="0.01" damping="0.05" frictionloss="0.1"/.test(xml), `${name} must carry the wrist friction-loss class`);
  }
  assert.ok(/name="floating_base_joint" type="free"/.test(models.freebase), 'the free-base scene must have a free pelvis');
  assert.ok(/name="floating_base_joint" type="free"/.test(models.freebase_drop));
  assert.ok(!/floating_base_joint/.test(models.mounted), 'the mounted fixture must have no root joint at all');
  assert.ok(!/floating_base_joint/.test(models.mounted_blocked));
  assert.ok(models.mounted.includes('body1="pelvis" body2="left_hip_pitch_link"'), 'the mounted fixture restores MuJoCo parent filtering the world weld disables');
  assert.ok(!models.freebase.includes('body1="pelvis" body2="left_hip_pitch_link"'), 'the free-base scene must not carry the mounted-only exclusions');
  assert.ok(models.freebase.includes(`name="${UNITREE_G1_EXTERNAL_OBJECT}"`), 'the free-base scene must declare its external object');
  assert.ok(models.mounted_blocked.includes('name="hip_abduction_stop_wall"'), 'the blocked fixture must declare its stop geometry');
  assert.ok(!models.mounted.includes('hip_abduction_stop_wall'), 'the unobstructed mount must not carry the stop wall');
  const freebaseKey = /<key name="initial" qpos="([^"]+)"/.exec(models.freebase);
  assert.ok(freebaseKey, 'the free-base scene declares an initial keyframe');
  const qpos = freebaseKey[1].split(' ').map(Number);
  assert.equal(qpos[2], UNITREE_G1_STAND_PELVIS_Z_M, 'the free-base initial pelvis height is the source-derived stand height');
  assert.deepEqual(qpos.slice(7, 36), [...G1_STAND_POSE_RAD], 'the free-base initial pose is the source standing posture');
}

// The committed hull assets must match the manifest, and the manifest must pin the source.
{
  const manifest = JSON.parse(read('models/unitree_g1/source/MESH_MANIFEST.json'));
  assert.equal(manifest.upstream.revision, UNITREE_ROS_SOURCE.revision);
  assert.equal(manifest.meshes.length, 25, 'the source declares 25 collision meshes');
  for (const row of manifest.meshes) {
    assert.equal(sha256(`models/unitree_g1/assets/${row.mesh}`), row.hullSha256, `${row.mesh} hull hash`);
    assert.ok(row.hullBytes < row.sourceBytes);
    assert.match(row.sourceSha256, /^[0-9a-f]{64}$/);
  }
  assert.match(manifest.derivation, /collision-identical/);
}

// --- 4. the authoritative worker must contain no state-writing path -----------------------------
{
  const worker = read('src/physics/unitree-g1-mujoco-worker.js');
  // Reads of qpos/qvel are how the plant is observed. What must not exist is an assignment to
  // them, or any external-force channel at all.
  for (const field of ['qpos', 'qvel', 'xpos', 'xquat', 'body_pos', 'body_quat']) {
    const write = new RegExp(`(data|model)\\.${field}\\s*\\[[^\\]]*\\]\\s*(=|\\+=|-=|\\*=)[^=]`);
    assert.ok(!write.test(worker), `the G1 worker must contain no assignment to ${field}`);
  }
  for (const forbidden of ['xfrc_applied', 'qfrc_applied', 'mju_copy', 'mj_setState', 'applyForce']) {
    assert.ok(!worker.includes(forbidden), `the G1 worker must contain no ${forbidden} path`);
  }
  // Exactly one array in the worker is ever assigned, and it is the actuator command.
  const assignments = [...worker.matchAll(/data\.(\w+)\s*\[[^\]]*\]\s*=[^=]/g)].map((match) => match[1]);
  assert.deepEqual([...new Set(assignments)], ['ctrl'], `the G1 worker may only ever write d.ctrl, found writes to ${[...new Set(assignments)].join(', ')}`);
  assert.ok(worker.includes('data.ctrl[actuator.id] = actuationEnabled ? lowLevelTorqueNm('), 'the only influence on the plant is a bounded actuator torque');
  assert.ok(worker.includes('mj_resetDataKeyframe'), 'the declared initial condition comes from the model keyframe');
  assert.ok(worker.includes("descriptor.initialCommand === 'passive'"), 'the worker honours the declared initial command');
  assert.ok(/if \(equalityCount !== 0\)/.test(worker), 'the worker rejects any equality constraint');
  assert.ok(/gravity\[2\] < -9/.test(worker), 'the worker rejects a scene without real gravity');
  assert.ok(worker.includes("payload.type !== 'set_actuation'"), 'the declared setup allowlist is exactly one operation');
  for (const forbidden of ['set_root_pose', 'set_root_velocity', 'force_upright', 'walk']) {
    assert.ok(!worker.includes(forbidden), `the G1 worker must expose no ${forbidden} operation`);
  }
}

// --- 5. scenes, gate and evaluator ---------------------------------------------------------------
assert.equal(UNITREE_G1_SCENES.freebase.modelPackage, UNITREE_G1_FREEBASE_PACKAGE.id);
assert.equal(UNITREE_G1_STAND_GATE.evaluationStartSeconds, UNITREE_FIXSTAND_RAMP_SECONDS, 'the evaluated interval starts only after the controller ramp');
assert.equal(UNITREE_G1_STAND_GATE.evaluationSeconds, 6);
assert.deepEqual(UNITREE_G1_STAND_GATE.pelvisHeightRangeM, [UNITREE_G1_STAND_PELVIS_Z_M - 0.05, UNITREE_G1_STAND_PELVIS_Z_M + 0.05]);
assert.equal(UNITREE_G1_STAND_GATE.requireBothFeetSupported, true);
assert.equal(UNITREE_G1_STAND_GATE.forbidNonFootGroundContact, true);
assert.equal(UNITREE_G1_STAND_GATE.forbidExternalSupport, true);

{
  const contact = (geoms) => ({ geoms, bodies: ['a', 'b'], distanceM: -0.001, normalForceN: 10 });
  const sample = (seconds, { z = 0.7809, tilt = 0.05, feet = true, nonFoot = 0, external = 0, effort = 5, actuation = true } = {}) => ({
    simulationTimeSeconds: seconds,
    contactsReadable: true, engine: { gravity: [0, 0, -9.81] },
    root: { mode: 'free-base', free: true, positionM: [0.01, 0, z], quaternionWxyz: [1, 0, 0, 0], linearVelocityMS: [0.001, 0, 0], angularVelocityRadS: [0.001, 0, 0], uprightZ: Math.cos(tilt), tiltRad: tilt },
    joints: Object.fromEntries(G1_JOINT_ORDER.map((id, index) => [id, { effortNm: id === 'left_knee_joint' ? effort : 0, effortLimitNm: G1_EFFORT_LIMIT_NM[index] }])),
    contactClasses: {
      leftFootFloor: feet ? [contact(['left_foot_toe_medial', 'floor'])] : [],
      rightFootFloor: feet ? [contact(['right_foot_toe_medial', 'floor'])] : [],
      otherBodyFloor: Array.from({ length: nonFoot }, () => contact(['torso_link_hull', 'floor'])),
      robotSelf: [], robotExternalObject: Array.from({ length: external }, () => contact(['torso_link_hull', 'contact_probe_block_geom'])), robotFixture: [],
    },
    controller: { id: G1_CONTROLLERS.STAND },
    actuationEnabled: actuation,
  });
  const feed = (evaluator, options = {}) => {
    for (let t = 0; t <= 8.0001; t += 0.02) evaluator.observe(sample(Number(t.toFixed(4)), options));
    return evaluator.snapshot();
  };
  assert.equal(feed(new UnitreeG1StandEvaluator()).standing, true, 'a clean nominal run stands');
  // The old evaluator accepted two snapshots six seconds apart and silently treated missing
  // velocities/effort as zero. Neither is sufficient evidence of a continuously held stand.
  const sparse = new UnitreeG1StandEvaluator();
  sparse.observe(sample(2)); sparse.observe(sample(8));
  assert.equal(sparse.snapshot().standing, false, 'two endpoint poses are not six seconds of standing evidence');
  for (const corrupt of [
    (o) => { delete o.root.angularVelocityRadS; },
    (o) => { o.root.linearVelocityMS[0] = NaN; },
    (o) => { o.root.mode = 'fixed-mounted'; o.root.free = false; },
    (o) => { delete o.joints.left_knee_joint; },
    (o) => { o.contactsReadable = false; },
    (o) => { o.engine.gravity = [0, 0, 0]; },
    (o) => { for (const side of ['leftFootFloor', 'rightFootFloor']) for (const c of o.contactClasses[side]) c.normalForceN = 0; },
  ]) {
    const evaluator = new UnitreeG1StandEvaluator();
    for (let i = 0; i <= 400; ++i) { const o = sample(i * .02); corrupt(o); evaluator.observe(o); }
    assert.equal(evaluator.snapshot().standing, false, 'incomplete or unsupported observations cannot earn standing');
  }

  assert.equal(feed(new UnitreeG1StandEvaluator(), { feet: false }).checks.bothFeetSupportedThroughout, false);
  assert.equal(feed(new UnitreeG1StandEvaluator(), { nonFoot: 3 }).checks.noNonFootGroundContact, false);
  assert.equal(feed(new UnitreeG1StandEvaluator(), { external: 1 }).checks.noExternalOrFixtureSupport, false,
    'contact with the external object during the evaluated interval disqualifies the stand');
  assert.equal(feed(new UnitreeG1StandEvaluator(), { z: 0.60 }).checks.pelvisHeightInBand, false);
  assert.equal(feed(new UnitreeG1StandEvaluator(), { tilt: 0.4 }).checks.tiltWithinLimit, false);
  assert.equal(feed(new UnitreeG1StandEvaluator(), { effort: 138 }).checks.actuatorEffortWithinFraction, false);
  const disabled = feed(new UnitreeG1StandEvaluator(), { actuation: false });
  assert.equal(disabled.actuationDisabledSeen, true, 'a motors-disabled run is always reported as such');
  // A short run cannot pass by never reaching the evaluated interval.
  const short = new UnitreeG1StandEvaluator();
  for (let t = 0; t <= 2.5; t += 0.02) short.observe(sample(Number(t.toFixed(4))));
  assert.equal(short.snapshot().checks.evaluationWindowReached, false);
  assert.equal(short.snapshot().standing, false);

  // The evaluated interval opens one ramp after the controller was engaged, not one ramp after
  // the clock started, so settling first cannot let the ramp tail count as held posture.
  {
    const engaged = 1.5;
    const withEngagement = (seconds) => {
      const record = sample(seconds);
      record.controller = { id: G1_CONTROLLERS.STAND, engagedAtSeconds: engaged };
      return record;
    };
    const late = new UnitreeG1StandEvaluator();
    for (let t = 0; t <= 6.0001; t += 0.02) late.observe(withEngagement(Number(t.toFixed(4))));
    const lateSnapshot = late.snapshot();
    assert.equal(lateSnapshot.measured.controllerEngagedAtSeconds, engaged);
    assert.equal(lateSnapshot.measured.evaluationOpensAtSeconds, engaged + UNITREE_G1_STAND_GATE.evaluationStartSeconds);
    assert.equal(lateSnapshot.checks.evaluationWindowReached, false, 'six evaluated seconds are not yet available');
    assert.equal(lateSnapshot.standing, false);
    const full = new UnitreeG1StandEvaluator();
    for (let t = 0; t <= 9.6001; t += 0.02) full.observe(withEngagement(Number(t.toFixed(4))));
    assert.equal(full.snapshot().standing, true, 'the same run does pass once the full evaluated interval elapses');
    // Re-engaging a controller restarts the window rather than carrying the old one over.
    const restarted = new UnitreeG1StandEvaluator();
    for (let t = 0; t <= 9.6001; t += 0.02) restarted.observe(withEngagement(Number(t.toFixed(4))));
    const switched = sample(9.62);
    switched.controller = { id: G1_CONTROLLERS.SOURCE_FIXSTAND, engagedAtSeconds: 9.62 };
    restarted.observe(switched);
    assert.equal(restarted.snapshot().standing, false, 'switching controllers must restart the evaluated interval');
  }
}

// --- 6. capability labels ------------------------------------------------------------------------
assert.ok(assertCapabilityAudit() >= 9);
assert.equal(unitreeG1CapabilityById('walking').capability, 'unsupported');
assert.equal(unitreeG1CapabilityById('perturbation_recovery').capability, 'unsupported');
assert.equal(unitreeG1CapabilityById('dexterous_hands').capability, 'unsupported');
assert.equal(unitreeG1CapabilityById('hardware_calibration').capability, 'unsupported');
assert.equal(unitreeG1CapabilityById('standing').capability, 'physical / verified');
assert.equal(unitreeG1CapabilityById('kinematic_pose_inspection').capability, 'kinematic / verified');
assert.equal(unitreeG1CapabilityById('kinematic_pose_inspection').backend, 'legacy');
for (const item of UNITREE_G1_CAPABILITY_AUDIT) assert.ok(item.limitations.length > 0, `${item.id} must state what it is not`);
assert.equal(physicsCapabilityFor('unitree').backend, 'browser-mujoco');
assert.equal(physicsCapabilityFor('unitree', { physical: false }).backend, 'legacy');
assert.notEqual(physicsCapabilityFor('unitree').capability, physicsCapabilityFor('unitree', { physical: false }).capability,
  'the pose workspace must never inherit the physical workspace badge');

// --- 7. task catalog: both workspaces survive, physical is distinct -------------------------------
{
  const tasks = tasksForProfile('unitree');
  assert.equal(tasks.length, 2, 'the profile keeps the kinematic pose workspace beside the physical one');
  const physical = tasks.find((task) => task.simulationMode === 'physical_mujoco');
  const kinematic = tasks.find((task) => task.simulationMode === 'kinematic_pose');
  assert.ok(physical && kinematic, 'both workspaces must remain selectable');
  assert.equal(physical.id, 'unitree-g1-physical-dynamics');
  assert.equal(kinematic.id, 'unitree-g1-kinematic-pose-inspection');
  assert.notEqual(physical.robotId, kinematic.robotId, 'the physical workspace has its own robot identity');
  assert.equal(physical.physicalSceneId, UNITREE_G1_SCENES.freebase.id);
}

// --- 8. bounded WebMCP surface --------------------------------------------------------------------
{
  const schema = createUnitreeG1ControlSchema();
  const commands = schema.oneOf.map((branch) => branch.properties.command.const);
  assert.deepEqual([...commands].sort(), [...UNITREE_G1_CONTROL_LIMITS.exposedCommands].sort());
  const text = JSON.stringify(schema);
  for (const forbidden of UNITREE_G1_CONTROL_LIMITS.neverExposed) {
    assert.ok(!text.includes(forbidden), `the WebMCP schema must never expose ${forbidden}`);
  }
  const targets = schema.oneOf.find((branch) => branch.properties.command.const === 'set_joint_targets').properties.targets_rad;
  assert.equal(Object.keys(targets.properties).length, 29);
  assert.equal(targets.additionalProperties, false);
  for (const [index, jointId] of G1_JOINT_ORDER.entries()) {
    assert.equal(targets.properties[jointId].minimum, G1_JOINT_RANGE_RAD[index][0]);
    assert.equal(targets.properties[jointId].maximum, G1_JOINT_RANGE_RAD[index][1]);
  }
  assert.equal(UNITREE_G1_CONTROL_LIMITS.agentStandController, G1_CONTROLLERS.STAND);
  assert.ok(!UNITREE_G1_CONTROL_LIMITS.exposedCommands.includes('walk'));
  const control = read('src/webmcp/unitree-g1-physical-control.js');
  assert.ok(!control.includes(`'${G1_CONTROLLERS.SOURCE_FIXSTAND}'`) || control.includes('deliberately not agent-reachable'),
    'the measured-to-fail source controller must not be silently agent-reachable');

  // The retained kinematic pose workspace keeps its own pose-write tool. The two tools must never
  // share a name, or an agent holding the pose tool's name would silently reach a physical plant
  // (or, worse, reach a pose write while a physical badge is displayed).
  const physicalToolName = /name: '([^']+)'/.exec(control.slice(control.indexOf('getUnitreeG1PhysicalControlDefinition')))[1];
  const legacy = read('src/webmcp/robot-controls.js');
  const poseToolName = /unitree: Object\.freeze\(\{\s*\n\s*name: '([^']+)'/.exec(legacy)[1];
  assert.equal(physicalToolName, 'control_unitree_g1_physical_simulation');
  assert.equal(poseToolName, 'control_unitree_g1_simulation');
  assert.notEqual(physicalToolName, poseToolName, 'the physical and pose tools must not share a name');
  assert.match(legacy, /profileId === 'unitree' && context\.simulationMode === 'physical_mujoco'\) return null/,
    'the kinematic pose tool must be withdrawn while the physical workspace is displayed');
}

// --- 8b. the physical simulator withdraws its own presentation claims on dispose ------------------
{
  const simulator = read('src/physics/unitree-g1-physical-simulator.js');
  const declared = [...simulator.matchAll(/this\.canvas\.dataset\.(\w+)\s*=/g)].map((match) => match[1]);
  const cleared = /static DATASET_KEYS = Object\.freeze\(\[([\s\S]*?)\]\)/.exec(simulator)[1].match(/'(\w+)'/g).map((token) => token.slice(1, -1));
  // Anything the simulator claims while it owns the canvas must be withdrawn when it stops owning
  // it, or the pose workspace could inherit a stale "free-base", "standing", "physics-session" or
  // physical model-package attribute. Only keys the next backend itself re-stamps are exempt, and
  // the exemption is checked against that backend rather than asserted.
  const source = read('src/source-simulator.js');
  const hostOwned = new Set(['simulatorBackend', 'presentationGroundColor', 'simulationClockS']);
  for (const key of hostOwned) {
    assert.ok(source.includes(`this.canvas.dataset.${key} =`), `${key} is exempt only because the source backend re-stamps it`);
  }
  const leaked = declared.filter((key) => !cleared.includes(key) && !hostOwned.has(key));
  assert.deepEqual(leaked, [], `these dataset keys are set but never withdrawn: ${leaked.join(', ')}`);
}

// --- 9. the live Python surface exposes no walk and no state assignment ---------------------------
{
  const worker = read('src/runtime/live-python-worker.js');
  assert.ok(worker.includes('async def set_joint_targets'), 'live Python exposes bounded joint targets');
  assert.ok(worker.includes('async def wait_sim'), 'live Python advances simulated time');
  assert.ok(worker.includes('async def get_state'), 'live Python reads ground truth');
  assert.ok(worker.includes('async def stand'), 'live Python can engage the verified standing controller');
  assert.ok(!/async def walk/.test(worker), 'live Python must expose no walk()');
  assert.ok(!/set_root|force_upright|teleport/.test(worker), 'live Python must expose no root or upright write');
  const bridge = read('src/runtime/live-python-bridge.js');
  assert.ok(!/set_root_pose|set_root_velocity|force_upright/.test(bridge));
}

// --- 10. timing is recorded separately --------------------------------------------------------------
assert.equal(G1_PHYSICS_TIMESTEP_SECONDS, 0.002);
assert.equal(G1_LOWLEVEL_CONTROL_INTERVAL_SECONDS, 0.002);
assert.equal(G1_LOWLEVEL_CONTROL_INTERVAL_SECONDS % G1_PHYSICS_TIMESTEP_SECONDS, 0, 'the control interval must be a whole number of physics steps');
assert.equal(UNITREE_G1_BLOCKED_JOINT, 'left_hip_roll_joint');

console.log(`Unitree G1 Phase 5D core contracts: ${UNITREE_G1_MODEL_PACKAGES.length} model packages, ${UNITREE_G1_CAPABILITY_AUDIT.length} capability rows, ${UNITREE_G1_CONTROL_LIMITS.exposedCommands.length} bounded WebMCP commands: OK`);
