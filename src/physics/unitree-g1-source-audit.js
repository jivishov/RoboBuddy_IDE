import { PARAMETER_EVIDENCE } from './model-registry.js';

// Phase 5D source pins.
//
// Tier 1 is the model authority: it is the exact revision the existing RoboBuddy Unitree G1
// visual rig was derived from, so the physical and kinematic workspaces describe the same robot.
// Tier 2 is Unitree's own low-level MuJoCo simulator at the same model family; it supplies the
// dynamic augmentation the Tier 1 description omits and the low-level command semantics.
// Tier 3 is Unitree's current MuJoCo-native RL/controller repository; it supplies the standing
// posture and the FixStand controller structure and gains.
export const UNITREE_ROS_SOURCE = Object.freeze({
  repository: 'unitreerobotics/unitree_ros',
  revision: 'dd4fa6866e523ad61324f658d63736e4eda3a6e4',
  variant: 'Unitree G1 29-DoF, fixed rubber hands (left_rubber_hand / right_rubber_hand)',
  urdfPath: 'robots/g1_description/g1_29dof.urdf',
  urdfSha256: 'ed3b86a04d190b2206c10b44dce685832bc20f72e176d8735aa2d18ffeae1cf0',
  mjcfPath: 'robots/g1_description/g1_29dof.xml',
  mjcfSha256: '599a3f3e55f92543a404944cd0501f8fbcba57a57210d1a1e42ef45839768c84',
  license: 'BSD-3-Clause',
  role: 'body tree, inertials, joint names/axes/ranges/effort limits, foot contact primitives, imu site, 29 motor transmissions',
});

export const UNITREE_MUJOCO_SOURCE = Object.freeze({
  repository: 'unitreerobotics/unitree_mujoco',
  revision: '1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d',
  mjcfPath: 'unitree_robots/g1/g1_29dof.xml',
  mjcfSha256: '423e28bd718b19f7a65cda539b6f794ddbb268b4b9bdbd85f4bd982b30729617',
  bridgePath: 'simulate/src/unitree_sdk2_bridge.h',
  license: 'BSD-3-Clause',
  role: 'same-model dynamic augmentation (joint armature/damping/frictionloss, motor ctrlrange) and the low-level motor command law',
  compatibility: 'Verified byte-equal to the Tier 1 model on body_pos, body_quat, body_mass, body_inertia, body_ipos, body_iquat, jnt_range, jnt_axis, jnt_pos, jnt_type and jnt_actfrcrange (all maxdiff 0.0), and its link meshes are byte-identical by SHA-256. This is therefore the same model, not a different revision.',
});

export const UNITREE_RL_MJLAB_SOURCE = Object.freeze({
  repository: 'unitreerobotics/unitree_rl_mjlab',
  revision: '1425b15f73bd4095f0df53709d7c389c3eb9e790',
  configPath: 'deploy/robots/g1/config/config.yaml',
  fixStandPath: 'deploy/include/FSM/State_FixStand.h',
  fsmPath: 'deploy/include/FSM/CtrlFSM.h',
  velocityPolicyPath: 'deploy/robots/g1/config/policy/velocity/v0/params/deploy.yaml',
  license: 'Apache-2.0',
  role: 'G1 standing posture, FixStand controller structure and gains, 1 kHz FSM cadence, and the audited G1 velocity locomotion policy contract',
});

export const MENAGERIE_G1_REFERENCE = Object.freeze({
  repository: 'google-deepmind/mujoco_menagerie',
  revision: '8161bba264d7fa7c99ca301e91e7fb44737676ad',
  path: 'unitree_g1/g1.xml',
  status: 'not-adopted',
  reason:
    'Menagerie derives its G1 from g1_29dof_rev_1_0.xml, a different Unitree revision: it carries rev_1_0 waist/torso meshes, no waist_support_link, hip_roll actuatorfrcrange of +/-139 N m against this model\'s +/-88, frictionloss 0.3 with no joint damping, and tuned position actuators. Its MJX variant\'s hand-designed capsule colliders were inspected as evidence that primitive colliders are a standard approach, but no Menagerie geometry, gain, inertial, actuator or contact parameter is used anywhere in this package.',
});

export const CANONICAL_VISUAL_SOURCE = Object.freeze({
  repository: 'jivishov/RoboBuddy_AI',
  revision: '66d18a029a0caeb6a6075e681dbd9ecd6b22affa',
  module: 'simulator/js/robot-mesh-data-unitree-g1.js',
  sha256: '2d0623dc17ac1026678232d10378cadd310cc0a736b77e91cf00da9ecbad8dcb',
  role: 'presentation-only canonical Three.js rig, shared by the kinematic pose workspace and the physical workspace',
  authority: 'presentation-only; the MuJoCo PhysicsSession is the sole physical authority',
});

// The 29 actuated joints in the exact order of the source MJCF, which is also the exact order of
// the Unitree 29-DoF DDS motor index table (unitree_mujoco/unitree_robots/g1/g1_joint_index_dds.md)
// and of every gain/target vector in the Tier 3 controller configuration.
export const G1_JOINT_ORDER = Object.freeze([
  'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
  'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
  'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
  'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
  'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
  'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
  'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
  'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
  'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint',
]);

// The compiled joint range: the source MJCF <joint range>. It equals the source URDF
// <limit lower upper> on 23 of the 29 joints. On the six wrist joints the MJCF rounds the URDF
// value to five decimals - at most 2.055e-6 rad, or 1.2e-4 degrees. The MJCF value is the one the
// physical model actually compiles, so it is the authority here, and the URDF values are kept
// beside it in G1_URDF_JOINT_RANGE_RAD rather than silently dropped.
export const G1_JOINT_RANGE_RAD = Object.freeze([
  [-2.5307, 2.8798], [-0.5236, 2.9671], [-2.7576, 2.7576], [-0.087267, 2.8798], [-0.87267, 0.5236], [-0.2618, 0.2618],
  [-2.5307, 2.8798], [-2.9671, 0.5236], [-2.7576, 2.7576], [-0.087267, 2.8798], [-0.87267, 0.5236], [-0.2618, 0.2618],
  [-2.618, 2.618], [-0.52, 0.52], [-0.52, 0.52],
  [-3.0892, 2.6704], [-1.5882, 2.2515], [-2.618, 2.618], [-1.0472, 2.0944], [-1.97222, 1.97222], [-1.61443, 1.61443], [-1.61443, 1.61443],
  [-3.0892, 2.6704], [-2.2515, 1.5882], [-2.618, 2.618], [-1.0472, 2.0944], [-1.97222, 1.97222], [-1.61443, 1.61443], [-1.61443, 1.61443],
].map((pair) => Object.freeze(pair)));

// The source URDF <limit lower upper>, kept so the audited difference stays visible.
export const G1_URDF_JOINT_RANGE_RAD = Object.freeze([
  [-2.5307, 2.8798], [-0.5236, 2.9671], [-2.7576, 2.7576], [-0.087267, 2.8798], [-0.87267, 0.5236], [-0.2618, 0.2618],
  [-2.5307, 2.8798], [-2.9671, 0.5236], [-2.7576, 2.7576], [-0.087267, 2.8798], [-0.87267, 0.5236], [-0.2618, 0.2618],
  [-2.618, 2.618], [-0.52, 0.52], [-0.52, 0.52],
  [-3.0892, 2.6704], [-1.5882, 2.2515], [-2.618, 2.618], [-1.0472, 2.0944], [-1.972222054, 1.972222054], [-1.614429558, 1.614429558], [-1.614429558, 1.614429558],
  [-3.0892, 2.6704], [-2.2515, 1.5882], [-2.618, 2.618], [-1.0472, 2.0944], [-1.972222054, 1.972222054], [-1.614429558, 1.614429558], [-1.614429558, 1.614429558],
].map((pair) => Object.freeze(pair)));

// The exact set of joints where the two source files differ, and by how much.
export const G1_URDF_MJCF_RANGE_DISCREPANCIES = Object.freeze([
  'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
  'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint',
]);
export const G1_URDF_MJCF_MAX_RANGE_DELTA_RAD = 2.055e-6;

// Source URDF <limit effort>, identical to the source MJCF actuatorfrcrange magnitude and to the
// unitree_mujoco motor ctrlrange magnitude.
export const G1_EFFORT_LIMIT_NM = Object.freeze([
  88, 88, 88, 139, 50, 50,
  88, 88, 88, 139, 50, 50,
  88, 50, 50,
  25, 25, 25, 25, 25, 5, 5,
  25, 25, 25, 25, 25, 5, 5,
]);

// Source URDF <limit velocity>. The source MJCF motor declaration carries no velocity limiter, so
// this bound would silently disappear if the MJCF were used on its own. The bounded command layer
// re-imposes it at command level; it is a source limit, not a repository invention.
export const G1_VELOCITY_LIMIT_RAD_S = Object.freeze([
  32, 32, 32, 20, 37, 37,
  32, 32, 32, 20, 37, 37,
  32, 37, 37,
  37, 37, 37, 37, 37, 22, 22,
  37, 37, 37, 37, 37, 22, 22,
]);

// Source MJCF joint axes, in G1_JOINT_ORDER. The worker asserts every one of these against the
// compiled model, so a joint-order or axis mistake cannot pass silently.
export const G1_JOINT_AXIS = Object.freeze([
  [0, 1, 0], [1, 0, 0], [0, 0, 1], [0, 1, 0], [0, 1, 0], [1, 0, 0],
  [0, 1, 0], [1, 0, 0], [0, 0, 1], [0, 1, 0], [0, 1, 0], [1, 0, 0],
  [0, 0, 1], [1, 0, 0], [0, 1, 0],
  [0, 1, 0], [1, 0, 0], [0, 0, 1], [0, 1, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1],
  [0, 1, 0], [1, 0, 0], [0, 0, 1], [0, 1, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1],
].map((axis) => Object.freeze(axis)));

export const G1_BODY_ORDER = Object.freeze([
  'pelvis',
  'left_hip_pitch_link', 'left_hip_roll_link', 'left_hip_yaw_link', 'left_knee_link', 'left_ankle_pitch_link', 'left_ankle_roll_link',
  'right_hip_pitch_link', 'right_hip_roll_link', 'right_hip_yaw_link', 'right_knee_link', 'right_ankle_pitch_link', 'right_ankle_roll_link',
  'waist_yaw_link', 'waist_roll_link', 'torso_link',
  'left_shoulder_pitch_link', 'left_shoulder_roll_link', 'left_shoulder_yaw_link', 'left_elbow_link', 'left_wrist_roll_link', 'left_wrist_pitch_link', 'left_wrist_yaw_link',
  'right_shoulder_pitch_link', 'right_shoulder_roll_link', 'right_shoulder_yaw_link', 'right_elbow_link', 'right_wrist_roll_link', 'right_wrist_pitch_link', 'right_wrist_yaw_link',
]);

export const G1_FOOT_CONTACT_GEOMS = Object.freeze({
  left: Object.freeze(['left_foot_heel_lateral', 'left_foot_heel_medial', 'left_foot_toe_lateral', 'left_foot_toe_medial']),
  right: Object.freeze(['right_foot_heel_lateral', 'right_foot_heel_medial', 'right_foot_toe_lateral', 'right_foot_toe_medial']),
});

export const G1_TOTAL_MASS_KG = 35.112142;
export const G1_SOURCE_PELVIS_HEIGHT_M = 0.793;
export const G1_STAND_PELVIS_HEIGHT_M = 0.7842;
// Centre of mass above the ankle roll joint axis in the source standing posture, measured by
// forward kinematics on the compiled source model. It is what sets the free-base ankle stiffness
// the standing controller has to exceed.
export const G1_STAND_COM_HEIGHT_ABOVE_ANKLE_M = 0.6715;

export const G1_SELF_CONTACT_EXCLUSIONS = Object.freeze([
  Object.freeze({
    bodies: Object.freeze(['torso_link', 'left_shoulder_roll_link']),
    justification: 'The shoulder roll housing is mechanically seated inside the torso shell. The source torso shell is concave at the shoulder, and MuJoCo collides mesh geoms through their convex hull, so the hull fills the armpit and reports the housing as penetrating the torso from a source-legal 0.5 rad of adduction onward. Excluding this one pair keeps torso vs shoulder_yaw, elbow and every distal arm body enabled, so arm-to-torso self-contact remains part of the plant.',
  }),
  Object.freeze({ bodies: Object.freeze(['torso_link', 'right_shoulder_roll_link']), justification: 'Mirror of the left exclusion, same justification.' }),
  Object.freeze({
    bodies: Object.freeze(['left_wrist_roll_link', 'left_wrist_yaw_link']),
    justification: 'The three wrist bodies form one nested mechanical assembly; their hull envelopes overlap by at most 1.6 mm at the source joint limits. The intervening wrist_pitch_link is already parent-child filtered against both.',
  }),
  Object.freeze({ bodies: Object.freeze(['right_wrist_roll_link', 'right_wrist_yaw_link']), justification: 'Mirror of the left exclusion, same justification.' }),
]);

const row = (parameter, value, evidence, source, note) => Object.freeze({ parameter, value, evidence, source, note });

export const UNITREE_G1_RECONCILIATION = Object.freeze([
  row('robot variant', 'G1 29-DoF, fixed rubber hands', PARAMETER_EVIDENCE.SOURCE_DERIVED, `${UNITREE_ROS_SOURCE.repository}@${UNITREE_ROS_SOURCE.revision} ${UNITREE_ROS_SOURCE.mjcfPath}`,
    'The pinned model declares left_rubber_hand and right_rubber_hand and 29 actuated joints. It is not the 23-DoF variant, not the dexterous-hand variant, not the dual-arm variant and not the rev_1_0 revision.'),
  row('joint count / order', 29, PARAMETER_EVIDENCE.SOURCE_DERIVED, `${UNITREE_ROS_SOURCE.mjcfPath} and unitree_mujoco g1_joint_index_dds.md`,
    'The MJCF joint order is exactly the published Unitree 29-DoF DDS motor index order, and exactly the order of every gain and target vector in the Tier 3 controller configuration.'),
  row('joint axes / positions / ranges', 'source', PARAMETER_EVIDENCE.SOURCE_DERIVED, `${UNITREE_ROS_SOURCE.urdfPath} and ${UNITREE_ROS_SOURCE.mjcfPath}`,
    'URDF and MJCF agree exactly on all 29 joint axes and on 23 of the 29 joint ranges. Audited discrepancy: on the six wrist roll/pitch/yaw joints the MJCF rounds the URDF limit to five decimals, at most 2.055e-6 rad (1.2e-4 degrees). The MJCF value is what the physical model compiles and is therefore the authority; the URDF values are kept in G1_URDF_JOINT_RANGE_RAD. The generated model asserts the compiled values against this table to 1e-9.'),
  row('effort limits', 'source, 88/139/50/25/5 N m', PARAMETER_EVIDENCE.SOURCE_DERIVED, 'URDF <limit effort> and MJCF actuatorfrcrange',
    'The URDF effort limit and the MJCF actuatorfrcrange are the same numbers on every joint. MuJoCo enforces them as a per-joint actuator force clamp.'),
  row('velocity limits', 'source, 32/20/37/22 rad/s', PARAMETER_EVIDENCE.SOURCE_DERIVED, 'URDF <limit velocity>',
    'Audited discrepancy: the source MJCF motor declaration has no velocity limiter, so this URDF bound is not enforced by the model. The bounded command layer re-imposes it on the commanded velocity target.'),
  row('masses / inertias / total mass', G1_TOTAL_MASS_KG, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'MJCF <inertial> of all 30 bodies',
    'Every body carries an explicit source <inertial>, so no geom density affects the plant. 35.112142 kg is the source figure, not a weighed robot.'),
  row('joint armature / damping / friction loss', [0.01, 0.05, 0.2], PARAMETER_EVIDENCE.SOURCE_DERIVED, `${UNITREE_MUJOCO_SOURCE.repository}@${UNITREE_MUJOCO_SOURCE.revision} ${UNITREE_MUJOCO_SOURCE.mjcfPath}`,
    'Taken from Unitree\'s own low-level MuJoCo simulator at the same model. Friction loss is 0.1 on the four wrist pitch/yaw joints. These are simulator parameters chosen by Unitree, not measured G1 gearbox measurements.'),
  row('motor control range', 'ctrlrange = actuatorfrcrange', PARAMETER_EVIDENCE.SOURCE_DERIVED, UNITREE_MUJOCO_SOURCE.mjcfPath,
    'The Tier 1 MJCF leaves motor ctrl unlimited. Tier 2 bounds each motor command to the joint\'s own effort limit; this package adopts that so no actuator is ever effectively unlimited.'),
  row('low-level command law', 'tau = tau_ff + kp (q* - q) + kd (dq* - dq)', PARAMETER_EVIDENCE.SOURCE_DERIVED, UNITREE_MUJOCO_SOURCE.bridgePath,
    'Exactly the expression the official unitree_mujoco SDK bridge writes into d->ctrl, applied to the same direct-torque motors.'),
  row('collision geometry', 'convex hulls of the 25 source collision meshes, plus the source foot spheres and shoulder cylinders', PARAMETER_EVIDENCE.SOURCE_DERIVED, `${UNITREE_ROS_SOURCE.mjcfPath} meshes`,
    'MuJoCo collides mesh geoms through their convex hull, so hull assets are collision-identical to the source meshes. Verified: a 6 s contact-rich free-base standing trial gives root and joint maxdiff 0.0 against the source meshes.'),
  row('fixed rubber hands', 'visual-only in the source; no collision geometry', PARAMETER_EVIDENCE.SOURCE_DERIVED, UNITREE_ROS_SOURCE.mjcfPath,
    'The source gives left_rubber_hand and right_rubber_hand contype="0" conaffinity="0" geoms only. This package keeps that exactly: the hands are passive geometry that generates no contact, and the wrist_yaw_link hull is the distal contact body. No finger joint, finger actuator or grasp capability exists.'),
  row('foot contact primitives', '4 spheres of radius 5 mm per ankle_roll_link', PARAMETER_EVIDENCE.SOURCE_DERIVED, UNITREE_ROS_SOURCE.mjcfPath,
    'Source positions (-0.05, +/-0.025, -0.03) and (0.12, +/-0.03, -0.03) in the foot frame. This package names them so contact evidence can identify a foot.'),
  row('self-contact exclusions', G1_SELF_CONTACT_EXCLUSIONS.length, PARAMETER_EVIDENCE.ESTIMATED, 'repository-authored',
    'Four excluded pairs, each individually justified. All other non-adjacent pairs collide, including knee-to-knee, thigh-to-torso, elbow-to-torso and forearm-to-thigh.'),
  row('physics timestep / integrator / solver', [0.002, 'Euler', 100, 50], PARAMETER_EVIDENCE.SOURCE_DERIVED, 'MuJoCo defaults, which both pinned Unitree models leave unchanged',
    'Neither Unitree model declares an <option>, so this is exactly what Unitree\'s own low-level simulator runs. The generated model states them explicitly so the compiled values can be asserted.'),
  row('low-level controller interval', 0.002, PARAMETER_EVIDENCE.ESTIMATED, `${UNITREE_RL_MJLAB_SOURCE.repository} ${UNITREE_RL_MJLAB_SOURCE.fsmPath}`,
    'Unitree\'s FSM thread runs at dt = 0.001 s. Lockstep simulation-time scheduling requires the control interval to be a whole number of physics steps, so the nominal configuration decimates the 1 kHz cadence to one update per 2 ms physics step. The tighter numerical configuration runs 1 ms physics with the exact 1 kHz cadence, and the sensitivity study compares the two.'),
  row('standing posture', 'source FixStand qs', PARAMETER_EVIDENCE.SOURCE_DERIVED, `${UNITREE_RL_MJLAB_SOURCE.configPath} FSM.FixStand.qs[1]`,
    'The same 29 values are also the default_joint_pos of that repository\'s G1 velocity locomotion policy, so the posture is corroborated by two independent Unitree artefacts.'),
  row('standing controller structure', 'interpolate measured q to the standing posture, per-joint PD, dq* = 0, tau_ff = 0', PARAMETER_EVIDENCE.SOURCE_DERIVED, UNITREE_RL_MJLAB_SOURCE.fixStandPath,
    'Exactly Unitree\'s State_FixStand: it latches the measured joint vector on entry, linearly interpolates to the standing posture over 2 s, and holds it with per-joint kp/kd.'),
  row('standing controller waist and arm gains', 'source FixStand kp/kd', PARAMETER_EVIDENCE.SOURCE_DERIVED, `${UNITREE_RL_MJLAB_SOURCE.configPath} FSM.FixStand.kp/kd`, 'Used unchanged.'),
  // These must stay equal to the shipped G1_ROBOBUDDY_ANKLE_KP/KD. The controller module imports
  // this one, so the values cannot be imported back without a cycle; the core contract test
  // asserts the two agree instead.
  row('standing controller ankle gains', [250, 10], PARAMETER_EVIDENCE.ESTIMATED, 'repository-authored',
    `Unitree's FixStand ankle gains of kp 40 / kd 2 give a total ankle-pitch stiffness of 80 N m/rad, against the m g h of ${(G1_TOTAL_MASS_KG * 9.81 * G1_STAND_COM_HEIGHT_ABOVE_ANKLE_M).toFixed(1)} N m/rad estimated by a rigid inverted-pendulum approximation. This heuristic is not a stability proof for the full articulated plant. Measured: source FixStand topples in about 1.5 s at only 12 N m of peak torque. kp 250 is chosen from that criterion with a margin. kd is bounded from above by explicit-integration stability on the unloaded foot, kd * dt < 2 I: measured, kd 30 limit-cycles a free foot at 4 rad/s while kd 10 tracks it to 0.9 mrad and holds the stand identically. Both are repository-authored simulator gains, never Unitree hardware settings.`),
  row('hardware alignment', 'none', PARAMETER_EVIDENCE.CALIBRATION_REQUIRED, '-',
    'No measurement of an assembled Unitree G1 was used or is claimed. Motor bandwidth, gearbox friction, joint compliance, backlash, contact material, sensor latency, control-network timing and stability margin are all unmeasured.'),
]);

export function reconciliationByEvidence(evidence) {
  return UNITREE_G1_RECONCILIATION.filter((item) => item.evidence === evidence);
}

export function assertReconciliationCoverage() {
  const seen = new Set(UNITREE_G1_RECONCILIATION.map((item) => item.parameter));
  const required = [
    'robot variant', 'joint count / order', 'joint axes / positions / ranges', 'effort limits', 'velocity limits',
    'masses / inertias / total mass', 'joint armature / damping / friction loss', 'motor control range',
    'low-level command law', 'collision geometry', 'fixed rubber hands', 'foot contact primitives',
    'self-contact exclusions', 'physics timestep / integrator / solver', 'low-level controller interval',
    'standing posture', 'standing controller structure', 'standing controller ankle gains', 'hardware alignment',
  ];
  const missing = required.filter((name) => !seen.has(name));
  if (missing.length) throw new Error(`Unitree G1 reconciliation table is missing: ${missing.join(', ')}`);
  for (const item of UNITREE_G1_RECONCILIATION) {
    if (!Object.values(PARAMETER_EVIDENCE).includes(item.evidence)) {
      throw new Error(`Unitree G1 reconciliation row ${item.parameter} has unknown evidence ${item.evidence}`);
    }
  }
  return UNITREE_G1_RECONCILIATION.length;
}
