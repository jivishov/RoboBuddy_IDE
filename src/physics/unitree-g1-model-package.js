import { PARAMETER_EVIDENCE, registerModelPackage } from './model-registry.js';
import {
  CANONICAL_VISUAL_SOURCE, G1_BODY_ORDER, G1_EFFORT_LIMIT_NM, G1_FOOT_CONTACT_GEOMS, G1_JOINT_AXIS, G1_JOINT_ORDER,
  G1_JOINT_RANGE_RAD, G1_SELF_CONTACT_EXCLUSIONS, G1_VELOCITY_LIMIT_RAD_S, MENAGERIE_G1_REFERENCE,
  UNITREE_MUJOCO_SOURCE, UNITREE_RL_MJLAB_SOURCE, UNITREE_ROS_SOURCE,
} from './unitree-g1-source-audit.js';
import { G1_CONTROLLERS, G1_PHYSICS_TIMESTEP_SECONDS, G1_STAND_POSE_RAD } from './unitree-g1-controller.js';

export const UNITREE_G1_ROBOT_ID = 'unitree_g1_29dof_physical';
export const UNITREE_G1_MOUNT_PELVIS_Z_M = 1.2;
export const UNITREE_G1_STAND_PELVIS_Z_M = 0.7842;
export const UNITREE_G1_DROP_PELVIS_Z_M = 0.95;
export const UNITREE_G1_EXTERNAL_OBJECT = 'contact_probe_block';
export const UNITREE_G1_BLOCK_FIXTURE_GEOM = 'hip_abduction_stop_wall';
export const UNITREE_G1_BLOCKED_JOINT = 'left_hip_roll_joint';

const SOURCE_URL = `https://github.com/${UNITREE_ROS_SOURCE.repository}/blob/${UNITREE_ROS_SOURCE.revision}/${UNITREE_ROS_SOURCE.mjcfPath}`;

const joints = () => G1_JOINT_ORDER.map((id, index) => ({
  id,
  rangeRad: [...G1_JOINT_RANGE_RAD[index]],
  axis: [...G1_JOINT_AXIS[index]],
  velocityLimitRadS: G1_VELOCITY_LIMIT_RAD_S[index],
  effortLimitNm: G1_EFFORT_LIMIT_NM[index],
  evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED,
}));

// One direct-torque motor per joint, exactly the source transmission. The command is a torque in
// newton metres, because the Unitree low-level interface commands torque; a position servo would
// be a different actuator with different dynamics and is deliberately not used.
const actuators = () => G1_JOINT_ORDER.map((id, index) => ({
  id,
  jointId: id,
  controllerId: G1_CONTROLLERS.LOWLEVEL,
  command: 'torque-nm',
  forceRangeNm: [-G1_EFFORT_LIMIT_NM[index], G1_EFFORT_LIMIT_NM[index]],
  controlRangeNm: [-G1_EFFORT_LIMIT_NM[index], G1_EFFORT_LIMIT_NM[index]],
  evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED,
}));

const BASE_EVIDENCE = Object.freeze({
  robotVariantIdentity: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  jointNamesOrderAxesRanges: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  linkMassesAndInertias: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  actuatorEffortLimits: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  jointVelocityLimits: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  collisionGeometry: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  footContactPrimitives: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  imuFrame: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  jointArmatureDampingFrictionLoss: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  lowLevelCommandLaw: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  standingPosture: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  standingControllerStructure: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  standingControllerAnkleGains: PARAMETER_EVIDENCE.ESTIMATED,
  selfContactExclusions: PARAMETER_EVIDENCE.ESTIMATED,
  fixtureGeometry: PARAMETER_EVIDENCE.ESTIMATED,
  externalObjectMassAndFriction: PARAMETER_EVIDENCE.ESTIMATED,
  surfaceFriction: PARAMETER_EVIDENCE.ESTIMATED,
  hardwareAlignment: PARAMETER_EVIDENCE.CALIBRATION_REQUIRED,
});

const BASE_LIMITATIONS = Object.freeze([
  `Robot identity: Unitree G1 29-DoF with fixed rubber hands, from ${UNITREE_ROS_SOURCE.repository}@${UNITREE_ROS_SOURCE.revision.slice(0, 7)} ${UNITREE_ROS_SOURCE.mjcfPath}. It is not the 23-DoF variant, not the dexterous-hand variant, not the dual-arm variant and not the rev_1_0 revision that MuJoCo Menagerie derives from.`,
  `Body tree, inertials, joint names, order, axes, ranges and actuator force limits are the pinned source values. The joint armature, damping and friction loss, and the motor control range, come from Unitree's own low-level MuJoCo simulator ${UNITREE_MUJOCO_SOURCE.repository}@${UNITREE_MUJOCO_SOURCE.revision.slice(0, 7)}, which is byte-equal to the pinned model on every kinematic and inertial quantity. They are simulator parameters Unitree chose, not measured G1 gearbox behaviour.`,
  'Collision geometry is the convex hull of each source collision mesh. MuJoCo collides mesh geoms through their convex hull, so this is collision-identical to the source model rather than an approximation, and the source foot contact spheres and shoulder cylinders are carried through unchanged.',
  'The fixed rubber hands are visual-only in the pinned source and stay visual-only here: they carry no collision geometry, generate no contact, and have no finger joint, finger actuator or grasp capability. Dexterous hand control is unsupported.',
  `Self-collision is enabled across the plant. ${G1_SELF_CONTACT_EXCLUSIONS.length} body pairs are excluded, each individually justified in the source audit; every other non-adjacent pair collides, including knee-to-knee, thigh-to-torso, elbow-to-torso and forearm-to-thigh.`,
  'The physics timestep, integrator and solver settings are MuJoCo defaults, which both pinned Unitree models leave unchanged, so they are what Unitree\'s own low-level simulator runs.',
  'No measurement of an assembled Unitree G1 was used. Motor bandwidth, gearbox friction, joint compliance, backlash, contact material, sensor latency, control-network timing and stability margin are all unmodelled and uncalibrated.',
  'Nothing in this package writes root position, root orientation, root velocity, joint position or joint velocity during ordinary execution. Every motion comes from a bounded actuator torque acting through MuJoCo dynamics.',
]);

function g1Package({ id, modelId, asset, sha256, variant, rootMode, controllers, initialCommand, extraFixtures = [], objects = [], initialJointPositionsRad = null, limitations = [] }) {
  const freeRoot = rootMode === 'free-base';
  return {
    id, modelId, asset, sha256,
    meshManifest: { asset: 'models/unitree_g1/source/MESH_MANIFEST.json', sha256: 'a7a70ed60b67b018f85df632080f7320c72ad8974badedc722f3b76eb2a60e91' },
    robotId: UNITREE_G1_ROBOT_ID,
    source: {
      url: SOURCE_URL,
      revision: UNITREE_ROS_SOURCE.revision,
      variant,
      urdf: `https://github.com/${UNITREE_ROS_SOURCE.repository}/blob/${UNITREE_ROS_SOURCE.revision}/${UNITREE_ROS_SOURCE.urdfPath}`,
      urdfSha256: UNITREE_ROS_SOURCE.urdfSha256,
      mjcfSha256: UNITREE_ROS_SOURCE.mjcfSha256,
      dynamicAugmentation: `${UNITREE_MUJOCO_SOURCE.repository}@${UNITREE_MUJOCO_SOURCE.revision}/${UNITREE_MUJOCO_SOURCE.mjcfPath}`,
      controllerSource: `${UNITREE_RL_MJLAB_SOURCE.repository}@${UNITREE_RL_MJLAB_SOURCE.revision}/${UNITREE_RL_MJLAB_SOURCE.configPath}`,
      canonicalVisual: `${CANONICAL_VISUAL_SOURCE.repository}@${CANONICAL_VISUAL_SOURCE.revision}/${CANONICAL_VISUAL_SOURCE.module}`,
      notAdopted: MENAGERIE_G1_REFERENCE,
    },
    license: 'BSD-3-Clause for Unitree-derived model geometry and inertials; Apache-2.0 for the Unitree controller configuration the standing posture and gains are read from; repository-authored fixtures and scene objects are MIT',
    physics: { timestepSeconds: G1_PHYSICS_TIMESTEP_SECONDS, integrator: 'Euler', iterations: 100, lsIterations: 50 },
    controllers,
    joints: joints(),
    actuators: actuators(),
    // Every source body is observed, so the canonical visual rig can be checked frame by frame
    // against the physical model rather than only at the pelvis.
    bodies: [
      ...G1_BODY_ORDER.map((id) => (id === 'pelvis' && freeRoot ? { id, freeJointId: 'floating_base_joint' } : { id })),
      ...objects.map((objectId) => ({ id: objectId, freeJointId: `${objectId}_free` })),
    ],
    rootMode,
    // What the actuators are commanded to do at load and after reset, before any caller says
    // anything. 'joint-hold' issues a bounded hold at the model's own declared initial joint
    // positions through the Unitree source hold gains; 'passive' issues zero torque, which is
    // Unitree's own Passive-state behaviour and is what the gravity-release fixture needs. Both
    // are commands: neither writes joint or root state.
    initialCommand,
    footContactGeoms: { left: [...G1_FOOT_CONTACT_GEOMS.left], right: [...G1_FOOT_CONTACT_GEOMS.right] },
    selfContactExclusions: G1_SELF_CONTACT_EXCLUSIONS.map((item) => [...item.bodies]),
    sceneConstraints: { fixtures: ['floor', ...extraFixtures], objects: [...objects] },
    ...(initialJointPositionsRad ? { initialJointPositionsRad } : {}),
    evidence: BASE_EVIDENCE,
    limitations: [...BASE_LIMITATIONS, ...limitations],
  };
}

const STAND_POSE_MAP = Object.freeze(Object.fromEntries(G1_JOINT_ORDER.map((id, index) => [id, G1_STAND_POSE_RAD[index]])));

export const UNITREE_G1_MOUNTED_PACKAGE = registerModelPackage(g1Package({
  id: 'unitree-g1-29dof-mounted-dd4fa68-v1',
  modelId: 'robobuddy-unitree-g1-mounted-v1',
  asset: 'models/unitree_g1/mounted.xml',
  sha256: '78b99a41889c15f677b723349f4f6f478725bc39a4efa5d7dcf37730d50d0a01',
  variant: 'Unitree G1 29-DoF on a declared root-fixed mount, legs clear of the floor',
  rootMode: 'fixed-mounted',
  initialCommand: 'joint-hold',
  controllers: [G1_CONTROLLERS.LOWLEVEL, G1_CONTROLLERS.JOINT_HOLD],
  extraFixtures: ['robobuddy_g1_mount'],
  limitations: [
    `Declared root-fixed dynamic test fixture. The pelvis is welded to the world by the model itself (it carries no root joint) and is held at ${UNITREE_G1_MOUNT_PELVIS_Z_M} m by a visible, declared mount. Joint dynamics, actuation response, contact identity and known-pose comparison are all valid here; balance, standing and locomotion are not, and no result from this fixture may be presented as standing evidence.`,
    'Three additional contact exclusions apply only in the mounted variants: pelvis to left_hip_pitch_link, pelvis to right_hip_pitch_link and pelvis to waist_yaw_link. MuJoCo skips its parent-child contact filter when the parent body is welded to the world, so without these the mounted fixture would carry hull grazes the free-base plant does not have and would not be testing the same plant.',
  ],
}));

export const UNITREE_G1_MOUNTED_BLOCKED_PACKAGE = registerModelPackage(g1Package({
  id: 'unitree-g1-29dof-mounted-blocked-dd4fa68-v1',
  modelId: 'robobuddy-unitree-g1-mounted-blocked-v1',
  asset: 'models/unitree_g1/mounted_blocked.xml',
  sha256: '5f0af3108fc496fae98a4d1eb2c59d752423668363939a0eb020519d9cc31065',
  variant: 'Unitree G1 29-DoF on the declared root-fixed mount, with a declared rigid wall obstructing left hip abduction',
  rootMode: 'fixed-mounted',
  initialCommand: 'joint-hold',
  controllers: [G1_CONTROLLERS.LOWLEVEL, G1_CONTROLLERS.JOINT_HOLD],
  extraFixtures: ['robobuddy_g1_mount', 'robobuddy_g1_joint_stop'],
  limitations: [
    `Declared blocked-joint fixture. A visible rigid wall physically obstructs ${UNITREE_G1_BLOCKED_JOINT}; the obstruction is a real contact between the named ${UNITREE_G1_BLOCK_FIXTURE_GEOM} geom and the robot, not a narrowed joint range or a masked command.`,
    'The wall carries declared stiff contact parameters so an 88 N m hip motor cannot squeeze through it. No robot geom contact parameter is changed.',
    'Same mounted-fixture limitations as the unobstructed mount: no balance, standing or locomotion conclusion may be drawn here.',
  ],
}));

export const UNITREE_G1_FREEBASE_PACKAGE = registerModelPackage(g1Package({
  id: 'unitree-g1-29dof-freebase-dd4fa68-v1',
  modelId: 'robobuddy-unitree-g1-freebase-v1',
  asset: 'models/unitree_g1/freebase.xml',
  sha256: '3aad0869bf2a83d1f9acd84f5c627745a9a56dc1ab426be8a1f82463a6a76bf3',
  variant: 'Unitree G1 29-DoF free-base scene: free pelvis, gravity, floor, source foot contacts, self-contact, and one declared free external object',
  rootMode: 'free-base',
  initialCommand: 'joint-hold',
  controllers: [G1_CONTROLLERS.LOWLEVEL, G1_CONTROLLERS.JOINT_HOLD, G1_CONTROLLERS.STAND, G1_CONTROLLERS.SOURCE_FIXSTAND],
  objects: [UNITREE_G1_EXTERNAL_OBJECT],
  initialJointPositionsRad: STAND_POSE_MAP,
  limitations: [
    `The pelvis is a free MuJoCo body and the model starts in the source standing posture at ${UNITREE_G1_STAND_PELVIS_Z_M} m, the height at which the source foot contact spheres just reach the floor. There is no support fixture, elastic band, weld, equality constraint or external force in this scene at any time.`,
    'The declared free contact-probe block is a 100 mm, 0.4 kg cube resting on the floor. Its mass and friction are repository estimates; it exists to prove the plant handles robot-to-environment contact beyond the floor, and it is not a manipulation task.',
    'Standing is a posture hold produced by a bounded joint-space controller. It is not dynamic balance, not perturbation recovery and not locomotion-ready. Walking is unsupported in this workspace and no walk operation exists.',
    'The declared initial command is a bounded joint hold at the source standing posture using Unitree\'s own source hold gains. It is a command, not a state write, and it is deliberately not a standing controller: left holding, the source gains lose the posture within about half a second, which is the same 80 against 231 N m/rad ankle-stiffness shortfall that makes source FixStand fail the free-base gate.',
  ],
}));

export const UNITREE_G1_FREEBASE_DROP_PACKAGE = registerModelPackage(g1Package({
  id: 'unitree-g1-29dof-freebase-drop-dd4fa68-v1',
  modelId: 'robobuddy-unitree-g1-freebase-drop-v1',
  asset: 'models/unitree_g1/freebase_drop.xml',
  sha256: 'e222c0b8bf12f51352373f31965752893f5cbb987dc28a172300fcaa25165ab5',
  variant: 'Unitree G1 29-DoF free-base gravity release: neutral pose released above the floor',
  rootMode: 'free-base',
  initialCommand: 'passive',
  controllers: [G1_CONTROLLERS.LOWLEVEL, G1_CONTROLLERS.JOINT_HOLD],
  limitations: [
    `Deterministic gravity and fall fixture. The neutral pose is released at ${UNITREE_G1_DROP_PELVIS_Z_M} m with no external object, so root height, root orientation and non-foot ground contact can be read without any other influence.`,
    'The declared initial command is passive: every actuator is commanded to zero torque, exactly Unitree\'s own Passive state. Nothing holds the robot up, so the fall is gravity acting on the plant.',
    'The fallen state persists: nothing in this package resets the robot upright, and reset is a new initial condition rather than a recovery.',
  ],
}));

export const UNITREE_G1_MODEL_PACKAGES = Object.freeze([
  UNITREE_G1_MOUNTED_PACKAGE,
  UNITREE_G1_MOUNTED_BLOCKED_PACKAGE,
  UNITREE_G1_FREEBASE_PACKAGE,
  UNITREE_G1_FREEBASE_DROP_PACKAGE,
]);
