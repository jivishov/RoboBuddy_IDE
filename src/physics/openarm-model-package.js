import { OPENARM_MODEL_SHA256 } from './openarm-generated.js';
import { PARAMETER_EVIDENCE, registerModelPackage } from './model-registry.js';

export const OPENARM_V2_SOURCE = Object.freeze({
  repository: 'enactic/openarm_mujoco',
  revision: 'a8c979629f2591ad035d99d338ce114969e6cddc',
  sourcePath: 'v2/openarm_bimanual.xml',
  sourceBlob: '0bd77d3bf7e0a5f3d2361fdf5e8328d00d0b2cc9',
  cellPath: 'v2/cell/cell.xml',
  variant: 'OpenArm V2 bimanual plant mounted with the V2 cell origin relationship',
});

const joint = (id, rangeRad, axis) => ({ id, rangeRad, axis, evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED });
const actuator = (id, jointId, rangeRad) => ({
  id,
  jointId,
  controllerId: 'openarm_v2_position',
  command: 'position-rad',
  controlRangeRad: rangeRad,
  evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED,
});

const joints = Object.freeze([
  joint('openarm_left_joint1', [-3.4907, 1.3963], [0, 1, 0]),
  joint('openarm_left_joint2', [-3.3161, 0.17453], [-1, 0, 0]),
  joint('openarm_left_joint3', [-1.5708, 1.5708], [0, 0, -1]),
  joint('openarm_left_joint4', [0, 2.4435], [0, -1, 0]),
  joint('openarm_left_joint5', [-1.5708, 1.5708], [0, 0, -1]),
  joint('openarm_left_joint6', [-0.7854, 0.7854], [0, -1, 0]),
  joint('openarm_left_joint7', [-1.5708, 1.5708], [1, 0, 0]),
  joint('openarm_left_finger_joint1', [0, 0.7854], [-1, 0, 0]),
  joint('openarm_right_joint1', [-1.3963, 3.4907], [0, -1, 0]),
  joint('openarm_right_joint2', [-0.17453, 3.3161], [-1, 0, 0]),
  joint('openarm_right_joint3', [-1.5708, 1.5708], [0, 0, -1]),
  joint('openarm_right_joint4', [0, 2.4435], [0, -1, 0]),
  joint('openarm_right_joint5', [-1.5708, 1.5708], [0, 0, -1]),
  joint('openarm_right_joint6', [-0.7854, 0.7854], [0, 1, 0]),
  joint('openarm_right_joint7', [-1.5708, 1.5708], [1, 0, 0]),
  joint('openarm_right_finger_joint1', [-0.7854, 0], [-1, 0, 0]),
]);

const actuators = Object.freeze([
  actuator('left_joint1_ctrl', 'openarm_left_joint1', [-3.49066, 1.39626]),
  actuator('left_joint2_ctrl', 'openarm_left_joint2', [-3.31613, 0.174533]),
  actuator('left_joint3_ctrl', 'openarm_left_joint3', [-1.5708, 1.5708]),
  actuator('left_joint4_ctrl', 'openarm_left_joint4', [0, 2.44346]),
  actuator('left_joint5_ctrl', 'openarm_left_joint5', [-1.5708, 1.5708]),
  actuator('left_joint6_ctrl', 'openarm_left_joint6', [-0.785398, 0.785398]),
  actuator('left_joint7_ctrl', 'openarm_left_joint7', [-1.5708, 1.5708]),
  actuator('left_finger1_ctrl', 'openarm_left_finger_joint1', [0, 0.7854]),
  actuator('right_joint1_ctrl', 'openarm_right_joint1', [-1.39626, 3.49066]),
  actuator('right_joint2_ctrl', 'openarm_right_joint2', [-0.174533, 3.31613]),
  actuator('right_joint3_ctrl', 'openarm_right_joint3', [-1.5708, 1.5708]),
  actuator('right_joint4_ctrl', 'openarm_right_joint4', [0, 2.44346]),
  actuator('right_joint5_ctrl', 'openarm_right_joint5', [-1.5708, 1.5708]),
  actuator('right_joint6_ctrl', 'openarm_right_joint6', [-0.785398, 0.785398]),
  actuator('right_joint7_ctrl', 'openarm_right_joint7', [-1.5708, 1.5708]),
  actuator('right_finger1_ctrl', 'openarm_right_finger_joint1', [-0.7854, 0]),
]);

const mechanicalCouplings = Object.freeze([
  Object.freeze({
    id: 'openarm_left_ee_finger_joint_mimic',
    driverJointId: 'openarm_left_finger_joint1',
    followerJointId: 'openarm_left_finger_joint2',
    multiplier: 1,
    offsetRad: 0,
    evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  }),
  Object.freeze({
    id: 'openarm_right_ee_finger_joint_mimic',
    driverJointId: 'openarm_right_finger_joint1',
    followerJointId: 'openarm_right_finger_joint2',
    multiplier: 1,
    offsetRad: 0,
    evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  }),
]);

const armBodies = [];
for (const side of ['left', 'right']) {
  armBodies.push({ id: `openarm_${side}_base_link` });
  for (let index = 1; index <= 6; index += 1) armBodies.push({ id: `openarm_${side}_link${index}` });
  armBodies.push({ id: `openarm_${side}_ee_base_link` });
  armBodies.push({ id: `openarm_${side}_ee_inner_finger` });
  armBodies.push({ id: `openarm_${side}_ee_outer_finger` });
}

export const OPENARM_V2_PHASE5A_MODEL_PACKAGE = registerModelPackage({
  id: 'openarm-v2-phase5a-a8c9796-v3',
  robotId: 'openarm_v2_bimanual',
  modelId: 'robobuddy-openarm-v2-phase5a-v3',
  source: {
    url: `https://github.com/${OPENARM_V2_SOURCE.repository}/blob/${OPENARM_V2_SOURCE.revision}/${OPENARM_V2_SOURCE.sourcePath}`,
    revision: OPENARM_V2_SOURCE.revision,
    variant: `${OPENARM_V2_SOURCE.variant}; source-matched convex-component contact and rendering geometry`,
  },
  license: 'Apache-2.0 for OpenArm-derived plant; repository-authored dry benchmark fixtures are MIT',
  asset: 'models/openarm_v2/manipulation.xml',
  sha256: OPENARM_MODEL_SHA256,
  physics: { timestepSeconds: 0.001, integrator: 'Euler' },
  controllers: ['openarm_v2_position'],
  joints,
  actuators,
  mechanicalCouplings,
  bodies: [
    ...armBodies,
    { id: 'openarm_mount' },
    { id: 'flask', freeJointId: 'flask_free' },
    { id: 'beaker', freeJointId: 'beaker_free' },
  ],
  initialJointPositionsRad: {
    openarm_left_joint1: -0.004216999605119041,
    openarm_left_joint2: 0,
    openarm_left_joint3: 0,
    openarm_left_joint4: 1.6592956429571402,
    openarm_left_joint5: 0,
    openarm_left_joint6: -0.09271631576736272,
    openarm_left_joint7: 0,
    openarm_left_finger_joint1: 0.65,
    openarm_right_joint1: 0.004216999605119041,
    openarm_right_joint2: 0,
    openarm_right_joint3: 0,
    openarm_right_joint4: 1.6592956429571402,
    openarm_right_joint5: 0,
    openarm_right_joint6: 0.09271631576736272,
    openarm_right_joint7: 0,
    openarm_right_finger_joint1: -0.65,
  },
  sceneConstraints: {
    fixtures: ['cell_table', 'left_source_support', 'left_hotplate', 'right_source_support', 'right_ring_post', 'right_ring_gauze'],
    objects: ['flask', 'beaker'],
  },
  benchmark: {
    taskGeometrySource: { repository: 'jivishov/RoboBuddy_AI', revision: '75fe2669c0ab0b029986de424c69162071174df8', scenarioId: 'openarm-04-filtration-workcell', path: 'missions/lab-assistant/v2/definitions/openarm/openarm-04-filtration-workcell.json' },
    mountOriginM: [0.185, 0, 1.34],
    sourceHomeEeM: {
      left: [0.401, 0.1535, 1.14],
      right: [0.401, -0.1535, 1.14],
    },
    sourceSupports: {
      flaskTopZM: 1.035,
      beakerTopZM: 1.075,
    },
    destinationSupports: {
      flask: { id: 'left_hotplate', centerXYM: [0.67, 0.1535], topZM: 1.035, halfExtentsXYM: [0.058, 0.054] },
      beaker: { id: 'right_ring_gauze', centerXYM: [0.67, -0.1535], topZM: 1.075, halfExtentsXYM: [0.048, 0.048] },
    },
    objects: {
      flask: { massKg: 0.060, initialPositionM: [0.55, 0.1535, 1.092], supportBottomZM: 1.035, sourceProfileM: { lowerRadius: 0.039, shoulderRadius: 0.031, neckRadius: 0.015, height: 0.114, centerOfMassFromBottom: 0.042 } },
      beaker: { massKg: 0.050, initialPositionM: [0.55, -0.1535, 1.105], supportBottomZM: 1.075, sourceProfileM: { radius: 0.025, height: 0.060, centerOfMassFromBottom: 0.028 } },
    },
    controllerVersion: 'openarm-v2-bimanual-stack-v3',
  },
  evidence: {
    kinematicsAndMirroredFrames: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    linkMassAndInertia: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    dampingArmatureFrictionLoss: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    sourceActuatorGainsAndArmEffortCaps: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    mechanicalFingerCoupling: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    convexComponentCollisionGeometry: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    operatingGripLimitAndServoProfile: PARAMETER_EVIDENCE.ESTIMATED,
    dryTaskVesselAndReceiverGeometry: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    stagingSupportGeometry: PARAMETER_EVIDENCE.ESTIMATED,
    vesselMassInertiaAndFriction: PARAMETER_EVIDENCE.ESTIMATED,
    hardwareAlignment: PARAMETER_EVIDENCE.CALIBRATION_REQUIRED,
  },
  limitations: [
    'Robot contacts and presentation share convex hulls of the pinned upstream collision components. These are collision approximations, not measured hardware surfaces or compliant contact patches.',
    'Both arms share one world. Nonadjacent same-arm collisions are enabled; MuJoCo connected-body filtering remains in effect.',
    'Robot inertials, axes, mechanical finger couplings and arm effort caps are source-derived. The quintic reference-speed bounds, bias-assisted servo and tighter 1.2 Nm gripper operating cap are simulation design choices, not a measured hardware controller.',
    'Fixtures, dry vessel masses, inertia and friction are estimated. Vessels use solid exterior envelopes; no fluid, thermal, chemical or glass-deformation process is simulated.',
    'The default reference task transfers the flask and then the beaker. Custom equipment scenes require their own outcome checks; Cartesian IK does not promise a collision-free path.',
    'No object attachment, grasp weld, teleport, hardware transport or hardware validation is provided.',
  ],
});
