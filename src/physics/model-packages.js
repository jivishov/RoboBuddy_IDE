import { PARAMETER_EVIDENCE, registerModelPackage } from './model-registry.js';

export const PHASE1_MODEL_PACKAGE = registerModelPackage({
  id: 'phase1-vertical-slice',
  robotId: 'phase1_articulated_joint',
  modelId: 'phase1-vertical-slice-v1',
  source: {
    url: 'https://github.com/jivishov/RoboBuddy_IDE_v020/blob/0fce3ee17fa605e4de63d53be255656984abccac/models/vertical-slice/model.xml',
    revision: '0fce3ee17fa605e4de63d53be255656984abccac',
    variant: 'Phase 1 conformance fixture',
  },
  license: 'MIT (repository-authored fixture)',
  asset: 'models/vertical-slice/model.xml',
  sha256: 'c8bfd81bb212afb88fd3ca173fc1235f1c1b45a79c3db0563ac22330ae7dab26',
  physics: { timestepSeconds: 0.002, integrator: 'RK4' },
  controllers: ['hinge_position'],
  joints: [{ id: 'hinge', rangeRad: [-1.57, 1.57], axis: [0, 1, 0], evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED }],
  actuators: [{ id: 'hinge_position', jointId: 'hinge', controllerId: 'hinge_position', command: 'position-rad', controlRangeRad: [-1.2, 1.2], evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED }],
  bodies: [{ id: 'free_box' }],
  sceneConstraints: { fixtures: ['floor', 'table'], objects: ['free_box'] },
  evidence: {
    geometry: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    inertia: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    actuators: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  },
  limitations: ['Minimal conformance fixture; not a production robot or hardware validation.'],
});

const SO101_JOINTS = [
  ['shoulder_pan', [-1.91986, 1.91986]],
  ['shoulder_lift', [-1.7453293, 1.7453293]],
  ['elbow_flex', [-1.69, 1.69]],
  ['wrist_flex', [-1.658063, 1.658063]],
  ['wrist_roll', [-2.7438473, 2.7438473]],
  ['gripper', [-0.174533, 1.7453292]],
];

const SO101_CONTROL_RANGES = Object.freeze({
  shoulder_pan: [-1.91986, 1.91986],
  shoulder_lift: [-1.74533, 1.74533],
  elbow_flex: [-1.69, 1.69],
  wrist_flex: [-1.65806, 1.65806],
  wrist_roll: [-2.74385, 2.84121],
  gripper: [-0.17453, 1.74533],
});

const SO101_SOURCE = Object.freeze({
  url: 'https://github.com/google-deepmind/mujoco_menagerie/blob/8161bba264d7fa7c99ca301e91e7fb44737676ad/robotstudio_so101/so101.xml',
  revision: '8161bba264d7fa7c99ca301e91e7fb44737676ad',
  upstreamRevision: 'aec17bbc256d1a7342d53aaa4950595d4c30b40d',
});

const so101Joints = () => SO101_JOINTS.map(([id, rangeRad]) => ({ id, rangeRad, axis: [0, 0, 1], evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED }));
const so101Actuators = () => SO101_JOINTS.map(([id]) => ({ id, jointId: id, controllerId: 'so101_position', command: 'position-rad', controlRangeRad: SO101_CONTROL_RANGES[id], evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED }));

export const SO101_PHASE2A_MODEL_PACKAGE = registerModelPackage({
  id: 'so101-phase2a-menagerie-8161bba',
  robotId: 'so101_follower',
  modelId: 'robobuddy-so101-phase2a-v1',
  source: {
    ...SO101_SOURCE,
    variant: 'Pinned MuJoCo Menagerie SO-101 simulation adaptation of The Robot Studio follower arm; Phase 2A self-contained adaptation omits the source camera-mount child',
  },
  license: 'Apache-2.0',
  asset: 'models/so101/model.xml',
  sha256: '8573559b58eb522ca80c8cd4c88d30e0b2ab20fb222f67c57e3808a4783af085',
  physics: { timestepSeconds: 0.005, integrator: 'implicitfast', iterations: 10, lsIterations: 20 },
  controllers: ['so101_position'],
  joints: so101Joints(),
  actuators: so101Actuators(),
  bodies: [{ id: 'base' }, { id: 'gripper' }, { id: 'moving_jaw_so101_v1' }],
  evidence: {
    kinematics: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    actuatedLinkInertias: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    retainedCollisionPrimitives: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    servoControllerParameters: PARAMETER_EVIDENCE.ESTIMATED,
    cameraMountDynamics: PARAMETER_EVIDENCE.CALIBRATION_REQUIRED,
    hardwareAlignment: PARAMETER_EVIDENCE.CALIBRATION_REQUIRED,
  },
  limitations: [
    'Phase 2A articulated-plant validation package only; normal IDE capability is not promoted yet.',
    'Actuator names, joint mappings and control ranges are source-derived from the pinned Menagerie MJCF; servo gains, effort behavior and hardware alignment remain estimated or calibration-required.',
    'The pinned Menagerie wrist_roll upper joint limit is 2.7438473 rad, while The Robot Studio source revision recorded by Menagerie uses about 2.8412063 rad; RoboBuddy preserves the pinned Menagerie simulation constraint and does not present it as a measured hardware limit.',
    'Upstream visual meshes, mesh gripper collisions, the camera-mount child and some nonessential source collision geometry are omitted from this self-contained browser validation MJCF.',
    'The omitted upstream camera-mount child carries a source mesh mass of 0.012 kg, so this adapted plant must not be described as dynamically identical to the full pinned Menagerie model.',
    'The Phase 2A-only base_proxy is visual-only and non-colliding so it cannot introduce source-unsupported contact.',
    'Servo gains/force settings are upstream simulation estimates, not hardware calibration.',
    'No grasp/block-transfer task or hardware-fidelity claim in Phase 2A.',
  ],
});

export const SO101_MANIPULATION_MODEL_PACKAGE = registerModelPackage({
  id: 'so101-manipulation-menagerie-8161bba-v1',
  robotId: 'so101_follower',
  modelId: 'robobuddy-so101-manipulation-v1',
  source: {
    ...SO101_SOURCE,
    variant: 'Manipulation-ready self-contained adaptation of the pinned Menagerie SO-101 with source camera collision boxes and source 0.012 kg camera-mount mass restored through an explicit box-inertia surrogate',
  },
  license: 'Apache-2.0 for SO-101-derived model; repository-authored synthetic benchmark workcell is MIT',
  asset: 'models/so101/manipulation.xml',
  sha256: 'cf0064059a56e2c6748f72733c606b833ebf61131802b27b3645c1f25089aecc',
  physics: { timestepSeconds: 0.005, integrator: 'implicitfast', iterations: 10, lsIterations: 20 },
  controllers: ['so101_position'],
  joints: so101Joints(),
  actuators: so101Actuators(),
  bodies: [{ id: 'base' }, { id: 'gripper' }, { id: 'camera_mount' }, { id: 'moving_jaw_so101_v1' }, { id: 'benchmark_block' }],
  initialJointPositionsRad: {
    shoulder_pan: 0,
    shoulder_lift: -0.18,
    elbow_flex: 0,
    wrist_flex: 0,
    wrist_roll: Math.PI / 2,
    gripper: 0.60,
  },
  sceneConstraints: { fixtures: ['benchmark_source_support', 'benchmark_target_support', 'benchmark_target_region'], objects: ['benchmark_block'] },
  benchmark: {
    object: { id: 'benchmark_block', dimensionsM: [0.016, 0.012, 0.014], massKg: 0.020, inertiaKgM2: [5.6666667e-7, 7.5333333e-7, 6.6666667e-7], friction: [0.8, 0.005, 0.0001] },
    workSurface: { topZM: 0.227, friction: [0.8, 0.005, 0.0001] },
    supportPads: {
      source: { centerXYM: [0.39416, -0.00169], halfExtentsXYM: [0.025, 0.025] },
      target: { centerXYM: [0.358, -0.156], halfExtentsXYM: [0.025, 0.045] },
    },
    target: { centerXYM: [0.358, -0.156], halfExtentsXYM: [0.020, 0.030] },
    controllerVersion: 'so101-benchmark-transfer-v1',
  },
  evidence: {
    kinematics: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    actuatedLinkInertias: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    gripperCollisionPrimitives: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    cameraMountMassAndCollisionBoxes: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    cameraMountInertiaDistribution: PARAMETER_EVIDENCE.ESTIMATED,
    servoControllerParameters: PARAMETER_EVIDENCE.ESTIMATED,
    benchmarkObjectAndSurface: PARAMETER_EVIDENCE.ESTIMATED,
    hardwareAlignment: PARAMETER_EVIDENCE.CALIBRATION_REQUIRED,
  },
  limitations: [
    'Controlled synthetic benchmark workcell for physical-manipulation verification; object dimensions, mass, support-pad dimensions and surface friction are declared benchmark parameters, not measured laboratory hardware.',
    'The pickup and placement supports are deliberately split so the synthetic workcell does not occupy the swept volume of the retained source-derived camera collision geometry; this is a workcell geometry correction, not collision disabling.',
    'The source camera-mount mesh mass is restored as 0.012 kg using the pinned source camera collision boxes; the resulting box-derived inertia is an explicit approximation, not the source mesh inertia.',
    'Source gripper collision primitives are retained, while upstream collision meshes/visual meshes remain omitted for browser economy; grasp conclusions apply only to this declared benchmark geometry.',
    'Servo gains and 2.94 N m force range are upstream simulation estimates and are not calibrated measurements of the installed SO-101 servos.',
    'The pinned Menagerie wrist_roll joint limit remains authoritative for this simulation package; the known wider Robot Studio source value is not substituted to make the task easier.',
    'No hardware trajectory-error, backlash, compliance, fingertip-friction, work-surface-friction, servo-latency, or installed-tool calibration has been performed.',
  ],
});

import './asimov-model-package.js';
