import { PARAMETER_EVIDENCE, registerModelPackage } from './model-registry.js';
import { LEKIWI_SOURCE, LEROBOT_SOURCE, SO_ARM101_SOURCE, LEGACY_TASK_SOURCE } from './lekiwi-source-audit.js';
import { LEKIWI_WHEEL_GEOMETRY, MAX_WHEEL_RAD_S } from './lekiwi-kinematics.js';

export const LEKIWI_ROBOT_ID = 'lekiwi_v1_mobile_manipulator';
export const LEKIWI_WHEEL_CONTROLLER = 'lekiwi_wheel_velocity';
export const LEKIWI_ARM_CONTROLLER = 'lekiwi_arm_position';
export const LEKIWI_TIMESTEP_SECONDS = 0.002;
export const LEKIWI_WHEEL_FORCE_NM = 2.94;
export const LEKIWI_WHEEL_RADIUS_M = 0.0508;
export const LEKIWI_FLOOR_Z_M = 0;

const SOURCE_URL = `https://github.com/${LEKIWI_SOURCE.repository}/blob/${LEKIWI_SOURCE.revision}/URDF/LeKiwi.urdf`;

// Wheel drive joints. The pinned URDF declares them as continuous joints, so no jnt_range is
// asserted; the axis is asserted against the compiled model.
// The compiled MuJoCo hinge axis is expressed in the wheel body frame, whose local z is the
// spin axis (the body carries xyaxes = world-up, drive-direction). The audited model-frame and
// pinned-source axes are recorded alongside it so the worker can assert the compiled value while
// the audit keeps the full frame relationship visible.
const wheelJoint = (item) => ({
  id: item.id,
  axis: [0, 0, 1],
  evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  modelFrameAxis: [...item.modelAxis],
  sourceUrdfAxis: [...item.urdfAxis],
  note: 'axis is the compiled wheel-body-frame hinge axis; modelFrameAxis is the LeRobot-positive spin axis in the LeKiwi base frame, and sourceUrdfAxis is the pinned URDF axis, which is anti-parallel to it.',
});

// A bounded simulated velocity servo, matching the pinned LeRobot driver, which puts the three
// base ST3215 servos into VELOCITY operating mode. Control range is the LeRobot software limit;
// the torque limit is an explicit estimate.
const wheelActuator = (item) => ({
  id: item.id,
  jointId: item.id,
  controllerId: LEKIWI_WHEEL_CONTROLLER,
  command: 'velocity-rad-s',
  controlRangeRadS: [-MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S],
  forceRangeNm: [-LEKIWI_WHEEL_FORCE_NM, LEKIWI_WHEEL_FORCE_NM],
  evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED,
});

const rollerJointIds = (prefix) => {
  const ids = [];
  for (const row of ['a', 'b']) for (let index = 0; index < 6; index += 1) ids.push(`${prefix}_roller_${row}${index}`);
  return ids;
};
const passiveRollerJoint = (id) => ({ id, evidence: PARAMETER_EVIDENCE.ESTIMATED, passive: true });

const WHEEL_PREFIX = Object.freeze({
  base_left_wheel: 'lekiwi_left',
  base_back_wheel: 'lekiwi_back',
  base_right_wheel: 'lekiwi_right',
});

const ARM_JOINTS = Object.freeze([
  ['arm_shoulder_pan', [-1.91986, 1.91986], [-1.91986, 1.91986]],
  ['arm_shoulder_lift', [-1.7453293, 1.7453293], [-1.74533, 1.74533]],
  ['arm_elbow_flex', [-1.69, 1.69], [-1.69, 1.69]],
  ['arm_wrist_flex', [-1.658063, 1.658063], [-1.65806, 1.65806]],
  ['arm_wrist_roll', [-2.7438473, 2.7438473], [-2.74385, 2.74385]],
  ['arm_gripper', [-0.174533, 1.7453292], [-0.17453, 1.74533]],
]);

const armJoints = () => ARM_JOINTS.map(([id, rangeRad]) => ({ id, rangeRad, axis: [0, 0, 1], evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED }));
const armActuators = () => ARM_JOINTS.map(([id, , controlRangeRad]) => ({
  id, jointId: id, controllerId: LEKIWI_ARM_CONTROLLER, command: 'position-rad', controlRangeRad, evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED,
}));

export const LEKIWI_STOW_POSE_RAD = Object.freeze({
  arm_shoulder_pan: 0,
  arm_shoulder_lift: -1.53,
  arm_elbow_flex: 0.19,
  arm_wrist_flex: 1.04,
  arm_wrist_roll: 0,
  arm_gripper: 1.2,
});

const BASE_EVIDENCE = Object.freeze({
  wheelPlacementAndAxes: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  wheelRadiusAndWidth: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  rollerCountAndStagger: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  rollerBarrelProfile: PARAMETER_EVIDENCE.ESTIMATED,
  chassisMassAndInertia: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  wheelActuatorSemantics: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  wheelTorqueLimitAndGain: PARAMETER_EVIDENCE.ESTIMATED,
  jointDampingAndFrictionLoss: PARAMETER_EVIDENCE.ESTIMATED,
  surfaceFriction: PARAMETER_EVIDENCE.ESTIMATED,
  hardwareAlignment: PARAMETER_EVIDENCE.CALIBRATION_REQUIRED,
});

const BASE_LIMITATIONS = Object.freeze([
  'Wheel placement, wheel joint axes, wheel radius/width, roller count and stagger, chassis mass and chassis inertia are read from the pinned SIGRobotics-UIUC/LeKiwi URDF and wheel mesh. The URDF assigns CAD default densities, so the resulting 10.7 kg base is a source-derived CAD figure, not a weighed assembled LeKiwi.',
  'Every omni wheel carries its twelve source-count rollers as individually hinged passive bodies. Each roller barrel is an ellipsoid fitted to the source wheel envelope; it is an explicit approximation of the fused single-body source mesh, which does not itself articulate its rollers.',
  'The bounded wheel velocity servo matches the pinned LeRobot VELOCITY operating mode. Its 4.60061 rad/s control range is the LeRobot software limit, and its 2.94 N m torque limit, servo gain, damping and friction loss are estimates rather than measured ST3215 behaviour.',
  'Surface friction, roller bearing losses and contact stiffness are declared simulator parameters for a flat indoor floor. Carpet, curbs, thresholds, rough terrain, suspension, wheel wear, battery droop and motor thermal behaviour are outside the modelled scope.',
  'The base is a free MuJoCo body. Nothing in this package writes base pose or base velocity outside explicit, logged setup and reset.',
]);

export const LEKIWI_WHEEL_REFERENCE_PACKAGE = registerModelPackage({
  id: 'lekiwi-wheel-reference-efa608d-v1',
  robotId: 'lekiwi_v1_wheel_reference',
  modelId: 'robobuddy-lekiwi-wheel-reference-v1',
  source: { url: SOURCE_URL, revision: LEKIWI_SOURCE.revision, variant: 'single LeKiwi omni wheel with explicit passive rollers on a declared frictionless-bearing characterization carriage' },
  license: 'Apache-2.0 for LeKiwi-derived geometry; the repository-authored reference carriage is MIT',
  asset: 'models/lekiwi/wheel_reference.xml',
  sha256: '4ee345ae84cd0acd1626529f260bf58e18bd580a8f1f0df82fa8837ecffff778',
  physics: { timestepSeconds: LEKIWI_TIMESTEP_SECONDS, integrator: 'implicitfast', iterations: 20, lsIterations: 20 },
  controllers: [LEKIWI_WHEEL_CONTROLLER],
  joints: [
    { id: 'carriage_x', evidence: PARAMETER_EVIDENCE.ESTIMATED, passive: true },
    { id: 'carriage_y', evidence: PARAMETER_EVIDENCE.ESTIMATED, passive: true },
    { id: 'carriage_z', evidence: PARAMETER_EVIDENCE.ESTIMATED, passive: true },
    { id: 'carriage_yaw', evidence: PARAMETER_EVIDENCE.ESTIMATED, passive: true },
    wheelJoint(LEKIWI_WHEEL_GEOMETRY[1]),
    ...rollerJointIds('lekiwi_back').map(passiveRollerJoint),
  ],
  actuators: [wheelActuator(LEKIWI_WHEEL_GEOMETRY[1])],
  bodies: [{ id: 'wheel_carriage' }, { id: 'lekiwi_back_wheel' }],
  sceneConstraints: { fixtures: ['reference_floor'], objects: [] },
  evidence: BASE_EVIDENCE,
  limitations: [
    'Single-wheel contact characterization rig, not a LeKiwi workspace. The carriage runs on declared frictionless planar and vertical bearings so the rig cannot tip; every propulsive force still has to come from roller/ground contact.',
    ...BASE_LIMITATIONS,
  ],
});

function basePackage({ id, modelId, asset, sha256, variant, extraFixtures = [], limitations = [] }) {
  const joints = [];
  const actuators = [];
  for (const item of LEKIWI_WHEEL_GEOMETRY) {
    joints.push(wheelJoint(item));
    for (const rollerId of rollerJointIds(WHEEL_PREFIX[item.id])) joints.push(passiveRollerJoint(rollerId));
    actuators.push(wheelActuator(item));
  }
  return {
    id, modelId, asset, sha256,
    robotId: LEKIWI_ROBOT_ID,
    source: { url: SOURCE_URL, revision: LEKIWI_SOURCE.revision, variant },
    license: 'Apache-2.0 for LeKiwi-derived geometry; repository-authored fixtures are MIT',
    physics: { timestepSeconds: LEKIWI_TIMESTEP_SECONDS, integrator: 'implicitfast', iterations: 20, lsIterations: 20 },
    controllers: [LEKIWI_WHEEL_CONTROLLER],
    joints,
    actuators,
    bodies: [
      { id: 'lekiwi_base', freeJointId: 'lekiwi_base_free' },
      { id: 'lekiwi_left_wheel' }, { id: 'lekiwi_back_wheel' }, { id: 'lekiwi_right_wheel' },
    ],
    sceneConstraints: { fixtures: ['lekiwi_floor', ...extraFixtures], objects: [] },
    evidence: BASE_EVIDENCE,
    limitations: [...BASE_LIMITATIONS, ...limitations],
  };
}

export const LEKIWI_BASE_PACKAGE = registerModelPackage(basePackage({
  id: 'lekiwi-base-efa608d-v1',
  modelId: 'robobuddy-lekiwi-base-v1',
  asset: 'models/lekiwi/base.xml',
  sha256: '144c3e96012ef63eb94df32c16a3cdc4fbbb7ca888d5f6bdc62be337088285ea',
  variant: 'LeKiwi V1 three-wheel Kiwi base with explicit passive omni-wheel rollers, free chassis body, flat indoor floor',
}));

export const LEKIWI_BASE_STAND_PACKAGE = registerModelPackage(basePackage({
  id: 'lekiwi-base-stand-efa608d-v1',
  modelId: 'robobuddy-lekiwi-base-stand-v1',
  asset: 'models/lekiwi/base_stand.xml',
  sha256: 'b0ecf8e603260090a09764dffea6d8db0fe569fc97070bb1f087b03b6e36bc08',
  variant: 'LeKiwi V1 base resting on a declared visible rigid test stand that holds every wheel 20 mm clear of the floor',
  extraFixtures: ['lekiwi_test_stand'],
  limitations: ['Adverse-condition fixture: the base is supported by a visible declared test stand, so wheel actuation has no ground to react against. It is an explicit physical fixture, not a hidden constraint or a disabled contact.'],
}));

export const LEKIWI_BASE_LOWTRACTION_PACKAGE = registerModelPackage(basePackage({
  id: 'lekiwi-base-lowtraction-efa608d-v1',
  modelId: 'robobuddy-lekiwi-base-lowtraction-v1',
  asset: 'models/lekiwi/base_lowtraction.xml',
  sha256: '15ccba4c20eb7dd274de59faca136cde2b4a0a3b8af832f4cac09776330dcd01',
  variant: 'LeKiwi V1 base on a deliberately reduced-traction surface (sliding friction 0.02)',
  limitations: ['Adverse-condition fixture: roller and floor sliding friction are reduced to 0.02. This is a declared degraded surface, not a modelled real material.'],
}));

const COURIER_ARM_BODIES = [
  'arm_base', 'arm_shoulder', 'arm_upper', 'arm_lower', 'arm_wrist', 'arm_gripper_body', 'arm_moving_jaw', 'arm_wrist_camera',
].map((id) => ({ id }));

export const LEKIWI_COURIER_PACKAGE = registerModelPackage((() => {
  const base = basePackage({
    id: 'lekiwi-courier-efa608d-v1',
    modelId: 'robobuddy-lekiwi-courier-v1',
    asset: 'models/lekiwi/courier.xml',
    sha256: 'e947b6b007461083b65a12504857f23ae5e4f33481922491e2ba429e7c4fee41',
    variant: 'LeKiwi V1 base plus mounted SO-ARM101 arm, configured transfer bench and a free hollow beaker in one MuJoCo plant',
  });
  return {
    ...base,
    license: 'Apache-2.0 for LeKiwi-derived and SO-ARM101-derived geometry; the repository-authored workcell is MIT',
    source: { ...base.source, armSource: `https://github.com/${SO_ARM101_SOURCE.repository}/blob/${SO_ARM101_SOURCE.revision}/${SO_ARM101_SOURCE.path}` },
    controllers: [LEKIWI_WHEEL_CONTROLLER, LEKIWI_ARM_CONTROLLER],
    joints: [...base.joints, ...armJoints()],
    actuators: [...base.actuators, ...armActuators()],
    bodies: [...base.bodies, ...COURIER_ARM_BODIES, { id: 'empty_beaker', freeJointId: 'empty_beaker_free' }],
    initialJointPositionsRad: { ...LEKIWI_STOW_POSE_RAD },
    sceneConstraints: {
      fixtures: ['lekiwi_floor', 'lekiwi_transfer_worktop', 'lekiwi_transfer_back_panel', 'lekiwi_delivery_marker'],
      objects: ['empty_beaker'],
    },
    benchmark: {
      taskGeometrySource: { ...LEGACY_TASK_SOURCE, scenarioId: 'lekiwi-01-beaker-courier' },
      apiCompatibilitySource: LEROBOT_SOURCE,
      armSource: SO_ARM101_SOURCE,
      worktopTopZM: 0.211,
      pickupXYM: [0.305, -0.292],
      deliveryXYM: [0.242928, -0.370144],
      homeXYM: [0, 0],
      serviceStopXYM: [0.274, 0.030],
      serviceStopYawRad: -Math.PI / 2,
      restrictedStopXYM: [0.1, -0.65],
      restrictedStopRadiusM: 0.15,
      beakerMassKg: 0.06,
      controllerVersion: 'lekiwi-beaker-courier-v1',
    },
    evidence: {
      ...BASE_EVIDENCE,
      armKinematicsAndInertials: PARAMETER_EVIDENCE.SOURCE_DERIVED,
      armMountTranslation: PARAMETER_EVIDENCE.SOURCE_DERIVED,
      armMountYaw: PARAMETER_EVIDENCE.ESTIMATED,
      armServoParameters: PARAMETER_EVIDENCE.ESTIMATED,
      taskWorkcellGeometry: PARAMETER_EVIDENCE.ESTIMATED,
      beakerMassAndFriction: PARAMETER_EVIDENCE.ESTIMATED,
    },
    limitations: [
      ...base.limitations,
      'The SO-ARM101 arm links, joint ranges and inertials come from the pinned MuJoCo Menagerie robotstudio_so101 derivation already used by this repository, mounted at the arm axis position read from the LeKiwi URDF. The mount yaw is an explicit repository convention because the two sources do not share an arm-base origin convention.',
      'The transfer bench, receiving-zone marker and beaker envelope are configured educational geometry from the pinned legacy courier scenario, not measured laboratory hardware. The beaker is repaired into a hollow wall/rim vessel so a rim pinch is real contact instead of a convex-block grasp.',
      'A top-down rim grasp is not kinematically reachable for this arm at this bench height; the physical grasp uses a 50 degree tilted rim-wall pinch. This is a documented repair of the legacy grasp pose, not a change to the robot physics.',
      'Beaker mass, wall friction and contact stiffness are estimates. No gripping-force, glass-compliance, liquid, thermal or hardware-safety behaviour is modelled or claimed.',
      'There is no weld, parent attachment, attachment flag, snap, teleport, upright payload lock, or task-success state overwrite anywhere in this package.',
    ],
  };
})());

export const LEKIWI_MODEL_PACKAGES = Object.freeze([
  LEKIWI_WHEEL_REFERENCE_PACKAGE,
  LEKIWI_BASE_PACKAGE,
  LEKIWI_BASE_STAND_PACKAGE,
  LEKIWI_BASE_LOWTRACTION_PACKAGE,
  LEKIWI_COURIER_PACKAGE,
]);
