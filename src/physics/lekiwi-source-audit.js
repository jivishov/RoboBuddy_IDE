import { PARAMETER_EVIDENCE } from './model-registry.js';

// Phase 5B source pins. The official LeKiwi project is the geometry authority; the pinned
// LeRobot revision is the public API/controller-mapping compatibility reference; the SO-ARM101
// arm links reuse the Menagerie derivation this repository already pins for SO-101.
export const LEKIWI_SOURCE = Object.freeze({
  repository: 'SIGRobotics-UIUC/LeKiwi',
  revision: 'efa608d7ee5a495a4803b1d28cd0c955b4f1e033',
  variant: 'LeKiwi Version 1 — three-wheel Kiwi (holonomic) omni drive with an SO-ARM101 arm',
  paths: Object.freeze(['README.md', 'URDF/LeKiwi.urdf', 'URDF/JOINT_NAMES.md', 'URDF/meshes/4-Omni-Directional-Wheel_Single_Body-v1.stl']),
  license: 'Apache-2.0',
});

export const LEROBOT_SOURCE = Object.freeze({
  repository: 'huggingface/lerobot',
  revision: '7e241bd630a3719a56157a497ce5d08f244784f1',
  path: 'src/lerobot/robots/lekiwi/lekiwi.py',
  role: 'public action/observation semantics and the body-to-wheel Kiwi mapping',
  license: 'Apache-2.0',
});

export const SO_ARM101_SOURCE = Object.freeze({
  repository: 'google-deepmind/mujoco_menagerie',
  revision: '8161bba264d7fa7c99ca301e91e7fb44737676ad',
  path: 'robotstudio_so101/so101.xml',
  role: 'SO-ARM101 arm links, inertials and STS3215 actuator parameters (already pinned by this repository as models/so101)',
  license: 'Apache-2.0',
});

export const LEGACY_TASK_SOURCE = Object.freeze({
  repository: 'jivishov/RoboBuddy_AI',
  revision: '75fe2669c0ab0b029986de424c69162071174df8',
  path: 'missions/lab-assistant/v2/definitions/lekiwi/lekiwi-01-beaker-courier.json',
  role: 'task identity, educational objective, configured workcell envelope, route intent, success-sequence intent',
});

export const LEKIWI_SIM_SECONDARY_SOURCE = Object.freeze({
  repository: 'SIGRobotics-UIUC/LeKiwi-sim',
  role: 'secondary MuJoCo reference; inspected as evidence only',
  status: 'not-adopted',
  reason: 'Its arm variant, wheel representation and actuator definitions were not adopted because the official LeKiwi URDF at the pinned revision is the geometry authority for the selected configuration, and no revision of LeKiwi-sim was pinned for this work package. No parameter in this package is derived from it.',
});

// The nine stable actuated names published by URDF/JOINT_NAMES.md at the pin.
export const LEKIWI_ACTUATED_NAMES = Object.freeze([
  'base_left_wheel', 'base_back_wheel', 'base_right_wheel',
  'arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
  'arm_wrist_flex', 'arm_wrist_roll', 'arm_gripper',
]);

export const LEKIWI_CANONICAL_PRESENTATION_MAP = Object.freeze({
  joints: Object.freeze({
    arm_shoulder_pan: Object.freeze({ sign: 1, offsetRad: 0.01654725 }),
    arm_shoulder_lift: Object.freeze({ sign: -1, offsetRad: -1.65460081 }),
    arm_elbow_flex: Object.freeze({ sign: -1, offsetRad: 1.48772696 }),
    arm_wrist_flex: Object.freeze({ sign: -1, offsetRad: 1.30528119 }),
    arm_wrist_roll: Object.freeze({ sign: -1, offsetRad: 1.62034568 }),
  }),
  gripper: Object.freeze({ physicalOpenRad: 1.20, physicalClosedRad: -0.17, canonicalOpenValue: 20, canonicalCloseValue: 85 }),
  evidence: 'source-reconciled from the pinned LeKiwi canonical URDF chain and pinned Menagerie SO-ARM101 physical chain; presentation only',
});

export const LEKIWI_WHEEL_NAMES = Object.freeze(['base_left_wheel', 'base_back_wheel', 'base_right_wheel']);
export const LEKIWI_ARM_JOINT_NAMES = Object.freeze(LEKIWI_ACTUATED_NAMES.slice(3));

// Physical model frame. The pinned URDF root link is base_plate_layer1-v5, whose +X points to
// the robot's right-hand wheel side and whose +Y points toward the arm mount. The public
// LeRobot body frame is x forward / y left / z up, so the physical package is authored in the
// LeRobot body frame and the source relationship is recorded here rather than left implicit.
export const MODEL_FRAME = Object.freeze({
  id: 'lekiwi_base',
  description: 'x forward, y left, z up; origin at the centroid of the three wheel centres, at wheel-axle height',
  sourceFrame: 'SIGRobotics-UIUC/LeKiwi URDF link base_plate_layer1-v5',
  urdfToModel: 'model_x = urdf_y, model_y = -urdf_x, model_z = urdf_z (a +90 deg rotation about Z)',
  originInUrdfM: Object.freeze([0.000231, -0.006533, 0.017860]),
  forwardDirectionInUrdf: '+Y (the wheel the source names base_back_wheel is at -Y, and the arm mounts at +Y)',
});

const row = (parameter, value, evidence, source, note) => Object.freeze({ parameter, value, evidence, source, note });

// Reconciliation table. Every physical parameter used by the Phase 5B packages appears here
// with its evidence class. "source-derived" means it is read from a pinned source; "estimated"
// means this repository chose it; "calibration-required" means only a measurement of an
// assembled LeKiwi can establish it.
export const LEKIWI_RECONCILIATION = Object.freeze([
  row('base frame / forward direction', MODEL_FRAME.forwardDirectionInUrdf, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'LeKiwi URDF joint names and arm mount',
    'Cross-checked against the pinned LeRobot Kiwi mapping: with this forward direction the three source wheel axes reproduce the LeRobot wheel angles exactly.'),
  row('wheel centre (left)', [0.059193, 0.092659, 0], PARAMETER_EVIDENCE.SOURCE_DERIVED, 'URDF forward kinematics at zero configuration', 'Origin of 4-Omni-Directional-Wheel_Single_Body-v1.'),
  row('wheel centre (back)', [-0.119842, -0.000379, 0], PARAMETER_EVIDENCE.SOURCE_DERIVED, 'URDF forward kinematics at zero configuration', 'Origin of 4-Omni-Directional-Wheel_Single_Body-v1-2.'),
  row('wheel centre (right)', [0.060649, -0.09228, 0], PARAMETER_EVIDENCE.SOURCE_DERIVED, 'URDF forward kinematics at zero configuration', 'Origin of 4-Omni-Directional-Wheel_Single_Body-v1-1.'),
  row('wheel joint axes', 'URDF axes negated', PARAMETER_EVIDENCE.SOURCE_DERIVED, 'URDF <axis> of base_*_wheel',
    'Audited discrepancy: the URDF joint axes are anti-parallel to the LeRobot motor positive-velocity convention. Negating all three reproduces the pinned LeRobot wheel angles 150/-90/30 deg and a positive rotation moment arm for every wheel. The model declares the LeRobot-positive axis and records the source axis.'),
  row('wheel radius', 0.0508, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'URDF/meshes/4-Omni-Directional-Wheel_Single_Body-v1.stl bounding geometry',
    'Exactly a 4 inch (101.6 mm) omni wheel. The LeRobot controller default of 0.05 m is an interface parameter, not this measured source radius.'),
  row('wheel axial width', 0.038828, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'wheel mesh bounding geometry', 'Two roller rows, each one roller diameter wide.'),
  row('roller layout', '2 rows x 6 rollers, staggered 30 deg', PARAMETER_EVIDENCE.SOURCE_DERIVED, 'angular clustering of the wheel mesh outer band', 'Double-row omni wheel; 12 rollers total per wheel.'),
  row('roller barrel profile', 'ellipsoid, semi-axes 0.009707 / 0.009707 / 0.0205 m', PARAMETER_EVIDENCE.ESTIMATED, 'repository-authored fit',
    'The mesh is a single fused body, so the individual roller barrel is not directly readable. The ellipsoid is fitted so the wheel envelope stays within 0.1 mm of the source 0.0508 m outer radius across the +/-15 deg arc each roller carries.'),
  row('effective chassis radius per wheel', [0.109842, 0.119842, 0.110242], PARAMETER_EVIDENCE.SOURCE_DERIVED, 'URDF wheel centres and axes',
    'Moment arm of each wheel drive direction about the wheel centroid. The LeRobot controller default base_radius of 0.125 m is an interface parameter and is 4.7 to 13.8 percent larger than the source geometry.'),
  row('chassis composite mass', 6.658975, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'URDF inertials of every non-wheel, non-arm link',
    'The pinned URDF assigns CAD default densities: the value is source-derived but is not a measured assembled-robot mass. Absolute mass, traction margin and motor loading remain calibration-required.'),
  row('chassis composite inertia', 'diag ~ (0.0230, 0.0239, 0.0384) kg m^2 about the composite CoM', PARAMETER_EVIDENCE.SOURCE_DERIVED, 'URDF inertials, parallel-axis composition', 'Same CAD-density caveat as the mass.'),
  row('wheel mass', 1.354285, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'URDF 4-Omni-Directional-Wheel_Single_Body inertial', 'Same CAD-density caveat. Split between hub and rollers to match the source spin inertia.'),
  row('wheel spin inertia', 0.00187981, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'URDF wheel inertial about the spin axis', 'The physical package reproduces it to within 0.5 percent from the modelled hub and roller bodies.'),
  row('arm mount transform', [0.034846, 0.000231, 0.03914], PARAMETER_EVIDENCE.SOURCE_DERIVED, 'URDF arm_shoulder_pan axis position, minus the Menagerie SO-101 base-to-shoulder offset',
    'The URDF arm_shoulder_pan axis sits 73.681 mm ahead of the wheel centroid on the centreline at 101.54 mm above the axle plane. The mount yaw is 0 (arm reaches forward).'),
  row('arm mount yaw', 0, PARAMETER_EVIDENCE.ESTIMATED, 'repository-authored convention',
    'The URDF arm base link and the Menagerie SO-101 base link do not share an origin convention, so the yaw is fixed by requiring the zero-pan reach to point along the robot forward direction.'),
  row('arm joint axes / limits / inertials', 'pinned Menagerie robotstudio_so101', PARAMETER_EVIDENCE.SOURCE_DERIVED, SO_ARM101_SOURCE.path,
    'The LeKiwi arm is an SO-ARM101. The URDF models it as continuous joints with CAD-density inertials; the Menagerie derivation supplies real joint ranges and plausible link inertials, and is already pinned by this repository.'),
  row('gripper mechanism', 'single actuated moving jaw, source collision primitives', PARAMETER_EVIDENCE.SOURCE_DERIVED, SO_ARM101_SOURCE.path, 'No equality constraint, weld, or attachment flag exists in any LeKiwi package.'),
  row('wheel actuator type', 'bounded velocity servo', PARAMETER_EVIDENCE.SOURCE_DERIVED, LEROBOT_SOURCE.path,
    'The pinned LeRobot driver writes Operating_Mode VELOCITY to the three base ST3215 servos, so a bounded MuJoCo velocity actuator with an explicit torque limit is the matched abstraction.'),
  row('wheel control range', 4.60061, PARAMETER_EVIDENCE.SOURCE_DERIVED, LEROBOT_SOURCE.path,
    'LeRobot max_raw = 3000 ticks at 4096 ticks per 360 deg = 263.7 deg/s = 4.60061 rad/s. This is a software/interface limit, not a measured motor no-load speed.'),
  row('wheel torque limit', 2.94, PARAMETER_EVIDENCE.ESTIMATED, 'ST3215/STS3215 30 kg cm servo class; matches the SO-101 package already pinned here', 'Not a measurement of an installed LeKiwi base servo.'),
  row('wheel servo gain kv', 2.2, PARAMETER_EVIDENCE.ESTIMATED, 'repository-authored', 'Chosen so the servo saturates its torque limit at roughly 1.3 rad/s of speed error.'),
  row('wheel joint damping / friction loss / armature', [0.004, 0.006, 0.01], PARAMETER_EVIDENCE.ESTIMATED, 'repository-authored', 'Gearbox back-drive behaviour is not measured.'),
  row('roller joint damping / friction loss / armature', [2e-5, 2e-5, 2e-6], PARAMETER_EVIDENCE.ESTIMATED, 'repository-authored', 'Roller bearing losses are not measured.'),
  row('floor / roller sliding friction', 0.85, PARAMETER_EVIDENCE.ESTIMATED, 'repository-authored', 'A declared flat indoor laboratory surface. Not a measured floor.'),
  row('reduced-traction surface', 0.02, PARAMETER_EVIDENCE.ESTIMATED, 'repository-authored adverse-condition fixture', 'A deliberately degraded surface for the traction gate; not a modelled real material.'),
  row('transfer bench worktop', '290 x 290 mm top at 211 mm, 24 mm thick', PARAMETER_EVIDENCE.ESTIMATED, LEGACY_TASK_SOURCE.path, 'Configured educational task geometry from the pinned legacy scenario, not measured laboratory hardware.'),
  row('empty beaker envelope', '74 x 80 x 74 mm', PARAMETER_EVIDENCE.ESTIMATED, LEGACY_TASK_SOURCE.path, 'Configured legacy envelope, repaired into a hollow wall/rim vessel so a rim pinch is a real contact rather than a convex block grasp.'),
  row('beaker mass', 0.06, PARAMETER_EVIDENCE.ESTIMATED, 'repository-authored', 'The legacy scenario declares no mass. Not a weighed vessel.'),
  row('canonical visual joint convention', 'five physical-to-visual sign/zero transforms plus normalized gripper mapping', PARAMETER_EVIDENCE.SOURCE_DERIVED, 'pinned LeKiwi canonical URDF chain compared with pinned Menagerie SO-ARM101 chain', 'Presentation-only reconciliation. Corresponding arm pivots agree within single-digit millimetres over randomized poses; wrist-roll sign/zero is additionally checked from the downstream gripper-hinge axis. It never feeds state back into MuJoCo.'),
  row('hardware alignment', 'none', PARAMETER_EVIDENCE.CALIBRATION_REQUIRED, '-', 'No measurement of an assembled LeKiwi was used or is claimed anywhere in Phase 5B.'),
]);

export function reconciliationByEvidence(evidence) {
  return LEKIWI_RECONCILIATION.filter((item) => item.evidence === evidence);
}

export function assertReconciliationCoverage() {
  const seen = new Set(LEKIWI_RECONCILIATION.map((item) => item.parameter));
  const required = [
    'base frame / forward direction', 'wheel centre (left)', 'wheel centre (back)', 'wheel centre (right)',
    'wheel joint axes', 'wheel radius', 'effective chassis radius per wheel', 'arm mount transform',
    'arm joint axes / limits / inertials', 'gripper mechanism', 'chassis composite mass', 'chassis composite inertia',
    'wheel mass', 'wheel actuator type', 'wheel control range', 'wheel torque limit', 'hardware alignment',
  ];
  const missing = required.filter((name) => !seen.has(name));
  if (missing.length) throw new Error(`LeKiwi reconciliation table is missing: ${missing.join(', ')}`);
  for (const item of LEKIWI_RECONCILIATION) {
    if (!Object.values(PARAMETER_EVIDENCE).includes(item.evidence)) {
      throw new Error(`LeKiwi reconciliation row ${item.parameter} has unknown evidence ${item.evidence}`);
    }
  }
  return LEKIWI_RECONCILIATION.length;
}
