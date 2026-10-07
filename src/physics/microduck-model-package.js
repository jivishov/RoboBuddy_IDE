import { PARAMETER_EVIDENCE, registerModelPackage } from './model-registry.js';
import { MICRODUCK_RL_SOURCE, MICRODUCK_RUNTIME_SOURCE } from './microduck-source-audit.js';
import {
  MICRODUCK_CONTROL_DECIMATION,
  MICRODUCK_HOME_POSITION_RAD,
  MICRODUCK_PHYSICS_TIMESTEP_SECONDS,
  MICRODUCK_POLICY_JOINT_ORDER,
} from './microduck-controller.js';
import {
  MICRODUCK_BAM_M6,
  MICRODUCK_BAM_REVISION,
  MICRODUCK_BAM_VERSION,
  MICRODUCK_DEPLOYMENT_PLANT_PROFILE,
  MICRODUCK_TRAINING_PLANT_PROFILE,
  microDuckBamForceCeilingNm,
} from './microduck-bam-plant.js';

export const MICRODUCK_ROBOT_ID = 'microduck_alpha_biped';
export const MICRODUCK_JOINT_CONTROLLER = 'microduck_joint_position';
export const MICRODUCK_FLOOR_ID = 'microduck_floor';
export const MICRODUCK_TRUNK_BODY = 'trunk_base';
export const MICRODUCK_BALL_BODY = 'microduck_ball';
export const MICRODUCK_FOOT_GEOMS = Object.freeze(['left_foot_collision', 'right_foot_collision']);
export const MICRODUCK_BALL_GEOM = 'microduck_ball_geom';

// Source-XML fallback actuator. microduck_rl's actual training builder replaces this position
// actuator with BAM M6 before simulation. These constants remain for source-model validation
// and backwards-compatible diagnostics; they are no longer the browser plant implementation.
export const MICRODUCK_SERVO_KP_NM_RAD = 0.55;
export const MICRODUCK_SERVO_KV = 0;
export const MICRODUCK_SERVO_FORCE_NM = 0.96;
export const MICRODUCK_SERVO_CTRL_RAD = 10;
export const MICRODUCK_NOMINAL_FIRMWARE_GAIN = 200;

// Equivalent interpolation through the source XML's two annotated fallback points
// (200 -> 0.55, 125 -> 0.35). Do not use this as the BAM actuator law.
export function microDuckKpForFirmwareGain(gain) {
  const value = Number(gain);
  if (!Number.isFinite(value) || value < 0) throw new TypeError('Firmware gain must be a finite, non-negative number');
  return MICRODUCK_SERVO_KP_NM_RAD * value / MICRODUCK_NOMINAL_FIRMWARE_GAIN;
}

// Kept as source-XML evidence. BAM replaces damping/frictionloss dynamically and sets its own
// identified armature when the physical worker configures the torque-motor plant.
export const MICRODUCK_JOINT_DAMPING = 0.053;
export const MICRODUCK_JOINT_FRICTIONLOSS = 0.0048;
export const MICRODUCK_JOINT_ARMATURE = 0.0018;

export const MICRODUCK_BALL_RADIUS_M = 0.035;
export const MICRODUCK_BALL_MASS_KG = 0.015;
export const MICRODUCK_BALL_OFFSET_M = Object.freeze([0.09, -0.042]);
export const MICRODUCK_NOMINAL_FLOOR_FRICTION = 1.0;
export const MICRODUCK_LOW_TRACTION_FRICTION = 0.02;

const RUNTIME_URL = `https://github.com/${MICRODUCK_RUNTIME_SOURCE.repository}/blob/${MICRODUCK_RUNTIME_SOURCE.revision}/kinematics/assets/alpha/robot_walk.xml`;
const RL_URL = `https://github.com/${MICRODUCK_RL_SOURCE.repository}/blob/${MICRODUCK_RL_SOURCE.revision}/src/mjlab_microduck/robot/microduck`;
const BAM_URL = `https://github.com/Rhoban/bam/tree/${MICRODUCK_BAM_REVISION}`;

const JOINT_RANGES = Object.freeze({
  left_hip_yaw: [-0.4363323129985824, 0.5235987755982988],
  left_hip_roll: [-0.3839724354387516, 0.38397243543875337],
  left_hip_pitch: [-1.5707963267949037, 1.5707963267948895],
  left_knee: [-1.570796326794901, 1.5707963267948921],
  left_ankle: [-1.5707963267949019, 1.5707963267948912],
  neck_pitch: [-1.5707963267948966, 1.0471975511965976],
  head_pitch: [-1.5707963267948966, 1.5707963267948966],
  head_yaw: [-2.967059728390364, 2.967059728390357],
  head_roll: [-0.4363323129986037, 0.43633231299856107],
  right_hip_yaw: [-0.5235987755982988, 0.4363323129985824],
  right_hip_roll: [-0.3839724354387525, 0.3839724354387525],
  right_hip_pitch: [-1.5707963267949, 1.570796326794893],
  right_knee: [-1.570796326794901, 1.5707963267948921],
  right_ankle: [-1.5707963267949028, 1.5707963267948903],
});

const joints = () => MICRODUCK_POLICY_JOINT_ORDER.map((id) => ({
  id,
  rangeRad: [...JOINT_RANGES[id]],
  axis: [0, 0, 1],
  evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED,
}));

const bamForceCeiling = microDuckBamForceCeilingNm();
const actuators = () => MICRODUCK_POLICY_JOINT_ORDER.map((id) => ({
  id: `act_${id}`,
  jointId: id,
  controllerId: MICRODUCK_JOINT_CONTROLLER,
  command: 'position-rad',
  controlRangeRad: [-MICRODUCK_SERVO_CTRL_RAD, MICRODUCK_SERVO_CTRL_RAD],
  // The committed XML compiles with this fallback range and is checked before conversion.
  sourceForceRangeNm: [-MICRODUCK_SERVO_FORCE_NM, MICRODUCK_SERVO_FORCE_NM],
  // BAM's training builder converts the actuator to a motor and uses max(vin_range)*Kt/R as
  // the safe MuJoCo ceiling. The actual torque remains voltage/back-EMF limited below it.
  forceRangeNm: [-bamForceCeiling, bamForceCeiling],
  actuatorModel: 'BAM XL330 M6 voltage-domain torque motor',
  evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED,
}));

const initialJointPositionsRad = () => Object.fromEntries(
  MICRODUCK_POLICY_JOINT_ORDER.map((id, slot) => [id, MICRODUCK_HOME_POSITION_RAD[slot]]),
);

// Publish every source body, not just task-evaluation landmarks. The renderer consumes
// one coherent MuJoCo xpos/xquat snapshot instead of recomputing FK from separately integrated joint state.
const ROBOT_BODIES = Object.freeze([
  { id: MICRODUCK_TRUNK_BODY, freeJointId: 'trunk_base_freejoint' },
  ...['yaw2roll', 'hip_l', 'left_upper_leg', 'leg', 'ankle_left',
    'neck', 'neck_pitch', 'yaw_roll_motion', 'bottom_head_shell',
    'bearing_roll', 'hip_l_2', 'right_upper_leg', 'leg_2', 'ankle_right'].map((id) => ({ id })),
]);

const BASE_EVIDENCE = Object.freeze({
  articulatedHierarchy: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  jointAxesAndRanges: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  linkInertials: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  namedSitesAndFrames: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  homePose: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  actuatorModel: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  bamM6Parameters: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  trainingVoltageDelayAndFrictionDistributions: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  deployedGainSchedule: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  deployedImuPreprocessing: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  physicsTimestepAndCadence: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  collisionSetIdentityAndPlacement: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  contactParameters: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  collisionMeshBytes: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  colliderPrimitiveShape: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  deploymentElectricalCondition: PARAMETER_EVIDENCE.CALIBRATION_REQUIRED,
  hardwareAlignment: PARAMETER_EVIDENCE.CALIBRATION_REQUIRED,
});

const BASE_LIMITATIONS = Object.freeze([
  'The articulated hierarchy, body transforms, joint axes, joint ranges, link inertials and named sites are read from the pinned Apache-2.0 MicroDuck runtime model. The contact set and training-plant parameters are reconciled against the pinned microduck_rl environment.',
  `Actuation now uses BAM ${MICRODUCK_BAM_VERSION} XL330/M6 (${BAM_URL}) rather than the XML position-servo approximation. The worker applies the BAM voltage law, back-EMF, load-dependent directional/Stribeck friction, identified armature and load-dependent voltage sag. The pinned training profile is ${JSON.stringify(MICRODUCK_TRAINING_PLANT_PROFILE)}.`,
  `The interactive physical workspace uses ${MICRODUCK_DEPLOYMENT_PLANT_PROFILE.id}: the deployed 200/160 firmware-gain schedule and deployed median-of-three IMU preprocessing, with BAM at the source CPU regression's nominal 7.4 V / 0.1 V-per-Nm sag condition. Those electrical values are a repeatable source-backed rehearsal condition, not a measurement of a particular assembled robot.`,
  'Task-specific collision geometry uses the exact STL mesh bytes from the pinned microduck_rl source. Locomotion uses robot_walk.xml; explicit stand/recovery, sit/stand, ground-pick and the conservative roulade runtime mapping use robot_allcollisions.xml; kick uses that all-collision robot plus the source ball.xml prop. These 3D model files remain under upstream Creative Commons BY-SA-NC terms (version not specified upstream) and are outside RoboBuddy original-code MIT scope.',
  'The trunk is a free MuJoCo body. Nothing writes root pose, root velocity, joint state or object state outside explicit logged setup/reset. There is no weld, snap, teleport, hidden grasp attachment, kick impulse, boundary clamp or success overwrite.',
  'No assembled-MicroDuck hardware comparison is available in this repository. Absolute walking speed, recovery probability, battery/internal-resistance values, real bus latency, thermal behavior and individual actuator calibration remain hardware-validation items and are not claimed as measured truth.',
]);

function basePackage({ id, modelId, asset, sha256, variant, floorFriction, limitations = [] }) {
  return {
    id,
    modelId,
    asset,
    sha256,
    robotId: MICRODUCK_ROBOT_ID,
    source: {
      url: RUNTIME_URL,
      revision: MICRODUCK_RUNTIME_SOURCE.revision,
      variant,
      physicalEnvironmentUrl: RL_URL,
      physicalEnvironmentRevision: MICRODUCK_RL_SOURCE.revision,
      bamUrl: BAM_URL,
      bamRevision: MICRODUCK_BAM_REVISION,
    },
    license: 'Multi-license: RoboBuddy original software/floor wrapper MIT; source-derived MicroDuck/BAM software parameters Apache-2.0; pinned MicroDuck RL 3D model/collision files Creative Commons BY-SA-NC (upstream does not specify a version).',
    physics: { timestepSeconds: MICRODUCK_PHYSICS_TIMESTEP_SECONDS, integrator: 'Euler', iterations: 100, lsIterations: 50 },
    controllers: [MICRODUCK_JOINT_CONTROLLER],
    joints: joints(),
    actuators: actuators(),
    bodies: [...ROBOT_BODIES],
    initialJointPositionsRad: initialJointPositionsRad(),
    sceneConstraints: { fixtures: [MICRODUCK_FLOOR_ID], objects: [] },
    controlDecimation: MICRODUCK_CONTROL_DECIMATION,
    floorFriction,
    plant: {
      actuator: { family: 'BAM', version: MICRODUCK_BAM_VERSION, revision: MICRODUCK_BAM_REVISION, motor: MICRODUCK_BAM_M6.motor, model: MICRODUCK_BAM_M6.model },
      interactiveProfile: MICRODUCK_DEPLOYMENT_PLANT_PROFILE.id,
      trainingReferenceProfile: MICRODUCK_TRAINING_PLANT_PROFILE.id,
    },
    evidence: BASE_EVIDENCE,
    limitations: [...BASE_LIMITATIONS, ...limitations],
  };
}

export const MICRODUCK_WALK_PACKAGE = registerModelPackage(basePackage({
  id: 'microduck-walk-519142b-v1',
  modelId: 'robobuddy-microduck-walk-v1',
  asset: 'models/microduck/walk.xml',
  sha256: 'd972a3b1fd41eb204bdd8e9b578c9d8559b778d21424fb13afa6e8bef78a5a27',
  variant: 'MicroDuck alpha free-base biped with reconciled ground-contact collision set on a flat indoor floor',
  floorFriction: MICRODUCK_NOMINAL_FLOOR_FRICTION,
}));

export const MICRODUCK_LOW_TRACTION_PACKAGE = registerModelPackage(basePackage({
  id: 'microduck-walk-lowtraction-519142b-v1',
  modelId: 'robobuddy-microduck-walk-lowtraction-v1',
  asset: 'models/microduck/walk_lowtraction.xml',
  sha256: '1a2b91de42c6ceb5fea5fb0381a70052270b323bb82c36f3f2da2fc3a7d08e2d',
  variant: 'MicroDuck alpha free-base biped on a deliberately reduced-traction surface (sliding friction 0.02)',
  floorFriction: MICRODUCK_LOW_TRACTION_FRICTION,
  limitations: ['Adverse-condition fixture: sole and floor sliding friction are reduced to 0.02. This is a declared degraded surface, not a modelled real material.'],
}));

export const MICRODUCK_GROUNDCONTACT_PACKAGE = registerModelPackage(basePackage({
  id: 'microduck-groundcontact-519142b-v1',
  modelId: 'robobuddy-microduck-groundcontact-v1',
  asset: 'models/microduck/groundcontact.xml',
  sha256: '43c828db58973bc2d1a56ccba72a46f0ae89837f859f64641ea1c251d2cfd108',
  variant: 'MicroDuck alpha free-base biped with the pinned robot_allcollisions.xml mesh-contact plant for body-on-ground skills',
  floorFriction: MICRODUCK_NOMINAL_FLOOR_FRICTION,
  limitations: [
    'This package is the broad source collision plant used by pinned stand-up, sit/stand and ground-pick task configurations. It intentionally differs from the reduced walking collision plant.',
    'The pinned microduck_rl revision does not contain the roulade task configuration even though the deployed runtime carries roulade.onnx. Roulade is therefore routed to this broad contact plant conservatively and remains experimental rather than being claimed as an exact pinned training-task match.',
  ],
}));

export const MICRODUCK_KICK_PACKAGE = registerModelPackage((() => {
  const base = basePackage({
    id: 'microduck-kick-519142b-v1',
    modelId: 'robobuddy-microduck-kick-v1',
    asset: 'models/microduck/kick.xml',
    sha256: '4bb983b61918b662f6bbe9ef46b5796bb4839421a9f2c56c85dbe919a6fdd284',
    variant: 'MicroDuck alpha free-base biped plus the source 70 mm / 15 g ball prop at the source kick-task placement',
    floorFriction: MICRODUCK_NOMINAL_FLOOR_FRICTION,
  });
  return {
    ...base,
    bodies: [...base.bodies, { id: MICRODUCK_BALL_BODY, freeJointId: 'microduck_ball_free' }],
    sceneConstraints: { fixtures: [MICRODUCK_FLOOR_ID], objects: [MICRODUCK_BALL_BODY] },
    evidence: { ...base.evidence, ballMassRadiusInertia: PARAMETER_EVIDENCE.SOURCE_DERIVED, ballFriction: PARAMETER_EVIDENCE.SOURCE_DERIVED, ballPlacement: PARAMETER_EVIDENCE.SOURCE_DERIVED },
    limitations: [
      ...base.limitations,
      'The ball is the source prop from microduck_rl ball.xml: 70 mm diameter, 15 g, thin-hollow-sphere inertia 1.225e-5 kg m^2, friction (0.5, 0.005, 0.0001). It is placed at the source kick-task offset (0.09, -0.042) m. Training additionally randomised placement by +/-15 mm; the interactive deployment-reference scene keeps placement deterministic.',
      'Ball motion comes only from MuJoCo contact. There is no kick impulse, ball-velocity overwrite, rolling-resistance rule or synthetic contact inference.',
    ],
  };
})());

export const MICRODUCK_MODEL_PACKAGES = Object.freeze([
  MICRODUCK_WALK_PACKAGE,
  MICRODUCK_LOW_TRACTION_PACKAGE,
  MICRODUCK_GROUNDCONTACT_PACKAGE,
  MICRODUCK_KICK_PACKAGE,
]);