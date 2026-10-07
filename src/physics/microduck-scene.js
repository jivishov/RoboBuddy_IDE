import { PHYSICS_BACKEND_API_VERSION } from './backend-contract.js';
import {
  MICRODUCK_BALL_BODY, MICRODUCK_BALL_OFFSET_M, MICRODUCK_BALL_RADIUS_M,
  MICRODUCK_FLOOR_ID, MICRODUCK_GROUNDCONTACT_PACKAGE, MICRODUCK_JOINT_CONTROLLER, MICRODUCK_KICK_PACKAGE,
  MICRODUCK_LOW_TRACTION_PACKAGE, MICRODUCK_ROBOT_ID, MICRODUCK_WALK_PACKAGE,
} from './microduck-model-package.js';

// The bounded command surface. These are the ranges a human, a Python program or a WebMCP
// tool may ask for; the policy decides what the robot does with them and the plant decides
// what actually happens.
//
// The linear and angular bounds are the training command ranges of the matched velstand
// recipe (lin_vel_x +/-0.4, lin_vel_y +/-0.3, ang_vel_z +/-1.0). Asking outside them would be
// asking the policy for behaviour it never saw.
export const MICRODUCK_COMMAND_LIMITS = Object.freeze({
  vxMS: Object.freeze([-0.4, 0.4]),
  vyMS: Object.freeze([-0.3, 0.3]),
  vyawRadS: Object.freeze([-1.0, 1.0]),
  neckPitchRad: Object.freeze([-1.5707963267948966, 1.0471975511965976]),
  headPitchRad: Object.freeze([-1.5707963267948966, 1.5707963267948966]),
  headYawRad: Object.freeze([-2.967059728390364, 2.967059728390357]),
  headRollRad: Object.freeze([-0.4363323129986037, 0.43633231299856107]),
  bodyZM: Object.freeze([-0.03, 0.03]),
  bodyRollRad: Object.freeze([-0.5235987755982988, 0.5235987755982988]),
  bodyPitchRad: Object.freeze([-0.5235987755982988, 0.5235987755982988]),
});

// The command magnitude below which the velstand policy does not initiate a gait in this
// plant. Measured, not assumed: at 0.25 m/s the native reference initiates a gait in the
// upstream mesh model but not in this one, and both walk from 0.30 m/s upward.
export const MICRODUCK_GAIT_ONSET_MS = 0.30;

export const MICRODUCK_SCENE_REVISION = 'microduck-physical-2026-09-09';

const floorFixture = () => ({
  id: MICRODUCK_FLOOR_ID,
  kind: 'plane',
  description: 'flat indoor floor, sliding friction as declared by the selected package',
});

function scene({ id, modelPackage, legacyTaskId = null, objects = [], taskGoal = null }) {
  return Object.freeze({
    schemaVersion: PHYSICS_BACKEND_API_VERSION,
    id,
    revision: MICRODUCK_SCENE_REVISION,
    robotId: MICRODUCK_ROBOT_ID,
    modelPackage: modelPackage.id,
    legacyTaskId,
    physics: { ...modelPackage.physics },
    fixtures: [floorFixture()],
    objects,
    controllers: [MICRODUCK_JOINT_CONTROLLER],
    taskGoal,
  });
}

export const MICRODUCK_WALK_SCENE = scene({
  id: 'microduck-physical-walk',
  modelPackage: MICRODUCK_WALK_PACKAGE,
  taskGoal: {
    id: 'microduck-contact-driven-locomotion',
    description: 'Follow a bounded velocity command through actual foot-floor contact.',
    successCriteria: 'commanded-axis displacement with alternating foot-contact transitions and no fall',
    evaluatedFrom: 'authoritative MuJoCo trunk pose and named foot-floor contacts',
  },
});

export const MICRODUCK_LOW_TRACTION_SCENE = scene({
  id: 'microduck-physical-walk-lowtraction',
  modelPackage: MICRODUCK_LOW_TRACTION_PACKAGE,
  taskGoal: {
    id: 'microduck-traction-negative-control',
    description: 'Adverse-condition fixture: the same command on a declared reduced-traction floor.',
    successCriteria: 'commanded propulsion must materially degrade or disappear',
    evaluatedFrom: 'authoritative MuJoCo trunk pose and named foot-floor contacts',
  },
});

export const MICRODUCK_GROUNDCONTACT_SCENE = scene({
  id: 'microduck-physical-groundcontact',
  modelPackage: MICRODUCK_GROUNDCONTACT_PACKAGE,
  taskGoal: {
    id: 'microduck-body-ground-contact-skills',
    description: 'Run body-on-ground skills against the pinned all-collision robot plant.',
    successCriteria: 'outcomes come from the requested policy plus actual MuJoCo body/floor contact; reset/setup never counts as progress',
    evaluatedFrom: 'authoritative MuJoCo trunk pose and named robot/floor contacts',
  },
});

export const MICRODUCK_KICK_SCENE = scene({
  id: 'microduck-physical-kick',
  modelPackage: MICRODUCK_KICK_PACKAGE,
  objects: [{
    id: MICRODUCK_BALL_BODY,
    kind: 'free-body',
    radiusM: MICRODUCK_BALL_RADIUS_M,
    initialXYM: [...MICRODUCK_BALL_OFFSET_M],
    description: 'source 70 mm / 15 g ball prop at the source kick-task placement',
  }],
  taskGoal: {
    id: 'microduck-contact-driven-kick',
    description: 'Move the ball by actual foot-ball contact.',
    successCriteria: 'at least one named sole/ball contact, and ball displacement that follows it',
    evaluatedFrom: 'authoritative MuJoCo contact pairs and ball free-body pose',
  },
});

export const MICRODUCK_SCENES = Object.freeze({
  walk: MICRODUCK_WALK_SCENE,
  lowTraction: MICRODUCK_LOW_TRACTION_SCENE,
  groundContact: MICRODUCK_GROUNDCONTACT_SCENE,
  kick: MICRODUCK_KICK_SCENE,
});

function clampTo(value, [minimum, maximum], label) {
  const number = Number(value);
  if (!Number.isFinite(number)) throw new TypeError(`${label} must be finite`);
  return { value: Math.max(minimum, Math.min(maximum, number)), limited: number < minimum || number > maximum };
}

/**
 * Clamp a requested command to the declared bounds and report what was limited.
 * The caller always learns the difference between what it asked for and what was accepted.
 */
export function boundedMicroDuckCommand(requested = {}) {
  const L = MICRODUCK_COMMAND_LIMITS;
  const limitedBy = [];
  const take = (raw, range, label) => {
    const { value, limited } = clampTo(raw ?? 0, range, label);
    if (limited) limitedBy.push(label);
    return value;
  };
  const twist = [
    take(requested.vx, L.vxMS, 'vx'),
    take(requested.vy, L.vyMS, 'vy'),
    take(requested.vyaw, L.vyawRadS, 'vyaw'),
  ];
  const head = [
    take(requested.neckPitch, L.neckPitchRad, 'neckPitch'),
    take(requested.headPitch, L.headPitchRad, 'headPitch'),
    take(requested.headYaw, L.headYawRad, 'headYaw'),
    take(requested.headRoll, L.headRollRad, 'headRoll'),
  ];
  const body = {
    z: take(requested.bodyZ, L.bodyZM, 'bodyZ'),
    roll: take(requested.bodyRoll, L.bodyRollRad, 'bodyRoll'),
    pitch: take(requested.bodyPitch, L.bodyPitchRad, 'bodyPitch'),
  };
  return { command: { twist, head, body }, limitedBy: Object.freeze(limitedBy) };
}
