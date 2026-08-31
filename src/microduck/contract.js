export const PHYSICS_TIMESTEP_SECONDS = 0.005;
export const POLICY_DECIMATION = 4;
export const POLICY_TIMESTEP_SECONDS = PHYSICS_TIMESTEP_SECONDS * POLICY_DECIMATION;

export const WIRE_JOINT_ORDER = Object.freeze([
  'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee', 'left_ankle',
  'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll', 'mouth',
  'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle',
]);
export const MOUTH_WIRE_INDEX = 9;
export const POLICY_JOINT_ORDER = Object.freeze(WIRE_JOINT_ORDER.filter((_, index) => index !== MOUTH_WIRE_INDEX));
export const HOME_WIRE_POSITION = Object.freeze([
  0, -0.0873, -0.4579, -0.0049, 0.4530,
  0.3491, 0.3491, 0, 0, 0,
  0, 0.0873, 0.4579, 0.0049, -0.4530,
]);
export const HOME_POLICY_POSITION = Object.freeze(HOME_WIRE_POSITION.filter((_, index) => index !== MOUTH_WIRE_INDEX));

export const POLICY_FILES = Object.freeze({
  walking: 'alpha_walking.onnx',
  stand: 'alpha_stand.onnx',
  ground_pick: 'alpha_ground_pick.onnx',
  sitstand: 'alpha_sitstand.onnx',
  kick_left: 'ball_kick_left.onnx',
  kick_right: 'ball_kick_right.onnx',
  roulade: 'roulade.onnx',
  roller: 'roller.onnx',
  roller_crouch: 'roller_crouch.onnx',
});
export const EAGER_WALKING_POLICIES = Object.freeze(['walking', 'stand', 'ground_pick', 'sitstand', 'kick_left', 'kick_right', 'roulade']);
export const LAZY_ROLLER_POLICIES = Object.freeze(['roller', 'roller_crouch']);

export const MODE_TUNING = deepFreeze({
  walking: {
    movement: { vx: [-0.3, 0.3], vy: [-0.3, 0.3], yaw: [-1.5, 1.5] },
    actionScale: 0.9,
    groundPickPeriodSeconds: 4,
    groundPickActionScale: 1,
  },
  roller: {
    movement: { vx: [-0.5, 0.6], vy: [0, 0], yaw: [-0.3, 0.3] },
    actionScale: 0.8,
    groundPickPeriodSeconds: 3,
    groundPickActionScale: 0.8,
  },
});
export const POLICY_TUNING = deepFreeze({
  standingActionScale: 1,
  standingGainRatio: 0.8,
  headLowPassAlpha: 0.5,
  legsLowPassAlpha: 0.7,
  groundPickEndPhase: 0.7,
  kickDurationSeconds: 0.5,
  rouladeDurationSeconds: 1,
  sitRiseDurationSeconds: 1,
  standingThreshold: 0.01,
  maxTargetStepRad: { legs: 0.06, head: 0.04 },
});

export const BODY_LIMITS = deepFreeze({ z: [-0.025, 0.010], roll: [-0.26, 0.26], pitch: [-0.26, 0.26] });
export const HEAD_LIMITS = deepFreeze({
  neckPitch: [-1.5707963267948966, 1.0471975511965976],
  headPitch: [-1.5707963267948966, 1.5707963267948966],
  headYaw: [-2.967059728390364, 2.967059728390357],
  headRoll: [-0.4363323129986037, 0.43633231299856107],
});

export function clamp(value, [minimum, maximum]) {
  const finite = Number.isFinite(Number(value)) ? Number(value) : 0;
  return Math.max(minimum, Math.min(maximum, finite));
}

export function deepFreeze(value) {
  if (!value || typeof value !== 'object' || Object.isFrozen(value)) return value;
  for (const child of Object.values(value)) deepFreeze(child);
  return Object.freeze(value);
}

export function policyToWire(values, mouth = 0) {
  if (!values || values.length !== 14) throw new TypeError('Expected exactly fourteen policy joint values.');
  const wire = [];
  for (let index = 0; index < 15; index += 1) wire.push(index === MOUTH_WIRE_INDEX ? Number(mouth) : Number(values[index < MOUTH_WIRE_INDEX ? index : index - 1]));
  return wire;
}

export function wireToPolicy(values) {
  if (!values || values.length !== 15) throw new TypeError('Expected exactly fifteen wire joint values.');
  return Array.from(values, Number).filter((_, index) => index !== MOUTH_WIRE_INDEX);
}
