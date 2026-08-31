import { MICRODUCK_COMMANDS } from './command-catalog.js';
import { deepFreeze } from './contract.js';

const TOF_SIZE = 64;

export function createModeledTof({ distanceM = 0.4, source = 'synthetic', sequence = 0, valuesM: sampledValues = null } = {}) {
  const range = MICRODUCK_COMMANDS.set_tof_stimulus.ui.fields.distanceM.range;
  const distance = Math.max(range[0], Math.min(range[1], Number(distanceM) || range[1]));
  const syntheticValues = Array.from({ length: TOF_SIZE }, (_, index) => {
    const row = Math.floor(index / 8);
    const col = index % 8;
    const radial = ((row - 3.5) ** 2 + (col - 3.5) ** 2) / 24.5;
    const lattice = ((row * 13 + col * 7 + sequence) % 9) * 0.001;
    return Number(Math.min(range[1], distance + radial * 0.07 + lattice).toFixed(3));
  });
  const valuesM = Array.isArray(sampledValues) && sampledValues.length === TOF_SIZE
    ? sampledValues.map((value) => Number(Math.max(range[0], Math.min(range[1], Number(value) || range[1])).toFixed(3)))
    : syntheticValues;
  const minimumM = Math.min(...valuesM);
  return deepFreeze({ rows: 8, cols: 8, valuesM, minimumM, usable: valuesM.filter((value) => value < range[1]).length, source, frame: 'tof', modeled: true, sequence });
}

export function deriveFrameAngularVelocity(previousQuaternion, currentQuaternion, dtSeconds) {
  const previous = normalizeQuaternion(previousQuaternion);
  const current = normalizeQuaternion(currentQuaternion);
  const inversePrevious = [-previous[0], -previous[1], -previous[2], previous[3]];
  let delta = multiplyQuaternions(current, inversePrevious);
  if (delta[3] < 0) delta = delta.map((value) => -value);
  const vectorLength = Math.hypot(delta[0], delta[1], delta[2]);
  const dt = Number(dtSeconds);
  if (!Number.isFinite(dt) || dt <= 0 || vectorLength < 1e-9) return deepFreeze([0, 0, 0]);
  const angle = 2 * Math.atan2(vectorLength, Math.max(0, delta[3]));
  const world = delta.slice(0, 3).map((value) => (value / vectorLength) * angle / dt);
  return deepFreeze(rotateVector(world, [-current[0], -current[1], -current[2], current[3]]));
}

export function deriveModeledImu({ trunkGyro = [0, 0, 0], projectedGravity = [0, 0, -1], headGyro = [0, 0, 0] } = {}) {
  return deepFreeze({
    trunk: { frame: 'imu', gyro: Array.from(trunkGyro, Number), projectedGravity: Array.from(projectedGravity, Number), modeled: true },
    head: { frame: 'head_imu', gyro: Array.from(headGyro, Number), modeled: true },
  });
}

function normalizeQuaternion(value = [0, 0, 0, 1]) {
  const quaternion = Array.from(value, Number);
  const length = Math.hypot(...quaternion) || 1;
  return quaternion.map((item, index) => (Number.isFinite(item) ? item : index === 3 ? 1 : 0) / length);
}

function multiplyQuaternions(a, b) {
  return [
    a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
    a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
    a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
    a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
  ];
}

function rotateVector(vector, quaternion) {
  const rotated = multiplyQuaternions(multiplyQuaternions(quaternion, [...vector, 0]), [-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]]);
  return rotated.slice(0, 3).map((value) => Number(value.toFixed(9)));
}

export function mapTheremin(distanceM, { previous = null, nowMs = 0, lastUsableMs = -Infinity } = {}) {
  const [near, far] = MICRODUCK_COMMANDS.theremin.ui.playableRangeM;
  const holdMs = MICRODUCK_COMMANDS.theremin.ui.dropoutHoldMs;
  const distance = Number(distanceM);
  if (!Number.isFinite(distance) || distance < near || distance > far) {
    if (previous?.active && nowMs - lastUsableMs <= holdMs) return deepFreeze({ ...previous, held: true });
    return deepFreeze({ active: false, frequencyHz: 0, mouth: 0, held: false });
  }
  const amount = 1 - ((distance - near) / (far - near));
  const frequencyHz = 146.83 * (2 ** (amount * 2.15));
  return deepFreeze({ active: true, frequencyHz: Number(frequencyHz.toFixed(2)), mouth: Number((0.18 + amount * 0.72).toFixed(3)), held: false });
}
