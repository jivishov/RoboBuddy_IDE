import { LEKIWI_WHEEL_NAMES } from './lekiwi-source-audit.js';

// Body-to-wheel mapping for the three-wheel Kiwi drive.
//
// The structural form is the pinned LeRobot mapping
//   huggingface/lerobot@7e241bd630a3719a56157a497ce5d08f244784f1
//   src/lerobot/robots/lekiwi/lekiwi.py::_body_to_wheel_raw
//
//   angles           = radians([240, 0, 120] - 90) = [150, -90, 30] deg for [left, back, right]
//   m                = [[cos(a), sin(a), base_radius] for a in angles]
//   wheel_linear     = m . [x_vel, y_vel, theta_rad_per_s]
//   wheel_angular    = wheel_linear / wheel_radius
//
// Two parameter sets are declared. `LEROBOT_DEFAULT_PARAMETERS` are the controller/interface
// defaults published by that revision. `SOURCE_PARAMETERS` replaces the two scalars with the
// per-wheel geometry read from the pinned LeKiwi URDF: each wheel's drive direction is derived
// from its own joint axis and each wheel's rotation coefficient is its own moment arm about
// the wheel centroid. Both sets are exercised by the deterministic mapping tests, and their
// agreement is reported rather than assumed.

export const WHEEL_ORDER = Object.freeze([...LEKIWI_WHEEL_NAMES]);

const wheel = (id, driveAngleDeg, driveDirection, momentArmM, modelAxis, urdfAxis, positionM) => Object.freeze({
  id, driveAngleDeg, driveDirection: Object.freeze(driveDirection), momentArmM,
  modelAxis: Object.freeze(modelAxis), urdfAxis: Object.freeze(urdfAxis), positionM: Object.freeze(positionM),
});

// Source-derived per-wheel geometry in the LeKiwi model frame (x forward, y left, z up).
// `modelAxis` is the LeRobot-positive spin axis; `urdfAxis` is the raw pinned URDF axis, which
// is anti-parallel to it. `driveDirection` is modelAxis x zHat, i.e. the direction the wheel
// centre travels for a positive wheel angular velocity.
export const LEKIWI_WHEEL_GEOMETRY = Object.freeze([
  wheel('base_left_wheel', 150, [-0.866025, 0.5], 0.109842, [-0.5, -0.866025, 0], [-0.866025, 0.5, 0], [0.059193, 0.092659, 0]),
  wheel('base_back_wheel', -90, [0, -1], 0.119842, [1, 0, 0], [0, -1, 0], [-0.119842, -0.000379, 0]),
  wheel('base_right_wheel', 30, [0.866025, 0.5], 0.110242, [-0.5, 0.866025, 0], [0.866025, 0.5, 0], [0.060649, -0.09228, 0]),
]);

export const LEROBOT_DEFAULT_PARAMETERS = Object.freeze({
  id: 'lerobot-default',
  wheelRadiusM: 0.05,
  rows: Object.freeze(WHEEL_ORDER.map((id, index) => {
    const angleDeg = [150, -90, 30][index];
    const angle = angleDeg * Math.PI / 180;
    return Object.freeze({ id, driveDirection: Object.freeze([Math.cos(angle), Math.sin(angle)]), momentArmM: 0.125 });
  })),
});

export const SOURCE_PARAMETERS = Object.freeze({
  id: 'lekiwi-urdf-efa608d',
  wheelRadiusM: 0.0508,
  rows: Object.freeze(LEKIWI_WHEEL_GEOMETRY.map((item) => Object.freeze({
    id: item.id, driveDirection: Object.freeze([...item.driveDirection]), momentArmM: item.momentArmM,
  }))),
});

// LeRobot exposes theta.vel in degrees per second at the public boundary and converts to
// radians per second before the mapping. The physical package keeps that compatibility.
export const PUBLIC_THETA_UNIT = 'deg/s';
export const MAX_WHEEL_RAD_S = 4.60061;     // LeRobot max_raw 3000 ticks at 4096 ticks / 360 deg
export const MAX_WHEEL_RAW_TICKS = 3000;
export const TICKS_PER_DEGREE = 4096 / 360;

function finite(value, label) {
  const number = Number(value);
  if (!Number.isFinite(number)) throw new TypeError(`${label} must be a finite number`);
  return number;
}

export function degreesPerSecondToRadians(thetaVelDegS) {
  return finite(thetaVelDegS, 'theta.vel') * Math.PI / 180;
}

export function radiansPerSecondToDegrees(thetaRadS) {
  return finite(thetaRadS, 'theta rad/s') * 180 / Math.PI;
}

/**
 * Convert a desired chassis velocity into bounded wheel angular-velocity targets.
 * @param {{x:number,y:number,thetaRadS:number}} body metres per second and radians per second
 * @param {object} parameters SOURCE_PARAMETERS (default) or LEROBOT_DEFAULT_PARAMETERS
 * @param {{maxWheelRadS?:number}} options
 */
export function bodyToWheelRadS({ x = 0, y = 0, thetaRadS = 0 } = {}, parameters = SOURCE_PARAMETERS, { maxWheelRadS = MAX_WHEEL_RAD_S } = {}) {
  const vx = finite(x, 'x.vel');
  const vy = finite(y, 'y.vel');
  const omega = finite(thetaRadS, 'theta.vel');
  const limit = finite(maxWheelRadS, 'maxWheelRadS');
  if (limit <= 0) throw new RangeError('maxWheelRadS must be positive');
  const raw = parameters.rows.map((row) => (
    (row.driveDirection[0] * vx + row.driveDirection[1] * vy + row.momentArmM * omega) / parameters.wheelRadiusM
  ));
  // LeRobot scales all three wheels proportionally when any one saturates, which preserves the
  // requested motion direction instead of distorting it. The physical package keeps that rule.
  const peak = Math.max(...raw.map((value) => Math.abs(value)));
  const scale = peak > limit ? limit / peak : 1;
  const targets = {};
  parameters.rows.forEach((row, index) => { targets[row.id] = raw[index] * scale; });
  return Object.freeze({ targetsRadS: Object.freeze(targets), saturated: scale < 1, scale, unclampedRadS: Object.freeze(raw) });
}

/** Invert the mapping: wheel angular velocities to the chassis velocity they imply. */
export function wheelRadSToBody(wheelRadS = {}, parameters = SOURCE_PARAMETERS) {
  const rows = parameters.rows;
  const linear = rows.map((row) => finite(wheelRadS[row.id], `wheel ${row.id}`) * parameters.wheelRadiusM);
  const a = rows.map((row) => [row.driveDirection[0], row.driveDirection[1], row.momentArmM]);
  // 3x3 solve by Cramer's rule; the Kiwi matrix is well conditioned for all three parameter sets.
  const det = (m) => (
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
    - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
    + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
  );
  const base = det(a);
  if (Math.abs(base) < 1e-12) throw new Error('Kiwi mapping matrix is singular');
  const replaced = (column) => det(a.map((rowValues, index) => rowValues.map((value, position) => (position === column ? linear[index] : value))));
  return Object.freeze({ x: replaced(0) / base, y: replaced(1) / base, thetaRadS: replaced(2) / base });
}

/** LeRobot raw servo ticks for a wheel angular velocity, for interface-compatibility reporting. */
export function radiansPerSecondToRawTicks(radS) {
  return Math.round(radiansPerSecondToDegrees(radS) * TICKS_PER_DEGREE);
}

/** Public LeRobot-shaped action to a validated chassis command. */
export function publicActionToBodyCommand(action = {}) {
  if (!action || typeof action !== 'object' || Array.isArray(action)) throw new TypeError('action must be an object');
  const x = finite(action['x.vel'] ?? 0, 'x.vel');
  const y = finite(action['y.vel'] ?? 0, 'y.vel');
  const thetaRadS = degreesPerSecondToRadians(action['theta.vel'] ?? 0);
  return Object.freeze({ x, y, thetaRadS });
}

/** Maximum chassis speed the bounded wheel targets can sustain along a body direction. */
export function maxBodySpeedMS(direction = [1, 0], parameters = SOURCE_PARAMETERS, { maxWheelRadS = MAX_WHEEL_RAD_S } = {}) {
  const [dx, dy] = direction;
  const norm = Math.hypot(dx, dy) || 1;
  const unit = [dx / norm, dy / norm];
  const perUnit = parameters.rows.map((row) => Math.abs(row.driveDirection[0] * unit[0] + row.driveDirection[1] * unit[1]) / parameters.wheelRadiusM);
  const peak = Math.max(...perUnit);
  return peak > 0 ? maxWheelRadS / peak : 0;
}
