export const BALL_MASS_KG = 0.045;
export const BALL_KICK_IMPULSE_NS = 0.0405;
export const BALL_KICK_CONTACT_RADIUS_M = 0.18;
export const BALL_KICK_DELAY_SECONDS = 0.16;
export const BALL_MAX_SPEED_MPS = 1.2;
export const BALL_ROLLING_DECELERATION_MPS2 = 0.45;
export const BALL_STOP_SPEED_MPS = 0.015;

function finite(value) { return Number.isFinite(Number(value)) ? Number(value) : 0; }
function planarMagnitude([x, y]) { return Math.hypot(x, y); }
function normalizePlanar(vector, fallback) {
  const magnitude = planarMagnitude(vector);
  return magnitude > 1e-9 ? [vector[0] / magnitude, vector[1] / magnitude] : fallback;
}

function rootAxes(quaternion) {
  const [w, x, y, z] = Array.from(quaternion || [], finite);
  const forward = normalizePlanar([1 - 2 * (y * y + z * z), 2 * (x * y + w * z)], [1, 0]);
  return { forward, left: [-forward[1], forward[0]] };
}

export function applyRollingResistance(velocity, dtSeconds) {
  const next = Array.from({ length: 3 }, (_, index) => finite(velocity?.[index]));
  const speed = planarMagnitude(next);
  const duration = Math.max(0, finite(dtSeconds));
  if (speed <= BALL_STOP_SPEED_MPS || duration === 0) {
    if (speed <= BALL_STOP_SPEED_MPS) { next[0] = 0; next[1] = 0; }
    return Object.freeze(next);
  }
  const slowedSpeed = Math.max(0, speed - BALL_ROLLING_DECELERATION_MPS2 * duration);
  if (slowedSpeed <= BALL_STOP_SPEED_MPS) { next[0] = 0; next[1] = 0; return Object.freeze(next); }
  const scale = slowedSpeed / speed;
  next[0] *= scale; next[1] *= scale;
  return Object.freeze(next);
}

export function applyConfiguredKick({ skill, rootPosition, rootQuaternion, ballPosition, ballVelocity }) {
  const side = skill === 'kick_left' ? 1 : skill === 'kick_right' ? -1 : 0;
  const velocity = Array.from({ length: 3 }, (_, index) => finite(ballVelocity?.[index]));
  if (!side) return Object.freeze({ contacted: false, velocity: Object.freeze(velocity) });
  const root = Array.from({ length: 3 }, (_, index) => finite(rootPosition?.[index]));
  const ball = Array.from({ length: 3 }, (_, index) => finite(ballPosition?.[index]));
  const { forward, left } = rootAxes(rootQuaternion);
  const foot = [root[0] + forward[0] * 0.17 + left[0] * side * 0.06, root[1] + forward[1] * 0.17 + left[1] * side * 0.06];
  const delta = [ball[0] - foot[0], ball[1] - foot[1]];
  if (planarMagnitude(delta) > BALL_KICK_CONTACT_RADIUS_M) return Object.freeze({ contacted: false, velocity: Object.freeze(velocity) });
  const direction = normalizePlanar(delta, forward);
  if (direction[0] * forward[0] + direction[1] * forward[1] < 0.2) return Object.freeze({ contacted: false, velocity: Object.freeze(velocity) });
  const impulseSpeed = BALL_KICK_IMPULSE_NS / BALL_MASS_KG;
  velocity[0] += direction[0] * impulseSpeed;
  velocity[1] += direction[1] * impulseSpeed;
  const speed = planarMagnitude(velocity);
  if (speed > BALL_MAX_SPEED_MPS) { const scale = BALL_MAX_SPEED_MPS / speed; velocity[0] *= scale; velocity[1] *= scale; }
  return Object.freeze({ contacted: true, velocity: Object.freeze(velocity) });
}
