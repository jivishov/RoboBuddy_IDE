export const MICRODUCK_FIELD_HALF_EXTENT_M = 4;
export const MICRODUCK_FIELD_SIZE_M = MICRODUCK_FIELD_HALF_EXTENT_M * 2;
export const MICRODUCK_DUCK_BOUNDARY_RADIUS_M = 0.07;
export const MICRODUCK_BALL_BOUNDARY_RADIUS_M = 0.035;

function finite(value) { return Number.isFinite(Number(value)) ? Number(value) : 0; }

// This is a configured browser-simulation boundary, not a sourced MicroDuck
// collision model. Reaching any field edge stops planar travel rather than
// allowing a free body to leave the visible workcell.
export function stopPlanarBodyAtFieldEdge(position, velocity, radiusM = 0) {
  const nextPosition = Array.from({ length: 3 }, (_, index) => finite(position?.[index]));
  const nextVelocity = Array.from({ length: 3 }, (_, index) => finite(velocity?.[index]));
  const radius = Math.max(0, Math.min(MICRODUCK_FIELD_HALF_EXTENT_M, finite(radiusM)));
  const limit = MICRODUCK_FIELD_HALF_EXTENT_M - radius;
  let reachedBoundary = false;
  for (const axis of [0, 1]) {
    if (nextPosition[axis] > limit) { nextPosition[axis] = limit; reachedBoundary = true; }
    else if (nextPosition[axis] < -limit) { nextPosition[axis] = -limit; reachedBoundary = true; }
  }
  if (reachedBoundary) { nextVelocity[0] = 0; nextVelocity[1] = 0; }
  return Object.freeze({ position: Object.freeze(nextPosition), velocity: Object.freeze(nextVelocity), reachedBoundary });
}
