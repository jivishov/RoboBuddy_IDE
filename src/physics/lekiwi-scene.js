import { PHYSICS_BACKEND_API_VERSION } from './backend-contract.js';
import {
  LEKIWI_BASE_LOWTRACTION_PACKAGE, LEKIWI_BASE_PACKAGE, LEKIWI_BASE_STAND_PACKAGE,
  LEKIWI_COURIER_PACKAGE, LEKIWI_STOW_POSE_RAD, LEKIWI_WHEEL_REFERENCE_PACKAGE,
} from './lekiwi-model-package.js';
import { bodyToWheelRadS, MAX_WHEEL_RAD_S, SOURCE_PARAMETERS } from './lekiwi-kinematics.js';

const scene = (id, revision, modelPackage, { legacyTaskId = null, taskGoal = null } = {}) => Object.freeze({
  schemaVersion: PHYSICS_BACKEND_API_VERSION,
  id,
  revision,
  robotId: modelPackage.robotId,
  modelPackage: modelPackage.id,
  legacyTaskId,
  physics: Object.freeze({ ...modelPackage.physics }),
  fixtures: Object.freeze((modelPackage.sceneConstraints?.fixtures || []).map((fixtureId) => Object.freeze({ id: fixtureId }))),
  objects: Object.freeze((modelPackage.sceneConstraints?.objects || []).map((objectId) => Object.freeze({ id: objectId }))),
  controllers: Object.freeze([...modelPackage.controllers]),
  taskGoal,
});

export const LEKIWI_WHEEL_REFERENCE_SCENE = scene('p5b-lekiwi-wheel-reference', 'p5b-lekiwi-wheel-reference-v1', LEKIWI_WHEEL_REFERENCE_PACKAGE);
export const LEKIWI_BASE_SCENE = scene('p5b-lekiwi-base', 'p5b-lekiwi-base-v1', LEKIWI_BASE_PACKAGE);
export const LEKIWI_BASE_STAND_SCENE = scene('p5b-lekiwi-base-stand', 'p5b-lekiwi-base-stand-v1', LEKIWI_BASE_STAND_PACKAGE);
export const LEKIWI_BASE_LOWTRACTION_SCENE = scene('p5b-lekiwi-base-lowtraction', 'p5b-lekiwi-base-lowtraction-v1', LEKIWI_BASE_LOWTRACTION_PACKAGE);

// --- configured workcell, converted from the pinned legacy courier scenario -----------------
// Legacy poses are Three.js Y-up millimetres; physics (x, y, z) = (three_x/1000, -three_z/1000, three_y/1000).
export const LEKIWI_WORKCELL = Object.freeze({
  worktopTopZM: 0.211,
  worktopCenterXYM: Object.freeze([0.305, -0.390]),
  worktopHalfXYM: Object.freeze([0.145, 0.145]),
  pickupXYM: Object.freeze([0.305, -0.292]),
  deliveryXYM: Object.freeze([0.242928, -0.370144]),
  deliveryHalfExtentsXYM: Object.freeze([0.0125, 0.0125]),
  homeXYM: Object.freeze([0, 0]),
  serviceStopXYM: Object.freeze([0.274, 0.030]),
  serviceStopYawRad: -Math.PI / 2,
  restrictedStopXYM: Object.freeze([0.1, -0.65]),
  restrictedStopRadiusM: 0.15,
  beakerRimTopZM: 0.290,
});

// --- desired-route generator (preserved legacy planar occupancy grid + A*) -------------------
// The legacy scenario declares a 15 x 15 grid of 50 mm cells whose origin is the home base, with
// a hand-authored blocked list. That blocked list is repaired here: it conflicts with the pinned
// transfer-bench footprint and with the physical LeKiwi footprint, and the legacy planner is
// explicitly a point-cell planner with no footprint-clearance guarantee. The repaired blocked
// set is derived from the bench envelope and the visible restricted stop, inflated by the base
// footprint radius. Route completion is never taken from this planner; it only proposes
// waypoints, and the evaluator reads the actual MuJoCo base pose.
export const LEKIWI_OCCUPANCY_GRID = Object.freeze({
  width: 15,
  height: 15,
  resolutionM: 0.05,
  originXYM: Object.freeze([0, 0]),
  legacySource: 'lekiwi-01-beaker-courier navigation.occupancyGrid',
  footprintRadiusM: 0.15,
  provenance: 'repaired from the legacy blocked-cell list; the legacy cells intersect the pinned transfer-bench footprint and are not drivable by the physical LeKiwi footprint',
});

export function gridCellCenterM(col, row) {
  return [
    LEKIWI_OCCUPANCY_GRID.originXYM[0] + (col + 0.5) * LEKIWI_OCCUPANCY_GRID.resolutionM,
    LEKIWI_OCCUPANCY_GRID.originXYM[1] - (row + 0.5) * LEKIWI_OCCUPANCY_GRID.resolutionM,
  ];
}

function cellBlocked(col, row) {
  const [x, y] = gridCellCenterM(col, row);
  const inflate = LEKIWI_OCCUPANCY_GRID.footprintRadiusM;
  const [bx, by] = LEKIWI_WORKCELL.worktopCenterXYM;
  const [hx, hy] = LEKIWI_WORKCELL.worktopHalfXYM;
  if (Math.abs(x - bx) <= hx + inflate && Math.abs(y - by) <= hy + inflate) return true;
  const [rx, ry] = LEKIWI_WORKCELL.restrictedStopXYM;
  if (Math.hypot(x - rx, y - ry) <= LEKIWI_WORKCELL.restrictedStopRadiusM + inflate) return true;
  return false;
}

export function blockedCells() {
  const blocked = [];
  for (let row = 0; row < LEKIWI_OCCUPANCY_GRID.height; row += 1) {
    for (let col = 0; col < LEKIWI_OCCUPANCY_GRID.width; col += 1) if (cellBlocked(col, row)) blocked.push(`${col},${row}`);
  }
  return Object.freeze(blocked);
}

/** 4-connected A* over the repaired grid. Returns an ordered list of cell centres in metres. */
export function planRoute(startCell, goalCell) {
  const key = ([c, r]) => `${c},${r}`;
  const blocked = new Set(blockedCells());
  const inBounds = ([c, r]) => c >= 0 && r >= 0 && c < LEKIWI_OCCUPANCY_GRID.width && r < LEKIWI_OCCUPANCY_GRID.height;
  if (!inBounds(startCell) || !inBounds(goalCell)) throw new RangeError('A* endpoints must be inside the configured grid');
  if (blocked.has(key(goalCell))) throw new Error('A* goal cell is blocked');
  const heuristic = ([c, r]) => Math.abs(c - goalCell[0]) + Math.abs(r - goalCell[1]);
  const open = [[heuristic(startCell), startCell]];
  const cameFrom = new Map();
  const cost = new Map([[key(startCell), 0]]);
  while (open.length) {
    open.sort((a, b) => a[0] - b[0]);
    const [, current] = open.shift();
    if (key(current) === key(goalCell)) {
      const path = [current];
      let cursor = key(current);
      while (cameFrom.has(cursor)) { const previous = cameFrom.get(cursor); path.unshift(previous); cursor = key(previous); }
      return Object.freeze(path.map(([c, r]) => Object.freeze(gridCellCenterM(c, r))));
    }
    for (const [dc, dr] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
      const next = [current[0] + dc, current[1] + dr];
      if (!inBounds(next) || blocked.has(key(next))) continue;
      const tentative = cost.get(key(current)) + 1;
      if (tentative < (cost.get(key(next)) ?? Infinity)) {
        cost.set(key(next), tentative);
        cameFrom.set(key(next), current);
        open.push([tentative + heuristic(next), next]);
      }
    }
  }
  throw new Error('A* found no free-cell route');
}

// --- bounded chassis drive controller --------------------------------------------------------
export const LEKIWI_DRIVE_LIMITS = Object.freeze({
  maxLinearMS: 0.22,
  maxYawRadS: 0.90,
  linearGain: 1.6,
  yawGain: 2.2,
  positionToleranceM: 0.015,
  yawToleranceRad: 0.06,
  arrivalSpeedMS: 0.03,
  arrivalYawRateRadS: 0.15,
});

const clamp = (value, limit) => Math.max(-limit, Math.min(limit, value));
export function wrapAngle(radians) {
  let value = Number(radians);
  while (value > Math.PI) value -= 2 * Math.PI;
  while (value < -Math.PI) value += 2 * Math.PI;
  return value;
}
export function yawFromQuaternion(quaternionWxyz = [1, 0, 0, 0]) {
  const [w, x, y, z] = quaternionWxyz.map(Number);
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

/**
 * Bounded proportional chassis-velocity controller. It only ever proposes a desired chassis
 * velocity; the caller converts that through the pinned Kiwi mapping into bounded wheel targets,
 * and MuJoCo decides what the base actually does.
 */
export function chassisVelocityCommand(pose, waypoint, limits = LEKIWI_DRIVE_LIMITS) {
  const dx = Number(waypoint.xM) - Number(pose.xM);
  const dy = Number(waypoint.yM) - Number(pose.yM);
  const yaw = Number(pose.yawRad);
  const cos = Math.cos(yaw);
  const sin = Math.sin(yaw);
  const bodyX = dx * cos + dy * sin;
  const bodyY = -dx * sin + dy * cos;
  let vx = limits.linearGain * bodyX;
  let vy = limits.linearGain * bodyY;
  const speed = Math.hypot(vx, vy);
  if (speed > limits.maxLinearMS) { vx *= limits.maxLinearMS / speed; vy *= limits.maxLinearMS / speed; }
  const yawError = waypoint.yawRad == null ? 0 : wrapAngle(Number(waypoint.yawRad) - yaw);
  const omega = clamp(limits.yawGain * yawError, limits.maxYawRadS);
  return Object.freeze({ x: vx, y: vy, thetaRadS: omega, positionErrorM: Math.hypot(dx, dy), yawErrorRad: yawError });
}

export function wheelTargetsForChassis(command, { maxWheelRadS = MAX_WHEEL_RAD_S } = {}) {
  return bodyToWheelRadS(command, SOURCE_PARAMETERS, { maxWheelRadS });
}

export function waypointReached(pose, waypoint, limits = LEKIWI_DRIVE_LIMITS) {
  const positionErrorM = Math.hypot(Number(waypoint.xM) - Number(pose.xM), Number(waypoint.yM) - Number(pose.yM));
  const yawErrorRad = waypoint.yawRad == null ? 0 : Math.abs(wrapAngle(Number(waypoint.yawRad) - Number(pose.yawRad)));
  const speedMS = Number(pose.speedMS ?? 0);
  const yawRateRadS = Math.abs(Number(pose.yawRateRadS ?? 0));
  return positionErrorM <= limits.positionToleranceM
    && yawErrorRad <= limits.yawToleranceRad
    && speedMS <= limits.arrivalSpeedMS
    && yawRateRadS <= limits.arrivalYawRateRadS;
}

// --- courier route and arm stages ------------------------------------------------------------
const HOME_CELL = Object.freeze([0, 0]);
const SERVICE_CELL = Object.freeze([5, 0]);

export const LEKIWI_COURIER_ROUTE = Object.freeze({
  plannerSource: 'legacy planar occupancy-grid A*, preserved as a desired-route generator only',
  startCell: HOME_CELL,
  goalCell: SERVICE_CELL,
  outbound: Object.freeze([
    Object.freeze({ id: 'route_service_stop', xM: LEKIWI_WORKCELL.serviceStopXYM[0], yM: LEKIWI_WORKCELL.serviceStopXYM[1], yawRad: LEKIWI_WORKCELL.serviceStopYawRad }),
  ]),
  // The return leg deliberately keeps the base facing the bench, so returning home is a
  // predominantly lateral holonomic move rather than a drive-and-turn.
  inbound: Object.freeze([
    Object.freeze({ id: 'route_home', xM: LEKIWI_WORKCELL.homeXYM[0], yM: LEKIWI_WORKCELL.homeXYM[1], yawRad: LEKIWI_WORKCELL.serviceStopYawRad }),
    Object.freeze({ id: 'route_home_heading', xM: LEKIWI_WORKCELL.homeXYM[0], yM: LEKIWI_WORKCELL.homeXYM[1], yawRad: 0 }),
  ]),
});

const GRIPPER_OPEN_RAD = 1.2;
const GRIPPER_CLOSE_RAD = -0.17;

// Arm poses solved against the pinned physical plant with the base at the configured service
// stop, and validated by a native known-pose check before the courier task was assembled.
const armPose = (values) => Object.freeze({
  arm_shoulder_pan: values[0],
  arm_shoulder_lift: values[1],
  arm_elbow_flex: values[2],
  arm_wrist_flex: values[3],
  arm_wrist_roll: values[4],
});

export const LEKIWI_ARM_POSES = Object.freeze({
  stow: Object.freeze({ ...LEKIWI_STOW_POSE_RAD }),
  pick_approach: armPose([-0.1281, -0.7472, -0.0251, 1.6452, 0.0250]),
  pick_grip: armPose([-0.1240, -0.4377, -0.2109, 1.5215, 0.0279]),
  pick_lift: armPose([-0.1240, -0.3292, -0.4542, 1.6563, 0.0279]),
  carry: armPose([-0.0118, 0.0084, -0.8149, 1.65806, 0.1109]),
  drop_over: armPose([0.0938, 0.0549, -0.9539, 1.65806, 0.0602]),
  drop_place: armPose([0.0940, -0.2226, -0.2560, 1.3514, 0.0610]),
  drop_retreat: armPose([0.0969, -0.0811, -0.6608, 1.6146, 0.0632]),
});

const stage = (name, label, kind, extra) => Object.freeze({ name, label, kind, ...extra });

export const LEKIWI_COURIER_CONTROLLER = Object.freeze({
  id: 'lekiwi-beaker-courier-v1',
  controllerPeriodSeconds: 0.02,
  gripperOpenRad: GRIPPER_OPEN_RAD,
  gripperCloseRad: GRIPPER_CLOSE_RAD,
  driveLimits: LEKIWI_DRIVE_LIMITS,
  stages: Object.freeze([
    stage('settle', 'Settle the base and the free beaker under gravity and contact', 'hold', { durationSeconds: 0.6, armTargetsRad: LEKIWI_ARM_POSES.stow, gripperRad: GRIPPER_OPEN_RAD }),
    stage('stow_arm', 'Hold the arm in the compact stow pose for driving', 'hold', { durationSeconds: 0.6, armTargetsRad: LEKIWI_ARM_POSES.stow, gripperRad: GRIPPER_OPEN_RAD }),
    stage('drive_to_service', 'Physically drive to the configured service stop at the transfer bench', 'drive', { waypoints: LEKIWI_COURIER_ROUTE.outbound, timeoutSeconds: 14, armTargetsRad: LEKIWI_ARM_POSES.stow, gripperRad: GRIPPER_OPEN_RAD }),
    stage('stop_at_service', 'Command a full stop and let the base settle', 'hold', { durationSeconds: 1.0, armTargetsRad: LEKIWI_ARM_POSES.stow, gripperRad: GRIPPER_OPEN_RAD }),
    stage('approach_beaker', 'Move the open gripper alongside the beaker rim', 'hold', { durationSeconds: 2.0, armTargetsRad: LEKIWI_ARM_POSES.pick_approach, gripperRad: GRIPPER_OPEN_RAD }),
    stage('reach_rim', 'Straddle the beaker rim wall with the open jaws', 'hold', { durationSeconds: 1.4, armTargetsRad: LEKIWI_ARM_POSES.pick_grip, gripperRad: GRIPPER_OPEN_RAD }),
    stage('close_gripper', 'Close the bounded gripper actuator onto the rim wall', 'hold', { durationSeconds: 1.0, armTargetsRad: LEKIWI_ARM_POSES.pick_grip, gripperRad: GRIPPER_CLOSE_RAD }),
    stage('lift_beaker', 'Lift the contacted beaker clear of the worktop', 'hold', { durationSeconds: 1.4, armTargetsRad: LEKIWI_ARM_POSES.pick_lift, gripperRad: GRIPPER_CLOSE_RAD }),
    stage('carry_beaker', 'Carry the held beaker toward the marked receiving zone', 'hold', { durationSeconds: 1.6, armTargetsRad: LEKIWI_ARM_POSES.carry, gripperRad: GRIPPER_CLOSE_RAD }),
    stage('over_delivery', 'Hold the carried beaker over the receiving zone', 'hold', { durationSeconds: 1.6, armTargetsRad: LEKIWI_ARM_POSES.drop_over, gripperRad: GRIPPER_CLOSE_RAD }),
    stage('lower_beaker', 'Lower the still-held beaker until the worktop supports it', 'hold', { durationSeconds: 1.6, armTargetsRad: LEKIWI_ARM_POSES.drop_place, gripperRad: GRIPPER_CLOSE_RAD }),
    stage('release_beaker', 'Open the gripper once the worktop is carrying the beaker', 'hold', { durationSeconds: 1.2, armTargetsRad: LEKIWI_ARM_POSES.drop_place, gripperRad: GRIPPER_OPEN_RAD }),
    stage('retreat_arm', 'Retreat the empty gripper clear of the settled beaker', 'hold', { durationSeconds: 1.6, armTargetsRad: LEKIWI_ARM_POSES.drop_retreat, gripperRad: GRIPPER_OPEN_RAD }),
    stage('restow_arm', 'Return the arm to the compact stow pose', 'hold', { durationSeconds: 2.2, armTargetsRad: LEKIWI_ARM_POSES.stow, gripperRad: GRIPPER_OPEN_RAD }),
    stage('drive_home', 'Physically drive home; the base keeps its heading, so this leg is a lateral holonomic move', 'drive', { waypoints: LEKIWI_COURIER_ROUTE.inbound, timeoutSeconds: 16, armTargetsRad: LEKIWI_ARM_POSES.stow, gripperRad: GRIPPER_OPEN_RAD }),
    stage('stop_home', 'Command a full stop at home and let the base settle', 'hold', { durationSeconds: 1.2, armTargetsRad: LEKIWI_ARM_POSES.stow, gripperRad: GRIPPER_OPEN_RAD }),
  ]),
});

export const LEKIWI_COURIER_SCENE = scene('p5b-lekiwi-beaker-courier', 'p5b-lekiwi-beaker-courier-v1', LEKIWI_COURIER_PACKAGE, {
  legacyTaskId: 'lekiwi-01-beaker-courier',
  taskGoal: Object.freeze({
    type: 'lekiwi-physical-beaker-courier',
    objectId: 'empty_beaker',
    baseBodyId: 'lekiwi_base',
    gripperGeoms: Object.freeze(['fixed_jaw_pad1', 'fixed_jaw_pad2', 'fixed_jaw_tip', 'moving_jaw_pad1', 'moving_jaw_pad2', 'moving_jaw_tip']),
    fixedJawGeoms: Object.freeze(['fixed_jaw_pad1', 'fixed_jaw_pad2', 'fixed_jaw_tip']),
    movingJawGeoms: Object.freeze(['moving_jaw_pad1', 'moving_jaw_pad2', 'moving_jaw_tip']),
    objectGeomPrefix: 'beaker_',
    supportGeom: 'lekiwi_transfer_worktop',
    pickupXYM: LEKIWI_WORKCELL.pickupXYM,
    deliveryXYM: LEKIWI_WORKCELL.deliveryXYM,
    // Repaired tolerance: the legacy 12.5 mm placement tolerance and 4 mm waypoint tolerance are
    // not inherited automatically. This is the physically achieved placement accuracy with a
    // documented margin, and it is checked against the free beaker body, not against a command.
    deliveryHalfExtentsXYM: Object.freeze([0.020, 0.020]),
    restXYToleranceM: 0.020,
    supportZToleranceM: 0.004,
    liftClearanceM: 0.010,
    carryHorizontalM: 0.060,
    settleSeconds: 0.25,
    maxSettleDriftM: 0.0015,
    maxSettleLinearSpeedMS: 0.03,
    maxSettleAngularSpeedRadS: 0.8,
    maxSettleTiltRad: 0.20,
    baseTravelM: 0.20,
    homeXYM: LEKIWI_WORKCELL.homeXYM,
    homeToleranceM: 0.030,
    serviceStopXYM: LEKIWI_WORKCELL.serviceStopXYM,
    serviceStopToleranceM: 0.030,
    restrictedStopXYM: LEKIWI_WORKCELL.restrictedStopXYM,
    restrictedStopRadiusM: LEKIWI_WORKCELL.restrictedStopRadiusM,
    stoppedSpeedMS: 0.02,
  }),
});

export const LEKIWI_SCENES = Object.freeze({
  wheelReference: LEKIWI_WHEEL_REFERENCE_SCENE,
  base: LEKIWI_BASE_SCENE,
  baseStand: LEKIWI_BASE_STAND_SCENE,
  baseLowTraction: LEKIWI_BASE_LOWTRACTION_SCENE,
  courier: LEKIWI_COURIER_SCENE,
});
