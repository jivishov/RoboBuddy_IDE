import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

import { PARAMETER_EVIDENCE, requireModelPackage } from '../../src/physics/model-registry.js';
import {
  LEKIWI_ACTUATED_NAMES, LEKIWI_CANONICAL_PRESENTATION_MAP, LEKIWI_RECONCILIATION, LEKIWI_SIM_SECONDARY_SOURCE, LEKIWI_SOURCE,
  LEROBOT_SOURCE, LEGACY_TASK_SOURCE, MODEL_FRAME, SO_ARM101_SOURCE, assertReconciliationCoverage,
  reconciliationByEvidence,
} from '../../src/physics/lekiwi-source-audit.js';
import {
  LEKIWI_WHEEL_GEOMETRY, LEROBOT_DEFAULT_PARAMETERS, MAX_WHEEL_RAD_S, SOURCE_PARAMETERS,
  WHEEL_ORDER, bodyToWheelRadS, degreesPerSecondToRadians, maxBodySpeedMS,
  publicActionToBodyCommand, radiansPerSecondToRawTicks, wheelRadSToBody,
} from '../../src/physics/lekiwi-kinematics.js';
import {
  LEKIWI_BASE_LOWTRACTION_PACKAGE, LEKIWI_BASE_PACKAGE, LEKIWI_BASE_STAND_PACKAGE,
  LEKIWI_COURIER_PACKAGE, LEKIWI_MODEL_PACKAGES, LEKIWI_STOW_POSE_RAD, LEKIWI_WHEEL_REFERENCE_PACKAGE,
} from '../../src/physics/lekiwi-model-package.js';
import {
  LEKIWI_ARM_POSES, LEKIWI_COURIER_CONTROLLER, LEKIWI_COURIER_ROUTE, LEKIWI_COURIER_SCENE,
  LEKIWI_DRIVE_LIMITS, LEKIWI_OCCUPANCY_GRID, LEKIWI_SCENES, LEKIWI_WORKCELL,
  blockedCells, chassisVelocityCommand, gridCellCenterM, planRoute, waypointReached,
  wheelTargetsForChassis, wrapAngle, yawFromQuaternion,
} from '../../src/physics/lekiwi-scene.js';
import { LeKiwiCourierEvaluator, basePoseFromObservation, bodyFrameDisplacement } from '../../src/physics/lekiwi-task-evaluator.js';
import { LEKIWI_PHYSICAL_CONTROL_LIMITS, WEBMCP_LEKIWI_PHYSICAL_SCHEMA_VERSION, createLeKiwiPhysicalControlSchema } from '../../src/webmcp/lekiwi-physical-control.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const read = (relative) => readFileSync(join(ROOT, relative), 'utf8');
const sha256 = (text) => createHash('sha256').update(text).digest('hex');
const close = (actual, expected, tolerance, label) => assert.ok(Math.abs(actual - expected) <= tolerance, `${label}: ${actual} is not within ${tolerance} of ${expected}`);

// --- 1. source pins and the reconciliation audit ---------------------------------------------
assert.equal(LEKIWI_SOURCE.repository, 'SIGRobotics-UIUC/LeKiwi');
assert.equal(LEKIWI_SOURCE.revision, 'efa608d7ee5a495a4803b1d28cd0c955b4f1e033');
assert.match(LEKIWI_SOURCE.variant, /Version 1/);
assert.equal(LEROBOT_SOURCE.repository, 'huggingface/lerobot');
assert.equal(LEROBOT_SOURCE.revision, '7e241bd630a3719a56157a497ce5d08f244784f1');
assert.equal(SO_ARM101_SOURCE.revision, '8161bba264d7fa7c99ca301e91e7fb44737676ad');
assert.equal(LEGACY_TASK_SOURCE.revision, '75fe2669c0ab0b029986de424c69162071174df8');
assert.equal(LEKIWI_SIM_SECONDARY_SOURCE.status, 'not-adopted');
assert.ok(assertReconciliationCoverage() >= 25, 'reconciliation table must cover the audited parameter set');
assert.ok(reconciliationByEvidence(PARAMETER_EVIDENCE.CALIBRATION_REQUIRED).some((item) => item.parameter === 'hardware alignment'));
assert.ok(reconciliationByEvidence(PARAMETER_EVIDENCE.ESTIMATED).length >= 8, 'estimated parameters must be enumerated, not hidden');
for (const row of LEKIWI_RECONCILIATION) assert.ok(row.source && row.note, `reconciliation row ${row.parameter} needs a source and a note`);
// The LeRobot controller defaults must be classified as interface parameters, not measurements.
const radiusRow = LEKIWI_RECONCILIATION.find((row) => row.parameter === 'wheel radius');
assert.match(radiusRow.note, /interface parameter/);
const baseRadiusRow = LEKIWI_RECONCILIATION.find((row) => row.parameter === 'effective chassis radius per wheel');
assert.match(baseRadiusRow.note, /interface parameter/);
const controlRangeRow = LEKIWI_RECONCILIATION.find((row) => row.parameter === 'wheel control range');
assert.match(controlRangeRow.note, /software\/interface limit/);
assert.deepEqual([...LEKIWI_ACTUATED_NAMES], [
  'base_left_wheel', 'base_back_wheel', 'base_right_wheel',
  'arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex', 'arm_wrist_flex', 'arm_wrist_roll', 'arm_gripper',
]);
assert.match(MODEL_FRAME.urdfToModel, /model_x = urdf_y/);


// The canonical visual is a different pinned source chain from the Menagerie physical arm.
// Its sign/zero conversion is explicit provenance, never a physical-state correction.
assert.deepEqual(Object.fromEntries(Object.entries(LEKIWI_CANONICAL_PRESENTATION_MAP.joints).map(([id, m]) => [id, m.sign])), {
  arm_shoulder_pan: 1, arm_shoulder_lift: -1, arm_elbow_flex: -1, arm_wrist_flex: -1, arm_wrist_roll: -1,
});
close(LEKIWI_CANONICAL_PRESENTATION_MAP.gripper.physicalOpenRad, LEKIWI_COURIER_CONTROLLER.gripperOpenRad, 1e-12, 'presentation gripper open');
close(LEKIWI_CANONICAL_PRESENTATION_MAP.gripper.physicalClosedRad, LEKIWI_COURIER_CONTROLLER.gripperCloseRad, 1e-12, 'presentation gripper close');
assert.match(LEKIWI_CANONICAL_PRESENTATION_MAP.evidence, /presentation only/);
assert.ok(LEKIWI_RECONCILIATION.some((row) => row.parameter === 'canonical visual joint convention' && row.evidence === PARAMETER_EVIDENCE.SOURCE_DERIVED));

// --- 2. model packages, assets, bounded actuators ---------------------------------------------
assert.equal(LEKIWI_MODEL_PACKAGES.length, 5);
for (const pkg of LEKIWI_MODEL_PACKAGES) {
  assert.equal(requireModelPackage(pkg.id), pkg);
  assert.equal(sha256(read(pkg.asset)), pkg.sha256, `${pkg.asset} hash must match the registered package`);
  assert.equal(pkg.physics.timestepSeconds, 0.002);
  assert.equal(pkg.physics.integrator, 'implicitfast');
  assert.ok(pkg.limitations.length >= 5);
  assert.ok(pkg.evidence.hardwareAlignment === PARAMETER_EVIDENCE.CALIBRATION_REQUIRED);
  for (const actuator of pkg.actuators) {
    assert.ok(['position-rad', 'velocity-rad-s'].includes(actuator.command), `actuator ${actuator.id} must declare a supported command`);
    const range = actuator.controlRangeRad || actuator.controlRangeRadS;
    assert.ok(Array.isArray(range) && range.every(Number.isFinite) && range[0] < range[1], `actuator ${actuator.id} must be command-bounded`);
    if (actuator.command === 'velocity-rad-s') {
      assert.ok(Array.isArray(actuator.forceRangeNm) && actuator.forceRangeNm[0] < actuator.forceRangeNm[1], `wheel actuator ${actuator.id} must be effort-bounded`);
      close(actuator.controlRangeRadS[1], MAX_WHEEL_RAD_S, 1e-9, `${actuator.id} control range`);
    }
  }
}
// Every wheel has 12 declared passive roller joints, matching the source roller count.
for (const pkg of [LEKIWI_BASE_PACKAGE, LEKIWI_BASE_STAND_PACKAGE, LEKIWI_BASE_LOWTRACTION_PACKAGE, LEKIWI_COURIER_PACKAGE]) {
  const rollers = pkg.joints.filter((joint) => joint.id.includes('_roller_'));
  assert.equal(rollers.length, 36, `${pkg.id} must declare 3 wheels x 12 source rollers`);
  assert.ok(rollers.every((joint) => joint.passive === true));
  assert.equal(pkg.joints.filter((joint) => WHEEL_ORDER.includes(joint.id)).length, 3);
  assert.equal(pkg.bodies.find((body) => body.id === 'lekiwi_base').freeJointId, 'lekiwi_base_free');
}
assert.equal(LEKIWI_WHEEL_REFERENCE_PACKAGE.joints.filter((joint) => joint.id.includes('_roller_')).length, 12);
// The courier package carries the free payload and the nine stable actuated names.
assert.equal(LEKIWI_COURIER_PACKAGE.bodies.find((body) => body.id === 'empty_beaker').freeJointId, 'empty_beaker_free');
assert.deepEqual(LEKIWI_COURIER_PACKAGE.actuators.map((item) => item.jointId).sort(), [...LEKIWI_ACTUATED_NAMES].sort());
for (const [jointId, value] of Object.entries(LEKIWI_STOW_POSE_RAD)) {
  const joint = LEKIWI_COURIER_PACKAGE.joints.find((item) => item.id === jointId);
  assert.ok(value >= joint.rangeRad[0] && value <= joint.rangeRad[1], `stow ${jointId} must be inside the source joint range`);
}
// The wheel joints declare the LeRobot-positive axis and record the anti-parallel source axis.
for (const item of LEKIWI_WHEEL_GEOMETRY) {
  const joint = LEKIWI_BASE_PACKAGE.joints.find((entry) => entry.id === item.id);
  // The compiled hinge axis is the wheel body's local z; the audited model-frame axis is separate.
  assert.deepEqual(joint.axis, [0, 0, 1]);
  assert.deepEqual(joint.modelFrameAxis, [...item.modelAxis]);
  // The source axis is anti-parallel to the LeRobot-positive axis, expressed in the URDF frame:
  // model = R(+90 deg about Z) applied to the negated URDF axis, i.e. (ux, uy) -> (-uy, ux).
  const negated = joint.sourceUrdfAxis.map((value) => -value);
  const expectedModelAxis = [negated[1], -negated[0], negated[2]];
  for (let index = 0; index < 3; index += 1) close(joint.modelFrameAxis[index], expectedModelAxis[index], 1e-6, `${item.id} source axis must be anti-parallel through the declared frame map`);
}

// --- 3. body-to-wheel mapping: signs, units, saturation ---------------------------------------
// The source-derived drive angles must equal the pinned LeRobot mapping angles exactly.
for (const [index, item] of LEKIWI_WHEEL_GEOMETRY.entries()) {
  const expected = [150, -90, 30][index];
  close(item.driveAngleDeg, expected, 1e-9, `${item.id} drive angle`);
  close(item.driveDirection[0], Math.cos(expected * Math.PI / 180), 1e-6, `${item.id} drive direction x`);
  close(item.driveDirection[1], Math.sin(expected * Math.PI / 180), 1e-6, `${item.id} drive direction y`);
  assert.ok(item.momentArmM > 0, `${item.id} moment arm must be positive under the LeRobot theta convention`);
}
const forward = bodyToWheelRadS({ x: 0.15 });
assert.ok(forward.targetsRadS.base_left_wheel < 0 && forward.targetsRadS.base_right_wheel > 0);
close(forward.targetsRadS.base_back_wheel, 0, 1e-9, 'forward back-wheel');
close(forward.targetsRadS.base_left_wheel, -forward.targetsRadS.base_right_wheel, 2e-3, 'forward symmetry');
const reverse = bodyToWheelRadS({ x: -0.15 });
for (const id of WHEEL_ORDER) close(reverse.targetsRadS[id], -forward.targetsRadS[id], 1e-9, `${id} reverse antisymmetry`);
const left = bodyToWheelRadS({ y: 0.15 });
assert.ok(left.targetsRadS.base_back_wheel < 0, 'lateral-left must drive the back wheel negative');
assert.ok(left.targetsRadS.base_left_wheel > 0 && left.targetsRadS.base_right_wheel > 0);
const right = bodyToWheelRadS({ y: -0.15 });
for (const id of WHEEL_ORDER) close(right.targetsRadS[id], -left.targetsRadS[id], 1e-9, `${id} lateral antisymmetry`);
const yawPositive = bodyToWheelRadS({ thetaRadS: 0.8 });
for (const id of WHEEL_ORDER) assert.ok(yawPositive.targetsRadS[id] > 0, `${id} must turn positive for a positive yaw request`);
const yawNegative = bodyToWheelRadS({ thetaRadS: -0.8 });
for (const id of WHEEL_ORDER) close(yawNegative.targetsRadS[id], -yawPositive.targetsRadS[id], 1e-9, `${id} yaw antisymmetry`);
const zero = bodyToWheelRadS({});
for (const id of WHEEL_ORDER) assert.equal(zero.targetsRadS[id], 0);
assert.equal(zero.saturated, false);
// Mixed motion superposes, and saturation scales every wheel proportionally.
const mixed = bodyToWheelRadS({ x: 0.12, y: 0.08, thetaRadS: 0.4 });
for (const id of WHEEL_ORDER) {
  const expected = bodyToWheelRadS({ x: 0.12 }).targetsRadS[id] + bodyToWheelRadS({ y: 0.08 }).targetsRadS[id] + bodyToWheelRadS({ thetaRadS: 0.4 }).targetsRadS[id];
  close(mixed.targetsRadS[id], expected, 1e-9, `${id} superposition`);
}
const saturating = bodyToWheelRadS({ x: 4 });
assert.equal(saturating.saturated, true);
assert.ok(saturating.scale < 1);
close(Math.max(...WHEEL_ORDER.map((id) => Math.abs(saturating.targetsRadS[id]))), MAX_WHEEL_RAD_S, 1e-9, 'saturated peak');
for (const id of WHEEL_ORDER) {
  close(saturating.targetsRadS[id] / saturating.scale, saturating.unclampedRadS[WHEEL_ORDER.indexOf(id)], 1e-9, `${id} proportional scaling`);
}
// Round trip through the inverse mapping.
for (const command of [{ x: 0.15 }, { y: -0.12 }, { thetaRadS: 0.5 }, { x: 0.08, y: 0.05, thetaRadS: -0.3 }]) {
  const body = wheelRadSToBody(bodyToWheelRadS(command).targetsRadS);
  close(body.x, command.x ?? 0, 1e-9, 'round-trip x');
  close(body.y, command.y ?? 0, 1e-9, 'round-trip y');
  close(body.thetaRadS, command.thetaRadS ?? 0, 1e-9, 'round-trip theta');
}
// Public boundary keeps LeRobot degrees per second and converts internally.
close(degreesPerSecondToRadians(180), Math.PI, 1e-12, 'deg/s conversion');
const publicCommand = publicActionToBodyCommand({ 'x.vel': 0.1, 'y.vel': -0.05, 'theta.vel': 45 });
close(publicCommand.thetaRadS, Math.PI / 4, 1e-12, 'public theta conversion');
close(radiansPerSecondToRawTicks(MAX_WHEEL_RAD_S), 3000, 1, 'LeRobot raw tick ceiling');
// The pinned LeRobot defaults and the source-derived parameters must agree in sign everywhere
// and stay within a documented magnitude band.
for (const command of [{ x: 0.15 }, { x: -0.15 }, { y: 0.15 }, { y: -0.15 }, { thetaRadS: 0.6 }, { thetaRadS: -0.6 }]) {
  const source = bodyToWheelRadS(command, SOURCE_PARAMETERS);
  const lerobot = bodyToWheelRadS(command, LEROBOT_DEFAULT_PARAMETERS);
  for (const id of WHEEL_ORDER) {
    const signOf = (value) => Math.sign(Number(value.toFixed(9))) + 0;
    assert.equal(signOf(source.targetsRadS[id]), signOf(lerobot.targetsRadS[id]), `${id} sign must agree between parameter sets`);
    const magnitude = Math.abs(source.targetsRadS[id]);
    if (magnitude > 1e-6) {
      const ratio = magnitude / Math.abs(lerobot.targetsRadS[id]);
      assert.ok(ratio > 0.85 && ratio < 1.15, `${id} magnitude ratio ${ratio} must stay inside the documented band`);
    }
  }
}
assert.ok(maxBodySpeedMS([1, 0]) > 0.25 && maxBodySpeedMS([1, 0]) < 0.30, 'forward speed ceiling');

// --- 4. desired-route generator ----------------------------------------------------------------
assert.equal(LEKIWI_OCCUPANCY_GRID.width, 15);
assert.equal(LEKIWI_OCCUPANCY_GRID.resolutionM, 0.05);
assert.match(LEKIWI_OCCUPANCY_GRID.provenance, /repaired from the legacy blocked-cell list/);
const blocked = new Set(blockedCells());
assert.ok(blocked.size > 10, 'the repaired grid must block the bench and the restricted stop');
const benchCell = (() => {
  for (let row = 0; row < 15; row += 1) for (let col = 0; col < 15; col += 1) {
    const [x, y] = gridCellCenterM(col, row);
    if (Math.abs(x - LEKIWI_WORKCELL.worktopCenterXYM[0]) < 0.03 && Math.abs(y - LEKIWI_WORKCELL.worktopCenterXYM[1]) < 0.03) return `${col},${row}`;
  }
  return null;
})();
assert.ok(blocked.has(benchCell), 'the transfer bench footprint must be blocked');
const route = planRoute(LEKIWI_COURIER_ROUTE.startCell, LEKIWI_COURIER_ROUTE.goalCell);
assert.ok(route.length >= 2, 'A* must produce a waypoint list');
for (const [x, y] of route) {
  assert.ok(Math.hypot(x - LEKIWI_WORKCELL.restrictedStopXYM[0], y - LEKIWI_WORKCELL.restrictedStopXYM[1]) > LEKIWI_WORKCELL.restrictedStopRadiusM);
}
assert.throws(() => planRoute([0, 0], [3, 6]), /blocked|no free-cell route/);

// --- 5. bounded drive controller ----------------------------------------------------------------
const poseAt = (xM, yM, yawRad, extra = {}) => ({ xM, yM, yawRad, speedMS: 0, yawRateRadS: 0, ...extra });
const target = { xM: 1, yM: 0, yawRad: 0 };
const far = chassisVelocityCommand(poseAt(0, 0, 0), target);
close(Math.hypot(far.x, far.y), LEKIWI_DRIVE_LIMITS.maxLinearMS, 1e-9, 'linear command must saturate at the declared limit');
assert.ok(Math.abs(far.thetaRadS) <= LEKIWI_DRIVE_LIMITS.maxYawRadS);
// A target directly to the left of a robot facing +x is a pure lateral request.
const lateral = chassisVelocityCommand(poseAt(0, 0, 0), { xM: 0, yM: 1, yawRad: 0 });
close(lateral.x, 0, 1e-9, 'pure lateral must not request forward motion');
assert.ok(lateral.y > 0);
// Rotating the robot rotates the body-frame request, not the world target.
const rotated = chassisVelocityCommand(poseAt(0, 0, Math.PI / 2), target);
close(rotated.x, 0, 1e-9, 'body-frame projection');
assert.ok(rotated.y < 0);
const yawOnly = chassisVelocityCommand(poseAt(0, 0, 0), { xM: 0, yM: 0, yawRad: 1.0 });
close(Math.hypot(yawOnly.x, yawOnly.y), 0, 1e-9, 'yaw-only request');
assert.ok(yawOnly.thetaRadS > 0);
// Arrival is decided from the observed pose, including that the base actually stopped.
assert.equal(waypointReached(poseAt(0, 0, 0), { xM: 0.005, yM: 0, yawRad: 0 }), true);
assert.equal(waypointReached(poseAt(0, 0, 0, { speedMS: 0.2 }), { xM: 0.005, yM: 0, yawRad: 0 }), false, 'a moving base has not arrived');
assert.equal(waypointReached(poseAt(0, 0, 0, { yawRateRadS: 0.9 }), { xM: 0.005, yM: 0, yawRad: 0 }), false, 'a spinning base has not arrived');
assert.equal(waypointReached(poseAt(0, 0, 0), { xM: 0.2, yM: 0, yawRad: 0 }), false);
assert.equal(waypointReached(poseAt(0, 0, 0), { xM: 0.005, yM: 0, yawRad: 0.5 }), false, 'yaw error blocks arrival');
close(wrapAngle(3 * Math.PI), Math.PI, 1e-9, 'angle wrap');
close(yawFromQuaternion([Math.cos(0.3), 0, 0, Math.sin(0.3)]), 0.6, 1e-9, 'yaw from quaternion');
// Wheel targets from a chassis request stay bounded.
const driveWheels = wheelTargetsForChassis({ x: 5, y: 5, thetaRadS: 5 });
for (const id of WHEEL_ORDER) assert.ok(Math.abs(driveWheels.targetsRadS[id]) <= MAX_WHEEL_RAD_S + 1e-9);

// --- 6. scenes and controller stages -----------------------------------------------------------
assert.equal(Object.keys(LEKIWI_SCENES).length, 5);
for (const scene of Object.values(LEKIWI_SCENES)) {
  const pkg = requireModelPackage(scene.modelPackage);
  assert.equal(scene.robotId, pkg.robotId);
  assert.deepEqual([...scene.controllers], [...pkg.controllers]);
  assert.deepEqual(scene.fixtures.map((item) => item.id), pkg.sceneConstraints.fixtures);
  assert.deepEqual(scene.objects.map((item) => item.id), pkg.sceneConstraints.objects);
}
assert.equal(LEKIWI_COURIER_SCENE.legacyTaskId, 'lekiwi-01-beaker-courier');
assert.equal(LEKIWI_COURIER_CONTROLLER.stages.filter((stage) => stage.kind === 'drive').length, 2);
assert.ok(LEKIWI_COURIER_CONTROLLER.stages.some((stage) => stage.name === 'drive_home' && /lateral/.test(stage.label)));
for (const stage of LEKIWI_COURIER_CONTROLLER.stages) {
  assert.ok(stage.armTargetsRad, `${stage.name} must declare arm targets`);
  assert.ok(Number.isFinite(stage.gripperRad));
  if (stage.kind === 'drive') assert.ok(stage.waypoints.length >= 1 && stage.timeoutSeconds > 0);
  else assert.ok(stage.durationSeconds > 0);
}
for (const [name, pose] of Object.entries(LEKIWI_ARM_POSES)) {
  for (const [jointId, value] of Object.entries(pose)) {
    const joint = LEKIWI_COURIER_PACKAGE.joints.find((item) => item.id === jointId);
    assert.ok(joint, `arm pose ${name} references ${jointId}`);
    assert.ok(value >= joint.rangeRad[0] && value <= joint.rangeRad[1], `arm pose ${name}.${jointId} is outside the source joint range`);
  }
}
// The repaired delivery tolerance is not the inherited legacy 4 mm waypoint tolerance.
assert.ok(LEKIWI_COURIER_SCENE.taskGoal.restXYToleranceM > 0.004);
assert.ok(LEKIWI_DRIVE_LIMITS.positionToleranceM > 0.004);

// --- 7. evaluator: causal success and adverse cases ---------------------------------------------
const GOAL = LEKIWI_COURIER_SCENE.taskGoal;
const PICK = GOAL.pickupXYM;
const DROP = GOAL.deliveryXYM;
const REST_Z = 0.211;
function frame({
  t, base = [0, 0], baseVel = [0, 0], yawRate = 0, beaker = [PICK[0], PICK[1], REST_Z],
  fixedJaw = false, movingJaw = false, support = true, beakerVel = [0, 0, 0],
}) {
  const contacts = [];
  if (support) contacts.push({ geom1Name: 'beaker_bottom', geom2Name: GOAL.supportGeom });
  if (fixedJaw) contacts.push({ geom1Name: 'beaker_rim_2', geom2Name: 'fixed_jaw_pad1' });
  if (movingJaw) contacts.push({ geom1Name: 'moving_jaw_pad1', geom2Name: 'beaker_rim_2' });
  return {
    simulationTimeSeconds: t,
    joints: {},
    bodies: {
      lekiwi_base: { positionM: [base[0], base[1], 0.0508], quaternionWxyz: [1, 0, 0, 0], linearVelocityMS: [baseVel[0], baseVel[1], 0], angularVelocityRadS: [0, 0, yawRate] },
      empty_beaker: { positionM: [...beaker], quaternionWxyz: [1, 0, 0, 0], linearVelocityMS: [...beakerVel], angularVelocityRadS: [0, 0, 0] },
    },
    contactCount: contacts.length,
    contactsReadable: true,
    contacts,
  };
}
function nominalRun(evaluator, { skipRelease = false, dropPayload = false, enterRestricted = false, skipHome = false } = {}) {
  let t = 0;
  const step = 0.05;
  const push = (options) => { evaluator.observe(frame({ t, ...options })); t += step; };
  push({});
  // drive to the service stop
  for (let i = 1; i <= 10; i += 1) {
    const fraction = i / 10;
    const x = GOAL.serviceStopXYM[0] * fraction;
    const y = GOAL.serviceStopXYM[1] * fraction;
    push({ base: [x, y], baseVel: [0.2, 0] });
  }
  if (enterRestricted) push({ base: [...GOAL.restrictedStopXYM], baseVel: [0.1, 0] });
  for (let i = 0; i < 4; i += 1) push({ base: [...GOAL.serviceStopXYM] });
  // grasp, lift, carry
  for (let i = 0; i < 3; i += 1) push({ base: [...GOAL.serviceStopXYM], fixedJaw: true, movingJaw: true });
  for (let i = 1; i <= 4; i += 1) push({ base: [...GOAL.serviceStopXYM], fixedJaw: true, movingJaw: true, support: false, beaker: [PICK[0], PICK[1], REST_Z + 0.02 * i] });
  for (let i = 1; i <= 10; i += 1) {
    const fraction = i / 10;
    push({
      base: [...GOAL.serviceStopXYM], fixedJaw: true, movingJaw: true, support: false,
      beaker: [PICK[0] + (DROP[0] - PICK[0]) * fraction - 0.06 * (1 - fraction), PICK[1] + (DROP[1] - PICK[1]) * fraction, REST_Z + 0.08],
    });
  }
  if (dropPayload) {
    for (let i = 0; i < 6; i += 1) push({ base: [...GOAL.serviceStopXYM], support: false, beaker: [DROP[0], DROP[1], REST_Z - 0.15] });
    return;
  }
  // lower until the worktop supports it while still held
  for (let i = 0; i < 4; i += 1) push({ base: [...GOAL.serviceStopXYM], fixedJaw: true, movingJaw: true, support: true, beaker: [DROP[0], DROP[1], REST_Z] });
  if (!skipRelease) {
    for (let i = 0; i < 12; i += 1) push({ base: [...GOAL.serviceStopXYM], support: true, beaker: [DROP[0], DROP[1], REST_Z] });
  } else {
    for (let i = 0; i < 12; i += 1) push({ base: [...GOAL.serviceStopXYM], fixedJaw: true, movingJaw: true, support: true, beaker: [DROP[0], DROP[1], REST_Z] });
  }
  if (skipHome) return;
  const held = skipRelease ? { fixedJaw: true, movingJaw: true } : {};
  for (let i = 10; i >= 0; i -= 1) {
    const fraction = i / 10;
    push({ base: [GOAL.serviceStopXYM[0] * fraction, GOAL.serviceStopXYM[1] * fraction], baseVel: i > 0 ? [0.2, 0] : [0, 0], support: true, beaker: [DROP[0], DROP[1], REST_Z], ...held });
  }
  for (let i = 0; i < 4; i += 1) push({ base: [0, 0], support: true, beaker: [DROP[0], DROP[1], REST_Z], ...held });
}
{
  const evaluator = new LeKiwiCourierEvaluator();
  nominalRun(evaluator);
  const snapshot = evaluator.snapshot();
  assert.equal(snapshot.success, true, `nominal evaluator run must succeed: ${JSON.stringify(snapshot)}`);
  assert.equal(snapshot.causalOrder, true);
  assert.ok(snapshot.serviceStopTimeSeconds <= snapshot.graspTimeSeconds);
  assert.ok(snapshot.supportWhileHeldTimeSeconds < snapshot.releaseTimeSeconds);
  assert.ok(snapshot.releaseTimeSeconds < snapshot.settleTimeSeconds);
  assert.ok(snapshot.settleTimeSeconds <= snapshot.homeTimeSeconds);
  assert.ok(snapshot.basePathLengthM >= GOAL.baseTravelM);
  assert.match(snapshot.evidence, /No weld, parenting, attachment flag, snap, teleport, upright lock, timer, command echo, or program-reported success/);
}
for (const [label, options] of [
  ['no release', { skipRelease: true }],
  ['dropped payload', { dropPayload: true }],
  ['restricted stop visited', { enterRestricted: true }],
  ['never returned home', { skipHome: true }],
]) {
  const evaluator = new LeKiwiCourierEvaluator();
  nominalRun(evaluator, options);
  assert.equal(evaluator.snapshot().success, false, `${label} must not succeed`);
}
{
  // A release credited then contradicted by jaw re-contact must be invalidated.
  const evaluator = new LeKiwiCourierEvaluator();
  nominalRun(evaluator);
  assert.equal(evaluator.snapshot().success, true);
  evaluator.observe(frame({ t: 999, base: [0, 0], fixedJaw: true, movingJaw: true, support: true, beaker: [DROP[0], DROP[1], REST_Z] }));
  const after = evaluator.snapshot();
  assert.equal(after.releaseSeen, false);
  assert.equal(after.settled, false);
  assert.equal(after.success, false);
  assert.ok(after.releaseInvalidations >= 1);
}
{
  // Commands alone can never satisfy the evaluator: an observation stream with no contact and no
  // base motion stays a failure no matter how many times it is fed.
  const evaluator = new LeKiwiCourierEvaluator();
  for (let i = 0; i < 500; i += 1) evaluator.observe(frame({ t: i * 0.05 }));
  assert.equal(evaluator.snapshot().success, false);
  assert.equal(evaluator.snapshot().graspSeen, false);
}
{
  const pose = basePoseFromObservation(frame({ t: 0, base: [0.3, -0.2], baseVel: [0.1, 0.1], yawRate: 0.2 }));
  close(pose.xM, 0.3, 1e-9, 'observed base x');
  close(pose.speedMS, Math.hypot(0.1, 0.1), 1e-9, 'observed base speed');
  const displacement = bodyFrameDisplacement({ xM: 0, yM: 0, yawRad: Math.PI / 2 }, { xM: 0, yM: 1, yawRad: Math.PI / 2 });
  close(displacement.forwardM, 1, 1e-9, 'body-frame forward');
  close(displacement.leftM, 0, 1e-9, 'body-frame lateral');
}

// --- 8. WebMCP physical schema -------------------------------------------------------------------
const schema = createLeKiwiPhysicalControlSchema();
assert.equal(WEBMCP_LEKIWI_PHYSICAL_SCHEMA_VERSION, 'robobuddy.lekiwi.physical.v1');
const commands = schema.oneOf.map((branch) => branch.properties.command.const);
assert.deepEqual(commands, ['set_chassis_velocity', 'set_arm_targets', 'stop', 'reset']);
const chassisBranch = schema.oneOf[0].properties.chassis_velocity;
assert.ok(chassisBranch.properties['x.vel'].maximum <= LEKIWI_PHYSICAL_CONTROL_LIMITS.maxForwardMS + 1e-9);
assert.equal(chassisBranch.additionalProperties, false);
assert.match(chassisBranch.properties['theta.vel'].description, /deg\/s/);
for (const branch of schema.oneOf) assert.equal(branch.additionalProperties, false);
assert.ok(LEKIWI_PHYSICAL_CONTROL_LIMITS.maxCommandSteps <= 8000);
assert.ok(LEKIWI_PHYSICAL_CONTROL_LIMITS.maxAdvanceSeconds <= 2);

// --- 9. browser/native controller conformance -----------------------------------------------------
const nativeSource = read('native/lekiwi_reference.py');
const jsonMatch = nativeSource.match(/SHARED_CONTROLLER_JSON = r"""\n([\s\S]*?)"""\nSHARED = json\.loads/);
assert.ok(jsonMatch, 'the native runner must expose its shared controller contract as parseable JSON');
const nativeShared = JSON.parse(jsonMatch[1]);
assert.deepEqual(nativeShared.wheelOrder, [...WHEEL_ORDER]);
close(nativeShared.wheelRadiusM, SOURCE_PARAMETERS.wheelRadiusM, 1e-12, 'native wheel radius');
close(nativeShared.maxWheelRadS, MAX_WHEEL_RAD_S, 1e-12, 'native wheel speed limit');
nativeShared.driveDirections.forEach((direction, index) => {
  close(direction[0], SOURCE_PARAMETERS.rows[index].driveDirection[0], 1e-9, 'native drive direction x');
  close(direction[1], SOURCE_PARAMETERS.rows[index].driveDirection[1], 1e-9, 'native drive direction y');
  close(nativeShared.momentArmsM[index], SOURCE_PARAMETERS.rows[index].momentArmM, 1e-9, 'native moment arm');
});
for (const [key, value] of Object.entries({
  maxLinearMS: 'maxLinearMS', maxYawRadS: 'maxYawRadS', linearGain: 'linearGain', yawGain: 'yawGain',
  positionToleranceM: 'positionToleranceM', yawToleranceRad: 'yawToleranceRad',
  arrivalSpeedMS: 'arrivalSpeedMS', arrivalYawRateRadS: 'arrivalYawRateRadS',
})) close(nativeShared.driveLimits[key], LEKIWI_DRIVE_LIMITS[value], 1e-12, `native drive limit ${key}`);
close(nativeShared.gripperOpenRad, LEKIWI_COURIER_CONTROLLER.gripperOpenRad, 1e-12, 'native gripper open');
close(nativeShared.gripperCloseRad, LEKIWI_COURIER_CONTROLLER.gripperCloseRad, 1e-12, 'native gripper close');
close(nativeShared.controllerPeriodSeconds, LEKIWI_COURIER_CONTROLLER.controllerPeriodSeconds, 1e-12, 'native controller period');
for (const [jointId, value] of Object.entries(LEKIWI_STOW_POSE_RAD)) close(nativeShared.stowPoseRad[jointId], value, 1e-12, `native stow ${jointId}`);
const ARM_ORDER = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex', 'arm_wrist_flex', 'arm_wrist_roll'];
for (const [name, values] of Object.entries(nativeShared.armPosesRad)) {
  ARM_ORDER.forEach((jointId, index) => close(values[index], LEKIWI_ARM_POSES[name][jointId], 1e-9, `native arm pose ${name}.${jointId}`));
}
for (const [key, value] of Object.entries(nativeShared.taskGoal)) {
  const actual = LEKIWI_COURIER_SCENE.taskGoal[key];
  if (Array.isArray(value)) value.forEach((item, index) => close(item, actual[index], 1e-12, `native task goal ${key}[${index}]`));
  else close(value, actual, 1e-12, `native task goal ${key}`);
}
close(nativeShared.workcell.serviceStopXYM[0], LEKIWI_WORKCELL.serviceStopXYM[0], 1e-12, 'native service stop x');
close(nativeShared.workcell.serviceStopXYM[1], LEKIWI_WORKCELL.serviceStopXYM[1], 1e-12, 'native service stop y');
close(nativeShared.workcell.deliveryXYM[0], LEKIWI_WORKCELL.deliveryXYM[0], 1e-12, 'native delivery x');
close(nativeShared.lerobotDefaults.wheelRadiusM, LEROBOT_DEFAULT_PARAMETERS.wheelRadiusM, 1e-12, 'native LeRobot default radius');
close(nativeShared.lerobotDefaults.baseRadiusM, LEROBOT_DEFAULT_PARAMETERS.rows[0].momentArmM, 1e-12, 'native LeRobot default base radius');

// --- 10. forbidden success paths ------------------------------------------------------------------
const LEKIWI_SOURCES = [
  'src/physics/lekiwi-model-package.js', 'src/physics/lekiwi-scene.js', 'src/physics/lekiwi-kinematics.js',
  'src/physics/lekiwi-task-evaluator.js', 'src/physics/lekiwi-physical-simulator.js',
  'src/physics/lekiwi-mujoco-worker.js', 'src/webmcp/lekiwi-physical-control.js',
];
const FORBIDDEN = [
  /\bweld\s*=/i, /attachToGripper/i, /\bsnapTo\b/i, /teleport\s*\(/i,
  /qpos\s*\[[^\]]*\]\s*\+=/, /qvel\s*\[[^\]]*\]\s*=(?!=)/,
  /basePose\.(x|y|yaw)\s*\+=/, /position\.(x|y|z)\s*\+=/,
];
for (const relative of LEKIWI_SOURCES) {
  const text = read(relative);
  for (const pattern of FORBIDDEN) assert.ok(!pattern.test(text), `${relative} must not contain ${pattern}`);
}
// Only the worker may assign qpos at all, and only inside the declared setup/reset path.
const worker = read('src/physics/lekiwi-mujoco-worker.js');
const qposAssignments = worker.match(/data\.qpos\[[^\]]+\]\s*=/g) || [];
assert.equal(qposAssignments.length, 1, 'the worker may assign qpos only in applyDeclaredInitialState');
assert.ok(worker.indexOf('function applyDeclaredInitialState') < worker.indexOf(qposAssignments[0]));
assert.ok(!/data\.qvel\[[^\]]+\]\s*=(?!=)/.test(worker), 'the worker must never assign qvel');
assert.match(worker, /LeKiwi models must declare no equality constraints/);
// The MJCF assets themselves must not contain any constraint, weld or motor without limits.
for (const pkg of LEKIWI_MODEL_PACKAGES) {
  const xml = read(pkg.asset);
  assert.ok(!/<equality/.test(xml), `${pkg.asset} must not declare equality constraints`);
  assert.ok(!/<weld/.test(xml), `${pkg.asset} must not declare a weld`);
  assert.ok(!/<connect/.test(xml), `${pkg.asset} must not declare a connect constraint`);
  const velocityActuators = xml.match(/<velocity class="drive_wheel" name="[^"]+"/g) || [];
  assert.equal(velocityActuators.length, xml.includes('robobuddy_lekiwi_wheel_reference') ? 1 : 3, `${pkg.asset} wheel actuator count`);
  // The bounded velocity servo and its torque limit are declared once in the drive_wheel class.
  assert.match(xml, /<velocity kv="2\.2" ctrlrange="-4\.60061 4\.60061" forcerange="-2\.94 2\.94"\/>/);
}
// The physical simulator must not integrate the base or write back to physics.
const simulator = read('src/physics/lekiwi-physical-simulator.js');
assert.match(simulator, /rendererIntegratesBase: false/);
assert.match(simulator, /baseTransformSource: 'observed MuJoCo lekiwi_base free-body pose'/);
assert.ok(!/requestAnimationFrame/.test(simulator), 'the LeKiwi simulator must not own an animation clock');
assert.match(simulator, /Legacy LeKiwi \.pos\/kinematic base replay is disabled/);

// --- 11. no silent legacy fallback -----------------------------------------------------------------
const host = read('src/simulator-host.js');
assert.match(host, /physical && profileId === 'lekiwi'/);
const hostPhysicalBranch = host.slice(host.indexOf('const physical ='), host.indexOf('backend.setControllerPreemptHandler'));
assert.ok(!/catch[\s\S]*sourceFactory/.test(hostPhysicalBranch), 'a failed physical backend must not fall back to the source plant');
const controls = read('src/webmcp/robot-controls.js');
assert.match(controls, /profileId === 'lekiwi' && context\.simulationMode === 'physical_mujoco'/);

// --- 12. the WebMCP tool's host surface actually exists ------------------------------------------
// The physical control tool reaches the plant through app.sim (SimulatorHost), so every method it
// calls must be declared there. A missing delegation is invisible until a browser runs the tool.
const webmcpSource = read('src/webmcp/lekiwi-physical-control.js');
const hostMethods = new Set([...host.matchAll(/^\s{2}(?:async\s+)?([A-Za-z][A-Za-z0-9]*)\s*\(/gm)].map((match) => match[1]));
const hostCalls = [...new Set([...webmcpSource.matchAll(/facade\.app\.sim\.([A-Za-z0-9_]+)/g)].map((match) => match[1]))];
assert.ok(hostCalls.includes('applyChassisVelocity'), 'the physical tool must drive the chassis through the host');
for (const name of hostCalls) assert.ok(hostMethods.has(name), `SimulatorHost is missing ${name}, which the LeKiwi WebMCP tool calls`);
// Those physical paths must fail loudly rather than returning undefined, which a caller would
// otherwise treat as an accepted command.
const hostMethodSource = (name) => {
  const start = host.indexOf(`\n  ${name}(...args) {`);
  assert.ok(start > 0, `cannot locate SimulatorHost.${name}`);
  return host.slice(start, host.indexOf('\n  }', start) + 4);
};
for (const [name, pattern] of [['applyChassisVelocity', /no physical chassis-velocity path/], ['applyArmTargets', /no physical arm-target path/]]) {
  const body = hostMethodSource(name);
  assert.match(body, pattern, `SimulatorHost.${name} must reject an unsupported backend explicitly`);
  const method = new Function(`return ({ ${body.slice(3)} }).${name};`)();
  const calls = [];
  assert.deepEqual(method.call({ backend: { [name]: (...args) => { calls.push(args); return { status: 'accepted' }; } } }, { probe: 1 }), { status: 'accepted' });
  assert.deepEqual(calls, [[{ probe: 1 }]]);
  assert.throws(() => method.call({ backend: {} }, {}), pattern);
}


// --- 13. capability labelling follows the selected workspace ------------------------------------
// LeKiwi is the one profile with both a physical and a legacy workspace, so a legacy run must not
// carry the physical workspace's browser-mujoco/numerically-verified claim.
const { physicsCapabilityFor, capabilityLabel } = await import('../../src/physics/capabilities.js');
const lekiwiPhysicalCapability = physicsCapabilityFor('lekiwi');
assert.equal(lekiwiPhysicalCapability.backend, 'browser-mujoco');
assert.equal(lekiwiPhysicalCapability.evidence, 'numerically-verified');
assert.notEqual(lekiwiPhysicalCapability.evidence, 'hardware-compared');
const lekiwiLegacyCapability = physicsCapabilityFor('lekiwi', { physical: false });
assert.equal(lekiwiLegacyCapability.backend, 'legacy');
assert.equal(lekiwiLegacyCapability.evidence, 'model-derived');
assert.notEqual(capabilityLabel(lekiwiLegacyCapability), capabilityLabel(lekiwiPhysicalCapability));
// Profiles without a legacy workspace are unaffected by the option.
for (const profileId of ['so101', 'openarm']) {
  assert.equal(physicsCapabilityFor(profileId, { physical: false }).backend, physicsCapabilityFor(profileId).backend);
}
// The physical chip text must name LeKiwi rather than falling through to another robot's label.
const uiStatus = read('src/physics/ui-status.js');
assert.match(uiStatus, /lekiwi: 'LEKIWI V1 PHYSICAL WORKSPACE/);
assert.ok(!/profileId === 'openarm'\s*\n?\s*\? 'OPENARM V2 PHYSICAL WORKSPACE/.test(uiStatus), 'physical chip text must be keyed per profile');
assert.match(read('src/app-v2.js'), /applyPhysicsPreviewStatus\(id, \{ physical: this\.isPhysicalWorkspace\(\) \}\)/);

console.log('LeKiwi Phase 5B core contracts: OK');
console.log('  reconciliation rows:', LEKIWI_RECONCILIATION.length);
console.log('  registered model packages:', LEKIWI_MODEL_PACKAGES.map((pkg) => pkg.id).join(', '));
console.log('  source drive angles (deg):', LEKIWI_WHEEL_GEOMETRY.map((item) => item.driveAngleDeg).join(', '));
console.log('  source moment arms (m):', LEKIWI_WHEEL_GEOMETRY.map((item) => item.momentArmM).join(', '));
console.log('  A* desired route waypoints:', route.length);
