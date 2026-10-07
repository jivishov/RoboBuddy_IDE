import assert from 'node:assert/strict';
import {
  So101BlockTransferEvaluator,
  hasSo101BlockGripperContact,
  hasSo101BlockTargetSupportContact,
} from '../../src/physics/so101-task-evaluator.js';

const SOURCE = [0.39416, -0.00169, 0.234];
const TARGET = [0.358, -0.156, 0.234];
const gripperContact = [{ geom1Name: 'benchmark_block_geom', geom2Name: 'fixed_jaw_box1' }];
const targetSupportContact = [{ geom1Name: 'benchmark_block_geom', geom2Name: 'benchmark_target_support' }];

function observation(time, position, contacts = []) {
  return {
    simulationTimeSeconds: time,
    bodies: { benchmark_block: { positionM: [...position] } },
    contacts: structuredClone(contacts),
  };
}

assert.equal(hasSo101BlockGripperContact(observation(0, SOURCE, gripperContact)), true);
assert.equal(hasSo101BlockTargetSupportContact(observation(0, TARGET, targetSupportContact)), true);

const evaluator = new So101BlockTransferEvaluator();
evaluator.observe(observation(0.0, SOURCE));
evaluator.observe(observation(1.3, SOURCE, gripperContact));
evaluator.observe(observation(2.1, [SOURCE[0], SOURCE[1], 0.31], gripperContact));
evaluator.observe(observation(3.0, [TARGET[0], TARGET[1], 0.31], gripperContact));
evaluator.observe(observation(4.3, [TARGET[0], TARGET[1], 0.245]));
let state = evaluator.observe(observation(4.7, TARGET, targetSupportContact));
assert.equal(state.settleSeen, false, 'one support-contact sample cannot establish final rest');
state = evaluator.observe(observation(5.1, TARGET, targetSupportContact));
assert.equal(state.contactSeen, true);
assert.equal(state.liftSeen, true);
assert.equal(state.carrySeen, true);
assert.equal(state.releaseSeen, true);
assert.equal(state.settleSeen, true);
assert.equal(state.inTarget, true);
assert.equal(state.currentTargetSupportContact, true);
assert.ok(state.maxHeldHorizontalTravelM > 0.05, 'carry travel must be measured while the post-lift grasp remains continuous');
assert.ok(state.settleEvidenceDurationSeconds >= 0.20, 'settling must be supported by simulation-time dwell evidence');
assert.ok(state.settlePositionDriftM <= 0.0005, 'settling must remain positionally stable');
assert.equal(state.success, true, 'final success requires supported lift/carry, release, and stable post-release target rest');

const outside = new So101BlockTransferEvaluator();
outside.observe(observation(0.0, SOURCE));
outside.observe(observation(1.0, SOURCE, gripperContact));
outside.observe(observation(2.0, [SOURCE[0], SOURCE[1], 0.31], gripperContact));
outside.observe(observation(3.0, [0.42, -0.16, 0.31], gripperContact));
outside.observe(observation(4.0, [0.42, -0.16, 0.245]));
outside.observe(observation(4.6, [0.42, -0.16, 0.234], targetSupportContact));
state = outside.observe(observation(5.0, [0.42, -0.16, 0.234], targetSupportContact));
assert.equal(state.inTarget, false);
assert.equal(state.success, false, 'support contact cannot substitute for final target inclusion');

const noCarry = new So101BlockTransferEvaluator();
noCarry.observe(observation(0.0, SOURCE));
noCarry.observe(observation(1.0, SOURCE, gripperContact));
noCarry.observe(observation(2.0, [SOURCE[0], SOURCE[1], 0.31], gripperContact));
noCarry.observe(observation(3.0, TARGET));
noCarry.observe(observation(4.0, TARGET, targetSupportContact));
assert.equal(noCarry.snapshot().carrySeen, false);
assert.equal(noCarry.snapshot().success, false, 'teleport-like target appearance without carried contact cannot succeed');

const touchedThenLiftedWithoutGrip = new So101BlockTransferEvaluator();
touchedThenLiftedWithoutGrip.observe(observation(0.0, SOURCE));
touchedThenLiftedWithoutGrip.observe(observation(1.0, SOURCE, gripperContact));
touchedThenLiftedWithoutGrip.observe(observation(2.0, [SOURCE[0], SOURCE[1], 0.31]));
state = touchedThenLiftedWithoutGrip.snapshot();
assert.equal(state.contactSeen, true);
assert.equal(state.liftSeen, false, 'historical contact cannot convert a later contactless rise into a supported lift');

const travelBeforeRegrasp = new So101BlockTransferEvaluator();
travelBeforeRegrasp.observe(observation(0.0, SOURCE));
travelBeforeRegrasp.observe(observation(1.0, SOURCE, gripperContact));
travelBeforeRegrasp.observe(observation(2.0, [SOURCE[0], SOURCE[1], 0.31], gripperContact));
travelBeforeRegrasp.observe(observation(2.5, [TARGET[0], TARGET[1], 0.31]));
travelBeforeRegrasp.observe(observation(3.0, [TARGET[0], TARGET[1], 0.31], gripperContact));
state = travelBeforeRegrasp.snapshot();
assert.equal(state.liftSeen, true);
assert.equal(state.carrySeen, false, 'horizontal travel cannot bridge a lost grasp and later re-contact');
assert.equal(state.maxHeldHorizontalTravelM, 0, 'a re-grasp establishes a new carry anchor instead of inheriting earlier free-body travel');

const transientRest = new So101BlockTransferEvaluator();
transientRest.observe(observation(0.0, SOURCE));
transientRest.observe(observation(1.0, SOURCE, gripperContact));
transientRest.observe(observation(2.0, [SOURCE[0], SOURCE[1], 0.31], gripperContact));
transientRest.observe(observation(3.0, [TARGET[0], TARGET[1], 0.31], gripperContact));
transientRest.observe(observation(4.0, [TARGET[0], TARGET[1], 0.245]));
transientRest.observe(observation(4.4, TARGET, targetSupportContact));
state = transientRest.observe(observation(4.7, [TARGET[0] + 0.003, TARGET[1], TARGET[2]], targetSupportContact));
assert.equal(state.settleSeen, false, 'continued post-release translation must restart the settling dwell');
state = transientRest.observe(observation(4.8, [TARGET[0] + 0.003, TARGET[1], TARGET[2]], targetSupportContact));
assert.equal(state.settleSeen, false, 'a restarted dwell shorter than the declared minimum is not rest');
state = transientRest.observe(observation(5.0, [TARGET[0] + 0.003, TARGET[1], TARGET[2]], targetSupportContact));
assert.equal(state.settleSeen, true, 'stable target support for the declared simulation-time dwell can establish rest');
assert.equal(state.success, true);

const lostSupportAfterRest = new So101BlockTransferEvaluator();
lostSupportAfterRest.observe(observation(0.0, SOURCE));
lostSupportAfterRest.observe(observation(1.0, SOURCE, gripperContact));
lostSupportAfterRest.observe(observation(2.0, [SOURCE[0], SOURCE[1], 0.31], gripperContact));
lostSupportAfterRest.observe(observation(3.0, [TARGET[0], TARGET[1], 0.31], gripperContact));
lostSupportAfterRest.observe(observation(4.0, [TARGET[0], TARGET[1], 0.245]));
lostSupportAfterRest.observe(observation(4.5, TARGET, targetSupportContact));
lostSupportAfterRest.observe(observation(4.8, TARGET, targetSupportContact));
assert.equal(lostSupportAfterRest.snapshot().success, true);
state = lostSupportAfterRest.observe(observation(5.0, [TARGET[0], TARGET[1], 0.25]));
assert.equal(state.settleSeen, false, 'final-rest evidence must fail closed if target support is later lost');
assert.equal(state.success, false);

evaluator.reset();
state = evaluator.snapshot();
assert.equal(state.success, false);
assert.equal(state.contactSeen, false);
assert.equal(state.initialPositionM, null);

console.log('SO-101 observation-derived task evaluator: OK');
