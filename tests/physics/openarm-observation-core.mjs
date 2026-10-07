import assert from 'node:assert/strict';
import { conditionMet, graspState, contactEvidenceComplete } from '../../src/physics/openarm-observation.js';

const contact = (name, depth = -.0001, force = 1) => ({
  geom1Name: 'flask_grip_geom', geom2Name: name, distanceM: depth, normalForceN: force,
});
const inner = 'finger_inner_left_collision_02';
const outer = 'finger_outer_left_collision_02';
const palm = 'ee_base_link_left_collision_00';
const obs = (contacts = [], extra = {}) => ({ bodies: { flask: { positionM: [.55, .15, 1.1] } }, contactsReadable: true, contactCount: contacts.length, contacts, ...extra });
const released = { type: 'released', side: 'left', object_id: 'flask' };
const grasped = { ...released, type: 'bilateral_grasp' };
const supported = { type: 'supported', object_id: 'flask', support_geom: 'left_hotplate' };

assert.equal(conditionMet(obs(), released), true, 'complete empty contact evidence establishes no gripper contact');
for (const extra of [{ contactsReadable: false }, { contactsReadable: undefined }, { contactCount: 1 }, { contactCount: undefined }, { contactCount: -1 }, { bodies: {} }]) {
  assert.equal(conditionMet(obs([], extra), released), false, 'missing, partial or unreadable evidence cannot establish release');
}
assert.equal(conditionMet(obs(), { ...released, object_id: 'not_a_body' }), false);
assert.equal(contactEvidenceComplete(obs()), true);
for (const name of [inner, outer, palm]) {
  for (const [depth, force] of [[-.01, 2], [0, 0], [-.0001, 0], [.0001, .001], [NaN, 1], [-.0001, NaN]]) {
    assert.equal(conditionMet(obs([contact(name, depth, force)]), released), false, `contact ${name} must not disappear from release evidence`);
  }
  assert.equal(conditionMet(obs([contact(name, .0001, 0)]), released), true, 'zero-load positive separation is proximity, not touch');
}
const pinch = [contact(inner), contact(outer)];
assert.equal(conditionMet(obs(pinch), grasped), true);
assert.equal(conditionMet(obs(pinch, { contactsReadable: false }), grasped), false);
assert.equal(conditionMet(obs(pinch, { contactCount: 3 }), grasped), false);
assert.equal(conditionMet(obs([...pinch, contact(inner, -.01)]), grasped), false, 'a deep contact rejects grasp quality even alongside good contacts');
const deep = graspState(obs([contact(inner, -.01)]), 'left', 'flask');
assert.equal(deep.bilateralContact, false);
assert.equal(deep.anyFingerContact, true);
assert.equal(deep.anyGripperContact, true);
assert.equal(deep.excessivePenetrationSeen, true);
assert.equal(graspState(obs([contact(palm)]), 'left', 'flask').anyPalmContact, true);
assert.equal(conditionMet(obs([contact('finger_inner_right_collision_02')]), released), true, 'release is scoped to the requested gripper');
assert.equal(conditionMet(obs([contact('left_hotplate')]), supported), true);
assert.equal(conditionMet(obs([contact('left_hotplate')], { contactsReadable: false }), supported), false);
assert.equal(conditionMet(obs([contact('left_hotplate', -.01)]), supported), false);
for (const p of [[], [1], [NaN, 0, 0], [Infinity, 0, 0]]) {
  const o = obs([], { bodies: { flask: { positionM: p } }, openarm: { pinchReferences: { left: { positionM: p } } } });
  assert.equal(conditionMet(o, { type: 'in_region', object_id: 'flask', center_m: [0,0,0], half_extents_m: [1,1,1] }), false);
  assert.equal(conditionMet(o, { type: 'tool_reached', side: 'left', position_m: [0,0,0], tolerance_m: .01 }), false);
}
// This module is imported by the model/schema gate as well as runnable alone.
