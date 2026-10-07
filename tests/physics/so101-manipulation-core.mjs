import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFileSync } from 'node:fs';
import { SO101_MANIPULATION_MODEL_PACKAGE, SO101_PHASE2A_MODEL_PACKAGE } from '../../src/physics/model-packages.js';
import { SO101_MANIPULATION_SCENE, SO101_BENCHMARK_TRANSFER_CONTROLLER } from '../../src/physics/so101-scene.js';

assert.equal(SO101_PHASE2A_MODEL_PACKAGE.sha256, '8573559b58eb522ca80c8cd4c88d30e0b2ab20fb222f67c57e3808a4783af085', 'Phase 2A model bytes must remain the frozen reference');
assert.equal(SO101_MANIPULATION_MODEL_PACKAGE.source.revision, '8161bba264d7fa7c99ca301e91e7fb44737676ad');
assert.equal(SO101_MANIPULATION_MODEL_PACKAGE.source.upstreamRevision, 'aec17bbc256d1a7342d53aaa4950595d4c30b40d');
assert.equal(SO101_MANIPULATION_MODEL_PACKAGE.physics.timestepSeconds, 0.005);
assert.equal(SO101_MANIPULATION_MODEL_PACKAGE.benchmark.object.massKg, 0.020);
assert.deepEqual(SO101_MANIPULATION_MODEL_PACKAGE.benchmark.object.dimensionsM, [0.016, 0.012, 0.014]);
assert.deepEqual(SO101_MANIPULATION_MODEL_PACKAGE.benchmark.target.halfExtentsXYM, [0.020, 0.030]);
assert.equal(SO101_MANIPULATION_MODEL_PACKAGE.benchmark.controllerVersion, SO101_BENCHMARK_TRANSFER_CONTROLLER.id);
assert.equal(SO101_MANIPULATION_SCENE.modelPackage, SO101_MANIPULATION_MODEL_PACKAGE.id);
assert.deepEqual(SO101_MANIPULATION_SCENE.fixtures.map((item) => item.id), ['benchmark_source_support', 'benchmark_target_support', 'benchmark_target_region']);
assert.deepEqual(SO101_MANIPULATION_SCENE.objects.map((item) => item.id), ['benchmark_block']);
assert.deepEqual(SO101_MANIPULATION_SCENE.taskGoal.targetHalfExtentsXYM, [0.020, 0.030]);
assert.equal(SO101_MANIPULATION_SCENE.taskGoal.requireContact, true);
assert.equal(SO101_MANIPULATION_SCENE.taskGoal.requireFinalRest, true);

const xml = readFileSync('models/so101/manipulation.xml');
assert.equal(createHash('sha256').update(xml).digest('hex'), SO101_MANIPULATION_MODEL_PACKAGE.sha256);
const xmlText = xml.toString('utf8');
assert.match(xmlText, /<freejoint name="benchmark_block_free"\/>/);
assert.match(xmlText, /name="benchmark_source_support"/);
assert.match(xmlText, /name="benchmark_target_support"/);
assert.doesNotMatch(xmlText, /name="benchmark_work_surface"/);
assert.match(xmlText, /name="benchmark_target_region"[^>]+contype="0" conaffinity="0"/);
assert.match(xmlText, /name="camera_box1"[^>]+mass="0\.0040540541"/);
assert.match(xmlText, /name="camera_box2"[^>]+mass="0\.0079459459"/);
assert.doesNotMatch(xmlText, /weld|equality/);

const ranges = Object.fromEntries(SO101_MANIPULATION_MODEL_PACKAGE.joints.map((joint) => [joint.id, joint.rangeRad]));
for (const [jointId, value] of Object.entries(SO101_MANIPULATION_MODEL_PACKAGE.initialJointPositionsRad)) {
  assert.ok(ranges[jointId], `initial state joint ${jointId} must be declared`);
  assert.ok(value >= ranges[jointId][0] && value <= ranges[jointId][1], `initial ${jointId} must be in joint range`);
}
assert.equal(SO101_MANIPULATION_MODEL_PACKAGE.initialJointPositionsRad.wrist_roll, Math.PI / 2);
assert.equal(SO101_MANIPULATION_MODEL_PACKAGE.joints.find((joint) => joint.id === 'wrist_roll').rangeRad[1], 2.7438473, 'known pinned Menagerie wrist_roll constraint must remain unchanged');

const dt = SO101_MANIPULATION_MODEL_PACKAGE.physics.timestepSeconds;
for (const stage of SO101_BENCHMARK_TRANSFER_CONTROLLER.stages) {
  assert.ok(Number.isInteger(Math.round(stage.durationSeconds / dt)));
  assert.ok(Math.abs(stage.durationSeconds / dt - Math.round(stage.durationSeconds / dt)) < 1e-12, `${stage.name} duration must align to physics timestep`);
  assert.ok(Math.abs(stage.durationSeconds / SO101_BENCHMARK_TRANSFER_CONTROLLER.controllerPeriodSeconds - Math.round(stage.durationSeconds / SO101_BENCHMARK_TRANSFER_CONTROLLER.controllerPeriodSeconds)) < 1e-12, `${stage.name} must align to controller interval`);
}
assert.equal(SO101_BENCHMARK_TRANSFER_CONTROLLER.controllerPeriodSeconds / dt, 4, 'controller interval must remain distinct from the 5 ms physics timestep');

console.log('SO-101 manipulation package/scene contract: OK');
