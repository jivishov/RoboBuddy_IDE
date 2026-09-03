import assert from 'node:assert/strict';
import test from 'node:test';
import { MicroDuckCommandBus } from '../src/microduck/command-bus.js';
import { MICRODUCK_COMMANDS, validateCommand } from '../src/microduck/command-catalog.js';
import { HOME_POLICY_POSITION, POLICY_JOINT_ORDER, policyToWire, wireToPolicy } from '../src/microduck/contract.js';
import { InferenceGate } from '../src/microduck/inference-gate.js';
import { buildObservation, OBSERVATION_LAYOUT } from '../src/microduck/observation.js';
import { MicroDuckPolicyDirector } from '../src/microduck/policy-director.js';
import { MicroDuckPolicyRuntime } from '../src/microduck/policy-runtime.js';
import { createMicroDuckState } from '../src/microduck/state.js';
import { MICRODUCK_FIELD_HALF_EXTENT_M, MICRODUCK_FIELD_SIZE_M, stopPlanarBodyAtFieldEdge } from '../src/microduck/field-bounds.js';
import { applyConfiguredKick, applyRollingResistance, BALL_STOP_SPEED_MPS } from '../src/microduck/ball-motion.js';
import { buildPatchedWorkspace } from '../src/task-workspace.js';

test('the frozen catalog has the complete shared command vocabulary', () => {
  assert.deepEqual(Object.keys(MICRODUCK_COMMANDS), ['move','head','look','stop','enable','init','relax','do','pose','mouth','sound','theremin','chorale','get_mode','set_mode','get_state','set_color','spawn_ball','reset','set_tof_stimulus','set_camera']);
  assert(Object.isFrozen(MICRODUCK_COMMANDS));
  for (const definition of Object.values(MICRODUCK_COMMANDS)) {
    for (const field of ['authorities', 'modes', 'classification', 'completion', 'timeoutMs', 'cancellable', 'safeAbort', 'neutral', 'expiry']) assert(Object.hasOwn(definition, field), `missing ${field}`);
    assert.deepEqual(definition.authority.human, { priority: 3, defaultDurationMs: 250, minimumDurationMs: 1, maximumDurationMs: 250, durationRequired: false });
    assert.deepEqual(definition.authority.python, { priority: 2, defaultDurationMs: 5000, minimumDurationMs: 1, maximumDurationMs: 5000, durationRequired: false });
    assert.deepEqual(definition.authority.webmcp, { priority: 1, defaultDurationMs: null, minimumDurationMs: 20, maximumDurationMs: 5000, durationRequired: true });
  }
});

test('the MicroDuck starter Run program produces clearly visible bounded forward travel', () => {
  const workspace = buildPatchedWorkspace('microduck', { id: 'test-microduck', simulationMode: 'policy_sim' });
  assert.match(workspace['main.py'], /await robot\.move\(0\.30, 0\.0, 0\.0\)/);
  assert.match(workspace['main.py'], /await robot\.sleep\(4\.0\)/);
  assert.equal(0.30 * 4.0, 1.2, 'the starter command travels 1.2 m, visibly within the configured 8 m field');
});

test('observation layout is exact and mouth stays outside policy vectors', () => {
  const wire = policyToWire(HOME_POLICY_POSITION, 0.75);
  assert.equal(wire.length, 15); assert.equal(wire[9], 0.75); assert.deepEqual(wireToPolicy(wire), [...HOME_POLICY_POSITION]);
  const observation = buildObservation({ gyro: [1,2,3], projectedGravity: [4,5,6], wireJointPosition: wire, wireJointVelocity: new Array(15).fill(0), lastAction: new Array(14).fill(0.25), command: { twist: [7,8,9], head: [10,11,12,13], body: { x: 99, y: 99, z: 14, roll: 15, pitch: 16, yaw: 99 } } });
  assert.equal(observation.length, 61);
  assert.deepEqual(Array.from(observation.slice(...OBSERVATION_LAYOUT.jointPosition)), new Array(14).fill(0));
  assert.deepEqual(Array.from(observation.slice(...OBSERVATION_LAYOUT.command)), [7,8,9,10,11,12,13,0,0,14,15,16,0]);
});

test('limits and look solving follow walking and roller contracts', () => {
  assert.deepEqual(validateCommand('move', { vx: 2, vy: -2, yaw: 3 }, 'walking').applied, { vx: 0.3, vy: -0.3, yaw: 1.5 });
  const roller = validateCommand('move', { vx: -2, vy: 0.2, yaw: -2 }, 'roller');
  assert.deepEqual(roller.applied, { vx: -0.5, vy: 0, yaw: -0.3 }); assert(roller.limitedBy.includes('roller_no_strafe'));
  const look = validateCommand('look', { x: 0.25, y: 0.1, z: 0.2 }, 'walking');
  assert.equal(Object.keys(look.applied.solvedHead).length, 4);
  assert.equal(look.applied.solvedHead.neckPitch, 0);
  assert(Object.values(look.applied.solvedHead).every(Number.isFinite));
  assert.throws(() => validateCommand('look', { x: 0.25, y: 0.1 }, 'walking'), { code: 'INVALID_ARGUMENT' });
  const pose = validateCommand('pose', { x: 1, z: -1, roll: 1, pitch: -1, yaw: 1 }, 'walking');
  assert.deepEqual([pose.applied.x, pose.applied.y, pose.applied.yaw], [0,0,0]);
  assert.deepEqual([pose.applied.z, pose.applied.roll, pose.applied.pitch], [-0.025,0.26,-0.26]);
});

test('human preempts lower authority while peer conflicts and expiry neutralizes owned intent', () => {
  let now = 100;
  let expired = null;
  const bus = new MicroDuckCommandBus({ now: () => now, onExpire: (owner) => { expired = owner; } });
  bus.execute('move', { vx: 0.2 }, { source: 'python', controllerId: 'run-1', durationMs: 5000 });
  assert.throws(() => bus.execute('head', { headYaw: 0.2 }, { source: 'webmcp', controllerId: 'tool', durationMs: 100 }), { code: 'COMMAND_CONFLICT' });
  assert.throws(() => bus.execute('reset', {}, { source: 'webmcp', controllerId: 'tool' }), { code: 'COMMAND_CONFLICT' });
  assert.doesNotThrow(() => bus.execute('get_state', {}, { source: 'webmcp', controllerId: 'tool' }), 'read-only commands remain available across a controller lease');
  bus.execute('move', { vx: -0.1 }, { source: 'human', controllerId: 'keyboard' });
  assert.equal(bus.snapshot().lease.source, 'human');
  now += 251; assert.equal(bus.snapshot().lease, null); assert.deepEqual(bus.snapshot().values.move, { vx: 0, vy: 0, yaw: 0 }); assert.equal(expired.source, 'human');

  const oneShotPreemption = new MicroDuckCommandBus({ now: () => now });
  oneShotPreemption.execute('move', { vx: 0.1 }, { source: 'python', controllerId: 'run-3' });
  oneShotPreemption.execute('reset', {}, { source: 'human', controllerId: 'deck' });
  assert.equal(oneShotPreemption.snapshot().lease, null);
  assert.deepEqual(oneShotPreemption.snapshot().values.move, { vx: 0, vy: 0, yaw: 0 });

  const persistentLook = new MicroDuckCommandBus({ now: () => now });
  persistentLook.execute('head', { headYaw: 0.2 }, { source: 'python', controllerId: 'run-2', durationMs: 100 });
  const look = persistentLook.execute('look', { x: 0.25, y: 0.1, z: 0.2 }, { source: 'python', controllerId: 'run-2' });
  now += 101;
  assert.deepEqual(persistentLook.snapshot().values.head, look.applied.solvedHead, 'expiry must not neutralize a later unleased look intent');
});

test('WebMCP duration rules distinguish active continuous audio from immediate stops', () => {
  const bus = new MicroDuckCommandBus({ now: () => 0 });
  assert.throws(() => bus.execute('theremin', { active: true }, { source: 'webmcp', controllerId: 'tool' }), { code: 'INVALID_ARGUMENT' });
  assert.doesNotThrow(() => bus.execute('theremin', { active: false }, { source: 'webmcp', controllerId: 'tool' }));
  assert.throws(() => bus.execute('sound', { tag: 'wheee', hold: true }, { source: 'webmcp', controllerId: 'tool' }), { code: 'INVALID_ARGUMENT' });
  bus.execute('sound', { tag: 'wheee', hold: true }, { source: 'webmcp', controllerId: 'tool', durationMs: 100 });
  assert.deepEqual(bus.snapshot().values.sound, { tag: 'wheee', hold: true });
  bus.expire(101); assert.deepEqual(bus.snapshot().values.sound, { tag: null, hold: false });
});

test('policy priority, mode mapping, timings, scaling and bounded target filtering use the runtime contract', () => {
  const ranges = Object.fromEntries(POLICY_JOINT_ORDER.map((name) => [name, [-Math.PI, Math.PI]]));
  const director = new MicroDuckPolicyDirector({ jointRanges: ranges });
  assert.equal(director.currentPolicy(), 'stand');
  assert.equal(director.currentPolicy({ twist: [0.2,0,0] }), 'walking');
  director.trigger('kick_left'); assert.equal(director.currentPolicy(), 'kick_left'); director.advance(0.5); assert.equal(director.currentPolicy(), 'stand');
  director.trigger('ground_pick'); assert.equal(director.currentPolicy(), 'ground_pick'); director.advance(2.8); assert.equal(director.currentPolicy(), 'stand');
  director.trigger('roulade'); assert.equal(director.currentPolicy(), 'roulade'); director.advance(1); assert.equal(director.currentPolicy(), 'stand');
  director.trigger('sit_toggle'); assert.equal(director.currentPolicy(), 'sitstand'); director.trigger('sit_toggle'); director.advance(1); assert.equal(director.currentPolicy(), 'stand');
  director.reset('roller'); assert.equal(director.currentPolicy({ twist: [0,0,0] }), 'roller'); director.trigger('ground_pick'); assert.equal(director.currentPolicy(), 'roller_crouch');
  const first = director.applyAction(new Float32Array(14).fill(1), 'roller');
  const second = director.applyAction(new Float32Array(14).fill(0), 'roller');
  assert(Math.abs(first[0] - (HOME_POLICY_POSITION[0] + 0.06)) < 1e-6);
  assert(Math.abs(first[5] - (HOME_POLICY_POSITION[5] + 0.04)) < 1e-6);
  assert(Math.abs(second[0] - (HOME_POLICY_POSITION[0] + 0.018)) < 1e-6);
  assert(Math.abs(second[5] - (HOME_POLICY_POSITION[5] + 0.02)) < 1e-6);
  const surge = director.applyAction(new Float32Array(14).fill(-20), 'roller');
  assert(Math.max(...surge.map((value, index) => Math.abs(value - second[index]))) <= 0.060001);
});

test('stale and overlapping inference are rejected by epoch and single-flight gate', () => {
  const gate = new InferenceGate(); const first = gate.begin(); assert(first); assert.equal(gate.begin(), null); assert(gate.accepts(first));
  gate.invalidate(); assert(!gate.accepts(first)); const second = gate.begin(); assert(second); assert(!gate.accepts(first)); assert(gate.accepts(second)); gate.finish(second); assert.equal(gate.inFlight, false);
});

test('policy runtime resolves its ONNX files relative to the module for GitHub Pages project deployments', async () => {
  const requests = [];
  const ort = {
    env: { wasm: {} },
    InferenceSession: {
      create: async (url) => {
        requests.push(url);
        return { release: async () => {} };
      },
    },
  };
  const runtime = new MicroDuckPolicyRuntime({ importOrt: async () => ort });
  await runtime.initialize();
  const assetRoot = new URL('../assets/microduck/', import.meta.url);
  assert.equal(ort.env.wasm.wasmPaths, new URL('runtime/onnx/', assetRoot).href);
  assert.equal(requests.length, 7);
  assert(requests.every((url) => url.startsWith(assetRoot.href)));
  await runtime.dispose();
});

test('published state is bounded, immutable and truthful about fidelity', () => {
  const state = createMicroDuckState({ joints: new Array(30).fill(1), targets: new Array(30).fill(2), mouth: 4, ball: { position: [1,2,3] } });
  assert.equal(state.joints.length, 14); assert.equal(state.targets.length, 14); assert.equal(state.mouth, 1); assert.equal(state.ball.attached, false);
  assert.equal(state.sourcePlantAvailable, false); assert.equal(state.policySimulationAvailable, true); assert.equal(state.hardwareValidated, false); assert(Object.isFrozen(state)); assert(Object.isFrozen(state.ball.position));
  assert.throws(() => { state.ball.position[0] = 9; }, TypeError);
});

test('the configured field is 8 m square and stops duck and ball travel at every edge', () => {
  assert.equal(MICRODUCK_FIELD_HALF_EXTENT_M, 4);
  assert.equal(MICRODUCK_FIELD_SIZE_M, 8);
  const duck = stopPlanarBodyAtFieldEdge([4.2, -1, 0.13], [0.3, 0.2, 0], 0.07);
  assert.deepEqual(duck.position, [3.93, -1, 0.13]);
  assert.deepEqual(duck.velocity, [0, 0, 0]);
  assert.equal(duck.reachedBoundary, true);
  const ball = stopPlanarBodyAtFieldEdge([0.4, -4.2, 0.035], [0.1, -0.3, 0], 0.035);
  assert.deepEqual(ball.position, [0.4, -3.965, 0.035]);
  assert.deepEqual(ball.velocity, [0, 0, 0]);
  assert.equal(ball.reachedBoundary, true);
  const interior = stopPlanarBodyAtFieldEdge([1, -1, 0.13], [0.2, -0.1, 0], 0.07);
  assert.deepEqual(interior, { position: [1, -1, 0.13], velocity: [0.2, -0.1, 0], reachedBoundary: false });
});

test('configured ball motion transfers a nearby kick and dissipates rolling motion', () => {
  const kicked = applyConfiguredKick({ skill: 'kick_left', rootPosition: [0, 0, 0.13], rootQuaternion: [1, 0, 0, 0], ballPosition: [0.32, 0, 0.035], ballVelocity: [0, 0, 0] });
  assert.equal(kicked.contacted, true);
  assert(kicked.velocity[0] > 0, 'a forward ball in the modeled foot zone receives forward speed');
  const far = applyConfiguredKick({ skill: 'kick_right', rootPosition: [0, 0, 0.13], rootQuaternion: [1, 0, 0, 0], ballPosition: [1, 0, 0.035], ballVelocity: [0, 0, 0] });
  assert.deepEqual(far, { contacted: false, velocity: [0, 0, 0] });
  const slowed = applyRollingResistance(kicked.velocity, 5);
  assert(Math.hypot(slowed[0], slowed[1]) <= BALL_STOP_SPEED_MPS);
});
