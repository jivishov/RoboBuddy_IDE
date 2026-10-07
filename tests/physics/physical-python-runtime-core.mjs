import assert from 'node:assert/strict';
import { PhysicalPythonRuntime } from '../../src/runtime/physical-python-runtime.js';

class FakeWorker {
  constructor() { this.messages = []; this.terminated = false; this.onmessage = null; this.onerror = null; }
  postMessage(message) { this.messages.push(structuredClone(message)); }
  terminate() { this.terminated = true; }
  emit(message) { this.onmessage?.({ data: structuredClone(message) }); }
}

class FakeBridge {
  constructor() { this.calls = []; this.cancelled = false; this.disposed = false; }
  async connect(robotId) { this.calls.push(['connect', robotId]); return { apiVersion: 'robobuddy.sim.v1', robotId, epoch: 1, sceneRevision: 'v1', timestepSeconds: 0.005 }; }
  async sendAction(targets, options) { this.calls.push(['sendAction', targets, options]); return { status: 'accepted', acceptedTargetsRad: targets }; }
  async advance(seconds) { this.calls.push(['advance', seconds]); return { simulationTimeSeconds: seconds }; }
  async getObservation(options) { this.calls.push(['getObservation', options]); return { simulationTimeSeconds: 0, joints: {} }; }
  async waitForGoal(targets, options) { this.calls.push(['waitForGoal', targets, options]); return { status: 'timeout' }; }
  async pause() { this.calls.push(['pause']); return { paused: true }; }
  async resume() { this.calls.push(['resume']); return { paused: false }; }
  async reset() { this.calls.push(['reset']); return { connection: { epoch: 2 } }; }
  disconnect() { this.calls.push(['disconnect']); return { disconnected: true }; }
  async cancel(reason) { this.calls.push(['cancel', reason]); this.cancelled = true; return { cancelled: true }; }
  dispose() { this.disposed = true; }
}

const tick = () => new Promise((resolve) => setTimeout(resolve, 0));
let worker = null;
let bridge = null;
const states = [];
const runtime = new PhysicalPythonRuntime({
  workerFactory: () => (worker = new FakeWorker()),
  bridgeFactory: () => (bridge = new FakeBridge()),
  onState: (state) => states.push(state),
  cooperativeGraceMs: 5,
  runTimeoutMs: 10000,
});
const completion = runtime.start({ 'main.py': 'while True: pass' }, { workspaceEpoch: 4, robotId: 'so101_follower' });
assert.equal(runtime.isActive(), true);
const runMessage = worker.messages[0];
assert.equal(runMessage.type, 'run');
const runEpoch = runMessage.runEpoch;

worker.emit({ type: 'bridge-request', runEpoch, workspaceEpoch: 4, requestId: 1, method: 'connect', args: { robot_id: 'so101_follower' }, source: { file: 'main.py', line: 1 } });
await tick();
assert.deepEqual(bridge.calls[0], ['connect', 'so101_follower']);
assert.equal(worker.messages.at(-1).type, 'bridge-response');

assert.equal(await runtime.pause(), true);
assert.equal(runtime.isPaused(), true);
assert.deepEqual(bridge.calls.at(-1), ['pause']);
const callsWhilePaused = bridge.calls.length;
worker.emit({ type: 'bridge-request', runEpoch, workspaceEpoch: 4, requestId: 2, method: 'advance', args: { seconds: 0.2 }, source: { file: 'main.py', line: 2 } });
await tick();
assert.equal(bridge.calls.length, callsWhilePaused, 'externally paused Python must not advance physical simulation');
assert.equal(await runtime.resume(), true);
await tick();
assert.equal(runtime.isPaused(), false);
assert.deepEqual(bridge.calls.at(-2), ['resume']);
assert.deepEqual(bridge.calls.at(-1), ['advance', 0.2]);

worker.emit({ type: 'bridge-request', runEpoch, workspaceEpoch: 4, requestId: 3, method: 'send_action', args: { targets: { shoulder_pan: 0.2 }, max_steps: 100 }, source: { file: 'main.py', line: 3 } });
await tick();
assert.deepEqual(bridge.calls.at(-1), ['sendAction', { shoulder_pan: 0.2 }, { maxSteps: 100 }]);

// The fake worker now models a non-yielding user loop: it sends no more messages and never completes.
await runtime.cancel('STOP', { immediate: true });
await assert.rejects(completion, (error) => error.code === 'OPERATION_CANCELLED');
assert.equal(worker.terminated, true, 'non-yielding Python must be terminable by worker termination');
assert.equal(bridge.cancelled, true, 'program cancellation must revoke the physical session');
assert.equal(runtime.isActive(), false);

const priorCallCount = bridge.calls.length;
worker.emit({ type: 'bridge-request', runEpoch, workspaceEpoch: 4, requestId: 99, method: 'send_action', args: { targets: { shoulder_pan: 0.9 }, max_steps: 10 }, source: { file: 'main.py', line: 99 } });
await tick();
assert.equal(bridge.calls.length, priorCallCount, 'stale worker response must not command a replaced/cancelled physical session');
assert.ok(states.some((item) => item.state === 'paused'));
assert.ok(states.some((item) => item.state === 'running'));
assert.ok(states.some((item) => item.state === 'idle' && item.reason === 'STOP'));

console.log('Physical Python pause/cancellation/epoch checks: OK');
