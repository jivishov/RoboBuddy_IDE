import assert from 'node:assert/strict';
import { MICRODUCK_COMMANDS } from '../src/microduck/command-catalog.js';
import { MicroDuckCommandBus } from '../src/microduck/command-bus.js';
import { MICRODUCK_PYTHON_METHODS, MicroDuckPythonBridge } from '../src/microduck/python-bridge.js';
import { buildPatchedWorkspace } from '../src/task-workspace.js';

const catalogMethods = Object.values(MICRODUCK_PYTHON_METHODS).filter(Boolean);
assert.deepEqual(new Set(catalogMethods), new Set(Object.keys(MICRODUCK_COMMANDS)), 'Python methods must cover the frozen command catalog exactly');
assert.deepEqual(Object.keys(MICRODUCK_PYTHON_METHODS).filter((name) => !MICRODUCK_PYTHON_METHODS[name]), ['connect', 'disconnect', 'sleep']);

const workspace = buildPatchedWorkspace('microduck', { id: 'microduck-policy-simulator', simulationMode: 'policy_sim' });
assert.deepEqual(Object.keys(workspace), ['main.py', 'trajectories.py', 'robot_config.py', 'workcell.py'], 'the WebMCP read allowlist remains exactly four files');
const workspaceText = Object.values(workspace).join('\n');
for (const method of Object.keys(MICRODUCK_PYTHON_METHODS)) assert.match(workspaceText, new RegExp(`\\.${method}\\(`), `${method} must have a callable starter example`);
assert.match(workspaceText, /approximate browser dynamics/);
assert.match(workspaceText, /never opens a socket or discovers hardware/);
const TEST_WORKSPACE = { 'main.py': '', 'trajectories.py': '', 'robot_config.py': '', 'workcell.py': '' };

let now = 10;
let preemption = null;
const bus = new MicroDuckCommandBus({ now: () => now, onPreempt: (previous, replacement) => { preemption = { previous, replacement }; } });
bus.execute('move', { vx: 0.2 }, { source: 'python', controllerId: 'run-1' });
now = 4000;
assert.equal(bus.refresh({ source: 'python', controllerId: 'run-1' }), true);
now = 6000;
assert.equal(bus.snapshot().values.move.vx, 0.2, 'an ordinary SDK call may refresh the five-second Python lease');
bus.execute('move', { vx: -0.1 }, { source: 'human', controllerId: 'manual' });
assert.equal(preemption.previous.source, 'python');
assert.equal(preemption.replacement.source, 'human');

class FakeWorker {
  constructor() { this.sent = []; this.terminated = false; this.onmessage = null; this.onerror = null; }
  postMessage(message) { this.sent.push(structuredClone(message)); }
  emit(message) { this.onmessage?.({ data: structuredClone(message) }); }
  terminate() { this.terminated = true; }
}

const workers = [];
const simulator = {
  paused: false,
  commands: [],
  advanced: [],
  refreshes: 0,
  refreshResult: true,
  acquisitions: 0,
  cancellations: [],
  pause() { this.paused = true; },
  resume() { this.paused = false; },
  stop() {},
  acquireController() { this.acquisitions += 1; return { source: 'python' }; },
  cancelController(source, controllerId) { this.cancellations.push({ source, controllerId }); return true; },
  refreshControllerLease() { this.refreshes += 1; return this.refreshResult; },
  async advanceTime(seconds, options) { this.advanced.push({ seconds, options: structuredClone(options) }); return true; },
  async executeCommand(command, args, context) {
    this.commands.push({ command, args: structuredClone(args), context: structuredClone(context) });
    if (command === 'get_state') return { ok: true, command, state: { time: 1.25, joints: [1, 2, 3], virtualCamera: { mode: 'head', name: 'Head POV', purpose: 'Modeled source-frame render; not hardware video.', frame: 'head_camera', inset: false, transport: 'rendered simulation imagery only; no hardware video or media transport' } } };
    return { ok: true, command, applied: structuredClone(args), limitedBy: [] };
  },
};
const makeBridge = () => new MicroDuckPythonBridge({ simulator, workerFactory: () => { const worker = new FakeWorker(); workers.push(worker); return worker; }, cooperativeGraceMs: 5, runTimeoutMs: 2000 });

const bridge = makeBridge();
const completion = bridge.start({ ...TEST_WORKSPACE, 'main.py': 'await robot.connect()' }, { workspaceEpoch: 7, mode: 'step' });
const worker = workers.at(-1);
worker.emit({ type: 'bridge-request', runEpoch: 1, workspaceEpoch: 999, requestId: 10, method: 'connect', args: {}, source: { file: 'main.py', line: 2 } });
assert.equal(worker.sent.some((item) => item.requestId === 10), false, 'stale workspace-epoch requests are ignored');
worker.emit({ type: 'bridge-request', runEpoch: 1, workspaceEpoch: 7, requestId: 11, method: 'connect', args: {}, source: { file: 'main.py', line: 3 } });
const first = bridge.step();
await first;
assert.equal(worker.sent.at(-1).requestId, 11, 'the correlated response must retain the worker request id');

simulator.refreshResult = false;
worker.emit({ type: 'bridge-request', runEpoch: 1, workspaceEpoch: 7, requestId: 14, method: 'connect', args: {}, source: { file: 'main.py', line: 3 } });
await bridge.step();
assert.equal(simulator.acquisitions, 2, 'a repeated connect reacquires an expired simulation lease');
simulator.refreshResult = true;

const refreshesBeforeSleep = simulator.refreshes;
worker.emit({ type: 'bridge-request', runEpoch: 1, workspaceEpoch: 7, requestId: 12, method: 'sleep', args: { seconds: 0.04 }, source: { file: 'main.py', line: 4 } });
const sleepOne = await bridge.step();
assert.equal(sleepOne.remainingSeconds, 0.02);
assert.equal(worker.sent.filter((item) => item.requestId === 12).length, 0, 'a partial sleep Step keeps the bridge request unresolved');
await bridge.step();
assert.equal(worker.sent.filter((item) => item.requestId === 12).length, 1);
assert.deepEqual(simulator.advanced, [{ seconds: 0.02, options: { controlStep: true } }, { seconds: 0.02, options: { controlStep: true } }]);
assert.equal(simulator.refreshes, refreshesBeforeSleep, 'cooperative sleep must not refresh the five-second no-SDK-call lease');

worker.emit({ type: 'bridge-request', runEpoch: 1, workspaceEpoch: 7, requestId: 13, method: 'get_state', args: {}, source: { file: 'main.py', line: 5 } });
simulator.refreshResult = false;
await bridge.step();
assert.equal(simulator.refreshes, refreshesBeforeSleep + 1, 'a catalog-backed SDK call refreshes an existing Python lease');
assert.equal(simulator.acquisitions, 3, 'an SDK call reacquires an expired Python lease before reaching the command bus');
simulator.refreshResult = true;
const stateResponse = worker.sent.find((item) => item.requestId === 13);
simulator.commands.at(-1).args.changed = true;
assert.deepEqual(stateResponse.result.state.joints, [1, 2, 3], 'state returned to Python is a bounded clone, not a simulator object');
assert.deepEqual(stateResponse.result.state.virtualCamera, { mode: 'head', name: 'Head POV', purpose: 'Modeled source-frame render; not hardware video.', frame: 'head_camera', inset: false, transport: 'rendered simulation imagery only; no hardware video or media transport' }, 'Python receives the visible camera identity and no-inset rendered-simulation boundary');

worker.emit({ type: 'complete', runEpoch: 1, workspaceEpoch: 7, ok: true, sourceAttribution: { filename: 'main.py', topLevelAwait: true } });
const result = await completion;
assert.deepEqual(result.sourceAttribution, { filename: 'main.py', topLevelAwait: true });
assert.equal(simulator.cancellations.at(-1).source, 'python', 'normal completion also releases the Python controller lease');

const recoveryBridge = makeBridge();
const cancelled = recoveryBridge.start({ ...TEST_WORKSPACE, 'main.py': 'while True: pass' }, { workspaceEpoch: 8 });
const stuckWorker = workers.at(-1);
await recoveryBridge.cancel('STOP');
await assert.rejects(cancelled, { code: 'OPERATION_CANCELLED' });
await new Promise((resolve) => setTimeout(resolve, 10));
assert.equal(stuckWorker.terminated, true, 'an uncooperative worker is forcibly terminated after the grace period');
const later = recoveryBridge.start({ ...TEST_WORKSPACE, 'main.py': 'print("ok")' }, { workspaceEpoch: 8 });
const laterWorker = workers.at(-1);
laterWorker.emit({ type: 'complete', runEpoch: 3, workspaceEpoch: 8, ok: true });
assert.equal((await later).ok, true, 'a forced stop must leave a later run usable');

const cooperativeBridge = makeBridge();
const cooperative = cooperativeBridge.start({ ...TEST_WORKSPACE, 'main.py': 'await robot.sleep(1)' }, { workspaceEpoch: 9 });
const cooperativeWorker = workers.at(-1);
await cooperativeBridge.cancel('RESET');
cooperativeWorker.emit({ type: 'complete', runEpoch: 1, workspaceEpoch: 9, ok: false, error: { code: 'OPERATION_CANCELLED' } });
await assert.rejects(cooperative, { code: 'OPERATION_CANCELLED' });
assert.equal(cooperativeWorker.terminated, true, 'a cooperatively settled cancellation terminates before forced cleanup is needed');

const timeoutWorkers = [];
const timeoutBridge = new MicroDuckPythonBridge({ simulator, workerFactory: () => { const worker = new FakeWorker(); timeoutWorkers.push(worker); return worker; }, cooperativeGraceMs: 5, runTimeoutMs: 5 });
const timedOut = timeoutBridge.start({ ...TEST_WORKSPACE, 'main.py': 'while True: pass' }, { workspaceEpoch: 10 });
await assert.rejects(timedOut, { code: 'POLICY_TIMEOUT' });
await new Promise((resolve) => setTimeout(resolve, 10));
assert.equal(timeoutWorkers[0].terminated, true, 'timeout invalidation also forces an unresponsive worker down');

await assert.rejects(makeBridge().start({ ...TEST_WORKSPACE, 'extra.py': '' }, { workspaceEpoch: 11 }), { code: 'INVALID_ARGUMENT' });

const invalidSourceBridge = makeBridge();
const invalidSourceRun = invalidSourceBridge.start(TEST_WORKSPACE, { workspaceEpoch: 12 });
const invalidSourceWorker = workers.at(-1);
invalidSourceWorker.emit({ type: 'bridge-request', runEpoch: 1, workspaceEpoch: 12, requestId: 21, method: 'get_state', args: {}, source: { file: '../secret.py', line: 1 } });
await assert.rejects(invalidSourceRun, { code: 'INVALID_ARGUMENT' });
assert.equal(invalidSourceWorker.terminated, true, 'invalid source attribution fails closed and terminates the run');

const boundedBridge = makeBridge();
const boundedRun = boundedBridge.start(TEST_WORKSPACE, { workspaceEpoch: 13 });
const boundedWorker = workers.at(-1);
boundedWorker.emit({ type: 'output', runEpoch: 1, workspaceEpoch: 13, stream: 'stdout', text: 'x'.repeat(70 * 1024) });
await assert.rejects(boundedRun, { code: 'INVALID_ARGUMENT' });
assert.equal(boundedWorker.terminated, true, 'oversized worker output fails closed and terminates the run');

console.log('MicroDuck Python bridge checks passed');
