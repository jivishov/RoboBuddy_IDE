import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { mock } from 'node:test';
import { MicroDuckCommandBus } from '../src/microduck/command-bus.js';
import { MicroDuckPythonBridge } from '../src/microduck/python-bridge.js';
import { buildPatchedWorkspace } from '../src/task-workspace.js';

// Execute the actual generated Python, rather than a hand-maintained copy of
// its loop. This recorder models SDK boundaries, not robot dynamics or Pyodide.
const python = String.raw`
import ast, asyncio, contextlib, io, json, sys, types
calls = []
class Robot:
    def __getattr__(self, method):
        async def call(*args):
            calls.append([method, list(args)])
            if method == 'get_state':
                return {'time': 4, 'mode': 'walking', 'simulatedPose': {'position': [0, 0, 0]}}
        return call
config = types.ModuleType('robot_config')
config.create_robot = Robot
sys.modules['robot_config'] = config
code = compile(sys.stdin.read(), 'main.py', 'exec', flags=ast.PyCF_ALLOW_TOP_LEVEL_AWAIT)
with contextlib.redirect_stdout(io.StringIO()):
    asyncio.run(eval(code, {}))
print(json.dumps(calls))
`;
const workspace = buildPatchedWorkspace('microduck', { id: 'starter-timing', simulationMode: 'policy_sim' });
const result = spawnSync(process.platform === 'win32' ? 'py' : 'python3', [...(process.platform === 'win32' ? ['-3'] : []), '-c', python], {
  input: workspace['main.py'], encoding: 'utf8', timeout: 10_000,
});
assert.ifError(result.error);
assert.equal(result.status, 0, result.stderr);
const calls = JSON.parse(result.stdout);
const moves = calls.filter(([method]) => method === 'move');
const sleeps = calls.filter(([method]) => method === 'sleep');
assert.equal(moves.length, 16);
assert(moves.every(([, args]) => JSON.stringify(args) === JSON.stringify([0.30, 0, 0])));
assert.equal(sleeps.reduce((total, [, args]) => total + args[0], 0), 4);
assert(sleeps.every(([, args]) => args[0] > 0 && args[0] <= 0.25));

async function replay(boundaries) {
  let wallMs = 0;
  let simTime = 0;
  let position = 0;
  let enabled = false;
  let expirations = 0;
  let refreshes = 0;
  const bus = new MicroDuckCommandBus({ now: () => wallMs, onExpire: () => { expirations += 1; } });
  const simulator = {
    getState: () => ({ time: simTime, lifecycle: 'ready', enabled, actuationEnabled: true }),
    acquireController: (source, controllerId, durationMs) => bus.connect({ source, controllerId, durationMs }),
    refreshControllerLease: (source, controllerId, durationMs) => { refreshes += 1; return bus.refresh({ source, controllerId, durationMs }); },
    cancelController: (source, controllerId) => bus.cancel({ source, controllerId }),
    async executeCommand(name, args, context) {
      if (name === 'enable') enabled = args.enabled;
      return bus.execute(name, args, context);
    },
  };
  const bridge = new MicroDuckPythonBridge({ simulator });
  const active = { connected: false, paused: false, controllerId: 'timing-test', worker: {} };
  bridge.active = active;
  bridge.worker = active.worker;
  mock.timers.enable({ apis: ['setTimeout'] });
  try {
    for (const [method, args] of boundaries) {
      if (method === 'sleep') {
        const before = refreshes;
        let done = false;
        const sleep = bridge.executeSleepBoundary(active, { args: { seconds: args[0] } }, false);
        sleep.then(() => { done = true; });
        // Simulate a browser running physics at one fifth of real-time speed.
        // Advance virtual timers so the regression does not take 20 real seconds.
        for (let polls = 0; !done && polls < 2000; polls += 1) {
          const vx = bus.snapshot().values.move.vx;
          wallMs += 20;
          simTime += 0.004;
          if (enabled) position += vx * 0.004;
          mock.timers.tick(20);
          await Promise.resolve();
          await Promise.resolve();
        }
        assert(done, 'cooperative sleep must finish as modeled time advances');
        await sleep;
        assert.equal(refreshes, before, 'sleep itself must not refresh the safety lease');
      } else {
        const parameters = method === 'move' ? { vx: args[0], vy: args[1], yaw: args[2] }
          : method === 'enable' ? { enabled: args[0] } : {};
        await bridge.executeCommandBoundary(active, { method, args: parameters });
      }
    }
    return { wallMs, simTime, position, expirations, lease: bus.snapshot().lease };
  } finally {
    mock.timers.reset();
    bridge.active = null;
    bridge.worker = null;
  }
}

// The prior one-shot move + four-second sleep loses motion at five wall seconds.
const previous = await replay([
  ['connect', []], ['enable', [true]], ['move', [0.30, 0, 0]],
  ['sleep', [4]], ['stop', []], ['disconnect', []],
]);
assert.equal(previous.expirations, 1, 'a long silent sleep still expires the five-second watchdog');
assert(previous.position < 1.1, 'the old starter must reproduce the short-travel failure');
const current = await replay(calls);
assert(current.wallMs > 5000, 'exercise actual wall/simulation clock divergence');
assert(current.simTime >= 4 && current.simTime < 4.2, 'preserve four modeled seconds rather than increasing requested travel');
assert.equal(current.expirations, 0, 'explicit SDK commands must keep this slow starter moving');
assert(current.position > 1.1 && current.position < 1.3, 'preserve the browser distance requirement');
assert.equal(current.lease, null, 'the starter must release control on completion');
console.log(`MicroDuck slow-clock regression: previous=${previous.position.toFixed(3)} m; current=${current.position.toFixed(3)} m; watchdog preserved`);
