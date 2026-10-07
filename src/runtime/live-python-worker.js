const PYODIDE_VERSION = '0.29.4';
const PYODIDE_ROOT = `https://cdn.jsdelivr.net/pyodide/v${PYODIDE_VERSION}/full/`;
const MAX_WORKSPACE_BYTES = 512 * 1024;
const MAX_OUTPUT_BYTES = 64 * 1024;
const MAX_FILES = 24;
const PYTHON_FILE_RE = /^[A-Za-z0-9_.-]+\.py$/;
const pending = new Map();
let sequence = 0;
let active = null;

self.onmessage = (event) => {
  const message = event.data || {};
  if (message.type === 'run') { void runWorkspace(message); return; }
  if (message.type === 'bridge-response') {
    if (!active || message.runEpoch !== active.runEpoch || message.workspaceEpoch !== active.workspaceEpoch) return;
    const entry = pending.get(message.requestId);
    if (!entry) return;
    pending.delete(message.requestId);
    if (message.ok) entry.resolve(JSON.stringify(message.result ?? null));
    else entry.reject(new Error(`[${message.error?.code || 'PYTHON_BRIDGE'}] ${message.error?.message || 'Physical simulator bridge request failed.'}`));
    return;
  }
  if (message.type === 'cancel') {
    for (const [requestId, entry] of pending) {
      pending.delete(requestId);
      entry.reject(new Error(`[OPERATION_CANCELLED] ${message.reason || 'Physical Python run cancelled.'}`));
    }
  }
};

async function runWorkspace(message) {
  active = { runEpoch: message.runEpoch, workspaceEpoch: message.workspaceEpoch };
  try {
    const files = validateWorkspace(message.files || {});
    const { loadPyodide } = await import(`${PYODIDE_ROOT}pyodide.mjs`);
    const pyodide = await loadPyodide({ indexURL: PYODIDE_ROOT });
    pyodide.setStdout({ batched: (text) => post('output', { stream: 'stdout', text: `${text}\n` }) });
    pyodide.setStderr({ batched: (text) => post('output', { stream: 'stderr', text: `${text}\n` }) });
    pyodide.registerJsModule('_robobuddy_sim_bridge', { request: bridgeRequest });
    for (const [name, content] of Object.entries(files)) pyodide.FS.writeFile(name, content);
    await pyodide.runPythonAsync(PYTHON_MODULE, { filename: '<robobuddy-sim-shim>' });
    await pyodide.runPythonAsync(String(files['main.py'] || ''), { filename: 'main.py' });
    post('complete', { ok: true, sourceAttribution: { evaluator: 'Pyodide runPythonAsync', apiVersion: 'robobuddy.sim.v1', topLevelAwait: true, filename: 'main.py' } });
  } catch (error) {
    const text = clip(error?.message || error);
    const code = text.match(/\[([A-Z][A-Z0-9_]+)\]/)?.[1] || error?.code || 'PYTHON';
    post('complete', { ok: false, error: { code, message: text, stack: clip(error?.stack || '') } });
  }
}

function validateWorkspace(files) {
  const entries = Object.entries(files);
  if (!Object.hasOwn(files, 'main.py')) throw new Error('Physical Python workspace requires main.py.');
  if (!entries.length || entries.length > MAX_FILES) throw new Error(`Physical Python workspace supports 1..${MAX_FILES} Python files.`);
  let total = 0;
  const clean = {};
  for (const [name, raw] of entries) {
    if (!PYTHON_FILE_RE.test(name)) throw new Error(`Physical Python worker rejected workspace filename ${name}.`);
    const content = String(raw);
    total += new TextEncoder().encode(content).byteLength;
    if (total > MAX_WORKSPACE_BYTES) throw new Error(`Physical Python workspace exceeds ${MAX_WORKSPACE_BYTES} bytes.`);
    clean[name] = content;
  }
  return clean;
}

function bridgeRequest(method, argsJson, file, line) {
  if (!active) return Promise.reject(new Error('No active physical Python run.'));
  const requestId = ++sequence;
  const promise = new Promise((resolve, reject) => pending.set(requestId, { resolve, reject }));
  post('bridge-request', { requestId, method: String(method), args: JSON.parse(String(argsJson || '{}')), source: { file: String(file), line: Number(line) } });
  return promise;
}

function post(type, extra = {}) { self.postMessage({ type, runEpoch: active?.runEpoch, workspaceEpoch: active?.workspaceEpoch, ...extra }); }
function clip(value) { const text = String(value); return text.length <= MAX_OUTPUT_BYTES ? text : `${text.slice(0, MAX_OUTPUT_BYTES)}\n…truncated by the physical Python worker`; }

const PYTHON_MODULE = String.raw`
import inspect, json, sys, types
from _robobuddy_sim_bridge import request as _js_request

_API_VERSION = "robobuddy.sim.v1"
_SHIM_FILE = "<robobuddy-sim-shim>"

def _source():
    frame = inspect.currentframe().f_back
    while frame and frame.f_code.co_filename == _SHIM_FILE:
        frame = frame.f_back
    return (frame.f_code.co_filename, int(frame.f_lineno)) if frame else ("main.py", 1)

async def _call(method, **kwargs):
    file, line = _source()
    try:
        result = await _js_request(method, json.dumps(kwargs), file, line)
        return json.loads(str(result))
    except Exception as error:
        raise RuntimeError(f"{method} failed: {error}") from error

class PhysicalRobot:
    """Browser-only async physical-simulation client. Angles are radians; time is simulation seconds."""
    def __init__(self, robot_id, connection):
        self.robot_id = robot_id
        self.connection = connection

    async def send_action(self, targets, max_steps=1000, duration_seconds=None):
        args = dict(targets=dict(targets), max_steps=int(max_steps))
        if duration_seconds is not None:
            args["duration_seconds"] = float(duration_seconds)
        return await _call("send_action", **args)

    async def advance(self, seconds):
        return await _call("advance", seconds=float(seconds))

    async def get_observation(self, view="ground_truth"):
        return await _call("get_observation", view=str(view))

    async def wait_for_goal(self, targets, tolerance_rad=0.02, timeout_seconds=2.0, controller_period_seconds=0.02):
        return await _call(
            "wait_for_goal",
            targets=dict(targets),
            tolerance_rad=float(tolerance_rad),
            timeout_seconds=float(timeout_seconds),
            controller_period_seconds=float(controller_period_seconds),
        )

    # --- free-base physical surface -----------------------------------------------------
    # set_joint_targets() is send_action() named for what it is: it latches a bounded low-level
    # command. It never means the joint reached the target, and it releases any engaged
    # standing controller. There is deliberately no walk(): walking is unsupported.
    async def set_joint_targets(self, targets, max_steps=1000):
        return await _call("send_action", targets=dict(targets), max_steps=int(max_steps))

    async def wait_sim(self, seconds):
        return await _call("advance", seconds=float(seconds))

    async def get_state(self):
        return await _call("get_state")

    async def stand(self, controller_id="robobuddy_g1_stand_v1", max_steps=200000):
        return await _call("engage_stand", controller_id=str(controller_id), max_steps=int(max_steps))

    async def release_stand(self):
        return await _call("release_stand")

    async def set_actuation(self, enabled):
        """Declared pre-trial setup. It removes actuator effort; it is logged and is not control."""
        return await _call("set_actuation", enabled=bool(enabled))

    async def pause(self): return await _call("pause")
    async def resume(self): return await _call("resume")
    async def reset(self): return await _call("reset")
    async def disconnect(self): return await _call("disconnect")

async def connect(robot_id="so101_follower"):
    connection = await _call("connect", robot_id=str(robot_id))
    return PhysicalRobot(str(robot_id), connection)

robobuddy = sys.modules.get("robobuddy") or types.ModuleType("robobuddy")
sim = types.ModuleType("robobuddy.sim")
sim.API_VERSION = _API_VERSION
sim.PhysicalRobot = PhysicalRobot
sim.connect = connect
sim.__doc__ = "Versioned browser physical-simulation API. SI units; no hardware transport."
robobuddy.sim = sim
sys.modules["robobuddy"] = robobuddy
sys.modules["robobuddy.sim"] = sim
`;