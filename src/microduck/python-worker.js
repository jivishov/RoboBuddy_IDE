const PYODIDE_VERSION = '0.29.4';
const PYODIDE_ROOT = `https://cdn.jsdelivr.net/pyodide/v${PYODIDE_VERSION}/full/`;
const WORKSPACE_FILES = Object.freeze(['main.py', 'trajectories.py', 'robot_config.py', 'workcell.py']);
const MAX_ERROR_CHARACTERS = 64 * 1024;
const pending = new Map();
let sequence = 0;
let active = null;

self.onmessage = (event) => {
  const message = event.data || {};
  if (message.type === 'run') { void runWorkspace(message); return; }
  if (message.type === 'bridge-response') {
    const entry = pending.get(message.requestId);
    if (!entry) return;
    pending.delete(message.requestId);
    if (message.ok) entry.resolve(JSON.stringify(message.result));
    else {
      const error = new Error(`[${message.error?.code || 'PYTHON_BRIDGE'}] ${message.error?.message || 'Simulator bridge request failed.'}`);
      error.code = message.error?.code || 'PYTHON_BRIDGE';
      entry.reject(error);
    }
    return;
  }
  if (message.type === 'cancel') {
    for (const [requestId, entry] of pending) {
      pending.delete(requestId);
      const error = new Error(`[${message.error?.code || 'OPERATION_CANCELLED'}] ${message.error?.message || 'MicroDuck Python run cancelled.'}`);
      error.code = message.error?.code || 'OPERATION_CANCELLED';
      entry.reject(error);
    }
  }
};

async function runWorkspace(message) {
  active = { runEpoch: message.runEpoch, workspaceEpoch: message.workspaceEpoch };
  try {
    const { loadPyodide } = await import(`${PYODIDE_ROOT}pyodide.mjs`);
    const pyodide = await loadPyodide({ indexURL: PYODIDE_ROOT });
    pyodide.setStdout({ batched: (text) => post('output', { stream: 'stdout', text: `${text}\n` }) });
    pyodide.setStderr({ batched: (text) => post('output', { stream: 'stderr', text: `${text}\n` }) });
    pyodide.registerJsModule('_microduck_bridge', { request: bridgeRequest });
    const names = Object.keys(message.files || {});
    if (names.length !== WORKSPACE_FILES.length || WORKSPACE_FILES.some((name) => !Object.hasOwn(message.files, name)) || names.some((name) => !WORKSPACE_FILES.includes(name))) throw new Error('MicroDuck worker received an invalid workspace file set.');
    for (const name of WORKSPACE_FILES) pyodide.FS.writeFile(name, String(message.files[name]));
    await pyodide.runPythonAsync(PYTHON_MODULE, { filename: '<microduck-shim>' });
    await pyodide.runPythonAsync('import sys\nfor _name in ("main", "trajectories", "robot_config", "workcell"):\n    sys.modules.pop(_name, None)', { filename: '<microduck-bootstrap>' });
    await pyodide.runPythonAsync(String(message.files?.['main.py'] || ''), { filename: 'main.py' });
    post('complete', { ok: true, sourceAttribution: { evaluator: 'Pyodide runPythonAsync', topLevelAwait: true, filename: 'main.py' } });
  } catch (error) {
    const text = clip(error?.message || error);
    const code = text.match(/\[([A-Z][A-Z0-9_]+)\]/)?.[1] || error?.code || 'PYTHON';
    post('complete', { ok: false, error: { code, message: text, stack: clip(error?.stack || '') } });
  }
}

function bridgeRequest(method, argsJson, file, line) {
  if (!active) return Promise.reject(new Error('No active MicroDuck run.'));
  const requestId = ++sequence;
  const promise = new Promise((resolve, reject) => pending.set(requestId, { resolve, reject }));
  post('bridge-request', { requestId, method: String(method), args: JSON.parse(String(argsJson || '{}')), source: { file: String(file), line: Number(line) } });
  return promise;
}

function post(type, extra = {}) { self.postMessage({ type, runEpoch: active?.runEpoch, workspaceEpoch: active?.workspaceEpoch, ...extra }); }
function clip(value) { const text = String(value); return text.length <= MAX_ERROR_CHARACTERS ? text : `${text.slice(0, MAX_ERROR_CHARACTERS)}\n…truncated by the MicroDuck worker`; }

const PYTHON_MODULE = String.raw`
import inspect, json, sys, types
from _microduck_bridge import request as _js_request

_SHIM_FILE = "<microduck-shim>"

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

class MicroDuck:
    """Async controller for the approximate browser-only MicroDuck policy simulation; never a physical SDK."""
    async def connect(self): return await _call("connect")
    async def disconnect(self): return await _call("disconnect")
    async def move(self, vx=0, vy=0, vyaw=0): return await _call("move", vx=vx, vy=vy, yaw=vyaw)
    async def head(self, neck_pitch=0, head_pitch=0, head_yaw=0, head_roll=0): return await _call("head", neckPitch=neck_pitch, headPitch=head_pitch, headYaw=head_yaw, headRoll=head_roll)
    async def look(self, x, y, z, neck_pitch=0): return await _call("look", x=x, y=y, z=z, neckPitch=neck_pitch)
    async def stop(self): return await _call("stop")
    async def enable(self, on=None): return await _call("enable", **({} if on is None else {"on": on}))
    async def init(self): return await _call("init")
    async def relax(self): return await _call("relax")
    async def do(self, skill): return await _call("do", skill=skill)
    async def pose(self, z=0, roll=0, pitch=0, active=True): return await _call("pose", z=z if active else 0, roll=roll if active else 0, pitch=pitch if active else 0)
    async def mouth(self, open): return await _call("mouth", open=open)
    async def sound(self, tag, hold=False): return await _call("sound", tag=tag, hold=hold)
    async def theremin(self, active): return await _call("theremin", active=active)
    async def chorale(self, active, piece=None, voices=1): return await _call("chorale", active=active, **({} if piece is None else {"piece": piece}), voices=voices)
    async def mode(self): return (await _call("mode"))["mode"]
    async def set_mode(self, mode): return await _call("set_mode", mode=mode)
    async def get_state(self): return (await _call("get_state"))["state"]
    async def set_color(self, color): return await _call("set_color", color=color)
    async def spawn_ball(self, position=None): return await _call("spawn_ball", **({} if position is None else {"position": position}))
    async def reset(self): return await _call("reset")
    async def set_tof_stimulus(self, distance=None): return await _call("set_tof_stimulus", distanceM=0.4 if distance is None else distance)
    async def set_camera(self, mode): return await _call("set_camera", camera=mode)
    async def sleep(self, seconds): return await _call("sleep", seconds=max(0, float(seconds)))

microduck = types.ModuleType("microduck")
microduck.MicroDuck = MicroDuck
microduck.__doc__ = "Browser-only approximate policy simulation API; no socket, discovery, physical control, or hardware parity."
sys.modules["microduck"] = microduck
`;
