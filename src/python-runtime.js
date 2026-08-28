const SHIM = String.raw`
import sys, types, inspect, json, io, contextlib, time as _real_time
_rb_events = []
_rb_state = {}
_rb_connected = False
for _user_module in ("trajectories", "robot_config", "workcell"):
    sys.modules.pop(_user_module, None)

def _src(frame):
    return {"file": frame.f_code.co_filename, "line": int(frame.f_lineno)}

def _record(kind, payload, frame):
    item = {"kind": kind, **_src(frame)}
    item.update(payload)
    _rb_events.append(item)

def _sleep(seconds):
    frame = inspect.currentframe().f_back
    value = max(0.0, float(seconds))
    _record("sleep", {"seconds": value}, frame)
_real_time.sleep = _sleep

def _config_class(name):
    class Config:
        def __init__(self, *args, **kwargs):
            self.args = args
            for k, v in kwargs.items(): setattr(self, k, v)
        def __repr__(self): return f"{name}({self.__dict__!r})"
    Config.__name__ = name
    return Config

class _Robot:
    def __init__(self, config=None): self.config = config
    def connect(self):
        global _rb_connected
        _rb_connected = True
        _record("connect", {}, inspect.currentframe().f_back)
    def disconnect(self):
        global _rb_connected
        _record("disconnect", {}, inspect.currentframe().f_back)
        _rb_connected = False
    def send_action(self, action):
        if not _rb_connected: raise RuntimeError("Robot is not connected")
        clean = {str(k): float(v) for k, v in dict(action).items()}
        _rb_state.update(clean)
        _record("send_action", {"action": clean}, inspect.currentframe().f_back)
        return dict(clean)
    def get_observation(self):
        if not _rb_connected: raise RuntimeError("Robot is not connected")
        _record("get_observation", {}, inspect.currentframe().f_back)
        return dict(_rb_state)

class _LeKiwiRobot(_Robot):
    _order = ("arm_shoulder_pan.pos","arm_shoulder_lift.pos","arm_elbow_flex.pos","arm_wrist_flex.pos","arm_wrist_roll.pos","arm_gripper.pos","x.vel","y.vel","theta.vel")
    def send_action(self, action):
        clean = super().send_action(action)
        vec = [float(clean.get(k, 0.0)) for k in self._order]
        sent = {k: vec[i] for i, k in enumerate(self._order)}
        sent["action"] = vec
        return sent

lerobot = types.ModuleType("lerobot")
robots = types.ModuleType("lerobot.robots")
so = types.ModuleType("lerobot.robots.so_follower")
so.SO101FollowerConfig = _config_class("SO101FollowerConfig")
so.SO101Follower = type("SO101Follower", (_Robot,), {})
openarm = types.ModuleType("lerobot.robots.openarm_follower")
openarm.OpenArmFollowerConfigBase = _config_class("OpenArmFollowerConfigBase")
openarm.OpenArmFollowerConfig = _config_class("OpenArmFollowerConfig")
openarm.OpenArmFollower = type("OpenArmFollower", (_Robot,), {})
bi = types.ModuleType("lerobot.robots.bi_openarm_follower")
bi.BiOpenArmFollowerConfig = _config_class("BiOpenArmFollowerConfig")
bi.BiOpenArmFollower = type("BiOpenArmFollower", (_Robot,), {})
lekiwi = types.ModuleType("lerobot.robots.lekiwi")
lekiwi.LeKiwiClientConfig = _config_class("LeKiwiClientConfig")
lekiwi.LeKiwiClient = type("LeKiwiClient", (_LeKiwiRobot,), {})
for name, mod in [("lerobot",lerobot),("lerobot.robots",robots),("lerobot.robots.so_follower",so),("lerobot.robots.openarm_follower",openarm),("lerobot.robots.bi_openarm_follower",bi),("lerobot.robots.lekiwi",lekiwi)]:
    sys.modules[name] = mod
`;

export class PythonRuntime {
  constructor(){this.pyodide=null;this.loading=null;}
  async ensure(){
    if(this.pyodide) return this.pyodide;
    if(this.loading) return this.loading;
    if(!window.loadPyodide) throw new Error('Pyodide loader failed to load.');
    this.loading=window.loadPyodide({indexURL:'https://cdn.jsdelivr.net/pyodide/v0.29.4/full/'}).then(p=>{this.pyodide=p;return p;});
    return this.loading;
  }
  async compileWorkspace(files){
    const py=await this.ensure();
    for(const [name,content] of Object.entries(files)) py.FS.writeFile(name,String(content));
    const runner = `${SHIM}\n_out = io.StringIO()\n_err = io.StringIO()\n_exc = None\nwith contextlib.redirect_stdout(_out), contextlib.redirect_stderr(_err):\n    try:\n        _g = {"__name__": "__main__", "__file__": "main.py"}\n        exec(compile(open("main.py", encoding="utf-8").read(), "main.py", "exec"), _g, _g)\n    except Exception as _e:\n        import traceback\n        _exc = traceback.format_exc()\n_result_json = json.dumps({"events": _rb_events, "stdout": _out.getvalue(), "stderr": _err.getvalue(), "exception": _exc})`;
    await py.runPythonAsync(runner);
    const text=py.globals.get('_result_json');
    return JSON.parse(String(text));
  }
}
