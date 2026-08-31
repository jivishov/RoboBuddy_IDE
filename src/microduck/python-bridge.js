import { MICRODUCK_COMMANDS } from './command-catalog.js';

export const MICRODUCK_PYTHON_METHODS = Object.freeze({
  connect: null,
  disconnect: null,
  move: 'move',
  head: 'head',
  look: 'look',
  stop: 'stop',
  enable: 'enable',
  init: 'init',
  relax: 'relax',
  do: 'do',
  pose: 'pose',
  mouth: 'mouth',
  sound: 'sound',
  theremin: 'theremin',
  chorale: 'chorale',
  mode: 'get_mode',
  set_mode: 'set_mode',
  get_state: 'get_state',
  set_color: 'set_color',
  spawn_ball: 'spawn_ball',
  reset: 'reset',
  set_tof_stimulus: 'set_tof_stimulus',
  set_camera: 'set_camera',
  sleep: null,
});

const MAX_PAYLOAD_BYTES = 256 * 1024;
const MAX_OUTPUT_BYTES = 64 * 1024;
const CONTROL_TICK_SECONDS = 0.02;
const WORKSPACE_FILES = Object.freeze(['main.py', 'trajectories.py', 'robot_config.py', 'workcell.py']);

export class MicroDuckPythonBridge {
  constructor({
    simulator,
    workerFactory = () => new Worker(new URL('./python-worker.js', import.meta.url), { type: 'module' }),
    onBoundary = () => {},
    onOutput = () => {},
    onState = () => {},
    cooperativeGraceMs = 250,
    runTimeoutMs = 30_000,
  } = {}) {
    this.simulator = simulator;
    this.workerFactory = workerFactory;
    this.onBoundary = onBoundary;
    this.onOutput = onOutput;
    this.onState = onState;
    this.cooperativeGraceMs = cooperativeGraceMs;
    this.runTimeoutMs = runTimeoutMs;
    this.runEpoch = 0;
    this.workerEpoch = 0;
    this.worker = null;
    this.active = null;
  }

  isActive() { return Boolean(this.active); }
  isPaused() { return Boolean(this.active?.paused); }

  start(files, { workspaceEpoch, mode = 'run', cursor = null } = {}) {
    void this.cancel('REPLACED_RUN', { silent: true, immediate: true });
    let workspace;
    try { workspace = boundedWorkspace(files); }
    catch (error) { return Promise.reject(error); }
    const runEpoch = ++this.runEpoch;
    const workerEpoch = ++this.workerEpoch;
    const worker = this.workerFactory();
    this.worker = worker;
    const completion = deferred();
    const active = {
      runEpoch,
      workerEpoch,
      workspaceEpoch,
      controllerId: `python-run-${runEpoch}`,
      mode,
      cursor,
      connected: false,
      paused: mode === 'step',
      pending: null,
      stepBudget: 0,
      stepWaiters: [],
      completion,
      worker,
      completed: false,
      stdout: '',
      stderr: '',
    };
    this.active = active;
    worker.onmessage = (event) => { void this.handleWorkerMessage(event.data, active); };
    worker.onerror = (event) => this.fail(active, bridgeError('PYTHON_WORKER', event.message || 'MicroDuck Python worker failed.'));
    active.timeout = setTimeout(() => { void this.cancel('POLICY_TIMEOUT', { error: bridgeError('POLICY_TIMEOUT', 'MicroDuck Python run exceeded its 30 second browser limit.') }); }, this.runTimeoutMs);
    worker.postMessage(boundedClone({ type: 'run', runEpoch, workspaceEpoch, files: workspace }));
    this.onState({ state: mode === 'step' ? 'paused' : 'running', runEpoch });
    return completion.promise;
  }

  pause() {
    if (!this.active || this.active.paused) return false;
    this.active.paused = true;
    this.simulator.pause();
    this.onState({ state: 'paused', runEpoch: this.active.runEpoch });
    return true;
  }

  resume() {
    if (!this.active || !this.active.paused) return false;
    this.active.paused = false;
    this.simulator.resume();
    this.onState({ state: 'running', runEpoch: this.active.runEpoch });
    void this.flushPending(this.active);
    return true;
  }

  async step() {
    const active = this.active;
    if (!active) throw bridgeError('OPERATION_CANCELLED', 'No MicroDuck Python run is available to step.');
    active.paused = true;
    this.simulator.pause();
    active.stepBudget += 1;
    const waiter = deferred();
    active.stepWaiters.push(waiter);
    void this.flushPending(active);
    return waiter.promise;
  }

  async handleWorkerMessage(message, active) {
    if (!this.isCurrent(active) || message.runEpoch !== active.runEpoch || message.workspaceEpoch !== active.workspaceEpoch) return;
    if (message.type === 'bridge-request') {
      if (active.pending) return this.fail(active, bridgeError('PYTHON_BRIDGE', 'The worker attempted more than one unresolved simulator request.'));
      try {
        active.pending = boundedClone({ ...message, source: boundedSource(message.source) });
      } catch (error) {
        this.fail(active, error);
        return;
      }
      this.onBoundary(active.pending.source, active.pending.method);
      await this.flushPending(active);
      return;
    }
    if (message.type === 'output') {
      const text = String(message.text || '');
      const nextStdout = active.stdout + (message.stream === 'stdout' ? text : '');
      const nextStderr = active.stderr + (message.stream === 'stderr' ? text : '');
      if (byteLength(nextStdout) > MAX_OUTPUT_BYTES || byteLength(nextStderr) > MAX_OUTPUT_BYTES) {
        this.fail(active, bridgeError('INVALID_ARGUMENT', `MicroDuck Python output exceeds ${MAX_OUTPUT_BYTES} bytes per stream.`));
        return;
      }
      active.stdout = nextStdout;
      active.stderr = nextStderr;
      this.onOutput({ stdout: active.stdout, stderr: active.stderr });
      return;
    }
    if (message.type === 'complete') {
      if (message.ok) this.complete(active, { ok: true, stdout: active.stdout, stderr: active.stderr, sourceAttribution: message.sourceAttribution });
      else this.fail(active, bridgeError(message.error?.code || 'PYTHON', message.error?.message || 'MicroDuck Python execution failed.', message.error));
    }
  }

  async flushPending(active) {
    if (!this.isCurrent(active) || !active.pending) return;
    const request = active.pending;
    const stepping = active.paused && active.stepBudget > 0;
    if (active.paused && !stepping) return;
    if (stepping) active.stepBudget -= 1;
    try {
      let result;
      if (request.completedResult !== undefined) result = request.completedResult;
      else if (request.method === 'sleep') result = await this.executeSleepBoundary(active, request, stepping);
      else result = await this.executeCommandBoundary(active, request);
      if (result?.partialSleep) {
        this.resolveStepWaiter(active, { source: request.source, method: 'sleep', remainingSeconds: request.remainingSeconds });
        return;
      }
      if (active.paused && !stepping) {
        request.completedResult = result;
        return;
      }
      active.pending = null;
      if (!this.isCurrent(active)) return;
      this.worker.postMessage(boundedClone({ type: 'bridge-response', runEpoch: active.runEpoch, workspaceEpoch: active.workspaceEpoch, requestId: request.requestId, ok: true, result }));
      this.resolveStepWaiter(active, { source: request.source, method: request.method, result });
      if (active.mode === 'cursor' && cursorReached(request.source, active.cursor)) {
        await this.cancel('RUN_TO_CURSOR', { result: { ok: true, cursor: request.source, stdout: active.stdout, stderr: active.stderr } });
      }
    } catch (error) {
      active.pending = null;
      if (this.isCurrent(active)) this.worker.postMessage({ type: 'bridge-response', runEpoch: active.runEpoch, workspaceEpoch: active.workspaceEpoch, requestId: request.requestId, ok: false, error: serializeError(error) });
      this.rejectStepWaiter(active, error);
      if (['COMMAND_CONFLICT', 'OPERATION_CANCELLED'].includes(error.code)) await this.cancel(error.code, { error });
    }
  }

  async executeCommandBoundary(active, request) {
    const command = MICRODUCK_PYTHON_METHODS[request.method];
    if (!(request.method in MICRODUCK_PYTHON_METHODS)) throw bridgeError('INVALID_ARGUMENT', `Unknown MicroDuck Python method: ${request.method}`);
    if (request.method === 'connect') {
      if (active.connected) {
        if (!this.simulator.refreshControllerLease('python', active.controllerId, 5000)) this.simulator.acquireController('python', active.controllerId, 5000);
        return { ok: true, connected: true, simulation: true };
      }
      this.simulator.acquireController('python', active.controllerId, 5000);
      active.connected = true;
      return { ok: true, connected: true, simulation: true, transport: 'browser-only' };
    }
    if (request.method === 'disconnect') {
      this.simulator.cancelController('python', active.controllerId);
      active.connected = false;
      return { ok: true, connected: false };
    }
    if (!active.connected) throw bridgeError('SIMULATION_BUSY', 'Call await robot.connect() before using the simulated MicroDuck SDK.');
    if (!MICRODUCK_COMMANDS[command]) throw bridgeError('INVALID_ARGUMENT', `${request.method} is not catalog-backed.`);
    if (!this.simulator.refreshControllerLease('python', active.controllerId, 5000)) this.simulator.acquireController('python', active.controllerId, 5000);
    const result = await this.simulator.executeCommand(command, boundedClone(request.args || {}), { source: 'python', controllerId: active.controllerId, durationMs: 5000 });
    return boundedClone(result);
  }

  async executeSleepBoundary(active, request, stepping) {
    if (!active.connected) throw bridgeError('SIMULATION_BUSY', 'Call await robot.connect() before using cooperative sleep().');
    if (request.remainingSeconds === undefined) request.remainingSeconds = Math.max(0, Number(request.args?.seconds) || 0);
    if (stepping) {
      const tick = Math.min(CONTROL_TICK_SECONDS, request.remainingSeconds);
      if (tick > 0) await this.simulator.advanceTime(tick, { controlStep: true });
      request.remainingSeconds = Math.max(0, request.remainingSeconds - tick);
      if (request.remainingSeconds > 1e-9) return { partialSleep: true };
      return { ok: true, sleptSeconds: Number(request.args?.seconds) || 0 };
    }
    while (request.remainingSeconds > 1e-9 && this.isCurrent(active) && !active.paused) {
      const tick = Math.min(CONTROL_TICK_SECONDS, request.remainingSeconds);
      await delay(tick * 1000);
      request.remainingSeconds = Math.max(0, request.remainingSeconds - tick);
    }
    if (active.paused && request.remainingSeconds > 1e-9) return { partialSleep: true };
    return { ok: true, sleptSeconds: Number(request.args?.seconds) || 0 };
  }

  async cancel(reason = 'OPERATION_CANCELLED', { error = null, result = null, silent = false, immediate = false } = {}) {
    const active = this.active;
    if (!active) return false;
    this.runEpoch += 1;
    this.active = null;
    clearTimeout(active.timeout);
    this.simulator.cancelController('python', active.controllerId);
    this.simulator.stop();
    const cancellation = error || bridgeError(reason === 'POLICY_TIMEOUT' ? 'POLICY_TIMEOUT' : 'OPERATION_CANCELLED', cancellationMessage(reason), { reason });
    for (const waiter of active.stepWaiters.splice(0)) waiter.reject(cancellation);
    if (result) active.completion.resolve(result);
    else if (!silent) active.completion.reject(cancellation);
    else active.completion.resolve({ ok: false, cancelled: true, reason });
    const worker = this.worker;
    let forceTimer = null;
    if (worker && !immediate) {
      worker.onmessage = (event) => {
        if (event.data?.type !== 'complete' || event.data?.runEpoch !== active.runEpoch) return;
        clearTimeout(forceTimer);
        worker.terminate();
      };
    }
    try { worker?.postMessage({ type: 'cancel', runEpoch: active.runEpoch, workspaceEpoch: active.workspaceEpoch, error: serializeError(cancellation) }); } catch {}
    if (immediate) worker?.terminate();
    else forceTimer = setTimeout(() => worker?.terminate(), this.cooperativeGraceMs);
    if (this.worker === worker) this.worker = null;
    this.onState({ state: 'idle', reason });
    return true;
  }

  complete(active, result) {
    if (!this.isCurrent(active)) return;
    this.active = null;
    clearTimeout(active.timeout);
    this.simulator.cancelController('python', active.controllerId);
    active.completed = true;
    active.worker?.terminate?.();
    if (this.worker === active.worker) this.worker = null;
    for (const waiter of active.stepWaiters.splice(0)) waiter.resolve({ complete: true });
    active.completion.resolve(result);
    this.onState({ state: 'idle', reason: 'complete' });
  }

  fail(active, error) {
    if (!this.isCurrent(active)) return;
    this.active = null;
    clearTimeout(active.timeout);
    this.simulator.cancelController('python', active.controllerId);
    this.simulator.stop();
    active.worker?.terminate?.();
    if (this.worker === active.worker) this.worker = null;
    for (const waiter of active.stepWaiters.splice(0)) waiter.reject(error);
    active.completion.reject(error);
    this.onState({ state: 'idle', reason: error.code || 'error' });
  }

  resolveStepWaiter(active, value) { active.stepWaiters.shift()?.resolve(value); }
  rejectStepWaiter(active, error) { active.stepWaiters.shift()?.reject(error); }
  isCurrent(active) { return this.active === active && this.worker === active.worker; }
}

function cursorReached(source, cursor) { return Boolean(cursor && source?.file === cursor.file && Number(source.line) >= Number(cursor.line)); }
function delay(ms) { return new Promise((resolve) => setTimeout(resolve, ms)); }
function deferred() { let resolve; let reject; const promise = new Promise((yes, no) => { resolve = yes; reject = no; }); return { promise, resolve, reject }; }
function cancellationMessage(reason) { return ({ MANUAL_PREEMPTION: 'A trusted manual command preempted the Python simulation lease.', WORKSPACE_CHANGED: 'The MicroDuck workspace changed during execution.', PROFILE_CHANGED: 'The robot profile changed during execution.', RESET: 'The MicroDuck simulation was reset.', STOP: 'The MicroDuck Python run was stopped.' })[reason] || 'The MicroDuck Python operation was cancelled.'; }
function bridgeError(code, message, details = {}) { const error = new Error(message); error.code = code; Object.assign(error, details); return error; }
function serializeError(error) { return { code: error?.code || 'PYTHON_BRIDGE', message: String(error?.message || error), stack: String(error?.stack || '') }; }
function byteLength(value) { return new TextEncoder().encode(String(value)).byteLength; }
function boundedClone(value) { const text = JSON.stringify(value); if (typeof text !== 'string' || byteLength(text) > MAX_PAYLOAD_BYTES) throw bridgeError('INVALID_ARGUMENT', `MicroDuck bridge payload exceeds ${MAX_PAYLOAD_BYTES} bytes.`); return JSON.parse(text); }
function boundedWorkspace(files) {
  const names = files && typeof files === 'object' && !Array.isArray(files) ? Object.keys(files) : [];
  if (names.length !== WORKSPACE_FILES.length || WORKSPACE_FILES.some((name) => !Object.hasOwn(files, name)) || names.some((name) => !WORKSPACE_FILES.includes(name))) throw bridgeError('INVALID_ARGUMENT', `MicroDuck workspaces must contain exactly ${WORKSPACE_FILES.join(', ')}.`);
  return boundedClone(Object.fromEntries(WORKSPACE_FILES.map((name) => [name, String(files[name])])));
}
function boundedSource(source) {
  const file = String(source?.file || '');
  const line = Number(source?.line);
  if (!WORKSPACE_FILES.includes(file) || !Number.isInteger(line) || line < 1) throw bridgeError('INVALID_ARGUMENT', 'MicroDuck bridge source attribution must reference one of the four workspace files and a positive line.');
  return { file, line };
}
