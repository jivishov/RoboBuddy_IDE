import { LIVE_SIM_API_VERSION } from './live-python-bridge.js';

const MAX_PAYLOAD_BYTES = 512 * 1024;
const MAX_OUTPUT_BYTES = 64 * 1024;
const DEFAULT_RUN_TIMEOUT_MS = 30_000;
const MAX_RUN_TIMEOUT_MS = 120_000;
const DEFAULT_COOPERATIVE_GRACE_MS = 150;

export class PhysicalPythonRuntime {
  constructor({
    bridgeFactory,
    workerFactory = () => new Worker(new URL('./live-python-worker.js', import.meta.url), { type: 'module', name: 'robobuddy-physical-python' }),
    onBoundary = () => {},
    onOutput = () => {},
    onState = () => {},
    runTimeoutMs = DEFAULT_RUN_TIMEOUT_MS,
    cooperativeGraceMs = DEFAULT_COOPERATIVE_GRACE_MS,
  } = {}) {
    if (typeof bridgeFactory !== 'function') throw new TypeError('PhysicalPythonRuntime requires bridgeFactory');
    this.bridgeFactory = bridgeFactory;
    this.workerFactory = workerFactory;
    this.onBoundary = onBoundary;
    this.onOutput = onOutput;
    this.onState = onState;
    this.runTimeoutMs = runTimeoutMs;
    this.cooperativeGraceMs = cooperativeGraceMs;
    this.runEpoch = 0;
    this.workerEpoch = 0;
    this.worker = null;
    this.active = null;
  }

  isActive() { return Boolean(this.active); }
  isPaused() { return Boolean(this.active?.externallyPaused); }
  getRunEpoch() { return this.active?.runEpoch ?? this.runEpoch; }

  start(files, { workspaceEpoch, robotId = '', runTimeoutMs = this.runTimeoutMs } = {}) {
    // A declared long physical trial may need more wall time on slow clients.
    // The limit is finite, independent of simulation time, and never renewed by progress.
    if (!Number.isInteger(runTimeoutMs) || runTimeoutMs < 1 || runTimeoutMs > MAX_RUN_TIMEOUT_MS) {
      return Promise.reject(runtimeError('INVALID_ARGUMENT', `Physical Python wall deadline must be 1..${MAX_RUN_TIMEOUT_MS} ms`));
    }
    void this.cancel('REPLACED_RUN', { silent: true, immediate: true });
    const workspace = boundedWorkspace(files);
    if (!Number.isInteger(workspaceEpoch) || workspaceEpoch < 0) return Promise.reject(runtimeError('INVALID_ARGUMENT', 'workspaceEpoch must be a non-negative integer'));
    const runEpoch = ++this.runEpoch;
    const workerEpoch = ++this.workerEpoch;
    const worker = this.workerFactory();
    const bridge = this.bridgeFactory();
    const completion = deferred();
    const active = {
      runEpoch, workerEpoch, workspaceEpoch, robotId: String(robotId || ''), worker, bridge, completion,
      connected: false, pendingRequestId: null, stdout: '', stderr: '', completed: false,
      externallyPaused: false, pauseWaiters: [],
    };
    this.worker = worker;
    this.active = active;
    worker.onmessage = (event) => { void this.#handleWorkerMessage(event.data || {}, active); };
    worker.onerror = (event) => this.#fail(active, runtimeError('PYTHON_WORKER', event.message || 'Physical Python worker failed'));
    active.timeout = setTimeout(() => {
      void this.cancel('PROGRAM_TIMEOUT', { error: runtimeError('PROGRAM_TIMEOUT', 'Physical Python run exceeded its bounded browser deadline') });
    }, runTimeoutMs);
    worker.postMessage(boundedClone({ type: 'run', runEpoch, workspaceEpoch, files: workspace }));
    this.onState({ state: 'running', runEpoch, apiVersion: LIVE_SIM_API_VERSION });
    return completion.promise;
  }

  async pause() {
    const active = this.active;
    if (!active || active.externallyPaused) return false;
    active.externallyPaused = true;
    try {
      if (active.connected) await active.bridge.pause();
    } catch (error) {
      active.externallyPaused = false;
      throw error;
    }
    this.onState({ state: 'paused', runEpoch: active.runEpoch, apiVersion: LIVE_SIM_API_VERSION });
    return true;
  }

  async resume() {
    const active = this.active;
    if (!active || !active.externallyPaused) return false;
    if (active.connected) await active.bridge.resume();
    active.externallyPaused = false;
    this.#releasePauseWaiters(active);
    this.onState({ state: 'running', runEpoch: active.runEpoch, apiVersion: LIVE_SIM_API_VERSION });
    return true;
  }

  async cancel(reason = 'OPERATION_CANCELLED', { error = null, silent = false, immediate = false } = {}) {
    const active = this.active;
    if (!active) return false;
    this.runEpoch += 1;
    this.active = null;
    clearTimeout(active.timeout);
    this.#releasePauseWaiters(active);
    const cancellation = error || runtimeError('OPERATION_CANCELLED', cancellationMessage(reason), { reason });
    try { await active.bridge.cancel(reason); } catch {}
    active.bridge.dispose?.();
    if (!silent) active.completion.reject(cancellation);
    else active.completion.resolve({ ok: false, cancelled: true, reason });
    const worker = active.worker;
    let forceTimer = null;
    if (worker && !immediate) {
      worker.onmessage = (event) => {
        if (event.data?.type !== 'complete' || event.data?.runEpoch !== active.runEpoch) return;
        clearTimeout(forceTimer);
        worker.terminate();
      };
    }
    try { worker?.postMessage({ type: 'cancel', runEpoch: active.runEpoch, workspaceEpoch: active.workspaceEpoch, reason }); } catch {}
    if (immediate) worker?.terminate();
    else forceTimer = setTimeout(() => worker?.terminate(), this.cooperativeGraceMs);
    if (this.worker === worker) this.worker = null;
    this.onState({ state: 'idle', reason, runEpoch: active.runEpoch });
    return true;
  }

  dispose() { return this.cancel('DISPOSED', { silent: true, immediate: true }); }

  async #handleWorkerMessage(message, active) {
    if (!this.#isCurrent(active) || message.runEpoch !== active.runEpoch || message.workspaceEpoch !== active.workspaceEpoch) return;
    if (message.type === 'output') {
      const text = String(message.text || '');
      if (message.stream === 'stderr') active.stderr += text;
      else active.stdout += text;
      if (byteLength(active.stdout) > MAX_OUTPUT_BYTES || byteLength(active.stderr) > MAX_OUTPUT_BYTES) {
        this.#fail(active, runtimeError('INVALID_ARGUMENT', `Physical Python output exceeds ${MAX_OUTPUT_BYTES} bytes per stream`));
        return;
      }
      this.onOutput({ stdout: active.stdout, stderr: active.stderr });
      return;
    }
    if (message.type === 'bridge-request') {
      if (active.pendingRequestId != null) {
        this.#fail(active, runtimeError('PYTHON_BRIDGE', 'Physical Python worker attempted concurrent unresolved simulator calls'));
        return;
      }
      active.pendingRequestId = message.requestId;
      const source = boundedSource(message.source);
      this.onBoundary(source, String(message.method || 'unknown'));
      try {
        const result = await this.#executeBoundary(active, String(message.method || ''), boundedClone(message.args || {}));
        if (!this.#isCurrent(active)) return;
        active.pendingRequestId = null;
        active.worker.postMessage(boundedClone({ type: 'bridge-response', runEpoch: active.runEpoch, workspaceEpoch: active.workspaceEpoch, requestId: message.requestId, ok: true, result }));
      } catch (error) {
        active.pendingRequestId = null;
        if (this.#isCurrent(active)) active.worker.postMessage({ type: 'bridge-response', runEpoch: active.runEpoch, workspaceEpoch: active.workspaceEpoch, requestId: message.requestId, ok: false, error: serializeError(error) });
      }
      return;
    }
    if (message.type === 'complete') {
      if (message.ok) this.#complete(active, { ok: true, stdout: active.stdout, stderr: active.stderr, sourceAttribution: message.sourceAttribution });
      else this.#fail(active, runtimeError(message.error?.code || 'PYTHON', message.error?.message || 'Physical Python execution failed', message.error || {}));
    }
  }

  async #executeBoundary(active, method, args) {
    if (!['pause', 'resume', 'disconnect'].includes(method)) await this.#waitIfExternallyPaused(active);
    switch (method) {
      case 'connect': {
        // The app-level runtime hint can be stale when a new physical robot is added.
        // The live bridge, which owns the current PhysicsSession diagnostics, is the
        // authority for robot identity and rejects any actual profile mismatch.
        const requested = String(args.robot_id || active.robotId || '');
        if (!requested) throw runtimeError('INVALID_ARGUMENT', 'connect() requires an explicit physical robot id');
        const result = await active.bridge.connect(requested);
        active.robotId = String(result?.robotId || requested);
        active.connected = true;
        if (active.externallyPaused) await active.bridge.pause();
        return result;
      }
      case 'disconnect':
        active.connected = false;
        return active.bridge.disconnect();
      case 'send_action':
        this.#assertConnected(active);
        return active.bridge.sendAction(args.targets, { maxSteps: Number(args.max_steps), ...(args.duration_seconds === undefined ? {} : { durationSeconds: args.duration_seconds }) });
      case 'advance':
        this.#assertConnected(active);
        return active.bridge.advance(Number(args.seconds));
      case 'get_observation':
        this.#assertConnected(active);
        return active.bridge.getObservation({ view: String(args.view || 'ground_truth') });
      case 'engage_stand':
        return active.bridge.engageStand({ controllerId: String(args.controller_id || ''), maxSteps: Number.isInteger(args.max_steps) ? args.max_steps : 200000 });
      case 'release_stand':
        return active.bridge.releaseStand();
      case 'get_state':
        return active.bridge.getState();
      case 'set_actuation':
        return active.bridge.applyDeclaredSetup({ type: 'set_actuation', enabled: Boolean(args.enabled) });
      case 'wait_for_goal':
        this.#assertConnected(active);
        return active.bridge.waitForGoal(args.targets, { toleranceRad: Number(args.tolerance_rad), timeoutSeconds: Number(args.timeout_seconds), controllerPeriodSeconds: Number(args.controller_period_seconds) });
      case 'pause':
        this.#assertConnected(active);
        return active.bridge.pause();
      case 'resume':
        this.#assertConnected(active);
        return active.bridge.resume();
      case 'reset':
        this.#assertConnected(active);
        return active.bridge.reset();
      default:
        throw runtimeError('INVALID_ARGUMENT', `Unknown physical Python API method: ${method}`);
    }
  }

  #waitIfExternallyPaused(active) {
    if (!active.externallyPaused) return Promise.resolve();
    return new Promise((resolve, reject) => active.pauseWaiters.push({ resolve, reject }));
  }
  #releasePauseWaiters(active, error = null) {
    for (const waiter of active.pauseWaiters || []) {
      if (error) waiter.reject(error);
      else waiter.resolve();
    }
    active.pauseWaiters = [];
  }
  #assertConnected(active) { if (!active.connected) throw runtimeError('SIMULATION_NOT_READY', 'Call await connect(...) before using the physical simulation API'); }
  #complete(active, result) {
    if (!this.#isCurrent(active)) return;
    this.active = null;
    clearTimeout(active.timeout);
    this.#releasePauseWaiters(active);
    active.completed = true;
    active.bridge.disconnect?.();
    active.bridge.dispose?.();
    active.worker?.terminate?.();
    if (this.worker === active.worker) this.worker = null;
    active.completion.resolve(result);
    this.onState({ state: 'idle', reason: 'complete', runEpoch: active.runEpoch });
  }
  #fail(active, error) {
    if (!this.#isCurrent(active)) return;
    this.active = null;
    clearTimeout(active.timeout);
    this.#releasePauseWaiters(active, error);
    void active.bridge.cancel?.(error.code || 'PYTHON_FAILURE').catch?.(() => {});
    active.bridge.dispose?.();
    active.worker?.terminate?.();
    if (this.worker === active.worker) this.worker = null;
    active.completion.reject(error);
    this.onState({ state: 'idle', reason: error.code || 'error', runEpoch: active.runEpoch });
  }
  #isCurrent(active) { return this.active === active && this.worker === active.worker; }
}

function boundedWorkspace(files) {
  if (!files || typeof files !== 'object' || Array.isArray(files) || !Object.hasOwn(files, 'main.py')) throw runtimeError('INVALID_ARGUMENT', 'Physical Python workspace requires main.py');
  const clone = boundedClone(files);
  return Object.fromEntries(Object.entries(clone).map(([name, value]) => [String(name), String(value)]));
}
function boundedSource(source) { return { file: String(source?.file || 'main.py').slice(0, 160), line: Math.max(1, Number(source?.line) || 1) }; }
function boundedClone(value) { const text = JSON.stringify(value); if (typeof text !== 'string' || byteLength(text) > MAX_PAYLOAD_BYTES) throw runtimeError('INVALID_ARGUMENT', `Physical Python bridge payload exceeds ${MAX_PAYLOAD_BYTES} bytes`); return JSON.parse(text); }
function byteLength(value) { return new TextEncoder().encode(String(value)).byteLength; }
function deferred() { let resolve; let reject; const promise = new Promise((yes, no) => { resolve = yes; reject = no; }); return { promise, resolve, reject }; }
function runtimeError(code, message, details = {}) { const error = new Error(message); error.code = code; Object.assign(error, details); return error; }
function serializeError(error) { return { code: error?.code || 'PYTHON_BRIDGE', message: String(error?.message || error).slice(0, 4096) }; }
function cancellationMessage(reason) { return ({ STOP: 'The physical Python run was stopped.', WORKSPACE_CHANGED: 'The physical workspace changed during execution.', PROFILE_CHANGED: 'The robot profile changed during execution.', RESET: 'The physical simulation reset cancelled the active Python run.', MANUAL_PREEMPTION: 'Manual control preempted the active Python run.', REPLACED_RUN: 'A newer physical Python run replaced this run.', PROGRAM_TIMEOUT: 'The physical Python run exceeded its deadline.' })[reason] || 'The physical Python operation was cancelled.'; }
