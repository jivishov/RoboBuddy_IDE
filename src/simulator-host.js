import { SourceRobotSimulator } from './source-simulator.js';
import { MicroDuckPolicySimulator } from './microduck/policy-simulator.js';

export class SimulatorHost {
  constructor(canvas, {
    sourceFactory = (target) => new SourceRobotSimulator(target, { externalClock: true }),
    microduckFactory = (target) => new MicroDuckPolicySimulator(target, { externalClock: true }),
  } = {}) {
    this.canvas = canvas;
    this.epoch = 0;
    this.backend = null;
    this.pending = new Set();
    this.highContrast = true;
    this.sourceFactory = sourceFactory;
    this.microduckFactory = microduckFactory;
    this.controllerPreemptHandler = () => {};
    this.disposed = false;
    this.animationFrame = requestAnimationFrame((time) => this.renderFrame(time));
    this.syncLifecycleDiagnostics();
  }

  renderFrame(time) {
    if (this.disposed) return;
    this.animationFrame = requestAnimationFrame((next) => this.renderFrame(next));
    this.backend?.renderFrame?.(time);
  }

  syncLifecycleDiagnostics() {
    this.canvas.dataset.simulatorHostPendingCount = String(this.pending.size);
  }

  async setScenario(profileId, scenario, fallbackRest = {}) {
    const epoch = ++this.epoch;
    const previous = this.backend;
    this.backend = null;
    previous?.dispose?.();
    for (const pendingBackend of this.pending) pendingBackend.dispose?.();
    this.pending.clear();
    const backend = profileId === 'microduck' ? this.microduckFactory(this.canvas) : this.sourceFactory(this.canvas);
    backend.setControllerPreemptHandler?.(this.controllerPreemptHandler);
    this.pending.add(backend);
    this.syncLifecycleDiagnostics();
    backend.setHighContrastScene?.(this.highContrast);
    try { await backend.setScenario(profileId, scenario, fallbackRest); }
    catch (error) {
      this.pending.delete(backend);
      this.syncLifecycleDiagnostics();
      backend.dispose();
      if (epoch === this.epoch) throw error;
      return false;
    }
    this.pending.delete(backend);
    this.syncLifecycleDiagnostics();
    if (epoch !== this.epoch) { backend.dispose(); return false; }
    this.backend = backend;
    this.canvas.dataset.simulatorHostEpoch = String(epoch);
    return true;
  }
  async reset(...args) { return this.backend?.reset?.(...args); }
  setHighContrastScene(value) { this.highContrast = Boolean(value); return this.backend?.setHighContrastScene?.(this.highContrast) ?? this.highContrast; }
  isHighContrastSceneEnabled() { return this.backend?.isHighContrastSceneEnabled?.() ?? this.highContrast; }
  applyAction(...args) { return this.backend?.applyAction?.(...args); }
  advanceTime(...args) { return this.backend?.advanceTime?.(...args); }
  advanceBase(...args) { return this.backend?.advanceBase?.(...args); }
  getTelemetry() { return this.backend?.getTelemetry?.() || {}; }
  getContacts() { return this.backend?.getContacts?.() || {}; }
  fit() { return this.backend?.fit?.(); }
  resize() { return this.backend?.resize?.(); }
  setVariant(value) { return this.backend?.setVariant?.(value); }
  pause() { return this.backend?.pause?.() ?? false; }
  resume() { return this.backend?.resume?.() ?? false; }
  stop() { return this.backend?.stop?.() ?? false; }
  isReady() { return this.backend?.isReady?.() ?? false; }
  executeCommand(...args) { return this.backend?.executeCommand?.(...args); }
  abortCommand(...args) { return this.backend?.abortCommand?.(...args) ?? false; }
  isCommandComplete(...args) { return this.backend?.isCommandComplete?.(...args) ?? true; }
  isControllerActive(...args) { return this.backend?.isControllerActive?.(...args) ?? false; }
  unlockAudio(...args) { return this.backend?.unlockAudio?.(...args) ?? Promise.resolve(false); }
  releaseHumanIntent(...args) { return this.backend?.releaseHumanIntent?.(...args) ?? false; }
  cancelController(...args) { return this.backend?.cancelController?.(...args) ?? false; }
  acquireController(...args) { return this.backend?.acquireController?.(...args) ?? null; }
  refreshControllerLease(...args) { return this.backend?.refreshControllerLease?.(...args) ?? false; }
  setControllerPreemptHandler(handler) { this.controllerPreemptHandler = typeof handler === 'function' ? handler : () => {}; this.backend?.setControllerPreemptHandler?.(this.controllerPreemptHandler); }
  getState() { return this.backend?.getState?.() || null; }
  getEpoch() { return this.epoch; }
  perturb(...args) { return this.backend?.perturb?.(...args); }
  dispose() {
    if (this.disposed) return;
    this.disposed = true;
    if (this.animationFrame) cancelAnimationFrame(this.animationFrame);
    this.epoch += 1;
    this.backend?.dispose?.();
    this.backend = null;
    for (const pendingBackend of this.pending) pendingBackend.dispose?.();
    this.pending.clear();
    this.syncLifecycleDiagnostics();
  }
}
