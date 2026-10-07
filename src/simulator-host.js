import { AsimovPhysicalSimulator } from './physics/asimov-physical-simulator.js';
import { SourceRobotSimulator } from './source-simulator.js';
import { So101PhysicalSimulator } from './physics/so101-physical-simulator.js';
import { OpenArmPhysicalSimulator } from './physics/openarm-physical-simulator.js';
import { LeKiwiPhysicalSimulator } from './physics/lekiwi-physical-simulator.js';
import { MicroDuckPhysicalSimulator } from './physics/microduck-physical-simulator.js';
import { UnitreeG1PhysicalSimulator } from './physics/unitree-g1-physical-simulator.js';

export class SimulatorHost {
  constructor(canvas, {
    sourceFactory = (target) => new SourceRobotSimulator(target, { externalClock: true }),
    so101PhysicalFactory = (target) => new So101PhysicalSimulator(target),
    openarmPhysicalFactory = (target) => new OpenArmPhysicalSimulator(target),
    lekiwiPhysicalFactory = (target) => new LeKiwiPhysicalSimulator(target),
    microduckPhysicalFactory = (target) => new MicroDuckPhysicalSimulator(target),
    asimovPhysicalFactory = (target) => new AsimovPhysicalSimulator(target),
    unitreeG1PhysicalFactory = (target) => new UnitreeG1PhysicalSimulator(target),
  } = {}) {
    this.canvas = canvas;
    this.epoch = 0;
    this.profileId = null;
    this.backend = null;
    this.pending = new Set();
    this.highContrast = true;
    this.sourceFactory = sourceFactory;
    this.so101PhysicalFactory = so101PhysicalFactory;
    this.openarmPhysicalFactory = openarmPhysicalFactory;
    this.lekiwiPhysicalFactory = lekiwiPhysicalFactory;
    this.microduckPhysicalFactory = microduckPhysicalFactory;
    this.asimovPhysicalFactory = asimovPhysicalFactory;
    this.unitreeG1PhysicalFactory = unitreeG1PhysicalFactory;
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
  syncLifecycleDiagnostics() { this.canvas.dataset.simulatorHostPendingCount = String(this.pending.size); }
  async setScenario(profileId, scenario, fallbackRest = {}) {
    if (profileId === 'asimov' && scenario?.simulationMode !== 'physical_mujoco') throw new Error('Asimov is physical-only');
    if (profileId === 'microduck' && scenario?.simulationMode !== 'physical_mujoco') {
      throw new Error('MicroDuck supports physical tasks only; the legacy policy demonstrator has been retired.');
    }
    const epoch = ++this.epoch;
    const previous = this.backend;
    this.backend = null;
    this.profileId = null;
    previous?.dispose?.();
    for (const pendingBackend of this.pending) pendingBackend.dispose?.();
    this.pending.clear();
    const physical = scenario?.simulationMode === 'physical_mujoco';
    const backend = physical && profileId === 'asimov'
      ? this.asimovPhysicalFactory(this.canvas)
      : physical && profileId === 'microduck'
      ? this.microduckPhysicalFactory(this.canvas)
      : physical && profileId === 'so101'
      ? this.so101PhysicalFactory(this.canvas)
      : physical && profileId === 'openarm'
      ? this.openarmPhysicalFactory(this.canvas)
      : physical && profileId === 'lekiwi'
      ? this.lekiwiPhysicalFactory(this.canvas)
      : physical && profileId === 'unitree'
      ? this.unitreeG1PhysicalFactory(this.canvas)
      : this.sourceFactory(this.canvas);
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
    this.profileId = profileId;
    this.canvas.dataset.simulatorHostEpoch = String(epoch);
    if (profileId !== 'microduck') this.canvas.dataset.cameraView = 'front';
    return true;
  }
  async reset(...args) { return this.backend?.reset?.(...args); }
  setHighContrastScene(value) { this.highContrast = Boolean(value); return this.backend?.setHighContrastScene?.(this.highContrast) ?? this.highContrast; }
  isHighContrastSceneEnabled() { return this.backend?.isHighContrastSceneEnabled?.() ?? this.highContrast; }
  applyAction(...args) { return this.backend?.applyAction?.(...args); }
  applyPhysicalTargets(...args) { return this.backend?.applyPhysicalTargets?.(...args); }
  // Physical mobile-manipulation paths. These fail loudly rather than returning undefined:
  // the WebMCP and live-Python callers treat the result as an accepted command, so a missing
  // backend method must surface as an error instead of a silent no-op.
  applyChassisVelocity(...args) {
    if (typeof this.backend?.applyChassisVelocity !== 'function') throw new Error('The active simulator backend has no physical chassis-velocity path.');
    return this.backend.applyChassisVelocity(...args);
  }
  applyArmTargets(...args) {
    if (typeof this.backend?.applyArmTargets !== 'function') throw new Error('The active simulator backend has no physical arm-target path.');
    return this.backend.applyArmTargets(...args);
  }
  // Free-base physical paths. Like the mobile-manipulation paths above they fail loudly: the
  // WebMCP and live-Python callers treat the result as an accepted command, so a missing backend
  // method must surface as an error rather than a silent no-op.
  engageStand(...args) {
    if (typeof this.backend?.engageStand !== 'function') throw new Error('The active simulator backend has no verified standing controller.');
    return this.backend.engageStand(...args);
  }
  releaseStand(...args) {
    if (typeof this.backend?.releaseStand !== 'function') throw new Error('The active simulator backend has no standing controller to release.');
    return this.backend.releaseStand(...args);
  }
  setActuationEnabled(...args) {
    if (typeof this.backend?.setActuationEnabled !== 'function') throw new Error('The active simulator backend declares no actuation setup path.');
    return this.backend.setActuationEnabled(...args);
  }
  applyLowLevelCommands(...args) {
    if (typeof this.backend?.applyLowLevelCommands !== 'function') throw new Error('The active simulator backend has no low-level motor command path.');
    return this.backend.applyLowLevelCommands(...args);
  }
  getPresentationAudit() { return this.backend?.getPresentationAudit?.() || null; }
  getPresentationAlignment() { return this.backend?.getPresentationAlignment?.() || null; }
  advanceTime(...args) { return this.backend?.advanceTime?.(...args); }
  advanceBase(...args) { return this.backend?.advanceBase?.(...args); }
  getTelemetry() { return this.backend?.getTelemetry?.() || {}; }
  getContacts() { return this.backend?.getContacts?.() || {}; }
  getSensorObservation() {
    if(typeof this.backend?.getSensorObservation!=='function')throw new Error('No sensor sensitivity view on this backend');
    return this.backend.getSensorObservation();
  }
  getTaskEvaluation() { return this.backend?.getTaskEvaluation?.() || null; }
  getPhysicalSession() { return this.backend?.getPhysicalSession?.() || null; }
  getPhysicalAuthorityToken() { return this.backend?.getPhysicalAuthorityToken?.() || null; }
  fit() {
    const result = this.backend?.fit?.();
    if (this.profileId && this.profileId !== 'microduck') this.canvas.dataset.cameraView = 'front';
    return result;
  }
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
    this.profileId = null;
    this.backend?.dispose?.();
    this.backend = null;
    for (const pendingBackend of this.pending) pendingBackend.dispose?.();
    this.pending.clear();
    this.syncLifecycleDiagnostics();
  }
}