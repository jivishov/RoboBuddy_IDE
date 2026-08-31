import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { MicroDuckCommandBus } from './command-bus.js';
import { MICRODUCK_COMMANDS, commandError } from './command-catalog.js';
import { HOME_POLICY_POSITION, PHYSICS_TIMESTEP_SECONDS, POLICY_DECIMATION, policyToWire } from './contract.js';
import { MicroDuckMujocoDynamics } from './mujoco-dynamics.js';
import { buildObservation } from './observation.js';
import { InferenceGate } from './inference-gate.js';
import { MicroDuckPolicyDirector } from './policy-director.js';
import { MicroDuckPolicyRuntime } from './policy-runtime.js';
import { MicroDuckRigAdapter } from './rig-adapter.js';
import { createMicroDuckState } from './state.js';
import { MicroDuckAudioEngine } from './audio-engine.js';
import { createModeledTof, deriveFrameAngularVelocity, deriveModeledImu } from './peripherals.js';
import { MICRODUCK_FIELD_SIZE_M } from './field-bounds.js';

const RIG_URL = './assets/microduck/generated/procedural-rig.json';
const MAX_PHYSICS_STEPS_PER_FRAME = 4;
// The pinned site frame uses the runtime camera convention. This fixed basis
// maps it to Three.js camera axes (-Z forward, +Y up) without changing the
// source-authored head pose. At neutral: rendered forward is +X and up is +Y.
const CAMERA_FROM_SITE_FRAME = new THREE.Quaternion(-0.5, 0.5, 0.5, -0.5).normalize();
const DEFAULT_CAMERA_FOV_DEG = 44;
// A wide vertical projection keeps floor/ball context readable in the IDE's
// intentionally narrow simulator pane. Pose still comes only from head_camera.
const HEAD_CAMERA_FOV_DEG = 100;
const OVERVIEW_DIRECTION_WIDE = new THREE.Vector3(1.55, 0.78, 1.55).normalize();
const OVERVIEW_DIRECTION_TALL = new THREE.Vector3(2.8, 1.05, 0.62).normalize();
const WORLD_UP = new THREE.Vector3(0, 1, 0);

export class MicroDuckPolicySimulator {
  constructor(canvas, { externalClock = false } = {}) {
    this.canvas = canvas;
    this.externalClock = externalClock;
    this.disposed = false;
    this.paused = false;
    this.lifecycle = 'loading';
    this.animationFrame = 0;
    this.runEpoch = 0;
    this.requestSequence = 0;
    this.inferenceInFlight = false;
    this.inferenceGate = new InferenceGate();
    this.accumulator = 0;
    this.controlTicks = 0;
    this.missedTicks = 0;
    this.mode = 'walking';
    this.enabled = false;
    this.actuationEnabled = true;
    this.mouth = 0;
    this.color = 'cream';
    this.cameraMode = 'orbit';
    this.followDistance = 420;
    this.lastCameraUpdateTime = null;
    this.audioEngine = new MicroDuckAudioEngine();
    this.tofStimulus = { distanceM: 0.4, source: 'synthetic', sequence: 0 };
    this.tof = createModeledTof(this.tofStimulus);
    this.recovery = { stage: 'none', tiltedFor: 0, uprightFor: 0, startedAt: 0, resetFallback: false };
    this.forcedPerturbation = false;
    this.lastMovement = { requested: [0, 0, 0], applied: [0, 0, 0], limitedBy: [] };
    this.targets = new Float32Array(14);
    this.initRamp = null;
    this.webmcpOperation = null;
    this.webmcpOutcomes = new Map();
    this.controllerPreemptHandler = () => {};
    this.commandBus = new MicroDuckCommandBus({
      onPreempt: (previous, replacement) => {
        this.releaseOwnedAudio(previous.owned);
        if (previous.source === 'webmcp') this.markWebmcpOperationCancelled(previous.controllerId);
        this.controllerPreemptHandler(previous, replacement);
      },
      onExpire: (expired) => {
        this.releaseOwnedAudio(expired.owned);
        if (expired.source === 'webmcp') this.markWebmcpOperationCancelled(expired.controllerId);
      },
    });
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xb9c1c4);
    this.camera = new THREE.PerspectiveCamera(DEFAULT_CAMERA_FOV_DEG, 1, 1, 5000);
    this.cameraProbe = new THREE.PerspectiveCamera(DEFAULT_CAMERA_FOV_DEG, 1, 1, 5000);
    this.cameraBoundsScratch = new THREE.Box3();
    this.cameraBallBoundsScratch = new THREE.Box3();
    this.cameraCenterScratch = new THREE.Vector3();
    this.cameraDirectionScratch = new THREE.Vector3();
    this.cameraDesiredPosition = new THREE.Vector3();
    this.cameraFollowTarget = new THREE.Vector3();
    this.overviewTrackingCenter = new THREE.Vector3();
    this.robotForwardScratch = new THREE.Vector3();
    this.robotSideScratch = new THREE.Vector3();
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.shadowMap.enabled = true;
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x334155, 1.5));
    const key = new THREE.DirectionalLight(0xffffff, 2.1); key.position.set(-600, 900, 600); this.scene.add(key);
    const fieldSizeMm = MICRODUCK_FIELD_SIZE_M * 1000;
    this.floor = new THREE.Mesh(new THREE.PlaneGeometry(fieldSizeMm, fieldSizeMm), new THREE.MeshStandardMaterial({ color: 0x687378, roughness: 0.95 }));
    this.floor.rotation.x = -Math.PI / 2; this.floor.receiveShadow = true; this.scene.add(this.floor);
    this.fieldBoundary = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.PlaneGeometry(fieldSizeMm, fieldSizeMm)), new THREE.LineBasicMaterial({ color: 0xe8f0f2 }));
    this.fieldBoundary.rotation.x = -Math.PI / 2; this.fieldBoundary.position.y = 1; this.scene.add(this.fieldBoundary);
    this.ballMesh = new THREE.Mesh(new THREE.SphereGeometry(35, 24, 16), new THREE.MeshStandardMaterial({ color: 0xe34d2f, roughness: 0.72 }));
    this.ballMesh.castShadow = true; this.scene.add(this.ballMesh);
    this.tofRaycaster = new THREE.Raycaster();
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(canvas.parentElement || canvas);
    this.canvas.dataset.simulatorBackend = 'microduck-policy-sim';
    this.canvas.dataset.simulationClockS = '0';
    this.canvas.dataset.cameraView = this.cameraMode;
    this.resize();
    if (!externalClock) this.animate();
  }

  async setScenario(_profileId, scenario) {
    if (this.disposed) return false;
    const epoch = ++this.runEpoch;
    this.lifecycle = 'loading';
    this.scenario = scenario;
    this.rig?.dispose(); this.rig = null;
    this.dynamics?.dispose(); this.dynamics = null;
    await this.policyRuntime?.dispose?.();
    const rigResponse = await fetch(RIG_URL, { cache: 'force-cache' });
    if (!rigResponse.ok) throw new Error(`MicroDuck rig contract returned HTTP ${rigResponse.status}.`);
    const rigData = await rigResponse.json();
    const policyRuntime = new MicroDuckPolicyRuntime();
    const [rig, dynamics] = await Promise.all([MicroDuckRigAdapter.load(), MicroDuckMujocoDynamics.create(rigData), policyRuntime.initialize().then(() => policyRuntime)]);
    if (this.disposed || epoch !== this.runEpoch) { rig.dispose(); dynamics.dispose(); await policyRuntime.dispose(); return false; }
    this.rig = rig; this.dynamics = dynamics; this.policyRuntime = policyRuntime;
    this.director = new MicroDuckPolicyDirector({ jointRanges: Object.fromEntries(rigData.joints.map((joint) => [joint.name, joint.rangeRad])) });
    this.scene.add(this.rig.root);
    this.cameraMode = 'orbit';
    this.resetRuntimeState(scenario?.variant || 'walking');
    this.lifecycle = 'ready';
    this.fit('orbit');
    return true;
  }

  async reset(_profileId = 'microduck', scenario = this.scenario) {
    if (!this.isReady()) return false;
    this.abortTrackedWebmcpOperation('reset');
    this.runEpoch += 1;
    const cameraMode = this.cameraMode;
    this.resetRuntimeState(scenario?.variant || this.mode);
    this.fit(cameraMode);
    return true;
  }

  resetRuntimeState(mode = 'walking', { enabled = false } = {}) {
    this.mode = mode === 'roller' ? 'roller' : 'walking';
    this.director.reset(this.mode);
    this.commandBus.cancel();
    this.dynamics.reset();
    this.targets = Float32Array.from(HOME_POLICY_POSITION);
    this.mouth = 0; this.enabled = Boolean(enabled); this.actuationEnabled = true; this.paused = false; this.initRamp = null;
    this.inferenceInFlight = false; this.requestSequence += 1;
    this.inferenceGate.invalidate();
    this.accumulator = 0; this.controlTicks = 0; this.lastFrameTime = null;
    this.recovery = { stage: 'none', tiltedFor: 0, uprightFor: 0, startedAt: 0, resetFallback: false };
    this.forcedPerturbation = false;
    this.audioEngine.stopAll();
    this.tofStimulus.sequence = 0;
    this.rig.setVariant(this.mode); this.canvas.dataset.microduckVariant = this.mode;
    const physics = this.dynamics.snapshot();
    this.syncVisuals(physics);
    this.updateModeledPeripherals(physics, true);
    this.canvas.dataset.simulationClockS = physics.time.toFixed(3);
    this.canvas.dataset.microduckMotionState = this.enabled ? 'active' : 'holding';
  }

  setHighContrastScene(enabled) { this.canvas.dataset.highContrastScene = String(Boolean(enabled)); return Boolean(enabled); }
  isHighContrastSceneEnabled() { return this.canvas.dataset.highContrastScene !== 'false'; }
  async setVariant(variant) { return (await this.executeCommand('set_mode', { mode: variant }, { source: 'human' })).applied.mode; }

  rememberWebmcpOutcome(controllerId, status) {
    if (!controllerId) return;
    this.webmcpOutcomes.delete(controllerId);
    this.webmcpOutcomes.set(controllerId, status);
    while (this.webmcpOutcomes.size > 32) this.webmcpOutcomes.delete(this.webmcpOutcomes.keys().next().value);
  }

  markWebmcpOperationCancelled(controllerId) {
    if (!controllerId) return false;
    if (this.webmcpOperation?.controllerId === controllerId) this.webmcpOperation = null;
    this.rememberWebmcpOutcome(controllerId, 'cancelled');
    return true;
  }

  isTrackedOperationRunning(name) {
    if (name === 'init' || name === 'enable') return Boolean(this.initRamp);
    if (name === 'do') return Boolean(this.director?.skill);
    if (name === 'sound') return Boolean(this.audioEngine.snapshot().sound);
    return name === 'set_mode';
  }

  reconcileWebmcpOperation() {
    const operation = this.webmcpOperation;
    if (!operation || this.isTrackedOperationRunning(operation.name)) return;
    this.webmcpOperation = null;
    this.rememberWebmcpOutcome(operation.controllerId, 'completed');
  }

  beginWebmcpOperation(name, controllerId) {
    if (!controllerId) return;
    this.webmcpOperation = { name, controllerId };
    this.webmcpOutcomes.delete(controllerId);
  }

  completeWebmcpOperation(controllerId) {
    if (this.webmcpOperation?.controllerId === controllerId) this.webmcpOperation = null;
    this.rememberWebmcpOutcome(controllerId, 'completed');
  }

  releaseOwnedAudio(commands = []) {
    const owned = new Set(commands || []);
    if (owned.has('sound')) this.audioEngine.releaseSound('wheee');
    if (owned.has('theremin')) this.audioEngine.setTheremin(false, this.tof.minimumM);
    if (owned.has('chorale')) this.audioEngine.setChorale({ active: false });
  }

  abortTrackedWebmcpOperation(_reason = 'cancelled') {
    const operation = this.webmcpOperation;
    if (!operation) return false;
    this.webmcpOperation = null;
    this.rememberWebmcpOutcome(operation.controllerId, 'cancelled');
    const owned = this.commandBus.ownedCommandsFor('webmcp', operation.controllerId);
    this.commandBus.cancel({ source: 'webmcp', controllerId: operation.controllerId });
    this.releaseOwnedAudio(owned);
    if (operation.name === 'do') this.director.skill = null;
    else if (operation.name === 'init' || operation.name === 'enable') { this.initRamp = null; this.invalidateInference(); }
    else if (operation.name === 'set_mode') this.invalidateInference();
    else if (operation.name === 'sound') this.audioEngine.stopAll();
    return true;
  }

  prepareForCommand(name, context) {
    this.reconcileWebmcpOperation();
    const operation = this.webmcpOperation;
    if (!operation || (context.source === 'webmcp' && context.controllerId === operation.controllerId)) return;
    const incomingPriority = MICRODUCK_COMMANDS[name]?.authority?.[context.source]?.priority;
    const currentPriority = MICRODUCK_COMMANDS[operation.name]?.authority?.webmcp?.priority;
    if (!Number.isFinite(incomingPriority) || incomingPriority <= currentPriority) throw commandError('COMMAND_CONFLICT', 'A WebMCP one-shot operation is still active.');
    const previous = { source: 'webmcp', controllerId: operation.controllerId, command: operation.name };
    this.abortTrackedWebmcpOperation('preempted');
    this.controllerPreemptHandler(previous, { source: context.source, controllerId: context.controllerId, command: name });
  }

  async executeCommand(name, input = {}, context = { source: 'human' }) {
    if (!this.isReady()) { const error = new Error('MicroDuck policy simulation is not ready.'); error.code = 'SIMULATION_NOT_READY'; throw error; }
    if (name === 'get_state') return { ok: true, command: name, state: this.getState() };
    if (name === 'get_mode') return { ok: true, command: name, mode: this.mode };
    this.prepareForCommand(name, context);
    if (name === 'set_mode') {
      const validated = this.commandBus.execute(name, input, context, this.mode);
      const targetMode = validated.applied.mode;
      const resumePolicy = this.enabled;
      if (context.source === 'webmcp') this.beginWebmcpOperation(name, context.controllerId);
      const epoch = ++this.runEpoch;
      this.inferenceGate.invalidate();
      try {
        await withTimeout(this.policyRuntime.ensureMode(targetMode), MICRODUCK_COMMANDS.set_mode.timeoutMs, 'Policy mode switch timed out.');
        if (this.disposed || epoch !== this.runEpoch) { const error = new Error('Mode switch was cancelled.'); error.code = 'OPERATION_CANCELLED'; throw error; }
        this.resetRuntimeState(targetMode, { enabled: resumePolicy });
        if (context.source === 'webmcp') this.completeWebmcpOperation(context.controllerId);
        return { ...validated, completed: true, state: this.getState() };
      } catch (error) {
        if (context.source === 'webmcp') this.markWebmcpOperationCancelled(context.controllerId);
        throw error;
      }
    }
    if (['sound', 'theremin', 'chorale'].includes(name) && (name === 'sound' || input.active !== false)) this.audioEngine.requireUnlocked();
    const normalizedInput = name === 'enable' && input.toggle ? { ...input, enabled: !this.enabled, toggle: false } : input;
    const busResult = this.commandBus.execute(name, normalizedInput, context, this.mode);
    const result = name === 'enable' && input.toggle ? { ...busResult, requested: { ...input } } : busResult;
    if (name === 'stop' && context.source === 'human') this.audioEngine.stopAll();
    if (name === 'move') this.lastMovement = { requested: [Number(input.vx) || 0, Number(input.vy) || 0, Number(input.yaw) || 0], applied: [result.applied.vx, result.applied.vy, result.applied.yaw], limitedBy: [...result.limitedBy] };
    if (name === 'do') {
      this.director.trigger(result.applied.skill);
      this.dynamics.queueKick(result.applied.skill);
    }
    else if (name === 'enable') {
      if (result.applied.enabled && !this.actuationEnabled) this.beginHomeRamp(true);
      else {
        this.enabled = result.applied.enabled;
        if (!this.enabled) {
          const held = Float32Array.from(this.dynamics.snapshot().joints);
          this.director.reset(this.mode);
          this.director.previousTargets = held;
          this.targets = held;
          this.invalidateInference();
        }
      }
    }
    else if (name === 'init') {
      this.beginHomeRamp(false);
    }
    else if (name === 'reset') await this.reset();
    else if (name === 'relax') {
      this.commandBus.cancel();
      this.enabled = false; this.actuationEnabled = false; this.initRamp = null;
      this.director.reset(this.mode); this.invalidateInference();
    }
    else if (name === 'mouth') this.mouth = result.applied.open;
    else if (name === 'set_color') { this.color = result.applied.value; this.rig.setColor(this.color); }
    else if (name === 'spawn_ball') {
      this.dynamics.spawnBall(result.applied.position);
      this.syncVisuals(this.dynamics.snapshot());
      if (this.cameraMode === 'orbit') this.fit('orbit');
    }
    else if (name === 'set_camera') {
      this.fit(result.applied.value);
    }
    else if (name === 'set_tof_stimulus') {
      this.tofStimulus = { distanceM: result.applied.distanceM, source: result.applied.source, sequence: this.tofStimulus.sequence + 1 };
      this.tof = createModeledTof({ ...this.tofStimulus, valuesM: this.tofStimulus.source === 'raycast' ? this.sampleTofValues() : null });
    }
    else if (name === 'sound') {
      if (result.applied.tag === 'wheee' && result.applied.hold === false) this.audioEngine.releaseSound('wheee');
      else this.audioEngine.playSound(result.applied.tag, { hold: result.applied.hold });
    }
    else if (name === 'theremin') this.audioEngine.setTheremin(Boolean(result.applied.active), this.tof.minimumM);
    else if (name === 'chorale') this.audioEngine.setChorale(result.applied);
    const completed = name === 'init' || (name === 'enable' && this.initRamp)
      ? false
      : name === 'do'
        ? !this.director.skill
        : name === 'sound'
          ? !this.audioEngine.snapshot().sound
          : true;
    if (context.source === 'webmcp' && completed === false) this.beginWebmcpOperation(name, context.controllerId);
    return { ...result, completed, state: this.getState(), audio: this.audioEngine.snapshot() };
  }

  async unlockAudio(event) { const unlocked = await this.audioEngine.unlock(event); return unlocked; }

  beginHomeRamp(enableAfter) {
    this.commandBus.cancel();
    this.director.reset(this.mode);
    this.enabled = false;
    this.actuationEnabled = true;
    this.invalidateInference();
    this.initRamp = { elapsed: 0, duration: 2, enableAfter: Boolean(enableAfter), start: Float32Array.from(this.dynamics.snapshot().joints) };
  }

  renderFrame(time) {
    if (this.disposed) return;
    const shouldAdvance = this.enabled || !this.actuationEnabled || Boolean(this.initRamp);
    if (this.isReady() && !this.paused && shouldAdvance) {
      if (this.lastFrameTime === null) this.lastFrameTime = time;
      const elapsed = Math.max(0, Math.min(0.05, (time - this.lastFrameTime) / 1000));
      this.lastFrameTime = time;
      this.accumulator += elapsed;
      let steps = 0;
      while (this.accumulator >= PHYSICS_TIMESTEP_SECONDS && steps < MAX_PHYSICS_STEPS_PER_FRAME) {
        this.stepPhysics(); this.accumulator -= PHYSICS_TIMESTEP_SECONDS; steps += 1;
      }
      if (this.accumulator >= PHYSICS_TIMESTEP_SECONDS) { this.missedTicks += Math.floor(this.accumulator / PHYSICS_TIMESTEP_SECONDS); this.accumulator %= PHYSICS_TIMESTEP_SECONDS; }
    }
    this.canvas.dataset.microduckMotionState = this.paused ? 'paused' : shouldAdvance ? 'active' : 'holding';
    this.updateCameras(time);
    const width = Math.max(1, this.canvas.clientWidth);
    const height = Math.max(1, this.canvas.clientHeight);
    this.renderer.setScissorTest(false);
    this.renderer.setViewport(0, 0, width, height);
    this.renderer.render(this.scene, this.camera);
  }

  stepPhysics({ allowPolicyWhilePaused = false } = {}) {
    const bus = this.commandBus.snapshot();
    const requestedMove = bus.values.move || { vx: 0, vy: 0, yaw: 0 };
    const movement = [requestedMove.vx || 0, requestedMove.vy || 0, requestedMove.yaw || 0];
    this.lastMovement = { ...this.lastMovement, applied: movement, limitedBy: [...this.lastMovement.limitedBy] };
    if (this.initRamp) {
      this.initRamp.elapsed = Math.min(this.initRamp.duration, this.initRamp.elapsed + PHYSICS_TIMESTEP_SECONDS);
      const progress = this.initRamp.elapsed / this.initRamp.duration;
      const smooth = progress * progress * (3 - 2 * progress);
      this.targets = Float32Array.from(HOME_POLICY_POSITION, (value, index) => this.initRamp.start[index] + (value - this.initRamp.start[index]) * smooth);
      if (progress >= 1) {
        const enableAfter = this.initRamp.enableAfter;
        this.initRamp = null;
        this.enabled = enableAfter;
      }
    }
    this.dynamics.step(this.targets, movement, { actuationEnabled: this.actuationEnabled, movementEnabled: this.enabled && this.actuationEnabled });
    const physics = this.dynamics.snapshot();
    this.updateRecovery(physics);
    this.syncVisuals(physics);
    this.updateModeledPeripherals(physics);
    const audioIntent = this.commandBus.snapshot().values;
    if (!audioIntent.sound?.hold) this.audioEngine.releaseSound('wheee');
    if (!audioIntent.theremin?.active && this.audioEngine.thereminState.active) this.audioEngine.setTheremin(false, this.tof.minimumM);
    else if (audioIntent.theremin?.active) this.audioEngine.setTheremin(true, this.tof.minimumM);
    if (!audioIntent.chorale?.active && this.audioEngine.chorale.active) this.audioEngine.setChorale({ active: false });
    const audioState = this.audioEngine.snapshot();
    this.mouth = audioState.theremin ? audioState.thereminMouth : audioState.chorale ? 0.22 + 0.58 * Math.abs(Math.sin(physics.time * 8)) : Number(audioIntent.mouth?.open || 0);
    this.syncVisuals(physics);
    this.controlTicks += 1;
    if (!this.initRamp && this.controlTicks % POLICY_DECIMATION === 0) this.requestPolicy(physics, bus.values, { allowPaused: allowPolicyWhilePaused });
    this.canvas.dataset.simulationClockS = physics.time.toFixed(3);
  }

  requestPolicy(physics, values, { allowPaused = false } = {}) {
    if (!this.enabled || this.inferenceGate.inFlight || (this.paused && !allowPaused) || this.disposed) { if (this.inferenceGate.inFlight) this.missedTicks += 1; return null; }
    const head = values.head;
    const body = values.pose || {};
    const input = { twist: this.lastMovement.applied, head: [head.neckPitch || 0, head.headPitch || 0, head.headYaw || 0, head.headRoll || 0], body, bodyActive: Boolean(body.z || body.roll || body.pitch) };
    const effective = this.director.effectiveCommand(input);
    const policy = this.recovery.stage !== 'none' && this.mode === 'walking' ? 'stand' : effective.policy;
    const observation = buildObservation({ gyro: physics.gyro, projectedGravity: physics.projectedGravity, policyJointPosition: physics.joints, policyJointVelocity: physics.jointVelocities, lastAction: this.director.lastAction, command: effective.command });
    const gateTag = this.inferenceGate.begin();
    if (!gateTag) { this.missedTicks += 1; return; }
    const tag = { ...gateTag, runEpoch: this.runEpoch };
    this.inferenceInFlight = true;
    const inference = this.policyRuntime.infer(policy, observation).then((action) => {
      if (this.disposed || (this.paused && !allowPaused) || tag.runEpoch !== this.runEpoch || !this.inferenceGate.accepts(gateTag)) return;
      this.targets = this.director.applyAction(action, policy);
      this.director.advance(PHYSICS_TIMESTEP_SECONDS * POLICY_DECIMATION);
    }).catch((error) => { if (!this.disposed && tag.runEpoch === this.runEpoch) { this.lifecycle = 'error'; this.error = String(error.message || error); } })
      .finally(() => { this.inferenceGate.finish(gateTag); this.inferenceInFlight = this.inferenceGate.inFlight; });
    this.lastInferencePromise = inference;
    return inference;
  }

  updateRecovery(physics) {
    const tilted = this.forcedPerturbation || physics.projectedGravity[2] > -0.5;
    if (this.recovery.stage === 'none') {
      this.recovery.tiltedFor = tilted ? this.recovery.tiltedFor + PHYSICS_TIMESTEP_SECONDS : 0;
      if (this.recovery.tiltedFor >= 0.2) {
        if (this.mode === 'roller') { this.resetRuntimeState('roller', { enabled: this.enabled }); this.recovery.resetFallback = true; return; }
        this.recovery.stage = 'stand_policy'; this.recovery.startedAt = physics.time; this.recovery.uprightFor = 0;
      }
      return;
    }
    this.recovery.uprightFor = physics.projectedGravity[2] < -0.85 ? this.recovery.uprightFor + PHYSICS_TIMESTEP_SECONDS : 0;
    if (this.recovery.uprightFor >= 0.4) { this.recovery.stage = 'none'; this.forcedPerturbation = false; }
    else if (physics.time - this.recovery.startedAt >= 6) { this.recovery.resetFallback = true; this.forcedPerturbation = false; this.dynamics.reset(); this.director.reset('walking'); this.recovery.stage = 'none'; this.recovery.tiltedFor = 0; }
  }

  syncVisuals(physics) {
    if (!this.rig) return;
    const wire = policyToWire(physics.joints, this.mouth);
    if (this.audioEngine.chorale.active) {
      wire[6] += Math.sin(physics.time * 2.1) * 0.08;
      wire[7] += Math.sin(physics.time * 1.3) * 0.12;
    }
    this.rig.applyState({ ...Object.fromEntries(this.rig.data.jointContract.wireJointOrder.map((name, index) => [name, wire[index]])), mouth: this.mouth });
    this.rig.applyRootPose(physics.position, physics.quaternion);
    this.ballMesh.position.set(physics.ball.position[0] * 1000, physics.ball.position[2] * 1000, -physics.ball.position[1] * 1000);
  }

  updateModeledPeripherals(physics, reset = false) {
    this.tofStimulus.sequence += 1;
    this.tof = createModeledTof({ ...this.tofStimulus, valuesM: this.tofStimulus.source === 'raycast' ? this.sampleTofValues() : null });
    const headPose = this.rig.getSiteWorldPose('head_imu');
    const currentHeadQuaternion = headPose ? headPose.quaternion.toArray() : [0, 0, 0, 1];
    const headGyro = !reset && this.previousHeadImuQuaternion
      ? deriveFrameAngularVelocity(this.previousHeadImuQuaternion, currentHeadQuaternion, PHYSICS_TIMESTEP_SECONDS)
      : [0, 0, 0];
    this.previousHeadImuQuaternion = currentHeadQuaternion;
    this.imu = deriveModeledImu({ trunkGyro: physics.gyro, projectedGravity: physics.projectedGravity, headGyro });
  }

  sampleTofValues() {
    const range = MICRODUCK_COMMANDS.set_tof_stimulus.ui.fields.distanceM.range;
    const pose = this.rig?.getSiteWorldPose('tof');
    if (!pose) return Array(64).fill(range[1]);
    this.ballMesh.updateWorldMatrix(true, false);
    return Array.from({ length: 64 }, (_, index) => {
      const row = Math.floor(index / 8);
      const col = index % 8;
      const direction = new THREE.Vector3((col - 3.5) * 0.07, (3.5 - row) * 0.07, 1).normalize().applyQuaternion(pose.quaternion);
      this.tofRaycaster.set(pose.position, direction);
      this.tofRaycaster.near = range[0] * 1000;
      this.tofRaycaster.far = range[1] * 1000;
      return (this.tofRaycaster.intersectObject(this.ballMesh, false)[0]?.distance || this.tofRaycaster.far) / 1000;
    });
  }

  invalidateInference() { this.runEpoch += 1; this.requestSequence += 1; this.inferenceGate.invalidate(); this.inferenceInFlight = false; }
  pause() { if (this.disposed) return false; this.paused = true; this.invalidateInference(); return true; }
  resume() { if (this.disposed) return false; this.paused = false; this.lastFrameTime = null; return true; }
  stop() {
    if (this.disposed) return false;
    this.abortTrackedWebmcpOperation('stop');
    this.commandBus.cancel();
    const held = Float32Array.from(this.dynamics.snapshot().joints);
    this.director.reset(this.mode);
    this.director.previousTargets = held;
    this.targets = held;
    this.enabled = false;
    this.audioEngine.stopAll();
    this.invalidateInference();
    this.paused = false;
    this.canvas.dataset.microduckMotionState = 'holding';
    return true;
  }
  cancelController(source, controllerId) {
    const owned = this.commandBus.ownedCommandsFor(source, controllerId);
    const cancelled = this.commandBus.cancel({ source, controllerId });
    if (cancelled) this.releaseOwnedAudio(owned);
    if (source === 'webmcp' && this.webmcpOperation?.controllerId === controllerId) this.markWebmcpOperationCancelled(controllerId);
    return cancelled;
  }
  abortCommand(name, { source, controllerId } = {}) {
    const definition = MICRODUCK_COMMANDS[name];
    if (!definition) { void this.reset(); return true; }
    if (source === 'webmcp' && this.webmcpOperation?.controllerId === controllerId) return this.abortTrackedWebmcpOperation('abort');
    if (name === 'sound') {
      this.cancelController(source, controllerId);
      if (source === 'webmcp') this.audioEngine.releaseSound('wheee');
      else this.audioEngine.stopAll();
      return true;
    }
    if (definition.classification === 'continuous') {
      const cancelled = this.cancelController(source, controllerId);
      return cancelled;
    }
    if (name === 'do') { this.director.skill = null; return true; }
    if (name === 'init') { this.initRamp = null; this.invalidateInference(); return true; }
    if (name === 'set_mode') { this.invalidateInference(); return true; }
    if (definition.cancellable === false) return true;
    void this.reset();
    return true;
  }
  isCommandComplete(name, { source, controllerId } = {}) {
    if (source === 'webmcp' && controllerId) {
      this.reconcileWebmcpOperation();
      const outcome = this.webmcpOutcomes.get(controllerId);
      if (outcome === 'cancelled') throw commandError('OPERATION_CANCELLED', 'The WebMCP one-shot operation was cancelled by a higher-priority action.');
      if (this.webmcpOperation?.controllerId === controllerId) return false;
      if (outcome === 'completed') return true;
    }
    if (name === 'init' || name === 'enable') return !this.initRamp;
    if (name === 'do') return !this.director.skill;
    if (name === 'sound') return !this.audioEngine.snapshot().sound;
    return true;
  }
  isControllerActive(source, controllerId) { return this.commandBus.isOwnedBy(source, controllerId); }
  acquireController(source, controllerId, durationMs = 5000) { return this.commandBus.connect({ source, controllerId, durationMs }); }
  refreshControllerLease(source, controllerId, durationMs = 5000) { return this.commandBus.refresh({ source, controllerId, durationMs }); }
  setControllerPreemptHandler(handler) { this.controllerPreemptHandler = typeof handler === 'function' ? handler : () => {}; }
  releaseHumanIntent() {
    const owned = this.commandBus.ownedCommandsFor('human');
    const released = this.commandBus.cancel({ source: 'human' });
    if (released) this.releaseOwnedAudio(owned);
    return released;
  }
  perturb(orientation) { if (!this.isReady()) return false; this.forcedPerturbation = true; this.dynamics.perturb(orientation); return true; }
  async applyAction(action, options = {}) { return this.executeCommand(action?.command || action?.name, action?.args || action, options); }
  async advanceTime(seconds, { controlStep = false } = {}) { const steps = Math.ceil(Math.max(0, Number(seconds) || 0) / PHYSICS_TIMESTEP_SECONDS); const before = this.lastInferencePromise; for (let index = 0; index < steps; index += 1) this.stepPhysics({ allowPolicyWhilePaused: controlStep }); if (controlStep && this.lastInferencePromise && this.lastInferencePromise !== before) await this.lastInferencePromise; return true; }
  advanceBase(seconds) { return this.advanceTime(seconds); }
  isReady() { return !this.disposed && this.lifecycle === 'ready' && Boolean(this.rig && this.dynamics && this.policyRuntime); }

  getState() {
    const physics = this.dynamics?.snapshot() || {};
    const bus = this.commandBus.snapshot();
    const body = bus.values.pose || {};
    const appliedMove = [bus.values.move?.vx || 0, bus.values.move?.vy || 0, bus.values.move?.yaw || 0];
    const movement = { ...this.lastMovement, applied: appliedMove };
    const activePolicy = this.initRamp ? 'home_ramp' : this.enabled ? this.director?.currentPolicy({ twist: appliedMove, bodyActive: Boolean(body.z || body.roll || body.pitch) }) : 'disabled';
    const head = bus.values.head || {};
    const headValues = [head.neckPitch, head.headPitch, head.headYaw, head.headRoll];
    const camera = this.cameraDefinition();
    return createMicroDuckState({ time: physics.time, enabled: this.enabled, actuationEnabled: this.actuationEnabled, lifecycle: this.lifecycle, mode: this.mode, movement, head: headValues, body: [0, 0, body.z, body.roll, body.pitch, 0], mouth: this.mouth, activePolicy, phase: this.initRamp ? 'initializing' : this.director?.skill?.name || this.director?.sit || 'idle', safety: { recovery: this.recovery.stage, fallen: this.recovery.tiltedFor >= 0.2, resetFallback: this.recovery.resetFallback }, loop: { rateHz: 50, missedTicks: this.missedTicks, inferenceInFlight: this.inferenceInFlight }, joints: physics.joints, targets: this.targets, simulatedPose: { position: physics.position, quaternion: physics.quaternion }, contacts: physics.contacts, ball: physics.ball, imu: this.imu || deriveModeledImu({ trunkGyro: physics.gyro, projectedGravity: physics.projectedGravity }), tof: this.tof, virtualCamera: { mode: this.cameraMode, name: camera.label, purpose: camera.purpose, frame: camera.frame, inset: false }, audio: this.audioEngine.snapshot(), color: this.color });
  }

  getTelemetry() { const state = this.getState(); return { simulation_clock_s: state.time, policy_rate_hz: state.loop.rateHz, missed_policy_ticks: state.loop.missedTicks, joint_count: state.joints.length, mouth_open_ratio: state.mouth, enabled: Number(state.enabled), recovery_active: Number(state.safety.recovery !== 'none'), modeled_tof_minimum_m: state.tof.minimumM, modeled_tof_usable_zones: state.tof.usable, modeled_trunk_gyro_x_rad_s: state.imu.trunk.gyro[0], modeled_head_gyro_z_rad_s: state.imu.head.gyro[2], local_audio_unlocked: Number(state.audio.unlocked), local_chorale_voices: state.audio.voices }; }
  getContacts() { const state = this.getState(); return { simulationMode: state.simulationMode, stateKind: state.stateKind, contactModel: state.contacts.model, contactCount: state.contacts.count, ballContact: state.contacts.ballContact, ballAttached: false, hardwareValidation: false, rlModelParity: false }; }
  cameraDefinition(mode = this.cameraMode) {
    return MICRODUCK_COMMANDS.set_camera.ui.options[mode] || MICRODUCK_COMMANDS.set_camera.ui.options.orbit;
  }

  applyCameraIdentity() {
    const definition = this.cameraDefinition();
    this.canvas.dataset.cameraView = this.cameraMode;
    this.canvas.dataset.cameraName = definition.label;
    this.canvas.dataset.cameraPurpose = definition.purpose;
    this.canvas.setAttribute('aria-label', `MicroDuck ${definition.label} camera. ${definition.purpose}`);
    const label = this.canvas.closest('.sim-viewport')?.querySelector('.camera-mode-label');
    if (label) {
      label.textContent = definition.overlay;
      label.title = definition.purpose;
      label.setAttribute('aria-label', `${definition.label}. ${definition.purpose}`);
    }
  }

  getOverviewBounds() {
    const bounds = this.rig.getBounds(this.cameraBoundsScratch);
    this.ballMesh.updateWorldMatrix(true, false);
    this.cameraBallBoundsScratch.setFromObject(this.ballMesh);
    return bounds.union(this.cameraBallBoundsScratch);
  }

  followDirection() {
    this.robotForwardScratch.set(1, 0, 0).applyQuaternion(this.rig.root.quaternion);
    this.robotForwardScratch.y = 0;
    if (this.robotForwardScratch.lengthSq() < 1e-8) this.robotForwardScratch.set(1, 0, 0);
    else this.robotForwardScratch.normalize();
    this.robotSideScratch.crossVectors(WORLD_UP, this.robotForwardScratch).normalize();
    return this.cameraDirectionScratch.copy(this.robotForwardScratch).multiplyScalar(-1.65)
      .addScaledVector(WORLD_UP, 0.76)
      .addScaledVector(this.robotSideScratch, 0.42)
      .normalize();
  }

  framedPose(bounds, direction, { margin = 0.72, minimumDistance = 340 } = {}) {
    const center = bounds.getCenter(new THREE.Vector3());
    const unitDirection = direction.clone().normalize();
    this.cameraProbe.fov = this.camera.fov;
    this.cameraProbe.aspect = this.camera.aspect;
    this.cameraProbe.up.copy(WORLD_UP);
    this.cameraProbe.position.copy(center).add(unitDirection);
    this.cameraProbe.lookAt(center);
    this.cameraProbe.updateMatrixWorld(true);
    const cameraRotationInverse = this.cameraProbe.quaternion.clone().invert();
    const verticalFov = THREE.MathUtils.degToRad(this.camera.fov);
    const horizontalFov = 2 * Math.atan(Math.tan(verticalFov / 2) * Math.max(0.2, this.camera.aspect));
    const tanHorizontal = Math.max(0.01, Math.tan(horizontalFov / 2));
    const tanVertical = Math.max(0.01, Math.tan(verticalFov / 2));
    let distance = minimumDistance;
    for (const x of [bounds.min.x, bounds.max.x]) {
      for (const y of [bounds.min.y, bounds.max.y]) {
        for (const z of [bounds.min.z, bounds.max.z]) {
          const point = new THREE.Vector3(x, y, z).sub(center).applyQuaternion(cameraRotationInverse);
          distance = Math.max(
            distance,
            point.z + Math.abs(point.x) / (margin * tanHorizontal),
            point.z + Math.abs(point.y) / (margin * tanVertical)
          );
        }
      }
    }
    distance *= 1.04;
    return { center, direction: unitDirection, distance, position: center.clone().addScaledVector(unitDirection, distance) };
  }

  configureProjection(fov, distance = 400) {
    this.camera.fov = fov;
    this.camera.near = Math.max(1, distance / 1200);
    this.camera.far = Math.max(3000, distance * 8);
    this.camera.updateProjectionMatrix();
  }

  fitOverview() {
    this.configureProjection(DEFAULT_CAMERA_FOV_DEG);
    const direction = this.camera.aspect < 0.72 ? OVERVIEW_DIRECTION_TALL : OVERVIEW_DIRECTION_WIDE;
    const pose = this.framedPose(this.getOverviewBounds(), direction, { margin: 0.74, minimumDistance: 360 });
    this.configureProjection(DEFAULT_CAMERA_FOV_DEG, pose.distance);
    const damping = this.controls.enableDamping;
    this.controls.enabled = true;
    this.controls.enableDamping = false;
    // Flush any partially damped learner orbit before restoring the exact fit.
    this.controls.update();
    this.controls.target.copy(pose.center);
    this.camera.position.copy(pose.position);
    this.camera.lookAt(pose.center);
    this.camera.updateMatrixWorld(true);
    this.controls.update();
    this.controls.saveState();
    this.controls.enableDamping = damping;
    this.overviewTrackingCenter.copy(pose.center);
  }

  fitFollow() {
    this.configureProjection(DEFAULT_CAMERA_FOV_DEG);
    const pose = this.framedPose(this.rig.getBounds(this.cameraBoundsScratch), this.followDirection(), { margin: 0.7, minimumDistance: 380 });
    this.followDistance = pose.distance;
    this.configureProjection(DEFAULT_CAMERA_FOV_DEG, pose.distance);
    this.controls.enabled = false;
    this.cameraFollowTarget.copy(pose.center);
    this.camera.position.copy(pose.position);
    this.camera.lookAt(pose.center);
    this.camera.updateMatrixWorld(true);
  }

  alignHeadView() {
    const pose = this.rig.getSiteWorldPose('head_camera');
    if (!pose) return;
    this.controls.enabled = false;
    this.camera.position.copy(pose.position);
    this.camera.quaternion.copy(pose.quaternion).multiply(CAMERA_FROM_SITE_FRAME);
    this.camera.updateMatrixWorld(true);
  }

  fit(mode = this.cameraMode) {
    if (!this.rig) return false;
    this.cameraMode = MICRODUCK_COMMANDS.set_camera.values.includes(mode) ? mode : 'orbit';
    this.lastCameraUpdateTime = null;
    if (this.cameraMode === 'orbit') this.fitOverview();
    else if (this.cameraMode === 'chase') this.fitFollow();
    else {
      this.configureProjection(HEAD_CAMERA_FOV_DEG, 360);
      this.alignHeadView();
    }
    this.applyCameraIdentity();
    return this.cameraMode;
  }

  updateOverview() {
    const center = this.getOverviewBounds().getCenter(this.cameraCenterScratch);
    const translation = this.cameraDirectionScratch.copy(center).sub(this.overviewTrackingCenter);
    if (translation.lengthSq() > 1e-10) {
      this.camera.position.add(translation);
      this.controls.target.add(translation);
      this.overviewTrackingCenter.copy(center);
    }
    this.controls.enabled = true;
    this.controls.update();
  }

  updateFollow(time) {
    const target = this.rig.getBounds(this.cameraBoundsScratch).getCenter(this.cameraCenterScratch);
    this.cameraDesiredPosition.copy(target).addScaledVector(this.followDirection(), this.followDistance);
    const dt = Number.isFinite(time) && Number.isFinite(this.lastCameraUpdateTime)
      ? Math.max(0, Math.min(0.05, (time - this.lastCameraUpdateTime) / 1000))
      : 1 / 60;
    const alpha = 1 - Math.exp(-10 * dt);
    this.camera.position.lerp(this.cameraDesiredPosition, alpha);
    this.cameraFollowTarget.lerp(target, alpha);
    this.camera.lookAt(this.cameraFollowTarget);
    this.camera.updateMatrixWorld(true);
    this.lastCameraUpdateTime = Number.isFinite(time) ? time : this.lastCameraUpdateTime;
  }

  updateCameras(time) {
    if (!this.rig) return;
    if (this.cameraMode === 'orbit') this.updateOverview();
    else if (this.cameraMode === 'chase') this.updateFollow(time);
    else this.alignHeadView();
  }

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    const width = Math.max(1, rect.width);
    const height = Math.max(1, rect.height);
    const previousAspect = this.camera.aspect;
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    if (this.rig && Math.abs(previousAspect - this.camera.aspect) > 0.01) this.fit(this.cameraMode);
  }
  animate() { if (this.disposed) return; this.animationFrame = requestAnimationFrame((time) => { this.renderFrame(time); this.animate(); }); }
  dispose() { if (this.disposed) return; this.disposed = true; this.invalidateInference(); if (this.animationFrame) cancelAnimationFrame(this.animationFrame); this.audioEngine.dispose(); this.rig?.dispose(); this.dynamics?.dispose(); void this.policyRuntime?.dispose?.(); this.ballMesh.geometry.dispose(); this.ballMesh.material.dispose(); this.floor.geometry.dispose(); this.floor.material.dispose(); this.fieldBoundary.geometry.dispose(); this.fieldBoundary.material.dispose(); this.controls?.dispose(); this.resizeObserver?.disconnect(); this.renderer?.dispose(); }
}

async function withTimeout(promise, timeoutMs, message) {
  let timeout;
  try {
    return await Promise.race([
      promise,
      new Promise((_, reject) => { timeout = setTimeout(() => reject(commandError('POLICY_TIMEOUT', message)), timeoutMs); }),
    ]);
  } finally {
    clearTimeout(timeout);
  }
}
