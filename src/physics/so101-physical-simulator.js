import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js';
import { OrbitControls } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/controls/OrbitControls.js';
import { CanonicalRobotRig } from '../canonical-rig.js';
import { BrowserMuJoCoBackend } from './browser-mujoco-backend.js';
import { PhysicsSession } from './session.js';
import { SO101_MANIPULATION_MODEL_PACKAGE } from './model-packages.js';
import { SO101_MANIPULATION_SCENE } from './so101-scene.js';
import {
  So101BlockTransferEvaluator,
  hasSo101BlockGripperContact,
  hasSo101BlockTargetSupportContact,
} from './so101-task-evaluator.js';

const RAD_TO_DEG = 180 / Math.PI;
const MAX_WEBMCP_ADVANCE_SECONDS = 2;
const STEP_ALIGNMENT_TOLERANCE_SECONDS = 1e-9;
const PRESENTATION_GROUND_COLOR = 0x687378;

function toThreePosition(positionM = [0, 0, 0]) {
  return new THREE.Vector3(Number(positionM[0]) * 1000, Number(positionM[2]) * 1000, -Number(positionM[1]) * 1000);
}

function toThreeQuaternion(quaternionWxyz = [1, 0, 0, 0]) {
  const [w, x, y, z] = quaternionWxyz.map(Number);
  const physical = new THREE.Quaternion(x, y, z, w).normalize();
  const basis = new THREE.Matrix4().makeRotationX(-Math.PI / 2);
  const inverse = basis.clone().invert();
  const physicalRotation = new THREE.Matrix4().makeRotationFromQuaternion(physical);
  const converted = basis.clone().multiply(physicalRotation).multiply(inverse);
  return new THREE.Quaternion().setFromRotationMatrix(converted).normalize();
}

function finiteTargetMap(targetsRad) {
  if (!targetsRad || typeof targetsRad !== 'object' || Array.isArray(targetsRad)) throw new TypeError('targetsRad must be an object');
  const entries = Object.entries(targetsRad);
  if (!entries.length) throw new TypeError('targetsRad must contain at least one joint target');
  const allowed = new Set(SO101_MANIPULATION_MODEL_PACKAGE.joints.map((joint) => joint.id));
  const clean = {};
  for (const [jointId, raw] of entries) {
    if (!allowed.has(jointId)) throw new Error(`Unknown physical SO-101 joint ${jointId}`);
    const value = Number(raw);
    if (!Number.isFinite(value)) throw new TypeError(`Physical SO-101 target ${jointId} must be finite radians`);
    clean[jointId] = value;
  }
  return clean;
}

function supportVisual({ centerXYM, halfExtentsXYM }, topZM, material) {
  const halfZ = 0.020;
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(halfExtentsXYM[0] * 2000, halfZ * 2000, halfExtentsXYM[1] * 2000),
    material,
  );
  mesh.position.copy(toThreePosition([centerXYM[0], centerXYM[1], topZM - halfZ]));
  mesh.receiveShadow = true;
  return mesh;
}

export class So101PhysicalSimulator {
  constructor(canvas) {
    this.canvas = canvas;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xb9c1c4);
    this.camera = new THREE.PerspectiveCamera(43, 1, 1, 5000);
    this.camera.position.set(650, 500, 650);
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.shadowMap.enabled = true;
    this.controls = new OrbitControls(this.camera, this.canvas);
    this.controls.enableDamping = true;
    this.controls.target.set(260, 150, 75);

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x334155, 1.35));
    const key = new THREE.DirectionalLight(0xffffff, 2.0);
    key.position.set(450, 900, 500);
    key.castShadow = true;
    this.scene.add(key);
    const fill = new THREE.DirectionalLight(0x93c5fd, 0.4);
    fill.position.set(-350, 500, -450);
    this.scene.add(fill);

    const grid = new THREE.GridHelper(1000, 20, 0x334155, 0x4b5563);
    grid.position.y = 0;
    grid.userData.presentationOnly = true;
    this.scene.add(grid);

    this.workcellRoot = new THREE.Group();
    this.workcellRoot.name = 'so101-physical-workcell-presentation';
    this.scene.add(this.workcellRoot);

    this.rig = null;
    this.blockMesh = null;
    this.targetMarker = null;
    this.session = null;
    this.unsubscribeSession = null;
    this.evaluator = null;
    this.lastObservation = null;
    this.ready = false;
    this.disposed = false;
    this.highContrast = true;
    this.sessionSequence = 0;
    this.setHighContrastScene(true);
    this.resize();
  }

  async setScenario(profileId, scenario) {
    if (profileId !== 'so101') throw new Error('SO-101 physical simulator only accepts the so101 profile');
    if (scenario?.simulationMode !== 'physical_mujoco' || scenario?.physicalSceneId !== SO101_MANIPULATION_SCENE.id) {
      throw new Error('SO-101 physical simulator requires the versioned manipulation physical workspace');
    }
    await this.#disposeSession();
    this.#clearPresentation();
    this.rig = await CanonicalRobotRig.load('so101');
    this.scene.add(this.rig.root);
    this.#buildWorkcell();
    this.evaluator = new So101BlockTransferEvaluator();
    await this.#createSession();
    this.ready = true;
    this.canvas.dataset.simulatorBackend = 'browser-mujoco';
    this.canvas.dataset.simulationAuthority = 'physics-session';
    this.canvas.dataset.physicalSceneId = SO101_MANIPULATION_SCENE.id;
    this.canvas.dataset.modelPackageId = SO101_MANIPULATION_SCENE.modelPackage;
    this.canvas.dataset.presentationGroundColor = '#687378';
    this.fit();
    return true;
  }

  async reset() {
    this.#assertNotDisposed();
    this.ready = false;
    this.evaluator = new So101BlockTransferEvaluator();
    if (!this.session) {
      await this.#createSession();
    } else {
      let diagnostics = null;
      try { diagnostics = await this.session.getDiagnostics(); } catch {}
      if (!diagnostics?.loaded) {
        await this.#disposeSession();
        await this.#createSession();
      } else {
        await this.session.reset({ reason: 'explicit-user-reset' });
      }
    }
    this.ready = true;
    return true;
  }

  renderFrame() {
    if (this.disposed) return;
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  resize() {
    if (this.disposed) return false;
    const width = Math.max(1, this.canvas.clientWidth || this.canvas.width || 640);
    const height = Math.max(1, this.canvas.clientHeight || this.canvas.height || 480);
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const targetWidth = Math.floor(width * dpr);
    const targetHeight = Math.floor(height * dpr);
    if (this.canvas.width !== targetWidth || this.canvas.height !== targetHeight) this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    return true;
  }

  fit() {
    if (!this.camera || !this.controls) return false;
    this.controls.target.set(260, 150, 75);
    this.camera.position.set(680, 480, 620);
    this.camera.near = 1;
    this.camera.far = 5000;
    this.camera.updateProjectionMatrix();
    this.controls.update();
    return true;
  }

  setHighContrastScene(value) {
    this.highContrast = Boolean(value);
    if (this.targetMarker) this.targetMarker.visible = this.highContrast;
    this.canvas.dataset.highContrastScene = String(this.highContrast);
    this.canvas.dataset.highContrastPerimeterCount = this.highContrast ? '1' : '0';
    this.canvas.dataset.presentationGroundColor = '#687378';
    return this.highContrast;
  }

  isHighContrastSceneEnabled() { return this.highContrast; }

  isReady() {
    return Boolean(
      this.ready
      && this.session
      && this.lastObservation
      && this.session.sceneRevision
      && this.session.robotId
    );
  }

  getPhysicalSession() { return this.session; }

  getPhysicalAuthorityToken() {
    if (!this.isReady()) return null;
    return Object.freeze({
      sessionId: this.session.sessionId,
      epoch: this.session.epoch,
      sceneRevision: this.session.sceneRevision,
      robotId: this.session.robotId,
      simulationTimeSeconds: this.lastObservation.simulationTimeSeconds,
    });
  }

  getTaskEvaluation() { return this.evaluator?.snapshot?.() || null; }

  getTelemetry() {
    const observation = this.lastObservation;
    if (!observation) return {};
    const out = { simulation_time_s: Number(observation.simulationTimeSeconds) };
    for (const [jointId, state] of Object.entries(observation.joints || {})) out[`${jointId}_rad`] = Number(state.positionRad);
    const block = observation.bodies?.benchmark_block?.positionM;
    if (Array.isArray(block)) {
      out.block_x_m = Number(block[0]);
      out.block_y_m = Number(block[1]);
      out.block_z_m = Number(block[2]);
    }
    return out;
  }

  getContacts() {
    const observation = this.lastObservation;
    const evaluation = this.getTaskEvaluation();
    if (!observation || !evaluation) return {};
    return {
      contact_count: Number(observation.contactCount || 0),
      block_gripper_contact: hasSo101BlockGripperContact(observation),
      block_target_support_contact: hasSo101BlockTargetSupportContact(observation),
      grasp_contact_seen: evaluation.contactSeen,
      lift_seen: evaluation.liftSeen,
      carry_seen: evaluation.carrySeen,
      release_seen: evaluation.releaseSeen,
      settled: evaluation.settleSeen,
      inside_target: evaluation.inTarget,
      task_success: evaluation.success,
    };
  }

  getState() {
    return this.lastObservation ? {
      observation: structuredClone(this.lastObservation),
      evaluation: this.getTaskEvaluation(),
      authority: this.getPhysicalAuthorityToken(),
    } : null;
  }

  async applyAction() {
    throw new Error('Legacy SO-101 .pos replay is disabled in the physical workspace. Use robobuddy.sim.v1 radians or the versioned physical WebMCP schema.');
  }

  async applyPhysicalTargets(targetsRad, { maxSteps = 1000, advanceSeconds = 0 } = {}) {
    this.#assertReady();
    const cleanTargets = finiteTargetMap(targetsRad);
    if (!Number.isInteger(maxSteps) || maxSteps < 1 || maxSteps > 5000) throw new RangeError('maxSteps must be an integer from 1 to 5000');
    const advance = Number(advanceSeconds);
    if (!Number.isFinite(advance) || advance < 0 || advance > MAX_WEBMCP_ADVANCE_SECONDS) {
      throw new RangeError(`advanceSeconds must be between 0 and ${MAX_WEBMCP_ADVANCE_SECONDS}`);
    }
    const accepted = await this.session.sendCommand(
      { type: 'set_joint_targets', targetsRad: cleanTargets },
      { maxSteps },
    );
    let observation = accepted.observation;
    if (advance > 0) observation = await this.advanceTime(advance);
    return {
      schemaVersion: 'robobuddy.so101.physical.v1',
      status: accepted.status,
      commandId: accepted.commandId,
      acceptedTargetsRad: cleanTargets,
      observation: structuredClone(observation),
      taskEvaluation: this.getTaskEvaluation(),
    };
  }

  async advanceTime(seconds) {
    this.#assertReady();
    const duration = Number(seconds);
    if (!Number.isFinite(duration) || duration < 0) throw new RangeError('Physical advance duration must be finite and non-negative');
    if (duration === 0) return this.session.getObservation({ view: 'ground_truth' });
    const dt = Number(this.lastObservation?.engine?.timestepSeconds);
    const steps = Math.round(duration / dt);
    if (steps < 1 || Math.abs(duration - steps * dt) > STEP_ALIGNMENT_TOLERANCE_SECONDS) {
      throw new RangeError(`Physical advance duration ${duration}s must align to the ${dt}s MuJoCo timestep`);
    }
    return this.session.advanceSteps(steps);
  }

  pause() { return this.session?.pause?.() ?? false; }
  resume() { return this.session?.resume?.() ?? false; }

  async stop() {
    if (!this.session) return false;
    this.ready = false;
    try {
      const diagnostics = await this.session.getDiagnostics();
      if (!diagnostics?.loaded) return false;
      await this.session.cancelRun('human-stop');
      return true;
    } catch {
      return false;
    }
  }

  dispose() {
    if (this.disposed) return;
    this.disposed = true;
    this.ready = false;
    void this.#disposeSession();
    this.#clearPresentation();
    this.controls?.dispose?.();
    this.renderer?.dispose?.();
  }

  async #createSession() {
    const session = new PhysicsSession(new BrowserMuJoCoBackend(), { sessionId: `ide-so101-${++this.sessionSequence}` });
    this.session = session;
    this.unsubscribeSession = session.subscribe(({ observation }) => this.#consumeObservation(observation));
    await session.loadScene(structuredClone(SO101_MANIPULATION_SCENE));
  }

  async #disposeSession() {
    this.unsubscribeSession?.();
    this.unsubscribeSession = null;
    if (this.session) this.session.dispose();
    this.session = null;
    this.lastObservation = null;
  }

  #consumeObservation(observation) {
    if (!observation || this.disposed) return;
    this.lastObservation = structuredClone(observation);
    this.evaluator?.observe(observation);
    this.#applyRobotObservation(observation);
    this.#applyBlockObservation(observation);
    this.canvas.dataset.simulationClockS = String(Number(observation.simulationTimeSeconds || 0));
    const evaluation = this.getTaskEvaluation();
    this.canvas.dataset.physicalTaskSuccess = String(Boolean(evaluation?.success));
    this.canvas.dataset.physicalTaskContact = String(Boolean(evaluation?.contactSeen));
    this.canvas.dataset.physicalTaskLift = String(Boolean(evaluation?.liftSeen));
    this.canvas.dataset.physicalTaskCarry = String(Boolean(evaluation?.carrySeen));
    this.canvas.dataset.physicalTaskRelease = String(Boolean(evaluation?.releaseSeen));
    this.canvas.dataset.physicalTaskSettled = String(Boolean(evaluation?.settleSeen));
  }

  #applyRobotObservation(observation) {
    if (!this.rig) return;
    const degreeState = {};
    for (const jointId of ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']) {
      const value = Number(observation.joints?.[jointId]?.positionRad);
      if (Number.isFinite(value)) degreeState[`${jointId}.pos`] = value * RAD_TO_DEG;
    }
    this.rig.applyPhysicalState(degreeState);

    const gripperRad = Number(observation.joints?.gripper?.positionRad);
    const group = this.rig.groups?.gripper_jaw;
    const joint = group?.userData?.joint;
    if (Number.isFinite(gripperRad) && group && joint) {
      const axis = new THREE.Vector3().fromArray(joint.axis || [0, 1, 0]).normalize();
      const motion = new THREE.Quaternion().setFromAxisAngle(axis, (Number(joint.sign ?? 1) * gripperRad) + (Number(joint.offsetDeg || 0) / RAD_TO_DEG));
      group.quaternion.copy(group.userData.baseQuaternion).multiply(motion).normalize();
      this.rig.root.updateMatrixWorld(true);
    }
  }

  #applyBlockObservation(observation) {
    if (!this.blockMesh) return;
    const body = observation.bodies?.benchmark_block;
    if (!body?.positionM) return;
    this.blockMesh.position.copy(toThreePosition(body.positionM));
    if (body.quaternionWxyz) this.blockMesh.quaternion.copy(toThreeQuaternion(body.quaternionWxyz));
  }

  #buildWorkcell() {
    const benchmark = SO101_MANIPULATION_MODEL_PACKAGE.benchmark;
    const supportMaterial = new THREE.MeshStandardMaterial({ color: PRESENTATION_GROUND_COLOR, roughness: 0.76, metalness: 0.03 });
    const source = supportVisual(benchmark.supportPads.source, benchmark.workSurface.topZM, supportMaterial);
    source.name = 'visual-benchmark-source-support';
    const target = supportVisual(benchmark.supportPads.target, benchmark.workSurface.topZM, supportMaterial);
    target.name = 'visual-benchmark-target-support';
    this.workcellRoot.add(source, target);

    const markerMaterial = new THREE.MeshBasicMaterial({ color: 0x22c55e, transparent: true, opacity: 0.45, depthWrite: false });
    const marker = new THREE.Mesh(
      new THREE.BoxGeometry(benchmark.target.halfExtentsXYM[0] * 2000, 1, benchmark.target.halfExtentsXYM[1] * 2000),
      markerMaterial,
    );
    marker.name = 'visual-benchmark-target-region';
    marker.position.copy(toThreePosition([benchmark.target.centerXYM[0], benchmark.target.centerXYM[1], benchmark.workSurface.topZM + 0.0005]));
    marker.userData.presentationOnly = true;
    this.targetMarker = marker;
    this.workcellRoot.add(marker);

    const dims = benchmark.object.dimensionsM;
    const blockMaterial = new THREE.MeshStandardMaterial({ color: 0x2f80ed, roughness: 0.58, metalness: 0.02 });
    this.blockMesh = new THREE.Mesh(new THREE.BoxGeometry(dims[0] * 1000, dims[2] * 1000, dims[1] * 1000), blockMaterial);
    this.blockMesh.name = 'visual-benchmark-block';
    this.blockMesh.castShadow = true;
    this.blockMesh.receiveShadow = true;
    this.workcellRoot.add(this.blockMesh);
    this.setHighContrastScene(this.highContrast);
  }

  #clearPresentation() {
    if (this.rig) {
      this.scene.remove(this.rig.root);
      this.rig.dispose();
      this.rig = null;
    }
    for (const child of [...this.workcellRoot.children]) {
      child.traverse?.((node) => {
        if (!node.isMesh) return;
        node.geometry?.dispose?.();
        if (Array.isArray(node.material)) node.material.forEach((material) => material?.dispose?.());
        else node.material?.dispose?.();
      });
      this.workcellRoot.remove(child);
    }
    this.blockMesh = null;
    this.targetMarker = null;
  }

  #assertReady() {
    this.#assertNotDisposed();
    if (!this.isReady()) throw new Error('SO-101 physical session is not ready');
  }

  #assertNotDisposed() {
    if (this.disposed) throw new Error('SO-101 physical simulator is disposed');
  }
}
