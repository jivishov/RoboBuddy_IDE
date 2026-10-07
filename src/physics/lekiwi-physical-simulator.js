import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js';
import { OrbitControls } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/controls/OrbitControls.js';
import { CanonicalRobotRig, canonicalVisualProvenance } from '../canonical-rig.js';
import { BrowserMuJoCoBackend } from './browser-mujoco-backend.js';
import { PhysicsSession } from './session.js';
import { LEKIWI_COURIER_PACKAGE } from './lekiwi-model-package.js';
import {
  LEKIWI_COURIER_CONTROLLER, LEKIWI_COURIER_SCENE, LEKIWI_DRIVE_LIMITS, LEKIWI_WORKCELL,
  chassisVelocityCommand, waypointReached, wheelTargetsForChassis,
} from './lekiwi-scene.js';
import { LeKiwiCourierEvaluator, basePoseFromObservation } from './lekiwi-task-evaluator.js';
import { MAX_WHEEL_RAD_S, publicActionToBodyCommand, WHEEL_ORDER } from './lekiwi-kinematics.js';
import { LEKIWI_CANONICAL_PRESENTATION_MAP, MODEL_FRAME } from './lekiwi-source-audit.js';

const MAX_WEBMCP_ADVANCE_SECONDS = 2;
const STEP_ALIGNMENT_TOLERANCE_SECONDS = 1e-9;
// Declared observation cadence, in authoritative MuJoCo steps. The narrowest causal event this
// task depends on is the overlap between worktop support contact and a still-closed rim pinch;
// native 1 ms evidence measures that overlap at more than one simulated second, and the shortest
// evaluated dwell is the 0.25 s settle window. Five physics steps (10 ms at the pinned 0.002 s
// timestep) resolve both by a wide margin at a fraction of OpenArm's cost. This is a software
// observation parameter, not a hardware sensor rate.
const LEKIWI_OBSERVATION_BATCH_STEPS = 5;
const PRESENTATION_GROUND_COLOR = 0x687378;
const RAD_TO_DEG = 180 / Math.PI;
// The canonical mesh is baked in the pinned URDF root frame, while the physical MuJoCo base
// body uses the audited LeRobot body frame at the wheel-centroid/axle origin.  The fixed
// relationship is source-derived from MODEL_FRAME and is presentation-only.
const MODEL_ORIGIN_IN_URDF_M = Object.freeze(MODEL_FRAME.originInUrdfM.map(Number));
const URDF_ROOT_FROM_MODEL_ORIGIN_THREE_MM = Object.freeze([
  -MODEL_ORIGIN_IN_URDF_M[1] * 1000,
  -MODEL_ORIGIN_IN_URDF_M[2] * 1000,
  -MODEL_ORIGIN_IN_URDF_M[0] * 1000,
]);
const URDF_TO_MODEL_THREE_YAW_RAD = -Math.PI / 2;

function toThreePosition(positionM = [0, 0, 0]) {
  return new THREE.Vector3(Number(positionM[0]) * 1000, Number(positionM[2]) * 1000, -Number(positionM[1]) * 1000);
}
function toThreeQuaternion(quaternionWxyz = [1, 0, 0, 0]) {
  const [w, x, y, z] = quaternionWxyz.map(Number);
  const physical = new THREE.Quaternion(x, y, z, w).normalize();
  const basis = new THREE.Matrix4().makeRotationX(-Math.PI / 2);
  const inverse = basis.clone().invert();
  const rotation = new THREE.Matrix4().makeRotationFromQuaternion(physical);
  return new THREE.Quaternion().setFromRotationMatrix(basis.clone().multiply(rotation).multiply(inverse)).normalize();
}
function box(widthM, heightM, depthM, material) {
  return new THREE.Mesh(new THREE.BoxGeometry(widthM * 1000, heightM * 1000, depthM * 1000), material);
}
function cylinder(radiusM, heightM, material, segments = 28) {
  return new THREE.Mesh(new THREE.CylinderGeometry(radiusM * 1000, radiusM * 1000, heightM * 1000, segments), material);
}
function canonicalArmState(observation) {
  const state = {};
  for (const [jointId, mapping] of Object.entries(LEKIWI_CANONICAL_PRESENTATION_MAP.joints)) {
    const value = Number(observation?.joints?.[jointId]?.positionRad);
    if (!Number.isFinite(value)) continue;
    // Presentation-only source reconciliation: the canonical mesh is baked from the official
    // LeKiwi URDF, while the physical arm uses the pinned Menagerie SO-ARM101 convention.
    state[`${jointId}.pos`] = (mapping.sign * value + mapping.offsetRad) * RAD_TO_DEG;
  }
  const grip = Number(observation?.joints?.arm_gripper?.positionRad);
  if (Number.isFinite(grip)) {
    const g = LEKIWI_CANONICAL_PRESENTATION_MAP.gripper;
    const denominator = g.physicalOpenRad - g.physicalClosedRad;
    const closedRatio = THREE.MathUtils.clamp((g.physicalOpenRad - grip) / denominator, 0, 1);
    state['arm_gripper.pos'] = THREE.MathUtils.lerp(g.canonicalOpenValue, g.canonicalCloseValue, closedRatio);
  }
  return state;
}
function applyCanonicalBaseTransform(rig, observation) {
  const base = observation?.bodies?.lekiwi_base;
  if (!rig || !base?.positionM || !base?.quaternionWxyz) return false;
  const baseThreeQuaternion = toThreeQuaternion(base.quaternionWxyz);
  const offset = new THREE.Vector3(...URDF_ROOT_FROM_MODEL_ORIGIN_THREE_MM).applyQuaternion(baseThreeQuaternion);
  const frameCorrection = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(0, 1, 0), URDF_TO_MODEL_THREE_YAW_RAD,
  );
  rig.root.position.copy(toThreePosition(base.positionM)).add(offset);
  rig.root.quaternion.copy(baseThreeQuaternion).multiply(frameCorrection).normalize();
  rig.root.updateMatrixWorld(true);
  return true;
}

function applyCanonicalWheelState(rig, observation) {
  if (!rig) return;
  for (const wheelId of WHEEL_ORDER) {
    const positionRad = Number(observation?.joints?.[wheelId]?.positionRad);
    const group = rig.groups?.[wheelId];
    const joint = group?.userData?.joint;
    if (!Number.isFinite(positionRad) || !group || !joint) continue;
    const axis = new THREE.Vector3().fromArray(joint.axis || [0, 1, 0]);
    if (axis.lengthSq() < 1e-9) continue;
    axis.normalize();
    // The physical wheel joints use the audited LeRobot-positive axes, which are anti-parallel
    // to the pinned URDF axes used by the canonical visual.  Reverse only the presentation angle.
    const motion = new THREE.Quaternion().setFromAxisAngle(axis, -positionRad);
    group.quaternion.copy(group.userData.baseQuaternion).multiply(motion).normalize();
  }
  rig.root.updateMatrixWorld(true);
}

function finiteArmTargets(targetsRad) {
  if (!targetsRad || typeof targetsRad !== 'object' || Array.isArray(targetsRad)) throw new TypeError('targetsRad must be an object');
  const allowed = new Set(LEKIWI_COURIER_PACKAGE.actuators.filter((item) => item.command === 'position-rad').map((item) => item.jointId));
  const clean = {};
  for (const [jointId, raw] of Object.entries(targetsRad)) {
    if (!allowed.has(jointId)) throw new Error(`Unknown physical LeKiwi arm joint ${jointId}`);
    const value = Number(raw);
    if (!Number.isFinite(value)) throw new TypeError(`Physical LeKiwi target ${jointId} must be finite radians`);
    clean[jointId] = value;
  }
  if (!Object.keys(clean).length) throw new TypeError('targetsRad must contain at least one arm joint target');
  return clean;
}

export class LeKiwiPhysicalSimulator {
  constructor(canvas) {
    this.canvas = canvas;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xb4bcc0);
    this.camera = new THREE.PerspectiveCamera(45, 1, 1, 8000);
    this.camera.position.set(900, 780, 900);
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.shadowMap.enabled = true;
    this.controls = new OrbitControls(this.camera, this.canvas);
    this.controls.enableDamping = true;
    this.controls.target.set(200, 120, -200);

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x30404a, 1.3));
    const key = new THREE.DirectionalLight(0xffffff, 1.9);
    key.position.set(600, 1200, 600);
    key.castShadow = true;
    this.scene.add(key);
    const grid = new THREE.GridHelper(2000, 40, 0x334155, 0x4b5563);
    grid.userData.presentationOnly = true;
    this.scene.add(grid);

    this.workcellRoot = new THREE.Group();
    this.workcellRoot.name = 'lekiwi-physical-workcell-presentation';
    this.scene.add(this.workcellRoot);
    this.robotRoot = new THREE.Group();
    this.robotRoot.name = 'lekiwi-canonical-mobile-presentation';
    this.scene.add(this.robotRoot);

    this.canonicalRig = null;
    this.objectMeshes = new Map();
    this.markers = [];
    this.session = null;
    this.unsubscribeSession = null;
    this.evaluator = null;
    this.lastObservation = null;
    this.presentationDirty = false;
    this.ready = false;
    this.disposed = false;
    this.highContrast = true;
    this.sessionSequence = 0;
    this.#buildPresentation();
    this.setHighContrastScene(true);
    this.resize();
  }

  async setScenario(profileId, scenario) {
    if (profileId !== 'lekiwi') throw new Error('LeKiwi physical simulator only accepts the lekiwi profile');
    if (scenario?.simulationMode !== 'physical_mujoco' || scenario?.physicalSceneId !== LEKIWI_COURIER_SCENE.id) {
      throw new Error('LeKiwi physical simulator requires the Phase 5B beaker-courier scene');
    }
    await this.#ensureCanonicalPresentation();
    await this.#disposeSession();
    this.evaluator = new LeKiwiCourierEvaluator();
    await this.#createSession();
    this.ready = true;
    this.canvas.dataset.simulatorBackend = 'browser-mujoco';
    this.canvas.dataset.simulationAuthority = 'physics-session';
    this.canvas.dataset.physicalSceneId = LEKIWI_COURIER_SCENE.id;
    this.canvas.dataset.modelPackageId = LEKIWI_COURIER_SCENE.modelPackage;
    this.canvas.dataset.presentationGroundColor = '#687378';
    this.canvas.dataset.lekiwiBaseIntegration = 'mujoco-free-body';
    this.canvas.dataset.lekiwiWheelContact = 'explicit-passive-rollers';
    this.fit();
    return true;
  }

  async reset() {
    this.#assertNotDisposed();
    this.ready = false;
    this.evaluator = new LeKiwiCourierEvaluator();
    if (!this.session) await this.#createSession();
    else {
      let diagnostics = null;
      try { diagnostics = await this.session.getDiagnostics(); } catch {}
      if (!diagnostics?.loaded) { await this.#disposeSession(); await this.#createSession(); }
      else await this.session.reset({ reason: 'explicit-user-reset' });
    }
    this.ready = true;
    return true;
  }

  renderFrame() {
    if (this.disposed) return;
    if (this.presentationDirty && this.lastObservation) {
      this.#applyObservation(this.lastObservation);
      this.presentationDirty = false;
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
  resize() {
    if (this.disposed) return false;
    const width = Math.max(1, this.canvas.clientWidth || this.canvas.width || 640);
    const height = Math.max(1, this.canvas.clientHeight || this.canvas.height || 480);
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    if (this.canvas.width !== Math.floor(width * dpr) || this.canvas.height !== Math.floor(height * dpr)) this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    return true;
  }
  fit() {
    this.controls.target.set(200, 120, -200);
    this.camera.position.set(900, 780, 900);
    this.camera.updateProjectionMatrix();
    this.controls.update();
    return true;
  }
  setHighContrastScene(value) {
    this.highContrast = Boolean(value);
    for (const marker of this.markers) marker.visible = this.highContrast;
    this.canvas.dataset.highContrastScene = String(this.highContrast);
    this.canvas.dataset.highContrastPerimeterCount = this.highContrast ? String(this.markers.length) : '0';
    this.canvas.dataset.presentationGroundColor = '#687378';
    return this.highContrast;
  }
  isHighContrastSceneEnabled() { return this.highContrast; }
  isReady() { return Boolean(this.ready && this.session && this.lastObservation && this.session.sceneRevision && this.session.robotId); }
  getPhysicalSession() { return this.session; }
  getPhysicalAuthorityToken() {
    if (!this.isReady()) return null;
    return Object.freeze({ sessionId: this.session.sessionId, epoch: this.session.epoch, sceneRevision: this.session.sceneRevision, robotId: this.session.robotId, simulationTimeSeconds: this.lastObservation.simulationTimeSeconds });
  }
  getTaskEvaluation() { return this.evaluator?.snapshot?.() || null; }
  getBasePose() { return this.lastObservation ? basePoseFromObservation(this.lastObservation) : null; }
  getPresentationAudit() {
    return Object.freeze({
      source: canonicalVisualProvenance('lekiwi'),
      physicalAuthority: 'MuJoCo PhysicsSession only',
      baseTransformSource: 'observed MuJoCo lekiwi_base free-body pose',
      jointPresentationSource: 'observed MuJoCo joint positions',
      payloadTransformSource: 'observed MuJoCo empty_beaker free-body pose',
      wheelTransformSource: 'observed MuJoCo wheel joint positions mapped onto the anti-parallel pinned URDF visual axes',
      visualRootFrameSource: MODEL_FRAME.urdfToModel,
      visualRootOffsetThreeMm: [...URDF_ROOT_FROM_MODEL_ORIGIN_THREE_MM],
      visualRootYawCorrectionRad: URDF_TO_MODEL_THREE_YAW_RAD,
      rendererIntegratesBase: false,
      observationBatchSteps: LEKIWI_OBSERVATION_BATCH_STEPS,
      observationPeriodSeconds: LEKIWI_OBSERVATION_BATCH_STEPS * Number(this.lastObservation?.engine?.timestepSeconds || 0.002),
    });
  }
  getPresentationAlignment() {
    if (!this.canonicalRig || !this.lastObservation) return null;
    if (this.presentationDirty) {
      this.#applyObservation(this.lastObservation);
      this.presentationDirty = false;
    }
    const pairs = [
      ['shoulder_pan', 'arm_shoulder'],
      ['shoulder_lift', 'arm_upper'],
      ['elbow_flex', 'arm_lower'],
      ['wrist_flex', 'arm_wrist'],
      ['wrist_roll', 'arm_gripper_body'],
    ];
    const armPivotErrorsMm = {};
    const visualPivotsMm = {};
    const physicalPivotsMm = {};
    for (const [visualId, bodyId] of pairs) {
      const visual = this.canonicalRig.getWorldPosition(visualId);
      const body = this.lastObservation.bodies?.[bodyId];
      if (!visual || !body?.positionM) continue;
      const physical = toThreePosition(body.positionM);
      visualPivotsMm[visualId] = visual.toArray();
      physicalPivotsMm[bodyId] = physical.toArray();
      armPivotErrorsMm[visualId] = visual.distanceTo(physical);
    }
    const errors = Object.values(armPivotErrorsMm);
    const beakerMesh = this.objectMeshes.get('empty_beaker');
    const beakerBody = this.lastObservation.bodies?.empty_beaker;
    const beakerPhysical = beakerBody?.positionM ? toThreePosition(beakerBody.positionM) : null;
    const visualWrist = this.canonicalRig.getWorldPosition('wrist_roll');
    return Object.freeze({
      armPivotErrorsMm,
      maxArmPivotErrorMm: errors.length ? Math.max(...errors) : null,
      visualPivotsMm,
      physicalPivotsMm,
      beakerErrorMm: beakerMesh && beakerPhysical ? beakerMesh.position.distanceTo(beakerPhysical) : null,
      visualWristToBeakerMm: visualWrist && beakerMesh ? visualWrist.distanceTo(beakerMesh.position) : null,
      wheelPositionsRad: Object.fromEntries(WHEEL_ORDER.map((id) => [id, Number(this.lastObservation.joints?.[id]?.positionRad ?? 0)])),
      wheelVisualQuaternions: Object.fromEntries(WHEEL_ORDER.map((id) => [id, this.canonicalRig.groups?.[id]?.quaternion?.toArray?.() || null])),
      rootPositionMm: this.canonicalRig.root.position.toArray(),
      rootQuaternion: this.canonicalRig.root.quaternion.toArray(),
    });
  }
  getTelemetry() {
    const observation = this.lastObservation;
    if (!observation) return {};
    const pose = basePoseFromObservation(observation);
    const out = { simulation_time_s: Number(observation.simulationTimeSeconds) };
    if (pose) Object.assign(out, { base_x_m: pose.xM, base_y_m: pose.yM, base_yaw_rad: pose.yawRad, base_speed_m_s: pose.speedMS, base_yaw_rate_rad_s: pose.yawRateRadS });
    for (const wheelId of WHEEL_ORDER) {
      const joint = observation.joints?.[wheelId];
      if (!joint) continue;
      out[`${wheelId}_rad_s`] = Number(joint.velocityRadS);
      out[`${wheelId}_target_rad_s`] = Number(joint.targetVelocityRadS);
      out[`${wheelId}_effort_nm`] = Number(joint.effortNm);
    }
    for (const jointId of ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex', 'arm_wrist_flex', 'arm_wrist_roll', 'arm_gripper']) {
      const joint = observation.joints?.[jointId];
      if (joint) out[`${jointId}_rad`] = Number(joint.positionRad);
    }
    const beaker = observation.bodies?.empty_beaker;
    if (beaker?.positionM) {
      out.beaker_x_m = Number(beaker.positionM[0]);
      out.beaker_y_m = Number(beaker.positionM[1]);
      out.beaker_z_m = Number(beaker.positionM[2]);
      out.beaker_linear_speed_m_s = Math.hypot(...(beaker.linearVelocityMS || [0, 0, 0]).map(Number));
    }
    return out;
  }
  getContacts() {
    const observation = this.lastObservation;
    const evaluation = this.getTaskEvaluation();
    if (!observation || !evaluation) return {};
    return {
      contact_count: Number(observation.contactCount || 0),
      grasp_seen: evaluation.graspSeen,
      current_grasp: evaluation.currentGrasp,
      support_while_held_seen: evaluation.supportWhileHeldSeen,
      current_support_contact: evaluation.currentSupportContact,
      lift_seen: evaluation.liftSeen,
      carry_seen: evaluation.carrySeen,
      release_seen: evaluation.releaseSeen,
      settled: evaluation.settled,
      service_stop_reached: evaluation.serviceStopReached,
      home_returned: evaluation.homeReturned,
      restricted_violation: evaluation.restrictedViolation,
      task_success: evaluation.success,
    };
  }
  getState() {
    return this.lastObservation
      ? { observation: structuredClone(this.lastObservation), evaluation: this.getTaskEvaluation(), authority: this.getPhysicalAuthorityToken(), basePose: this.getBasePose() }
      : null;
  }

  async applyAction() {
    throw new Error('Legacy LeKiwi .pos/kinematic base replay is disabled in the physical workspace. Use robobuddy.sim.v1 radians or the bounded LeKiwi physical WebMCP schema.');
  }

  /** Public LeRobot-shaped chassis command routed through the pinned Kiwi mapping. */
  async applyChassisVelocity(action = {}, { maxSteps = 2000, advanceSeconds = 0 } = {}) {
    this.#assertReady();
    const body = publicActionToBodyCommand(action);
    const { targetsRadS, saturated, scale } = wheelTargetsForChassis(body, { maxWheelRadS: MAX_WHEEL_RAD_S });
    if (!Number.isInteger(maxSteps) || maxSteps < 1 || maxSteps > 20000) throw new RangeError('maxSteps must be an integer from 1 to 20000');
    const advance = Number(advanceSeconds);
    if (!Number.isFinite(advance) || advance < 0 || advance > MAX_WEBMCP_ADVANCE_SECONDS) throw new RangeError(`advanceSeconds must be between 0 and ${MAX_WEBMCP_ADVANCE_SECONDS}`);
    const accepted = await this.session.sendCommand({ type: 'set_wheel_velocity_targets', targetsRadS: { ...targetsRadS } }, { maxSteps });
    let observation = accepted.observation;
    if (advance > 0) observation = await this.advanceTime(advance);
    return {
      schemaVersion: 'robobuddy.lekiwi.physical.v1',
      status: accepted.status,
      commandId: accepted.commandId,
      requestedChassisVelocity: { 'x.vel': body.x, 'y.vel': body.y, 'theta.vel': Number(action['theta.vel'] ?? 0) },
      acceptedWheelTargetsRadS: { ...targetsRadS },
      saturated,
      saturationScale: scale,
      observation: structuredClone(observation),
      basePose: basePoseFromObservation(observation),
      taskEvaluation: this.getTaskEvaluation(),
    };
  }

  async applyArmTargets(targetsRad, { maxSteps = 2000, advanceSeconds = 0 } = {}) {
    this.#assertReady();
    const clean = finiteArmTargets(targetsRad);
    if (!Number.isInteger(maxSteps) || maxSteps < 1 || maxSteps > 20000) throw new RangeError('maxSteps must be an integer from 1 to 20000');
    const advance = Number(advanceSeconds);
    if (!Number.isFinite(advance) || advance < 0 || advance > MAX_WEBMCP_ADVANCE_SECONDS) throw new RangeError(`advanceSeconds must be between 0 and ${MAX_WEBMCP_ADVANCE_SECONDS}`);
    const accepted = await this.session.sendCommand({ type: 'set_joint_targets', targetsRad: clean }, { maxSteps });
    let observation = accepted.observation;
    if (advance > 0) observation = await this.advanceTime(advance);
    return {
      schemaVersion: 'robobuddy.lekiwi.physical.v1',
      status: accepted.status,
      commandId: accepted.commandId,
      acceptedTargetsRad: clean,
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
    if (steps < 1 || Math.abs(duration - steps * dt) > STEP_ALIGNMENT_TOLERANCE_SECONDS) throw new RangeError(`Physical advance duration ${duration}s must align to the ${dt}s MuJoCo timestep`);
    return this.session.advanceSteps(steps);
  }

  /**
   * One bounded closed-loop drive interval. The controller only proposes a chassis velocity from
   * the observed base pose; the wheels do the work and MuJoCo decides the result.
   */
  async driveTowards(waypoint, { limits = LEKIWI_DRIVE_LIMITS } = {}) {
    this.#assertReady();
    const pose = this.getBasePose();
    if (!pose) throw new Error('No authoritative LeKiwi base pose is available');
    if (waypointReached(pose, waypoint, limits)) {
      await this.applyChassisVelocity({ 'x.vel': 0, 'y.vel': 0, 'theta.vel': 0 }, { maxSteps: 200 });
      return { arrived: true, pose, command: null };
    }
    const command = chassisVelocityCommand(pose, waypoint, limits);
    const result = await this.applyChassisVelocity(
      { 'x.vel': command.x, 'y.vel': command.y, 'theta.vel': command.thetaRadS * 180 / Math.PI },
      { maxSteps: 2000, advanceSeconds: LEKIWI_COURIER_CONTROLLER.controllerPeriodSeconds },
    );
    return { arrived: false, pose, command, result };
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
    } catch { return false; }
  }
  dispose() {
    if (this.disposed) return;
    this.disposed = true;
    this.ready = false;
    void this.#disposeSession();
    if (this.canonicalRig) {
      this.robotRoot.remove(this.canonicalRig.root);
      this.canonicalRig.dispose();
      this.canonicalRig = null;
    }
    this.controls?.dispose?.();
    this.renderer?.dispose?.();
  }

  async #ensureCanonicalPresentation() {
    if (this.canonicalRig) return this.canonicalRig;
    const rig = await CanonicalRobotRig.load('lekiwi');
    rig.root.userData.presentationAuthority = 'observed MuJoCo base pose and joints only';
    this.robotRoot.add(rig.root);
    this.canonicalRig = rig;
    return rig;
  }
  async #createSession() {
    const session = new PhysicsSession(
      new BrowserMuJoCoBackend({ workerUrl: new URL('./lekiwi-mujoco-worker.js', import.meta.url) }),
      { sessionId: `ide-lekiwi-v1-${++this.sessionSequence}`, observationBatchSteps: LEKIWI_OBSERVATION_BATCH_STEPS },
    );
    this.session = session;
    this.unsubscribeSession = session.subscribe(({ observation }) => this.#consumeObservation(observation));
    await session.loadScene(structuredClone(LEKIWI_COURIER_SCENE));
  }
  async #disposeSession() {
    this.unsubscribeSession?.();
    this.unsubscribeSession = null;
    if (this.session) this.session.dispose();
    this.session = null;
    this.lastObservation = null;
    this.presentationDirty = false;
  }
  #consumeObservation(observation) {
    if (!observation || this.disposed) return;
    this.lastObservation = structuredClone(observation);
    this.evaluator?.observe(observation);
    // Presentation is pulled by the render loop from the newest authoritative sample. Rendering
    // never advances physics and never writes back into the plant.
    this.presentationDirty = true;
    const evaluation = this.getTaskEvaluation();
    const pose = basePoseFromObservation(observation);
    this.canvas.dataset.simulationClockS = String(Number(observation.simulationTimeSeconds || 0));
    if (pose) {
      this.canvas.dataset.lekiwiBaseXM = pose.xM.toFixed(5);
      this.canvas.dataset.lekiwiBaseYM = pose.yM.toFixed(5);
      this.canvas.dataset.lekiwiBaseYawRad = pose.yawRad.toFixed(5);
      this.canvas.dataset.lekiwiBaseSpeedMS = pose.speedMS.toFixed(5);
    }
    this.canvas.dataset.physicalTaskSuccess = String(Boolean(evaluation?.success));
    this.canvas.dataset.physicalTaskContact = String(Boolean(evaluation?.graspSeen));
    this.canvas.dataset.physicalTaskLift = String(Boolean(evaluation?.liftSeen));
    this.canvas.dataset.physicalTaskCarry = String(Boolean(evaluation?.carrySeen));
    this.canvas.dataset.physicalTaskRelease = String(Boolean(evaluation?.releaseSeen));
    this.canvas.dataset.physicalTaskSettled = String(Boolean(evaluation?.settled));
    this.canvas.dataset.physicalTaskHome = String(Boolean(evaluation?.homeReturned));
  }
  #applyObservation(observation) {
    if (this.canonicalRig) {
      // Arm joints, base frame and wheel spin are all presentation consumers of the same
      // authoritative MuJoCo observation.  No presentation transform is fed back into physics.
      this.canonicalRig.applyPhysicalState(canonicalArmState(observation));
      applyCanonicalBaseTransform(this.canonicalRig, observation);
      applyCanonicalWheelState(this.canonicalRig, observation);
    }
    for (const [objectId, mesh] of this.objectMeshes) {
      const body = observation.bodies?.[objectId];
      if (!body?.positionM) continue;
      mesh.position.copy(toThreePosition(body.positionM));
      if (body.quaternionWxyz) mesh.quaternion.copy(toThreeQuaternion(body.quaternionWxyz));
    }
  }

  #buildPresentation() {
    const benchMaterial = new THREE.MeshStandardMaterial({ color: PRESENTATION_GROUND_COLOR, roughness: 0.74, metalness: 0.04 });
    const darkMaterial = new THREE.MeshStandardMaterial({ color: 0x353a41, roughness: 0.6, metalness: 0.1 });

    const worktop = box(0.290, 0.024, 0.290, benchMaterial);
    worktop.position.copy(toThreePosition([LEKIWI_WORKCELL.worktopCenterXYM[0], LEKIWI_WORKCELL.worktopCenterXYM[1], 0.199]));
    worktop.name = 'visual-transfer-worktop';
    this.workcellRoot.add(worktop);

    const backPanel = box(0.032, 0.199, 0.250, darkMaterial);
    backPanel.position.copy(toThreePosition([0.206, -0.390, 0.0995]));
    backPanel.name = 'visual-transfer-back-panel';
    this.workcellRoot.add(backPanel);

    const markerMaterial = new THREE.MeshBasicMaterial({ color: 0x22c55e, transparent: true, opacity: 0.42, depthWrite: false });
    const deliveryMarker = box(0.090, 0.001, 0.090, markerMaterial);
    deliveryMarker.position.copy(toThreePosition([LEKIWI_WORKCELL.deliveryXYM[0], LEKIWI_WORKCELL.deliveryXYM[1], 0.2115]));
    deliveryMarker.userData.presentationOnly = true;
    deliveryMarker.name = 'visual-delivery-zone';
    this.markers.push(deliveryMarker);
    this.workcellRoot.add(deliveryMarker);

    const homeMaterial = new THREE.MeshBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.32, depthWrite: false });
    const homeMarker = cylinder(0.09, 0.001, homeMaterial);
    homeMarker.position.copy(toThreePosition([LEKIWI_WORKCELL.homeXYM[0], LEKIWI_WORKCELL.homeXYM[1], 0.0005]));
    homeMarker.userData.presentationOnly = true;
    homeMarker.name = 'visual-home-base';
    this.markers.push(homeMarker);
    this.workcellRoot.add(homeMarker);

    const restrictedMaterial = new THREE.MeshBasicMaterial({ color: 0xef4444, transparent: true, opacity: 0.28, depthWrite: false });
    const restricted = cylinder(LEKIWI_WORKCELL.restrictedStopRadiusM, 0.001, restrictedMaterial);
    restricted.position.copy(toThreePosition([LEKIWI_WORKCELL.restrictedStopXYM[0], LEKIWI_WORKCELL.restrictedStopXYM[1], 0.0005]));
    restricted.userData.presentationOnly = true;
    restricted.name = 'visual-restricted-stop';
    this.markers.push(restricted);
    this.workcellRoot.add(restricted);

    const glass = new THREE.MeshStandardMaterial({ color: 0x8ecae6, transparent: true, opacity: 0.72, roughness: 0.24 });
    const beaker = new THREE.Group();
    const wall = new THREE.Mesh(new THREE.CylinderGeometry(37, 37, 80, 24, 1, true), glass);
    wall.position.y = 40;
    const bottom = cylinder(0.037, 0.003, glass);
    bottom.position.y = 1.5;
    beaker.add(wall, bottom);
    beaker.name = 'visual-empty-beaker';
    this.objectMeshes.set('empty_beaker', beaker);
    this.workcellRoot.add(beaker);
    this.setHighContrastScene(this.highContrast);
  }
  #assertReady() { this.#assertNotDisposed(); if (!this.isReady()) throw new Error('LeKiwi physical session is not ready'); }
  #assertNotDisposed() { if (this.disposed) throw new Error('LeKiwi physical simulator is disposed'); }
}
