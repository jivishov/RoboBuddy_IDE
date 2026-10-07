import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js';
import { OrbitControls } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/controls/OrbitControls.js';
import { CanonicalRobotRig, canonicalVisualProvenance } from '../canonical-rig.js';
import { BrowserMuJoCoBackend } from './browser-mujoco-backend.js';
import { PhysicsSession } from './session.js';
import { UNITREE_G1_EXTERNAL_OBJECT, UNITREE_G1_FREEBASE_PACKAGE, UNITREE_G1_STAND_PELVIS_Z_M } from './unitree-g1-model-package.js';
import { UNITREE_G1_FREEBASE_SCENE, UnitreeG1StandEvaluator } from './unitree-g1-scene.js';
import {
  G1_CONTROLLERS, G1_MAX_KD, G1_MAX_KP, G1_OBSERVATION_INTERVAL_SECONDS, G1_PHYSICS_TIMESTEP_SECONDS,
  G1_STAND_CONTROLLER_PROFILES, UNITREE_FIXSTAND_RAMP_SECONDS,
} from './unitree-g1-controller.js';
import { G1_BODY_ORDER, G1_JOINT_ORDER, G1_TOTAL_MASS_KG } from './unitree-g1-source-audit.js';

const RAD_TO_DEG = 180 / Math.PI;
const MAX_ADVANCE_SECONDS = 20;
const MAX_WEBMCP_ADVANCE_SECONDS = 2;
// Declared observation cadence, in authoritative MuJoCo steps. The shortest physical event the
// standing gate depends on is a foot leaving the floor, which at the source posture takes tens of
// milliseconds; ten physics steps (20 ms at the pinned 0.002 s timestep) resolve that with margin.
// It is a software observation parameter and not a hardware sensor rate.
const G1_OBSERVATION_BATCH_STEPS = Math.round(G1_OBSERVATION_INTERVAL_SECONDS / G1_PHYSICS_TIMESTEP_SECONDS);
const PRESENTATION_FLOOR_COLOR = 0x687378;

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

/**
 * The canonical Unitree G1 rig takes joint values in degrees. This is the only conversion between
 * the physical and presentation conventions, it is applied at the rendering boundary, and it never
 * feeds anything back into MuJoCo.
 */
// The source names one child body after the link rather than after its joint: the body below
// waist_pitch_joint is torso_link. Naming it explicitly keeps all 29 joints in the alignment check
// instead of silently skipping the torso.
const JOINT_BODY_OVERRIDES = Object.freeze({ waist_pitch_joint: 'torso_link' });

export function canonicalJointStateDeg(observation) {
  const state = {};
  for (const jointId of G1_JOINT_ORDER) {
    const value = Number(observation?.joints?.[jointId]?.positionRad);
    if (Number.isFinite(value)) state[jointId] = value * RAD_TO_DEG;
  }
  return state;
}

function validatedTargets(targetsRad) {
  if (!targetsRad || typeof targetsRad !== 'object' || Array.isArray(targetsRad)) throw new TypeError('targetsRad must be an object of joint name to radians');
  const clean = {};
  for (const [jointId, raw] of Object.entries(targetsRad)) {
    if (!G1_JOINT_ORDER.includes(jointId)) throw new RangeError(`Unknown Unitree G1 joint: ${jointId}`);
    const value = Number(raw);
    if (!Number.isFinite(value)) throw new TypeError(`Target for ${jointId} must be finite radians`);
    clean[jointId] = value;
  }
  if (!Object.keys(clean).length) throw new TypeError('targetsRad must contain at least one joint target');
  return clean;
}

export class UnitreeG1PhysicalSimulator {
  constructor(canvas) {
    this.canvas = canvas;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xb4bcc0);
    this.camera = new THREE.PerspectiveCamera(45, 1, 1, 12000);
    this.camera.position.set(1900, 1500, 2200);
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.shadowMap.enabled = true;
    this.controls = new OrbitControls(this.camera, this.canvas);
    this.controls.enableDamping = true;
    this.controls.target.set(0, 700, 0);

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x30404a, 1.25));
    const key = new THREE.DirectionalLight(0xffffff, 1.8);
    key.position.set(1200, 2400, 1200);
    key.castShadow = true;
    this.scene.add(key);
    const grid = new THREE.GridHelper(4000, 40, 0x334155, 0x4b5563);
    grid.userData.presentationOnly = true;
    this.scene.add(grid);

    this.workcellRoot = new THREE.Group();
    this.workcellRoot.name = 'unitree-g1-physical-workcell-presentation';
    this.scene.add(this.workcellRoot);
    this.robotRoot = new THREE.Group();
    this.robotRoot.name = 'unitree-g1-canonical-physical-presentation';
    this.scene.add(this.robotRoot);

    this.canonicalRig = null;
    this.objectMeshes = new Map();
    this.session = null;
    this.unsubscribeSession = null;
    this.evaluator = new UnitreeG1StandEvaluator();
    this.lastObservation = null;
    this.presentationDirty = false;
    this.renderedFrames = 0;
    this.ready = false;
    this.disposed = false;
    this.highContrast = true;
    this.sessionSequence = 0;
    this.standEngagedAtSeconds = null;
    this.#buildPresentation();
    this.setHighContrastScene(true);
    this.resize();
  }

  async setScenario(profileId, scenario) {
    if (profileId !== 'unitree') throw new Error('Unitree G1 physical simulator only accepts the unitree profile');
    if (scenario?.simulationMode !== 'physical_mujoco' || scenario?.physicalSceneId !== UNITREE_G1_FREEBASE_SCENE.id) {
      throw new Error('Unitree G1 physical simulator requires the Phase 5D free-base physical scene');
    }
    await this.#ensureCanonicalPresentation();
    await this.#disposeSession();
    this.evaluator = new UnitreeG1StandEvaluator();
    this.standEngagedAtSeconds = null;
    await this.#createSession();
    this.ready = true;
    this.canvas.dataset.simulatorBackend = 'browser-mujoco';
    this.canvas.dataset.simulationAuthority = 'physics-session';
    this.canvas.dataset.physicalSceneId = UNITREE_G1_FREEBASE_SCENE.id;
    this.canvas.dataset.physicalSceneRevision = UNITREE_G1_FREEBASE_SCENE.revision;
    this.canvas.dataset.modelPackageId = UNITREE_G1_FREEBASE_SCENE.modelPackage;
    this.canvas.dataset.unitreeG1RootMode = 'free-base';
    this.canvas.dataset.unitreeG1Walking = 'unsupported';
    this.canvas.dataset.unitreeG1Hands = 'fixed-rubber-passive';
    this.canvas.dataset.presentationGroundColor = '#687378';
    this.fit();
    return true;
  }

  async reset() {
    this.#assertNotDisposed();
    this.ready = false;
    this.evaluator = new UnitreeG1StandEvaluator();
    this.standEngagedAtSeconds = null;
    if (!this.session) await this.#createSession();
    else {
      let diagnostics = null;
      try { diagnostics = await this.session.getDiagnostics(); } catch { diagnostics = null; }
      if (!diagnostics?.loaded) { await this.#disposeSession(); await this.#createSession(); }
      else await this.session.reset({ reason: 'explicit-user-reset' });
    }
    this.ready = true;
    return true;
  }

  renderFrame() {
    if (this.disposed) return;
    this.renderedFrames += 1;
    if (this.presentationDirty && this.lastObservation) {
      this.#applyObservation(this.lastObservation);
      this.presentationDirty = false;
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this.canvas.dataset.renderedFrames = String(this.renderedFrames);
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
    this.controls.target.set(0, 700, 0);
    this.camera.position.set(1900, 1500, 2200);
    this.controls.update();
    return true;
  }

  setHighContrastScene(value) {
    this.highContrast = Boolean(value);
    this.scene.background = new THREE.Color(this.highContrast ? 0xb4bcc0 : 0x1b222c);
    return this.highContrast;
  }
  isHighContrastSceneEnabled() { return this.highContrast; }
  isReady() { return Boolean(this.ready && this.session && this.lastObservation && this.session.sceneRevision && this.session.robotId); }
  getPhysicalSession() { return this.session; }
  getPhysicalAuthorityToken() {
    if (!this.isReady()) return null;
    return { sessionId: this.session.sessionId, epoch: this.session.epoch, sceneRevision: this.session.sceneRevision, robotId: this.session.robotId };
  }
  getTaskEvaluation() { return this.evaluator?.snapshot?.() || null; }

  /** Ground-truth state, as the live Python and WebMCP surfaces publish it. */
  getState() {
    const observation = this.lastObservation;
    if (!observation) return null;
    const joints = {};
    for (const jointId of G1_JOINT_ORDER) {
      const joint = observation.joints?.[jointId];
      if (!joint) continue;
      joints[jointId] = {
        requested_target_rad: joint.requestedTargetRad,
        accepted_target_rad: joint.acceptedTargetRad,
        position_rad: joint.positionRad,
        velocity_rad_s: joint.velocityRadS,
        effort_nm: joint.effortNm,
        effort_limit_nm: joint.effortLimitNm,
        velocity_limit_rad_s: joint.velocityLimitRadS,
        joint_range_rad: joint.jointRangeRad,
        command_bounded: joint.commandBounded,
        kp: joint.kp,
        kd: joint.kd,
      };
    }
    const classes = observation.contactClasses || {};
    return {
      simulation_time_s: observation.simulationTimeSeconds,
      root: {
        mode: observation.root?.mode ?? null,
        position_m: observation.root?.positionM ?? null,
        quaternion_wxyz: observation.root?.quaternionWxyz ?? null,
        linear_velocity_m_s: observation.root?.linearVelocityMS ?? null,
        angular_velocity_rad_s: observation.root?.angularVelocityRadS ?? null,
        frame: observation.root?.frame ?? null,
        upright_z: observation.root?.uprightZ ?? null,
        tilt_rad: observation.root?.tiltRad ?? null,
      },
      joints,
      foot_contacts: {
        left: (classes.leftFootFloor || []).map((entry) => entry.geoms),
        right: (classes.rightFootFloor || []).map((entry) => entry.geoms),
      },
      contacts: {
        non_foot_ground: (classes.otherBodyFloor || []).map((entry) => entry.geoms),
        self: (classes.robotSelf || []).map((entry) => entry.geoms),
        external_object: (classes.robotExternalObject || []).map((entry) => entry.geoms),
      },
      controller_mode: observation.controller?.id ?? null,
      controller_claim: observation.controller?.claim ?? null,
      actuation_enabled: observation.actuationEnabled,
      walking: 'unsupported',
    };
  }

  getContacts() {
    const classes = this.lastObservation?.contactClasses || {};
    return {
      count: this.lastObservation?.contactCount ?? 0,
      readable: Boolean(this.lastObservation?.contactsReadable),
      leftFootFloor: classes.leftFootFloor || [],
      rightFootFloor: classes.rightFootFloor || [],
      otherBodyFloor: classes.otherBodyFloor || [],
      robotSelf: classes.robotSelf || [],
      robotExternalObject: classes.robotExternalObject || [],
    };
  }

  getTelemetry() {
    const observation = this.lastObservation;
    const evaluation = this.getTaskEvaluation();
    return {
      backend: 'browser-mujoco',
      authority: 'physics-session',
      simulationTimeSeconds: observation?.simulationTimeSeconds ?? 0,
      rootMode: observation?.root?.mode ?? null,
      pelvisHeightM: observation?.root?.positionM?.[2] ?? null,
      pelvisTiltRad: observation?.root?.tiltRad ?? null,
      controllerId: observation?.controller?.id ?? null,
      actuationEnabled: observation?.actuationEnabled ?? null,
      standing: Boolean(evaluation?.standing),
      totalMassKg: G1_TOTAL_MASS_KG,
      standPelvisHeightM: UNITREE_G1_STAND_PELVIS_Z_M,
      renderedFrames: this.renderedFrames,
      modelPackage: UNITREE_G1_FREEBASE_PACKAGE.id,
      walking: 'unsupported',
    };
  }

  getPresentationAudit() {
    const rig = this.canonicalRig;
    return {
      canonical: canonicalVisualProvenance('unitree'),
      groundOffsetMm: rig ? Number(rig.root.userData.groundOffsetMm) : null,
      presentationAuthority: rig?.root?.userData?.presentationAuthority ?? null,
      jointCount: rig ? rig.jointDefs.filter((joint) => joint.jointId).length : null,
      drivesFromObservation: Boolean(this.lastObservation),
    };
  }

  /**
   * Frame-by-frame presentation alignment.
   *
   * The canonical rig's joint groups sit at the source joint pivots, which are the same points as
   * the MuJoCo child body origins, so their world positions must agree. This is the check that
   * catches a joint-order, sign or frame mistake between the physical model and the visual rig -
   * and it only ever reads; nothing here is written back into MuJoCo.
   */
  getPresentationAlignment() {
    const rig = this.canonicalRig;
    const observation = this.lastObservation;
    if (!rig || !observation?.bodies) return null;
    rig.root.updateMatrixWorld(true);
    const errors = {};
    let worst = 0;
    let worstBody = null;
    for (const jointId of G1_JOINT_ORDER) {
      const bodyId = JOINT_BODY_OVERRIDES[jointId] || `${jointId.replace(/_joint$/, '')}_link`;
      if (!G1_BODY_ORDER.includes(bodyId)) continue;
      const body = observation.bodies[bodyId];
      const group = rig.groups?.[jointId];
      if (!body?.positionM || !group) continue;
      const expected = toThreePosition(body.positionM);
      const actual = group.getWorldPosition(new THREE.Vector3());
      const errorMm = actual.distanceTo(expected);
      errors[bodyId] = Number(errorMm.toFixed(4));
      if (errorMm > worst) { worst = errorMm; worstBody = bodyId; }
    }
    const object = this.objectMeshes.get(UNITREE_G1_EXTERNAL_OBJECT);
    const objectBody = observation.bodies?.[UNITREE_G1_EXTERNAL_OBJECT];
    return {
      comparedBodies: Object.keys(errors).length,
      maxBodyErrorMm: Number(worst.toFixed(4)),
      worstBody,
      bodyErrorsMm: errors,
      objectErrorMm: object && objectBody?.positionM ? Number(object.position.distanceTo(toThreePosition(objectBody.positionM)).toFixed(6)) : null,
      pelvisVisualMm: rig.root.position.toArray().map((value) => Number(value.toFixed(4))),
      pelvisPhysicalM: observation.root?.positionM ?? null,
      jointDeg: Object.fromEntries(G1_JOINT_ORDER.map((jointId) => [jointId, Number((Number(observation.joints?.[jointId]?.positionRad ?? 0) * RAD_TO_DEG).toFixed(6))])),
      measuredJointRad: Object.fromEntries(G1_JOINT_ORDER.map((jointId) => [jointId, observation.joints?.[jointId]?.positionRad ?? null])),
      acceptedTargetRad: Object.fromEntries(G1_JOINT_ORDER.map((jointId) => [jointId, observation.joints?.[jointId]?.acceptedTargetRad ?? null])),
      simulationTimeSeconds: observation.simulationTimeSeconds,
      renderedFrames: this.renderedFrames,
    };
  }

  /**
   * Bounded joint targets. The simulator sends a bounded command and advances simulated time; it
   * never claims the joints reached the targets, and the caller reads the achieved state back.
   */
  async applyPhysicalTargets(targetsRad, { advanceSeconds = 0, maxSteps = 40000 } = {}) {
    this.#assertReady();
    const clean = validatedTargets(targetsRad);
    const result = await this.session.sendCommand({ type: 'set_joint_targets', targetsRad: clean }, { maxSteps });
    if (advanceSeconds > 0) await this.advanceTime(advanceSeconds);
    return result;
  }
  applyAction(targetsRad, options) { return this.applyPhysicalTargets(targetsRad, options); }

  /** The internal full low-level surface: q*, dq*, kp, kd and feed-forward torque per joint. */
  async applyLowLevelCommands(commands, { advanceSeconds = 0, maxSteps = 40000 } = {}) {
    this.#assertReady();
    if (!commands || typeof commands !== 'object' || Array.isArray(commands) || !Object.keys(commands).length) {
      throw new TypeError('applyLowLevelCommands requires a non-empty joint command object');
    }
    for (const [jointId, request] of Object.entries(commands)) {
      if (!G1_JOINT_ORDER.includes(jointId)) throw new RangeError(`Unknown Unitree G1 joint: ${jointId}`);
      for (const [key, value] of Object.entries(request || {})) {
        if (!['positionRad', 'velocityRadS', 'feedforwardTorqueNm', 'kp', 'kd'].includes(key)) throw new RangeError(`Unknown low-level command field ${key}`);
        if (!Number.isFinite(Number(value))) throw new TypeError(`Low-level ${jointId}.${key} must be finite`);
      }
    }
    const result = await this.session.sendCommand({ type: 'set_lowlevel_targets', commands }, { maxSteps });
    if (advanceSeconds > 0) await this.advanceTime(advanceSeconds);
    return result;
  }

  /**
   * Engage the verified standing controller. It exists because the free-base standing gate passes;
   * it makes no balance, recovery or locomotion claim, and it can fail.
   */
  async engageStand({ controllerId = G1_CONTROLLERS.STAND, holdSeconds = 0, maxSteps = 200000 } = {}) {
    this.#assertReady();
    const profile = G1_STAND_CONTROLLER_PROFILES[controllerId];
    if (!profile) throw new RangeError(`Unknown Unitree G1 standing controller: ${controllerId}`);
    const result = await this.session.sendCommand({ type: 'engage_stand', controllerId }, { maxSteps });
    this.standEngagedAtSeconds = Number(this.lastObservation?.simulationTimeSeconds ?? 0);
    if (holdSeconds > 0) await this.advanceTime(holdSeconds);
    return result;
  }

  async releaseStand() {
    this.#assertReady();
    this.standEngagedAtSeconds = null;
    return this.session.sendCommand({ type: 'release_stand' }, { maxSteps: 1 });
  }

  /**
   * The declared setup path. It removes actuator effort rather than adding capability, it is
   * logged into the observation's setup log, and it can never be mistaken for control.
   */
  async setActuationEnabled(enabled) {
    this.#assertReady();
    return this.session.applySetup({ type: 'set_actuation', enabled: Boolean(enabled) });
  }

  async advanceTime(seconds, { maxSeconds = MAX_ADVANCE_SECONDS } = {}) {
    this.#assertReady();
    const value = Number(seconds);
    if (!Number.isFinite(value) || value <= 0) throw new RangeError('advanceTime requires a positive number of simulated seconds');
    if (value > maxSeconds) throw new RangeError(`advanceTime is bounded to ${maxSeconds} simulated seconds per call`);
    const steps = Math.round(value / G1_PHYSICS_TIMESTEP_SECONDS);
    if (steps < 1) throw new RangeError('advanceTime must request at least one physics step');
    return this.session.advanceSteps(steps);
  }

  pause() { return this.session?.pause?.() ?? false; }
  resume() { return this.session?.resume?.() ?? false; }
  async stop() {
    if (!this.session) return false;
    // A stop holds the measured joint state with the source hold gains. It is not a teleport to
    // zero velocity and it is not a reset.
    const positions = Object.fromEntries(G1_JOINT_ORDER.map((jointId) => [jointId, Number(this.lastObservation?.joints?.[jointId]?.positionRad ?? 0)]));
    await this.session.sendCommand({ type: 'set_joint_targets', targetsRad: positions }, { maxSteps: 1 });
    return true;
  }

  // Every dataset key this simulator publishes, so switching away from the physical workspace
  // cannot leave a stale "free-base" or "standing" attribute describing a workspace that has
  // neither. The retained kinematic pose workspace runs on the source backend, which re-stamps
  // simulatorBackend, presentationGroundColor and simulationClockS but claims no physical
  // authority, scene or model package of its own: those must be withdrawn here.
  static DATASET_KEYS = Object.freeze([
    'unitreeG1RootMode', 'unitreeG1Walking', 'unitreeG1Hands', 'unitreeG1PelvisZM', 'unitreeG1PelvisTiltRad',
    'unitreeG1UprightZ', 'unitreeG1LeftFootContacts', 'unitreeG1RightFootContacts', 'unitreeG1NonFootGroundContacts',
    'unitreeG1SelfContacts', 'unitreeG1ExternalObjectContacts', 'unitreeG1ControllerId', 'unitreeG1ActuationEnabled',
    'unitreeG1Standing', 'unitreeG1Fell',
    'simulationAuthority', 'physicalSceneId', 'physicalSceneRevision', 'modelPackageId', 'renderedFrames',
  ]);

  dispose() {
    if (this.disposed) return;
    this.disposed = true;
    this.ready = false;
    for (const key of UnitreeG1PhysicalSimulator.DATASET_KEYS) delete this.canvas.dataset[key];
    this.unsubscribeSession?.();
    this.unsubscribeSession = null;
    this.session?.dispose?.();
    this.session = null;
    this.canonicalRig?.dispose?.();
    this.canonicalRig = null;
    for (const mesh of this.objectMeshes.values()) {
      mesh.geometry?.dispose?.();
      mesh.material?.dispose?.();
    }
    this.objectMeshes.clear();
    this.controls?.dispose?.();
    this.renderer?.dispose?.();
  }

  async #ensureCanonicalPresentation() {
    if (this.canonicalRig) return this.canonicalRig;
    const rig = await CanonicalRobotRig.load('unitree');
    rig.root.userData.presentationAuthority = 'observed MuJoCo pelvis transform and 29 measured joint positions only';
    this.robotRoot.add(rig.root);
    this.canonicalRig = rig;
    return rig;
  }

  async #createSession() {
    const session = new PhysicsSession(
      new BrowserMuJoCoBackend({
        workerUrl: new URL('./unitree-g1-mujoco-worker.js', import.meta.url),
        setupOperations: ['set_actuation'],
      }),
      { sessionId: `ide-unitree-g1-v1-${++this.sessionSequence}`, observationBatchSteps: G1_OBSERVATION_BATCH_STEPS },
    );
    this.session = session;
    this.unsubscribeSession = session.subscribe(({ observation }) => this.#consumeObservation(observation));
    await session.loadScene(structuredClone(UNITREE_G1_FREEBASE_SCENE));
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
    const root = observation.root || {};
    this.canvas.dataset.simulationClockS = String(Number(observation.simulationTimeSeconds || 0));
    this.canvas.dataset.unitreeG1PelvisZM = Number(root.positionM?.[2] ?? 0).toFixed(5);
    this.canvas.dataset.unitreeG1PelvisTiltRad = Number(root.tiltRad ?? 0).toFixed(5);
    this.canvas.dataset.unitreeG1UprightZ = Number(root.uprightZ ?? 0).toFixed(5);
    this.canvas.dataset.unitreeG1LeftFootContacts = String(observation.contactClasses?.leftFootFloor?.length ?? 0);
    this.canvas.dataset.unitreeG1RightFootContacts = String(observation.contactClasses?.rightFootFloor?.length ?? 0);
    this.canvas.dataset.unitreeG1NonFootGroundContacts = String(observation.contactClasses?.otherBodyFloor?.length ?? 0);
    this.canvas.dataset.unitreeG1SelfContacts = String(observation.contactClasses?.robotSelf?.length ?? 0);
    this.canvas.dataset.unitreeG1ExternalObjectContacts = String(observation.contactClasses?.robotExternalObject?.length ?? 0);
    this.canvas.dataset.unitreeG1ControllerId = String(observation.controller?.id ?? '');
    this.canvas.dataset.unitreeG1ActuationEnabled = String(observation.actuationEnabled !== false);
    this.canvas.dataset.unitreeG1Standing = String(Boolean(evaluation?.standing));
    this.canvas.dataset.unitreeG1Fell = String(Boolean(evaluation?.fellDuringRun));
  }

  #applyObservation(observation) {
    if (this.canonicalRig) {
      // The canonical rig root is the pelvis frame. Both the transform and all 29 joint values
      // come from the same authoritative observation; nothing is integrated here, and no
      // presentation transform is ever fed back into physics.
      this.canonicalRig.applyPhysicalState(canonicalJointStateDeg(observation));
      const root = observation.root;
      if (root?.positionM && root?.quaternionWxyz) {
        this.canonicalRig.root.position.copy(toThreePosition(root.positionM));
        this.canonicalRig.root.quaternion.copy(toThreeQuaternion(root.quaternionWxyz));
        this.canonicalRig.root.updateMatrixWorld(true);
      }
    }
    for (const [objectId, mesh] of this.objectMeshes) {
      const body = observation.bodies?.[objectId];
      if (!body?.positionM) continue;
      mesh.position.copy(toThreePosition(body.positionM));
      if (body.quaternionWxyz) mesh.quaternion.copy(toThreeQuaternion(body.quaternionWxyz));
    }
  }

  #buildPresentation() {
    const floorMaterial = new THREE.MeshStandardMaterial({ color: PRESENTATION_FLOOR_COLOR, roughness: 0.8, metalness: 0.03 });
    const floor = box(4, 0.02, 4, floorMaterial);
    floor.position.set(0, -10, 0);
    floor.receiveShadow = true;
    floor.userData.presentationOnly = true;
    this.workcellRoot.add(floor);

    const blockMaterial = new THREE.MeshStandardMaterial({ color: 0x3fa65a, roughness: 0.55, metalness: 0.05 });
    const block = box(0.1, 0.1, 0.1, blockMaterial);
    block.name = UNITREE_G1_EXTERNAL_OBJECT;
    block.castShadow = true;
    this.workcellRoot.add(block);
    this.objectMeshes.set(UNITREE_G1_EXTERNAL_OBJECT, block);
  }

  #assertReady() { this.#assertNotDisposed(); if (!this.isReady()) throw new Error('Unitree G1 physical session is not ready'); }
  #assertNotDisposed() { if (this.disposed) throw new Error('Unitree G1 physical simulator is disposed'); }
}

export const UNITREE_G1_PRESENTATION_LIMITS = Object.freeze({
  maxAdvanceSeconds: MAX_ADVANCE_SECONDS,
  maxWebmcpAdvanceSeconds: MAX_WEBMCP_ADVANCE_SECONDS,
  observationBatchSteps: G1_OBSERVATION_BATCH_STEPS,
  standRampSeconds: UNITREE_FIXSTAND_RAMP_SECONDS,
  maxKp: G1_MAX_KP,
  maxKd: G1_MAX_KD,
});
