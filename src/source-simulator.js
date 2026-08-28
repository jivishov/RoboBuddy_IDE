import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js';
import { OrbitControls } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/controls/OrbitControls.js';
import { CanonicalRobotRig } from './canonical-rig.js';
import { TASK_PATCH_REVISION } from './task-catalog.js';

const DEG = Math.PI / 180;

// SO-101 reuses its pinned RoboBuddy_AI inspection-camera direction.
// The pinned LeKiwi and OpenArm inspection presets are rear-facing relative to the
// robot/workcell presentation. For the IDE's semantic Front / Fit View we mirror each
// rear-facing azimuth 180 degrees around the same target while preserving elevation
// and camera-target distance.
export const FRONT_CAMERA_PRESETS = Object.freeze({
  so101: Object.freeze({ position: [540, 410, 720], target: [140, 105, -35] }),
  lekiwi: Object.freeze({ position: [500, 350, -150], target: [-60, 125, 45] }),
  openarm: Object.freeze({ position: [1830, 820, 0], target: [140, 365, 0] }),
});
const ENGINE_URL = `https://cdn.jsdelivr.net/gh/jivishov/RoboBuddy_AI@${TASK_PATCH_REVISION}/lab/v2/scenario-engine.js`;
let engineModulePromise = null;
const loadEngineModule = () => engineModulePromise ||= import(ENGINE_URL);

function internalToPublic(profileId, jointState = {}) {
  if (profileId === 'openarm') {
    const out = {};
    for (const side of ['left', 'right']) {
      for (let index = 1; index <= 7; index += 1) out[`${side}_joint_${index}.pos`] = Number(jointState[`${side}_j${index}`] ?? 0);
      out[`${side}_gripper.pos`] = -Math.max(0, Math.min(45, Number(jointState[`${side}_gripper`] ?? 45))) * (65 / 45);
    }
    return out;
  }
  if (profileId === 'so101') {
    return Object.fromEntries(['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'].map((key) => [`${key}.pos`, Number(jointState[key] ?? 0)]));
  }
  return Object.fromEntries(['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'].map((key) => [`arm_${key}.pos`, Number(jointState[key] ?? 0)]));
}

function basePoseForRig(snapshot = {}) {
  const root = snapshot.rootPose || {};
  const p = root.positionMm || [0, 0, 0];
  return { x: Number(p[0]) || 0, z: Number(p[2]) || 0, yaw: (Number(root.headingDeg) || 0) * DEG };
}

function proxyMaterial(proxy = {}) {
  const support = proxy.physicalSupportSurface === true || proxy.planningRole === 'contact_surface';
  return new THREE.MeshStandardMaterial({
    color: support ? 0xcbd5e1 : 0x4b5563,
    roughness: support ? 0.72 : 0.58,
    metalness: support ? 0.04 : 0.18,
    transparent: false,
  });
}

function buildProxy(proxy) {
  if (!proxy) return null;
  const material = proxyMaterial(proxy);
  if (proxy.type === 'box' && Array.isArray(proxy.halfExtentsMm)) {
    const [hx, hy, hz] = proxy.halfExtentsMm.map(Number);
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(hx * 2, hy * 2, hz * 2), material);
    mesh.position.fromArray((proxy.centerMm || [0,0,0]).map(Number));
    if (Array.isArray(proxy.axes) && proxy.axes.length === 3) {
      const a = proxy.axes;
      const matrix = new THREE.Matrix4().set(
        a[0][0], a[1][0], a[2][0], 0,
        a[0][1], a[1][1], a[2][1], 0,
        a[0][2], a[1][2], a[2][2], 0,
        0, 0, 0, 1,
      );
      mesh.quaternion.setFromRotationMatrix(matrix);
    }
    return mesh;
  }
  if ((proxy.type === 'annulus' || proxy.type === 'cylinder') && Array.isArray(proxy.centerMm)) {
    const outer = Number(proxy.outerRadiusMm || proxy.radiusMm || proxy.halfExtentsMm?.[0] || 45);
    const height = Number(proxy.heightMm || (proxy.halfExtentsMm?.[1] || 2) * 2 || 4);
    const mesh = proxy.type === 'annulus'
      ? new THREE.Mesh(new THREE.RingGeometry(Number(proxy.innerRadiusMm || outer * 0.55), outer, 48), material)
      : new THREE.Mesh(new THREE.CylinderGeometry(outer, outer, height, 48), material);
    mesh.position.fromArray(proxy.centerMm.map(Number));
    if (proxy.type === 'annulus') mesh.rotation.x = -Math.PI / 2;
    return mesh;
  }
  if (proxy.type === 'capsule' && Array.isArray(proxy.startMm) && Array.isArray(proxy.endMm)) {
    const start = new THREE.Vector3().fromArray(proxy.startMm.map(Number));
    const end = new THREE.Vector3().fromArray(proxy.endMm.map(Number));
    const delta = end.clone().sub(start);
    const length = delta.length();
    const radius = Number(proxy.radiusMm) || 12;
    const mesh = new THREE.Mesh(new THREE.CapsuleGeometry(radius, Math.max(0, length - 2 * radius), 8, 16), material);
    mesh.position.copy(start).add(end).multiplyScalar(0.5);
    if (length > 1e-9) mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), delta.normalize());
    return mesh;
  }
  material.dispose();
  return null;
}

function buildRuler(fixture) {
  const m = fixture.measurement;
  if (!m || !Array.isArray(m.originMm)) return null;
  const group = new THREE.Group();
  const axis = String(m.axis || 'x');
  const length = Number(m.lengthMm) || 200;
  const minor = Math.max(1, Number(m.minorTickMm) || 10);
  const width = Number(m.widthMm) || 24;
  const origin = new THREE.Vector3().fromArray(m.originMm.map(Number));
  const barMat = new THREE.MeshStandardMaterial({ color: 0xf8fafc, roughness: 0.65, metalness: 0.05 });
  const tickMat = new THREE.MeshStandardMaterial({ color: 0x111827, roughness: 0.8 });
  const barDims = axis === 'x' ? [length, 2, width] : axis === 'z' ? [width, 2, length] : [width, length, 2];
  const bar = new THREE.Mesh(new THREE.BoxGeometry(...barDims), barMat);
  const halfVector = axis === 'x' ? [length / 2, 0, 0] : axis === 'z' ? [0, 0, length / 2] : [0, length / 2, 0];
  bar.position.copy(origin).add(new THREE.Vector3(...halfVector));
  group.add(bar);
  for (let d = 0; d <= length + 1e-6; d += minor) {
    const major = Math.round(d / minor) % Math.max(1, Math.round((Number(m.majorTickMm) || 50) / minor)) === 0;
    const size = major ? 12 : 7;
    const tick = new THREE.Mesh(new THREE.BoxGeometry(axis === 'x' ? 1 : size, axis === 'y' ? 1 : 3, axis === 'z' ? 1 : size), tickMat);
    tick.position.copy(origin);
    if (axis === 'x') tick.position.add(new THREE.Vector3(d, 3, 0));
    else if (axis === 'z') tick.position.add(new THREE.Vector3(0, 3, d));
    else tick.position.add(new THREE.Vector3(0, d, 3));
    group.add(tick);
  }
  return group;
}

function objectKind(item) {
  const text = `${item.id || ''} ${item.label || ''} ${item.type || ''}`.toLowerCase();
  if (text.includes('erlenmeyer') || text.includes('flask')) return 'flask';
  if (text.includes('beaker')) return 'beaker';
  if (text.includes('bottle')) return 'bottle';
  if (text.includes('cuvette')) return 'cuvette';
  return 'generic';
}

function buildObjectVisual(item) {
  const group = new THREE.Group();
  group.name = `object:${item.id}`;
  const kind = objectKind(item);
  const glass = new THREE.MeshStandardMaterial({ color: 0x7dd3fc, roughness: 0.18, metalness: 0, transparent: true, opacity: 0.43, side: THREE.DoubleSide });
  const cap = new THREE.MeshStandardMaterial({ color: 0x155e75, roughness: 0.62, metalness: 0.04 });
  if (kind === 'flask') {
    const body = new THREE.Mesh(new THREE.CylinderGeometry(18, 40, 70, 40, 1, false), glass); body.position.y = 35; group.add(body);
    const neck = new THREE.Mesh(new THREE.CylinderGeometry(14, 14, 44, 32), glass); neck.position.y = 92; group.add(neck);
  } else if (kind === 'beaker') {
    const body = new THREE.Mesh(new THREE.CylinderGeometry(36, 34, 73, 40, 1, true), glass); body.position.y = 36.5; group.add(body);
  } else if (kind === 'bottle') {
    const body = new THREE.Mesh(new THREE.CylinderGeometry(30, 30, 78, 32), glass); body.position.y = 39; group.add(body);
    const top = new THREE.Mesh(new THREE.CylinderGeometry(18, 18, 13, 24), cap); top.position.y = 84.5; group.add(top);
  } else if (kind === 'cuvette') {
    const body = new THREE.Mesh(new THREE.BoxGeometry(12, 45, 12), glass); body.position.y = 22.5; group.add(body);
  } else {
    const body = new THREE.Mesh(new THREE.BoxGeometry(45, 65, 45), glass); body.position.y = 32.5; group.add(body);
  }
  group.traverse((node) => { if (node.isMesh) { node.castShadow = true; node.receiveShadow = true; } });
  return group;
}

class ScenarioVisual {
  constructor(scene, definition) {
    this.scene = scene;
    this.definition = definition;
    this.root = new THREE.Group();
    this.root.name = 'scenario-geometry';
    this.objects = new Map();
    this.scene.add(this.root);
    this.build();
  }
  build() {
    const seen = new Set();
    for (const fixture of this.definition.fixtures || []) {
      if (fixture.visible === false) continue;
      if (fixture.type === 'configured_measurement_ruler') {
        const ruler = buildRuler(fixture); if (ruler) this.root.add(ruler);
      }
      const proxies = fixture.collisionProxies || (fixture.collisionProxy ? [fixture.collisionProxy] : []);
      for (const proxy of proxies) {
        if (proxy.planningRole === 'robot_mount_contact') continue;
        const key = JSON.stringify([proxy.type, proxy.centerMm, proxy.halfExtentsMm, proxy.radiusMm, proxy.outerRadiusMm]);
        if (seen.has(key)) continue;
        seen.add(key);
        const mesh = buildProxy(proxy);
        if (mesh) { mesh.name = `fixture:${fixture.id}:${proxy.id || 'proxy'}`; mesh.castShadow = true; mesh.receiveShadow = true; this.root.add(mesh); }
      }
    }
    for (const item of this.definition.objects || []) {
      const visual = buildObjectVisual(item);
      this.objects.set(item.id, visual);
      this.root.add(visual);
    }
  }
  update(snapshot) {
    for (const state of Object.values(snapshot?.objects || {})) {
      const visual = this.objects.get(state.id);
      if (!visual || !Array.isArray(state.worldPositionMm)) continue;
      visual.position.fromArray(state.worldPositionMm.map(Number));
      const r = state.worldRotationMatrix;
      if (Array.isArray(r) && r.length === 9) {
        const m = new THREE.Matrix4().set(r[0],r[1],r[2],0,r[3],r[4],r[5],0,r[6],r[7],r[8],0,0,0,0,1);
        visual.quaternion.setFromRotationMatrix(m);
      }
    }
  }
  dispose() {
    this.root.traverse((node) => { if (node.isMesh) { node.geometry?.dispose?.(); node.material?.dispose?.(); } });
    this.scene.remove(this.root);
  }
}

export class SourceRobotSimulator {
  constructor(canvas) {
    this.canvas = canvas;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0d1720);
    this.camera = new THREE.PerspectiveCamera(44, 1, 1, 5000);
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.shadowMap.enabled = true;
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.target.set(180, 300, 0);
    this.rig = null;
    this.engine = null;
    this.visual = null;
    this.profileId = '';
    this.scenario = null;
    this.connectedInstance = 'ide-robot';
    this.installScene();
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(canvas.parentElement || canvas);
    this.resize();
    this.animate();
  }
  installScene() {
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x334155, 1.35));
    const key = new THREE.DirectionalLight(0xffffff, 2.15); key.position.set(-650, 1000, 620); key.castShadow = true; this.scene.add(key);
    const fill = new THREE.DirectionalLight(0x93c5fd, 0.45); fill.position.set(700, 500, -600); this.scene.add(fill);
    const grid = new THREE.GridHelper(1800, 36, 0x334155, 0x1f2937); grid.position.y = -0.5; this.scene.add(grid);
    // Several reviewed work-surface proxies intentionally have their top face at y=0,
    // the same geometric plane as this presentation-only ground disk. Rendering both at
    // identical depth causes camera-angle-dependent z-fighting. Keep all authored
    // collision/support geometry untouched and bias only the decorative ground depth.
    const floorMaterial = new THREE.MeshStandardMaterial({
      color: 0x17212b,
      roughness: 0.95,
      polygonOffset: true,
      polygonOffsetFactor: 1,
      polygonOffsetUnits: 4,
    });
    const floor = new THREE.Mesh(new THREE.CircleGeometry(980, 96), floorMaterial);
    floor.name = 'presentation-ground-depth-biased';
    floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true; this.scene.add(floor);
  }
  async setScenario(profileId, scenario, fallbackRest = {}) {
    this.profileId = profileId;
    this.scenario = scenario || null;
    this.engine?.plant?.dispose?.();
    this.engine = null;
    this.visual?.dispose?.(); this.visual = null;
    if (this.rig) { this.scene.remove(this.rig.root); this.rig.dispose(); this.rig = null; }
    this.rig = await CanonicalRobotRig.load(profileId);
    this.scene.add(this.rig.root);
    if (scenario) {
      const { ScenarioV2Engine } = await loadEngineModule();
      this.engine = await ScenarioV2Engine.create(scenario, { autoStartPlant: false });
      const connectionConfig = this.profileId === 'openarm'
        ? { kind: 'bimanual', side: 'bimanual', cameras: {} }
        : { cameras: {} };
      this.engine.plant.connect(this.connectedInstance, connectionConfig);
      this.visual = new ScenarioVisual(this.scene, scenario);
      this.syncFromSource();
    } else {
      this.rig.applyPhysicalState(fallbackRest, { x: 0, z: 0, yaw: 0 });
    }
    this.fit();
  }
  async reset(profileId = this.profileId, scenario = this.scenario, fallbackRest = {}) {
    await this.setScenario(profileId, scenario, fallbackRest);
  }
  syncFromSource() {
    if (!this.engine || !this.rig) return;
    const snapshot = this.engine.snapshot();
    this.rig.applyPhysicalState(internalToPublic(this.profileId, snapshot.jointState), basePoseForRig(snapshot));
    this.visual?.update(snapshot);
  }
  async applyAction(action) {
    if (!this.engine?.plant) {
      this.rig?.applyPhysicalState(action, { x: 0, z: 0, yaw: 0 });
      return;
    }
    try {
      this.engine.plant.sendAction(this.connectedInstance, action, {});
    } catch (error) {
      throw new Error(`${error.code || 'ACTION_REJECTED'} — ${error.message}`);
    }
    await this.advanceTime(0.02, { realtime: false });
  }
  async advanceTime(seconds, { realtime = true } = {}) {
    if (!this.engine?.plant) return;
    const ticks = Math.max(0, Math.ceil(Math.max(0, Number(seconds) || 0) / this.engine.plant.tickSeconds));
    const visualStride = Math.max(1, Math.floor(ticks / 30));
    for (let i = 0; i < ticks; i += 1) {
      this.engine.plant.tick();
      if (this.engine.plant.fault) {
        this.syncFromSource();
        const fault = this.engine.plant.fault;
        const witness = fault.collision ? ` ${JSON.stringify(fault.collision)}` : '';
        throw new Error(`${fault.code} — ${fault.message}${witness}`);
      }
      if (i % visualStride === 0 || i === ticks - 1) {
        this.syncFromSource();
        if (realtime && ticks > 3) await new Promise((resolve) => requestAnimationFrame(resolve));
      }
    }
  }
  advanceBase(seconds) { return this.advanceTime(seconds); }
  getTelemetry() {
    if (!this.engine) return {};
    const snapshot = this.engine.snapshot();
    return { ...internalToPublic(this.profileId, snapshot.jointState), simulation_clock_s: Number(snapshot.simulationClockSeconds || this.engine.plant?.clockSeconds || 0) };
  }
  getContacts() {
    if (!this.engine) return { sourcePlant: 'not active' };
    const snapshot = this.engine.snapshot();
    const held = Object.values(snapshot.objects || {}).filter((item) => item.attachedTo).map((item) => `${item.id}→${item.attachedTo}`);
    const unsupported = Object.values(snapshot.objects || {}).filter((item) => item.releasedUnsupported).map((item) => item.id);
    return {
      sourcePlant: `RoboBuddy_AI@${TASK_PATCH_REVISION.slice(0, 12)}`,
      fault: this.engine.plant?.fault?.code || 'none',
      heldObjects: held.join(', ') || 'none',
      unsupportedReleased: unsupported.join(', ') || 'none',
      simulationClockS: Number(this.engine.plant?.clockSeconds || 0),
    };
  }
  fit() {
    const box = new THREE.Box3();
    if (this.rig?.root) box.expandByObject(this.rig.root);
    if (this.visual?.root) box.expandByObject(this.visual.root);
    if (box.isEmpty()) { this.camera.position.set(-900, 650, 850); this.controls.target.set(100, 250, 0); this.controls.update(); return; }
    const center = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3());
    const radius = Math.max(260, size.length() * 0.58);
    const preset = FRONT_CAMERA_PRESETS[this.profileId] || FRONT_CAMERA_PRESETS.so101;
    const direction = new THREE.Vector3().fromArray(preset.position).sub(new THREE.Vector3().fromArray(preset.target)).normalize();
    const distance = Math.max(460, radius * 1.72);
    this.controls.target.copy(center);
    this.camera.position.copy(center).addScaledVector(direction, distance);
    this.camera.near = Math.max(1, radius / 1000); this.camera.far = Math.max(3000, radius * 6); this.camera.updateProjectionMatrix(); this.controls.update();
    this.canvas.dataset.cameraView = 'front';
  }
  resize() {
    const rect = this.canvas.getBoundingClientRect(); const w = Math.max(1, rect.width); const h = Math.max(1, rect.height);
    this.renderer.setSize(w, h, false); this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
  }
  animate() { requestAnimationFrame(() => this.animate()); this.controls.update(); this.renderer.render(this.scene, this.camera); }
  dispose() { this.engine?.plant?.dispose?.(); this.visual?.dispose?.(); this.rig?.dispose?.(); this.resizeObserver?.disconnect?.(); this.renderer?.dispose?.(); }
}
