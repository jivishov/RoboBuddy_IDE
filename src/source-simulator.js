import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js';
import { OrbitControls } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/controls/OrbitControls.js';
import { CanonicalRobotRig, canonicalVisualProvenance } from './canonical-rig.js';
import { TASK_PATCH_REVISION } from './task-catalog.js';

const DEG = Math.PI / 180;

// Presentation tokens intentionally live outside the pinned robot and scenario
// sources. They tune the viewport only; they are not calibrated material values.
export const PRESENTATION_GROUND_COLOR = 0x687378;
export const HIGH_CONTRAST_SCENE_COLORS = Object.freeze({
  light: 0xf8fafc,
  dark: 0x111827,
});
const CONTACT_PERIMETER = Object.freeze({
  clearanceMm: 3,
  widthMm: 5,
  liftMm: 0.8,
});

// SO-101 reuses its pinned RoboBuddy_AI inspection-camera direction.
// The pinned LeKiwi and OpenArm inspection presets are rear-facing relative to the
// robot/workcell presentation. For the IDE's semantic Front / Fit View we mirror each
// rear-facing azimuth 180 degrees around the same target while preserving elevation
// and camera-target distance.
export const FRONT_CAMERA_PRESETS = Object.freeze({
  so101: Object.freeze({ position: [540, 410, 720], target: [140, 105, -35] }),
  lekiwi: Object.freeze({ position: [500, 350, -150], target: [-60, 125, 45] }),
  openarm: Object.freeze({ position: [1830, 820, 0], target: [140, 365, 0] }),
  unitree: Object.freeze({ position: [1950, 1180, 1650], target: [150, 660, 0] }),
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
  if (profileId === 'unitree') {
    return Object.fromEntries(Object.entries(jointState).map(([key, value]) => [key, Number(value ?? 0)]));
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

function presentationMaterial(color) {
  return new THREE.MeshBasicMaterial({
    color,
    transparent: true,
    opacity: 0.96,
    depthWrite: false,
    side: THREE.DoubleSide,
    toneMapped: false,
  });
}

function presentationOnly(node, name, colorRole) {
  node.name = name;
  node.renderOrder = 3;
  node.userData = {
    presentationOnly: true,
    configured: true,
    accessibilityRole: 'floor-contact-perimeter',
    colorRole,
    canonicalMesh: false,
    collisionGeometry: false,
    kinematics: false,
  };
  return node;
}

function rectangularPerimeter(halfX, halfZ, color, name) {
  const outerX = Math.max(5, Number(halfX) + CONTACT_PERIMETER.clearanceMm + CONTACT_PERIMETER.widthMm);
  const outerZ = Math.max(5, Number(halfZ) + CONTACT_PERIMETER.clearanceMm + CONTACT_PERIMETER.widthMm);
  const innerX = Math.max(1, outerX - CONTACT_PERIMETER.widthMm);
  const innerZ = Math.max(1, outerZ - CONTACT_PERIMETER.widthMm);
  const shape = new THREE.Shape();
  shape.moveTo(-outerX, -outerZ);
  shape.lineTo(outerX, -outerZ);
  shape.lineTo(outerX, outerZ);
  shape.lineTo(-outerX, outerZ);
  shape.closePath();
  const hole = new THREE.Path();
  hole.moveTo(-innerX, -innerZ);
  hole.lineTo(-innerX, innerZ);
  hole.lineTo(innerX, innerZ);
  hole.lineTo(innerX, -innerZ);
  hole.closePath();
  shape.holes.push(hole);
  const mesh = new THREE.Mesh(new THREE.ShapeGeometry(shape), presentationMaterial(color));
  mesh.rotation.x = -Math.PI / 2;
  return presentationOnly(mesh, name, color === HIGH_CONTRAST_SCENE_COLORS.light ? 'light' : 'dark');
}

function circularPerimeter(radius, color, name) {
  const inner = Math.max(4, Number(radius) + CONTACT_PERIMETER.clearanceMm);
  const outer = inner + CONTACT_PERIMETER.widthMm;
  const mesh = new THREE.Mesh(new THREE.RingGeometry(inner, outer, 64), presentationMaterial(color));
  mesh.rotation.x = -Math.PI / 2;
  return presentationOnly(mesh, name, color === HIGH_CONTRAST_SCENE_COLORS.light ? 'light' : 'dark');
}

function proxyOrientation(proxy, target) {
  if (!Array.isArray(proxy?.axes) || proxy.axes.length !== 3) return;
  const a = proxy.axes;
  const matrix = new THREE.Matrix4().set(
    a[0][0], a[1][0], a[2][0], 0,
    a[0][1], a[1][1], a[2][1], 0,
    a[0][2], a[1][2], a[2][2], 0,
    0, 0, 0, 1,
  );
  target.quaternion.setFromRotationMatrix(matrix);
}

function isSupportProxy(proxy = {}) {
  return proxy.physicalSupportSurface === true || proxy.planningRole === 'contact_surface' || proxy.planningRole === 'contact_support';
}

function objectFootprint(item = {}) {
  const visual = Array.isArray(item.visual?.footprintMm) ? item.visual.footprintMm.map(Number) : null;
  if (visual?.length >= 2 && visual.every(Number.isFinite)) return visual;
  const half = Array.isArray(item.physicalRest?.geometry?.halfExtentsMm) ? item.physicalRest.geometry.halfExtentsMm.map(Number) : null;
  if (half?.length >= 3 && half.every(Number.isFinite)) return [half[0] * 2, half[2] * 2];
  return [45, 45];
}

class HighContrastSceneLayer {
  constructor(scene) {
    this.scene = scene;
    this.enabled = true;
    this.root = new THREE.Group();
    this.root.name = 'presentation-only-high-contrast-scene';
    this.root.userData = {
      presentationOnly: true,
      configured: true,
      canonicalMesh: false,
      collisionGeometry: false,
      kinematics: false,
      description: 'Configured accessibility aids; they do not alter the source plant or canonical mesh.',
    };
    this.objectMarkers = new Map();
    this.rigMarker = null;
    this.rig = null;
    this.scene.add(this.root);
  }

  setEnabled(enabled) {
    this.enabled = Boolean(enabled);
    this.root.visible = this.enabled;
    return this.enabled;
  }

  getPerimeterCount() {
    let count = 0;
    this.root.traverse((node) => {
      if (node.isMesh && node.userData?.accessibilityRole === 'floor-contact-perimeter') count += 1;
    });
    return count;
  }

  clear() {
    this.root.traverse((node) => {
      if (!node.isMesh) return;
      node.geometry?.dispose?.();
      node.material?.dispose?.();
    });
    this.root.clear();
    this.objectMarkers.clear();
    this.rigMarker = null;
    this.rig = null;
  }

  addFixturePerimeters(definition) {
    const seen = new Set();
    for (const fixture of definition?.fixtures || []) {
      if (fixture.visible === false) continue;
      const proxies = fixture.collisionProxies || (fixture.collisionProxy ? [fixture.collisionProxy] : []);
      for (const proxy of proxies) {
        if (isSupportProxy(proxy) || proxy.planningRole === 'robot_mount_contact') continue;
        const center = Array.isArray(proxy.centerMm) ? proxy.centerMm.map(Number) : null;
        if (!center || center.length !== 3 || !center.every(Number.isFinite)) continue;
        const key = JSON.stringify([proxy.type, proxy.id, center, proxy.halfExtentsMm, proxy.radiusMm, proxy.outerRadiusMm]);
        if (seen.has(key)) continue;
        seen.add(key);
        let marker = null;
        let floorY = null;
        if (proxy.type === 'box' && Array.isArray(proxy.halfExtentsMm)) {
          const [halfX, halfY, halfZ] = proxy.halfExtentsMm.map(Number);
          floorY = center[1] - halfY;
          marker = rectangularPerimeter(halfX, halfZ, HIGH_CONTRAST_SCENE_COLORS.light, `presentation-contact:${fixture.id}:${proxy.id || 'box'}`);
        } else if (proxy.type === 'cylinder' || proxy.type === 'annulus') {
          const height = Number(proxy.heightMm || (proxy.halfExtentsMm?.[1] || 0) * 2 || 0);
          const radius = Number(proxy.outerRadiusMm || proxy.radiusMm || proxy.halfExtentsMm?.[0] || 0);
          if (!Number.isFinite(radius) || radius <= 0) continue;
          floorY = proxy.type === 'annulus' ? center[1] : center[1] - height / 2;
          marker = circularPerimeter(radius, HIGH_CONTRAST_SCENE_COLORS.light, `presentation-contact:${fixture.id}:${proxy.id || proxy.type}`);
        }
        // These are floor-contact cues only. Equipment resting on a light worktop
        // already has a distinct support boundary and intentionally receives no halo.
        if (!marker || !Number.isFinite(floorY) || Math.abs(floorY) > 2) continue;
        const anchor = presentationOnly(new THREE.Group(), `presentation-contact-anchor:${fixture.id}:${proxy.id || proxy.type}`, 'light');
        anchor.position.set(center[0], floorY + CONTACT_PERIMETER.liftMm, center[2]);
        proxyOrientation(proxy, anchor);
        anchor.add(marker);
        this.root.add(anchor);
      }
    }
  }

  addObjectPerimeters(definition) {
    for (const item of definition?.objects || []) {
      if (item.visible === false || !item.id) continue;
      const [width, depth] = objectFootprint(item);
      const kind = objectKind(item);
      const round = kind === 'flask' || kind === 'beaker' || kind === 'bottle';
      const perimeter = round
        ? circularPerimeter(Math.max(width, depth) / 2, HIGH_CONTRAST_SCENE_COLORS.dark, `presentation-glass-contact:${item.id}`)
        : rectangularPerimeter(width / 2, depth / 2, HIGH_CONTRAST_SCENE_COLORS.dark, `presentation-glass-contact:${item.id}`);
      const anchor = presentationOnly(new THREE.Group(), `presentation-glass-anchor:${item.id}`, 'dark');
      anchor.visible = false;
      anchor.add(perimeter);
      this.objectMarkers.set(item.id, anchor);
      this.root.add(anchor);
    }
  }

  addLeKiwiWheelPerimeters(rig) {
    if (rig?.profileId !== 'lekiwi') return;
    const marker = presentationOnly(new THREE.Group(), 'presentation-lekiwi-floor-contacts', 'light');
    const groundOffset = Number(rig.meshData?.groundOffsetMm) || 0;
    const wheelJoints = (rig.meshData?.chain || []).filter((joint) => /^base_(back|left|right)_wheel$/.test(String(joint.id || '')));
    for (const wheel of wheelJoints) {
      const [x = 0, , z = 0] = Array.isArray(wheel.pivotMm) ? wheel.pivotMm.map(Number) : [];
      const perimeter = circularPerimeter(51, HIGH_CONTRAST_SCENE_COLORS.light, `presentation-lekiwi-wheel-contact:${wheel.id}`);
      perimeter.position.set(x, -groundOffset + CONTACT_PERIMETER.liftMm, z);
      marker.add(perimeter);
    }
    if (marker.children.length) {
      this.rigMarker = marker;
      this.rig = rig;
      this.root.add(marker);
      this.syncRigMarker();
    }
  }

  configure({ definition, rig }) {
    this.clear();
    this.addFixturePerimeters(definition);
    this.addObjectPerimeters(definition);
    this.addLeKiwiWheelPerimeters(rig);
    this.root.visible = this.enabled;
  }

  syncRigMarker() {
    if (!this.rigMarker || !this.rig?.root) return;
    this.rig.root.updateMatrixWorld(true);
    this.rig.root.getWorldPosition(this.rigMarker.position);
    this.rig.root.getWorldQuaternion(this.rigMarker.quaternion);
  }

  update(snapshot) {
    for (const state of Object.values(snapshot?.objects || {})) {
      const marker = this.objectMarkers.get(state.id);
      if (!marker || !Array.isArray(state.worldPositionMm)) continue;
      marker.position.fromArray(state.worldPositionMm.map(Number));
      marker.position.y += CONTACT_PERIMETER.liftMm;
      const rotation = state.worldRotationMatrix;
      if (Array.isArray(rotation) && rotation.length === 9) {
        const matrix = new THREE.Matrix4().set(rotation[0], rotation[1], rotation[2], 0, rotation[3], rotation[4], rotation[5], 0, rotation[6], rotation[7], rotation[8], 0, 0, 0, 0, 1);
        const yaw = new THREE.Euler().setFromRotationMatrix(matrix, 'YXZ').y;
        marker.rotation.set(0, yaw, 0);
      }
      marker.visible = !state.attachedTo && !state.releasedUnsupported;
    }
    this.syncRigMarker();
  }

  dispose() {
    this.clear();
    this.scene.remove(this.root);
  }
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
    // A neutral studio grey keeps the canonical dark robot meshes readable without
    // making the simulator compete with the workcell itself.
    this.scene.background = new THREE.Color(0xb9c1c4);
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
    this.fallbackState = {};
    this.fallbackClockSeconds = 0;
    this.rigTransitionSeconds = 0.25;
    this.connectedInstance = 'ide-robot';
    this.installScene();
    this.highContrastScene = new HighContrastSceneLayer(this.scene);
    this.canvas.dataset.highContrastScene = 'true';
    this.canvas.dataset.presentationGroundColor = '#687378';
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
      color: PRESENTATION_GROUND_COLOR,
      roughness: 0.95,
      polygonOffset: true,
      polygonOffsetFactor: 1,
      polygonOffsetUnits: 4,
    });
    const floor = new THREE.Mesh(new THREE.CircleGeometry(980, 96), floorMaterial);
    floor.name = 'presentation-ground-depth-biased';
    floor.userData = {
      presentationOnly: true,
      configured: true,
      materialNote: 'Neutral Lab Grey #687378 is a configured viewport material, not a real-world calibrated surface.',
      collisionGeometry: false,
      kinematics: false,
    };
    floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true; this.scene.add(floor);
  }
  async setScenario(profileId, scenario, fallbackRest = {}) {
    this.profileId = profileId;
    this.scenario = scenario || null;
    this.fallbackState = { ...fallbackRest };
    this.fallbackClockSeconds = 0;
    this.canvas.dataset.simulationClockS = '0';
    this.engine?.plant?.dispose?.();
    this.engine = null;
    this.visual?.dispose?.(); this.visual = null;
    this.highContrastScene?.clear();
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
      this.highContrastScene.configure({ definition: scenario, rig: this.rig });
      this.canvas.dataset.highContrastPerimeterCount = String(this.highContrastScene.getPerimeterCount());
      this.syncFromSource();
    } else {
      this.rig.applyPhysicalState(this.fallbackState, { x: 0, z: 0, yaw: 0 });
      this.highContrastScene.configure({ definition: null, rig: this.rig });
      this.canvas.dataset.highContrastPerimeterCount = String(this.highContrastScene.getPerimeterCount());
    }
    this.fit();
  }
  async reset(profileId = this.profileId, scenario = this.scenario, fallbackRest = {}) {
    await this.setScenario(profileId, scenario, fallbackRest);
  }
  syncFromSource() {
    if (!this.engine || !this.rig) return;
    const snapshot = this.engine.snapshot();
    this.canvas.dataset.simulationClockS = String(Number(snapshot.simulationClockSeconds || this.engine.plant?.clockSeconds || 0));
    this.rig.applyPhysicalState(internalToPublic(this.profileId, snapshot.jointState), basePoseForRig(snapshot));
    this.visual?.update(snapshot);
    this.highContrastScene?.update(snapshot);
  }
  setHighContrastScene(enabled) {
    const active = this.highContrastScene?.setEnabled(enabled) ?? false;
    this.canvas.dataset.highContrastScene = String(active);
    return active;
  }
  isHighContrastSceneEnabled() { return Boolean(this.highContrastScene?.enabled); }
  async applyAction(action, options = {}) {
    if (!this.engine?.plant) {
      if (!this.rig) throw new Error('Canonical rig is unavailable.');
      const before = { ...this.fallbackState };
      const target = { ...before, ...action };
      const keys = Object.keys(action || {});
      const frames = Math.max(1, Math.ceil(this.rigTransitionSeconds / 0.025));
      for (let frame = 1; frame <= frames; frame += 1) {
        if (options.beforeTick && !(await options.beforeTick())) return false;
        const progress = frame / frames;
        const smooth = progress * progress * (3 - 2 * progress);
        const interpolated = { ...before };
        for (const key of keys) {
          const start = Number(before[key] ?? 0);
          const end = Number(target[key]);
          interpolated[key] = start + (end - start) * smooth;
        }
        this.fallbackState = interpolated;
        this.rig.applyPhysicalState(this.fallbackState, { x: 0, z: 0, yaw: 0 });
        this.fallbackClockSeconds += this.rigTransitionSeconds / frames;
        this.canvas.dataset.simulationClockS = String(this.fallbackClockSeconds);
        await new Promise((resolve) => requestAnimationFrame(resolve));
      }
      this.fallbackState = target;
      return true;
    }
    try {
      this.engine.plant.sendAction(this.connectedInstance, action, {});
    } catch (error) {
      throw new Error(`${error.code || 'ACTION_REJECTED'} — ${error.message}`);
    }
    return this.advanceTime(0.02, { ...options, realtime: false });
  }
  async advanceTime(seconds, { realtime = true, beforeTick = null } = {}) {
    if (!this.engine?.plant) {
      if (beforeTick && !(await beforeTick())) return false;
      this.fallbackClockSeconds += Math.max(0, Number(seconds) || 0);
      this.canvas.dataset.simulationClockS = String(this.fallbackClockSeconds);
      if (realtime && Number(seconds) > 0) await new Promise((resolve) => requestAnimationFrame(resolve));
      return true;
    }
    const ticks = Math.max(0, Math.ceil(Math.max(0, Number(seconds) || 0) / this.engine.plant.tickSeconds));
    const visualStride = Math.max(1, Math.floor(ticks / 30));
    for (let i = 0; i < ticks; i += 1) {
      if (beforeTick && !(await beforeTick())) return false;
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
    return true;
  }
  advanceBase(seconds) { return this.advanceTime(seconds); }
  getTelemetry() {
    if (!this.engine) return { ...this.fallbackState, simulation_clock_s: this.fallbackClockSeconds };
    const snapshot = this.engine.snapshot();
    return { ...internalToPublic(this.profileId, snapshot.jointState), simulation_clock_s: Number(snapshot.simulationClockSeconds || this.engine.plant?.clockSeconds || 0) };
  }
  getContacts() {
    if (!this.engine) return {
      simulationMode: 'kinematic pose only',
      canonicalVisual: canonicalVisualProvenance(this.profileId)?.robotId || this.profileId,
      sourcePlant: 'not active',
      contactModel: 'not simulated',
      locomotion: 'not simulated; fixed visual root',
      hardwareValidation: 'pending',
    };
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
    const distance = Math.max(460, radius * 1.72) * (this.profileId === 'unitree' ? 1.22 : 1);
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
  dispose() { this.engine?.plant?.dispose?.(); this.visual?.dispose?.(); this.highContrastScene?.dispose?.(); this.rig?.dispose?.(); this.resizeObserver?.disconnect?.(); this.renderer?.dispose?.(); }
}
