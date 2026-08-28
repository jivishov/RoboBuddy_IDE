import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js';

export const ROBOBUDDY_AI_VISUAL_REVISION = '66d18a029a0caeb6a6075e681dbd9ecd6b22affa';
const ASSET_BASE = `https://cdn.jsdelivr.net/gh/jivishov/RoboBuddy_AI@${ROBOBUDDY_AI_VISUAL_REVISION}/simulator/js`;

export const CANONICAL_VISUALS = Object.freeze({
  openarm: Object.freeze({
    robotId: 'openarm_v2_bimanual',
    title: 'OpenArm V2 Bimanual — RoboBuddy canonical mesh',
    module: `${ASSET_BASE}/robot-mesh-data-openarm-v2.js`,
    sourceModelRevision: '6c7b720f1ba48e8bafa3a3dc752c45f397b42221',
    toolFrames: Object.freeze({
      left: Object.freeze({ parent: 'left_j7', offsetMm: [0, -168, 0] }),
      right: Object.freeze({ parent: 'right_j7', offsetMm: [0, -168, 0] }),
    }),
    partCorrections: Object.freeze({
      right_finger_inner_mesh: Object.freeze({ meshKey: 'finger_outer' }),
      right_finger_outer_mesh: Object.freeze({ meshKey: 'finger_inner' }),
    }),
  }),
  so101: Object.freeze({
    robotId: 'so101_follower',
    title: 'LeRobot SO-101 Follower — RoboBuddy canonical mesh',
    module: `${ASSET_BASE}/robot-mesh-data-so101.js`,
    sourceModelRevision: 'SO101 official URDF baked by RoboBuddy_AI',
  }),
  lekiwi: Object.freeze({
    robotId: 'lekiwi_sim',
    title: 'LeKiwi Mobile Manipulator — RoboBuddy canonical mesh',
    module: `${ASSET_BASE}/robot-mesh-data-lekiwi.js`,
    sourceModelRevision: 'efa608d7ee5a495a4803b1d28cd0c955b4f1e033',
  }),
});

const DEG = Math.PI / 180;
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

function physicalToCanonical(profileId, state = {}) {
  if (profileId === 'openarm') {
    const out = { base_yaw: 0 };
    for (const side of ['left', 'right']) {
      for (let index = 1; index <= 7; index += 1) {
        const publicKey = `${side}_joint_${index}.pos`;
        const visualKey = `${side}_j${index}`;
        if (Number.isFinite(Number(state[publicKey]))) out[visualKey] = Number(state[publicKey]);
      }
      const publicGrip = Number(state[`${side}_gripper.pos`]);
      // LeRobot OpenArm public command: -65 = open, 0 = closed.
      // RoboBuddy canonical renderer joint: 45 deg = open, 0 deg = closed.
      if (Number.isFinite(publicGrip)) out[`${side}_gripper`] = clamp(-publicGrip * 45 / 65, 0, 45);
    }
    return out;
  }
  if (profileId === 'so101') {
    return Object.fromEntries(
      ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
        .filter((key) => Number.isFinite(Number(state[`${key}.pos`])))
        .map((key) => [key, Number(state[`${key}.pos`])])
    );
  }
  const out = {};
  for (const key of ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']) {
    const raw = Number(state[`arm_${key}.pos`]);
    if (Number.isFinite(raw)) out[key] = raw;
  }
  return out;
}

function decodePositions(meshData, payload) {
  if (!payload || typeof payload.positions !== 'string') throw new Error('Canonical mesh payload is missing quantized positions.');
  const vertexCount = Number(payload.vertexCount);
  const bounds = Array.isArray(payload.bounds) ? payload.bounds.map(Number) : [];
  if (!Number.isFinite(vertexCount) || vertexCount <= 0 || bounds.length !== 6) throw new Error('Canonical mesh payload has invalid metadata.');
  const binary = window.atob(payload.positions);
  const valueCount = vertexCount * 3;
  if (binary.length < valueCount * 2) throw new Error('Canonical mesh payload is shorter than expected.');
  const bytes = new Uint8Array(binary.length);
  for (let index = 0; index < binary.length; index += 1) bytes[index] = binary.charCodeAt(index);
  const view = new DataView(bytes.buffer);
  const positions = new Float32Array(valueCount);
  const quantization = Number(meshData.quantization) || 65535;
  for (let index = 0; index < valueCount; index += 1) {
    const axis = index % 3;
    const min = bounds[axis];
    const max = bounds[axis + 3];
    positions[index] = min + (max - min) * (view.getUint16(index * 2, true) / quantization);
  }
  return positions;
}

function makeMaterial(options = {}) {
  return new THREE.MeshStandardMaterial({
    color: Number.isFinite(Number(options.color)) ? Number(options.color) : 0xff6b6b,
    roughness: Number.isFinite(Number(options.roughness)) ? Number(options.roughness) : 0.62,
    metalness: Number.isFinite(Number(options.metalness)) ? Number(options.metalness) : 0.04,
    side: options.doubleSided ? THREE.DoubleSide : THREE.FrontSide,
  });
}

export class CanonicalRobotRig {
  static async load(profileId) {
    const source = CANONICAL_VISUALS[profileId];
    if (!source) throw new Error(`No canonical RoboBuddy visual source for ${profileId}.`);
    const module = await import(source.module);
    const meshData = module.ROBOT_RIG_MESH_DATA || module.default;
    if (!meshData || meshData.robotId !== source.robotId) throw new Error(`Canonical mesh robotId mismatch for ${profileId}.`);
    return new CanonicalRobotRig(profileId, source, meshData);
  }

  constructor(profileId, source, meshData) {
    this.profileId = profileId;
    this.source = source;
    this.meshData = meshData;
    this.groups = {};
    this.jointDefs = [];
    this.meshes = [];
    this.geometries = new Set();
    this.materials = new Map();
    this.toolFrames = {};
    this.root = this._build();
  }

  _build() {
    const meshData = this.meshData;
    const root = new THREE.Group();
    root.name = `${meshData.robotId}-canonical-root`;
    root.position.y = Number(meshData.groundOffsetMm) || 0;
    root.userData.groundOffsetMm = Number(meshData.groundOffsetMm) || 0;
    root.userData.canonicalSource = {
      repo: 'jivishov/RoboBuddy_AI',
      revision: ROBOBUDDY_AI_VISUAL_REVISION,
      robotId: meshData.robotId,
    };
    this.groups.root = root;

    for (const joint of meshData.chain || []) {
      const parent = this.groups[joint.parent] || root;
      const group = new THREE.Group();
      group.name = joint.id;
      group.position.fromArray(Array.isArray(joint.pivotMm) ? joint.pivotMm.map(Number) : [0, 0, 0]);
      if (Array.isArray(joint.baseQuat) && joint.baseQuat.length === 4) group.quaternion.fromArray(joint.baseQuat.map(Number)).normalize();
      group.userData.baseQuaternion = group.quaternion.clone();
      group.userData.joint = joint;
      parent.add(group);
      this.groups[joint.id] = group;
      this.jointDefs.push(joint);
    }

    const geometryCache = new Map();
    for (const part of meshData.parts || []) {
      const correction = this.source.partCorrections?.[part.key];
      const meshKey = correction?.meshKey || part.meshKey;
      const payload = meshData.meshes?.[meshKey];
      if (!payload) throw new Error(`Canonical mesh payload ${meshKey} is missing.`);
      let geometry = geometryCache.get(meshKey);
      if (!geometry) {
        geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(decodePositions(meshData, payload), 3));
        geometry.computeVertexNormals();
        geometry.computeBoundingBox();
        geometry.computeBoundingSphere();
        geometryCache.set(meshKey, geometry);
        this.geometries.add(geometry);
      }
      const materialKey = part.material || 'fallback';
      let material = this.materials.get(materialKey);
      if (!material) {
        material = makeMaterial(meshData.materials?.[materialKey] || {});
        this.materials.set(materialKey, material);
      }
      const mesh = new THREE.Mesh(geometry, material);
      mesh.name = part.key || meshKey;
      mesh.position.fromArray(Array.isArray(part.posMm) ? part.posMm.map(Number) : [0, 0, 0]);
      if (Array.isArray(part.quat) && part.quat.length === 4) mesh.quaternion.fromArray(part.quat.map(Number)).normalize();
      if (Array.isArray(part.scale3) && part.scale3.length === 3) mesh.scale.fromArray(part.scale3.map(Number));
      else {
        const scale = Number(part.scale);
        mesh.scale.setScalar(Number.isFinite(scale) && scale !== 0 ? scale : 1);
      }
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      (this.groups[part.group] || root).add(mesh);
      this.meshes.push(mesh);
    }

    for (const [id, definition] of Object.entries(this.source.toolFrames || {})) {
      const parent = this.groups[definition.parent];
      if (!parent) continue;
      const tool = new THREE.Group();
      tool.name = `${id}_tool_frame`;
      tool.position.fromArray(definition.offsetMm.map(Number));
      parent.add(tool);
      this.toolFrames[id] = tool;
    }
    root.updateMatrixWorld(true);
    return root;
  }

  applyPhysicalState(state = {}, basePose = null) {
    const pose = physicalToCanonical(this.profileId, state);
    for (const joint of this.jointDefs) {
      if (!joint.jointId || joint.sign === 0) continue;
      if (this.meshData.gripper && joint.id === this.meshData.gripper.node) continue;
      const value = Number(pose[joint.jointId]);
      if (!Number.isFinite(value)) continue;
      const axis = new THREE.Vector3().fromArray(joint.axis || [0, 1, 0]);
      if (axis.lengthSq() < 1e-9) continue;
      axis.normalize();
      const motion = new THREE.Quaternion().setFromAxisAngle(
        axis,
        (Number(joint.sign ?? 1) * value + Number(joint.offsetDeg || 0)) * DEG
      );
      this.groups[joint.id].quaternion.copy(this.groups[joint.id].userData.baseQuaternion).multiply(motion).normalize();
    }
    this._applySingleJawGripper(pose);
    if (this.profileId === 'lekiwi' && basePose) {
      const ground = Number(this.meshData.groundOffsetMm) || 0;
      this.root.position.set(Number(basePose.x) || 0, ground, Number(basePose.z) || 0);
      // RoboBuddy_AI's canonical LeKiwi mobile visual mapping uses thetaSign: -1.
      this.root.rotation.set(0, -(Number(basePose.yaw) || 0), 0);
    }
    this.root.updateMatrixWorld(true);
  }

  _applySingleJawGripper(pose) {
    const definition = this.meshData.gripper;
    if (!definition?.node) return;
    const group = this.groups[definition.node];
    const joint = group?.userData?.joint;
    if (!group || !joint) return;
    const value = Number(pose[definition.jointId]);
    if (!Number.isFinite(value)) return;
    const open = Number(definition.openValue);
    const close = Number(definition.closeValue);
    const denominator = Math.max(1, Math.abs(close - open));
    const openRatio = clamp((close - value) / denominator, 0, 1);
    const degrees = (Number(definition.sign) || 1) * THREE.MathUtils.lerp(
      Number(definition.closedDeg) || 0,
      Number(definition.openDeg) || 0,
      openRatio
    );
    const axis = new THREE.Vector3().fromArray(joint.axis || [0, 1, 0]).normalize();
    const motion = new THREE.Quaternion().setFromAxisAngle(axis, degrees * DEG);
    group.quaternion.copy(group.userData.baseQuaternion).multiply(motion).normalize();
  }

  getWorldPosition(id) {
    const object = this.toolFrames[id] || this.groups[id];
    if (!object) return null;
    this.root.updateMatrixWorld(true);
    return object.getWorldPosition(new THREE.Vector3());
  }

  getWorldQuaternion(id) {
    const object = this.toolFrames[id] || this.groups[id];
    if (!object) return null;
    this.root.updateMatrixWorld(true);
    return object.getWorldQuaternion(new THREE.Quaternion());
  }

  getBounds() {
    const bounds = Array.isArray(this.meshData.bboxMm) ? this.meshData.bboxMm.map(Number) : [-200, 0, -200, 200, 400, 200];
    return {
      min: new THREE.Vector3(bounds[0], bounds[1], bounds[2]),
      max: new THREE.Vector3(bounds[3], bounds[4], bounds[5]),
    };
  }

  getOpenArmTool(side = 'left') {
    return this.getWorldPosition(side);
  }

  dispose() {
    this.geometries.forEach((geometry) => geometry.dispose());
    this.materials.forEach((material) => material.dispose());
    this.geometries.clear();
    this.materials.clear();
  }
}

export function canonicalVisualProvenance(profileId) {
  const visual = CANONICAL_VISUALS[profileId];
  return visual ? {
    repository: 'jivishov/RoboBuddy_AI',
    repositoryRevision: ROBOBUDDY_AI_VISUAL_REVISION,
    robotId: visual.robotId,
    modelRevision: visual.sourceModelRevision,
    module: visual.module,
  } : null;
}
