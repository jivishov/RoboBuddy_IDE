import * as THREE from 'three';
import { decodeMicroDuckBakedVisual } from './baked-visual.js';

const RIG_URL = './assets/microduck/generated/procedural-rig.json';
const VISUAL_URL = './assets/microduck/visual/duck.bin';
const COLORS = Object.freeze({ cream: null, graphite: 0x59616c, lavender: 0x9c83d8, sky: 0x63a8d8 });
const TINTABLE_SOURCE_COLORS = new Set([0xeaeaea, 0xe6e6e6, 0xdddddd, 0xcfdbe5, 0xd9c1dd]);
const MODEL_TO_VIEW = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);

// The upstream terminal bake de-duplicates triangles after sorting each face's
// vertex indexes. Its software rasterizer is winding-agnostic, but WebGL is not:
// using those indexes verbatim creates black/chrome patches from culled and
// oppositely lit faces. Reorient each compact mesh around its own centroid before
// computing browser normals. Positions and part transforms stay source-exact.
function orientBakedTrianglesOutward(geometry) {
  geometry.computeBoundingBox();
  const center = geometry.boundingBox.getCenter(new THREE.Vector3());
  const positions = geometry.getAttribute('position').array;
  const index = geometry.getIndex();
  const indices = index.array;
  let flippedFaces = 0;
  for (let offset = 0; offset < indices.length; offset += 3) {
    const a = indices[offset] * 3;
    const b = indices[offset + 1] * 3;
    const c = indices[offset + 2] * 3;
    const abx = positions[b] - positions[a];
    const aby = positions[b + 1] - positions[a + 1];
    const abz = positions[b + 2] - positions[a + 2];
    const acx = positions[c] - positions[a];
    const acy = positions[c + 1] - positions[a + 1];
    const acz = positions[c + 2] - positions[a + 2];
    const nx = aby * acz - abz * acy;
    const ny = abz * acx - abx * acz;
    const nz = abx * acy - aby * acx;
    const fx = (positions[a] + positions[b] + positions[c]) / 3 - center.x;
    const fy = (positions[a + 1] + positions[b + 1] + positions[c + 1]) / 3 - center.y;
    const fz = (positions[a + 2] + positions[b + 2] + positions[c + 2]) / 3 - center.z;
    if (nx * fx + ny * fy + nz * fz < 0) {
      const swap = indices[offset + 1];
      indices[offset + 1] = indices[offset + 2];
      indices[offset + 2] = swap;
      flippedFaces += 1;
    }
  }
  index.needsUpdate = true;
  geometry.computeVertexNormals();
  geometry.userData = { sourceWinding: 'sorted-upstream-indices', browserWinding: 'centroid-oriented', flippedFaces };
}

function quaternionFromWxyz(values = [1, 0, 0, 0]) {
  const [w, x, y, z] = values.map(Number);
  return new THREE.Quaternion(
    Number.isFinite(x) ? x : 0,
    Number.isFinite(y) ? y : 0,
    Number.isFinite(z) ? z : 0,
    Number.isFinite(w) ? w : 1
  ).normalize();
}

export class MicroDuckRigAdapter {
  static async load() {
    const [rigResponse, visualResponse] = await Promise.all([
      fetch(RIG_URL, { cache: 'force-cache' }),
      fetch(VISUAL_URL, { cache: 'force-cache' }),
    ]);
    if (!rigResponse.ok) throw new Error(`MicroDuck rig contract returned HTTP ${rigResponse.status}.`);
    if (!visualResponse.ok) throw new Error(`MicroDuck official visual returned HTTP ${visualResponse.status}.`);
    const [data, visual] = await Promise.all([
      rigResponse.json(),
      visualResponse.arrayBuffer().then(decodeMicroDuckBakedVisual),
    ]);
    if (data?.schema !== 'robobuddy.microduck-procedural-rig.v1' || data?.joints?.length !== 14) {
      throw new Error('MicroDuck rig contract is invalid.');
    }
    if (visual.bodies.length !== data.bodies.length) throw new Error('MicroDuck official visual hierarchy does not match the pinned runtime hierarchy.');
    return new MicroDuckRigAdapter(data, visual);
  }

  constructor(data, visual) {
    this.data = data;
    this.visual = visual;
    this.root = new THREE.Group();
    this.root.name = 'microduck-official-runtime-visual-rig';
    this.root.scale.setScalar(1000);
    this.modelRoot = new THREE.Group();
    this.modelRoot.name = 'microduck-z-up-model-root';
    this.modelRoot.quaternion.copy(MODEL_TO_VIEW);
    this.root.add(this.modelRoot);
    this.bodies = new Map();
    this.bodyList = [];
    this.joints = new Map();
    this.geometries = new Set();
    this.materials = new Set();
    this.visualMaterials = new Map();
    this.officialParts = [];
    this.nodeBoundsScratch = new THREE.Box3();
    this.groundBoundsScratch = new THREE.Box3();
    this.variant = 'walking';
    this.color = 'cream';
    this.buildHierarchy();
    this.buildOfficialVisual();
    this.buildConfiguredAttachments();
    this.applyState({});
  }

  material(color, { configuredApproximation = false, tintable = false } = {}) {
    const material = new THREE.MeshStandardMaterial({
      color,
      roughness: configuredApproximation ? 0.68 : 0.82,
      metalness: 0,
      side: configuredApproximation ? THREE.FrontSide : THREE.DoubleSide,
    });
    material.userData = { configuredApproximation, tintable, sourceColor: color };
    this.materials.add(material);
    return material;
  }

  buildHierarchy() {
    this.visual.bodies.forEach((visualBody, index) => {
      const contractBody = this.data.bodies[index];
      const expectedParent = visualBody.parentIndex < 0 ? null : this.data.bodies[visualBody.parentIndex]?.name;
      if (!contractBody || contractBody.parent !== expectedParent) throw new Error(`MicroDuck official visual body ${index} does not match the pinned hierarchy.`);
      const group = new THREE.Group();
      group.name = contractBody.name;
      group.position.fromArray(visualBody.positionM);
      group.quaternion.copy(quaternionFromWxyz(visualBody.quaternionWxyz));
      group.userData = { provenance: 'exact_apache_runtime_monitor_visual' };
      this.bodyList.push(group);
      this.bodies.set(contractBody.name, group);
      (visualBody.parentIndex < 0 ? this.modelRoot : this.bodyList[visualBody.parentIndex]).add(group);

      if (visualBody.jointWireIndex >= 0) {
        const name = this.data.jointContract.wireJointOrder[visualBody.jointWireIndex];
        const joint = this.data.joints.find((item) => item.name === name);
        if (!joint || name === 'mouth') throw new Error(`MicroDuck official visual joint index ${visualBody.jointWireIndex} is invalid.`);
        this.joints.set(name, {
          body: group,
          axis: new THREE.Vector3().fromArray(visualBody.axis).normalize(),
          rest: group.quaternion.clone(),
          range: joint.rangeRad,
        });
      }
    });
  }

  officialMaterial(color) {
    let material = this.visualMaterials.get(color);
    if (!material) {
      material = this.material(color, { tintable: TINTABLE_SOURCE_COLORS.has(color) });
      material.userData.provenance = 'pollen-robotics/microduck robotctl/assets/duck.bin';
      this.visualMaterials.set(color, material);
    }
    return material;
  }

  buildOfficialVisual() {
    const geometries = this.visual.meshes.map((source, index) => {
      const geometry = new THREE.BufferGeometry();
      geometry.name = `microduck-official-mesh-${index}`;
      geometry.setAttribute('position', new THREE.BufferAttribute(source.positions, 3));
      geometry.setIndex(new THREE.BufferAttribute(source.indices, 1));
      orientBakedTrianglesOutward(geometry);
      geometry.computeBoundingSphere();
      this.geometries.add(geometry);
      return geometry;
    });
    this.visual.parts.forEach((part, index) => {
      const mesh = new THREE.Mesh(geometries[part.meshIndex], this.officialMaterial(part.color));
      mesh.name = `official-part-${index}-mesh-${part.meshIndex}`;
      mesh.position.fromArray(part.positionM);
      mesh.quaternion.copy(quaternionFromWxyz(part.quaternionWxyz));
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      mesh.userData = { provenance: 'exact_apache_runtime_monitor_visual', configuredApproximation: false };
      this.bodyList[part.bodyIndex].add(mesh);
      this.officialParts[index] = mesh;
    });
  }

  buildConfiguredAttachments() {
    const mouth = this.data.configuredAttachments?.mouth;
    const parent = this.bodies.get(mouth?.parentBody);
    if (parent) {
      this.mouthPivot = new THREE.Group();
      this.mouthPivot.name = 'configured-mouth-pivot';
      this.mouthPivot.position.fromArray(mouth.pivotM);
      this.mouthAxis = new THREE.Vector3().fromArray(mouth.axis || [0, 1, 0]).normalize();
      this.mouthClosed = Number(mouth.closedRad) || 0;
      this.mouthOpen = Number(mouth.openRad) || 0;
      parent.add(this.mouthPivot);
      this.root.updateWorldMatrix(true, true);
      for (const partIndex of mouth.officialPartIndices || []) {
        const lowerBill = this.officialParts[partIndex];
        if (!lowerBill || lowerBill.parent !== parent) throw new Error(`Configured MicroDuck mouth part ${partIndex} does not belong to ${mouth.parentBody}.`);
        this.mouthPivot.attach(lowerBill);
        lowerBill.userData = { ...lowerBill.userData, articulatedByConfiguredPivot: true };
      }
    }
    this.rollers = [];
    for (const roller of this.data.configuredAttachments?.rollers || []) {
      const body = this.bodies.get(roller.parentBody);
      if (!body) continue;
      const assembly = new THREE.Group();
      assembly.name = 'configured-passive-roller-assembly';
      assembly.position.fromArray(roller.offsetM);
      const wheelGeometry = new THREE.CylinderGeometry(roller.radiusM, roller.radiusM, roller.widthM, 24);
      const hubGeometry = new THREE.CylinderGeometry(roller.radiusM * 0.34, roller.radiusM * 0.34, roller.widthM * 1.08, 18);
      this.geometries.add(wheelGeometry);
      this.geometries.add(hubGeometry);
      const wheel = new THREE.Mesh(
        wheelGeometry,
        this.material(0xfab601, { configuredApproximation: true })
      );
      const hub = new THREE.Mesh(
        hubGeometry,
        this.material(0x39424e, { configuredApproximation: true })
      );
      wheel.castShadow = true;
      hub.castShadow = true;
      wheel.userData = { provenance: 'original_configured_approximation', configuredApproximation: true };
      hub.userData = { provenance: 'original_configured_approximation', configuredApproximation: true };
      assembly.add(wheel, hub);
      body.add(assembly);
      this.rollers.push(assembly);
    }
  }

  setVariant(variant) {
    this.variant = variant === 'roller' ? 'roller' : 'walking';
    for (const roller of this.rollers) roller.visible = this.variant === 'roller';
  }

  setColor(color) {
    this.color = Object.hasOwn(COLORS, color) ? color : 'cream';
    for (const material of this.visualMaterials.values()) {
      const selected = COLORS[this.color];
      material.color.setHex(material.userData.tintable && selected !== null ? selected : material.userData.sourceColor);
    }
  }

  applyState(state = {}) {
    for (const [name, joint] of this.joints) {
      const requested = Number(state[name] ?? 0);
      const min = joint.range?.[0] ?? -Infinity;
      const max = joint.range?.[1] ?? Infinity;
      const value = Math.max(min, Math.min(max, Number.isFinite(requested) ? requested : 0));
      joint.body.quaternion.copy(joint.rest).multiply(new THREE.Quaternion().setFromAxisAngle(joint.axis, value));
    }
    if (this.mouthPivot) {
      const amount = Math.max(0, Math.min(1, Number(state.mouth ?? 0)));
      this.mouthPivot.quaternion.setFromAxisAngle(this.mouthAxis, THREE.MathUtils.lerp(this.mouthClosed, this.mouthOpen, amount));
    }
  }

  visibleBounds(target = new THREE.Box3()) {
    target.makeEmpty();
    this.root.updateWorldMatrix(true, true);
    this.root.traverseVisible((node) => {
      if (!node.isMesh || !node.geometry) return;
      if (!node.geometry.boundingBox) node.geometry.computeBoundingBox();
      target.union(this.nodeBoundsScratch.copy(node.geometry.boundingBox).applyMatrix4(node.matrixWorld));
    });
    return target;
  }

  applyRootPose(position = [0, 0, 0], quaternionWxyz = [1, 0, 0, 0]) {
    this.root.position.set((Number(position[0]) || 0) * 1000, (Number(position[2]) || 0) * 1000, -(Number(position[1]) || 0) * 1000);
    this.root.quaternion.set(Number(quaternionWxyz[1]) || 0, Number(quaternionWxyz[3]) || 0, -(Number(quaternionWxyz[2]) || 0), Number(quaternionWxyz[0]) || 1).normalize();
    const bounds = this.visibleBounds(this.groundBoundsScratch);
    if (!bounds.isEmpty() && Number.isFinite(bounds.min.y)) {
      this.root.position.y -= bounds.min.y;
      this.root.updateWorldMatrix(true, true);
    }
  }

  getBounds(target = new THREE.Box3()) { return this.visibleBounds(target); }

  getSiteWorldPose(name) {
    const site = this.data.sites?.find((item) => item.name === name);
    const body = this.bodies.get(site?.body);
    if (!site || !body) return null;
    body.updateWorldMatrix(true, false);
    const position = body.localToWorld(new THREE.Vector3().fromArray(site.pos || [0, 0, 0]));
    const bodyQuaternion = body.getWorldQuaternion(new THREE.Quaternion());
    const quaternion = bodyQuaternion.multiply(quaternionFromWxyz(site.quatWxyz));
    return { position, quaternion, frame: name, modeled: true };
  }

  dispose() {
    for (const geometry of this.geometries) geometry.dispose();
    for (const material of this.materials) material.dispose();
    this.root.removeFromParent();
  }
}
