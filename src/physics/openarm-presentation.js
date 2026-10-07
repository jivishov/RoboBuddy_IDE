import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js';

// One basis conversion: MuJoCo metres/Z-up -> existing IDE millimetres/Y-up.
export const toThreePosition = (p = [0, 0, 0]) => new THREE.Vector3(p[0] * 1000, p[2] * 1000, -p[1] * 1000);
const BASIS = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
export function toThreeQuaternion(q = [1, 0, 0, 0]) {
  return BASIS.clone().multiply(new THREE.Quaternion(q[1], q[2], q[3], q[0])).multiply(BASIS.clone().invert()).normalize();
}

/** Render exactly the component geometry used for collision, without a second animated rig. */
export class OpenArmPresentation {
  constructor(definition, { preview = false } = {}) {
    this.root = new THREE.Group();
    this.root.name = preview ? 'openarm-staged-equipment-preview' : 'openarm-authoritative-contact-geometry';
    this.bodyGroups = new Map(); this.geomMeshes = new Map(); this.geometries = new Set(); this.materials = new Set();
    const cache = new Map();
    this.visualBodies = new Set((definition.visualGeoms || []).map(g=>g.bodyId)); this.visualMeshes = []; this.collisionMeshes = []; this.preview = preview;
    for (const geom of [...definition.geoms, ...(preview ? [] : definition.visualGeoms || [])]) {
      if (geom.id === 'floor') continue; // The physical floor is presented by the ground grid.
      let geometry;
      const size = geom.sizeM || [];
      if (geom.type === 'mesh') {
        geometry = cache.get(geom.mesh);
        if (!geometry) {
          const payload = definition.meshes[geom.mesh];
          if (!payload) throw new Error(`Missing shared OpenArm geometry ${geom.mesh}`);
          const vertices = new Float32Array(payload.vertices.length);
          for (let i = 0; i < vertices.length; i += 3) {
            vertices[i] = payload.vertices[i] * 1000;
            vertices[i + 1] = payload.vertices[i + 2] * 1000;
            vertices[i + 2] = -payload.vertices[i + 1] * 1000;
          }
          geometry = new THREE.BufferGeometry();
          geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
          geometry.setIndex(payload.indices); geometry.computeVertexNormals();
          cache.set(geom.mesh, geometry);
        }
      } else if (geom.type === 'box') geometry = new THREE.BoxGeometry(size[0] * 2000, size[2] * 2000, size[1] * 2000);
      else if (geom.type === 'cylinder') geometry = new THREE.CylinderGeometry(size[0] * 1000, size[0] * 1000, size[1] * 2000, 40);
      else if (geom.type === 'sphere') geometry = new THREE.SphereGeometry(size[0] * 1000, 24, 16);
      else throw new Error(`Unsupported shared OpenArm presentation geometry ${geom.type}`);
      geometry.computeBoundingBox(); this.geometries.add(geometry);
      const robot = geom.bodyId.startsWith('openarm_');
      const color = robot ? (geom.id.includes('finger') || geom.id.includes('ee_base') ? 0x36434a : 0x75818a) : new THREE.Color(...(geom.rgba || [.45, .51, .54]).slice(0, 3));
      const opacity = preview ? .65 : (geom.rgba?.[3] ?? 1);
      const material = new THREE.MeshStandardMaterial({ color, roughness: geom.roughness ?? (opacity < 1 ? .23 : .64), metalness: geom.metalness ?? (robot ? .26 : .06), wireframe: preview, transparent: opacity < 1, opacity, depthWrite: opacity === 1, side: opacity < 1 ? THREE.DoubleSide : THREE.FrontSide });
      this.materials.add(material);
      const mesh = new THREE.Mesh(geometry, material); mesh.name = geom.id;
      mesh.position.copy(toThreePosition(geom.positionM)); mesh.quaternion.copy(toThreeQuaternion(geom.quaternionWxyz));
      mesh.castShadow = !preview; mesh.receiveShadow = !preview;
      mesh.userData.physicalGeometryId = geom.id; mesh.userData.physicalBodyId = geom.bodyId;
      let group = this.bodyGroups.get(geom.bodyId);
      if (!group) { group = new THREE.Group(); group.name = geom.bodyId; this.root.add(group); this.bodyGroups.set(geom.bodyId, group); }
      group.add(mesh);
      if (geom.visualOnly) { this.visualMeshes.push(mesh); } else { this.geomMeshes.set(geom.id, mesh); this.collisionMeshes.push(mesh); }
    }
  }
  setCollisionView(value) {
    for (const mesh of this.visualMeshes) mesh.visible = !value;
    for (const mesh of this.collisionMeshes) mesh.visible = Boolean(value || this.preview || !this.visualBodies.has(mesh.userData.physicalBodyId));
  }
  applyObservation(observation) {
    for (const [bodyId, group] of this.bodyGroups) {
      if (bodyId === 'world') continue;
      const body = observation.bodies?.[bodyId];
      if (!body) throw new Error(`Missing observed OpenArm body ${bodyId}`);
      group.position.copy(toThreePosition(body.positionM)); group.quaternion.copy(toThreeQuaternion(body.quaternionWxyz));
    }
    this.root.updateMatrixWorld(true);
  }
  setWireframe(value) { for (const material of this.materials) material.wireframe = Boolean(value); }
  dispose() { this.root.removeFromParent(); this.geometries.forEach(x => x.dispose()); this.materials.forEach(x => x.dispose()); this.bodyGroups.clear(); this.geomMeshes.clear(); }
}
