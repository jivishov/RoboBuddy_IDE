import * as THREE from 'three';
import { MICRODUCK_VISUAL_CUE_LIMITS } from './visual-cue-contract.js';
import { MICRODUCK_BALL_BOUNDARY_RADIUS_M, MICRODUCK_DUCK_BOUNDARY_RADIUS_M, MICRODUCK_FIELD_HALF_EXTENT_M } from './field-bounds.js';

const METRES_TO_SCENE = 1000;

function scenePoint(point) {
  return new THREE.Vector3(point[0] * METRES_TO_SCENE, point[2] * METRES_TO_SCENE, -point[1] * METRES_TO_SCENE);
}

function hex(color) { return Number.parseInt(String(color).slice(1), 16); }

function labelSprite(color) {
  const canvas = document.createElement('canvas');
  canvas.width = 640;
  canvas.height = 96;
  const context = canvas.getContext('2d');
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.minFilter = THREE.LinearFilter;
  const material = new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false, depthWrite: false });
  const sprite = new THREE.Sprite(material);
  sprite.renderOrder = 10;
  return { canvas, context, texture, material, sprite, color, text: '' };
}

function drawLabel(record, text) {
  if (record.text === text) return;
  record.text = text;
  const { context, canvas, color, texture } = record;
  context.clearRect(0, 0, canvas.width, canvas.height);
  context.fillStyle = 'rgba(16, 25, 31, 0.9)';
  context.fillRect(4, 4, canvas.width - 8, canvas.height - 8);
  context.fillStyle = color;
  context.fillRect(4, 4, 11, canvas.height - 8);
  context.font = '600 25px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace';
  context.textBaseline = 'middle';
  context.fillStyle = '#f5fbff';
  context.fillText(text, 30, canvas.height / 2 + 1);
  texture.needsUpdate = true;
}

function disposeObject(object) {
  object.traverse((node) => {
    node.geometry?.dispose?.();
    if (Array.isArray(node.material)) node.material.forEach((material) => material.dispose?.());
    else node.material?.dispose?.();
    node.material?.map?.dispose?.();
  });
  object.removeFromParent();
}

function cloneCue(cue) {
  return JSON.parse(JSON.stringify(cue));
}

function planarDistance(left, right) { return Math.hypot(left[0] - right[0], left[1] - right[1]); }

function eastEdge(anchor) {
  return MICRODUCK_FIELD_HALF_EXTENT_M - (anchor === 'ball' ? MICRODUCK_BALL_BOUNDARY_RADIUS_M : MICRODUCK_DUCK_BOUNDARY_RADIUS_M);
}

function rulerTicks(points, distanceM, majorStepM, minorStepM) {
  const direction = points[1].clone().sub(points[0]).normalize();
  const sideways = new THREE.Vector3(-direction.z, 0, direction.x);
  if (sideways.lengthSq() < 1e-8) sideways.set(1, 0, 0);
  sideways.normalize();
  const segments = [];
  const minorCount = Math.floor(distanceM / minorStepM + 1e-9);
  for (let index = 0; index <= minorCount; index += 1) {
    const metres = index * minorStepM;
    const major = Math.abs(metres / majorStepM - Math.round(metres / majorStepM)) < 1e-7;
    const center = points[0].clone().addScaledVector(direction, metres * METRES_TO_SCENE);
    const half = (major ? 0.11 : 0.055) * METRES_TO_SCENE;
    segments.push(center.clone().addScaledVector(sideways, -half), center.clone().addScaledVector(sideways, half));
  }
  return { direction, sideways, segments };
}

export class MicroDuckVisualCueLayer {
  constructor(scene) {
    this.scene = scene;
    this.records = new Map();
  }

  get size() { return this.records.size; }

  list() { return [...this.records.values()].map(({ cue }) => cloneCue(cue)).sort((left, right) => left.id.localeCompare(right.id)); }

  upsert(cue, physics) {
    const existing = this.records.get(cue.id);
    if (!existing && this.records.size >= MICRODUCK_VISUAL_CUE_LIMITS.maxCues) {
      const error = new Error(`At most ${MICRODUCK_VISUAL_CUE_LIMITS.maxCues} visual cues may be active.`);
      error.code = 'INVALID_ARGUMENT';
      throw error;
    }
    if (existing) this.remove(cue.id);
    const record = this.createRecord(cue, physics);
    this.records.set(cue.id, record);
    this.scene.add(record.object);
    return { created: !existing, cue: cloneCue(cue) };
  }

  remove(id) {
    const record = this.records.get(id);
    if (!record) return false;
    this.records.delete(id);
    disposeObject(record.object);
    return true;
  }

  clear() {
    const count = this.records.size;
    for (const id of [...this.records.keys()]) this.remove(id);
    return count;
  }

  dispose() { this.clear(); }

  createRecord(cue, physics) {
    if (cue.kind === 'label') {
      const label = labelSprite(cue.color);
      label.sprite.scale.set(Math.min(420, Math.max(150, 80 + cue.text.length * 11)), 46, 1);
      const origin = this.anchorPosition(cue, physics).slice();
      return { cue, object: label.sprite, label, origin };
    }
    if (cue.kind === 'marker') {
      const object = new THREE.Mesh(new THREE.SphereGeometry(cue.sizeM * METRES_TO_SCENE, 20, 14), new THREE.MeshBasicMaterial({ color: hex(cue.color), depthTest: false, transparent: true, opacity: 0.9 }));
      object.renderOrder = 9;
      return { cue, object };
    }
    const points = [scenePoint(cue.start), scenePoint(cue.end)];
    const object = new THREE.Group();
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), new THREE.LineBasicMaterial({ color: hex(cue.color), transparent: true, opacity: 0.95 }));
    line.renderOrder = 8;
    object.add(line);
    if (cue.kind === 'ruler') {
      const distanceM = Math.hypot(cue.end[0] - cue.start[0], cue.end[1] - cue.start[1], cue.end[2] - cue.start[2]);
      const majorStepM = cue.majorStepM || 1;
      const minorStepM = cue.minorStepM || 0.25;
      const ticks = rulerTicks(points, distanceM, majorStepM, minorStepM);
      const tickGeometry = new THREE.BufferGeometry().setFromPoints(ticks.segments);
      object.add(new THREE.LineSegments(tickGeometry, new THREE.LineBasicMaterial({ color: hex(cue.color), transparent: true, opacity: 0.9 })));
      const label = labelSprite(cue.color);
      const text = `${cue.title ? `${cue.title} · ` : ''}${distanceM.toFixed(2)} m`;
      label.sprite.scale.set(Math.min(330, Math.max(140, 75 + text.length * 11)), 42, 1);
      label.sprite.position.copy(points[0]).lerp(points[1], 0.5).add(new THREE.Vector3(0, 55, 0));
      drawLabel(label, text);
      object.add(label.sprite);
      for (let metres = 0; metres <= distanceM + 1e-9; metres += majorStepM) {
        const marker = labelSprite(cue.color);
        const markerText = `${metres.toFixed(metres % 1 ? 1 : 0)} m`;
        marker.sprite.scale.set(92, 26, 1);
        marker.sprite.position.copy(points[0]).addScaledVector(ticks.direction, metres * METRES_TO_SCENE).addScaledVector(ticks.sideways, 145);
        drawLabel(marker, markerText);
        object.add(marker.sprite);
      }
      return { cue, object, label };
    }
    return { cue, object };
  }

  anchorPosition(cue, physics) {
    if (cue.anchor === 'duck') return physics.position;
    if (cue.anchor === 'ball') return physics.ball.position;
    return cue.position;
  }

  sync(physics) {
    for (const record of this.records.values()) {
      const { cue, object } = record;
      object.visible = cue.visible;
      if (cue.kind !== 'label' && cue.kind !== 'marker') continue;
      const point = this.anchorPosition(cue, physics);
      object.position.copy(scenePoint(point)).add(scenePoint(cue.offsetM));
      if (cue.kind === 'label' && (cue.metrics || []).length) {
        const values = cue.metrics.map((metric) => metric === 'covered_m'
          ? `${planarDistance(point, record.origin).toFixed(2)} m covered`
          : `${Math.max(0, eastEdge(cue.anchor) - point[0]).toFixed(2)} m to east edge`);
        drawLabel(record.label, `${cue.text}  ·  ${values.join('  ·  ')}`);
      } else if (cue.kind === 'label') drawLabel(record.label, cue.text);
    }
  }
}
