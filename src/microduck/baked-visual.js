const MAGIC = 'DUCK';
const FORMAT_VERSION = 1;

class BinaryReader {
  constructor(buffer) {
    if (!(buffer instanceof ArrayBuffer)) throw new TypeError('MicroDuck baked visual must be an ArrayBuffer.');
    this.view = new DataView(buffer);
    this.offset = 0;
  }

  require(byteLength) {
    if (this.offset + byteLength > this.view.byteLength) throw new Error('MicroDuck baked visual is truncated.');
  }

  ascii(byteLength) {
    this.require(byteLength);
    let value = '';
    for (let index = 0; index < byteLength; index += 1) value += String.fromCharCode(this.view.getUint8(this.offset + index));
    this.offset += byteLength;
    return value;
  }

  uint8() { this.require(1); const value = this.view.getUint8(this.offset); this.offset += 1; return value; }
  uint16() { this.require(2); const value = this.view.getUint16(this.offset, true); this.offset += 2; return value; }
  int16() { this.require(2); const value = this.view.getInt16(this.offset, true); this.offset += 2; return value; }
  uint32() { this.require(4); const value = this.view.getUint32(this.offset, true); this.offset += 4; return value; }
  float32() { this.require(4); const value = this.view.getFloat32(this.offset, true); this.offset += 4; return value; }
  float32Array(length) { return Float32Array.from({ length }, () => this.float32()); }
  uint16Array(length) { return Uint16Array.from({ length }, () => this.uint16()); }
}

export function decodeMicroDuckBakedVisual(buffer) {
  const reader = new BinaryReader(buffer);
  const magic = reader.ascii(4);
  const version = reader.uint32();
  if (magic !== MAGIC || version !== FORMAT_VERSION) throw new Error(`Unsupported MicroDuck baked visual ${magic} v${version}.`);

  const meshCount = reader.uint16();
  const bodyCount = reader.uint16();
  const partCount = reader.uint16();
  const meshes = Array.from({ length: meshCount }, () => {
    const vertexCount = reader.uint16();
    const faceCount = reader.uint16();
    const positions = reader.float32Array(vertexCount * 3);
    const indices = reader.uint16Array(faceCount * 3);
    if (!positions.every(Number.isFinite)) throw new Error('MicroDuck baked visual contains a non-finite vertex.');
    if (indices.some((index) => index >= vertexCount)) throw new Error('MicroDuck baked visual contains an out-of-range triangle index.');
    return { vertexCount, faceCount, positions, indices };
  });

  const bodies = Array.from({ length: bodyCount }, () => ({
    parentIndex: reader.int16(),
    jointWireIndex: reader.int16(),
    positionM: Array.from(reader.float32Array(3)),
    quaternionWxyz: Array.from(reader.float32Array(4)),
    axis: Array.from(reader.float32Array(3)),
  }));
  bodies.forEach((body, index) => {
    if (body.parentIndex < -1 || body.parentIndex >= index) throw new Error(`MicroDuck baked visual body ${index} has an invalid parent.`);
    if (body.jointWireIndex < -1 || body.jointWireIndex >= 15) throw new Error(`MicroDuck baked visual body ${index} has an invalid joint index.`);
    if (![...body.positionM, ...body.quaternionWxyz, ...body.axis].every(Number.isFinite)) throw new Error(`MicroDuck baked visual body ${index} contains a non-finite transform.`);
    if (Math.hypot(...body.quaternionWxyz) < 1e-8) throw new Error(`MicroDuck baked visual body ${index} has a zero quaternion.`);
  });

  const parts = Array.from({ length: partCount }, () => {
    const bodyIndex = reader.uint16();
    const meshIndex = reader.uint16();
    const red = reader.uint8();
    const green = reader.uint8();
    const blue = reader.uint8();
    reader.uint8();
    const positionM = Array.from(reader.float32Array(3));
    const quaternionWxyz = Array.from(reader.float32Array(4));
    if (bodyIndex >= bodyCount || meshIndex >= meshCount) throw new Error('MicroDuck baked visual contains an out-of-range part reference.');
    if (![...positionM, ...quaternionWxyz].every(Number.isFinite) || Math.hypot(...quaternionWxyz) < 1e-8) throw new Error('MicroDuck baked visual contains an invalid part transform.');
    return { bodyIndex, meshIndex, color: (red << 16) | (green << 8) | blue, positionM, quaternionWxyz };
  });

  if (reader.offset !== reader.view.byteLength) throw new Error('MicroDuck baked visual has unexpected trailing bytes.');
  return { format: `${MAGIC} v${version}`, meshes, bodies, parts };
}
