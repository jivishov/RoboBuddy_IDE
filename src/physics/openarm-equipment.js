import { OPENARM_LAB_ASSETS, LAB_ASSET_KINDS, validateLabAsset, compileLabAsset } from './openarm-lab-assets.js';
// Declarative, bounded rigid-body equipment. No caller XML, code, URLs or plugins.
export const OPENARM_EQUIPMENT_VERSION = 'robobuddy.openarm.equipment.v1';
export const TABLE_TOP_M = 1.005;
export const EQUIPMENT_KINDS = Object.freeze(['platform', 'tray', 'rack', 'vial', 'block', 'button', 'assembly', ...LAB_ASSET_KINDS]);
export const EQUIPMENT_LIMITS = Object.freeze({ items: 12, partsPerAssembly: 12, geometries: 256, inputBytes: 24000 });
const DEFAULTS = { platform: [.12, .10, .03], tray: [.14, .10, .025], rack: [.12, .09, .03], vial: [.025, .025, .055], block: [.025, .025, .025], button: [.04, .04, .025], assembly: [.12, .10, .05] };
const ID = /^[a-z][a-z0-9_]{0,23}$/;
const BLOCKED_IDS = new Set(['constructor', 'prototype', '__proto__']);
const vec = (v, n, label) => { if (!Array.isArray(v) || v.length !== n || v.some(x => typeof x !== 'number' || !Number.isFinite(x))) throw new TypeError(`${label} must contain ${n} finite numbers`); return [...v]; };
const range = (v, lo, hi, label) => { if (typeof v !== 'number' || !Number.isFinite(v) || v < lo || v > hi) throw new RangeError(`${label} must be ${lo}..${hi}`); return v; };
function plain(v, label) { if (!v || typeof v !== 'object' || Array.isArray(v) || ![Object.prototype, null].includes(Object.getPrototypeOf(v))) throw new TypeError(`${label} must be a plain object`); }
function keys(v, allowed, label) { for (const k of Object.keys(v)) if (!allowed.includes(k)) throw new TypeError(`Unknown ${label} field ${k}`); }
const yawQuat = a => [Math.cos(a / 2), 0, 0, Math.sin(a / 2)];
const join = v => v.join(' ');

export function validateOpenArmEquipment(items) {
  if (!Array.isArray(items) || items.length > EQUIPMENT_LIMITS.items) throw new RangeError(`equipment must contain 0..${EQUIPMENT_LIMITS.items} items`);
  if (new TextEncoder().encode(JSON.stringify(items)).length > EQUIPMENT_LIMITS.inputBytes) throw new RangeError('Equipment input exceeds byte budget');
  const ids = new Set();
  return items.map((raw, i) => {
    const label = `equipment[${i}]`; plain(raw, label);
    if (Object.values(raw).some(v => v === null)) throw new TypeError('Omit optional equipment fields instead of using null');
    keys(raw, ['id', 'label', 'kind', 'position_m', 'yaw_rad', 'dimensions_m', 'mass_kg', 'dynamic', 'parts', 'quaternion_wxyz'], label);
    if (typeof raw.id !== 'string' || !ID.test(raw.id) || BLOCKED_IDS.has(raw.id) || ids.has(raw.id)) throw new TypeError(`${label}.id must be a unique lowercase identifier (1..24 characters)`);
    ids.add(raw.id);
    if (!EQUIPMENT_KINDS.includes(raw.kind)) throw new TypeError(`${label}.kind is not supported`);
    if (raw.label != null && (typeof raw.label !== 'string' || raw.label.length > 64 || /[\x00-\x1f]/.test(raw.label))) throw new TypeError('Equipment label must be at most 64 printable characters');
    const dimensions = vec(raw.dimensions_m ?? OPENARM_LAB_ASSETS[raw.kind]?.dimensions_m ?? DEFAULTS[raw.kind], 3, `${label}.dimensions_m`);
    dimensions.forEach(x => range(x, raw.kind === 'tile' ? .004 : .008, OPENARM_LAB_ASSETS[raw.kind] ? .90 : .40, 'Equipment dimension (m)'));
    if (raw.kind === 'vial' && Math.abs(dimensions[0] - dimensions[1]) > 1e-9) throw new RangeError('A cylindrical vial requires equal X/Y diameters');
    const position = vec(raw.position_m, 3, `${label}.position_m`);
    const yaw = range(raw.yaw_rad ?? 0, -Math.PI, Math.PI, 'yaw_rad');
    const quaternion = raw.quaternion_wxyz === undefined ? null : vec(raw.quaternion_wxyz, 4, 'quaternion_wxyz');
    if (quaternion && (raw.yaw_rad !== undefined || Math.abs(Math.hypot(...quaternion)-1) > 1e-6)) throw new RangeError('Use normalized quaternion_wxyz OR yaw_rad, not both');
    const q = quaternion || yawQuat(yaw);
    const [qw,qx,qy,qz] = q;
    const rotate = ([x,y,z]) => [
      (1-2*(qy*qy+qz*qz))*x + 2*(qx*qy-qw*qz)*y + 2*(qx*qz+qw*qy)*z,
      2*(qx*qy+qw*qz)*x + (1-2*(qx*qx+qz*qz))*y + 2*(qy*qz-qw*qx)*z,
      2*(qx*qz-qw*qy)*x + 2*(qy*qz+qw*qx)*y + (1-2*(qx*qx+qy*qy))*z,
    ];
    // Check all corners after rotation, including a funnel lying on its side.
    for (const x of [-dimensions[0]/2,dimensions[0]/2]) for (const y of [-dimensions[1]/2,dimensions[1]/2]) for (const z of [0,dimensions[2]]) {
      const p = rotate([x,y,z]).map((v,i)=>v+position[i]);
      if (p[0] < -1e-9 || p[0] > .82+1e-9 || p[1] < -.55-1e-9 || p[1] > .55+1e-9 || p[2] < TABLE_TOP_M-1e-9 || p[2] > 1.90+1e-9) throw new RangeError('Initial equipment envelope must be over the tabletop, above its surface, and below 1.90 m');
    }
    if (raw.dynamic != null && typeof raw.dynamic !== 'boolean') throw new TypeError('dynamic must be a boolean');
    const dynamic = raw.dynamic ?? OPENARM_LAB_ASSETS[raw.kind]?.dynamic ?? ['vial', 'block'].includes(raw.kind);
    if (raw.kind === 'button' && (dynamic || dimensions.some((x, i) => Math.abs(x - DEFAULTS.button[i]) > 1e-9))) throw new RangeError('button is a fixed, dimensioned spring-loaded mechanism; custom dimensions/dynamic base are unsupported');
    const mass = range(raw.mass_kg ?? OPENARM_LAB_ASSETS[raw.kind]?.mass_kg ?? (dynamic ? .05 : .20), .005, 2, 'mass_kg');
    if (OPENARM_LAB_ASSETS[raw.kind]) validateLabAsset({ kind: raw.kind, dimensions_m: dimensions, dynamic });
    let parts = null;
    if (raw.kind === 'assembly') {
      if (!Array.isArray(raw.parts) || raw.parts.length < 1 || raw.parts.length > EQUIPMENT_LIMITS.partsPerAssembly) throw new RangeError('assembly requires 1..12 parts');
      parts = raw.parts.map((part, k) => {
        plain(part, 'part'); keys(part, ['shape', 'position_m', 'dimensions_m', 'radius_m', 'height_m'], 'part');
        if (!['box', 'cylinder', 'sphere'].includes(part.shape)) throw new TypeError('Assembly shape must be box, cylinder or sphere');
        const p = vec(part.position_m, 3, 'part.position_m'); let size, half;
        if (part.shape === 'box') {
          if (part.radius_m != null || part.height_m != null) throw new TypeError('Box parts use dimensions_m only');
          const dim = vec(part.dimensions_m, 3, 'part.dimensions_m'); dim.forEach(x => range(x, .002, .4, 'part dimension'));
          size = dim.map(x => x / 2); half = size;
        } else {
          if (part.dimensions_m != null || (part.shape === 'sphere' && part.height_m != null)) throw new TypeError('Round parts use radius_m and, for cylinders, height_m');
          const radius = range(part.radius_m, .001, .2, 'radius_m');
          size = part.shape === 'sphere' ? [radius] : [radius, range(part.height_m, .002, .4, 'height_m') / 2];
          half = [radius, radius, part.shape === 'sphere' ? radius : size[1]];
        }
        if (Math.abs(p[0]) + half[0] > dimensions[0]/2 + 1e-9 || Math.abs(p[1]) + half[1] > dimensions[1]/2 + 1e-9 || p[2] - half[2] < -1e-9 || p[2] + half[2] > dimensions[2] + 1e-9) throw new RangeError(`Part ${k} is outside its declared assembly envelope`);
        return { shape: part.shape, position_m: p, ...(part.shape === 'box' ? { dimensions_m: size.map(x => x * 2) } : { radius_m: size[0], ...(part.shape === 'cylinder' ? { height_m: size[1] * 2 } : {}) }) };
      });
    } else if (raw.parts != null) throw new TypeError('Only assembly equipment accepts parts');
    return { id: raw.id, label: raw.label ?? raw.id, kind: raw.kind, position_m: position, ...(quaternion ? { quaternion_wxyz: quaternion } : { yaw_rad: yaw }), dimensions_m: dimensions, mass_kg: mass, dynamic, ...(parts ? { parts } : {}) };
  });
}

// Accepts normalized input internally. Public boundaries always validate raw input.
export function compileEquipmentDefinitions(equipment) {
  const geoms = [], bodies = [], passiveJoints = [], records = [], meshes = {}; let xml = '', assetXml = '';
  for (const e of equipment) {
    if (OPENARM_LAB_ASSETS[e.kind]) {
      const compiled = compileLabAsset(e);
      xml += compiled.xml; assetXml += compiled.assetXml; Object.assign(meshes, compiled.meshes);
      geoms.push(...compiled.geoms); bodies.push(...compiled.bodies); records.push(...compiled.records);
      continue;
    }
    const id = `lab_${e.id}`, [w, depth, h] = e.dimensions_m;
    const parts = [];
    const add = (type, pos, size, suffix) => parts.push({ id: `${id}_${suffix}`, bodyId: id, type, positionM: pos, quaternionWxyz: [1, 0, 0, 0], sizeM: size, rgba: e.dynamic ? [.25, .65, .77, 1] : [.35, .46, .54, 1] });
    if (['platform', 'block'].includes(e.kind)) add('box', [0, 0, h/2], [w/2, depth/2, h/2], 'solid');
    else if (e.kind === 'vial') add('cylinder', [0, 0, h/2], [w/2, h/2], 'solid');
    else if (e.kind === 'assembly') e.parts.forEach((p, i) => add(p.shape, p.position_m, p.shape === 'box' ? p.dimensions_m.map(x => x/2) : p.shape === 'sphere' ? [p.radius_m] : [p.radius_m, p.height_m/2], `part${i}`));
    else {
      const t = Math.min(.003, w/6, depth/6, h/4);
      const wallHeight = e.kind === 'button' ? .013 : h;
      add('box', [0, 0, t/2], [w/2, depth/2, t/2], 'base');
      for (const sign of [-1, 1]) {
        add('box', [sign * (w-t)/2, 0, wallHeight/2], [t/2, depth/2, wallHeight/2], `wall_x${sign === 1 ? 'p' : 'n'}`);
        add('box', [0, sign * (depth-t)/2, wallHeight/2], [w/2, t/2, wallHeight/2], `wall_y${sign === 1 ? 'p' : 'n'}`);
      }
      if (e.kind === 'rack') {
        for (const x of [-w/6, w/6]) add('box', [x, 0, h/2], [t/2, depth/2, h/2], `divider_${x < 0 ? 'a' : 'b'}`);
        add('box', [0, 0, h/2], [w/2, t/2, h/2], 'divider_c');
      }
    }
    const quat = e.quaternion_wxyz || yawQuat(e.yaw_rad);
    bodies.push({ id, ...(e.dynamic ? { freeJointId: `${id}_free` } : {}) });
    const geomXml = p => `<geom name="${p.id}" type="${p.type}" pos="${join(p.positionM)}" size="${join(p.sizeM)}" mass="${e.mass_kg / parts.length}" contype="3" conaffinity="3" friction="0.8 0.005 0.0005" solref="0.005 1" rgba="${join(p.rgba)}"/>`;
    xml += `<body name="${id}" pos="${join(e.position_m)}" quat="${join(quat)}">${e.dynamic ? `<freejoint name="${id}_free"/>` : ''}${parts.map(geomXml).join('')}`;
    let affordances = { graspReferenceM: [0, 0, h/2], topReferenceM: [0, 0, h] };
    if (e.kind === 'button') {
      const capId = `${id}_cap`, jointId = `${id}_press`;
      bodies.push({ id: capId }); passiveJoints.push({ id: jointId, equipmentId: e.id, type: 'slide', unit: 'm', range: [0, .006], pressedThresholdM: .004 });
      const cap = { id: `${id}_plunger`, bodyId: capId, type: 'box', positionM: [0, 0, 0], quaternionWxyz: [1,0,0,0], sizeM: [.014,.014,.005], rgba: [.27,.70,.48,1] };
      xml += `<body name="${capId}" pos="0 0 .020"><joint name="${jointId}" type="slide" axis="0 0 -1" limited="true" range="0 .006" stiffness="400" damping="2"/><geom name="${cap.id}" type="box" size=".014 .014 .005" mass=".02" contype="3" conaffinity="3" friction="0.8 .005 .0005" solref=".005 1" rgba=".27 .70 .48 1"/></body>`;
      parts.push(cap); affordances = { pressReferenceM: [0,0,.025], passiveJointId: jointId, travelM: .006 };
    }
    xml += '</body>'; geoms.push(...parts);
    records.push({ ...e, bodyId: id, geometryIds: parts.map(p => p.id), affordances, processModel: e.kind === 'button' ? 'passive spring-loaded rigid mechanism; no electrical process' : 'dry rigid-body geometry only' });
  }
  if (geoms.length > EQUIPMENT_LIMITS.geometries) throw new RangeError('Equipment exceeds the collision geometry budget');
  return { xml, assetXml, meshes, geoms, bodies, passiveJoints, records };
}
export function appendEquipmentXml(baseXml, equipment) {
  const compiled = compileEquipmentDefinitions(equipment);
  if ((baseXml.match(/<\/worldbody>/g) || []).length !== 1) throw new Error('Unexpected OpenArm worldbody structure');
  let xml = baseXml.replace('</worldbody>', `${compiled.xml}</worldbody>`);
  if (compiled.assetXml) {
    if ((xml.match(/<\/asset>/g) || []).length !== 1) throw new Error('Unexpected OpenArm asset structure');
    xml = xml.replace('</asset>', `${compiled.assetXml}</asset>`);
  }
  return { ...compiled, xml };
}
