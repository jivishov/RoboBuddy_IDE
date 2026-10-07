// Curated dry rigid-body laboratory assets. Dimensions/materials are illustrative,
// not image measurements or calibrated glass. Hollow surfaces use separate convex
// wall sectors: a single mesh hull would incorrectly close an instrument's bore.
const asset = (dimensions_m, mass_kg, dynamic, description) => Object.freeze({
  dimensions_m: Object.freeze(dimensions_m), mass_kg, dynamic, description,
  evidence: 'illustrative geometry and estimated rigid-body parameters; no liquids or hardware validation',
});
export const OPENARM_LAB_ASSETS = Object.freeze({
  funnel: asset([.060, .060, .085], .025, true, 'Open conical bowl and hollow 10 mm outside-diameter stem; local origin at stem tip.'),
  burette: asset([.045, .030, .400], .12, false, 'Fixed/clamped burette with an open bore and closed stopcock; place a ring stand at its clamp height. Fixed mounting is an explicit boundary condition.'),
  ring_stand: asset([.160, .140, .600], 1.5, false, 'Fixed base, vertical rod and connected split clamp; clamp is at 72% of the specified height.'),
  beaker: asset([.075, .075, .100], .08, true, 'Open cylindrical vessel with a solid bottom and separate wall sectors.'),
  erlenmeyer_flask: asset([.080, .080, .125], .09, true, 'Open-neck conical flask with a closed bottom; no liquid process.'),
  bottle: asset([.050, .050, .115], .10, true, 'Capped reagent bottle; contents and labels are not analytical measurements.'),
  tile: asset([.130, .130, .008], .25, false, 'White support tile.'),
});
export const LAB_ASSET_KINDS = Object.freeze(Object.keys(OPENARM_LAB_ASSETS));
const SEGMENTS = 16;
const GLASS = [.62, .80, .88, .55], METAL = [.30, .34, .37, 1];
const IDENTITY = [1, 0, 0, 0];
const join = v => v.join(' ');
const quat = e => e.quaternion_wxyz || [Math.cos(e.yaw_rad / 2), 0, 0, Math.sin(e.yaw_rad / 2)];

export function validateLabAsset(e) {
  const spec = OPENARM_LAB_ASSETS[e.kind];
  if (!spec) throw new TypeError('Unknown laboratory asset');
  // Keep wall thicknesses, bores, and instrument proportions meaningful. The
  // bounded catalog is not arbitrary mesh/XML upload or unrestricted scaling.
  e.dimensions_m.forEach((v, i) => {
    const scale = v / spec.dimensions_m[i];
    if (scale < .6 || scale > 1.5) throw new RangeError(`${e.kind} dimensions must be 0.6..1.5 times the catalog dimensions`);
  });
  if (['funnel', 'beaker', 'erlenmeyer_flask', 'bottle'].includes(e.kind) && Math.abs(e.dimensions_m[0] - e.dimensions_m[1]) > 1e-9) throw new RangeError(`${e.kind} requires equal X/Y diameters`);
  if (['burette', 'ring_stand', 'tile'].includes(e.kind) && e.dynamic) throw new RangeError(`${e.kind} is a fixed fixture in this catalog`);
}

// Eight-vertex convex annular-sector frustum. Every face has outward winding.
// A complete ring consists of individually colliding sectors, leaving the centre open.
function sector(z0, z1, ri0, ro0, ri1, ro1, a0, a1) {
  const vertices = [];
  for (const [z, ri, ro] of [[z0, ri0, ro0], [z1, ri1, ro1]]) {
    for (const [r, a] of [[ri, a0], [ro, a0], [ro, a1], [ri, a1]]) vertices.push(r * Math.cos(a), r * Math.sin(a), z);
  }
  const faces = [[0,3,2,1], [4,5,6,7], [0,1,5,4], [1,2,6,5], [2,3,7,6], [3,0,4,7]];
  const indices = faces.flatMap(([a,b,c,d]) => [a,b,c,a,c,d]);
  // Polygon-sector area, integrated along the linearly changing radii.
  const meanSquared = (a,b) => (a*a + a*b + b*b)/3;
  const volume = .5 * Math.sin(a1-a0) * (z1-z0) * (meanSquared(ro0,ro1) - meanSquared(ri0,ri1));
  return { vertices, indices, volume };
}

export function compileLabAsset(e) {
  validateLabAsset(e);
  const id = `lab_${e.id}`, [w,d,h] = e.dimensions_m;
  const geoms = [], meshes = {}, masses = [];
  const add = (type, positionM, sizeM, suffix, rgba = GLASS, quaternionWxyz = IDENTITY, volume = null, mesh = null) => {
    geoms.push({ id: `${id}_${suffix}`, bodyId: id, type, positionM, sizeM, quaternionWxyz, rgba, ...(mesh ? { mesh } : {}) });
    masses.push(volume ?? (type === 'box' ? 8*sizeM[0]*sizeM[1]*sizeM[2] : Math.PI*sizeM[0]**2*2*sizeM[1]));
  };
  const ring = (suffix, z0, z1, ri0, ro0, ri1 = ri0, ro1 = ro0, rgba = GLASS) => {
    for (let i=0; i<SEGMENTS; i++) {
      const mesh = `${id}_${suffix}_${i}_mesh`;
      const part = sector(z0, z1, ri0, ro0, ri1, ro1, i*2*Math.PI/SEGMENTS, (i+1)*2*Math.PI/SEGMENTS);
      meshes[mesh] = { vertices: part.vertices, indices: part.indices };
      add('mesh', [0,0,0], [], `${suffix}_${i}`, rgba, IDENTITY, part.volume, mesh);
    }
  };
  let affordances = { graspReferenceM: [0,0,h/2], topReferenceM: [0,0,h] };
  if (e.kind === 'funnel') {
    const r = w/2, stemR = w/12, wall = w/40, stemH = h*.45;
    ring('stem', 0, stemH, stemR-wall, stemR);
    ring('bowl', stemH, h, stemR-wall, stemR, r-wall, r);
    affordances = { ...affordances, graspReferenceM: [0,0,h*.80], stemTipM: [0,0,0], axisM: [0,0,1],
      stemRadiusM: stemR, stemLengthM: stemH, bowlRadiusM: r, heightM: h, hollow: true };
  } else if (e.kind === 'burette') {
    const r = Math.min(w*.20, d*.30), ri = r*.76, z = h*.14;
    ring('tube', z, h, ri, r);
    add('cylinder', [0,0,z], [r,.0015], 'closed_bottom');
    add('cylinder', [0,0,h*.04], [r*.18,h*.04], 'tip');
    add('box', [0,0,h*.11], [w*.25,d*.27,h*.030], 'stopcock_body', [.9,.9,.86,1]);
    add('cylinder', [0,0,h*.11], [d*.18,w/2], 'stopcock_handle', [.10,.25,.70,1], [Math.SQRT1_2,0,Math.SQRT1_2,0]);
    affordances = { openingCenterM: [0,0,h], topReferenceM: [0,0,h], axisM: [0,0,1],
      innerRadiusM: ri*Math.cos(Math.PI/SEGMENTS), outerRadiusM: r,
      boreDepthM: h-z, clampReferenceM: [0,0,h*.65], hollow: true,
      mounting: 'fixed clamp boundary condition; not a free-standing stability simulation' };
  } else if (e.kind === 'ring_stand') {
    const rodX = -w*.34, rodR = .005, jaw = .003, gap = .010;
    add('box', [0,0,.006], [w/2,d/2,.006], 'base', METAL);
    add('cylinder', [rodX,0,h/2], [rodR,h/2], 'rod', METAL);
    // Crosspiece and separated jaws surround a 20 mm clear opening. Nothing
    // silently fills the bore or becomes a grasp attachment during execution.
    add('box', [rodX/2-.008,0,h*.72], [(-rodX-.016)/2,.005,.004], 'bracket', METAL);
    for (const sign of [-1,1]) add('box', [-.006,sign*(gap+jaw/2),h*.72], [.018,jaw/2,.005], `jaw_${sign>0?'p':'n'}`, METAL);
    add('box', [-.023,0,h*.72], [.003,gap+jaw,.005], 'clamp_back', METAL);
    affordances = { clampReferenceM: [0,0,h*.72], clampOpeningWidthM: .020, topReferenceM: [0,0,h], mounting: 'fixed laboratory stand' };
  } else if (e.kind === 'beaker') {
    const r = w/2, wall = w/35;
    add('cylinder', [0,0,wall/2], [r,wall/2], 'base');
    ring('wall', wall, h, r-wall, r);
    affordances = { ...affordances, openingCenterM: [0,0,h], innerRadiusM: (r-wall)*Math.cos(Math.PI/SEGMENTS), hollow: true };
  } else if (e.kind === 'erlenmeyer_flask') {
    const r = w/2, wall = w/40, neck = r*.32;
    add('cylinder', [0,0,wall/2], [r,wall/2], 'base');
    ring('body', wall, h*.75, r-wall, r, neck-wall, neck);
    ring('neck', h*.75, h, neck-wall, neck);
    affordances = { ...affordances, graspReferenceM: [0,0,h*.88], openingCenterM: [0,0,h], innerRadiusM: (neck-wall)*Math.cos(Math.PI/SEGMENTS), hollow: true };
  } else if (e.kind === 'bottle') {
    add('cylinder', [0,0,h*.37], [w/2,h*.37], 'body', [.35,.17,.04,1]);
    add('cylinder', [0,0,h*.83], [w*.27,h*.09], 'neck', [.35,.17,.04,1]);
    add('cylinder', [0,0,h*.96], [w*.29,h*.04], 'cap', [.11,.12,.13,1]);
  } else if (e.kind === 'tile') add('box', [0,0,h/2], [w/2,d/2,h/2], 'solid', [.94,.94,.90,1]);
  const totalVolume = masses.reduce((a,b)=>a+b,0);
  const assetXml = Object.entries(meshes).map(([name,m]) => `<mesh name="${name}" vertex="${join(m.vertices)}" face="${join(m.indices)}"/>`).join('');
  const geomXml = geoms.map((g,i) => `<geom name="${g.id}" type="${g.type}" ${g.mesh ? `mesh="${g.mesh}"` : `size="${join(g.sizeM)}"`} pos="${join(g.positionM)}" quat="${join(g.quaternionWxyz)}" mass="${e.mass_kg*masses[i]/totalVolume}" contype="3" conaffinity="3" friction="0.8 .005 .0005" solref=".005 1" rgba="${join(g.rgba)}"/>`).join('');
  const xml = `<body name="${id}" pos="${join(e.position_m)}" quat="${join(quat(e))}">${e.dynamic ? `<freejoint name="${id}_free"/>` : ''}${geomXml}</body>`;
  return { xml, assetXml, meshes, geoms, bodies: [{ id, ...(e.dynamic ? { freeJointId: `${id}_free` } : {}) }],
    records: [{ ...e, bodyId: id, geometryIds: geoms.map(g=>g.id), affordances, processModel: 'dry rigid-body catalog geometry; dimensions and friction are estimates; no fluid, heat, chemistry or glass deformation' }] };
}
