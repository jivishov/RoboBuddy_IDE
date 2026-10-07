// Explicit setup-only selection. The verified robot/mount/table/floor remain;
// optional reference-task fixtures and vessels are removed from BOTH authorities.
export const OPENARM_SCENE_MODES = Object.freeze(['baseline', 'blank', 'authored']);
const FIXTURES = new Set(['left_source_support','left_hotplate','right_source_support','right_ring_post','right_ring_gauze','right_ring_bracket']);
const VESSELS = new Set(['flask','beaker']);
export function validateOpenArmSceneMode(mode = 'baseline') {
  if (!OPENARM_SCENE_MODES.includes(mode)) throw new TypeError('scene_mode must be baseline, blank, or authored');
  return mode;
}
export function referenceWorkcellRecord(record) { return FIXTURES.has(record.id) || VESSELS.has(record.bodyId); }
export function workcellBodies(bodies, mode) { validateOpenArmSceneMode(mode); return mode !== 'baseline' ? bodies.filter(b=>!VESSELS.has(b.id)) : bodies; }
export function workcellConstraints(constraints, mode) {
  validateOpenArmSceneMode(mode);
  return mode !== 'baseline' ? { fixtures: constraints.fixtures.filter(id=>!FIXTURES.has(id) && (mode !== 'authored' || id !== 'cell_table')), objects: [] } : constraints;
}
export function workcellGeometry(geoms, mode) { validateOpenArmSceneMode(mode); return mode !== 'baseline' ? geoms.filter(g=>!referenceWorkcellRecord(g) && (mode !== 'authored' || g.id !== 'cell_table' && !g.id.startsWith('table_leg_'))) : geoms; }
export function workcellBaseXml(xml, mode = 'baseline') {
  validateOpenArmSceneMode(mode);
  if (mode === 'baseline') return xml;
  // This parser operates only on pinned, hash-verified repository XML, never
  // caller-provided XML. Nested bodies are removed with balanced tag scanning.
  for (const name of VESSELS) {
    const start = xml.indexOf(`<body name="${name}"`);
    if (start < 0) throw new Error(`Missing baseline vessel ${name}`);
    const tags = /<\/?body\b[^>]*>/g; tags.lastIndex = start;
    let depth = 0, end = null, match;
    while ((match = tags.exec(xml))) {
      if (match[0].startsWith('</')) depth--;
      else if (!match[0].endsWith('/>')) depth++;
      if (depth === 0) { end = tags.lastIndex; break; }
    }
    if (end == null) throw new Error('Unbalanced OpenArm body XML');
    xml = xml.slice(0,start) + xml.slice(end);
  }
  xml = xml.replace(/<geom\b[^>]*\bname="([^"]+)"[^>]*\/>/g, (tag,name) => FIXTURES.has(name) ? '' : tag);
  for (const name of [...FIXTURES,...VESSELS]) if (xml.includes(`name="${name}"`)) throw new Error(`Baseline component ${name} was not removed`);
  if (mode === 'authored') xml = xml.replace(/<geom\b[^>]*\bname="(cell_table|table_leg_[^"]+)"[^>]*\/>/g, '');
  for (const name of (mode === 'authored' ? ['floor','openarm_mount','mount_column'] : ['floor','cell_table','openarm_mount','mount_column'])) if (!xml.includes(`name="${name}"`)) throw new Error(`Blank scene lost required ${name}`);
  return xml;
}
