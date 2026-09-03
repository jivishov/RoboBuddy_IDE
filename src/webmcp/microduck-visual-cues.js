import { MICRODUCK_VISUAL_CUE_LIMITS } from '../microduck/visual-cue-contract.js';

const MAX_CUES = MICRODUCK_VISUAL_CUE_LIMITS.maxCues;
const MAX_ID_LENGTH = 32;
const MAX_TEXT_LENGTH = 120;
const MAX_RULER_TITLE_LENGTH = 80;
const METRIC_NAMES = Object.freeze(['covered_m', 'remaining_to_east_edge_m']);
const ID_PATTERN = /^[A-Za-z][A-Za-z0-9_-]*$/;
const COLOR_PATTERN = /^#[0-9a-fA-F]{6}$/;

const POINT_SCHEMA = Object.freeze({
  type: 'array',
  minItems: 3,
  maxItems: 3,
  items: { type: 'number' },
});

const COLOR_SCHEMA = Object.freeze({ type: 'string', pattern: '^#[0-9a-fA-F]{6}$' });
const ID_SCHEMA = Object.freeze({ type: 'string', minLength: 1, maxLength: MAX_ID_LENGTH, pattern: '^[A-Za-z][A-Za-z0-9_-]*$' });

function schema(kind, properties, required) {
  return Object.freeze({
    type: 'object',
    properties: { id: ID_SCHEMA, kind: { const: kind }, visible: { type: 'boolean' }, color: COLOR_SCHEMA, ...properties },
    required: ['id', 'kind', ...required],
    additionalProperties: false,
  });
}

const CUE_SCHEMAS = Object.freeze([
  schema('label', {
    text: { type: 'string', minLength: 1, maxLength: MAX_TEXT_LENGTH },
    anchor: { type: 'string', enum: ['duck', 'ball', 'world'] },
    position: POINT_SCHEMA,
    offset_m: POINT_SCHEMA,
    metrics: { type: 'array', minItems: 1, maxItems: 2, uniqueItems: true, items: { type: 'string', enum: METRIC_NAMES } },
  }, ['text', 'anchor']),
  schema('marker', {
    anchor: { type: 'string', enum: ['duck', 'ball', 'world'] },
    position: POINT_SCHEMA,
    offset_m: POINT_SCHEMA,
    size_m: { type: 'number', minimum: 0.02, maximum: 0.4 },
  }, ['anchor']),
  schema('line', { start: POINT_SCHEMA, end: POINT_SCHEMA }, ['start', 'end']),
  schema('ruler', {
    start: POINT_SCHEMA,
    end: POINT_SCHEMA,
    title: { type: 'string', minLength: 1, maxLength: MAX_RULER_TITLE_LENGTH },
    major_step_m: { type: 'number', minimum: 0.1, maximum: 2 },
    minor_step_m: { type: 'number', minimum: 0.05, maximum: 1 },
  }, ['start', 'end']),
]);

function invalid(message) {
  const error = new Error(message);
  error.code = 'INVALID_ARGUMENT';
  throw error;
}

function object(value, label) {
  if (!value || typeof value !== 'object' || Array.isArray(value)) invalid(`${label} must be an object.`);
  return value;
}

function onlyKeys(value, allowed) {
  for (const key of Object.keys(value)) if (!allowed.includes(key)) invalid(`Unexpected input field: ${key}.`);
}

function requiredString(value, label, maximum) {
  if (typeof value !== 'string' || !value.trim()) invalid(`${label} must be a non-empty string.`);
  if (value.length > maximum || value.includes('\u0000')) invalid(`${label} is invalid.`);
  return value;
}

function point(value, label) {
  if (!Array.isArray(value) || value.length !== 3) invalid(`${label} must contain exactly three coordinates.`);
  const parsed = value.map(Number);
  if (!parsed.every(Number.isFinite) || Math.abs(parsed[0]) > 4.25 || Math.abs(parsed[1]) > 4.25 || parsed[2] < -0.2 || parsed[2] > 2.4) {
    invalid(`${label} must be a finite world-metre point inside the bounded workcell.`);
  }
  return Object.freeze(parsed);
}

function offset(value, label, fallback) {
  if (value === undefined) return Object.freeze(fallback);
  if (!Array.isArray(value) || value.length !== 3) invalid(`${label} must contain exactly three coordinates.`);
  const parsed = value.map(Number);
  if (!parsed.every(Number.isFinite) || parsed.some((coordinate) => Math.abs(coordinate) > 1)) invalid(`${label} must stay within one metre of its anchor.`);
  return Object.freeze(parsed);
}

function color(value) {
  if (value === undefined) return '#5ed6bc';
  if (typeof value !== 'string' || !COLOR_PATTERN.test(value)) invalid('color must be a six-digit hex color.');
  return value.toLowerCase();
}

function commonCue(value, kind, allowed) {
  object(value, 'cue');
  onlyKeys(value, allowed);
  const id = requiredString(value.id, 'cue.id', MAX_ID_LENGTH);
  if (!ID_PATTERN.test(id)) invalid('cue.id must start with a letter and use only letters, digits, underscores, or hyphens.');
  if (value.kind !== kind) invalid(`cue.kind must be ${kind}.`);
  if (value.visible !== undefined && typeof value.visible !== 'boolean') invalid('cue.visible must be a boolean.');
  return { id, kind, visible: value.visible !== false, color: color(value.color) };
}

function anchoredCue(value, kind) {
  const cue = commonCue(value, kind, ['id', 'kind', 'visible', 'color', 'anchor', 'position', 'offset_m', ...(kind === 'label' ? ['text', 'metrics'] : ['size_m'])]);
  if (!['duck', 'ball', 'world'].includes(value.anchor)) invalid('cue.anchor must be duck, ball, or world.');
  if (value.anchor === 'world' && value.position === undefined) invalid('A world-anchored cue requires position.');
  if (value.anchor !== 'world' && value.position !== undefined) invalid('Only a world-anchored cue may define position.');
  cue.anchor = value.anchor;
  cue.position = value.position === undefined ? null : point(value.position, 'cue.position');
  cue.offsetM = offset(value.offset_m, 'cue.offset_m', kind === 'label' ? [0, 0, 0.17] : [0, 0, 0]);
  if (kind === 'label') {
    cue.text = requiredString(value.text, 'cue.text', MAX_TEXT_LENGTH);
    if (value.metrics !== undefined) {
      if (!Array.isArray(value.metrics) || !value.metrics.length || value.metrics.length > 2 || new Set(value.metrics).size !== value.metrics.length || !value.metrics.every((metric) => METRIC_NAMES.includes(metric))) invalid('cue.metrics must contain one or two supported metric names.');
      if (cue.anchor === 'world') invalid('cue.metrics requires a duck or ball anchor.');
      cue.metrics = Object.freeze([...value.metrics]);
    } else cue.metrics = Object.freeze([]);
  }
  else {
    if (value.size_m !== undefined && (!Number.isFinite(Number(value.size_m)) || Number(value.size_m) < 0.02 || Number(value.size_m) > 0.4)) invalid('cue.size_m must be between 0.02 and 0.4 metres.');
    cue.sizeM = value.size_m === undefined ? 0.07 : Number(value.size_m);
  }
  return Object.freeze(cue);
}

function segmentCue(value, kind) {
  const allowed = kind === 'ruler' ? ['id', 'kind', 'visible', 'color', 'start', 'end', 'title', 'major_step_m', 'minor_step_m'] : ['id', 'kind', 'visible', 'color', 'start', 'end'];
  const cue = commonCue(value, kind, allowed);
  cue.start = point(value.start, 'cue.start');
  cue.end = point(value.end, 'cue.end');
  if (cue.start.every((coordinate, index) => coordinate === cue.end[index])) invalid('cue.start and cue.end must differ.');
  if (kind === 'ruler') {
    if (value.title !== undefined) cue.title = requiredString(value.title, 'cue.title', MAX_RULER_TITLE_LENGTH);
    const major = value.major_step_m === undefined ? 1 : Number(value.major_step_m);
    const minor = value.minor_step_m === undefined ? 0.25 : Number(value.minor_step_m);
    if (!Number.isFinite(major) || major < 0.1 || major > 2 || !Number.isFinite(minor) || minor < 0.05 || minor > 1 || minor > major) invalid('Ruler tick steps must be finite, positive, and have minor_step_m no larger than major_step_m.');
    cue.majorStepM = major;
    cue.minorStepM = minor;
  }
  return Object.freeze(cue);
}

export function createMicroduckVisualCueSchema() {
  return {
    oneOf: [
      { type: 'object', properties: { operation: { const: 'upsert' }, cue: { oneOf: CUE_SCHEMAS } }, required: ['operation', 'cue'], additionalProperties: false },
      { type: 'object', properties: { operation: { const: 'remove' }, id: ID_SCHEMA }, required: ['operation', 'id'], additionalProperties: false },
      { type: 'object', properties: { operation: { const: 'clear' } }, required: ['operation'], additionalProperties: false },
      { type: 'object', properties: { operation: { const: 'list' } }, required: ['operation'], additionalProperties: false },
    ],
  };
}

export function parseMicroduckVisualCueInput(input) {
  object(input, 'Tool input');
  if (input.operation === 'upsert') {
    onlyKeys(input, ['operation', 'cue']);
    if (!Object.hasOwn(input, 'cue')) invalid('upsert requires cue.');
    const kind = input.cue?.kind;
    if (kind === 'label' || kind === 'marker') return Object.freeze({ operation: 'upsert', cue: anchoredCue(input.cue, kind) });
    if (kind === 'line' || kind === 'ruler') return Object.freeze({ operation: 'upsert', cue: segmentCue(input.cue, kind) });
    invalid('cue.kind must be label, marker, line, or ruler.');
  }
  if (input.operation === 'remove') {
    onlyKeys(input, ['operation', 'id']);
    const id = requiredString(input.id, 'id', MAX_ID_LENGTH);
    if (!ID_PATTERN.test(id)) invalid('id is invalid.');
    return Object.freeze({ operation: 'remove', id });
  }
  if (input.operation === 'clear' || input.operation === 'list') {
    onlyKeys(input, ['operation']);
    return Object.freeze({ operation: input.operation });
  }
  invalid('operation must be upsert, remove, clear, or list.');
}

export { MICRODUCK_VISUAL_CUE_LIMITS };
