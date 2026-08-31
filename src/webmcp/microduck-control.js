import { MICRODUCK_COMMANDS } from '../microduck/command-catalog.js';

export const MICRODUCK_WEBMCP_COMMANDS = Object.freeze(Object.keys(MICRODUCK_COMMANDS));

const durationSchema = Object.freeze({
  type: 'integer',
  minimum: MICRODUCK_COMMANDS.move.authority.webmcp.minimumDurationMs,
  maximum: MICRODUCK_COMMANDS.move.authority.webmcp.maximumDurationMs,
});

const numberSchema = (minimum, maximum) => ({ type: 'number', minimum, maximum });
const commandProperty = (command) => ({ type: 'string', const: command });
const strictBranch = (command, properties = {}, required = [], suffix = '') => ({
  title: `${command}${suffix}`,
  type: 'object',
  properties: { command: commandProperty(command), ...properties },
  required: ['command', ...required],
  additionalProperties: false,
});

function movementRange(field) {
  const ranges = Object.values(MICRODUCK_COMMANDS.move.ui.fieldsByMode).map((mode) => mode[field].range);
  return [Math.min(...ranges.map((range) => range[0])), Math.max(...ranges.map((range) => range[1]))];
}

function fieldSchemas(fields) {
  return Object.fromEntries(Object.entries(fields).map(([name, item]) => [name, numberSchema(item.range[0], item.range[1])]));
}

function schemaBranches() {
  const move = MICRODUCK_COMMANDS.move;
  const nonHeldSounds = MICRODUCK_COMMANDS.sound.values.filter((tag) => tag !== 'wheee');
  const branches = [
    strictBranch('move', {
      vx: numberSchema(...movementRange('vx')),
      vy: numberSchema(...movementRange('vy')),
      yaw: numberSchema(...movementRange('yaw')),
      duration_ms: durationSchema,
    }, ['duration_ms']),
    strictBranch('head', { ...fieldSchemas(MICRODUCK_COMMANDS.head.ui.fields), duration_ms: durationSchema }, ['duration_ms']),
    strictBranch('look', {
      ...fieldSchemas(MICRODUCK_COMMANDS.look.ui.fields),
      neckPitch: numberSchema(...MICRODUCK_COMMANDS.head.ui.fields.neckPitch.range),
    }, ['x', 'y', 'z']),
    strictBranch('stop'),
    strictBranch('enable', { enabled: { type: 'boolean' } }),
    strictBranch('init'),
    strictBranch('relax'),
    strictBranch('do', { skill: { type: 'string', enum: MICRODUCK_COMMANDS.do.values } }, ['skill']),
    strictBranch('pose', { ...fieldSchemas(MICRODUCK_COMMANDS.pose.ui.fields), duration_ms: durationSchema }, ['duration_ms']),
    strictBranch('mouth', { open: numberSchema(...MICRODUCK_COMMANDS.mouth.ui.fields.open.range), duration_ms: durationSchema }, ['open', 'duration_ms']),
    strictBranch('sound', { tag: { type: 'string', enum: nonHeldSounds }, hold: { type: 'boolean', const: false } }, ['tag'], ' non-held'),
    strictBranch('sound', { tag: { type: 'string', const: 'wheee' }, hold: { type: 'boolean', const: false } }, ['tag', 'hold'], ' wheee release'),
    strictBranch('sound', { tag: { type: 'string', const: 'wheee' }, hold: { type: 'boolean', const: true }, duration_ms: durationSchema }, ['tag', 'hold', 'duration_ms'], ' held wheee'),
    strictBranch('theremin', { active: { type: 'boolean', const: false } }, ['active'], ' inactive'),
    strictBranch('theremin', { active: { type: 'boolean', const: true }, duration_ms: durationSchema }, ['active', 'duration_ms'], ' active'),
    strictBranch('chorale', { active: { type: 'boolean', const: false } }, ['active'], ' inactive'),
    strictBranch('chorale', {
      active: { type: 'boolean', const: true },
      piece: { type: 'string', enum: MICRODUCK_COMMANDS.chorale.ui.pieces },
      voices: { type: 'integer', minimum: MICRODUCK_COMMANDS.chorale.ui.voices[0], maximum: MICRODUCK_COMMANDS.chorale.ui.voices[1] },
      duration_ms: durationSchema,
    }, ['active', 'duration_ms'], ' active'),
    strictBranch('get_mode'),
    strictBranch('set_mode', { mode: { type: 'string', enum: MICRODUCK_COMMANDS.set_mode.modes } }, ['mode']),
    strictBranch('get_state'),
    strictBranch('set_color', { color: { type: 'string', enum: MICRODUCK_COMMANDS.set_color.values } }, ['color']),
    strictBranch('spawn_ball', {
      position: { type: 'array', items: numberSchema(-2, 2), minItems: 3, maxItems: 3 },
    }),
    strictBranch('reset'),
    strictBranch('set_tof_stimulus', {
      distanceM: numberSchema(...MICRODUCK_COMMANDS.set_tof_stimulus.ui.fields.distanceM.range),
      source: { type: 'string', enum: MICRODUCK_COMMANDS.set_tof_stimulus.ui.sources },
    }, ['distanceM']),
    strictBranch('set_camera', { camera: { type: 'string', enum: MICRODUCK_COMMANDS.set_camera.values } }, ['camera']),
  ];
  const represented = new Set(branches.map((branch) => branch.properties.command.const));
  if (represented.size !== MICRODUCK_WEBMCP_COMMANDS.length || MICRODUCK_WEBMCP_COMMANDS.some((command) => !represented.has(command))) {
    throw new Error('The MicroDuck WebMCP schema is not in parity with MICRODUCK_COMMANDS.');
  }
  return branches;
}

export function createMicroDuckControlSchema() {
  return Object.freeze({ oneOf: Object.freeze(schemaBranches().map((branch) => Object.freeze(branch))) });
}

function matchesPrimitive(value, schema) {
  if (schema.const !== undefined && value !== schema.const) return false;
  if (schema.enum && !schema.enum.includes(value)) return false;
  if (schema.type === 'boolean') return typeof value === 'boolean';
  if (schema.type === 'string') return typeof value === 'string';
  if (schema.type === 'integer' && !Number.isInteger(value)) return false;
  if (schema.type === 'number' && (typeof value !== 'number' || !Number.isFinite(value))) return false;
  if (schema.type === 'integer' || schema.type === 'number') return value >= schema.minimum && value <= schema.maximum;
  if (schema.type === 'array') {
    return Array.isArray(value)
      && value.length >= schema.minItems
      && value.length <= schema.maxItems
      && value.every((item) => matchesPrimitive(item, schema.items));
  }
  return true;
}

function matchesBranch(input, branch) {
  if (!input || typeof input !== 'object' || Array.isArray(input)) return false;
  const keys = Object.keys(input);
  if (keys.some((key) => !Object.hasOwn(branch.properties, key))) return false;
  if (branch.required.some((key) => !Object.hasOwn(input, key))) return false;
  return keys.every((key) => matchesPrimitive(input[key], branch.properties[key]));
}

export function parseMicroDuckControlInput(input) {
  const matches = createMicroDuckControlSchema().oneOf.filter((branch) => matchesBranch(input, branch));
  if (matches.length !== 1) {
    const error = new Error('Input must match exactly one strict MicroDuck command branch. Check command fields and conditional duration_ms (20..5000).');
    error.code = 'INVALID_ARGUMENT';
    throw error;
  }
  const { command, duration_ms: durationMs, ...args } = input;
  return Object.freeze({ command, args: Object.freeze({ ...args }), durationMs });
}

function boundedClone(value, depth = 0) {
  if (value === null || typeof value === 'boolean') return value;
  if (typeof value === 'number') return Number.isFinite(value) ? value : null;
  if (typeof value === 'string') return value.slice(0, 180);
  if (depth >= 7) return '[bounded]';
  if (Array.isArray(value)) return value.slice(0, 64).map((item) => boundedClone(item, depth + 1));
  if (!value || typeof value !== 'object') return null;
  return Object.fromEntries(Object.entries(value).slice(0, 40).map(([key, item]) => [key, boundedClone(item, depth + 1)]));
}

export function boundedMicroDuckResult(command, result = {}) {
  const requested = result.requested ?? {};
  const applied = result.applied ?? (command === 'get_mode' ? { mode: result.mode } : {});
  return {
    ok: true,
    command,
    requested: boundedClone(requested),
    applied: boundedClone(applied),
    limitedBy: Array.from(result.limitedBy || [], String).slice(0, 8),
    completed: Boolean(result.completed ?? true),
    state: boundedClone(result.state || null),
    audio: boundedClone(result.audio || result.state?.audio || null),
  };
}

export function isRetainedMicroDuckCommand(command, args) {
  const definition = MICRODUCK_COMMANDS[command];
  return definition.classification === 'continuous'
    ? !((command === 'theremin' || command === 'chorale') && args.active === false)
    : command === 'sound' && args.tag === 'wheee' && args.hold === true;
}
