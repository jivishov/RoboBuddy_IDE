import { BODY_LIMITS, HEAD_LIMITS, MODE_TUNING, clamp, deepFreeze } from './contract.js';
import { solveLookAt } from './look-ik.js';

const authority = deepFreeze({
  human: { priority: 3, defaultDurationMs: 250, minimumDurationMs: 1, maximumDurationMs: 250, durationRequired: false },
  python: { priority: 2, defaultDurationMs: 5000, minimumDurationMs: 1, maximumDurationMs: 5000, durationRequired: false },
  webmcp: { priority: 1, defaultDurationMs: null, minimumDurationMs: 20, maximumDurationMs: 5000, durationRequired: true },
});
const sources = Object.freeze(Object.keys(authority));
const shared = { authorities: sources, authority };
const field = (label, unit, range, step) => ({ label, unit, range, step });
const continuous = (neutral, extra = {}) => ({ ...shared, classification: 'continuous', completion: 'lease expiry or explicit cancellation', timeoutMs: null, cancellable: true, safeAbort: 'apply command neutral', neutral, ...extra });
const oneShot = (timeoutMs, extra = {}) => ({ ...shared, classification: 'one_shot', completion: 'operation completion', timeoutMs, cancellable: true, safeAbort: 'cancel operation safely', neutral: null, expiry: 'safe abort on cancellation or timeout', ...extra });
const immediate = (extra = {}) => ({ ...shared, classification: 'immediate', completion: 'immediate', timeoutMs: 0, cancellable: false, safeAbort: 'no retained operation', neutral: null, expiry: 'none', ...extra });

export const MICRODUCK_COMMANDS = deepFreeze({
  move: { ...continuous({ vx: 0, vy: 0, yaw: 0 }), modes: ['walking', 'roller'], expiry: 'zero movement', ui: { label: 'Drive', input: { captureLeaseMs: authority.human.defaultDurationMs, refreshMs: 80, gamepadDeadzone: 0.1 }, fieldsByMode: Object.fromEntries(Object.entries(MODE_TUNING).map(([mode, item]) => [mode, { vx: field('Forward', 'm/s', item.movement.vx, 0.01), vy: field('Strafe', 'm/s', item.movement.vy, 0.01), yaw: field('Yaw rate', 'rad/s', item.movement.yaw, 0.05) }])) } },
  head: { ...continuous({ neckPitch: 0, headPitch: 0, headYaw: 0, headRoll: 0 }), modes: ['walking', 'roller'], expiry: 'neutral head', ui: { label: 'Head', fields: Object.fromEntries(Object.entries(HEAD_LIMITS).map(([name, range]) => [name, field(name.replace(/[A-Z]/g, (match) => ` ${match.toLowerCase()}`), 'rad', range, 0.01)])) } },
  look: { ...immediate(), modes: ['walking', 'roller'], completion: 'solved persistent head intent applied', writesIntent: 'head', ui: { label: 'Look target', fields: { x: field('X', 'm', [0.05, 1], 0.01), y: field('Y', 'm', [-0.5, 0.5], 0.01), z: field('Z', 'm', [-0.25, 0.6], 0.01) } } },
  stop: { ...immediate(), modes: ['walking', 'roller'], completion: 'movement neutral while policy and actuation remain unchanged', writesIntent: 'move' },
  enable: { ...immediate(), modes: ['walking', 'roller'] },
  init: { ...oneShot(3000), modes: ['walking', 'roller'], safeAbort: 'hold current modeled pose' },
  relax: { ...immediate(), modes: ['walking', 'roller'], completion: 'disable browser actuation under modeled gravity' },
  do: { ...oneShot(8000), modes: ['walking', 'roller'], values: ['ground_pick', 'kick_left', 'kick_right', 'sit_toggle', 'roulade'], safeAbort: 'return to mode main policy', ui: { label: 'Skills', input: { rouladeChainRefreshMs: 950 } } },
  pose: { ...continuous({ z: 0, roll: 0, pitch: 0 }), modes: ['walking', 'roller'], expiry: 'inactive neutral body pose', ui: { label: 'Body pose', fields: Object.fromEntries(Object.entries(BODY_LIMITS).map(([name, range]) => [name, field(name === 'z' ? 'Height' : name, name === 'z' ? 'm' : 'rad', range, name === 'z' ? 0.001 : 0.01)])) } },
  mouth: { ...continuous({ open: 0 }), modes: ['walking', 'roller'], expiry: 'closed mouth', ui: { label: 'Mouth', fields: { open: field('Open', 'ratio', [0, 1], 0.01) } } },
  sound: { ...oneShot(5000), modes: ['walking', 'roller'], values: ['alarm', 'greet', 'inquire', 'peck', 'chirp', 'coo', 'wheee'], safeAbort: 'release held wheee', heldLease: true, heldNeutral: { tag: null, hold: false }, ui: { input: { triggerThreshold: 0.3 } } },
  theremin: { ...continuous({ active: false }), modes: ['walking', 'roller'], expiry: 'disabled theremin', ui: { playableRangeM: [0.08, 0.8], dropoutHoldMs: 180 } },
  chorale: { ...continuous({ active: false }), modes: ['walking', 'roller'], expiry: 'disabled chorale', ui: { pieces: ['wistful', 'duck_strut'], voices: [1, 4] } },
  get_mode: { ...immediate(), modes: ['walking', 'roller'] },
  set_mode: { ...oneShot(5000), modes: ['walking', 'roller'], safeAbort: 'retain previous mode', ui: { input: { gamepadHoldMs: 3000 } } },
  get_state: { ...immediate(), modes: ['walking', 'roller'] },
  set_color: { ...immediate(), modes: ['walking', 'roller'], values: ['cream', 'graphite', 'lavender', 'sky'] },
  spawn_ball: { ...immediate(), modes: ['walking', 'roller'] },
  reset: { ...immediate(), modes: ['walking', 'roller'] },
  set_tof_stimulus: { ...immediate(), modes: ['walking', 'roller'], ui: { label: 'Modeled hand distance', fields: { distanceM: field('Distance', 'm', [0.05, 2], 0.01) }, sources: ['synthetic', 'raycast'] } },
  set_camera: {
    ...immediate(),
    modes: ['walking', 'roller'],
    values: ['orbit', 'chase', 'head'],
    ui: {
      label: 'Main camera view',
      shortcut: 'C cycles Overview → Follow → Head POV',
      options: {
        orbit: {
          label: 'Overview',
          frame: 'world',
          overlay: 'OVERVIEW · ORBIT ROBOT + BALL',
          purpose: 'Inspect the articulated robot and ball together; drag the main view to orbit after fitting.',
          fitLabel: 'Refit Overview',
        },
        chase: {
          label: 'Follow',
          frame: 'robot_root',
          overlay: 'FOLLOW · STABLE THIRD-PERSON TRACKING',
          purpose: 'Track behind and above the robot with aspect-aware framing while it moves.',
          fitLabel: 'Refit Follow',
        },
        head: {
          label: 'Head POV',
          frame: 'head_camera',
          overlay: 'HEAD POV · MODELED RENDERED VIEW · NO HARDWARE VIDEO',
          purpose: 'Render the main viewport from the modeled source head_camera frame; this is simulation imagery, not hardware video.',
          fitLabel: 'Align Head POV',
        },
      },
    },
  },
});

export function validateCommand(name, input = {}, mode = 'walking') {
  const definition = MICRODUCK_COMMANDS[name];
  if (!definition) throw commandError('INVALID_ARGUMENT', `Unknown MicroDuck command: ${name}`);
  if (!definition.modes.includes(mode)) throw commandError('INVALID_ARGUMENT', `${name} is unavailable in ${mode} mode.`);
  const value = { ...input };
  const limitedBy = [];
  if (name === 'move') {
    const limits = MODE_TUNING[mode].movement;
    for (const [field, range] of Object.entries(limits)) {
      const requested = Number(input[field] || 0);
      value[field] = clamp(requested, range);
      if (value[field] !== requested) limitedBy.push(mode === 'roller' && field === 'vy' ? 'roller_no_strafe' : `${field}_limit`);
    }
  } else if (name === 'head') {
    for (const [field, range] of Object.entries(HEAD_LIMITS)) {
      const requested = Number(input[field] || 0);
      value[field] = clamp(requested, range);
      if (value[field] !== requested) limitedBy.push(`${field}_limit`);
    }
  } else if (name === 'look') {
    const target = [Number(input.x), Number(input.y), Number(input.z)];
    if (!target.every(Number.isFinite)) throw commandError('INVALID_ARGUMENT', 'look requires finite trunk-frame x, y, and z coordinates in metres.');
    value.x = target[0]; value.y = target[1]; value.z = target[2];
    const neckPitch = input.neckPitch ?? input.neck_pitch ?? 0;
    const gaze = solveLookAt(target, neckPitch);
    value.neckPitch = gaze.joints.neckPitch;
    value.solvedHead = gaze.joints;
    value.clamped = gaze.clamped;
    if (gaze.clamped) limitedBy.push('look_ik_clamped');
  } else if (name === 'pose') {
    for (const [field, range] of Object.entries(BODY_LIMITS)) {
      const requested = Number(input[field] || 0);
      value[field] = clamp(requested, range);
      if (value[field] !== requested) limitedBy.push(`${field}_limit`);
    }
    for (const field of ['x', 'y', 'yaw']) if (Number(input[field] || 0) !== 0) limitedBy.push(`${field}_untrained`);
    value.x = 0; value.y = 0; value.yaw = 0;
  } else if (name === 'mouth') {
    const requested = Number(input.open || 0);
    value.open = clamp(requested, [0, 1]);
    if (value.open !== requested) limitedBy.push('mouth_limit');
  } else if (name === 'enable') {
    value.toggle = Boolean(input.toggle);
    value.enabled = input.on === undefined ? (input.enabled === undefined ? true : Boolean(input.enabled)) : Boolean(input.on);
  } else if (name === 'do') {
    if (!definition.values.includes(input.skill)) throw commandError('INVALID_ARGUMENT', `Unknown MicroDuck skill: ${input.skill}`);
  } else if (name === 'set_mode') {
    if (!['walking', 'roller'].includes(input.mode)) throw commandError('INVALID_ARGUMENT', 'Mode must be walking or roller.');
  } else if (name === 'sound') {
    if (!definition.values.includes(input.tag)) throw commandError('INVALID_ARGUMENT', `Unknown MicroDuck sound: ${input.tag}`);
    value.tag = input.tag;
    value.hold = Boolean(input.hold);
  } else if (name === 'set_color') {
    const color = input.color ?? input.value;
    if (!definition.values.includes(color)) throw commandError('INVALID_ARGUMENT', `Unknown MicroDuck color: ${color}`);
    value.value = color;
  } else if (name === 'set_camera') {
    const camera = input.camera ?? input.value;
    if (!definition.values.includes(camera)) throw commandError('INVALID_ARGUMENT', `Unknown MicroDuck camera: ${camera}`);
    value.value = camera;
  } else if (name === 'set_tof_stimulus') {
    const range = definition.ui.fields.distanceM.range;
    const requested = Number(input.distanceM ?? input.distance_m ?? input.value);
    if (!Number.isFinite(requested)) throw commandError('INVALID_ARGUMENT', 'set_tof_stimulus requires a finite distanceM value.');
    value.distanceM = clamp(requested, range);
    value.source = definition.ui.sources.includes(input.source) ? input.source : 'synthetic';
    if (value.distanceM !== requested) limitedBy.push('tof_distance_limit');
  } else if (name === 'theremin') {
    value.active = Boolean(input.active);
  } else if (name === 'chorale') {
    value.active = Boolean(input.active);
    if (input.piece !== undefined && !definition.ui.pieces.includes(input.piece)) throw commandError('INVALID_ARGUMENT', `Unknown release chorale piece: ${input.piece}`);
    value.piece = input.piece || definition.ui.pieces[0];
    const requested = Number(input.voices ?? 1);
    value.voices = Math.round(clamp(requested, definition.ui.voices));
    if (value.voices !== requested) limitedBy.push('chorale_voice_limit');
  } else if (name === 'spawn_ball') {
    const position = input.position ?? [0.28, 0, 0.035];
    if (!Array.isArray(position) || position.length !== 3 || !position.every((item) => Number.isFinite(Number(item)))) throw commandError('INVALID_ARGUMENT', 'spawn_ball position must contain three finite metre values.');
    value.position = position.map(Number);
  } else if (definition.values && input.value !== undefined && !definition.values.includes(input.value)) {
    throw commandError('INVALID_ARGUMENT', `Invalid value for ${name}.`);
  }
  return deepFreeze({ requested: { ...input }, applied: value, limitedBy });
}

export function commandError(code, message) {
  const error = new Error(message);
  error.code = code;
  return error;
}
