import { MICRODUCK_COMMANDS } from './command-catalog.js';

export const KEY_LAYOUTS = Object.freeze({
  wasd: Object.freeze({ forward: 'w', backward: 's', left: 'a', right: 'd', kickLeft: 'q', kickRight: 'e' }),
  zqsd: Object.freeze({ forward: 'z', backward: 's', left: 'q', right: 'd', kickLeft: 'j', kickRight: 'k' }),
});

export const PINNED_GAMEPAD_MAPPING = Object.freeze({
  source: 'pollen-robotics/microduck@590b986 padd mapping',
  axes: Object.freeze({ leftX: 0, leftY: 1, rightX: 2, rightY: 3 }),
  buttons: Object.freeze({ a: 0, b: 1, x: 2, y: 3, lb: 4, rb: 5, lt: 6, rt: 7, start: 9, dpadUp: 12, dpadDown: 13 }),
  excluded: Object.freeze(['select-held shutdown']),
});

const value = (gamepad, name) => Number(gamepad?.buttons?.[PINNED_GAMEPAD_MAPPING.buttons[name]]?.value || 0);
const inputSpec = MICRODUCK_COMMANDS.move.ui.input;
export const gamepadButtonActive = (gamepad, name) => ['lt', 'rt'].includes(name)
  ? value(gamepad, name) >= MICRODUCK_COMMANDS.sound.ui.input.triggerThreshold
  : Boolean(gamepad?.buttons?.[PINNED_GAMEPAD_MAPPING.buttons[name]]?.pressed);
export const gamepadMouth = (gamepad) => Math.max(value(gamepad, 'rt'), value(gamepad, 'lt'));
const axis = (gamepad, name) => {
  const raw = Number(gamepad?.axes?.[PINNED_GAMEPAD_MAPPING.axes[name]] || 0);
  return Math.abs(raw) < inputSpec.gamepadDeadzone ? 0 : raw;
};
const scaleRange = (input, range) => input === 0 ? 0 : input > 0 ? input * range[1] : input * Math.abs(range[0]);

export function keyboardIntent(keys, layoutName, mode) {
  const layout = KEY_LAYOUTS[layoutName] || KEY_LAYOUTS.wasd;
  const limits = MICRODUCK_COMMANDS.move.ui.fieldsByMode[mode];
  const forward = (keys.has(layout.forward) ? 1 : 0) - (keys.has(layout.backward) ? 1 : 0);
  const strafe = (keys.has(layout.left) ? 1 : 0) - (keys.has(layout.right) ? 1 : 0);
  const yaw = (keys.has('arrowleft') ? 1 : 0) - (keys.has('arrowright') ? 1 : 0);
  return { vx: forward * limits.vx.range[1], vy: strafe * Math.abs(limits.vy.range[0]), yaw: yaw * limits.yaw.range[1] };
}

export function gamepadIntent(gamepad, layer, mode) {
  const move = MICRODUCK_COMMANDS.move.ui.fieldsByMode[mode];
  if (layer === 'head') {
    const fields = MICRODUCK_COMMANDS.head.ui.fields;
    return { command: 'head', args: { headYaw: scaleRange(-axis(gamepad, 'leftX'), fields.headYaw.range), headPitch: scaleRange(axis(gamepad, 'leftY'), fields.headPitch.range), neckPitch: scaleRange(-axis(gamepad, 'rightY'), fields.neckPitch.range), headRoll: scaleRange(axis(gamepad, 'rightX'), fields.headRoll.range) } };
  }
  if (layer === 'body') {
    const fields = MICRODUCK_COMMANDS.pose.ui.fields;
    return { command: 'pose', args: { z: scaleRange(-axis(gamepad, 'leftY'), fields.z.range), pitch: scaleRange(-axis(gamepad, 'rightY'), fields.pitch.range), roll: scaleRange(axis(gamepad, 'rightX'), fields.roll.range) } };
  }
  return { command: 'move', args: { vx: scaleRange(-axis(gamepad, 'leftY'), move.vx.range), vy: scaleRange(-axis(gamepad, 'leftX'), move.vy.range), yaw: scaleRange(-axis(gamepad, 'rightX'), move.yaw.range) } };
}

export class MicroDuckInputController {
  constructor({ canvas, captureRoot = canvas, execute, releaseIntent = () => {}, getState, onCapture, onError = () => {}, now = () => performance.now(), navigatorObject = navigator } = {}) {
    this.canvas = canvas;
    this.captureRoot = captureRoot;
    this.execute = execute;
    this.releaseIntent = releaseIntent;
    this.getState = getState;
    this.onCapture = onCapture;
    this.onError = onError;
    this.now = now;
    this.navigator = navigatorObject;
    this.active = false;
    this.captured = false;
    this.layout = 'wasd';
    this.layer = 'drive';
    this.keys = new Set();
    this.previousButtons = new Set();
    this.modeHoldStarted = null;
    this.lastRoulade = -Infinity;
    this.lastRefresh = -Infinity;
    this.lastMouthRefresh = -Infinity;
    this.frame = 0;
    this.boundKeyDown = (event) => this.onKeyDown(event);
    this.boundKeyUp = (event) => this.onKeyUp(event);
    this.boundBlur = () => this.release('focus loss');
    this.boundFocusOut = (event) => { if (!this.captureRoot.contains(event.relatedTarget)) this.release('focus left simulator workbench'); };
    this.boundVisibility = () => { if (document.hidden) this.release('page hidden'); };
    this.boundDisconnect = () => this.release('gamepad disconnected');
    window.addEventListener('keydown', this.boundKeyDown);
    window.addEventListener('keyup', this.boundKeyUp);
    window.addEventListener('blur', this.boundBlur);
    window.addEventListener('gamepaddisconnected', this.boundDisconnect);
    document.addEventListener('visibilitychange', this.boundVisibility);
    this.captureRoot.addEventListener('focusout', this.boundFocusOut);
    this.poll();
  }

  setActive(active) { this.active = Boolean(active); if (!this.active) this.release('profile switch'); }
  setLayout(layout) { if (KEY_LAYOUTS[layout]) this.layout = layout; }
  capture() { if (!this.active) return false; this.captured = true; this.lastRefresh = -Infinity; this.canvas.focus(); this.canvas.dataset.microduckCapture = 'true'; this.captureRoot.dataset.microduckCapture = 'true'; this.onCapture?.(true); return true; }
  release(reason = 'capture released') { if (!this.captured && this.keys.size === 0) return false; this.captured = false; this.keys.clear(); this.previousButtons.clear(); this.modeHoldStarted = null; this.canvas.dataset.microduckCapture = 'false'; this.captureRoot.dataset.microduckCapture = 'false'; this.onCapture?.(false, reason); this.releaseIntent(); return true; }

  onKeyDown(event) {
    if (!this.active || !this.captured || event.ctrlKey || event.metaKey || event.altKey) return;
    const key = event.key.toLowerCase();
    const layout = KEY_LAYOUTS[this.layout];
    const handled = new Set([...Object.values(layout), 'arrowleft', 'arrowright', 'g', 'y', 'r', 'm', 'c', ' ']);
    if (!handled.has(key)) return;
    if (key.startsWith('arrow') && event.target?.matches?.('input[type="range"], input[type="number"], select')) return;
    event.preventDefault();
    this.keys.add(key);
    if (event.repeat) return;
    if (key === layout.kickLeft) void this.safeExecute('do', { skill: 'kick_left' });
    else if (key === layout.kickRight) void this.safeExecute('do', { skill: 'kick_right' });
    else if (key === 'g') void this.safeExecute('do', { skill: 'ground_pick' });
    else if (key === 'y') void this.safeExecute('do', { skill: 'sit_toggle' });
    else if (key === 'r') void this.safeExecute('do', { skill: 'roulade' });
    else if (key === 'm') void this.safeExecute('set_mode', { mode: this.getState().mode === 'walking' ? 'roller' : 'walking' }).finally(() => this.release('mode switch'));
    else if (key === 'c') { const cameras = MICRODUCK_COMMANDS.set_camera.values; const current = this.getState().virtualCamera.mode; void this.safeExecute('set_camera', { value: cameras[(cameras.indexOf(current) + 1) % cameras.length] }); }
    else if (key === ' ') void this.safeExecute('reset', {}).finally(() => this.release('reset'));
  }

  onKeyUp(event) {
    const key = event.key.toLowerCase();
    this.keys.delete(key);
    const layout = KEY_LAYOUTS[this.layout];
    if (this.active && this.captured && [...Object.values(layout).slice(0, 4), 'arrowleft', 'arrowright'].includes(key)) {
      void this.safeExecute('move', keyboardIntent(this.keys, this.layout, this.getState().mode), inputSpec.captureLeaseMs);
    }
  }

  poll() {
    this.frame = requestAnimationFrame(() => this.poll());
    if (!this.active || !this.captured) return;
    const at = this.now();
    const state = this.getState();
    if (this.keys.size && at - this.lastRefresh >= inputSpec.refreshMs) { this.lastRefresh = at; void this.safeExecute('move', keyboardIntent(this.keys, this.layout, state.mode), inputSpec.captureLeaseMs); }
    const gamepad = Array.from(this.navigator.getGamepads?.() || []).find(Boolean);
    if (!gamepad) return;
    const currentButtons = new Set(Object.keys(PINNED_GAMEPAD_MAPPING.buttons).filter((name) => gamepadButtonActive(gamepad, name)));
    const edge = (name) => currentButtons.has(name) && !this.previousButtons.has(name);
    if (edge('start')) void this.safeExecute('enable', { toggle: true });
    if (edge('y')) this.switchLayer(this.layer === 'head' ? 'drive' : 'head');
    if (edge('b')) this.switchLayer(this.layer === 'body' ? 'drive' : 'body');
    if (edge('a')) void this.safeExecute('do', { skill: 'ground_pick' });
    if (edge('lb')) void this.safeExecute('do', { skill: 'kick_left' });
    if (edge('rb')) void this.safeExecute('do', { skill: 'kick_right' });
    if (edge('dpadDown')) void this.safeExecute('do', { skill: 'sit_toggle' });
    if (currentButtons.has('x') && at - this.lastRoulade >= MICRODUCK_COMMANDS.do.ui.input.rouladeChainRefreshMs) { this.lastRoulade = at; void this.safeExecute('do', { skill: 'roulade' }); }
    if (edge('rt')) void this.safeExecute('sound', { tag: 'chirp' });
    if (currentButtons.has('lt')) void this.safeExecute('sound', { tag: 'wheee', hold: true }, inputSpec.captureLeaseMs);
    if (!currentButtons.has('lt') && this.previousButtons.has('lt')) void this.safeExecute('sound', { tag: 'wheee', hold: false });
    if (currentButtons.has('dpadUp')) {
      this.modeHoldStarted ??= at;
      if (at - this.modeHoldStarted >= MICRODUCK_COMMANDS.set_mode.ui.input.gamepadHoldMs) { this.modeHoldStarted = Infinity; void this.safeExecute('set_mode', { mode: state.mode === 'walking' ? 'roller' : 'walking' }).finally(() => this.release('mode switch')); }
    } else this.modeHoldStarted = null;
    const intent = gamepadIntent(gamepad, this.layer, state.mode);
    if (at - this.lastRefresh >= inputSpec.refreshMs) { this.lastRefresh = at; void this.safeExecute(intent.command, intent.args, inputSpec.captureLeaseMs); }
    const mouth = gamepadMouth(gamepad);
    const triggerReleased = mouth === 0 && (this.previousButtons.has('rt') || this.previousButtons.has('lt'));
    if (triggerReleased || at - this.lastMouthRefresh >= inputSpec.refreshMs) {
      this.lastMouthRefresh = at;
      void this.safeExecute('mouth', { open: mouth }, inputSpec.captureLeaseMs);
    }
    this.previousButtons = currentButtons;
  }

  switchLayer(layer) {
    if (this.layer === layer) return;
    this.releaseIntent();
    this.layer = layer;
    this.lastRefresh = -Infinity;
  }

  async safeExecute(command, args, durationMs) {
    try { return await this.execute(command, args, { source: 'human', controllerId: 'simulator-capture', durationMs }); }
    catch (error) { this.onError(error); return null; }
  }

  dispose() {
    this.release('disposed');
    cancelAnimationFrame(this.frame);
    window.removeEventListener('keydown', this.boundKeyDown);
    window.removeEventListener('keyup', this.boundKeyUp);
    window.removeEventListener('blur', this.boundBlur);
    window.removeEventListener('gamepaddisconnected', this.boundDisconnect);
    document.removeEventListener('visibilitychange', this.boundVisibility);
    this.captureRoot.removeEventListener('focusout', this.boundFocusOut);
  }
}
