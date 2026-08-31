import assert from 'node:assert/strict';
import { MICRODUCK_COMMANDS, validateCommand } from '../src/microduck/command-catalog.js';
import { MicroDuckAudioEngine } from '../src/microduck/audio-engine.js';
import { gamepadButtonActive, gamepadIntent, gamepadMouth, keyboardIntent, KEY_LAYOUTS, PINNED_GAMEPAD_MAPPING } from '../src/microduck/input-controller.js';
import { createModeledTof, deriveFrameAngularVelocity, deriveModeledImu, mapTheremin } from '../src/microduck/peripherals.js';

function testCatalogRenderingContract() {
  assert.deepEqual(MICRODUCK_COMMANDS.set_color.values, ['cream', 'graphite', 'lavender', 'sky']);
  assert.deepEqual(MICRODUCK_COMMANDS.set_camera.values, ['orbit', 'chase', 'head']);
  assert.equal(MICRODUCK_COMMANDS.set_camera.ui.shortcut, 'C cycles Overview → Follow → Head POV');
  assert.deepEqual(
    Object.fromEntries(Object.entries(MICRODUCK_COMMANDS.set_camera.ui.options).map(([mode, option]) => [mode, { label: option.label, frame: option.frame }])),
    {
      orbit: { label: 'Overview', frame: 'world' },
      chase: { label: 'Follow', frame: 'robot_root' },
      head: { label: 'Head POV', frame: 'head_camera' },
    },
  );
  assert.match(MICRODUCK_COMMANDS.set_camera.ui.options.head.purpose, /simulation imagery, not hardware video/i);
  assert.deepEqual(MICRODUCK_COMMANDS.sound.values, ['alarm', 'greet', 'inquire', 'peck', 'chirp', 'coo', 'wheee']);
  assert.deepEqual(MICRODUCK_COMMANDS.chorale.ui.pieces, ['wistful', 'duck_strut']);
  assert.deepEqual(MICRODUCK_COMMANDS.move.ui.fieldsByMode.roller.vy.range, [0, 0]);
  assert.equal(MICRODUCK_COMMANDS.move.ui.input.captureLeaseMs, 250);
  assert.equal(MICRODUCK_COMMANDS.move.ui.input.gamepadDeadzone, 0.1);
  assert.equal(MICRODUCK_COMMANDS.sound.ui.input.triggerThreshold, 0.3);
  assert.equal(validateCommand('set_tof_stimulus', { distanceM: 9 }, 'walking').applied.distanceM, MICRODUCK_COMMANDS.set_tof_stimulus.ui.fields.distanceM.range[1]);
  assert.throws(() => validateCommand('chorale', { active: true, piece: 'outer_wilds', voices: 9 }, 'walking'), { code: 'INVALID_ARGUMENT' });
  assert.equal(validateCommand('chorale', { active: true, piece: 'duck_strut', voices: 9 }, 'walking').applied.voices, 4);
}

function testKeyboardLayouts() {
  assert.equal(KEY_LAYOUTS.wasd.kickLeft, 'q');
  assert.equal(KEY_LAYOUTS.zqsd.kickLeft, 'j');
  assert.deepEqual(keyboardIntent(new Set(['w', 'a', 'arrowleft']), 'wasd', 'walking'), { vx: 0.3, vy: 0.3, yaw: 1.5 });
  assert.deepEqual(keyboardIntent(new Set(['z', 'q']), 'zqsd', 'roller'), { vx: 0.6, vy: 0, yaw: 0 });
}

function testPinnedGamepadFixture() {
  assert.deepEqual(PINNED_GAMEPAD_MAPPING.excluded, ['select-held shutdown']);
  assert.equal(Object.values(PINNED_GAMEPAD_MAPPING.buttons).includes(8), false);
  const gamepad = { axes: [0.5, -1, -0.5, 0.25], buttons: Array.from({ length: 16 }, () => ({ pressed: false, value: 0 })) };
  assert.deepEqual(gamepadIntent(gamepad, 'drive', 'walking'), { command: 'move', args: { vx: 0.3, vy: -0.15, yaw: 0.75 } });
  const head = gamepadIntent(gamepad, 'head', 'walking');
  assert.equal(head.command, 'head');
  assert.ok(head.args.headYaw < 0);
  assert.ok(head.args.headPitch < 0);
  assert.ok(head.args.neckPitch < 0);
  assert.ok(head.args.headRoll < 0);
  const body = gamepadIntent(gamepad, 'body', 'walking');
  assert.equal(body.command, 'pose');
  assert.ok(body.args.z > 0);
  assert.ok(body.args.pitch < 0);
  assert.ok(body.args.roll < 0);
  const deadzone = { ...gamepad, axes: [0.09, 0, 0, 0] };
  assert.equal(gamepadIntent(deadzone, 'drive', 'walking').args.vy, 0);
  gamepad.buttons[6].value = 0.29;
  assert.equal(gamepadButtonActive(gamepad, 'lt'), false);
  assert.equal(gamepadMouth(gamepad), 0.29);
  gamepad.buttons[6].value = 0.3;
  assert.equal(gamepadButtonActive(gamepad, 'lt'), true);
}

function testModeledSensors() {
  const first = createModeledTof({ distanceM: 0.31, sequence: 7 });
  const second = createModeledTof({ distanceM: 0.31, sequence: 7 });
  assert.deepEqual(first, second);
  assert.equal(first.valuesM.length, 64);
  assert.equal(first.frame, 'tof');
  assert.equal(first.modeled, true);
  const sampled = createModeledTof({ source: 'raycast', valuesM: Array(64).fill(0.27) });
  assert.equal(sampled.source, 'raycast');
  assert.deepEqual(sampled.valuesM, Array(64).fill(0.27));
  const halfAngle = 0.1;
  const headGyro = deriveFrameAngularVelocity([0, 0, 0, 1], [0, 0, Math.sin(halfAngle), Math.cos(halfAngle)], 0.1);
  assert.ok(Math.abs(headGyro[2] - 2) < 1e-8);
  const imu = deriveModeledImu({ trunkGyro: [0.1, 0.2, 0.3], headGyro });
  assert.equal(imu.trunk.frame, 'imu');
  assert.equal(imu.head.frame, 'head_imu');
  assert.notDeepEqual(imu.head.gyro, imu.trunk.gyro);
  const near = mapTheremin(0.1, { nowMs: 100 });
  assert.equal(near.active, true);
  assert.ok(near.frequencyHz > 500);
  assert.equal(mapTheremin(2, { previous: near, nowMs: 200, lastUsableMs: 100 }).held, true);
  assert.equal(mapTheremin(2, { previous: near, nowMs: 400, lastUsableMs: 100 }).active, false);
}

async function testAudioBoundaryAndAllowlist() {
  const context = new FakeAudioContext();
  const audio = new MicroDuckAudioEngine({ AudioContextClass: class { constructor() { return context; } }, now: () => 100, trustedEvent: (event) => event?.isTrusted === true });
  assert.throws(() => audio.playSound('chirp'), { code: 'AUDIO_LOCKED' });
  assert.equal(await audio.unlock({ isTrusted: false }), false);
  assert.equal(audio.unlocked, false);
  assert.equal(await audio.unlock({ isTrusted: true }), true);
  assert.equal(audio.playSound('chirp').generated, true);
  audio.playSound('wheee', { hold: true });
  const heldVoiceCount = context.oscillators;
  audio.playSound('wheee', { hold: true });
  assert.equal(context.oscillators, heldVoiceCount);
  assert.throws(() => audio.setChorale({ active: true, piece: 'outer_wilds', voices: 4 }), { code: 'INVALID_ARGUMENT' });
  assert.deepEqual(audio.setChorale({ active: true, piece: 'duck_strut', voices: 4 }), { active: true, piece: 'duck_strut', voices: 4 });
  const choraleVoiceCount = context.oscillators;
  audio.setChorale({ active: true, piece: 'duck_strut', voices: 4 });
  assert.equal(context.oscillators, choraleVoiceCount);
  audio.stopAll();
}

class FakeAudioParam {
  setValueAtTime() {}
  exponentialRampToValueAtTime() {}
  setTargetAtTime() {}
}
class FakeNode {
  constructor() { this.frequency = new FakeAudioParam(); this.gain = new FakeAudioParam(); }
  connect() { return this; }
  start() {}
  stop() {}
  addEventListener() {}
}
class FakeAudioContext {
  constructor() { this.currentTime = 0; this.destination = new FakeNode(); this.oscillators = 0; }
  resume() { return Promise.resolve(); }
  close() { return Promise.resolve(); }
  createOscillator() { this.oscillators += 1; return new FakeNode(); }
  createGain() { return new FakeNode(); }
}

testCatalogRenderingContract();
testKeyboardLayouts();
testPinnedGamepadFixture();
testModeledSensors();
await testAudioBoundaryAndAllowlist();
console.log('MicroDuck Cycle 03 UI/input/peripheral core checks passed.');
