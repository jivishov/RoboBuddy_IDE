import { MICRODUCK_COMMANDS, commandError } from './command-catalog.js';
import { mapTheremin } from './peripherals.js';

const SOUND_PATTERNS = Object.freeze({
  alarm: [[660, 0.08], [880, 0.08], [660, 0.08]],
  greet: [[330, 0.09], [440, 0.11], [554, 0.15]],
  inquire: [[392, 0.1], [523, 0.18]],
  peck: [[180, 0.035], [120, 0.045]],
  chirp: [[740, 0.06], [988, 0.09]],
  coo: [[220, 0.16], [196, 0.2]],
  wheee: [[294, 0.3]],
});

const CHORALE_NOTES = Object.freeze({
  wistful: [220, 261.63, 329.63, 392],
  duck_strut: [196, 246.94, 293.66, 369.99],
});

export class MicroDuckAudioEngine {
  constructor({ AudioContextClass = globalThis.AudioContext || globalThis.webkitAudioContext, now = () => performance.now(), trustedEvent = (event) => event instanceof Event && event.isTrusted } = {}) {
    this.AudioContextClass = AudioContextClass;
    this.now = now;
    this.trustedEvent = trustedEvent;
    this.context = null;
    this.unlocked = false;
    this.nodes = new Set();
    this.held = new Map();
    this.thereminState = { active: false, frequencyHz: 0, mouth: 0, held: false };
    this.lastUsableMs = -Infinity;
    this.chorale = { active: false, piece: null, voices: 0 };
    this.currentSound = null;
    this.currentSoundUntil = -Infinity;
  }

  async unlock(event) {
    if (!this.trustedEvent(event)) return false;
    if (!this.AudioContextClass) throw commandError('ASSET_UNAVAILABLE', 'Web Audio is unavailable in this browser.');
    this.context ||= new this.AudioContextClass();
    await this.context.resume?.();
    this.unlocked = true;
    return true;
  }

  requireUnlocked() {
    if (!this.unlocked) throw commandError('AUDIO_LOCKED', 'Audio is locked. Use the trusted Unlock audio control first.');
  }

  playSound(tag, { hold = false } = {}) {
    this.requireUnlocked();
    if (!MICRODUCK_COMMANDS.sound.values.includes(tag)) throw commandError('INVALID_ARGUMENT', `Unknown MicroDuck sound: ${tag}`);
    if (hold && tag === 'wheee' && this.held.has(tag)) {
      this.currentSound = tag;
      this.currentSoundUntil = Infinity;
      return { tag, hold: true, generated: true };
    }
    this.releaseSound(tag);
    const pattern = SOUND_PATTERNS[tag];
    let offset = 0;
    for (const [frequency, duration] of pattern) {
      const oscillator = this.makeVoice(frequency, this.context.currentTime + offset, duration, tag === 'peck' ? 'square' : 'triangle', hold && tag === 'wheee');
      if (hold && tag === 'wheee') this.held.set(tag, oscillator);
      offset += duration;
    }
    this.currentSound = tag;
    this.currentSoundUntil = hold && tag === 'wheee' ? Infinity : this.now() + offset * 1000;
    return { tag, hold: Boolean(hold && tag === 'wheee'), generated: true };
  }

  releaseSound(tag = 'wheee') {
    const node = this.held.get(tag);
    if (!node) return false;
    this.stopNode(node);
    this.held.delete(tag);
    if (this.currentSound === tag) { this.currentSound = null; this.currentSoundUntil = -Infinity; }
    return true;
  }

  setTheremin(active, distanceM) {
    if (!active) { this.stopHeld('theremin'); this.thereminState = { active: false, frequencyHz: 0, mouth: 0, held: false }; return this.thereminState; }
    this.requireUnlocked();
    const mapped = mapTheremin(distanceM, { previous: this.thereminState, nowMs: this.now(), lastUsableMs: this.lastUsableMs });
    if (!mapped.held && mapped.active) this.lastUsableMs = this.now();
    this.thereminState = mapped;
    if (!mapped.active) this.stopHeld('theremin');
    else {
      let oscillator = this.held.get('theremin');
      if (!oscillator) { oscillator = this.makeVoice(mapped.frequencyHz, this.context.currentTime, 60, 'sine', true); this.held.set('theremin', oscillator); }
      oscillator.frequency?.setTargetAtTime?.(mapped.frequencyHz, this.context.currentTime, 0.025);
    }
    return mapped;
  }

  setChorale({ active, piece = 'wistful', voices = 1 } = {}) {
    if (!active) { this.stopHeld('chorale'); this.chorale = { active: false, piece: null, voices: 0 }; return this.chorale; }
    this.requireUnlocked();
    if (!MICRODUCK_COMMANDS.chorale.ui.pieces.includes(piece)) throw commandError('INVALID_ARGUMENT', `Unknown release chorale piece: ${piece}`);
    const count = Math.round(Math.max(1, Math.min(4, Number(voices) || 1)));
    if (this.chorale.active && this.chorale.piece === piece && this.chorale.voices === count && this.held.has('chorale')) return this.chorale;
    this.stopHeld('chorale');
    const group = [];
    for (let index = 0; index < count; index += 1) group.push(this.makeVoice(CHORALE_NOTES[piece][index], this.context.currentTime, 60, index % 2 ? 'sine' : 'triangle', true, 0.035));
    this.held.set('chorale', group);
    this.chorale = { active: true, piece, voices: count };
    return this.chorale;
  }

  makeVoice(frequency, start, duration, type, held, gainValue = 0.055) {
    const oscillator = this.context.createOscillator();
    const gain = this.context.createGain();
    oscillator.type = type;
    oscillator.frequency.setValueAtTime(frequency, start);
    gain.gain.setValueAtTime(0.0001, start);
    gain.gain.exponentialRampToValueAtTime(gainValue, start + 0.012);
    if (!held) gain.gain.exponentialRampToValueAtTime(0.0001, start + duration);
    oscillator.connect(gain).connect(this.context.destination);
    oscillator.start(start);
    if (!held) oscillator.stop(start + duration + 0.02);
    oscillator.addEventListener?.('ended', () => this.nodes.delete(oscillator), { once: true });
    this.nodes.add(oscillator);
    return oscillator;
  }

  stopHeld(key) {
    const value = this.held.get(key);
    if (!value) return false;
    for (const node of Array.isArray(value) ? value : [value]) this.stopNode(node);
    this.held.delete(key);
    return true;
  }

  stopNode(node) { try { node.stop(); } catch {} this.nodes.delete(node); }
  stopAll() { for (const node of this.nodes) this.stopNode(node); this.held.clear(); this.currentSound = null; this.currentSoundUntil = -Infinity; this.thereminState = { active: false, frequencyHz: 0, mouth: 0, held: false }; this.chorale = { active: false, piece: null, voices: 0 }; }
  snapshot() { return { unlocked: this.unlocked, sound: this.currentSoundUntil > this.now() ? this.currentSound : null, theremin: this.thereminState.active, thereminFrequencyHz: this.thereminState.frequencyHz, thereminMouth: this.thereminState.mouth, thereminHeld: this.thereminState.held, chorale: this.chorale.active, piece: this.chorale.piece, voices: this.chorale.voices, implementation: 'deterministic local Web Audio synthesis' }; }
  dispose() { this.stopAll(); void this.context?.close?.(); this.context = null; }
}
