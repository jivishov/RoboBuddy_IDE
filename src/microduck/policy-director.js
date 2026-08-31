import {
  HOME_POLICY_POSITION, MODE_TUNING, POLICY_JOINT_ORDER, POLICY_TUNING, clamp, deepFreeze,
} from './contract.js';

const HEAD_POLICY_INDICES = Object.freeze([5, 6, 7, 8]);

export class MicroDuckPolicyDirector {
  constructor({ mode = 'walking', jointRanges = {} } = {}) {
    this.jointRanges = jointRanges;
    this.reset(mode);
  }

  reset(mode = this.mode || 'walking') {
    this.mode = mode === 'roller' ? 'roller' : 'walking';
    this.lastAction = new Float32Array(14);
    this.previousTargets = Float32Array.from(HOME_POLICY_POSITION);
    this.skill = null;
    this.sit = 'up';
    this.phase = 'idle';
  }

  trigger(skill) {
    if (skill === 'ground_pick') {
      if (this.skill?.name === 'ground_pick') throw busy('Ground-pick is already running.');
      this.skill = { name: this.mode === 'roller' ? 'roller_crouch' : 'ground_pick', elapsed: 0 };
    } else if (skill === 'kick_left' || skill === 'kick_right') {
      if (this.skill) throw busy('A scripted move is already running.');
      this.skill = { name: skill, remaining: POLICY_TUNING.kickDurationSeconds };
    } else if (skill === 'roulade') {
      if (this.skill?.name === 'ground_pick' || this.skill?.name === 'roller_crouch') throw busy('A ground-pick/crouch is running.');
      this.skill = { name: 'roulade', remaining: POLICY_TUNING.rouladeDurationSeconds };
    } else if (skill === 'sit_toggle') {
      if (this.sit === 'rising') throw busy('MicroDuck is already standing up.');
      this.sit = this.sit === 'sitting' ? 'rising' : 'sitting';
      if (this.sit === 'rising') this.skill = { name: 'rise', remaining: POLICY_TUNING.sitRiseDurationSeconds };
    } else {
      const error = new Error(`Unknown MicroDuck skill: ${skill}`); error.code = 'INVALID_ARGUMENT'; throw error;
    }
    return this.currentPolicy({ twist: [0, 0, 0], bodyActive: false });
  }

  currentPolicy({ twist = [0, 0, 0], bodyActive = false } = {}) {
    if (this.skill?.name === 'roulade') return 'roulade';
    if (this.skill?.name === 'kick_left' || this.skill?.name === 'kick_right') return this.skill.name;
    if (this.skill?.name === 'ground_pick' || this.skill?.name === 'roller_crouch') return this.skill.name;
    if (this.sit === 'sitting' || this.sit === 'rising') return 'sitstand';
    if (this.mode === 'roller') return 'roller';
    const magnitude = Math.hypot(...twist);
    return bodyActive || magnitude < POLICY_TUNING.standingThreshold ? 'stand' : 'walking';
  }

  effectiveCommand(command = {}) {
    const base = {
      twist: Array.from(command.twist || [0, 0, 0], Number),
      head: Array.from(command.head || [0, 0, 0, 0], Number),
      body: { z: Number(command.body?.z || 0), roll: Number(command.body?.roll || 0), pitch: Number(command.body?.pitch || 0) },
    };
    const policy = this.currentPolicy({ twist: base.twist, bodyActive: command.bodyActive });
    if (policy === 'roulade' || policy.startsWith('kick_')) return { policy, command: { twist: [0, 0, 0], head: [0, 0, 0, 0], body: {} } };
    if (policy === 'ground_pick' || policy === 'roller_crouch') {
      const period = MODE_TUNING[this.mode].groundPickPeriodSeconds;
      const angle = Math.PI * 2 * (this.skill.elapsed / period);
      return { policy, command: { twist: [Math.cos(angle), Math.sin(angle), 0], head: [0, 0, 0, 0], body: {} } };
    }
    if (this.sit === 'sitting') base.twist = [1, 0, 0];
    if (this.sit === 'rising' || command.bodyActive) base.twist = [0, 0, 0];
    return { policy, command: base };
  }

  actionScale(policy) {
    if (policy === 'ground_pick' || policy === 'roller_crouch') return MODE_TUNING[this.mode].groundPickActionScale;
    if (policy === 'sitstand' || policy === 'roulade') return 1;
    if (policy === 'stand' || policy.startsWith('kick_')) return POLICY_TUNING.standingActionScale;
    return MODE_TUNING[this.mode].actionScale;
  }

  applyAction(action, policy) {
    if (!action || action.length !== 14) throw new TypeError('Policy action must contain fourteen values.');
    const scale = this.actionScale(policy);
    const raw = Float32Array.from(action);
    const targets = raw.map((value, index) => clamp(HOME_POLICY_POSITION[index] + scale * value, this.jointRanges[POLICY_JOINT_ORDER[index]] || [-Math.PI, Math.PI]));
    for (let index = 0; index < targets.length; index += 1) {
      const isHead = HEAD_POLICY_INDICES.includes(index);
      const alpha = isHead ? POLICY_TUNING.headLowPassAlpha : POLICY_TUNING.legsLowPassAlpha;
      const filtered = alpha * targets[index] + (1 - alpha) * this.previousTargets[index];
      const maxStep = isHead ? POLICY_TUNING.maxTargetStepRad.head : POLICY_TUNING.maxTargetStepRad.legs;
      targets[index] = clamp(filtered, [this.previousTargets[index] - maxStep, this.previousTargets[index] + maxStep]);
    }
    this.lastAction = raw;
    this.previousTargets = Float32Array.from(targets);
    return Float32Array.from(targets);
  }

  advance(dt) {
    if (!this.skill) return;
    if (this.skill.name === 'ground_pick' || this.skill.name === 'roller_crouch') {
      this.skill.elapsed += dt;
      const cutoff = MODE_TUNING[this.mode].groundPickPeriodSeconds * POLICY_TUNING.groundPickEndPhase;
      if (this.skill.elapsed >= cutoff) this.skill = null;
      return;
    }
    this.skill.remaining -= dt;
    if (this.skill.remaining <= 0) {
      if (this.skill.name === 'rise') this.sit = 'up';
      this.skill = null;
    }
  }

  snapshot() {
    return deepFreeze({ mode: this.mode, activePolicy: this.currentPolicy(), skill: this.skill ? { ...this.skill } : null, sit: this.sit, lastAction: Array.from(this.lastAction) });
  }
}

function busy(message) { const error = new Error(message); error.code = 'SIMULATION_BUSY'; return error; }
