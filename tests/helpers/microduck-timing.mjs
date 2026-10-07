import { MODE_TUNING, POLICY_TIMESTEP_SECONDS, POLICY_TUNING } from '../../src/microduck/contract.js';

// Test-only observation: call the real audio engine and retain its instantaneous
// published state. An 80 ms peck can finish before a Playwright click returns.
export function observeSoundStarts(simulator) {
  const engine = simulator.audioEngine;
  const original = engine.playSound;
  const probe = { sequence: 0, last: null };
  engine.playSound = function (...args) {
    const result = original.apply(this, args);
    if (!result.hold) {
      probe.last = {
        sequence: ++probe.sequence,
        tag: result.tag,
        audio: { ...simulator.getState().audio },
        scheduledVoices: this.nodes.size,
      };
    }
    return result;
  };
  probe.restore = () => { engine.playSound = original; };
  return probe;
}

function skillDuration(phase, mode) {
  if (phase === 'ground_pick' || phase === 'roller_crouch') return MODE_TUNING[mode].groundPickPeriodSeconds * POLICY_TUNING.groundPickEndPhase;
  if (phase === 'kick_left' || phase === 'kick_right') return POLICY_TUNING.kickDurationSeconds;
  if (phase === 'roulade') return POLICY_TUNING.rouladeDurationSeconds;
  if (phase === 'rise') return POLICY_TUNING.sitRiseDurationSeconds;
  throw new Error(`No finite skill duration for ${phase}`);
}

const remainingTime = (skill, duration) => 'elapsed' in skill ? duration - skill.elapsed : skill.remaining;
const EPSILON = 1e-8;

// The caller must first observe the real UI/gamepad-triggered phase. Complete it
// via the existing controlStep path, including actual physics and awaited ONNX
// inference, rather than assuming that 5 wall seconds contain 2.8 policy seconds.
// Never assign a phase, call director.advance(), reset the robot, or extend leases.
export async function finishSkillAtControlRate(simulator, expectedPhase) {
  const wasPaused = simulator.paused;
  simulator.pause();
  try {
    // Let the invalidated pre-pause inference settle before issuing new work.
    await simulator.lastInferencePromise;
    const initial = simulator.getState();
    const duration = skillDuration(expectedPhase, initial.mode);
    const fail = (message) => {
      throw new Error(`${expectedPhase}: ${message}; state=${JSON.stringify(simulator.getState())}`);
    };
    if (initial.lifecycle !== 'ready' || !initial.enabled) fail('simulator is not enabled and ready');
    // Natural completion between the caller's phase assertion and this task is OK.
    if (initial.phase === 'up' && !simulator.director.skill) {
      if (initial.safety.resetFallback) fail('recovery reset is not skill completion');
      return { phase: expectedPhase, alreadyCompleted: true, steps: 0 };
    }
    if (initial.phase !== expectedPhase || simulator.director.skill?.name !== expectedPhase) fail('unexpected starting phase');
    const remaining = remainingTime(simulator.director.skill, duration);
    if (!Number.isFinite(remaining) || remaining <= 0 || remaining > duration + EPSILON) fail('invalid remaining policy time');
    const dt = POLICY_TIMESTEP_SECONDS;
    const maxSteps = Math.ceil(remaining / dt) + 1; // One step for floating-point boundary rounding.
    for (let steps = 1; steps <= maxSteps; steps += 1) {
      await simulator.advanceTime(dt, { controlStep: true });
      const current = simulator.getState();
      const advanced = steps * dt;
      if (current.lifecycle !== 'ready' || !current.enabled) fail('simulator stopped or failed');
      if (current.safety.resetFallback !== initial.safety.resetFallback) fail('recovery reset is not skill completion');
      if (Math.abs(current.time - initial.time - advanced) > EPSILON) fail('physics clock did not advance by the control-step budget');
      if (current.phase === 'up' && !simulator.director.skill) {
        if (advanced + EPSILON < remaining) fail('skill completed before its policy-time budget');
        return { phase: expectedPhase, remainingPolicySeconds: remaining, advancedSimulationSeconds: advanced, steps, finalPhase: current.phase };
      }
      if (current.phase !== expectedPhase || simulator.director.skill?.name !== expectedPhase) fail('unexpected phase transition');
      const nextRemaining = remainingTime(simulator.director.skill, duration);
      if (!Number.isFinite(nextRemaining) || Math.abs(nextRemaining - (remaining - advanced)) > EPSILON) fail('policy clock stalled or advanced incorrectly');
    }
    fail('skill did not complete within its finite control-step budget');
  } finally {
    if (!wasPaused) simulator.resume();
  }
}
