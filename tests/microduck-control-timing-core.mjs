import assert from 'node:assert/strict';
import test from 'node:test';
import { MicroDuckPolicyDirector } from '../src/microduck/policy-director.js';
import { POLICY_TIMESTEP_SECONDS } from '../src/microduck/contract.js';
import { observeSoundStarts, finishSkillAtControlRate } from './helpers/microduck-timing.mjs';

// Only the transport/physics adapter is mocked here; the policy director and its
// timing contract are the real modules. Chromium covers actual ONNX and physics.
function simulatorFixture(phase, { mode = 'walking', fault = null, paused = false } = {}) {
  const director = new MicroDuckPolicyDirector({ mode });
  if (phase === 'rise') { director.trigger('sit_toggle'); director.trigger('sit_toggle'); }
  else director.trigger(phase === 'roller_crouch' ? 'ground_pick' : phase);
  return {
    director, paused, time: 0, enabled: true, lifecycle: 'ready', resetFallback: false, resumes: 0, calls: 0,
    pause() { this.paused = true; },
    resume() { this.paused = false; this.resumes += 1; },
    getState() { return { time: this.time, enabled: this.enabled, lifecycle: this.lifecycle, mode, phase: director.skill?.name || director.sit, safety: { resetFallback: this.resetFallback } }; },
    async advanceTime(dt, options) {
      assert.equal(this.paused, true);
      assert.equal(dt, POLICY_TIMESTEP_SECONDS);
      assert.deepEqual(options, { controlStep: true });
      this.calls += 1;
      this.time += fault === 'clock' ? dt * 2 : dt;
      await new Promise(setImmediate); // Require awaiting the control result.
      if (fault === 'stall') return;
      if (fault === 'early') { director.skill = null; return; }
      if (fault === 'reset') { this.resetFallback = true; director.reset(mode); return; }
      if (fault === 'error') { this.lifecycle = 'error'; return; }
      director.advance(dt);
    },
  };
}

for (const [phase, mode, seconds] of [
  ['ground_pick', 'walking', 2.8], ['roller_crouch', 'roller', 2.1],
  ['kick_left', 'walking', 0.5], ['kick_right', 'walking', 0.5],
  ['roulade', 'walking', 1], ['rise', 'walking', 1],
]) {
  test(`${phase} completes within the unchanged ${seconds} s policy contract`, async () => {
    const sim = simulatorFixture(phase, { mode });
    const result = await finishSkillAtControlRate(sim, phase);
    assert.equal(result.finalPhase, 'up');
    assert.ok(Math.abs(result.advancedSimulationSeconds - seconds) < 1e-8);
    assert.equal(sim.resumes, 1);
    assert.equal(sim.paused, false);
  });
}

test('partly completed skills use only their remaining policy time', async () => {
  const sim = simulatorFixture('ground_pick');
  sim.director.advance(1);
  const result = await finishSkillAtControlRate(sim, 'ground_pick');
  assert.ok(Math.abs(result.advancedSimulationSeconds - 1.8) < 1e-8);
});

for (const [fault, message] of [
  ['stall', /policy clock stalled/], ['early', /before its policy-time budget/],
  ['clock', /physics clock/], ['reset', /recovery reset/], ['error', /stopped or failed/],
]) {
  test(`timing helper rejects ${fault} rather than manufacturing completion`, async () => {
    const sim = simulatorFixture('ground_pick', { fault });
    await assert.rejects(finishSkillAtControlRate(sim, 'ground_pick'), message);
    assert.equal(sim.calls, 1);
    assert.equal(sim.resumes, 1);
    assert.equal(sim.paused, false);
  });
}

test('timing helper preserves an already paused simulator', async () => {
  const sim = simulatorFixture('kick_left', { paused: true });
  await finishSkillAtControlRate(sim, 'kick_left');
  assert.equal(sim.paused, true);
  assert.equal(sim.resumes, 0);
});

test('timing helper rejects a different starting skill', async () => {
  const sim = simulatorFixture('kick_left');
  await assert.rejects(finishSkillAtControlRate(sim, 'ground_pick'), /unexpected starting phase/);
  assert.equal(sim.calls, 0);
  assert.equal(sim.resumes, 1);
});

test('sound observer retains a transient snapshot without extending or replaying audio', () => {
  let now = 0;
  let calls = 0;
  const engine = {
    unlocked: false, sound: null, until: 0, nodes: new Set(),
    playSound(tag, { hold = false } = {}) {
      calls += 1;
      if (!this.unlocked) throw new Error('AUDIO_LOCKED');
      this.sound = tag; this.until = now + 80; this.nodes.add({ tag });
      return { tag, hold, generated: true };
    },
  };
  const sim = { audioEngine: engine, getState: () => ({ audio: { unlocked: engine.unlocked, sound: now < engine.until ? engine.sound : null } }) };
  const original = engine.playSound;
  const probe = observeSoundStarts(sim);
  assert.throws(() => engine.playSound('peck'), /AUDIO_LOCKED/);
  assert.equal(probe.sequence, 0);
  engine.unlocked = true;
  const result = engine.playSound('peck');
  assert.deepEqual(result, { tag: 'peck', hold: false, generated: true });
  now = 1000; // Simulate the browser driver returning after the sound has ended.
  assert.equal(sim.getState().audio.sound, null);
  assert.deepEqual(probe.last, { sequence: 1, tag: 'peck', audio: { unlocked: true, sound: 'peck' }, scheduledVoices: 1 });
  assert.equal(engine.until, 80);
  assert.equal(calls, 2);
  engine.playSound('wheee', { hold: true });
  assert.equal(probe.sequence, 1);
  probe.restore();
  assert.equal(engine.playSound, original);
});
