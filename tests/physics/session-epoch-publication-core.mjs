import assert from 'node:assert/strict';
import { PhysicsSession } from '../../src/physics/session.js';
import { UNITREE_G1_FREEBASE_SCENE } from '../../src/physics/unitree-g1-scene.js';

// A consumer is allowed to reset/preempt a run on an intermediate observation. The rest of an
// already-returned batch still belongs to the old epoch and may never be relabelled as the new run.
function backendFixture(sampled) {
  let time = 0;
  let advances = 0;
  const observation = () => ({ simulationTimeSeconds: time });
  const backend = {
    async loadScene(scene) { return { sceneRevision: scene.revision, robotId: scene.robotId, observation: observation() }; },
    async reset() { time = 0; return observation(); },
    async acceptCommand() { return { observation: observation() }; },
    async advanceSteps(steps) { advances++; time += steps * 0.002; return observation(); },
    async getObservation() { return observation(); },
    async getDiagnostics() { return { loaded: true }; },
    async pause() { return observation(); }, async resume() { return observation(); },
    async cancelRun() { return {}; }, exportTrace() { return []; }, dispose() {},
    get advances() { return advances; },
  };
  if (sampled) backend.advanceStepsObserved = async (steps, { sampleEverySteps }) => {
    advances++;
    const observations = [];
    for (let n = 0; n < steps; n += sampleEverySteps) {
      time += Math.min(steps - n, sampleEverySteps) * 0.002;
      observations.push(observation());
    }
    return { executedSteps: steps, observations, finalObservation: observations.at(-1) };
  };
  return backend;
}

for (const sampled of [false, true]) {
  const backend = backendFixture(sampled);
  const session = new PhysicsSession(backend, { observationBatchSteps: 1 });
  await session.loadScene(UNITREE_G1_FREEBASE_SCENE);
  const events = [];
  let reset;
  session.subscribe((event) => {
    if (event.source !== 'advanceSteps') return;
    events.push({ epoch: event.epoch, time: event.observation.simulationTimeSeconds });
    if (events.length === 1) reset = session.reset();
  });
  await assert.rejects(session.advanceSteps(4), /stale|superseded|epoch/i, `old ${sampled ? 'sampled' : 'chunked'} advance must be revoked`);
  await reset;
  assert.equal(backend.advances, 1, 'old advance must not step the replacement run');
  assert.deepEqual(events, [{ epoch: 1, time: 0.002 }], 'old samples must not be published after reset');
  assert.equal(session.epoch, 2);
  assert.equal(session.robotId, UNITREE_G1_FREEBASE_SCENE.robotId, 'stale completion must not clear the replacement scene');
  session.dispose();
}
console.log('PhysicsSession epoch-pinned sample publication and reset preemption OK');
