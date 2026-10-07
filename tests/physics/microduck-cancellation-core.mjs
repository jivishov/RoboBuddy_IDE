import assert from 'node:assert/strict';
import { MicroDuckPhysicalSimulator } from '../../src/physics/microduck-physical-simulator.js';
import { MicroDuckController, MICRODUCK_HOME_POSITION_RAD, MICRODUCK_POLICY_JOINT_ORDER } from '../../src/physics/microduck-controller.js';
import { MICRODUCK_PHYSICAL_PACKAGES } from '../../src/physics/microduck-capabilities.js';
import { executeMicroDuckPhysicalControl, WEBMCP_MICRODUCK_PHYSICAL_SCHEMA_VERSION as VERSION } from '../../src/webmcp/microduck-physical-control.js';
import { WebMcpDomainError } from '../../src/webmcp/agent-facade.js';

// Exercise the real WebMCP dispatcher, simulator advancement/settling methods and controller.
// Only inference and the physics transport are stand-ins; these are cancellation tests,
// not claims about physical motion. Browser coverage uses real ONNX and MuJoCo separately.
function fixture(packageKey = 'walk') {
  const sim = new MicroDuckPhysicalSimulator();
  const hooks = {};
  const counts = { inference: 0, commands: 0, steps: 0, setup: 0, batches: [] };
  sim.packageKey = packageKey;
  sim.modelPackage = MICRODUCK_PHYSICAL_PACKAGES[packageKey];
  sim.scene = { id: 'cancellation-test', revision: 'test' };
  sim.controller = new MicroDuckController();
  sim.ready = true;
  sim.lastObservation = {
    simulationTimeSeconds: 0,
    sensors: { imu: { gyroRadS: [0, 0, 0], projectedGravity: [0, 0, -1] } },
    joints: Object.fromEntries(MICRODUCK_POLICY_JOINT_ORDER.map((id, slot) => [id, {
      positionRad: MICRODUCK_HOME_POSITION_RAD[slot], velocityRadS: 0,
    }])),
    bodies: {},
  };
  const observation = sim.lastObservation;
  sim.policyRuntime = { async infer() {
    counts.inference += 1;
    await hooks.infer?.(counts.inference);
    return new Array(14).fill(0.05);
  } };
  sim.session = {
    sessionId: 'cancellation-test', epoch: 0, sceneRevision: 'test', robotId: sim.modelPackage.robotId,
    async getObservation() { await hooks.observe?.(); return observation; },
    async sendCommand() { counts.commands += 1; await hooks.command?.(); return observation; },
    async advanceSteps(steps) {
      counts.batches.push(steps);
      counts.steps += steps;
      observation.simulationTimeSeconds = counts.steps * 0.005;
      await hooks.step?.();
      return observation;
    },
    async applySetup() { counts.setup += 1; await hooks.setup?.(); return observation; },
  };
  const context = { workspaceStatus: 'ready', simulationReady: true, profileId: 'microduck', simulationMode: 'physical_mujoco', workspaceGeneration: 1, simulatorEpoch: 1 };
  const facade = {
    controlSequence: 0, activeControlId: null, enabled: true,
    assertActive(epoch) {
      if (!this.enabled || epoch !== 1) throw new WebMcpDomainError('OPERATION_CANCELLED', 'Agent Access was revoked');
    },
    getRegistrationContext: () => ({ ...context }),
    app: {
      sim: { backend: sim, getPhysicalAuthorityToken: () => sim.getPhysicalAuthorityToken() },
      getExecutionState: () => 'idle', renderPanels() {}, setStatus() {},
    },
  };
  const abort = new AbortController();
  const run = (input, signal = abort.signal) => executeMicroDuckPhysicalControl(facade, { schema_version: VERSION, ...input }, signal, 1);
  return { sim, counts, hooks, context, facade, abort, run };
}
const cancelled = (error) => error.code === 'OPERATION_CANCELLED';
const requests = [
  ['advance', 'walk', { command: 'advance', advance_seconds: 2 }],
  ['set_command', 'walk', { command: 'set_command', request: { vx: 0.35 }, advance_seconds: 2 }],
  ['request_skill', 'groundContact', { command: 'request_skill', skill: 'sit', advance_seconds: 2 }],
];
let passed = 0;
async function check(name, fn) { await fn(); passed += 1; console.log(`ok ${name}`); }

for (const [name, plant, input] of requests) {
  await check(`${name}: pre-aborted calls cannot mutate or advance`, async () => {
    const f = fixture(plant);
    const before = structuredClone(f.sim.requested);
    f.abort.abort();
    await assert.rejects(f.run(input), cancelled);
    assert.equal(f.counts.inference, 0);
    assert.equal(f.counts.commands, 0);
    assert.equal(f.counts.steps, 0);
    assert.deepEqual(f.sim.requested, before);
    assert.equal(f.sim.controller.busy, false);
    assert.equal(f.facade.activeControlId, null);
  });
  await check(`${name}: abort during inference discards the pending action and permits a fresh call`, async () => {
    const f = fixture(plant);
    let controllerBefore;
    f.hooks.infer = (tick) => {
      if (tick === 2) {
        controllerBefore = structuredClone(f.sim.controller.lastStep);
        f.abort.abort();
      }
    };
    await assert.rejects(f.run(input), cancelled);
    assert.equal(f.counts.inference, 2);
    assert.equal(f.counts.commands, 1, 'cancelled inference must not send a new target frame');
    assert.equal(f.counts.steps, 4, 'the remaining 99 controller ticks must not run');
    assert.deepEqual(f.sim.controller.lastStep, controllerBefore, 'cancelled inference must not commit controller feedback');
    assert.equal(f.facade.activeControlId, null);
    delete f.hooks.infer;
    const next = await f.run({ command: 'advance', advance_seconds: 0.02 }, new AbortController().signal);
    assert.equal(next.executedTicks, 1, 'cancelling one tool call must not poison the simulator');
    assert.equal(f.counts.steps, 8);
  });
  await check(`${name}: access revocation without an aborted signal stops within the controller loop`, async () => {
    const f = fixture(plant);
    f.hooks.infer = (tick) => { if (tick === 2) f.facade.enabled = false; };
    await assert.rejects(f.run(input), cancelled);
    assert.equal(f.abort.signal.aborted, false);
    assert.equal(f.counts.commands, 1);
    assert.equal(f.counts.steps, 4);
    assert.equal(f.facade.activeControlId, null);
  });
  await check(`${name}: abort during a submitted physics batch cannot start another tick`, async () => {
    const f = fixture(plant);
    f.hooks.step = () => f.abort.abort();
    await assert.rejects(f.run(input), cancelled);
    assert.equal(f.counts.inference, 1);
    assert.equal(f.counts.commands, 1);
    assert.equal(f.counts.steps, 4, 'only the already-submitted 20 ms batch may finish');
    assert.equal(f.facade.activeControlId, null);
  });
}

await check('abort while a target frame is in flight prevents the following physics advance', async () => {
  const f = fixture();
  f.hooks.command = () => f.abort.abort();
  await assert.rejects(f.run(requests[0][2]), cancelled);
  assert.equal(f.counts.commands, 1);
  assert.equal(f.counts.steps, 0);
});
await check('abort while fetching the first observation prevents inference and command mutation', async () => {
  const f = fixture();
  f.sim.lastObservation = null;
  // The facade snapshot is valid before the observation fetch; preserve that identity.
  f.sim.getPhysicalAuthorityToken = () => ({ sessionId: 'cancellation-test', epoch: 0, sceneRevision: 'test', robotId: 'microduck' });
  f.hooks.observe = () => f.abort.abort();
  await assert.rejects(f.run(requests[0][2]), cancelled);
  assert.equal(f.counts.inference, 0);
  assert.equal(f.counts.commands, 0);
});
await check('workspace replacement during inference cannot receive the old call targets', async () => {
  const f = fixture();
  f.hooks.infer = () => { f.context.workspaceGeneration += 1; };
  await assert.rejects(f.run(requests[0][2]), cancelled);
  assert.equal(f.counts.commands, 0);
  assert.equal(f.counts.steps, 0);
});
await check('cancelling the final requested tick returns cancellation, not false success', async () => {
  const f = fixture();
  f.hooks.step = () => f.abort.abort();
  await assert.rejects(f.run({ command: 'advance', advance_seconds: 0.02 }), cancelled);
  assert.equal(f.counts.steps, 4);
});

const setup = { command: 'setup_perturbation', perturbation: 'face_down', settle_seconds: 2 };
for (const revoke of [false, true]) {
  await check(`settling: ${revoke ? 'revoked access' : 'abort'} stops after the submitted batch`, async () => {
    const f = fixture('groundContact');
    f.hooks.step = () => { if (revoke) f.facade.enabled = false; else f.abort.abort(); };
    await assert.rejects(f.run(setup), cancelled);
    assert.equal(f.counts.setup, 1);
    assert.equal(f.counts.inference, 0, 'settling must keep the home target, not run a skill');
    assert.equal(f.counts.commands, 1);
    assert.deepEqual(f.counts.batches, [4], 'settling must not submit the entire two-second budget');
    assert.equal(f.facade.activeControlId, null);
  });
}
await check('settling: abort after setup prevents any subsequent target or physics submission', async () => {
  const f = fixture('groundContact');
  f.hooks.setup = () => f.abort.abort();
  await assert.rejects(f.run(setup), cancelled);
  assert.equal(f.counts.commands, 0);
  assert.equal(f.counts.steps, 0);
});
await check('uncancelled guarded calls retain the original tick budget and zero-duration semantics', async () => {
  const f = fixture();
  const result = await f.run({ command: 'advance', advance_seconds: 0.06 });
  assert.equal(result.executedTicks, 3);
  assert.equal(result.completed, true);
  assert.equal(f.counts.commands, 3);
  assert.deepEqual(f.counts.batches, [4, 4, 4]);
  const zero = await f.run({ command: 'advance', advance_seconds: 0 });
  assert.equal(zero.executedTicks, 0);
  assert.equal(f.counts.steps, 12);
  assert.equal(f.facade.activeControlId, null);
});
await check('guarded settling preserves the exact physics-step budget, including a partial last batch', async () => {
  const f = fixture('groundContact');
  const result = await f.run({ ...setup, settle_seconds: 0.025 });
  assert.equal(result.ok, true);
  assert.equal(f.counts.commands, 1);
  assert.deepEqual(f.counts.batches, [4, 1]);
  assert.equal(f.counts.steps, 5);
  assert.equal(f.counts.inference, 0);
});
console.log(`MicroDuck cancellation regressions: ${passed} passed`);
