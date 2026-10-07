import assert from 'node:assert/strict';
import { readFileSync, writeFileSync } from 'node:fs';
import { MicroDuckPhysicalSimulator } from '../../src/physics/microduck-physical-simulator.js';
import { MicroDuckController } from '../../src/physics/microduck-controller.js';
import { MICRODUCK_PHYSICAL_PACKAGES, MICRODUCK_PHYSICAL_SKILL_IDS, microduckCapability } from '../../src/physics/microduck-capabilities.js';
import { MicroDuckPhysicalBridge } from '../../src/runtime/microduck-physical-bridge.js';
import { createMicroDuckPhysicalControlSchema, executeMicroDuckPhysicalControl, WEBMCP_MICRODUCK_PHYSICAL_SCHEMA_VERSION as VERSION } from '../../src/webmcp/microduck-physical-control.js';
import { MICRODUCK_PHYSICAL_TASKS, loadPatchedScenario } from '../../src/task-catalog.js';
import { buildPatchedWorkspace } from '../../src/task-workspace.js';

// These stand-ins test API routing and state ownership, NOT physical behaviour.
// Real controller/contact behaviour remains covered by the existing native/browser gates.
function makeSim(packageKey = 'walk') {
  const sim = new MicroDuckPhysicalSimulator();
  sim.packageKey = packageKey;
  sim.modelPackage = MICRODUCK_PHYSICAL_PACKAGES[packageKey];
  sim.session = {};
  sim.controller = new MicroDuckController();
  sim.runEpoch = 0;
  sim.testEpoch = 0;
  sim.testTime = 0;
  sim.testUpright = false;
  sim.testAdvances = [];
  sim.getPhysicalAuthorityToken = () => ({ sessionId: 'interface-test', epoch: sim.testEpoch, robotId: sim.modelPackage.robotId, sceneRevision: 'interface-test' });
  sim.getState = () => ({ requested: { command: structuredClone(sim.requested) }, controller: {}, actual: { simulationTimeSeconds: sim.testTime }, capabilities: [] });
  sim.report = () => ({ upright: sim.testUpright });
  sim.advanceSeconds = async (seconds) => {
    assert(seconds > 0 && seconds <= 2, 'an unbounded or zero duration reached the controller');
    sim.testAdvances.push(seconds);
    sim.testTime += seconds;
    return { executedTicks: Math.round(seconds / 0.02), simulatedSeconds: seconds, completed: true, cancelled: false };
  };
  sim.reset = async () => { sim.testEpoch += 1; sim.runEpoch += 1; sim.controller.reset(); };
  return sim;
}
function facadeFor(sim) {
  return {
    controlSequence: 0,
    activeControlId: null,
    assertActive(epoch) { assert.equal(epoch, 1); },
    getRegistrationContext: () => ({ workspaceStatus: 'ready', simulationReady: true, profileId: 'microduck', simulationMode: 'physical_mujoco', workspaceGeneration: 1, simulatorEpoch: 1 }),
    app: {
      // Deliberately no setCommand/requestSkill/advanceSeconds methods on this host.
      sim: { backend: sim, getPhysicalAuthorityToken: () => sim.getPhysicalAuthorityToken() },
      getExecutionState: () => 'idle', renderPanels() {}, setStatus() {},
      resetSimulation: async () => { await sim.reset(); return true; },
    },
  };
}
const call = (facade, command) => executeMicroDuckPhysicalControl(facade, { schema_version: VERSION, ...command }, undefined, 1);
let checks = 0;
async function check(name, fn) { await fn(); checks += 1; console.log(`ok ${name}`); }

await check('every advertised skill is an executable controller command on its declared plant', () => {
  for (const id of MICRODUCK_PHYSICAL_SKILL_IDS) {
    const sim = makeSim(id.startsWith('kick_') ? 'kick' : 'groundContact');
    const result = sim.requestSkill(id);
    assert.equal(result.accepted, true, id);
    assert.equal(result.capability, microduckCapability(id).id);
    assert.equal(sim.controller.busy, true, id);
  }
  assert.equal(microduckCapability('sit').id, 'sit_stand');
  assert.equal(microduckCapability('stand_up').id, 'sit_stand');
  assert(!MICRODUCK_PHYSICAL_SKILL_IDS.includes('sit_stand'));
});
await check('Python advertises the actual skill commands and accepts sit then stand_up', async () => {
  const sim = makeSim('groundContact');
  const bridge = new MicroDuckPhysicalBridge(sim);
  const identity = await bridge.connect();
  assert.deepEqual(identity.skillCommands, [...MICRODUCK_PHYSICAL_SKILL_IDS]);
  await bridge.sendAction({ skill: 'sit' });
  assert.equal(sim.controller.sit, 'sitting');
  await bridge.sendAction({ skill: 'stand_up' });
  assert.equal(sim.controller.sit, 'rising');
});
await check('rejected combined Python actions do not latch velocity or reset evaluation', async () => {
  const sim = makeSim();
  sim.setCommand({ vx: 0.1 });
  const before = structuredClone(sim.requested);
  const evaluator = sim.evaluator;
  const bridge = new MicroDuckPhysicalBridge(sim);
  await bridge.connect();
  for (const [skill, code] of [['kick_right', 'PLANT_MISMATCH'], ['roller', 'CAPABILITY_UNSUPPORTED'], ['sit_stand', 'INVALID_ARGUMENT']]) {
    await assert.rejects(bridge.sendAction({ vx: 0.35, skill }), (error) => error.code === code);
    assert.deepEqual(sim.requested, before);
    assert.equal(sim.evaluator, evaluator);
    assert.equal(sim.controller.busy, false);
  }
  for (const value of [null, true, '0.2', [], NaN, Infinity]) {
    await assert.rejects(bridge.sendAction({ vx: value }));
    assert.deepEqual(sim.requested, before);
  }
  const ground = makeSim('groundContact');
  const groundBridge = new MicroDuckPhysicalBridge(ground);
  await groundBridge.connect();
  await assert.rejects(groundBridge.sendAction({ vx: 0.35, skill: 'sit' }));
  assert.equal(ground.controller.busy, false);
});
await check('public WebMCP dispatch uses the captured physical backend and handles zero advance', async () => {
  const sim = makeSim();
  const facade = facadeFor(sim);
  const result = await call(facade, { command: 'set_command', request: { vx: 0.35 }, advance_seconds: 0.2 });
  assert.equal(result.accepted, true);
  assert.equal(sim.testTime, 0.2);
  assert.equal(sim.requested.twist[0], 0.35);
  const zero = await call(facade, { command: 'advance', advance_seconds: 0 });
  assert.equal(zero.executedTicks, 0);
  assert.equal(zero.simulatedSeconds, 0);
  assert.equal(sim.testAdvances.length, 1);
  assert.equal(facade.activeControlId, null);
  await assert.rejects(call(facade, { command: 'request_skill', skill: 'kick_right' }), (error) => error.code === 'PLANT_MISMATCH');
  assert.equal(facade.activeControlId, null);
  const ground = makeSim('groundContact');
  await call(facadeFor(ground), { command: 'request_skill', skill: 'sit' });
  assert.equal(ground.controller.sit, 'sitting');
});
await check('WebMCP rejects malformed inputs before any mutation', async () => {
  const sim = makeSim();
  const facade = facadeFor(sim);
  const before = structuredClone(sim.requested);
  const bad = [
    { command: 'set_command', request: { vx: null } },
    { command: 'set_command', request: { vx: '0.2' } },
    { command: 'set_command', request: { vx: true } },
    { command: 'set_command', request: { vx: [] } },
    { command: 'set_command', request: { vx: 100 } },
    { command: 'set_command', request: {} },
    { command: 'set_command', request: { vx: 0, root_position: 1 } },
    { command: 'advance', advance_seconds: null },
    { command: 'advance', advance_seconds: '1' },
    { command: 'advance', advance_seconds: -1 },
    { command: 'advance', advance_seconds: 2.1 },
    { command: 'stop', request: { vx: 0 } },
    { command: 'request_skill', skill: 'sit_stand' },
  ];
  for (const input of bad) {
    await assert.rejects(call(facade, input), (error) => error.code === 'INVALID_ARGUMENT', JSON.stringify(input));
    assert.deepEqual(sim.requested, before);
  }
  assert.equal(sim.testAdvances.length, 0);
});
await check('Python rejects stale session epoch and run epoch, but owns its own reset', async () => {
  for (const field of ['testEpoch', 'runEpoch']) {
    const sim = makeSim();
    const bridge = new MicroDuckPhysicalBridge(sim);
    await bridge.connect();
    sim[field] += 1;
    assert.throws(() => bridge.getObservation(), (error) => error.code === 'OPERATION_CANCELLED');
    await assert.rejects(bridge.sendAction({ vx: 0.2 }), (error) => error.code === 'OPERATION_CANCELLED');
  }
  const sim = makeSim();
  const bridge = new MicroDuckPhysicalBridge(sim);
  await bridge.connect();
  const result = await bridge.reset();
  assert.equal(result.reset, true);
  assert.equal(result.countsAsRecovery, false);
  bridge.getObservation();
});
await check('zero/subtick waits are no-ops and a long polling period cannot exceed the timeout', async () => {
  const sim = makeSim();
  const bridge = new MicroDuckPhysicalBridge(sim);
  await bridge.connect();
  for (const timeoutSeconds of [0, 0.01]) {
    const result = await bridge.waitForGoal({ upright: true }, { timeoutSeconds });
    assert.equal(result.reached, false);
    assert.equal(result.simulatedSeconds, 0);
  }
  assert.equal(sim.testAdvances.length, 0);
  const result = await bridge.waitForGoal({ upright: true }, { timeoutSeconds: 0.05, controllerPeriodSeconds: 100 });
  assert.equal(result.simulatedSeconds, 0.04);
  assert.deepEqual(sim.testAdvances, [0.04]);
  const already = await bridge.waitForGoal({ upright: false }, { timeoutSeconds: 0 });
  assert.equal(already.reached, true);
  assert.equal(already.timedOut, false);
  await assert.rejects(bridge.waitForGoal({ arbitraryJoint: 0 }));
});
await check('paused waits report interruption instead of spinning or claiming timeout progress', async () => {
  const sim = makeSim();
  sim.advanceSeconds = async () => ({ executedTicks: 0, completed: false });
  const bridge = new MicroDuckPhysicalBridge(sim);
  await bridge.connect();
  const result = await bridge.waitForGoal({ upright: true });
  assert.equal(result.interrupted, true);
  assert.equal(result.simulatedSeconds, 0);
  assert.equal(result.timedOut, false);
});

const workspaces = [];
await check('all three physical MicroDuck starters target the controller API, not SO-101 objects', async () => {
  for (const task of MICRODUCK_PHYSICAL_TASKS) {
    const scenario = await loadPatchedScenario('microduck', task.id);
    const files = buildPatchedWorkspace('microduck', scenario);
    assert(files['main.py'].includes('MicroDuck physical'));
    assert(files['main.py'].includes('robot.send_action(stage["action"])'));
    assert(!JSON.stringify(files).includes('benchmark_block'));
    assert(!JSON.stringify(files).includes('targets_rad'));
    assert(scenario.workspaceRevision.endsWith('-public-interfaces-v2'));
    workspaces.push({ id: task.id, files });
  }
  const app = readFileSync(new URL('../../src/app-v2.js', import.meta.url), 'utf8');
  assert(app.includes('new MicroDuckPhysicalBridge(this.sim.backend)'), 'the actual app bridge factory is unwired');
});

const versioned = (input) => ({ schema_version: VERSION, ...input });
const valid = [
  { command: 'set_command', request: { vx: 0.35 }, advance_seconds: 0.2 },
  { command: 'advance', advance_seconds: 0 },
  { command: 'advance', advance_seconds: 2 },
  { command: 'setup_perturbation', perturbation: 'face_down', settle_seconds: 0 },
  { command: 'setup_perturbation', perturbation: 'face_up', settle_seconds: 2 },
  { command: 'stop' }, { command: 'reset' },
  ...[...MICRODUCK_PHYSICAL_SKILL_IDS, 'roller', 'roller_crouch'].map((skill) => ({ command: 'request_skill', skill })),
].map(versioned);
const invalid = [
  {}, { schema_version: 'wrong', command: 'stop' },
  ...[
    { command: 'set_command', request: {} },
    { command: 'set_command', request: { vx: 100 } },
    { command: 'set_command', request: { vx: null } },
    { command: 'set_command', request: { vx: '0.35' } },
    { command: 'set_command', request: { vx: true } },
    { command: 'advance', advance_seconds: null },
    { command: 'advance', advance_seconds: -1 },
    { command: 'advance', advance_seconds: 2.1 },
    { command: 'advance' },
    { command: 'stop', advance_seconds: 0 },
    { command: 'request_skill', skill: 'sit_stand' },
    { command: 'request_skill', skill: 'not_a_skill' },
    { command: 'reset', root_position: [0, 0, 1] },
  ].map(versioned),
];
if (process.argv[2]) writeFileSync(process.argv[2], JSON.stringify({ schema: createMicroDuckPhysicalControlSchema(), valid, invalid, workspaces }, null, 2));
console.log(`MicroDuck public-interface regression checks: ${checks} passed`);
