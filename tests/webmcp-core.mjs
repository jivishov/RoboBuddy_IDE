import assert from 'node:assert/strict';
import { AgentFacade, domainErrorResult } from '../src/webmcp/agent-facade.js';
import { MICRODUCK_COMMANDS } from '../src/microduck/command-catalog.js';
import { MicroDuckCommandBus } from '../src/microduck/command-bus.js';
import {
  boundedMicroDuckResult,
  createMicroDuckControlSchema,
  MICRODUCK_WEBMCP_COMMANDS,
  parseMicroDuckControlInput,
} from '../src/webmcp/microduck-control.js';

const schema = createMicroDuckControlSchema();
assert(Array.isArray(schema.oneOf) && schema.oneOf.length > MICRODUCK_WEBMCP_COMMANDS.length);
assert.deepEqual([...new Set(schema.oneOf.map((branch) => branch.properties.command.const))].sort(), Object.keys(MICRODUCK_COMMANDS).sort());
assert(schema.oneOf.every((branch) => branch.type === 'object' && branch.additionalProperties === false));
assert(schema.oneOf.every((branch) => branch.required.includes('command') && branch.properties.command.const));
const exampleValue = (property) => property.const ?? property.enum?.[0] ?? (property.type === 'boolean' ? false : property.minimum ?? 0);
for (const branch of schema.oneOf) {
  const example = Object.fromEntries(branch.required.map((name) => [name, exampleValue(branch.properties[name])]));
  assert.equal(parseMicroDuckControlInput(example).command, branch.properties.command.const);
}

for (const valid of [
  { command: 'move', vx: 0.2, duration_ms: 20 },
  { command: 'head', headYaw: 0.1, duration_ms: 5000 },
  { command: 'pose', z: -0.01, duration_ms: 40 },
  { command: 'mouth', open: 0.5, duration_ms: 20 },
  { command: 'sound', tag: 'chirp' },
  { command: 'sound', tag: 'chirp', hold: false },
  { command: 'sound', tag: 'wheee', hold: false },
  { command: 'sound', tag: 'wheee', hold: true, duration_ms: 20 },
  { command: 'theremin', active: false },
  { command: 'theremin', active: true, duration_ms: 20 },
  { command: 'chorale', active: false },
  { command: 'chorale', active: true, piece: 'wistful', voices: 2, duration_ms: 20 },
  { command: 'spawn_ball', position: [0.28, 0, 0.035] },
  { command: 'set_tof_stimulus', distanceM: 0.4, source: 'synthetic' },
]) assert.equal(parseMicroDuckControlInput(valid).command, valid.command);

for (const invalid of [
  { command: 'move', vx: 0.2 },
  { command: 'move', duration_ms: 19 },
  { command: 'move', duration_ms: 20, hidden: true },
  { command: 'sound', tag: 'wheee', hold: true },
  { command: 'sound', tag: 'chirp', duration_ms: 20 },
  { command: 'theremin', active: false, duration_ms: 20 },
  { command: 'chorale', active: true, duration_ms: 5001 },
  { command: 'shutdown' },
]) assert.throws(() => parseMicroDuckControlInput(invalid), (error) => error.code === 'INVALID_ARGUMENT');

const sourceState = { long: 'x'.repeat(400), list: Array.from({ length: 100 }, (_, index) => index), nested: { value: 1 } };
const bounded = boundedMicroDuckResult('get_state', { state: sourceState, completed: true });
sourceState.nested.value = 2;
assert.equal(bounded.state.long.length, 180);
assert.equal(bounded.state.list.length, 64);
assert.equal(bounded.state.nested.value, 1);
assert.deepEqual(Object.keys(bounded), ['ok', 'command', 'requested', 'applied', 'limitedBy', 'completed', 'state', 'audio']);

let busTime = 0;
let expiredLease = null;
const commandBus = new MicroDuckCommandBus({ now: () => busTime, onExpire: (lease) => { expiredLease = lease; } });
commandBus.execute('theremin', { active: true }, { source: 'webmcp', controllerId: 'owned-audio', durationMs: 20 });
busTime = 20;
commandBus.expire();
assert.deepEqual(expiredLease.owned, ['theremin']);
assert.equal(commandBus.snapshot().values.theremin.active, false);

function makeApp() {
  const state = {
    access: 'assist',
    generation: 4,
    profileId: 'microduck',
    workspaceStatus: 'ready',
    ready: true,
    executionState: 'idle',
    controllerActive: true,
    commandComplete: true,
    commandError: null,
    simulatorEpoch: 3,
    aborts: 0,
  };
  const app = {
    state,
    getAgentAccess: () => state.access,
    getAgentSnapshot: () => ({
      workspaceStatus: state.workspaceStatus,
      workspaceGeneration: state.generation,
      profileId: state.profileId,
      simulatorEpoch: state.simulatorEpoch,
      simulationMode: state.profileId === 'microduck' ? 'policy_sim' : 'source_plant',
      stateKind: state.profileId === 'microduck' ? 'browser_policy_sim' : 'modeled_source_plant',
    }),
    getAgentRegistrationContext: () => ({
      workspaceStatus: state.workspaceStatus,
      profileId: state.profileId,
      simulationMode: state.profileId === 'microduck' ? 'policy_sim' : 'source_plant',
      simulationReady: state.ready,
    }),
    isAgentMicroduckSimulationReady: () => state.ready,
    getExecutionState: () => state.executionState,
    executeAgentMicroduckCommand: async (command, args) => ({ ok: true, command, requested: args, applied: args, limitedBy: [], completed: command !== 'init', state: app.getAgentMicroduckState() }),
    abortAgentMicroduckCommand: () => { state.aborts += 1; state.controllerActive = false; return true; },
    isAgentMicroduckCommandComplete: () => {
      if (state.commandError) throw state.commandError;
      return state.commandComplete;
    },
    isAgentMicroduckControllerActive: () => state.controllerActive,
    getAgentMicroduckState: () => ({ simulationMode: 'policy_sim', stateKind: 'browser_policy_sim', hardwareValidated: false, virtualCamera: { mode: 'head', name: 'Head POV', purpose: 'Modeled source-frame render; not hardware video.', frame: 'head_camera', inset: false, transport: 'rendered simulation imagery only; no hardware video or media transport' }, audio: { unlocked: false } }),
  };
  return app;
}

const app = makeApp();
const facade = new AgentFacade(app);
facade.setRegistrationEpoch(9);
const read = await facade.controlMicroduck({ command: 'get_state' }, new AbortController().signal, 9);
assert.equal(read.state.stateKind, 'browser_policy_sim');
assert.deepEqual(read.state.virtualCamera, { mode: 'head', name: 'Head POV', purpose: 'Modeled source-frame render; not hardware video.', frame: 'head_camera', inset: false, transport: 'rendered simulation imagery only; no hardware video or media transport' });

app.state.controllerActive = true;
const leased = await facade.controlMicroduck({ command: 'move', vx: 0.1, duration_ms: 20 }, new AbortController().signal, 9);
assert.equal(leased.completed, true);
assert(app.state.aborts >= 1);

const abortedController = new AbortController();
abortedController.abort();
await assert.rejects(
  facade.controlMicroduck({ command: 'get_state' }, abortedController.signal, 9),
  (error) => error.code === 'OPERATION_CANCELLED',
);

app.state.profileId = 'openarm';
await assert.rejects(
  facade.controlMicroduck({ command: 'get_state' }, new AbortController().signal, 9),
  (error) => error.code === 'PROFILE_MISMATCH',
);
app.state.profileId = 'microduck';

const epochSnapshot = app.getAgentSnapshot();
app.state.simulatorEpoch += 1;
assert.throws(
  () => facade.assertMicroduckControlCurrent(epochSnapshot, 9, new AbortController().signal),
  (error) => error.code === 'OPERATION_CANCELLED',
);
app.state.simulatorEpoch = epochSnapshot.simulatorEpoch;

app.state.commandComplete = false;
const cancelledOneShot = Object.assign(new Error('manual stop'), { code: 'OPERATION_CANCELLED' });
setTimeout(() => { app.state.commandError = cancelledOneShot; }, 10);
await assert.rejects(
  facade.controlMicroduck({ command: 'init' }, new AbortController().signal, 9),
  (error) => error.code === 'OPERATION_CANCELLED',
);
app.state.commandError = null;
app.state.commandComplete = true;

let timedOut = false;
await assert.rejects(
  facade.waitWithControlGuards({
    snapshot: app.getAgentSnapshot(), expectedEpoch: 9, signal: new AbortController().signal,
    until: () => false, timeoutMs: 25, onTimeout: () => { timedOut = true; },
  }),
  (error) => error.code === 'POLICY_TIMEOUT',
);
assert.equal(timedOut, true);
assert.equal(domainErrorResult(Object.assign(new Error('locked'), { code: 'AUDIO_LOCKED' })).error.code, 'AUDIO_LOCKED');

console.log(`WebMCP core checks: ${MICRODUCK_WEBMCP_COMMANDS.length} commands, ${schema.oneOf.length} strict branches: OK`);
