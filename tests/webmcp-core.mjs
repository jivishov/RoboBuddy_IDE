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
import { createMicroduckVisualCueSchema, parseMicroduckVisualCueInput } from '../src/webmcp/microduck-visual-cues.js';

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

const visualCueSchema = createMicroduckVisualCueSchema();
assert.equal(visualCueSchema.oneOf.length, 4);
assert(visualCueSchema.oneOf.every((branch) => branch.type === 'object' && branch.additionalProperties === false));
const parsedLabel = parseMicroduckVisualCueInput({ operation: 'upsert', cue: { id: 'pose-note', kind: 'label', text: 'modeled pose', anchor: 'duck', offset_m: [0, 0, 0.18], color: '#5ED6BC', metrics: ['covered_m', 'remaining_to_east_edge_m'] } });
assert.deepEqual(parsedLabel, { operation: 'upsert', cue: { id: 'pose-note', kind: 'label', visible: true, color: '#5ed6bc', anchor: 'duck', position: null, offsetM: [0, 0, 0.18], text: 'modeled pose', metrics: ['covered_m', 'remaining_to_east_edge_m'] } });
const parsedRuler = parseMicroduckVisualCueInput({ operation: 'upsert', cue: { id: 'span', kind: 'ruler', start: [0, 0, 0.02], end: [1.2, 0, 0.02], title: 'reference' } });
assert.equal(parsedRuler.cue.kind, 'ruler');
assert.throws(() => parseMicroduckVisualCueInput({ operation: 'upsert', cue: { id: 'bad', kind: 'line', start: [0, 0, 0], end: [0, 0, 0] } }), (error) => error.code === 'INVALID_ARGUMENT');
assert.throws(() => parseMicroduckVisualCueInput({ operation: 'clear', extra: true }), (error) => error.code === 'INVALID_ARGUMENT');

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
    visualCues: [],
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
    manageAgentMicroduckVisualCues: (request) => {
      if (request.operation === 'list') return { cues: state.visualCues, cueCount: state.visualCues.length };
      if (request.operation === 'clear') { const removed = state.visualCues.length; state.visualCues = []; return { removed, cueCount: 0 }; }
      if (request.operation === 'remove') { const before = state.visualCues.length; state.visualCues = state.visualCues.filter((cue) => cue.id !== request.id); return { removed: state.visualCues.length !== before, id: request.id, cueCount: state.visualCues.length }; }
      const index = state.visualCues.findIndex((cue) => cue.id === request.cue.id);
      const created = index < 0;
      if (created) state.visualCues.push(request.cue);
      else state.visualCues[index] = request.cue;
      return { created, cue: request.cue, cueCount: state.visualCues.length };
    },
  };
  return app;
}

const app = makeApp();
const facade = new AgentFacade(app);
facade.setRegistrationEpoch(9);
const read = await facade.controlMicroduck({ command: 'get_state' }, new AbortController().signal, 9);
assert.equal(read.state.stateKind, 'browser_policy_sim');
assert.deepEqual(read.state.virtualCamera, { mode: 'head', name: 'Head POV', purpose: 'Modeled source-frame render; not hardware video.', frame: 'head_camera', inset: false, transport: 'rendered simulation imagery only; no hardware video or media transport' });

const visualAdded = facade.manageMicroduckVisualCues({ operation: 'upsert', cue: { id: 'state-note', kind: 'label', text: 'ready', anchor: 'ball' } }, new AbortController().signal, 9);
assert.deepEqual(visualAdded, { ok: true, operation: 'upsert', created: true, cue: { id: 'state-note', kind: 'label', visible: true, color: '#5ed6bc', anchor: 'ball', position: null, offsetM: [0, 0, 0.17], text: 'ready', metrics: [] }, cueCount: 1 });
const visualList = facade.manageMicroduckVisualCues({ operation: 'list' }, new AbortController().signal, 9);
assert.equal(visualList.cues.length, 1);
assert.throws(() => facade.manageMicroduckVisualCues({ operation: 'upsert', cue: { id: 'unsafe', kind: 'shader' } }, new AbortController().signal, 9), (error) => error.code === 'INVALID_ARGUMENT');

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
