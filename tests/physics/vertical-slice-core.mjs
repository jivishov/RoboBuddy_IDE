import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { BrowserMuJoCoBackend } from '../../src/physics/browser-mujoco-backend.js';
import { assertPhysicalScene, assertSampledObservations, makeCommandEnvelope, MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE, PHYSICS_BACKEND_API_VERSION, sampledObservationCount, supportsSampledAdvance } from '../../src/physics/backend-contract.js';
import { EXECUTION_BACKENDS, MODEL_EVIDENCE, capabilityRecord, requireModelPackage } from '../../src/physics/model-registry.js';
import '../../src/physics/model-packages.js';
import { PHASE1_SCENE } from '../../src/physics/phase1-scene.js';
import { PhysicsSession } from '../../src/physics/session.js';

const requiredMethods = ['loadScene', 'reset', 'acceptCommand', 'advanceSteps', 'getObservation', 'getDiagnostics', 'pause', 'resume', 'cancelRun', 'exportTrace', 'dispose'];
for (const method of requiredMethods) assert.equal(typeof BrowserMuJoCoBackend.prototype[method], 'function', `${method} must exist on BrowserMuJoCoBackend`);
const capability = capabilityRecord({ backend: EXECUTION_BACKENDS.BROWSER_MUJOCO, capability: 'phase1-vertical-slice', evidence: MODEL_EVIDENCE.MODEL_DERIVED, limitations: ['Not hardware validation', 'Not a migrated production robot'] });
assert.equal(capability.backend, 'browser-mujoco'); assert.equal(capability.evidence, 'model-derived');
const phase1Package = requireModelPackage(PHASE1_SCENE.modelPackage);
assert.equal(phase1Package.robotId, PHASE1_SCENE.robotId); assert.equal(phase1Package.physics.timestepSeconds, PHASE1_SCENE.physics.timestepSeconds); assert.equal(phase1Package.physics.integrator, PHASE1_SCENE.physics.integrator);
assert.deepEqual(phase1Package.sceneConstraints.fixtures, PHASE1_SCENE.fixtures.map(({ id }) => id)); assert.deepEqual(phase1Package.sceneConstraints.objects, PHASE1_SCENE.objects.map(({ id }) => id));
assert.doesNotThrow(() => assertPhysicalScene(structuredClone(PHASE1_SCENE)));
assert.throws(() => assertPhysicalScene({ revision: 'partial', robotId: 'robot' }), /schemaVersion/);
assert.throws(() => assertPhysicalScene({ ...structuredClone(PHASE1_SCENE), unexpected: true }), /Unknown physical scene field/);
assert.throws(() => assertPhysicalScene({ ...structuredClone(PHASE1_SCENE), physics: { ...PHASE1_SCENE.physics, timestepSeconds: 0 } }), /timestepSeconds/);
assert.throws(() => makeCommandEnvelope({ sessionId: 's', epoch: 0, commandId: 'c', sceneRevision: 'r', robotId: 'robot', command: { type: 'x' } }), /positive integer/);
assert.throws(() => makeCommandEnvelope({ sessionId: 's', epoch: 1, commandId: 'c', sceneRevision: 'r', robotId: 'robot', command: { type: 'x' }, maxSteps: 0 }), /maxSteps/);

class FakeBackend {
  constructor() { this.calls = []; this.loaded = false; }
  async loadScene(scene, context) { this.calls.push(['loadScene', scene, context]); this.loaded = true; return { sceneRevision: scene.revision, robotId: scene.robotId, observation: { simulationTimeSeconds: 0 } }; }
  async reset(context) { this.calls.push(['reset', context]); return { simulationTimeSeconds: 0 }; }
  async acceptCommand(envelope) { this.calls.push(['acceptCommand', envelope]); return { status: 'accepted', commandId: envelope.commandId }; }
  async advanceSteps(steps, context) { this.calls.push(['advanceSteps', steps, context]); return { simulationTimeSeconds: steps * 0.002 }; }
  async getObservation(context) { this.calls.push(['getObservation', context]); return { simulationTimeSeconds: 0 }; }
  async getDiagnostics(context) { this.calls.push(['getDiagnostics', context]); return { backend: 'fake' }; }
  async pause(context) { this.calls.push(['pause', context]); return true; }
  async resume(context) { this.calls.push(['resume', context]); return true; }
  async cancelRun(context) { this.calls.push(['cancelRun', context]); this.loaded = false; return { cancelled: true, reloadRequired: true }; }
  async exportTrace(context) { this.calls.push(['exportTrace', context]); return { events: [] }; }
  dispose() { this.calls.push(['dispose']); }
}

assert.throws(() => new PhysicsSession(new FakeBackend(), { observationBatchSteps: 0 }), /positive integer/);

const fake = new FakeBackend(); const session = new PhysicsSession(fake, { sessionId: 'phase1-test-session' });
await assert.rejects(() => session.loadScene({ revision: 'partial', robotId: 'phase1_articulated_joint' }), /schemaVersion/);
const loaded = await session.loadScene(structuredClone(PHASE1_SCENE)); assert.equal(loaded.apiVersion, PHYSICS_BACKEND_API_VERSION); assert.equal(loaded.epoch, 1); assert.equal(fake.calls.at(-1)[2].epoch, 1);
assert.equal(fake.calls.at(-1)[1].modelPackage, PHASE1_SCENE.modelPackage); assert.equal(fake.calls.at(-1)[1].physics.timestepSeconds, 0.002); assert.equal(fake.calls.at(-1)[1].physics.integrator, 'RK4');
await session.sendCommand({ type: 'set_joint_target', targetRad: 0.4 }, { commandId: 'cmd-1', maxSteps: 50 });
const envelope = fake.calls.at(-1)[1]; assert.equal(envelope.sessionId, 'phase1-test-session'); assert.equal(envelope.epoch, 1); assert.equal(envelope.sceneRevision, PHASE1_SCENE.revision); assert.equal(envelope.robotId, PHASE1_SCENE.robotId); assert.equal(envelope.maxSteps, 50);
await session.reset(); assert.equal(session.epoch, 2); assert.equal(fake.calls.at(-1)[1].epoch, 2); await session.advanceSteps(10); assert.equal(fake.calls.at(-1)[2].epoch, 2);
const cancelled = await session.cancelRun('test-cancel'); assert.deepEqual(cancelled, { cancelled: true, reloadRequired: true }); assert.equal(session.robotId, null); assert.equal(session.sceneRevision, null);
await assert.rejects(() => session.sendCommand({ type: 'set_joint_target', targetRad: 0 }), /No physical scene loaded/);

const sampledFake = new FakeBackend();
const sampledSession = new PhysicsSession(sampledFake, { sessionId: 'sampled-advance-session', observationBatchSteps: 3 });
await sampledSession.loadScene(structuredClone(PHASE1_SCENE));
const sampledEvents = [];
const unsubscribeSampled = sampledSession.subscribe((event) => sampledEvents.push(event));
const sampledFinal = await sampledSession.advanceSteps(8);
const sampledCalls = sampledFake.calls.filter(([type]) => type === 'advanceSteps');
assert.deepEqual(sampledCalls.map(([, steps]) => steps), [3, 3, 2], 'long advances must be executed as deterministic physics-step observation batches');
assert.equal(sampledEvents.length, 3, 'each physics observation batch must publish actual backend state');
assert.equal(sampledEvents.every((event) => event.source === 'advanceSteps'), true);
assert.equal(sampledEvents.every((event) => event.sessionId === 'sampled-advance-session' && event.epoch === 1), true);
assert.deepEqual(sampledFinal, sampledEvents.at(-1).observation, 'advanceSteps must return the final authoritative observation');
unsubscribeSampled();
sampledSession.dispose();

class LostAuthorityBackend extends FakeBackend {
  async advanceSteps(steps, context) {
    this.calls.push(['advanceSteps', steps, context]);
    this.loaded = false;
    throw new Error('runtime plant failure');
  }
  async getDiagnostics(context) {
    this.calls.push(['getDiagnostics', context]);
    return { backend: 'fake', loaded: this.loaded };
  }
}

const lostBackend = new LostAuthorityBackend();
const lostSession = new PhysicsSession(lostBackend, { sessionId: 'lost-authority-session' });
await lostSession.loadScene(structuredClone(PHASE1_SCENE));
await lostSession.sendCommand({ type: 'set_joint_target', targetRad: 0.3 }, { commandId: 'before-runtime-failure', maxSteps: 20 });
assert.equal(lostSession.activeCommandId, 'before-runtime-failure');
await assert.rejects(() => lostSession.advanceSteps(1), /runtime plant failure/);
assert.equal(lostSession.robotId, null, 'fatal backend authority loss must clear the session robot');
assert.equal(lostSession.sceneRevision, null, 'fatal backend authority loss must clear the session scene revision');
assert.equal(lostSession.activeCommandId, null, 'fatal backend authority loss must clear the accepted command identity');
await assert.rejects(() => lostSession.advanceSteps(1), /No physical scene loaded/);
lostSession.dispose();

class RejectedCommandBackend extends FakeBackend {
  async acceptCommand(envelope) {
    this.calls.push(['acceptCommand', envelope]);
    throw new RangeError('target outside declared range');
  }
  async getDiagnostics(context) {
    this.calls.push(['getDiagnostics', context]);
    return { backend: 'fake', loaded: this.loaded };
  }
}

const rejectedBackend = new RejectedCommandBackend();
const rejectedSession = new PhysicsSession(rejectedBackend, { sessionId: 'rejected-command-session' });
await rejectedSession.loadScene(structuredClone(PHASE1_SCENE));
await assert.rejects(() => rejectedSession.sendCommand({ type: 'set_joint_target', targetRad: 99 }, { commandId: 'rejected-command', maxSteps: 20 }), /outside declared range/);
assert.equal(rejectedSession.robotId, PHASE1_SCENE.robotId, 'ordinary command rejection must not discard a valid physical plant');
assert.equal(rejectedSession.sceneRevision, PHASE1_SCENE.revision);
assert.equal(rejectedSession.activeCommandId, null, 'a rejected command must never become the active accepted command');
rejectedSession.dispose();

const workerSource = readFileSync(new URL('../../src/physics/mujoco-worker.js', import.meta.url), 'utf8');
for (const token of ['mj_name2id', 'jnt_qposadr', 'jnt_dofadr', 'actuator_trnid', 'data.xpos', "crypto.subtle.digest('SHA-256'", 'descriptor.joints', 'descriptor.actuators']) assert.ok(workerSource.includes(token), `worker must use named/address/model-descriptor access: ${token}`);
for (const forbidden of ['data.qpos?.[0]', 'data.qvel?.[0]', 'data.ctrl[0]', "'hinge_position'", "'free_box'", "'models/vertical-slice/model.xml'"]) assert.ok(!workerSource.includes(forbidden), `generic worker must not rely on Phase 1-specific model semantics: ${forbidden}`);
assert.ok(workerSource.includes('collection.delete?.()'), 'worker must release the copied MuJoCo contact vector'); assert.ok(workerSource.includes('contact.delete?.()'), 'worker must release copied MuJoCo contact handles');
assert.ok(workerSource.includes('is outside the actuator control range'), 'worker must reject out-of-range targets instead of silently clamping'); assert.ok(workerSource.includes('Worker rejected non-registry model asset path'), 'worker must reject arbitrary model URLs/paths');
const backendSource = readFileSync(new URL('../../src/physics/browser-mujoco-backend.js', import.meta.url), 'utf8');
assert.ok(backendSource.includes('executedSteps'), 'backend must account actual executed physics steps');
assert.ok(backendSource.includes('remainingSteps -= executedSteps'), 'paused/no-op requests must not consume unexecuted command steps');
assert.ok(backendSource.includes('requireModelPackage(scene.modelPackage)'), 'backend must resolve scenes through the controlled model registry');
assert.ok(!backendSource.includes('PHASE1_SCENE'), 'generic backend must not be intrinsically tied to the Phase 1 scene');
assert.ok(backendSource.includes('#failLoadedScene(reason)'), 'backend must centralize fatal runtime invalidation');
assert.ok(backendSource.includes("this.#failLoadedScene('runtime step failed')"), 'fatal stepping faults must invalidate physical authority');
assert.ok(backendSource.includes('else this.#failLoadedScene(reason)'), 'worker crashes must invalidate physical authority');
// ---------------------------------------------------------------------------
// Sampled-advance capability: one authoritative request advances many real physics
// steps and returns the ground-truth observations captured at a declared step cadence.
// These checks cover the architectural invariants the capability must never break.
// ---------------------------------------------------------------------------

assert.equal(sampledObservationCount(100, 20), 5, 'an exactly aligned request samples once per cadence period');
assert.equal(sampledObservationCount(103, 20), 6, 'a remainder must add one final sample for the true post-request state');
assert.equal(sampledObservationCount(10, 20), 1, 'a request shorter than one period still reports its true final state');
assert.throws(() => sampledObservationCount(0, 20), /stepCount/);
assert.throws(() => sampledObservationCount(20, 0), /sampleEverySteps/);

const sampleObs = (time) => ({ simulationTimeSeconds: time });
assert.throws(() => assertSampledObservations({ executedSteps: 4, observations: [] }, { stepCount: 4, sampleEverySteps: 2 }), /at least one authoritative observation/);
assert.throws(() => assertSampledObservations({ executedSteps: 4.5, observations: [sampleObs(1)] }, { stepCount: 4, sampleEverySteps: 2 }), /integer executedSteps/);
assert.throws(() => assertSampledObservations({ executedSteps: 6, observations: [sampleObs(1)] }, { stepCount: 4, sampleEverySteps: 2 }), /executed 6 steps/);
assert.throws(() => assertSampledObservations({ executedSteps: 4, observations: [sampleObs(0.004), sampleObs(0.002)] }, { stepCount: 4, sampleEverySteps: 2 }), /strictly increasing simulation time/);
assert.throws(() => assertSampledObservations({ executedSteps: 4, observations: [sampleObs(0.002), sampleObs(0.002)] }, { stepCount: 4, sampleEverySteps: 2 }), /strictly increasing simulation time/);
assert.throws(() => assertSampledObservations({ executedSteps: 4, observations: [sampleObs(0.002), sampleObs(0.004)], finalObservation: sampleObs(9) }, { stepCount: 4, sampleEverySteps: 2 }), /must be the last returned sample/);
assert.throws(() => assertSampledObservations({ executedSteps: 6, observations: [sampleObs(0.002), sampleObs(0.004), sampleObs(0.006), sampleObs(0.008)] }, { stepCount: 6, sampleEverySteps: 3 }), /for a declared 2-sample cadence/);

// A fake MuJoCo worker that answers the real worker protocol. It advances a step counter
// exactly as the browser worker advances mj_step, so backend accounting is exercised
// against real message traffic instead of a hand-written stub of the backend itself.
const sampledModelPackage = requireModelPackage(PHASE1_SCENE.modelPackage);
class FakeMuJoCoWorker {
  static instances = [];
  constructor() {
    this.onmessage = null; this.onerror = null; this.terminated = false;
    this.steps = 0; this.paused = false; this.ops = []; this.failNextSampled = null;
    FakeMuJoCoWorker.instances.push(this);
  }
  #observation() {
    return {
      simulationTime: this.steps * sampledModelPackage.physics.timestepSeconds,
      model: { id: sampledModelPackage.modelId, asset: sampledModelPackage.asset, sha256: sampledModelPackage.sha256 },
      engine: { version: '3.11.0', versionEvidence: 'fake worker', timestepSeconds: sampledModelPackage.physics.timestepSeconds },
      joints: {}, bodies: {}, contactCount: 0, contactsReadable: true, contacts: [],
    };
  }
  postMessage({ id, op, payload }) {
    this.ops.push(op);
    queueMicrotask(() => {
      if (this.terminated) return;
      try {
        let result;
        if (op === 'load') { this.steps = 0; this.paused = false; result = this.#observation(); }
        else if (op === 'reset') { this.steps = 0; this.paused = false; result = this.#observation(); }
        else if (op === 'observe') result = this.#observation();
        else if (op === 'command') result = this.#observation();
        else if (op === 'pause') { this.paused = true; result = this.#observation(); }
        else if (op === 'resume') { this.paused = false; result = this.#observation(); }
        else if (op === 'step') { if (!this.paused) this.steps += payload.count; result = this.#observation(); }
        else if (op === 'stepSampled') {
          const { count, sampleEverySteps } = payload;
          if (this.paused) result = { observations: [this.#observation()], executedSteps: 0, sampleEverySteps };
          else {
            const observations = [];
            for (let index = 1; index <= count; index += 1) {
              this.steps += 1;
              if (index === count || index % sampleEverySteps === 0) observations.push(this.#observation());
            }
            if (this.failNextSampled === 'duplicate') { observations.push(observations.at(-1)); this.failNextSampled = null; }
            if (this.failNextSampled === 'skew') { observations.splice(1, 1); this.failNextSampled = null; }
            result = { observations, executedSteps: count, sampleEverySteps };
          }
        } else throw new Error(`Unknown fake worker operation: ${op}`);
        this.onmessage?.({ data: { id, ok: true, payload: result } });
      } catch (error) {
        this.onmessage?.({ data: { id, ok: false, error: String(error?.message || error) } });
      }
    });
  }
  terminate() { this.terminated = true; }
}
globalThis.Worker = FakeMuJoCoWorker;

const workerBackend = () => new BrowserMuJoCoBackend({ workerUrl: 'about:blank' });
const loadedSampledSession = async (sessionId, observationBatchSteps) => {
  const backend = workerBackend();
  const built = new PhysicsSession(backend, { sessionId, observationBatchSteps });
  await built.loadScene(structuredClone(PHASE1_SCENE));
  return { backend, session: built };
};

assert.equal(typeof BrowserMuJoCoBackend.prototype.advanceStepsObserved, 'function', 'the browser backend must expose the sampled-advance capability');
assert.equal(supportsSampledAdvance(BrowserMuJoCoBackend.prototype), true);
assert.equal(supportsSampledAdvance(new FakeBackend()), false, 'a backend without the capability must not advertise it');

// Ordering, final state and publication: a 100-step advance sampled every 20 steps
// publishes five strictly increasing real observations and returns the true final state.
{
  const { backend, session: ordered } = await loadedSampledSession('sampled-order-session', 20);
  const events = [];
  ordered.subscribe((event) => events.push(event));
  const worker = FakeMuJoCoWorker.instances.at(-1);
  const final = await ordered.advanceSteps(100);
  assert.deepEqual(worker.ops.filter((op) => op === 'stepSampled').length, 1, 'a sampled advance must need exactly one worker request');
  assert.equal(worker.ops.includes('step'), false, 'the sampled path must not also issue unsampled step requests');
  assert.equal(worker.steps, 100, 'the worker must execute every requested physics step exactly once');
  assert.equal(events.length, 5, 'each declared cadence position must publish one authoritative observation');
  const times = events.map((event) => event.observation.simulationTimeSeconds);
  assert.deepEqual(times, [0.04, 0.08, 0.12, 0.16, 0.2], 'samples must be ordered by simulation time at the declared cadence');
  assert.equal(new Set(times).size, times.length, 'samples must never be published twice');
  assert.equal(final.simulationTimeSeconds, 0.2, 'the returned observation must be the state after all requested steps');
  assert.deepEqual(final, events.at(-1).observation, 'the final observation must be the last published sample');
  assert.equal(events.every((event) => event.source === 'advanceSteps' && event.epoch === 1), true);
  const trace = await backend.exportTrace({ sessionId: 'sampled-order-session' });
  const sampledTrace = trace.events.filter((event) => event.type === 'advanceStepsObserved');
  assert.equal(sampledTrace.length, 1, 'bounded evidence records one event per sampled advance, not one per sample');
  assert.deepEqual(
    { requestedSteps: sampledTrace[0].requestedSteps, executedSteps: sampledTrace[0].executedSteps, sampleEverySteps: sampledTrace[0].sampleEverySteps, sampleCount: sampledTrace[0].sampleCount },
    { requestedSteps: 100, executedSteps: 100, sampleEverySteps: 20, sampleCount: 5 },
  );
  ordered.dispose();
}

// Remainder: unaligned tails still execute, and the true final state is always published.
{
  const { session: remainder } = await loadedSampledSession('sampled-remainder-session', 20);
  const events = [];
  remainder.subscribe((event) => events.push(event));
  const worker = FakeMuJoCoWorker.instances.at(-1);
  const final = await remainder.advanceSteps(103);
  assert.equal(worker.steps, 103, 'the unaligned final steps must still execute');
  assert.equal(events.length, 6, 'the unaligned tail adds exactly one final sample');
  assert.equal(final.simulationTimeSeconds, 103 * 0.002);
  const remainderSteps = events.map((event) => Math.round(event.observation.simulationTimeSeconds / 0.002));
  assert.deepEqual(remainderSteps, [20, 40, 60, 80, 100, 103], 'cadence positions plus the unaligned final step must all be published in order');
  remainder.dispose();
}

// Command budget: sampled stepping charges the actual executed steps exactly once.
{
  const { backend, session: budget } = await loadedSampledSession('sampled-budget-session', 20);
  await budget.sendCommand({ type: 'set_joint_target', targetRad: 0.4 }, { commandId: 'budgeted', maxSteps: 150 });
  assert.equal(backend.commandBudget.remainingSteps, 150);
  await budget.advanceSteps(100);
  assert.equal(backend.commandBudget.remainingSteps, 50, 'a 100-step sampled advance must consume exactly 100 budget steps');
  await assert.rejects(() => budget.advanceSteps(60), /exceeds remaining command budget 50/);
  assert.equal(backend.commandBudget.remainingSteps, 50, 'a rejected request must not consume budget');
  budget.dispose();
}

// Pause: a paused sampled advance executes zero physics steps and publishes the unchanged state.
{
  const { backend, session: paused } = await loadedSampledSession('sampled-pause-session', 20);
  await paused.advanceSteps(40);
  const worker = FakeMuJoCoWorker.instances.at(-1);
  await paused.pause();
  const events = [];
  paused.subscribe((event) => events.push(event));
  const held = await paused.advanceSteps(100);
  assert.equal(worker.steps, 40, 'paused sampled advancement must not advance MuJoCo');
  assert.equal(held.simulationTimeSeconds, 0.08, 'a paused advance returns the unchanged authoritative state');
  assert.equal(events.length, 1, 'a paused advance publishes exactly one unchanged observation');
  assert.equal(backend.state, 'paused');
  await paused.resume();
  await paused.advanceSteps(20);
  assert.equal(worker.steps, 60, 'resuming restores ordinary sampled stepping');
  paused.dispose();
}

// Epoch/generation: samples produced for a superseded run can never reach its replacement.
{
  const { backend, session: stale } = await loadedSampledSession('sampled-epoch-session', 20);
  const events = [];
  stale.subscribe((event) => events.push(event));
  const inFlight = stale.advanceSteps(100);
  const reset = stale.reset({ reason: 'replacement-run' });
  await assert.rejects(() => inFlight, /Stale MuJoCo worker response|Stale physics epoch/);
  await reset;
  assert.equal(events.some((event) => event.source === 'advanceSteps'), false, 'a superseded sampled advance must publish nothing into the replacement run');
  assert.equal(backend.epoch, 2);
  stale.dispose();
}

// A worker that duplicates or skips a declared sample is rejected rather than published.
{
  const { session: malformed } = await loadedSampledSession('sampled-malformed-session', 20);
  const worker = FakeMuJoCoWorker.instances.at(-1);
  const events = [];
  malformed.subscribe((event) => events.push(event));
  worker.failNextSampled = 'duplicate';
  await assert.rejects(() => malformed.advanceSteps(100), /samples for a declared 5-sample cadence|strictly ordered/);
  assert.equal(events.length, 0, 'malformed sampled responses must never be published');
  malformed.dispose();
}
{
  const { session: skewed } = await loadedSampledSession('sampled-skew-session', 20);
  const worker = FakeMuJoCoWorker.instances.at(-1);
  worker.failNextSampled = 'skew';
  await assert.rejects(() => skewed.advanceSteps(100), /instead of the declared|samples for a declared/);
  skewed.dispose();
}

// Bounded responses: an interval longer than one bounded response is split into several
// authoritative requests whose samples stay ordered and unduplicated across the boundary.
{
  const { session: chunked } = await loadedSampledSession('sampled-chunked-session', 1);
  const events = [];
  chunked.subscribe((event) => events.push(event));
  const worker = FakeMuJoCoWorker.instances.at(-1);
  const final = await chunked.advanceSteps(3000);
  assert.equal(worker.ops.filter((op) => op === 'stepSampled').length, 2, `a ${3000}-step advance must split across the ${MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE}-observation response bound`);
  assert.equal(worker.steps, 3000, 'splitting a long advance must still execute every requested step exactly once');
  assert.equal(events.length, 3000, 'every declared cadence position must publish exactly once across request boundaries');
  const stepsSeen = events.map((event) => Math.round(event.observation.simulationTimeSeconds / 0.002));
  assert.equal(stepsSeen.every((value, index) => value === index + 1), true, 'samples must stay ordered and unduplicated across request boundaries');
  assert.equal(Math.round(final.simulationTimeSeconds / 0.002), 3000);
  chunked.dispose();
}

// The bounded response is enforced by the backend, not only by the session's chunking.
{
  const overBound = workerBackend();
  const context = { sessionId: 'sampled-bound-session', epoch: 1 };
  await overBound.loadScene(structuredClone(PHASE1_SCENE), context);
  await assert.rejects(
    () => overBound.advanceStepsObserved((MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE + 1) * 2, { ...context, sampleEverySteps: 2 }),
    /bounded response carries at most/,
  );
  await assert.rejects(() => overBound.advanceStepsObserved(10, { ...context, sampleEverySteps: 0 }), /sampleEverySteps must be a positive integer/);
  await assert.rejects(() => overBound.advanceStepsObserved(10, context), /sampleEverySteps must be a positive integer/);
  await assert.rejects(() => overBound.advanceStepsObserved(10, { sessionId: 'someone-else', epoch: 1, sampleEverySteps: 2 }), /Stale or foreign physics session/);
  overBound.dispose();
}

// Backward compatibility: a session without a declared cadence, and any backend without the
// capability, keep the original single-observation advanceSteps behaviour.
{
  const { backend, session: unsampled } = await loadedSampledSession('unsampled-session', null);
  const worker = FakeMuJoCoWorker.instances.at(-1);
  const events = [];
  unsampled.subscribe((event) => events.push(event));
  const final = await unsampled.advanceSteps(100);
  assert.deepEqual(worker.ops.filter((op) => op === 'step').length, 1, 'an unsampled session must use the original single-observation advance');
  assert.equal(worker.ops.includes('stepSampled'), false);
  assert.equal(events.length, 1, 'an unsampled advance publishes exactly one observation');
  assert.equal(final.simulationTimeSeconds, 0.2);
  assert.equal(backend.state, 'ready');
  unsampled.dispose();
}
{
  const legacyFake = new FakeBackend();
  const legacySession = new PhysicsSession(legacyFake, { sessionId: 'legacy-chunked-session', observationBatchSteps: 3 });
  await legacySession.loadScene(structuredClone(PHASE1_SCENE));
  const events = [];
  legacySession.subscribe((event) => events.push(event));
  await legacySession.advanceSteps(8);
  assert.deepEqual(legacyFake.calls.filter(([type]) => type === 'advanceSteps').map(([, steps]) => steps), [3, 3, 2], 'a backend without the sampled capability keeps the chunked advanceSteps fallback');
  assert.equal(events.length, 3);
  legacySession.dispose();
}

delete globalThis.Worker;

const openArmWorkerSource = readFileSync(new URL('../../src/physics/openarm-mujoco-worker.js', import.meta.url), 'utf8');
for (const source of [workerSource, openArmWorkerSource]) {
  assert.ok(source.includes('function stepSampled('), 'each MuJoCo worker must own the sampling loop so one request covers many steps');
  assert.ok(source.includes('if (index === count || index % sampleEverySteps === 0) observations.push(observation());'), 'samples must be plain observations of actual MuJoCo state at declared step positions');
  assert.ok(source.includes('return { observations: [observation()], executedSteps: 0, sampleEverySteps };'), 'a paused sampled advance must execute zero MuJoCo steps');
  assert.ok(source.includes('MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE'), 'sampled responses must stay bounded');
  for (const forbidden of ['supportWhileHeld', 'taskSuccess', 'grasp', 'evaluator']) {
    assert.ok(!source.includes(forbidden), `the worker must publish physical state only, never task events: ${forbidden}`);
  }
}
assert.ok(backendSource.includes('remainingSteps -= executedSteps'), 'sampled advances must charge actual executed steps to the command budget once');
assert.ok(backendSource.includes("this.#failLoadedScene('runtime sampled step failed')"), 'fatal sampled-stepping faults must invalidate physical authority');
assert.ok(backendSource.includes('sampleCount: observations.length'), 'sampled evidence must record how many authoritative samples a request produced');
const sessionSource = readFileSync(new URL('../../src/physics/session.js', import.meta.url), 'utf8');
assert.ok(sessionSource.includes('supportsSampledAdvance(this.backend)'), 'the session must treat sampled stepping as an optional backend capability');
assert.ok(sessionSource.includes('assertSampledObservations(result'), 'the session must validate sample ordering before publishing');

session.dispose(); console.log('Phase 1 vertical-slice contract checks: OK');
