// MicroDuck Phase 5C lifecycle gate.
//
// It drives the REAL PhysicsSession, BrowserMuJoCoBackend and MicroDuckController through a
// scripted stand-in for the MuJoCo worker, so the parts the browser lane would otherwise be
// the only witness to are checked here: the declared setup path, the command budget, the
// fixed-cadence control loop, cancellation mid-run, stale-session rejection, and the fact
// that ordinary control writes actuator targets and nothing else.
//
// The stand-in is deliberately not a physics model. It records every operation it is asked to
// perform, which is what lets this file assert what the physical path does and does not do.
//
// Run: node tests/physics/microduck-lifecycle-core.mjs
import { PhysicsSession } from '../../src/physics/session.js';
import { BrowserMuJoCoBackend } from '../../src/physics/browser-mujoco-backend.js';
import { PHYSICS_BACKEND_API_VERSION } from '../../src/physics/backend-contract.js';
import { MICRODUCK_NOMINAL_FIRMWARE_GAIN, MICRODUCK_WALK_PACKAGE, microDuckKpForFirmwareGain } from '../../src/physics/microduck-model-package.js';
import { MICRODUCK_WALK_SCENE } from '../../src/physics/microduck-scene.js';
import {
  MICRODUCK_CONTROL_DECIMATION, MICRODUCK_HOME_POSITION_RAD, MICRODUCK_POLICY_JOINT_ORDER,
  MicroDuckController, makeCommand,
} from '../../src/physics/microduck-controller.js';
import { MICRODUCK_PHYSICAL_POLICIES } from '../../src/physics/microduck-capabilities.js';
import { MicroDuckPhysicalEvaluator } from '../../src/physics/microduck-task-evaluator.js';
import { MicroDuckPhysicalSimulator } from '../../src/physics/microduck-physical-simulator.js';

let passed = 0;
const failures = [];
function check(name, fn) {
  return Promise.resolve()
    .then(fn)
    .then(() => { passed += 1; console.log(`  ok  ${name}`); })
    .catch((error) => { failures.push(name); console.log(`FAIL  ${name}\n      ${error?.message || error}`); });
}
function assert(condition, message) { if (!condition) throw new Error(message); }

// ---------------------------------------------------------------- worker stand-in
const TIMESTEP = MICRODUCK_WALK_PACKAGE.physics.timestepSeconds;

class ScriptedWorker {
  constructor() {
    this.onmessage = null;
    this.onerror = null;
    this.terminated = false;
    this.ops = [];
    this.time = 0;
    this.paused = false;
    this.actuationEnabled = true;
    this.firmwareGain = MICRODUCK_NOMINAL_FIRMWARE_GAIN;
    this.setupLog = [];
    this.ctrl = Object.fromEntries(MICRODUCK_POLICY_JOINT_ORDER.map((id, slot) => [id, MICRODUCK_HOME_POSITION_RAD[slot]]));
    this.trunk = { positionM: [0, 0, 0.12], quaternionWxyz: [1, 0, 0, 0] };
    this.footFloor = { left_foot_collision: true, right_foot_collision: true };
  }

  observation() {
    return {
      simulationTime: this.time,
      model: { id: MICRODUCK_WALK_PACKAGE.modelId, asset: MICRODUCK_WALK_PACKAGE.asset, sha256: MICRODUCK_WALK_PACKAGE.sha256 },
      engine: { version: '3.11.0', versionEvidence: 'scripted lifecycle stand-in', timestepSeconds: TIMESTEP },
      firmwareGain: this.firmwareGain,
      appliedServoKp: microDuckKpForFirmwareGain(this.firmwareGain),
      // A zero gain produces no force however far the target is from the joint.
      actuatorForceTotalNm: this.firmwareGain === 0 ? 0 : 1,
      joints: Object.fromEntries(MICRODUCK_POLICY_JOINT_ORDER.map((id, slot) => [id, {
        positionRad: MICRODUCK_HOME_POSITION_RAD[slot], velocityRadS: 0,
        targetRad: this.ctrl[id], effortNm: 0,
        controlRangeRad: [-10, 10], forceRangeNm: [-0.96, 0.96],
        jointRangeRad: [...MICRODUCK_WALK_PACKAGE.joints[slot].rangeRad], passive: false,
      }])),
      bodies: {
        trunk_base: {
          frame: 'mujoco_world', freeBody: true,
          positionM: [...this.trunk.positionM], quaternionWxyz: [...this.trunk.quaternionWxyz],
          linearVelocityMS: [0, 0, 0], angularVelocityRadS: [0, 0, 0],
        },
        ankle_left: { frame: 'mujoco_world', positionM: [0, 0.04, 0.01], quaternionWxyz: [1, 0, 0, 0] },
        ankle_right: { frame: 'mujoco_world', positionM: [0, -0.04, 0.01], quaternionWxyz: [1, 0, 0, 0] },
        bottom_head_shell: { frame: 'mujoco_world', positionM: [0, 0, 0.18], quaternionWxyz: [1, 0, 0, 0] },
        left_upper_leg: { frame: 'mujoco_world', positionM: [0, 0.03, 0.07], quaternionWxyz: [1, 0, 0, 0] },
        right_upper_leg: { frame: 'mujoco_world', positionM: [0, -0.03, 0.07], quaternionWxyz: [1, 0, 0, 0] },
      },
      imu: { site: 'imu', gyroRadS: [0, 0, 0], projectedGravity: [0, 0, -1], source: 'scripted' },
      contactCount: 2, contactsReadable: true, contacts: [],
      footContacts: { floor: { ...this.footFloor }, ball: [] },
      actuationEnabled: this.actuationEnabled,
      setupLog: this.setupLog.map((item) => ({ ...item })),
    };
  }

  postMessage({ id, op, payload }) {
    this.ops.push({ op, payload: payload ? structuredClone(payload) : null });
    queueMicrotask(() => {
      if (this.terminated) return;
      let ok = true; let result = null; let error = null;
      try {
        if (op === 'load') { this.time = 0; result = this.observation(); }
        else if (op === 'reset') { this.time = 0; this.setupLog = [{ event: 'reset', simulationTime: 0 }]; this.actuationEnabled = true; result = this.observation(); }
        else if (op === 'step') { if (!this.paused) this.time += payload.count * TIMESTEP; result = this.observation(); }
        else if (op === 'stepSampled') {
          // A paused authority executes no physics and returns exactly one unchanged sample,
          // matching src/physics/microduck-mujoco-worker.js.
          if (this.paused) result = { observations: [this.observation()], executedSteps: 0, sampleEverySteps: payload.sampleEverySteps };
          else {
            const observations = [];
            for (let index = 1; index <= payload.count; index += 1) {
              this.time += TIMESTEP;
              if (index === payload.count || index % payload.sampleEverySteps === 0) observations.push(this.observation());
            }
            result = { observations, executedSteps: payload.count, sampleEverySteps: payload.sampleEverySteps };
          }
        } else if (op === 'observe') result = this.observation();
        else if (op === 'command') {
          if (payload.type !== 'set_joint_targets') throw new Error(`unsupported command ${payload.type}`);
          // Mirrors the real worker: the request is always latched, and it is the servo gain
          // that a torque-off removes. Zeroing ctrl instead would be a full-strength command
          // to a straight-legged pose, which is actuation rather than its absence.
          if (payload.firmwareGain != null && this.actuationEnabled) this.firmwareGain = Number(payload.firmwareGain);
          for (const [jointId, value] of Object.entries(payload.targetsRad)) this.ctrl[jointId] = value;
          result = this.observation();
        } else if (op === 'setup') {
          if (payload.type === 'set_actuation_enabled') { this.actuationEnabled = payload.enabled !== false; this.firmwareGain = this.actuationEnabled ? MICRODUCK_NOMINAL_FIRMWARE_GAIN : 0; this.setupLog.push({ event: 'setup_actuation', actuationEnabled: this.actuationEnabled, firmwareGain: this.firmwareGain, simulationTime: this.time }); }
          else if (payload.type === 'set_trunk_orientation') { this.trunk.quaternionWxyz = [...payload.quaternionWxyz]; this.setupLog.push({ event: 'setup_trunk_orientation', label: payload.label, simulationTime: this.time }); }
          else throw new Error(`unsupported setup ${payload.type}`);
          result = this.observation();
        } else if (op === 'pause') { this.paused = true; result = this.observation(); }
        else if (op === 'resume') { this.paused = false; result = this.observation(); }
        else if (op === 'dispose') result = true;
        else throw new Error(`unknown op ${op}`);
      } catch (thrown) { ok = false; error = String(thrown?.message || thrown); }
      this.onmessage?.({ data: { id, ok, payload: result, error } });
    });
  }

  terminate() { this.terminated = true; }
  counts(op) { return this.ops.filter((item) => item.op === op).length; }
  lastPayload(op) { return [...this.ops].reverse().find((item) => item.op === op)?.payload ?? null; }
}

const SETUP_OPERATIONS = ['set_trunk_orientation', 'set_object_pose', 'set_actuation_enabled'];

async function makeSession({ setupOperations = SETUP_OPERATIONS } = {}) {
  const worker = new ScriptedWorker();
  const backend = new BrowserMuJoCoBackend({ workerUrl: 'about:blank', setupOperations });
  // Substitute the scripted worker for the real one; everything else is the real backend.
  backend.spawnForTest = () => { backend.worker = worker; };
  const originalSpawn = Object.getOwnPropertyDescriptor(BrowserMuJoCoBackend.prototype, 'loadScene');
  void originalSpawn;
  backend.worker = worker;
  worker.onmessage = ({ data }) => {
    const request = backend.pending.get(data?.id);
    if (!request) return;
    backend.pending.delete(data.id);
    if (data.ok) request.resolve(data.payload); else request.reject(new Error(data.error || 'worker failed'));
  };
  // The real backend fetches the model and compares its hash; the stand-in already reports the
  // registered identity, so loadScene can run unchanged.
  const session = new PhysicsSession(backend, { sessionId: 'microduck-lifecycle', observationBatchSteps: 1 });
  return { session, backend, worker };
}

// The controller loop the simulator runs, reproduced here against the real session so the
// scheduling contract is checked without a browser.
async function runControlTicks(session, controller, command, ticks, { infer = () => new Array(14).fill(0.05), onTick = null } = {}) {
  let executed = 0;
  for (let tick = 0; tick < ticks; tick += 1) {
    const observation = await session.getObservation();
    const pending = controller.beginTick({
      observation: {
        gyroRadS: observation.sensors.imu.gyroRadS,
        projectedGravity: observation.sensors.imu.projectedGravity,
        jointPositionRad: MICRODUCK_POLICY_JOINT_ORDER.map((id) => observation.joints[id].positionRad),
        jointVelocityRadS: MICRODUCK_POLICY_JOINT_ORDER.map((id) => observation.joints[id].velocityRadS),
      },
      command,
    });
    const step = controller.completeTick(pending, infer(tick), MICRODUCK_CONTROL_DECIMATION * TIMESTEP);
    await session.sendCommand({ type: 'set_joint_targets', targetsRad: { ...step.targetsRad }, firmwareGain: step.gain }, { maxSteps: MICRODUCK_CONTROL_DECIMATION });
    await session.advanceSteps(MICRODUCK_CONTROL_DECIMATION);
    executed += 1;
    if (onTick) onTick(tick, step);
  }
  return executed;
}

// ------------------------------------------------------------------------ checks
await check('the session loads the registered physical scene and publishes an IMU observation', async () => {
  const { session, worker } = await makeSession();
  await session.loadScene(MICRODUCK_WALK_SCENE);
  assert(session.robotId === MICRODUCK_WALK_PACKAGE.robotId, 'session robot id');
  assert(session.sceneRevision === MICRODUCK_WALK_SCENE.revision, 'session scene revision');
  const observation = await session.getObservation();
  assert(observation.schemaVersion === PHYSICS_BACKEND_API_VERSION, 'observation schema');
  assert(observation.sensors?.imu?.gyroRadS?.length === 3, 'the IMU gyro did not reach the observation');
  assert(observation.sensors.imu.projectedGravity[2] === -1, 'projected gravity did not reach the observation');
  assert(observation.footContacts?.floor?.left_foot_collision === true, 'named foot-floor contact did not reach the observation');
  assert(observation.actuationEnabled === true, 'the actuation flag did not reach the observation');
  assert(worker.counts('load') === 1, 'the worker was asked to load more than once');
  session.dispose();
});

await check('the control loop runs at the declared cadence and writes actuator targets only', async () => {
  const { session, worker } = await makeSession();
  await session.loadScene(MICRODUCK_WALK_SCENE);
  const controller = new MicroDuckController({ availablePolicies: MICRODUCK_PHYSICAL_POLICIES });
  const before = worker.time;
  const ticks = await runControlTicks(session, controller, makeCommand({ twist: [0.3, 0, 0] }), 5);
  assert(ticks === 5, 'not every requested tick ran');
  // Fixed simulation-time advancement: exactly decimation physics steps per controller tick.
  const advanced = worker.time - before;
  const expected = 5 * MICRODUCK_CONTROL_DECIMATION * TIMESTEP;
  assert(Math.abs(advanced - expected) < 1e-9, `advanced ${advanced}s, expected ${expected}s`);
  // Ordinary control asked the authority for joint targets and nothing else.
  const commandOps = worker.ops.filter((item) => item.op === 'command');
  assert(commandOps.length === 5, `expected five command ops, saw ${commandOps.length}`);
  for (const op of commandOps) {
    assert(op.payload.type === 'set_joint_targets', `ordinary control issued ${op.payload.type}`);
    assert(Object.keys(op.payload.targetsRad).length === 14, 'a control command did not carry fourteen joint targets');
  }
  assert(worker.counts('setup') === 0, 'ordinary control reached the declared setup path');
  session.dispose();
});

await check('a declared setup runs after a spent command budget, and is logged', async () => {
  const { session, worker } = await makeSession();
  await session.loadScene(MICRODUCK_WALK_SCENE);
  const controller = new MicroDuckController({ availablePolicies: MICRODUCK_PHYSICAL_POLICIES });
  await runControlTicks(session, controller, makeCommand({ twist: [0.3, 0, 0] }), 2);
  // The regression this check exists for: a fully spent budget is finished, not active, so a
  // declared perturbation after a run must still be possible.
  const observation = await session.applySetup({ type: 'set_actuation_enabled', enabled: false, label: 'declared torque-off condition' });
  assert(observation.actuationEnabled === false, 'the torque-off condition did not take effect');
  assert(observation.setupLog.some((item) => item.event === 'setup_actuation'), 'the torque-off condition was not logged as setup');
  const perturbed = await session.applySetup({ type: 'set_trunk_orientation', quaternionWxyz: [Math.SQRT1_2, 0, -Math.SQRT1_2, 0], label: 'declared pre-trial perturbation: face down' });
  assert(perturbed.setupLog.some((item) => item.event === 'setup_trunk_orientation'), 'the perturbation was not logged as setup');
  session.dispose();
});

await check('a setup operation outside the declared allowlist is refused', async () => {
  const { session } = await makeSession({ setupOperations: ['set_actuation_enabled'] });
  await session.loadScene(MICRODUCK_WALK_SCENE);
  let refused = false;
  try { await session.applySetup({ type: 'set_trunk_orientation', quaternionWxyz: [1, 0, 0, 0] }); }
  catch (error) { refused = /not declared by this backend/.test(error.message); }
  assert(refused, 'an undeclared setup operation was accepted');
  session.dispose();
});

await check('a backend that declares no setup path refuses setup entirely', async () => {
  const { session } = await makeSession({ setupOperations: [] });
  await session.loadScene(MICRODUCK_WALK_SCENE);
  let refused = false;
  try { await session.applySetup({ type: 'set_actuation_enabled', enabled: false }); }
  catch (error) { refused = /not declared by this backend/.test(error.message); }
  assert(refused, 'a backend with no allowlist accepted a setup operation');
  session.dispose();
});

await check('a torque-off condition removes the servo force, not the target', async () => {
  const { session } = await makeSession();
  await session.loadScene(MICRODUCK_WALK_SCENE);
  const controller = new MicroDuckController({ availablePolicies: MICRODUCK_PHYSICAL_POLICIES });
  await session.applySetup({ type: 'set_actuation_enabled', enabled: false, label: 'declared torque-off condition' });
  await runControlTicks(session, controller, makeCommand({ twist: [0.3, 0, 0] }), 2, { infer: () => new Array(14).fill(0.4) });
  const observation = await session.getObservation();

  // The controller still produces targets - a disabled actuator is not a disabled policy.
  assert(controller.lastStep.targetsRad.left_knee !== MICRODUCK_HOME_POSITION_RAD[3], 'the controller stopped producing targets');

  // What a torque-off removes is the SERVO GAIN, so the actuator produces no force. It must
  // not remove the target: these are position actuators, and commanding zero radians is a
  // full-strength command to a straight-legged pose - actuation, not its absence. Measured
  // on the real plant, that mistake left the servos working at up to 0.289 N m through a
  // trial labelled "no actuation", and settled the robot at a different tilt than a real
  // torque-off does.
  assert(observation.firmwareGain === 0, `torque-off left the firmware gain at ${observation.firmwareGain}`);
  assert(observation.appliedServoKp === 0, `torque-off left the servo stiffness at ${observation.appliedServoKp}`);
  assert(observation.actuatorForceTotalNm === 0, `torque-off still produced ${observation.actuatorForceTotalNm} N m of actuator force`);
  for (const id of MICRODUCK_POLICY_JOINT_ORDER) {
    assert(observation.joints[id].targetRad === controller.lastStep.targetsRad[id], `${id} lost the target the controller asked for under torque-off`);
  }
  assert(observation.actuationEnabled === false, 'the observation stopped reporting the torque-off condition');
  session.dispose();
});

await check('the controller gain schedule reaches the servos', async () => {
  const { session, worker } = await makeSession();
  await session.loadScene(MICRODUCK_WALK_SCENE);
  const controller = new MicroDuckController({ availablePolicies: MICRODUCK_PHYSICAL_POLICIES });

  // Walking runs at the full running gain.
  await runControlTicks(session, controller, makeCommand({ twist: [0.3, 0, 0] }), 1);
  const walking = worker.lastPayload('command');
  assert(walking.firmwareGain === MICRODUCK_NOMINAL_FIRMWARE_GAIN, `walking commanded gain ${walking.firmwareGain}`);

  // Standing runs softer, at the deployed 0.8 ratio. Before this was wired through, the
  // controller computed the softened gain, published it, and nothing acted on it: standing
  // ran 25% stiffer than the deployed robot at every tick.
  await runControlTicks(session, controller, makeCommand({ twist: [0, 0, 0] }), 1);
  const standing = worker.lastPayload('command');
  assert(standing.firmwareGain === 160, `standing commanded gain ${standing.firmwareGain}`);
  const observation = await session.getObservation();
  assert(Math.abs(observation.appliedServoKp - 0.44) < 1e-12, `standing applied kp ${observation.appliedServoKp}`);
  assert(Math.abs(microDuckKpForFirmwareGain(MICRODUCK_NOMINAL_FIRMWARE_GAIN) - 0.55) < 1e-12, 'the running gain no longer maps to the identified stiffness');
  session.dispose();
});

await check('the command budget bounds how far one command may run', async () => {
  const { session } = await makeSession();
  await session.loadScene(MICRODUCK_WALK_SCENE);
  await session.sendCommand({ type: 'set_joint_targets', targetsRad: { left_knee: 0 } }, { maxSteps: MICRODUCK_CONTROL_DECIMATION });
  let exceeded = false;
  try { await session.advanceSteps(MICRODUCK_CONTROL_DECIMATION + 1); }
  catch (error) { exceeded = /exceeds remaining command budget/.test(error.message); }
  assert(exceeded, 'a step request beyond the command budget was accepted');
  session.dispose();
});

await check('pause freezes simulated time and resume restores it', async () => {
  const { session, worker } = await makeSession();
  await session.loadScene(MICRODUCK_WALK_SCENE);
  await session.pause();
  const frozen = worker.time;
  await session.advanceSteps(MICRODUCK_CONTROL_DECIMATION);
  assert(worker.time === frozen, 'a paused authority advanced simulated time');
  await session.resume();
  await session.advanceSteps(MICRODUCK_CONTROL_DECIMATION);
  assert(worker.time > frozen, 'a resumed authority did not advance simulated time');
  session.dispose();
});

await check('a cancelled run stops the authority and rejects further work', async () => {
  const { session } = await makeSession();
  await session.loadScene(MICRODUCK_WALK_SCENE);
  const result = await session.cancelRun('cancelled');
  assert(result.cancelled === true && result.reloadRequired === true, 'cancelRun did not report a reload requirement');
  let rejected = false;
  try { await session.advanceSteps(1); }
  catch (error) { rejected = /No physical scene loaded|No MuJoCo scene is loaded/.test(error.message); }
  assert(rejected, 'a cancelled session still accepted work');
  session.dispose();
});

await check('a stale epoch cannot reach the authority', async () => {
  const { session, backend } = await makeSession();
  await session.loadScene(MICRODUCK_WALK_SCENE);
  const staleContext = { sessionId: session.sessionId, epoch: session.epoch };
  await session.reset();
  let rejected = false;
  try { await backend.advanceSteps(1, staleContext); }
  catch (error) { rejected = /Stale physics epoch/.test(error.message); }
  assert(rejected, 'a stale epoch reached the authority');
  let foreignRejected = false;
  try { await backend.advanceSteps(1, { sessionId: 'someone-else', epoch: session.epoch }); }
  catch (error) { foreignRejected = /Stale or foreign physics session/.test(error.message); }
  assert(foreignRejected, 'a foreign session reached the authority');
  session.dispose();
});

await check('a reset restores the declared initial state and clears the controller feedback', async () => {
  const { session, worker } = await makeSession();
  await session.loadScene(MICRODUCK_WALK_SCENE);
  const controller = new MicroDuckController({ availablePolicies: MICRODUCK_PHYSICAL_POLICIES });
  await runControlTicks(session, controller, makeCommand({ twist: [0.3, 0, 0] }), 3, { infer: () => new Array(14).fill(0.3) });
  assert(controller.previousRawAction.some((value) => value !== 0), 'the controller carried no feedback state to clear');
  await session.reset();
  controller.reset();
  assert(worker.time === 0, 'reset did not restore simulated time');
  assert(controller.previousRawAction.every((value) => value === 0), 'reset left a stale previous action');
  assert(controller.previousTargets === null, 'reset left a stale filter anchor');
  const observation = await session.getObservation();
  assert(observation.setupLog.some((item) => item.event === 'reset'), 'reset was not logged');
  session.dispose();
});

await check('every published sample reaches the evaluator, so a short contact cannot be missed', async () => {
  const { session } = await makeSession();
  const evaluator = new MicroDuckPhysicalEvaluator({ commandedTwist: [0.3, 0, 0] });
  session.subscribe((event) => evaluator.observe(event.observation));
  await session.loadScene(MICRODUCK_WALK_SCENE);
  const controller = new MicroDuckController({ availablePolicies: MICRODUCK_PHYSICAL_POLICIES });
  await runControlTicks(session, controller, makeCommand({ twist: [0.3, 0, 0] }), 4);
  const report = evaluator.report();
  // Four controller ticks at decimation 4, published every physics step, plus the
  // command/observation publications around them.
  assert(report.samples >= 4 * MICRODUCK_CONTROL_DECIMATION, `only ${report.samples} samples reached the evaluator`);
  assert(report.complete === true, 'the evaluator produced no report');
  session.dispose();
});

await check('a disposed session releases the worker and refuses further work', async () => {
  const { session, worker } = await makeSession();
  await session.loadScene(MICRODUCK_WALK_SCENE);
  session.dispose();
  assert(worker.terminated === true, 'dispose left the worker running');
  let refused = false;
  try { await session.getObservation(); }
  catch (error) { refused = /disposed/.test(error.message); }
  assert(refused, 'a disposed session still accepted work');
});

// -------------------------------------------------- the simulator's own control loop
// The class the workspace actually runs, driven headlessly: the same session and controller
// as above, but through MicroDuckPhysicalSimulator's orchestration rather than a hand-rolled
// loop. It needs two injection seams - a backend and a policy runtime - and nothing else.
function headlessSimulator({ infer = () => new Float32Array(14).fill(0.05) } = {}) {
  const workers = [];
  const backendFactory = ({ setupOperations }) => {
    const worker = new ScriptedWorker();
    workers.push(worker);
    const backend = new BrowserMuJoCoBackend({ workerUrl: 'about:blank', setupOperations });
    backend.worker = worker;
    worker.onmessage = ({ data }) => {
      const request = backend.pending.get(data?.id);
      if (!request) return;
      backend.pending.delete(data.id);
      if (data.ok) request.resolve(data.payload); else request.reject(new Error(data.error || 'worker failed'));
    };
    return backend;
  };
  const policyRuntime = { infer: async (policyId, observation) => infer(policyId, observation), dispose: () => {} };
  return { backendFactory, policyRuntime, workers };
}

await check('the simulator control loop advances the authority at the declared cadence', async () => {
  const seams = headlessSimulator();
  const sim = await MicroDuckPhysicalSimulator.create({ packageKey: 'walk', ...seams });
  const worker = seams.workers[0];
  sim.setCommand({ vx: 0.35 });
  const before = worker.time;
  const run = await sim.advanceSeconds(0.4);
  assert(run.executedTicks === 20, `expected 20 controller ticks, ran ${run.executedTicks}`);
  assert(run.completed === true, 'the bounded advance did not complete');
  const advanced = worker.time - before;
  const expected = 20 * MICRODUCK_CONTROL_DECIMATION * TIMESTEP;
  assert(Math.abs(advanced - expected) < 1e-9, `advanced ${advanced}s, expected ${expected}s`);
  // The command reached the controller, and the controller reached the authority.
  assert(sim.getState().controller.policyId === 'walking', 'the walking policy was not selected');
  assert(worker.counts('command') === 20, `expected 20 authority commands, saw ${worker.counts('command')}`);
  assert(worker.lastPayload('command').type === 'set_joint_targets', 'ordinary control wrote something other than joint targets');
  sim.dispose();
});

await check('the simulator bounds one advance and refuses an over-long one', async () => {
  const seams = headlessSimulator();
  const sim = await MicroDuckPhysicalSimulator.create({ packageKey: 'walk', ...seams });
  let refused = false;
  try { await sim.advanceSeconds(30); } catch (error) { refused = /bounded to/.test(error.message); }
  assert(refused, 'an unbounded advance was accepted');
  sim.dispose();
});

await check('cancelling mid-run stops the simulator loop without writing further targets', async () => {
  let ticks = 0;
  const seams = headlessSimulator({
    infer: () => { ticks += 1; return new Float32Array(14).fill(0.05); },
  });
  const sim = await MicroDuckPhysicalSimulator.create({ packageKey: 'walk', ...seams });
  const worker = seams.workers[0];
  sim.setCommand({ vx: 0.35 });
  // Cancel from inside the loop, the way a human Stop or a workspace switch would.
  const run = await sim.advanceSeconds(1.0, { onTick: ({ tick }) => { if (tick === 4) sim.cancel('user-stop'); } });
  assert(run.cancelled === true, 'the run did not report being cancelled');
  assert(run.completed === false, 'a cancelled run reported completion');
  assert(run.executedTicks <= 6, `a cancelled run executed ${run.executedTicks} ticks`);
  const commandsAtCancel = worker.counts('command');
  // Nothing further reaches the authority afterwards.
  const after = await sim.advanceSeconds(0.2);
  assert(after.executedTicks === 0, 'a cancelled simulator still executed controller ticks');
  assert(worker.counts('command') === commandsAtCancel, 'a cancelled simulator still wrote actuator targets');
  sim.dispose();
});

await check('the simulator refuses unsupported capabilities and routes them nowhere', async () => {
  const seams = headlessSimulator();
  const sim = await MicroDuckPhysicalSimulator.create({ packageKey: 'walk', ...seams });
  const worker = seams.workers[0];
  const before = worker.counts('command');
  for (const skill of ['roller', 'roller_crouch']) {
    const result = sim.requestSkill(skill);
    assert(result.accepted === false, `${skill} was accepted`);
    assert(result.status === 'unsupported', `${skill} did not report unsupported`);
  }
  assert(worker.counts('command') === before, 'a refused capability still reached the authority');
  const wrongPlant = sim.requestSkill('kick_right');
  assert(wrongPlant.accepted === false && wrongPlant.status === 'wrong-plant', 'kick was not rejected on the walking collision plant');
  assert(wrongPlant.requiredPackageKeys?.includes('kick'), 'wrong-plant response did not identify the required kick plant');
  for (const alias of ['sit', 'stand_up']) {
    const aliasResult = sim.requestSkill(alias);
    assert(aliasResult.accepted === false && aliasResult.status === 'wrong-plant', `${alias} bypassed the ground-contact plant gate`);
    assert(aliasResult.requiredPackageKeys?.includes('groundContact'), `${alias} did not identify the required ground-contact plant`);
  }
  assert(worker.counts('command') === before, 'a wrong-plant capability reached the authority');
  sim.dispose();
});

await check('a declared perturbation resets the controller feedback and is logged', async () => {
  const seams = headlessSimulator({ infer: () => new Float32Array(14).fill(0.3) });
  const sim = await MicroDuckPhysicalSimulator.create({ packageKey: 'walk', ...seams });
  sim.setCommand({ vx: 0.35 });
  await sim.advanceSeconds(0.2);
  assert(sim.controller.previousRawAction.some((value) => value !== 0), 'no controller feedback state to clear');
  await sim.applyPerturbation('face_down');
  // A perturbation begins a new trial: the previous trial's action must not carry into it.
  assert(sim.controller.previousRawAction.every((value) => value === 0), 'a perturbation left a stale previous action');
  assert(sim.controller.previousTargets === null, 'a perturbation left a stale filter anchor');
  const state = sim.getState();
  assert(state.actual.setupLog.some((item) => item.event === 'setup_trunk_orientation'), 'the perturbation was not logged as setup');
  // And the declared torque-off condition works after a run, which is the sequence the
  // recovery negative control needs.
  await sim.setActuationEnabled(false);
  assert(sim.getState().actual.actuationEnabled === false, 'the torque-off condition did not take effect after a run');
  sim.dispose();
});

await check('the simulator reports requested, controller and actual state separately', async () => {
  const seams = headlessSimulator();
  const sim = await MicroDuckPhysicalSimulator.create({ packageKey: 'walk', ...seams });
  const accepted = sim.setCommand({ vx: 9 });
  assert(accepted.limitedBy.includes('vx'), 'an out-of-range command was not reported as limited');
  await sim.advanceSeconds(0.1);
  const state = sim.getState();
  assert(state.requested.command.twist[0] === 0.4, 'the requested command was not clamped to the declared bound');
  assert(state.controller.policyId === 'walking' && state.controller.actionScale === 0.9, 'the controller view is missing');
  assert(Array.isArray(state.actual.trunkPositionM), 'the actual view is missing');
  assert(state.capability.backend === 'browser-mujoco' && state.hardwareValidated === false, 'the capability label is wrong');
  // The three views are distinct objects, so a caller cannot mistake one for another.
  assert(state.requested.command.twist[0] !== state.actual.trunkPositionM[0], 'request and measurement are indistinguishable in this fixture');
  sim.dispose();
});

console.log(`\nMicroDuck Phase 5C lifecycle: ${passed} passed, ${failures.length} failed`);
if (failures.length) { console.error(`failed: ${failures.join(', ')}`); process.exit(1); }
