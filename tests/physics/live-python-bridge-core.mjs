import assert from 'node:assert/strict';
import { LIVE_SIM_API_VERSION, LivePythonBridge } from '../../src/runtime/live-python-bridge.js';

class FakeSession {
  constructor(timestepSeconds, { robotId = 'so101_follower' } = {}) {
    this.timestepSeconds = timestepSeconds;
    this.robotId = robotId;
    this.epoch = 1;
    this.sceneRevision = 'scene-v1';
    this.sessionId = 'fake-session';
    this.calls = [];
    this.position = 0;
    this.target = 0;
    this.time = 0;
    this.paused = false;
  }
  async getDiagnostics() {
    this.calls.push(['getDiagnostics']);
    return { loaded: true, backend: 'browser-mujoco', sessionId: this.sessionId, epoch: this.epoch, sceneRevision: this.sceneRevision, robotId: this.robotId, timestepSeconds: this.timestepSeconds, modelPackageId: 'fake-model' };
  }
  async advanceSteps(steps) {
    this.calls.push(['advanceSteps', steps]);
    if (!this.paused) {
      this.time += steps * this.timestepSeconds;
      const direction = Math.sign(this.target - this.position);
      this.position += direction * Math.min(Math.abs(this.target - this.position), steps * 0.001);
    }
    return this.observation();
  }
  observation() {
    return { simulationTimeSeconds: this.time, view: 'ground_truth', joints: { shoulder_pan: { positionRad: this.position, velocityRadS: 0, targetRad: this.target } } };
  }
  async getObservation(options) { this.calls.push(['getObservation', options]); return { ...this.observation(), view: options.view }; }
  async sendCommand(command, options) {
    this.calls.push(['sendCommand', command, options]);
    assert.equal(command.type, 'set_joint_targets');
    this.target = Number(command.targetsRad.shoulder_pan ?? this.target);
    return { status: 'accepted', commandId: options.commandId || 'generated', remainingSteps: options.maxSteps, observation: this.observation() };
  }
  async pause() { this.calls.push(['pause']); this.paused = true; return this.observation(); }
  async resume() { this.calls.push(['resume']); this.paused = false; return this.observation(); }
  async reset() { this.calls.push(['reset']); this.epoch += 1; this.position = 0; this.target = 0; this.time = 0; return this.observation(); }
  async cancelRun(reason) { this.calls.push(['cancelRun', reason]); this.epoch += 1; return { cancelled: true, reloadRequired: true }; }
}

const phase1 = new FakeSession(0.002);
const bridge1 = new LivePythonBridge(phase1);
const connection1 = await bridge1.connect('so101_follower');
assert.equal(connection1.apiVersion, LIVE_SIM_API_VERSION);
assert.equal(connection1.timestepSeconds, 0.002);
await bridge1.advance(0.1);
assert.deepEqual(phase1.calls.at(-1), ['advanceSteps', 50], 'model timestep must determine physics step count');

const so101 = new FakeSession(0.005);
const bridge2 = new LivePythonBridge(so101);
await bridge2.connect('so101_follower');
const accepted = await bridge2.sendAction({ shoulder_pan: 0.4 }, { commandId: 'p3-action', maxSteps: 200 });
assert.equal(accepted.status, 'accepted');
assert.equal(accepted.acceptedTargetsRad.shoulder_pan, 0.4);
assert.notEqual(accepted.observation.joints.shoulder_pan.positionRad, accepted.acceptedTargetsRad.shoulder_pan, 'acceptance must not impersonate achievement');
assert.deepEqual(so101.calls.at(-1), ['sendCommand', { type: 'set_joint_targets', targetsRad: { shoulder_pan: 0.4 } }, { commandId: 'p3-action', maxSteps: 200 }]);

const cadence = await bridge2.advanceControllerInterval(0.02);
assert.equal(cadence.physicsSteps, 4, '20 ms controller interval must be four 5 ms physics steps');
assert.equal(cadence.timestepSeconds, 0.005);
assert.equal(cadence.observation.simulationTimeSeconds, 0.02);
await bridge2.advance(0.23);
assert.deepEqual(so101.calls.at(-1), ['advanceSteps', 46]);
await bridge2.advance(0);
assert.deepEqual(so101.calls.at(-1), ['getObservation', { view: 'ground_truth' }], 'zero seconds must not consume a physics step');
await assert.rejects(() => bridge2.advance(0.012), /integer number of 0.005s physics steps/);
await assert.rejects(() => bridge2.getObservation({ view: 'sensor' }), (error) => error.code === 'UNSUPPORTED_OBSERVATION_PROFILE');
await assert.rejects(() => bridge2.sendAction({ 'shoulder_pan.pos': 10 }), /not legacy \.pos fields/);

const impeded = new FakeSession(0.005);
const bridge3 = new LivePythonBridge(impeded);
await bridge3.connect('so101_follower');
await bridge3.sendAction({ shoulder_pan: 0.6 }, { maxSteps: 100 });
const result = await bridge3.waitForGoal({ shoulder_pan: 0.6 }, { toleranceRad: 0.01, timeoutSeconds: 0.04, controllerPeriodSeconds: 0.02 });
assert.equal(result.status, 'timeout', 'bounded controller must report timeout when actual state cannot reach target');
assert.ok(Math.abs(result.errorsRad.shoulder_pan) > 0.5, 'actual state must remain distinct from requested target in impeded case');
assert.equal(result.elapsedSimulationSeconds, 0.04);

const beforePause = await bridge3.getObservation();
await bridge3.pause();
const paused = await bridge3.advance(0.02);
assert.equal(paused.simulationTimeSeconds, beforePause.simulationTimeSeconds, 'pause must freeze simulation time');
await bridge3.resume();
const resumed = await bridge3.advance(0.02);
assert.equal(resumed.simulationTimeSeconds, beforePause.simulationTimeSeconds + 0.02);

const stale = new FakeSession(0.005);
const bridge4 = new LivePythonBridge(stale);
await bridge4.connect('so101_follower');
stale.epoch += 1;
await assert.rejects(() => bridge4.getObservation(), (error) => error.code === 'STALE_LIVE_SESSION');

const resetSession = new FakeSession(0.005);
const bridge5 = new LivePythonBridge(resetSession);
await bridge5.connect('so101_follower');
const reset = await bridge5.reset();
assert.equal(reset.connection.epoch, 2, 'explicit reset by the owning run adopts the new session epoch');
await bridge5.getObservation();

const invalid = new FakeSession(null);
const invalidBridge = new LivePythonBridge(invalid);
await assert.rejects(() => invalidBridge.connect(), /identified model timestep/);

console.log('Live physical Python API timing/ownership checks: OK');
