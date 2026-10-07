import assert from 'node:assert/strict';
import { LivePythonBridge } from '../../src/runtime/live-python-bridge.js';

// Delay a sensor reply across reset/reconnect without replacing the bridge object.
// No old observation may be stamped with the new session owner's epoch.
class SensorSession {
  constructor() { this.epoch = 1; this.pending = null; }
  async getDiagnostics() {
    return { loaded:true, sessionId:'sensor-session', epoch:this.epoch,
      sceneRevision:'actuator-v1', robotId:'asimov_1_23dof_physical',
      timestepSeconds:.0025, modelPackageId:'asimov-1-actuator-mounted-v1' };
  }
  async reset() { this.epoch++; return { reset:true }; }
  getObservation() {
    const epoch = this.epoch;
    return new Promise(resolve => { this.pending = () => resolve({ sessionId:'sensor-session', epoch,
      sensorObservation:{ view:'hardware_like', simulationTimeSeconds:0, joints:{} } }); });
  }
}
async function pendingRead(bridge,session) {
  const read = bridge.getObservation({ view:'hardware_like' });
  // Install the rejection handler immediately, then wait for the backend read.
  const outcome = read.then(value => ({value}), error => ({error}));
  while (!session.pending) await new Promise(resolve => setImmediate(resolve));
  return { outcome, finish:session.pending };
}
for (const operation of ['reset','reconnect','disconnect','dispose','external-reset']) {
  const session = new SensorSession(), bridge = new LivePythonBridge(session);
  await bridge.connect();
  const {outcome,finish} = await pendingRead(bridge,session);
  if (operation === 'reset') await bridge.reset();
  else if (operation === 'reconnect') { session.epoch++; await bridge.connect(); }
  else if (operation === 'disconnect') bridge.disconnect();
  else if (operation === 'dispose') bridge.dispose();
  else session.epoch++;
  finish();
  const result = await outcome;
  assert.ok(result.error, `sensor read across ${operation} must reject`);
}
const session = new SensorSession(), bridge = new LivePythonBridge(session);
await bridge.connect();
const {outcome,finish} = await pendingRead(bridge,session); finish();
const {value,error} = await outcome;
assert.equal(error,undefined); assert.equal(value.epoch,1); assert.equal(value.sessionId,'sensor-session');
assert.equal(value.view,'hardware_like');
console.log('Asimov sensor owner: reset, reconnect, disconnect, dispose and external reset reject stale observations; current reads pass');
