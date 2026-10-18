import assert from 'node:assert/strict';
import {JOINTS,visualPose} from '../../src/hardware/joint-map.js';
import {helperAddress} from '../../src/hardware/bridge-client.js';
const state={fresh:true,calibrated:true,joints:Object.fromEntries(JOINTS.map(j=>[j,{position:j==='gripper'?50:10}]))};
assert.equal(visualPose(state)['shoulder_pan.pos'],10);
assert.equal(visualPose(state,{shoulder_pan:{sign:-1,offset:30}})['shoulder_pan.pos'],20);
assert.equal(visualPose(state)['gripper.pos'],50);
assert.equal(visualPose({...state,fresh:false}),null);
assert.equal(visualPose({...state,calibrated:false}),null);
assert.throws(()=>visualPose(state,{elbow_flex:{sign:0,offset:0}}));
assert.equal(helperAddress('http://127.0.0.1:8765/'),'http://127.0.0.1:8765');
for(const value of ['http://evil.example','http://127.0.0.1@evil.example','file:///tmp/x','http://localhost:8765/path','http://localhost:8765/?token=secret'])assert.throws(()=>helperAddress(value));
console.log('Hardware visual mapping, stale-state isolation and loopback validation passed.');

