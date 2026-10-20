import assert from 'node:assert/strict';
import {simulatorGestureAdapter} from '../src/gestures/simulator-adapter.js';
import {GESTURE_JOINTS} from '../src/gestures/gesture-core.js';
const calls=[],token={sessionId:'test-so101',epoch:1};
const state={observation:{joints:Object.fromEntries(Object.keys(GESTURE_JOINTS).map(j=>[j,{positionRad:.1}]))}};
let pauses=0;
const app={profileId:'so101',workspaceStatus:'ready',executionState:'idle',getAgentAccess:()=> 'off',updateExecutionControls(){},sim:{
  isReady:()=>true,getState:()=>structuredClone(state),getPhysicalAuthorityToken:()=>({...token}),resume:async()=>{},pause:async()=>{pauses++;},
  applyPhysicalTargets:async(targets,options)=>{calls.push({targets,options});},
}};
const adapter=simulatorGestureAdapter(app);
await adapter.begin('shoulder_pan');
assert.deepEqual(calls[0].targets,Object.fromEntries(Object.keys(GESTURE_JOINTS).map(j=>[j,.1])),'Takeover holds every measured joint');
assert.equal(calls[0].options.advanceSeconds,0,'Takeover does not advance old program targets');
await adapter.send('shoulder_pan',1);
assert.ok(calls.at(-1).targets.shoulder_pan>.1 && calls.at(-1).targets.shoulder_pan<=.1+Math.PI/360);
state.observation.joints.shoulder_pan.positionRad=.105;
await adapter.send('shoulder_pan',0);
const hold=calls.at(-1).targets.shoulder_pan;
state.observation.joints.shoulder_pan.positionRad=.102; // Physics/load can deflect the joint.
await adapter.send('shoulder_pan',0);
assert.equal(calls.at(-1).targets.shoulder_pan,hold,'Neutral must not follow subsequent deflection');
assert.equal(state.observation.joints.shoulder_pan.positionRad,.102,'Adapter must never assign authoritative joint state');
await assert.rejects(adapter.send('elbow_flex',1),/Invalid gesture input/);
await assert.rejects(adapter.send('shoulder_pan',NaN),/Invalid gesture input/);
await assert.rejects(adapter.send('shoulder_pan',2),/Invalid gesture input/);
token.epoch++;
await assert.rejects(adapter.send('shoulder_pan',1),/Simulator control changed/);
await adapter.end();
assert.equal(pauses,0,'An old gesture owner must not pause a replacement authority');
console.log('Measured actuator takeover, latched neutral, bounded input and authority isolation passed.');
