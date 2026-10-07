import assert from 'node:assert/strict';
import {readFile,writeFile} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import {ASIMOV_SENSOR_PACKAGES,ASIMOV_PACKAGES} from '../../src/physics/asimov-model-package.js';
const workerUrl=new URL('../../src/physics/asimov-mujoco-worker.js',import.meta.url);
globalThis.self={location:workerUrl};let reply;globalThis.postMessage=v=>{reply=v};
globalThis.fetch=async input=>new Response(await readFile(fileURLToPath(new URL(input))),{status:200});
await import(workerUrl.href);let seq=0;
async function rpc(op,payload){await self.onmessage({data:{id:++seq,op,payload}});assert.equal(reply.id,seq);if(!reply.ok)throw new Error(reply.error);return reply.payload;}
const reports={};
for(const [key,pkg] of Object.entries(ASIMOV_SENSOR_PACKAGES)) {
 const initial=await rpc('load',{modelPackage:pkg});
 assert.equal(initial.sensorObservation.profileId,'asimov-body-sensors-v2');
 assert.equal(initial.sensorObservation.imu.valid,false);
 assert.equal(initial.sensorObservation.imu.frame,'pelvis_link');
 const before=await rpc('observe'); for(let i=0;i<5;i++)assert.deepEqual((await rpc('observe')).sensorObservation,before.sensorObservation);
 await assert.rejects(rpc('command',{type:'engage_stand',controllerId:'asimov-stance-feedback-v1'}));
 await rpc('command',{type:'engage_stand',controllerId:pkg.standingControllerId});
 const trace=[];
 for(let i=0;i<120;i++) trace.push(await rpc('step',{count:40}));
 const final=trace.at(-1);
 assert.equal(final.standingAssessment.sensorFeedback.groundTruthFallback,false);
 assert.equal(final.standingAssessment.sensorFeedback.status,'active');
 for(const o of trace) {
   assert.equal(o.standingAssessment.controllerId,'asimov-sensor-stance-v2');
   assert.equal('angularVelocityWorldRadS' in o.sensorObservation.imu,false);
   assert.ok(Math.abs(Math.hypot(...o.sensorObservation.imu.projectedGravityBase)-1)<1e-10);
   for(const j of o.actuatorModel.joints) {assert.ok(Number.isFinite(j.motorNm));assert.ok(Math.abs(j.motorNm)<=j.effectiveLimitNm+1e-9);}
 }
 // Nominal must pass. Stress is evidence, not a test forced to pass.
 if(key==='sensor-standing') assert.equal(final.standingAssessment.status,'passed',JSON.stringify(final.standingAssessment));
 if(key.endsWith('ankle-stress')) {
   assert.equal(final.actuatorModel.ankleStress.torqueScale,.7);
   assert.ok(trace.some(o=>[4,5,10,11].some(i=>Math.abs(o.actuatorModel.joints[i].frictionNm)>1e-6)));
 }
 console.log(key,JSON.stringify(final.standingAssessment));
 reports[key]={initial,final,trace};
 await rpc('pause');await rpc('step',{count:40});assert.deepEqual((await rpc('observe')).sensorObservation,final.sensorObservation);await rpc('resume');
 await rpc('setup',{type:'set_actuation',enabled:false});const off=await rpc('step',{count:40});assert.equal(off.standingAssessment.status,'failed');
 assert.ok(Object.values(off.joints).every(j=>j.effortNm===0));
 const reset=await rpc('reset');assert.deepEqual(reset.sensorObservation,initial.sensorObservation);assert.deepEqual(reset.standingAssessment,initial.standingAssessment);
 await rpc('command',{type:'engage_stand',controllerId:pkg.standingControllerId});
 const repeat=await rpc('step',{count:40});assert.deepEqual(repeat.sensorObservation,trace[0].sensorObservation);
 assert.deepEqual(repeat.root,trace[0].root);
 await assert.rejects(rpc('command',{type:'set_standing_targets',targetsRad:{left_knee_joint:.1}}));
 await rpc('command',{type:'set_standing_targets',targetsRad:{left_elbow_joint:1,right_elbow_joint:-1}});
 const upper=await rpc('step',{count:80});assert.equal(upper.controller.id,pkg.standingControllerId);assert.equal(upper.joints.left_elbow_joint.acceptedTargetRad,1);
 await rpc('command',{type:'set_joint_targets',targetsRad:{left_elbow_joint:1}});
 assert.equal((await rpc('observe')).standingAssessment.status,'failed','manual control releases standing, not covertly combined');
 await assert.rejects(rpc('command',{type:'engage_stand',controllerId:pkg.standingControllerId}),/Reset/);
}
await rpc('load',{modelPackage:ASIMOV_PACKAGES.mounted});assert.equal((await rpc('observe')).sensorObservation,undefined);
await assert.rejects(rpc('load',{modelPackage:{...ASIMOV_SENSOR_PACKAGES['sensor-standing'],sensitivityProfileId:'invented'}}));
await rpc('dispose');
if(process.env.ASIMOV_SENSOR_REPORT) await writeFile(process.env.ASIMOV_SENSOR_REPORT,JSON.stringify(reports));
console.log('Sensor-standing shipped WASM: deterministic trials, original isolation, motor caps, lifecycle and explicit stress outcomes: OK');
