import assert from 'node:assert/strict';
import {readFile,writeFile} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import {ASIMOV_ACTUATOR_PACKAGES,ASIMOV_PACKAGES} from '../../src/physics/asimov-model-package.js';
const workerUrl=new URL('../../src/physics/asimov-mujoco-worker.js',import.meta.url);
globalThis.self={location:workerUrl};let reply;globalThis.postMessage=v=>{reply=v};
globalThis.fetch=async input=>new Response(await readFile(fileURLToPath(new URL(input))),{status:200});
await import(workerUrl.href);let seq=0;
async function rpc(op,payload){await self.onmessage({data:{id:++seq,op,payload}});assert.equal(reply.id,seq);if(!reply.ok)throw new Error(reply.error);return reply.payload;}
const reports={};
for(const [variant,pkg] of Object.entries(ASIMOV_ACTUATOR_PACKAGES)){
 const initial=await rpc('load',{modelPackage:pkg});assert.equal(initial.engine.timestepSeconds,.0025);assert.equal(initial.actuatorModel.controlTicks,0);
 assert.equal(initial.root.free,variant!=='actuator-mounted');
 if(variant==='standing') await rpc('command',{type:'engage_stand',controllerId:pkg.standingControllerId});
 else {
  await assert.rejects(rpc('command',{type:'engage_stand',controllerId:'asimov-stance-feedback-v1'}));
  await rpc('command',{type:'set_joint_targets',targetsRad:{left_elbow_joint:1,right_elbow_joint:-1}});
 }
 const before=await rpc('observe');for(let i=0;i<10;i++)assert.deepEqual((await rpc('observe')).sensorObservation,before.sensorObservation);
 await assert.rejects(rpc('command',{type:'set_lowlevel_targets',commands:{left_elbow_joint:{kp:Infinity}}}));
 assert.deepEqual((await rpc('observe')).controller,before.controller);
 const trajectory=[]; const seconds=variant==='standing'?12:2;
 for(let i=0;i<seconds*10;i++)trajectory.push(await rpc('step',{count:40}));
 const final=trajectory.at(-1);console.log(variant,final.root.positionM,final.standingAssessment);
 if(variant==='standing')assert.equal(final.standingAssessment.status,'passed');
 if(variant==='actuator-mounted')assert.ok(Math.abs(final.joints.left_elbow_joint.positionRad-1)<.1);
 for(const sample of trajectory){
  assert.ok(Object.values(sample.joints).every(j=>Number.isFinite(j.positionRad)));
  for(const j of sample.actuatorModel.joints){assert.ok(Math.abs(j.motorNm)<=j.effectiveLimitNm+1e-10);}
 }
 reports[variant]={initial,final,trajectory};
 await rpc('pause');await rpc('step',{count:40});assert.deepEqual((await rpc('observe')).sensorObservation,final.sensorObservation);await rpc('resume');
 await rpc('setup',{type:'set_actuation',enabled:false});const off=await rpc('step',{count:40});assert.ok(Object.values(off.joints).every(j=>j.effortNm===0));
 if(variant==='standing')assert.equal(off.standingAssessment.status,'failed');
 const reset=await rpc('reset');assert.deepEqual(reset.sensorObservation,initial.sensorObservation);assert.deepEqual(reset.actuatorModel,initial.actuatorModel);
}
await rpc('load',{modelPackage:ASIMOV_PACKAGES.freebase});await assert.rejects(rpc('command',{type:'engage_stand',controllerId:'asimov-stance-feedback-v1'}));assert.equal((await rpc('observe')).sensorObservation,undefined);
await rpc('dispose');
if(process.env.ASIMOV_ACTUATOR_REPORT)await writeFile(process.env.ASIMOV_ACTUATOR_REPORT,JSON.stringify(reports));
console.log('Asimov experimental actual shipped WASM: three scenes, bounded motor laws, sensor read independence, standing, off/reset, reference isolation: OK');
