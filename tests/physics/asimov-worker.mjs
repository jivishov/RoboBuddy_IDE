import assert from 'node:assert/strict';
import {readFile,writeFile} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import {ASIMOV_PACKAGES} from '../../src/physics/asimov-model-package.js';
import {ASIMOV_JOINT_ORDER} from '../../src/physics/asimov-controller.js';
const workerUrl=new URL('../../src/physics/asimov-mujoco-worker.js',import.meta.url);
globalThis.self={location:workerUrl}; let reply;
globalThis.postMessage=value=>{reply=value};
let corrupt=false;
globalThis.fetch=async input=>{let bytes=await readFile(fileURLToPath(new URL(input))); if(corrupt) bytes=Buffer.concat([bytes,Buffer.from(' ')]); return new Response(bytes,{status:200});};
await import(workerUrl.href); let seq=0;
async function rpc(op,payload){await self.onmessage({data:{id:++seq,op,payload}}); assert.equal(reply.id,seq); if(!reply.ok)throw new Error(reply.error);return reply.payload;}
const report={runtime:'shipped MuJoCo WASM 3.11.0',variants:{}};
for(const [variant,pkg] of Object.entries(ASIMOV_PACKAGES)){
 const initial=await rpc('load',{modelPackage:pkg}); assert.equal(Object.keys(initial.joints).length,23);
 assert.equal(initial.root.free,variant!=='mounted');
 for(const command of [
   {type:'set_joint_targets',targetsRad:{left_elbow_joint:NaN}},
   {type:'set_joint_targets',targetsRad:{left_elbow_joint:'1'}},
   {type:'set_joint_targets',targetsRad:{neck_yaw_joint:.2}},
   {type:'set_lowlevel_targets',commands:{left_elbow_joint:{kp:Infinity}}},
   {type:'set_lowlevel_targets',commands:{left_elbow_joint:{wrong:1}}},
   {type:'engage_stand'},
 ]) {const before=await rpc('observe');await assert.rejects(rpc('command',command));assert.deepEqual((await rpc('observe')).joints,before.joints);}
 await assert.rejects(rpc('setup',{type:'set_actuation',enabled:'false'}));
 let motion;
 if(variant==='mounted'){
   await rpc('command',{type:'set_joint_targets',targetsRad:{left_elbow_joint:1.0,right_elbow_joint:-1.0}});
   motion=await rpc('stepSampled',{count:400,sampleEverySteps:4});
   const last=motion.observations.at(-1);
   assert.ok(Math.abs(last.joints.left_elbow_joint.positionRad-1)<.10,`mounted elbow=${last.joints.left_elbow_joint.positionRad}`);
   assert.deepEqual(last.root.positionM,initial.root.positionM);
 }else{
   motion=await rpc('stepSampled',{count:variant==='drop'?30:400,sampleEverySteps:4});
   if(variant==='drop') assert.ok(motion.observations.at(-1).root.positionM[2]<initial.root.positionM[2]-.05);
 }
 for(const sample of motion.observations){
  for(const j of Object.values(sample.joints)){assert.ok(Number.isFinite(j.positionRad)); assert.ok(Math.abs(j.effortNm)<=j.effortLimitNm+1e-8);}
  assert.equal(sample.contactsReadable,true);
 }
 const end=motion.observations.at(-1);
 report.variants[variant]={initial,final:end,trajectory:motion.observations};
 await rpc('pause');const before=await rpc('observe');await rpc('step',{count:20});assert.equal((await rpc('observe')).simulationTime,before.simulationTime);
 await assert.rejects(rpc('step',{count:-1}));await rpc('resume');
 await rpc('reset');const reset=await rpc('observe');assert.deepEqual(reset.root,initial.root);assert.deepEqual(reset.joints,initial.joints);
 await rpc('setup',{type:'set_actuation',enabled:false});const off=await rpc('step',{count:10});assert.ok(Object.values(off.joints).every(j=>j.effortNm===0));
}
corrupt=true;await assert.rejects(rpc('load',{modelPackage:ASIMOV_PACKAGES.freebase}),/SHA-256/);assert.equal((await rpc('observe')).model,null);await rpc('dispose');
const path=process.env.ASIMOV_WASM_REPORT;
if(path)await writeFile(path,JSON.stringify(report));
console.log(JSON.stringify({pass:true,variants:Object.fromEntries(Object.entries(report.variants).map(([k,v])=>[k,{rootHeight:v.final.root.positionM[2],elbow:v.final.joints.left_elbow_joint.positionRad,contacts:v.final.contactCount}]))}));
