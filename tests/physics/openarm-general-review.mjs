// Review regressions: actual MuJoCo placement plus adversarial evaluator observations.
// Edited observations below test fail-closed interpretation, not robot motion performance.
import assert from 'node:assert/strict';
import {readFile,writeFile,mkdir} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import {createHash} from 'node:crypto';
import {compileGeneralScene,appendGeneralSceneXml} from '../../src/physics/openarm-general-scene.js';
import {GeneralTaskEvaluator,validateGeneralTask,generalPlacementState} from '../../src/physics/openarm-general-task.js';
import {workcellBodies,workcellConstraints} from '../../src/physics/openarm-workcell-scene.js';
import {OPENARM_V2_PHASE5A_MODEL_PACKAGE as base} from '../../src/physics/openarm-model-package.js';
const version='robobuddy.lab.scene.v2';
const fixture=JSON.parse(await readFile(new URL('../fixtures/general-scenes/novel-adapter.json',import.meta.url),'utf8'));
const c=compileGeneralScene(fixture);
const task={schema_version:'robobuddy.lab.task.v2',id:'audit',type:'dry_transfer',object_id:'novel_adapter',receiver_id:'receiver',target_port:'receiving_top',side:'left',acknowledge_simulation_only:true};
const contact=(other,normalForceN=1)=>({geom1Name:'lab_novel_adapter_c0',geom2Name:other,normalForceN,distanceM:0});
const pads=()=>['finger_inner_left_collision_0','finger_outer_left_collision_0'].map(name=>contact(name));
function trial(){
 const o={simulationTimeSeconds:0,model:{sha256:'test-fixture-only'},openarm:{equipment:c.records,pinchReferences:{left:{positionM:[.554,.1535,1.07]}}},bodies:Object.fromEntries(c.records.map(r=>[r.bodyId,{positionM:[...r.position_m],quaternionWxyz:[...r.quaternion_wxyz],linearVelocityMS:[0,0,0],angularVelocityRadS:[0,0,0]}])),contactsReadable:true,contactCount:1,contacts:[contact('lab_bench_c0')]};
 const evaluator=new GeneralTaskEvaluator(validateGeneralTask(task,o),o,'test');
 function step({contacts=o.contacts,position=o.bodies.lab_novel_adapter.positionM,pinch=o.openarm.pinchReferences.left.positionM,dt=.02}={}){
   o.simulationTimeSeconds+=dt;o.contacts=structuredClone(contacts);o.contactCount=contacts.length;o.bodies.lab_novel_adapter.positionM=[...position];o.openarm.pinchReferences.left.positionM=[...pinch];evaluator.observe(structuredClone(o));
 }
 function lift(){for(let i=0;i<5;i++)step();for(let i=0;i<5;i++)step({contacts:pads()});step({contacts:pads(),position:[.554,.1535,1.055]});assert.equal(evaluator.snapshot().observedSequence.lift,true);}
 function carry(){lift();step({contacts:pads(),position:[.60,.1535,1.055]});assert.equal(evaluator.snapshot().observedSequence.carry,true);}
 return {o,evaluator,step,lift,carry};
}
const checks=[];
function check(name,fn){fn();checks.push(name);}
check('another load-bearing surface cannot be credited as unsupported carry',()=>{
 const t=trial();t.lift();t.step({contacts:[...pads(),contact('lab_bench_c0')],position:[.60,.1535,1.055]});
 assert.equal(t.evaluator.snapshot().observedSequence.carry,false);assert.match(t.evaluator.snapshot().invalidated,/another support/);
});
check('palm-assisted carry invalidates selected-finger evidence',()=>{
 const t=trial();t.lift();t.step({contacts:[...pads(),contact('ee_base_link_left_collision_0')],position:[.60,.1535,1.055]});assert.match(t.evaluator.snapshot().invalidated,/Palm/);
});
check('opposite-gripper assistance cannot pass a single-gripper trial',()=>{
 const t=trial();t.lift();t.step({contacts:[...pads(),contact('finger_inner_right_collision_0')]});assert.match(t.evaluator.snapshot().invalidated,/opposite/);
});
check('lost grasp cannot be stitched to a later regrasp',()=>{
 const t=trial();t.lift();for(let i=0;i<4;i++)t.step({contacts:[]});t.step({contacts:pads(),position:[.62,.1535,1.06]});assert.match(t.evaluator.snapshot().invalidated,/lost in transit/);assert.equal(t.evaluator.snapshot().observedSequence.carry,false);
});
check('a release far from the target cannot be credited',()=>{
 const t=trial();t.carry();for(let i=0;i<4;i++)t.step({contacts:[]});assert.equal(t.evaluator.snapshot().observedSequence.release,false);assert.equal(t.evaluator.snapshot().success,false);
});
check('regrasp after receiving release invalidates frozen history',()=>{
 const t=trial();t.carry();t.step({contacts:[contact('lab_receiver_c0')],position:[.67,.1535,1.03]});assert.equal(t.evaluator.snapshot().observedSequence.release,true);t.step({contacts:pads()});assert.match(t.evaluator.snapshot().invalidated,/regrasped/);
});
check('duplicate-time malformed contacts invalidate rather than preserving success',()=>{
 const t=trial();t.o.contactsReadable=false;t.evaluator.observe(t.o);assert.match(t.evaluator.snapshot().invalidated,/Incomplete/);
});
check('a model identity change invalidates the trial',()=>{
 const t=trial();t.o.model.sha256='changed';t.evaluator.observe(t.o);assert.match(t.evaluator.snapshot().invalidated,/identity/);
});
check('unreadable/negative contact forces cannot support a placement claim',()=>{
 const t=trial();t.carry();t.step({contacts:[contact('lab_receiver_c0')],position:[.67,.1535,1.03]});
 for(const force of [NaN,-1]){const o=structuredClone(t.o);o.contacts.push(contact('lab_receiver_c0',force));o.contactCount++;assert.equal(generalPlacementState(o,t.evaluator.task).placed,false);}
});
check('robot link contact is not an independently supported source',()=>{
 const t=trial();for(let i=0;i<8;i++)t.step({contacts:[contact('link3_left_collision_0')]});assert.equal(t.evaluator.snapshot().observedSequence.sourceSupported,false);
});
check('source reference is the settled physical pose rather than the setup origin',()=>{
 const t=trial();for(let i=0;i<5;i++)t.step({position:[.554,.1535,.955]});for(let i=0;i<5;i++)t.step({contacts:pads(),position:[.554,.1535,.955]});t.step({contacts:pads(),position:[.554,.1535,.985]});assert.equal(t.evaluator.snapshot().observedSequence.lift,true);
});
check('compiler defaults cannot masquerade as measured values',()=>{
 for(const [quantity,key] of [['mass','mass_kg'],['friction','friction']]){const s=structuredClone(fixture);delete s.objects[1][key];s.objects[1].quantity_evidence={[quantity]:{source:'user_measured'}};assert.throws(()=>compileGeneralScene(s),/provenance requires/);}
 assert.throws(()=>compileGeneralScene({schema_version:version,id:'bad',objects:[{id:'b',position_m:[.3,0,1.1],catalog:{kind:'bottle'},quantity_evidence:{geometry:{source:'source_provided'}}}]}),/explicit dimensions/);
});
check('single grid operation reports rigid coupling and volume overlap assumptions',()=>{
 const s={schema_version:version,id:'grid',objects:[{id:'grid',position_m:[.4,0,1.1],parts:[{id:'a',shape:'box',dimensions_m:[.02,.02,.02],grid:{counts:[2,1,1],spacing_m:[.04,0,0]}}]}]};assert.ok(compileGeneralScene(s).report.warnings.some(w=>w.code==='COMPOUND_MASS_ASSUMPTION'));
});

check('dry-transfer preflight does not claim nominal fit was checked',()=>{
 const t=trial();const p=validateGeneralTask(task,t.o).preflight;
 assert.equal(p.status,'feature_definitions_checked');assert.equal(p.nominalClearanceM,null);
 assert.ok(p.warnings.some(w=>w.includes('placement orientation exists')));
});

// Native physical regression: a 20 mm cylinder fits a 24 mm circular pedestal.
const worker=new URL('../../src/physics/openarm-mujoco-worker.js',import.meta.url);
globalThis.self={location:worker};let reply,sequence=0;
globalThis.postMessage=v=>{reply=v;};globalThis.fetch=async input=>new Response(await readFile(fileURLToPath(new URL(input))));await import(worker.href);
async function rpc(op,payload){await self.onmessage({data:{id:++sequence,op,payload}});if(!reply.ok)throw new Error(reply.error);return reply.payload;}
const xml=await readFile(new URL('../../models/openarm_v2/manipulation.xml',import.meta.url),'utf8');
function pkg(s){const a=appendGeneralSceneXml(xml,s);return {...base,id:'general-review',modelId:'general-review',sceneMode:'authored',baseSha256:base.sha256,sha256:createHash('sha256').update(a.xml).digest('hex'),generalScene:a.spec,sceneSpecSha256:createHash('sha256').update(JSON.stringify(a.spec)).digest('hex'),bodies:[...workcellBodies(base.bodies,'authored'),...a.bodies],sceneConstraints:workcellConstraints(base.sceneConstraints,'authored')};}
const bench=fixture.objects[0];
const platform={id:'platform',motion:'fixed',fixed_reason:'Declared installed pedestal',position_m:[.4,-.35,1.005],parts:[{id:'cylinder',shape:'cylinder',radius_m:.012,height_m:.025}],ports:[{id:'top',type:'support',part_id:'cylinder'}]};
const roundTask={...task,object_id:'sample',receiver_id:'platform',target_port:'top'};
const roundResults=[];
for(const shape of ['cylinder','sphere']){
 const sample={id:'sample',position_m:[.4,-.35,shape==='sphere'?1.04:1.03],mass_kg:.025,parts:[{id:'body',shape,radius_m:.01,...(shape==='cylinder'?{height_m:.04}:{})}]};
 let state=await rpc('load',{modelPackage:pkg({schema_version:version,id:shape,objects:[bench,platform,sample]})});
 for(let i=0;i<4;i++)state=await rpc('step',{count:500});
 assert.equal(state.bodies.lab_sample.centerOfMassM.length,3);
 const validated=validateGeneralTask(roundTask,state),placement=generalPlacementState(state,validated.task);
 assert.equal(placement.placed,true,JSON.stringify(placement));
 const off=structuredClone(state);off.bodies.lab_sample.positionM[0]+=.006;assert.equal(generalPlacementState(off,validated.task).placed,false);
 roundResults.push({shape,placement,centerOfMassM:state.bodies.lab_sample.centerOfMassM});
}
const socket={id:'socket',position_m:[.4,-.35,1.005],motion:'fixed',fixed_reason:'Installed socket',parts:[{id:'bore',shape:'hollow_profile',profile:[[0,.012,.016],[.08,.012,.016]]}],ports:[{id:'opening',type:'opening',part_id:'bore'}],quantity_evidence:{geometry:{source:'assumed',uncertainty_m:0,relative_uncertainty:1}}};
const peg={id:'peg',position_m:[.5,-.35,1.005],parts:[{id:'stem',shape:'cylinder',radius_m:.009,height_m:.06}],ports:[{id:'stem',type:'peg',part_id:'stem'}],quantity_evidence:{geometry:{source:'assumed',uncertainty_m:0,relative_uncertainty:1}}};
const state=await rpc('load',{modelPackage:pkg({schema_version:version,id:'uncertainty',objects:[bench,socket,peg]})});
const uncertainty=validateGeneralTask({...task,type:'insert',object_id:'peg',receiver_id:'socket',target_port:'opening',object_port:'stem'},state).preflight;
assert.ok(uncertainty.uncertaintyAllowanceM>uncertainty.nominalClearanceM);assert.ok(uncertainty.warnings.some(w=>w.includes('Real fit is unresolved')));checks.push('relative feature uncertainty cannot be ignored in nominal fit assessment');
await rpc('dispose');
await mkdir('test-results',{recursive:true});await writeFile('test-results/openarm-general-review.json',JSON.stringify({checks,roundResults,uncertainty,engine:'actual bundled MuJoCo WASM for round-object placement; synthetic adversarial observation sequences for evaluator guards',arbitraryPhotoBenchmark:false,hardwareValidated:false},null,2));
console.log(`General fidelity review: ${checks.length} adversarial/contract checks and ${roundResults.length} actual-MuJoCo round-object placements passed`);
