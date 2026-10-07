import assert from 'node:assert/strict';
import {readFile,writeFile,mkdir} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import {createHash} from 'node:crypto';
import {compileGeneralScene,normalizeGeneralScene,appendGeneralSceneXml} from '../../src/physics/openarm-general-scene.js';
import {normalizePart,partComponents,validateMesh} from '../../src/physics/openarm-general-geometry.js';
import {workcellBodies,workcellConstraints} from '../../src/physics/openarm-workcell-scene.js';
import {OPENARM_V2_PHASE5A_MODEL_PACKAGE as base} from '../../src/physics/openarm-model-package.js';
import {GeneralTaskEvaluator,validateGeneralTask,generalPlacementState} from '../../src/physics/openarm-general-task.js';
import {graspState} from '../../src/physics/openarm-observation.js';

const bench={id:'bench',role:'bench',motion:'fixed',fixed_reason:'Declared installed work surface, not an inferred attachment',mass_kg:15,position_m:[.41,0,.98],parts:[{id:'top',shape:'box',dimensions_m:[.82,1.1,.05]}],ports:[{id:'surface',type:'support',part_id:'top'}]};
const sample={id:'novel_adapter',label:'Previously undefined sample adapter',position_m:[.554,.1535,1.005],mass_kg:.025,parts:[{id:'body',shape:'box',position_m:[0,0,.0425],dimensions_m:[.04,.04,.085]},{id:'tab',shape:'box',position_m:[.024,0,.024],dimensions_m:[.012,.012,.006]}],ports:[{id:'grip',type:'grasp',position_m:[0,0,.065]}]};
const receiver={id:'receiver',role:'fixture',position_m:[.67,.1535,1.005],motion:'fixed',fixed_reason:'Explicit fixture assumption',parts:[{id:'top',shape:'box',position_m:[0,0,.0125],dimensions_m:[.12,.12,.025]}],ports:[{id:'receiving_top',type:'support',part_id:'top'}]};
const scene={schema_version:'robobuddy.lab.scene.v2',id:'unseen_adapter_transfer',objects:[bench,sample,receiver]};
const normalized=normalizeGeneralScene(scene);assert.deepEqual(normalizeGeneralScene(normalized),normalized);
assert.equal(compileGeneralScene(scene).geoms.length,4);
assert.throws(()=>compileGeneralScene({...scene,objects:[{...sample,parts:[{...sample.parts[0],script:'evil()'}]}]}),/unsupported/);
assert.throws(()=>compileGeneralScene({...scene,objects:[{...sample,motion:'fixed'}]}),/fixed_reason/);
assert.throws(()=>compileGeneralScene({...scene,objects:[{...sample,position_m:[1.7,1.2,2.2]}]}),/volume/);
assert.throws(()=>compileGeneralScene({...scene,reference:{mode:'external_reference'}}),/inventory/);
assert.throws(()=>compileGeneralScene({...scene,inventory:[{id:'missing',status:'unresolved',reason:'Occluded',object_ids:['receiver']}]}),/Unresolved/);
const withUnknown={...scene,reference:{mode:'external_reference',agent_image_access:'declared_by_agent'},inventory:[{id:'assembled',status:'approximated',object_ids:['bench','novel_adapter','receiver'],reason:'Author estimate'},{id:'hidden',status:'unresolved',reason:'Equipment occluded'}]};
assert.equal(compileGeneralScene(withUnknown).report.coverageComplete,false);
assert.equal(compileGeneralScene(withUnknown).report.imageReconstructionVerified,false);
for(const p of [
 {id:'cone',shape:'frustum',height_m:.08,bottom_radius_m:.035,top_radius_m:0},
 {id:'sleeve',shape:'hollow_profile',profile:[[0,.012,.015],[.05,.012,.015],[.09,.03,.034]],quaternion_wxyz:[Math.SQRT1_2,Math.SQRT1_2,0,0]},
 {id:'wedge',shape:'extrusion',height_m:.06,polygon_m:[[-.03,-.03],[.03,-.03],[0,.03]]},
 {id:'array',shape:'cylinder',height_m:.02,radius_m:.004,grid:{counts:[4,3,1],spacing_m:[.015,.015,0]}}
]){const n=normalizePart(p);assert.deepEqual(normalizePart(n),n);assert.ok(partComponents(n).length);}
const tetra={vertices_m:[[0,0,0],[.04,0,0],[0,.04,0],[0,0,.04]],triangles:[[0,2,1],[0,1,3],[1,2,3],[2,0,3]]};
assert.ok(validateMesh(tetra).volume>0);
assert.throws(()=>validateMesh({...tetra,triangles:tetra.triangles.map(t=>[t[0],t[2],t[1]])}),/convex/);
assert.throws(()=>validateMesh({...tetra,triangles:[...tetra.triangles.slice(0,3),[0,1,99]]}),/index/);
assert.throws(()=>compileGeneralScene({...scene,objects:[{...sample,parts:[{id:'over',shape:'hollow_profile',profile:[[0,.01,.012],[.1,.01,.012]],grid:{counts:[12,1,1],spacing_m:[.02,0,0]}}]}]}),/budget/);

const worker=new URL('../../src/physics/openarm-mujoco-worker.js',import.meta.url);
globalThis.self={location:worker};let reply,seq=0;globalThis.postMessage=r=>reply=r;
globalThis.fetch=async input=>new Response(await readFile(fileURLToPath(new URL(input))));await import(worker.href);
async function rpc(op,payload){await self.onmessage({data:{id:++seq,op,payload}});if(!reply.ok)throw new Error(reply.error);return reply.payload;}
const baseXml=await readFile(new URL('../../models/openarm_v2/manipulation.xml',import.meta.url),'utf8');
function packageFor(s){const c=appendGeneralSceneXml(baseXml,s);assert.ok(!c.xml.includes('name="cell_table"'));assert.ok(!c.xml.includes('name="flask"'));return {...base,id:'general-test',modelId:'general-test',sceneMode:'authored',baseSha256:base.sha256,sha256:createHash('sha256').update(c.xml).digest('hex'),generalScene:c.spec,sceneSpecSha256:createHash('sha256').update(JSON.stringify(c.spec)).digest('hex'),bodies:[...workcellBodies(base.bodies,'authored'),...c.bodies],sceneConstraints:workcellConstraints(base.sceneConstraints,'authored')};}
await assert.rejects(rpc('load',{modelPackage:{...packageFor(scene),sceneSpecSha256:'0'.repeat(64)}}),/SceneSpec checksum/);
let state=await rpc('load',{modelPackage:packageFor(scene)});
assert.equal(state.openarm.sceneMode,'authored');assert.equal(state.bodies.flask,undefined);
assert.equal(state.openarm.equipment.find(e=>e.id==='novel_adapter').ports[0].type,'grasp');
assert.deepEqual((await rpc('reset')).bodies,state.bodies);
// Each construction route is actually compiled by the bundled MuJoCo runtime.
const examples=[
 {...sample,id:'angled',position_m:[.4,-.35,1.06],parts:[{id:'a',shape:'box',dimensions_m:[.06,.03,.02],quaternion_wxyz:[.9238795325,0,.3826834324,0]}]},
 {...sample,id:'cone',position_m:[.4,-.35,1.005],parts:[{id:'a',shape:'frustum',height_m:.06,bottom_radius_m:.03,top_radius_m:0}]},
 {...sample,id:'prism',position_m:[.4,-.35,1.005],parts:[{id:'a',shape:'extrusion',height_m:.04,polygon_m:[[-.02,-.02],[.02,-.02],[0,.02]]}]},
 {...sample,id:'imported',position_m:[.4,-.35,1.005],parts:[{id:'a',shape:'convex_mesh',...tetra}]},
];
for(const e of examples){delete e.ports;await rpc('load',{modelPackage:packageFor({...scene,objects:[bench,e]})});}
// Real open-bore insertion, not credited as robot-caused transfer.
const socket={id:'novel_socket',motion:'fixed',fixed_reason:'Explicit installed socket boundary',position_m:[.4,-.35,1.005],parts:[{id:'bore',shape:'hollow_profile',profile:[[0,.012,.016],[.08,.012,.016]]}],ports:[{id:'opening',type:'opening',part_id:'bore'}]};
const pin={id:'novel_pin',mass_kg:.025,position_m:[.4,-.35,1.09],parts:[{id:'peg',shape:'cylinder',radius_m:.009,height_m:.06},{id:'shoulder',shape:'cylinder',radius_m:.02,height_m:.008,position_m:[0,0,.06]}],ports:[{id:'stem',type:'peg',part_id:'peg'}]};
const insertScene={...scene,id:'novel_socket_scene',objects:[bench,socket,pin]};
state=await rpc('load',{modelPackage:packageFor(insertScene)});
const insertTask={schema_version:'robobuddy.lab.task.v2',id:'fit',type:'insert',object_id:'novel_pin',receiver_id:'novel_socket',object_port:'stem',target_port:'opening',side:'left',insertion_depth_m:.05,acknowledge_simulation_only:true};
const v=validateGeneralTask(insertTask,state),passiveEvaluator=new GeneralTaskEvaluator(v,state,'fixture');
async function advance(seconds,evaluator=null){let n=Math.round(seconds/.001);while(n){const count=Math.min(500,n),result=await rpc('stepSampled',{count,sampleEverySteps:2});for(const o of result.observations)evaluator?.observe(o);state=result.observations.at(-1);n-=count;}return state;}
await advance(2,passiveEvaluator);
const passive=generalPlacementState(state,v.task);
assert.equal(passive.placed,true,JSON.stringify(passive));assert.equal(passiveEvaluator.snapshot().success,false);
const seated=structuredClone(state);
assert.throws(()=>new GeneralTaskEvaluator(v,state,'fixture'),/already in the receiving geometry/);
for(const change of [o=>o.contactsReadable=false,o=>o.contactCount++,o=>o.bodies.lab_novel_pin.positionM[0]+=.03,o=>o.bodies.lab_novel_pin.linearVelocityMS=[.1,0,0],o=>o.contacts[0].distanceM=-.01]){const o=structuredClone(seated);change(o);assert.equal(generalPlacementState(o,v.task).placed,false);}
await assert.rejects(rpc('load',{modelPackage:packageFor({...insertScene,objects:[bench,socket,{...pin,position_m:[.414,-.35,1.035]}]})}),/overlap/);

// A complete actuator/contact-driven transfer of a non-catalog compound object.
const taskSpec={schema_version:'robobuddy.lab.task.v2',id:'transfer',type:'dry_transfer',object_id:'novel_adapter',receiver_id:'receiver',target_port:'receiving_top',side:'left',acknowledge_simulation_only:true};
async function runTransfer(close){
 state=await rpc('load',{modelPackage:packageFor(scene)});const evaluator=new GeneralTaskEvaluator(validateGeneralTask(taskSpec,state),state,'fixture');
 const move=async p=>{await rpc('command',{type:'set_tool_target',side:'left',positionM:p,quaternionWxyz:[Math.SQRT1_2,0,-Math.SQRT1_2,0],durationSeconds:4});await advance(4,evaluator);};
 const grip=async q=>{await rpc('command',{type:'move_joint_targets',targetsRad:{openarm_left_finger_joint1:q},durationSeconds:3});await advance(3,evaluator);};
 await advance(.2,evaluator);await move([.554,.1535,1.068]);await grip(close?.08:.65);
 await move([.554,.1535,1.25]);if(!close)return evaluator.snapshot();await move([.67,.1535,1.25]);await move([.67,.1535,1.093]);await grip(.65);await move([.67,.1535,1.27]);await advance(.5,evaluator);
 return evaluator.snapshot();
}
const transfer=await runTransfer(true);
assert.ok(transfer.events.find(e=>e.event==='retreat').timeSeconds>transfer.events.find(e=>e.event==='release').timeSeconds);
const forged=structuredClone(state);forged.contacts=[];forged.contactCount=0;assert.throws(()=>new GeneralTaskEvaluator(validateGeneralTask(taskSpec,forged),forged,'fixture'),/receiving geometry/);

assert.equal(transfer.success,true,'Novel compound object must complete the entire observed causal transfer');
const negative=await runTransfer(false);assert.equal(negative.success,false);assert.equal(negative.observedSequence.grasp,false);
const exampleReports=[];
for(const name of ['titration-construction','novel-equipment-bench']){const s=JSON.parse(await readFile(new URL(`../fixtures/general-scenes/${name}.json`,import.meta.url),'utf8'));const started=performance.now();state=await rpc('load',{modelPackage:packageFor(s)});await advance(.3);exampleReports.push({name,objects:state.openarm.equipment.length,simulationTimeSeconds:state.simulationTime,elapsedWallMs:performance.now()-started,allBodyCoordinatesFinite:Object.values(state.bodies).every(b=>b.positionM.every(Number.isFinite)),imageReconstructionVerified:false});}
await rpc('dispose');
await mkdir('test-results',{recursive:true});await writeFile('test-results/openarm-general-scene.json',JSON.stringify({engine:'actual bundled MuJoCo WASM',scene,transfer,openGripperNegative:negative,passiveInsertion:passive,passiveInsertionTransfer:false,exampleReports,arbitraryPhotoBenchmark:false,hardwareValidated:false},null,2));
assert.deepEqual(JSON.parse(await readFile(new URL('../fixtures/general-scenes/novel-adapter.json',import.meta.url),'utf8')),scene);
console.log('General scene contracts, novel procedural/imported geometry, real insertion and full causal transfer: OK');
