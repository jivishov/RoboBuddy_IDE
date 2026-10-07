// Executes the pinned MuJoCo WASM, including open-bore contact geometry.
import assert from 'node:assert/strict';
import { readFile, writeFile, mkdir } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { createHash } from 'node:crypto';
import { OPENARM_V2_PHASE5A_MODEL_PACKAGE as base } from '../../src/physics/openarm-model-package.js';
import { appendEquipmentXml, validateOpenArmEquipment, compileEquipmentDefinitions } from '../../src/physics/openarm-equipment.js';
import { workcellBaseXml, workcellBodies, workcellGeometry, workcellConstraints } from '../../src/physics/openarm-workcell-scene.js';
import { funnelSeatingState, graspState } from '../../src/physics/openarm-observation.js';
import { OPENARM_LAB_ASSETS } from '../../src/physics/openarm-lab-assets.js';

const worker = new URL('../../src/physics/openarm-mujoco-worker.js', import.meta.url);
globalThis.self = { location: worker }; let reply, sequence = 0;
globalThis.postMessage = value => { reply = value; };
globalThis.fetch = async input => new Response(await readFile(fileURLToPath(new URL(input))));
await import(worker.href);
async function rpc(op, payload) { await self.onmessage({ data: { id: ++sequence, op, payload } }); if (!reply.ok) throw new Error(reply.error); return reply.payload; }
const sourceXml=await readFile(new URL('../../models/openarm_v2/manipulation.xml',import.meta.url),'utf8');
const geometry=JSON.parse(await readFile(new URL('../../models/openarm_v2/geometry.json',import.meta.url),'utf8'));
const blankXml=workcellBaseXml(sourceXml,'blank');
assert.equal(workcellBaseXml(sourceXml,'baseline'),sourceXml);
for (const name of ['flask','beaker','left_hotplate','left_source_support','right_source_support','right_ring_gauze','right_ring_bracket','right_ring_post']) assert.ok(!blankXml.includes(`name="${name}"`));
for (const name of ['floor','cell_table','mount_column','openarm_mount']) assert.ok(blankXml.includes(`name="${name}"`));
assert.ok(workcellGeometry(geometry.geoms,'blank').every(g=>!['flask','beaker'].includes(g.bodyId)));
assert.throws(()=>workcellBaseXml(sourceXml,'unknown'));
function packageFor(raw,mode='blank') {
  const equipment=validateOpenArmEquipment(raw), compiled=appendEquipmentXml(workcellBaseXml(sourceXml,mode),equipment);
  assert.deepEqual(validateOpenArmEquipment(equipment),equipment,'normalized input must be worker-idempotent');
  return { ...base, id:'lab-assets-test',modelId:'lab-assets-test',sceneMode:mode,baseSha256:base.sha256,sha256:createHash('sha256').update(compiled.xml).digest('hex'),equipment,bodies:[...workcellBodies(base.bodies,mode),...compiled.bodies],sceneConstraints:workcellConstraints(base.sceneConstraints,mode) };
}
let state=await rpc('load',{modelPackage:packageFor([])});
assert.equal(state.openarm.sceneMode,'blank'); assert.equal(state.bodies.flask,undefined); assert.equal(state.bodies.beaker,undefined);
assert.equal(state.setupLog[0].sceneMode,'blank');
const initialRobot=structuredClone(state.bodies.openarm_mount);
state=await rpc('reset'); assert.equal(state.openarm.sceneMode,'blank'); assert.deepEqual(state.bodies.openarm_mount,initialRobot);
for (const [kind,spec] of Object.entries(OPENARM_LAB_ASSETS)) {
  const normalized=validateOpenArmEquipment([{id:'asset',kind,position_m:[.4,-.4,1.005]}]);
  assert.equal(normalized[0].dynamic,spec.dynamic);
  const compiled=compileEquipmentDefinitions(normalized); assert.ok(compiled.geoms.length>0);
  assert.ok(compiled.geoms.every(g=>g.type!=='mesh'||compiled.meshes[g.mesh]));
  await rpc('load',{modelPackage:packageFor([{id:'asset',kind,position_m:[.4,-.4,1.005],dynamic:false}])});
}
assert.throws(()=>validateOpenArmEquipment([{id:'bad',kind:'funnel',position_m:[.4,0,1.005],quaternion_wxyz:[1,1,0,0]}]));
assert.throws(()=>validateOpenArmEquipment([{id:'bad',kind:'funnel',position_m:[.4,0,1.005],quaternion_wxyz:[1,0,0,0],yaw_rad:0}]));
assert.throws(()=>validateOpenArmEquipment([{id:'bad',kind:'burette',position_m:[.4,0,1.005],dynamic:true}]));
assert.throws(()=>validateOpenArmEquipment([{id:'bad',kind:'funnel',position_m:[.4,0,1.005],mesh_url:'https://example.org/evil.glb'}]));
const sideways={id:'sideways',kind:'funnel',position_m:[.4,-.4,1.04],quaternion_wxyz:[Math.SQRT1_2,0,Math.SQRT1_2,0]};
await rpc('load',{modelPackage:packageFor([sideways])});
assert.throws(()=>validateOpenArmEquipment([{...sideways,position_m:[.4,-.4,1.005]}]));

const receiver={id:'burette',kind:'burette',position_m:[.38,-.35,1.005]};
// Explicit initial drop above the opening verifies contact topology; this is NOT
// a robot transfer. Robot-causal manipulation is tested separately below.
const funnel={id:'funnel',kind:'funnel',position_m:[.38,-.35,1.410]};
state=await rpc('load',{modelPackage:packageFor([receiver,funnel])});
assert.equal(funnelSeatingState(state,'lab_funnel','lab_burette').seated,false);
async function advance(seconds) {
  let steps=Math.round(seconds/.001);
  while (steps) { const count=Math.min(1000,steps); state=await rpc('step',{count}); steps-=count; }
  return state;
}
await advance(2);
const seating=funnelSeatingState(state,'lab_funnel','lab_burette');

assert.equal(seating.seated,true,'open bore must admit the stem and physically support the released funnel');
const good=structuredClone(state);
for (const change of [
  o=>o.contactsReadable=false,
  o=>o.contactCount++,
  o=>o.bodies.lab_funnel.linearVelocityMS=[.1,0,0],
  o=>o.bodies.lab_funnel.positionM[0]+=.03,
  o=>o.bodies.lab_funnel.positionM[2]+=.1,
  o=>{ o.contacts.push({geom1Name:'lab_funnel_bowl_0',geom2Name:'finger_inner_left_collision_0',normalForceN:1,distanceM:0}); o.contactCount++; },
  o=>{ o.contacts.push({geom1Name:'lab_funnel_bowl_0',geom2Name:'cell_table',normalForceN:1,distanceM:0}); o.contactCount++; },
  o=>o.contacts[0].distanceM=-.01,
]) { const o=structuredClone(good);change(o);assert.equal(funnelSeatingState(o,'lab_funnel','lab_burette').seated,false); }
// Contact-aware startup rejects a stem that intersects the tube wall.
await assert.rejects(rpc('load',{modelPackage:packageFor([receiver,{...funnel,position_m:[.389,-.35,1.375]}])}),/overlaps/);
// A free funnel is grasped and lifted by the actual robot. This independently
// checks manipulability; it does not conflate the passive seating check above
// with an end-to-end funnel transfer.
const pickup=[{id:'funnel',kind:'funnel',position_m:[.554,.1535,1.005]}];
async function moveTool(positionM) { await rpc('command',{type:'set_tool_target',side:'left',positionM,quaternionWxyz:[Math.SQRT1_2,0,-Math.SQRT1_2,0],durationSeconds:4});await advance(4); }
async function grip(value) { await rpc('command',{type:'move_joint_targets',targetsRad:{openarm_left_finger_joint1:value},durationSeconds:3});await advance(3); }
state=await rpc('load',{modelPackage:packageFor(pickup)});
await moveTool([.554,.1535,1.073]);await grip(.08);assert.ok(graspState(state,'left','lab_funnel').bilateralContact);
await moveTool([.554,.1535,1.25]);assert.ok(graspState(state,'left','lab_funnel').bilateralContact);
assert.ok(state.bodies.lab_funnel.positionM[2]>1.16);
const causalLift={heightM:state.bodies.lab_funnel.positionM[2],grasp:graspState(state,'left','lab_funnel')};
state=await rpc('load',{modelPackage:packageFor(pickup)});
await moveTool([.554,.1535,1.073]);await grip(.65);await moveTool([.554,.1535,1.25]);
assert.ok(Math.abs(state.bodies.lab_funnel.positionM[2]-1.005)<.002,'open-gripper negative control must not carry the funnel');
state=await rpc('load',{modelPackage:packageFor([],'baseline')});assert.ok(state.bodies.flask && state.bodies.beaker);
await rpc('dispose');
await mkdir('test-results',{recursive:true});await writeFile('test-results/openarm-lab-assets.json',JSON.stringify({engine:'MuJoCo 3.11.0 WASM',hardwareValidated:false,dropOnlySeating:seating,causalFunnelGraspLift:causalLift,endToEndFunnelTransferVerified:false,tests:['blank-mode-removal','baseline-restoration','all-catalog-assets-compile','normalized-input-revalidation','quaternion-and-envelope-rejection','hollow-bore-passive-seating','held-offset-fast-unreadable-penetrating-negative-controls','initial-overlap-rejection','causal-funnel-grasp-and-lift','open-gripper-no-lift-negative']},null,2));
console.log('OpenArm blank workcell, laboratory assets and real MuJoCo seating checks: OK');
