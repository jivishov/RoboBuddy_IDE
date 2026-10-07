// Exercises the real pinned MuJoCo WASM worker, not a mocked plant.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { createHash } from 'node:crypto';
import { OPENARM_V2_PHASE5A_MODEL_PACKAGE as base } from '../../src/physics/openarm-model-package.js';
import { OPENARM_V2_BIMANUAL_CONTROLLER as controller } from '../../src/physics/openarm-scene.js';
import { OpenArmBimanualStackEvaluator } from '../../src/physics/openarm-task-evaluator.js';
import { appendEquipmentXml, validateOpenArmEquipment } from '../../src/physics/openarm-equipment.js';
const worker = new URL('../../src/physics/openarm-mujoco-worker.js', import.meta.url);
globalThis.self = { location: worker }; let reply, sequence = 0;
globalThis.postMessage = value => { reply = value; };
globalThis.fetch = async input => new Response(await readFile(fileURLToPath(new URL(input))));
await import(worker.href);
async function rpc(op, payload) { await self.onmessage({ data: { id: ++sequence, op, payload } }); if (!reply.ok) throw new Error(reply.error); return reply.payload; }
let state;
async function advance(seconds, evaluator = null) {
  let steps = Math.round(seconds / .001);
  while (steps) {
    const count = Math.min(1000, steps), result = await rpc('stepSampled', { count, sampleEverySteps: 2 });
    if (evaluator) for (const observation of result.observations) evaluator.observe({ ...observation, simulationTimeSeconds: observation.simulationTime });
    state = result.observations.at(-1); steps -= count;
  }
  return state;
}
state = await rpc('load', { modelPackage: base });
assert.ok(Math.abs(state.openarm.pinchReferences.left.positionM[0] - .554) < .002);
assert.ok(state.contacts.every(c => c.distanceM >= -.0005));
await rpc('command', { type: 'move_joint_targets', targetsRad: { openarm_left_joint1: -.10 }, durationSeconds: 1 });
const before = await rpc('observe');
await assert.rejects(rpc('command', { type: 'set_joint_targets', targetsRad: { openarm_left_joint1: -.2, openarm_right_joint1: 99 } }));
assert.deepEqual((await rpc('observe')).joints, before.joints, 'invalid mixed command must not partially latch');
await assert.rejects(rpc('command', { type: 'move_joint_targets', targetsRad: { openarm_left_joint1: -2 }, durationSeconds: .04 }));
assert.deepEqual((await rpc('observe')).joints, before.joints);
await advance(.5); const a = await rpc('observe');
for (let i=0; i<20; i++) assert.deepEqual(await rpc('observe'), a, 'observation must be a pure read');
await rpc('reset'); await rpc('command', { type: 'move_joint_targets', targetsRad: { openarm_left_joint1: -.10 }, durationSeconds: 1 });
const b = await rpc('step', { count: 500 });
for (const id of Object.keys(a.joints)) assert.ok(Math.abs(a.joints[id].positionRad - b.joints[id].positionRad) < 1e-12, 'sampling cannot change dynamics');
await rpc('reset'); const physicalBeforeIK = await rpc('observe');
const ik = await rpc('command', { type: 'set_tool_target', side: 'left', positionM: [.554, .1535, 1.25], quaternionWxyz: [Math.SQRT1_2, 0, -Math.SQRT1_2, 0], durationSeconds: 4 });
assert.ok(ik.openarm.motionPlan.positionResidualM < .0015);
assert.deepEqual(ik.bodies, physicalBeforeIK.bodies, 'IK must not assign the physical state');
await assert.rejects(rpc('command', { type: 'set_tool_target', side: 'left', positionM: [.95,.75,1.9], durationSeconds: 4 }));
assert.deepEqual((await rpc('observe')).joints, ik.joints, 'unreachable IK cannot change active targets');
state = await rpc('reset'); const evaluator = new OpenArmBimanualStackEvaluator(); evaluator.observe({ ...state, simulationTimeSeconds: state.simulationTime });
for (const stage of controller.stages) {
  if (Object.keys(stage.targetsRad).length) await rpc('command', { type: 'move_joint_targets', targetsRad: stage.targetsRad, durationSeconds: stage.durationSeconds });
  await advance(stage.durationSeconds, evaluator);
}
const nominal = evaluator.snapshot(); assert.equal(nominal.success, true); assert.equal(nominal.palmContactSeen, false); assert.ok(nominal.maximumPenetrationM < .002);
for (const object of [nominal.flask, nominal.beaker]) { assert.ok(object.maximumBilateralDurationSeconds >= .06); assert.ok(object.maxHeldHorizontalTravelM >= .06); assert.ok(object.settleEvidenceDurationSeconds >= .2); }
const modelXml = fs.readFileSync(new URL('../../models/openarm_v2/manipulation.xml',import.meta.url),'utf8');
function packageFor(raw) {
  const equipment = validateOpenArmEquipment(raw), compiled = appendEquipmentXml(modelXml, equipment);
  return { ...base, id: 'test-equipment', modelId: 'test-equipment', baseSha256: base.sha256, sha256: createHash('sha256').update(compiled.xml).digest('hex'), equipment, bodies: [...base.bodies, ...compiled.bodies] };
}
await assert.rejects(rpc('load',{ modelPackage: packageFor([{id:'overlap',kind:'block',position_m:[.55,.1535,1.10]}]) }), /overlaps/);
const equipment = [{id:'button1',kind:'button',position_m:[.57,.32,1.005]},{id:'tray1',kind:'tray',position_m:[.35,-.38,1.005]},{id:'vial1',kind:'vial',position_m:[.35,-.38,1.08]}];
state = await rpc('load', {modelPackage: packageFor(equipment)}); assert.equal(state.openarm.equipmentJoints[0].pressed, false);
async function tool(positionM,durationSeconds=4) { await rpc('command', {type:'set_tool_target', side:'left', positionM, quaternionWxyz:[Math.SQRT1_2,0,-Math.SQRT1_2,0],durationSeconds}); await advance(durationSeconds); }
await tool([.554,.1535,1.25]); await tool([.57,.32,1.23]);
await rpc('command',{type:'move_joint_targets',targetsRad:{openarm_left_finger_joint1:.02},durationSeconds:3}); await advance(3);
await tool([.57,.32,1.060]); await tool([.57,.32,1.050],2);
assert.equal(state.openarm.equipmentJoints[0].pressed, true, 'actual finger contact must depress the passive spring button');
assert.ok(state.contacts.some(c=>[c.geom1Name,c.geom2Name].includes('lab_button1_plunger') && [c.geom1Name,c.geom2Name].some(n=>n.startsWith('finger_')) && c.normalForceN > .01));
assert.ok(Math.hypot(...state.bodies.flask.positionM.map((v,i)=>v-[.55,.1535,1.092][i])) < .003, 'button operation must not accidentally carry the flask');
assert.ok(state.contacts.some(c=>[c.geom1Name,c.geom2Name].includes('lab_vial1_solid') && [c.geom1Name,c.geom2Name].includes('lab_tray1_base') && c.normalForceN > .01));
const buttonPressed = state.openarm.equipmentJoints[0].position;
await tool([.57,.32,1.15],2); assert.equal(state.openarm.equipmentJoints[0].pressed,false,'passive button springs back after the robot withdraws');
if (process.env.OPENARM_EVIDENCE_PATH) fs.writeFileSync(process.env.OPENARM_EVIDENCE_PATH,JSON.stringify({ engine:'MuJoCo 3.11.0 WASM', classification:'simulator verification, not hardware validation', nominal, equipment:{buttonPressedM:buttonPressed,buttonReleasedM:state.openarm.equipmentJoints[0].position,vialFinalPositionM:state.bodies.lab_vial1.positionM}, tests:['invalid-command-atomicity','observation-purity','sample-cadence-conformance','IK-no-plant-state-write','unreachable-IK-rejection','causal-bimanual-transfer','initial-equipment-penetration-rejection','robot-driven-passive-button','vial-tray-contact','button-spring-return']},null,2));
await rpc('dispose'); console.log('OpenArm actual WASM nominal transfer, IK, equipment contact, passive button and observation conformance: OK');
