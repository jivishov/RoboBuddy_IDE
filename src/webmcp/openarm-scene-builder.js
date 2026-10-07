import { captureOpenArm, assertOpenArmCurrent, backendFor, plain, onlyKeys, invalid } from './openarm-physical-control.js';
import { GENERAL_SCENE_VERSION, GENERAL_LIMITS, compileGeneralScene } from '../physics/openarm-general-scene.js';
import { SHAPES } from '../physics/openarm-general-geometry.js';
import { generalSceneSchema, generalTaskSchema, manageGeneralSceneSchema, manageGeneralTaskSchema } from '../physics/openarm-general-schema.js';

export function getGeneralSceneDefinitions(facade){
  const c=facade.getRegistrationContext();
  if(c.profileId!=='openarm'||c.simulationMode!=='physical_mujoco'||c.workspaceStatus!=='ready'||!c.simulationReady)return [];
  return [
    {name:'inspect_openarm_scene',title:'Inspect general laboratory builder',readOnly:true,description:'Read the general builder schema, current/staged reconstruction report, source inventory, unresolved equipment, assumptions, physical task evidence, or full reusable SceneSpec. The image must reach the external multimodal agent; a local image label is not perception. New equipment need NOT exist in a catalog: use parts, hollow profiles, convex extrusions, validated convex mesh components, optional visual meshes, and repeated components. Inspect schema for limits and units. Simulator ground truth is not hardware sensing.',inputSchema:{type:'object',properties:{view:{enum:['summary','schema','spec','object','evidence']},object_id:{type:'string',pattern:'^[a-z][a-z0-9_]{0,23}$'}},additionalProperties:false}},
    {name:'manage_openarm_scene',title:'Construct a laboratory scene from new geometry',readOnly:false,description:`Author ${GENERAL_SCENE_VERSION} in Z-up world metres. No preloaded equipment required. stage accepts a complete editable scene and source inventory; inspect current authority.sceneRevision first. Parts support independently rotated boxes/spheres/cylinders, frustums, hollow_profile stations [z,innerRadius,outerRadius], convex polygon extrusions and closed outward-wound convex meshes. Hollow shapes stay hollow; concave collision meshes are rejected, not filled by a hull. Every fixed object needs fixed_reason. Image quantity provenance is author-declared, never automatically verified. check compiles/settles a disposable candidate without changing active physics. apply explicitly resets into a robot/pedestal/floor plus your authored bench/equipment; failed candidates preserve the current scene. Scenes support 24 objects/512 collision components/512 kB. Unresolved image items must be listed, not silently omitted. No code, XML, remote URLs, fluid process, custom mechanisms, live pose editing, attachment, or automatic robot planner. Stage replaces the whole authored scene, not a patch.`,inputSchema:manageGeneralSceneSchema()},
    {name:'manage_openarm_task',title:'Validate and define an observed physical transfer task',readOnly:false,description:'assess checks nominal geometry and declared uncertainty without moving the robot. define freezes a dry_transfer to a support port or a vertical insert between cylindrical peg/opening ports, BEFORE grasping. IDs are SceneSpec object IDs without lab_ prefix. Constructor derives task ports from physical components, not arbitrary success regions. Execute existing control_openarm_simulation/run_openarm_program; wait_for authored_task_complete tests independent initial support, sustained bilateral grasp, unsupported lift/carry, release at the receiver, retreat and settled receiving contact. Single selected-gripper transfers only: palm assistance, another support after lift, prolonged loss of grip, or regrasp after release invalidate the trial. Reset, scene edits, incomplete observations and excessive penetration invalidate evidence. No reachability/collision-free-path or real-world fit guarantee. Explicit simulation-only acknowledgment required.',inputSchema:manageGeneralTaskSchema()},
  ];
}
export function inspectGeneralScene(facade,input,epoch){
  plain(input);onlyKeys(input,['view','object_id']);facade.assertActive(epoch);
  const c=facade.getRegistrationContext();if(c.profileId!=='openarm'||c.simulationMode!=='physical_mujoco'||!c.simulationReady||c.workspaceStatus!=='ready')invalid('Ready OpenArm workspace required');
  const view=input.view??'summary';if(!['summary','schema','spec','object','evidence'].includes(view))invalid('Unsupported scene inspection view');
  if(view==='schema')return {ok:true,schemaVersion:GENERAL_SCENE_VERSION,limits:GENERAL_LIMITS,constructionShapes:SHAPES,sceneSchema:generalSceneSchema(),taskSchema:generalTaskSchema(),validationNotes:'Schemas enforce structural limits; quaternion normalization, mesh topology, feature fit and physical compilation require runtime validation.',coordinateNotes:'Scene object position is its authored local origin. Boxes/spheres are centred; cylinders/frustums/profiles/extrusions start at z=0. Part transforms are relative to the object. Grid offsets are in object axes.',meshImport:'Numeric vertices_m/triangles for closed convex collision components. Concave visual_mesh needs a separately authored collision model. No raw GLB/OBJ decoder or automatic decomposition.',referenceTransport:'Provide the photo to the agent. This application does not independently interpret image pixels.'};
  if(view==='object'&&typeof input.object_id!=='string')invalid('object_id required for object inspection');
  return {ok:true,...backendFor(facade).getGeneralSceneState(view,input.object_id)};
}
export async function mutateGeneralScene(facade,input,signal,epoch){
  plain(input);
  const fields={stage:['expected_scene_revision','scene'],check:['stage_id','settle_seconds'],apply:['stage_id','acknowledge_reset'],discard:[]}[input.command];
  if(!fields)invalid('command must be stage, check, apply, or discard');onlyKeys(input,['command',...fields]);
  if(input.command==='stage')compileGeneralScene(input.scene);
  if(input.command==='apply'&&input.acknowledge_reset!==true)invalid('Explicit acknowledge_reset:true required');
  const baseline=captureOpenArm(facade,epoch),app=facade.app,backend=backendFor(facade);
  if(input.command==='stage'&&input.expected_scene_revision!==baseline.authority.sceneRevision)invalid('Stale scene revision');
  facade.activeControlId=`general-scene-${epoch}-${++facade.controlSequence}`;let token=null;
  try{
    token=app.beginExecution?.();if(token==null)throw new Error('Could not acquire scene execution lease');
    const guard=()=>assertOpenArmCurrent(facade,baseline,epoch,signal,token);guard();
    let result;
    if(input.command==='stage')result=backend.stageGeneralScene(input.scene);
    else if(input.command==='discard')result=backend.discardStagedEquipment();
    else if(input.command==='check')result=await backend.checkStagedGeneralScene(input.stage_id,input.settle_seconds??.3,guard);
    else result=await backend.applyStagedEquipment(input.stage_id,true,guard);
    assertOpenArmCurrent(facade,baseline,epoch,signal,token,input.command==='apply');
    app.setStatus?.(`General lab scene ${input.command}; reconstruction, physics and manipulation evidence remain separate.`);app.renderPanels?.();
    return {ok:true,result,authority:app.sim.getPhysicalAuthorityToken(),hardwareValidated:false};
  }finally{if(token!=null)app.finishExecution?.(token);facade.activeControlId=null;}
}
export function mutateGeneralTask(facade,input,signal,epoch){
  plain(input);onlyKeys(input,['command','expected_scene_revision',...(input.command==='clear'?[]:['task'])]);
  if(!['assess','define','clear'].includes(input.command))invalid('Task command must be assess, define or clear');
  const baseline=captureOpenArm(facade,epoch);if(input.expected_scene_revision!==baseline.authority.sceneRevision)invalid('Stale scene revision');
  assertOpenArmCurrent(facade,baseline,epoch,signal);
  const backend=backendFor(facade),result=input.command==='clear'?backend.clearGeneralTask():backend.defineGeneralTask(input.task,input.command==='assess');
  return {ok:true,result,authority:facade.app.sim.getPhysicalAuthorityToken(),hardwareValidated:false};
}
