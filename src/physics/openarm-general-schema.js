import { GENERAL_SCENE_VERSION, GENERAL_LIMITS, QUANTITY_SOURCES } from './openarm-general-scene.js';
import { GENERAL_TASK_VERSION } from './openarm-general-task.js';
import { OPENARM_LAB_ASSETS } from './openarm-lab-assets.js';
const num=(minimum,maximum)=>({type:'number',minimum,maximum});
const str=maxLength=>({type:'string',minLength:1,maxLength});
const id={type:'string',pattern:'^[a-z][a-z0-9_]{0,23}$',not:{enum:['constructor','prototype','__proto__']}};
const vec=(n,min=-3,max=3)=>({type:'array',items:num(min,max),minItems:n,maxItems:n});
const obj=(properties,required=[])=>({type:'object',properties,required,additionalProperties:false});
const array=(items,maxItems,minItems=0)=>({type:'array',items,minItems,maxItems});
const q=vec(4,-1,1),meshFields={vertices_m:array(vec(3),128,4),triangles:array({type:'array',items:{type:'integer',minimum:0,maximum:127},minItems:3,maxItems:3},256,4)};
export function generalSceneSchema(){
  const evidence=obj({source:{enum:[...QUANTITY_SOURCES]},source_detail:str(500),uncertainty_m:num(0,1),relative_uncertainty:num(0,10)},['source']);
  const common={id,position_m:vec(3),quaternion_wxyz:q,rgba:vec(4,0,1),density_kg_m3:num(1,20000),grid:obj({counts:array({type:'integer',minimum:1,maximum:12},3,3),spacing_m:vec(3,-2,2)},['counts','spacing_m'])};
  const shapeFields={box:{dimensions_m:vec(3,.002,2)},sphere:{radius_m:num(.001,1)},cylinder:{radius_m:num(.001,1),height_m:num(.002,2)},frustum:{bottom_radius_m:num(.001,1),top_radius_m:num(0,1),height_m:num(.002,2),segments:{type:'integer',minimum:8,maximum:32}},hollow_profile:{profile:array(vec(3,0,2),8,2),segments:{type:'integer',minimum:8,maximum:32}},extrusion:{polygon_m:array(vec(2,-1,1),16,3),height_m:num(.002,2)},convex_mesh:meshFields};
  const part={type:'object',oneOf:Object.entries(shapeFields).map(([shape,fields])=>obj({...common,shape:{const:shape},...fields},['id','shape',...Object.keys(fields).filter(k=>k!=='segments')]))};
  const ports={type:'object',oneOf:[obj({id,type:{const:'grasp'},position_m:vec(3)},['id','type','position_m']),obj({id,type:{enum:['support','opening','peg']},part_id:id},['id','type','part_id'])]};
  const objectSchema=obj({id,label:str(100),role:{enum:['bench','equipment','fixture','obstacle']},position_m:vec(3),quaternion_wxyz:q,motion:{enum:['free','fixed']},fixed_reason:str(500),mass_kg:num(.005,100),friction:vec(3,0,2),quantity_evidence:obj(Object.fromEntries(['geometry','pose','mass','friction'].map(k=>[k,evidence]))),parts:array(part,32,1),catalog:obj({kind:{enum:Object.keys(OPENARM_LAB_ASSETS)},dimensions_m:vec(3,.004,2)},['kind']),ports:array(ports,16),appearance:obj({rgba:vec(4,0,1),roughness:num(0,1),metalness:num(0,1)}),visual_mesh:obj({vertices_m:array(vec(3),2048,4),triangles:array(array({type:'integer',minimum:0,maximum:2047},3,3),4096,4)},['vertices_m','triangles']),limitations:array(str(500),16)},['id','position_m']);
  objectSchema.oneOf=[{required:['parts'],not:{required:['catalog']}},{required:['catalog'],not:{required:['parts']}}];
  // Structural constraints belong in the advertised schema as well as runtime checks.
  objectSchema.allOf = [
    {if:{properties:{motion:{const:'fixed'}},required:['motion']},then:{required:['fixed_reason']},else:{properties:{mass_kg:num(.005,2)},not:{required:['fixed_reason']}}},
    {if:{properties:{role:{const:'bench'}},required:['role']},then:{properties:{motion:{const:'fixed'}},required:['motion','fixed_reason']}},
    ...[['mass','mass_kg'],['friction','friction']].map(([quantity,field])=>({
      if:{properties:{quantity_evidence:{properties:{[quantity]:{properties:{source:{not:{const:'assumed'}}},required:['source']}},required:[quantity]}},required:['quantity_evidence']},
      then:{required:[field]}
    })),
    {if:{properties:{quantity_evidence:{properties:{geometry:{properties:{source:{not:{const:'assumed'}}},required:['source']}},required:['geometry']}},required:['catalog','quantity_evidence']},then:{properties:{catalog:{required:['dimensions_m']}}}}
  ];
  objectSchema.properties.friction.prefixItems=[num(0,2),num(0,.05),num(0,.01)];
  const reference=obj({mode:{enum:['none','external_reference','local_file','synthetic_fixture']},label:str(160),notes:str(1000),agent_image_access:{enum:['not_verified','declared_by_agent']},dimensions:obj({known_length_m:num(.001,10),description:str(500),source:{enum:[...QUANTITY_SOURCES]}},['known_length_m','description','source'])},['mode']);
  return obj({schema_version:{const:GENERAL_SCENE_VERSION},id,reference,objects:array(objectSchema,GENERAL_LIMITS.objects),inventory:array(obj({id,label:str(100),status:{enum:['represented','approximated','unresolved','supplemental']},object_ids:array(id,24),reason:str(500),image_bbox_uv:vec(4,0,1)},['id','status','reason']),64),relationships:array(obj({subject_id:id,relation:{enum:['supported_by','inserted_into','near','attached_to']},object_id:id,source:{enum:[...QUANTITY_SOURCES]},notes:str(500)},['subject_id','relation','object_id','source']),48),assumptions:array(str(500),32),adjustments:array(obj({object_id:id,description:str(500),reason:str(500)},['object_id','description','reason']),32)},['schema_version','id','objects']);
}
export function generalTaskSchema(){
  const schema=obj({schema_version:{const:GENERAL_TASK_VERSION},id,type:{enum:['dry_transfer','insert']},object_id:id,receiver_id:id,target_port:id,object_port:id,side:{enum:['left','right']},insertion_depth_m:num(.005,2),acknowledge_simulation_only:{const:true}},['schema_version','id','type','object_id','receiver_id','target_port','side','acknowledge_simulation_only']);
  schema.oneOf=[
    {properties:{type:{const:'dry_transfer'}},not:{anyOf:[{required:['object_port']},{required:['insertion_depth_m']}] }},
    {properties:{type:{const:'insert'}},required:['object_port']}
  ];
  return schema;
}
export function manageGeneralSceneSchema(){return {type:'object',oneOf:[
  obj({command:{const:'stage'},expected_scene_revision:str(240),scene:generalSceneSchema()},['command','expected_scene_revision','scene']),
  obj({command:{const:'check'},stage_id:str(80),settle_seconds:num(.02,.5)},['command','stage_id']),
  obj({command:{const:'apply'},stage_id:str(80),acknowledge_reset:{const:true}},['command','stage_id','acknowledge_reset']),
  obj({command:{const:'discard'}},['command']),
]};}
export function manageGeneralTaskSchema(){return {type:'object',oneOf:[
  obj({command:{enum:['assess','define']},expected_scene_revision:str(240),task:generalTaskSchema()},['command','expected_scene_revision','task']),
  obj({command:{const:'clear'},expected_scene_revision:str(240)},['command','expected_scene_revision']),
]};}
