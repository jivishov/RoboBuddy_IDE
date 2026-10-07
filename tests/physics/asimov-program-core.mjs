import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {executeAsimovPhysicalControl as execute,createAsimovControlSchema,WEBMCP_ASIMOV_SCHEMA_VERSION as V} from '../../src/webmcp/asimov-physical-control.js';
import {parseAsimovProgram,assertProgramTiming} from '../../src/webmcp/asimov-program.js';
const envelope=command=>({schema_version:V,command});
function fixture(){
 let time=0,target=0,position=0,mutations=0,revoked=false,batches=0;
 const authority={sessionId:'test',epoch:1,sceneRevision:'test-v1',robotId:'asimov_1_23dof_physical'};
 const sim={getPhysicalAuthorityToken:()=>authority,selectedScene:{physics:{timestepSeconds:.0025}},
   getState:()=>({simulation_time_s:time,joints:{left_elbow_joint:{position_rad:position}},controller_mode:'hold'}),
   async applyPhysicalTargets(t,o={}){o.assertActive?.();target=t.left_elbow_joint;mutations++;if(o.advanceSeconds)await this.advanceTime(o.advanceSeconds,o);},
   async advanceTime(seconds,o={}){o.assertActive?.();if(o.signal?.aborted)throw new Error('cancelled');time+=seconds;batches++;position=target;o.assertActive?.();},
   getTaskEvaluation:()=>({status:'observation-only'})};
 const facade={assertActive(){if(revoked)throw new Error('Access revoked');},controlSequence:0,activeControlId:null,
   getRegistrationContext:()=>({workspaceStatus:'ready',profileId:'asimov',simulationMode:'physical_mujoco',simulationReady:true,workspaceGeneration:1,simulatorEpoch:1}),
   app:{getExecutionState:()=> 'idle',scenario:{physicalSceneId:'asimov-actuator-mounted',controller:{physicsTimestepSeconds:.0025}},sim}};
 return {facade,sim,authority,counts:()=>({time,mutations,batches}),revoke:()=>{revoked=true;}};
}
for(const bad of [
 {segments:[]},{segments:[{duration_seconds:1,targets_rad:{left_elbow_joint:1}},{duration_seconds:1,targets_rad:{invented:0}}]},
 {segments:[{duration_seconds:Infinity}]},{segments:Array(7).fill({duration_seconds:2})},
 {segments:[{duration_seconds:1,targets_rad:{left_elbow_joint:1},preserve_standing:'yes'}]},
 {segments:[{duration_seconds:1,targets_rad:{left_knee_joint:1},preserve_standing:true}]},
 {segments:[{duration_seconds:1}],root_pose:[0,0,0]},
]){
 const f=fixture();await assert.rejects(execute(f.facade,{...envelope('run_sequence'),...bad},null,1));assert.equal(f.counts().mutations,0);assert.equal(f.counts().time,0);
}
{
 const f=fixture();await assert.rejects(execute(f.facade,{...envelope('run_sequence'),segments:[{targets_rad:{left_elbow_joint:1},duration_seconds:.1},{duration_seconds:.003}]},null,1));assert.equal(f.counts().mutations,0);
 const result=await execute(f.facade,{...envelope('run_sequence'),segments:[{targets_rad:{left_elbow_joint:1},duration_seconds:.1},{duration_seconds:.1}]},null,1);
 assert.equal(result.executionStatus,'completed');assert.equal(result.completedSegments,2);assert.ok(Math.abs(result.advancedSeconds-.2)<1e-10);
 const wait=await execute(f.facade,{...envelope('wait_for_joint'),joint_id:'left_elbow_joint',target_rad:1,tolerance_rad:.01,timeout_seconds:.2,dwell_seconds:.1},null,1);
 assert.equal(wait.achieved,true);assert.equal(wait.observationView,'ground_truth');
 const timeout=await execute(f.facade,{...envelope('wait_for_joint'),joint_id:'left_elbow_joint',target_rad:2,tolerance_rad:.01,timeout_seconds:.1,dwell_seconds:.05},null,1);
 assert.equal(timeout.executionStatus,'timeout');assert.equal(timeout.achieved,false);
}
{
 const f=fixture(),a=new AbortController();a.abort();await assert.rejects(execute(f.facade,{...envelope('run_sequence'),segments:[{duration_seconds:.1,targets_rad:{left_elbow_joint:1}}]},a.signal,1));assert.equal(f.counts().mutations,0);
}
for(const kind of ['abort','revoke','epoch','workspace']){
 const f=fixture(),a=new AbortController(),advance=f.sim.advanceTime.bind(f.sim);
 f.sim.advanceTime=async(s,o)=>{await advance(s,o);if(kind==='abort')a.abort();if(kind==='revoke')f.revoke();if(kind==='epoch')f.authority.epoch++;if(kind==='workspace')f.facade.getRegistrationContext=()=>({workspaceStatus:'ready',profileId:'asimov',simulationMode:'physical_mujoco',simulationReady:true,workspaceGeneration:2,simulatorEpoch:2});};
 await assert.rejects(execute(f.facade,{...envelope('run_sequence'),segments:[{duration_seconds:.1,targets_rad:{left_elbow_joint:1}},{duration_seconds:.1,targets_rad:{left_elbow_joint:2}}]},a.signal,1));
 assert.equal(f.counts().mutations,1);assert.equal(f.counts().batches,1);assert.equal(f.facade.activeControlId,null);
}
// Schema must not reject valid oneOf branches through an empty top-level properties list.
assert.equal(createAsimovControlSchema().additionalProperties,undefined);
assert.equal(createAsimovControlSchema().oneOf.length,12);
assert.throws(()=>assertProgramTiming(parseAsimovProgram({...envelope('run_sequence'),segments:[{duration_seconds:.003}]}),.0025));
// Exercise the real simulator's advancement method without constructing a renderer.
const url=new URL('../../src/physics/asimov-physical-simulator.js',import.meta.url);
let source=await readFile(url,'utf8');
source=source.replace(/^import .* from 'https:.*';$/gm,'').replace(/from '(\.\/[^']+)'/g,(_,p)=>`from '${new URL(p,url).href}'`);
source=source.replaceAll('import.meta.url',JSON.stringify(url.href));
const {AsimovPhysicalSimulator}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
for(const kind of ['abort','revoke','epoch','paused']){
 const sim=Object.create(AsimovPhysicalSimulator.prototype);sim.assertReady=()=>{};sim.isReady=()=>true;sim.selectedScene={physics:{timestepSeconds:.0025}};sim.lastObservation={simulationTimeSeconds:0};
 const a=new AbortController();let calls=0;
 sim.session={epoch:1,async advanceSteps(n){calls++;if(kind!=='paused')sim.lastObservation.simulationTimeSeconds+=n*.0025;if(kind==='abort')a.abort();if(kind==='epoch')this.epoch++;return {};}};
 await assert.rejects(sim.advanceTime(2,{signal:a.signal,assertActive:()=>{if(kind==='revoke'&&calls)throw new Error('revoked');}}));
 assert.equal(calls,1);assert.ok(sim.lastObservation.simulationTimeSeconds<=.05+1e-9);
}
console.log('Asimov WebMCP programs: atomic validation, achieved/timeout waits, strict schemas, captured ownership and <=50 ms cancellation batches: OK');
