import { ASIMOV_SOURCE } from '../physics/asimov-generated.js';
export const ASIMOV_UPPER_BODY_JOINTS=Object.freeze(ASIMOV_SOURCE.joints.slice(12).map(j=>j.id));
export const ASIMOV_PROGRAM_REVISION='robobuddy.asimov.program.v1';
export const ASIMOV_PROGRAM_LIMITS=Object.freeze({maxSegments:32,maxDurationSeconds:12,maxSegmentSeconds:2,maxWaitSeconds:2,maxWallSeconds:120,maxBatchSeconds:0.05});
const ranges=Object.freeze(Object.fromEntries(ASIMOV_SOURCE.joints.map(j=>[j.id,j.rangeRad])));
const fail=m=>{throw new RangeError(m);};
function object(v,keys,label) {
  if(!v||typeof v!=='object'||Array.isArray(v)||(Object.getPrototypeOf(v)!==Object.prototype&&Object.getPrototypeOf(v)!==null))fail(`${label} must be a plain object`);
  for(const k of Object.keys(v))if(!keys.includes(k))fail(`Unexpected ${label} field: ${k}`);
}
function number(v,min,max,label) {if(!Number.isFinite(v)||v<min||v>max)fail(`${label} must be ${min}..${max}`);return v;}
export function validateProgramTargets(v) {
  object(v,Object.keys(ranges),'targets_rad');
  if(!Object.keys(v).length)fail('targets_rad must not be empty');
  for(const [id,value] of Object.entries(v))number(value,...ranges[id],id);
  return structuredClone(v);
}
export function parseAsimovProgram(input) {
  if(input.command==='run_sequence') {
    object(input,['schema_version','command','segments'],'sequence');
    if(!Array.isArray(input.segments)||!input.segments.length||input.segments.length>32)fail('Provide 1..32 segments');
    let totalSeconds=0;
    const segments=input.segments.map(s=>{
      object(s,['targets_rad','duration_seconds','preserve_standing'],'segment');
      if(s.preserve_standing!==undefined&&typeof s.preserve_standing!=='boolean')fail('preserve_standing must be boolean');
      if(s.preserve_standing && (!s.targets_rad || Object.keys(s.targets_rad).some(id=>!ASIMOV_UPPER_BODY_JOINTS.includes(id))))fail('preserve_standing requires waist/arm targets only');
      const seconds=number(s.duration_seconds,Number.MIN_VALUE,2,'duration_seconds');totalSeconds+=seconds;
      return {seconds,preserveStanding:s.preserve_standing===true,...(Object.hasOwn(s,'targets_rad')?{targetsRad:validateProgramTargets(s.targets_rad)}:{})};
    });
    if(totalSeconds>12+1e-10)fail('Sequence exceeds 12 simulation seconds');
    return {command:'run_sequence',segments,totalSeconds};
  }
  if(input.command==='wait_for_joint') {
    object(input,['schema_version','command','joint_id','target_rad','tolerance_rad','dwell_seconds','timeout_seconds'],'joint wait');
    if(typeof input.joint_id!=='string'||!Object.hasOwn(ranges,input.joint_id))fail('Unknown joint_id');
    const targetRad=number(input.target_rad,...ranges[input.joint_id],'target_rad');
    const toleranceRad=number(input.tolerance_rad,0.001,0.25,'tolerance_rad');
    const timeoutSeconds=number(input.timeout_seconds,0.005,2,'timeout_seconds');
    const dwellSeconds=number(input.dwell_seconds===undefined?0.1:input.dwell_seconds,0.005,0.5,'dwell_seconds');
    if(dwellSeconds>timeoutSeconds)fail('dwell_seconds exceeds timeout_seconds');
    return {command:'wait_for_joint',jointId:input.joint_id,targetRad,toleranceRad,timeoutSeconds,dwellSeconds};
  }
  fail('Unknown programming command');
}
export function assertProgramTiming(program,dt) {
  if(!Number.isFinite(dt)||dt<=0)fail('Unknown scene timestep');
  const values=program.command==='run_sequence'?program.segments.map(s=>s.seconds):[program.timeoutSeconds,program.dwellSeconds];
  for(const s of values)if(Math.round(s/dt)<1||Math.abs(Math.round(s/dt)*dt-s)>1e-9)fail('Every duration must align with the selected physics timestep');
}
export function asimovProgramSchemas(version,targets) {
 const base=command=>({schema_version:version,command:{type:'string',const:command}});
 return [
  {type:'object',properties:{...base('run_sequence'),segments:{type:'array',minItems:1,maxItems:32,items:{type:'object',properties:{targets_rad:targets,preserve_standing:{type:'boolean',description:'Keep an active standing controller while changing only waist/arm targets. A fall can still occur.'},duration_seconds:{type:'number',exclusiveMinimum:0,maximum:2}},required:['duration_seconds'],additionalProperties:false}}},required:['schema_version','command','segments'],additionalProperties:false},
  {type:'object',properties:{...base('wait_for_joint'),joint_id:{type:'string',enum:Object.keys(ranges)},target_rad:{type:'number'},tolerance_rad:{type:'number',minimum:.001,maximum:.25},dwell_seconds:{type:'number',minimum:.005,maximum:.5},timeout_seconds:{type:'number',minimum:.005,maximum:2}},required:['schema_version','command','joint_id','target_rad','tolerance_rad','timeout_seconds'],additionalProperties:false},
 ];
}
/** Finite program execution against one captured backend. Ground-truth evaluation is explicit. */
export async function executeAsimovProgram(sim,program,{dt,guard,advance}) {
 assertProgramTiming(program,dt);guard();
 if(program.command==='run_sequence') {
   let standing=['asimov-stance-feedback-v1','asimov-sensor-stance-v2'].includes(sim.getState().controller_mode) && sim.getState().standing_assessment?.status!=='failed';
   for(const s of program.segments)if(s.targetsRad){if(s.preserveStanding&&!standing)throw new Error('Sequence requires an already active standing trial');if(!s.preserveStanding)standing=false;}
   const start=sim.getState().simulation_time_s, samples=[];
   for(let i=0;i<program.segments.length;i++) {
     const s=program.segments[i];guard();
     if(s.targetsRad) {await (s.preserveStanding?sim.applyStandingTargets.bind(sim):sim.applyPhysicalTargets.bind(sim))(s.targetsRad,{maxSteps:Math.ceil(program.totalSeconds/dt),assertActive:guard});guard();}
     await advance(s.seconds);guard();
     const state=sim.getState();
     samples.push({segment:i+1,simulationTimeSeconds:state.simulation_time_s,
       measuredPositionsRad:Object.fromEntries(Object.entries(state.joints).map(([id,j])=>[id,j.position_rad])),
       standingStatus:state.standing_assessment?.status??null});
   }
   return {programRevision:ASIMOV_PROGRAM_REVISION,executionStatus:'completed',completedSegments:samples.length,
     advancedSeconds:sim.getState().simulation_time_s-start,samples,
     note:'All segments executed; this is not a claim of target attainment or task success. Ordinary joint commands release standing. Use preserve_standing with waist/arm targets or duration-only segments to retain it.'};
 }
 const start=sim.getState().simulation_time_s, maxSteps=Math.round(program.timeoutSeconds/dt),required=Math.round(program.dwellSeconds/dt);
 let inside=0,executed=0,error=null;
 const measure=()=>{
   const p=sim.getState()?.joints?.[program.jointId]?.position_rad;
   if(!Number.isFinite(p))throw new Error('Measured joint position unavailable');
   error=Math.abs(p-program.targetRad);return error<=program.toleranceRad;
 };
 let previousInside=measure();
 // Per-step observation avoids claiming a continuous dwell from sparse endpoint samples.
 while(executed<maxSteps) {
   guard();await advance(dt);guard();executed++;
   const nowInside=measure();inside=nowInside&&previousInside?inside+1:0;previousInside=nowInside;
   if(inside>=required)break;
 }
 const achieved=inside>=required;
 return {programRevision:ASIMOV_PROGRAM_REVISION,executionStatus:achieved?'achieved':'timeout',achieved,
   jointId:program.jointId,targetRad:program.targetRad,errorRad:error,validDwellSeconds:inside*dt,
   advancedSeconds:sim.getState().simulation_time_s-start,observationView:'ground_truth',
   note:'Wait observes the existing controller; it does not issue a new target or reset the robot.'};
}
