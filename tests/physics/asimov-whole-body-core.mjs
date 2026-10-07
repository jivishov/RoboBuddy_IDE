import assert from 'node:assert/strict';
import { ASIMOV_SOURCE } from '../../src/physics/asimov-generated.js';
import {
  parseAsimovWholeBody,
  executeAsimovWholeBodyMotion,
  ASIMOV_WHOLE_BODY_LIMITS,
} from '../../src/webmcp/asimov-whole-body.js';

const joints=()=>Object.fromEntries(ASIMOV_SOURCE.joints.map(j=>[j.id,{position_rad:j.referenceRad,velocity_rad_s:0}]));
function fixture({rootMode='free-base',dropAfterAdvance=false,support='double',sceneId='asimov-actuator-freebase',actuationEnabled=true,controllerMode='joint-hold',quaternion=[1,0,0,0]}={}) {
  let time=0;
  let state={
    simulation_time_s:0,
    root:{mode:rootMode,position_m:[0,0,.63],quaternion_wxyz:[...quaternion],angular_velocity_rad_s:[0,0,0],tilt_rad:0},
    joints:joints(),actuation_enabled:actuationEnabled,controller_mode:controllerMode,
  };
  const commands=[];let advances=0;
  const contacts=()=>({
    leftFootFloor:support==='double'||support==='left'?[{geoms:['left_foot1_collision','floor']}]:[],
    rightFootFloor:support==='double'||support==='right'?[{geoms:['right_foot1_collision','floor']}]:[],
    otherBodyFloor:[],
  });
  const sim={
    selectedScene:{id:sceneId},
    getState:()=>structuredClone(state),
    getContacts:()=>structuredClone(contacts()),
    async applyPhysicalTargets(targets,o={}) {
      o.assertActive?.();commands.push(structuredClone(targets));
      for(const [id,value] of Object.entries(targets)) state.joints[id].position_rad=value;
      state.controller_mode='joint-hold';
    },
  };
  const advance=async(seconds)=>{
    time+=seconds;advances++;
    // The executor must report achieved root motion rather than infer success from the intent label.
    state.root.position_m[0]+=seconds*.1;
    if(dropAfterAdvance&&advances===1){state.root.position_m[2]=.30;state.root.tilt_rad=.8;}
    state.simulation_time_s=time;
  };
  return {sim,advance,commands,counts:()=>({time,advances})};
}
const guard=()=>{};
const envelope=(extra={})=>({command:'run_whole_body_motion',...extra});

// Input is atomic and narrow: no root writes, unknown joints, unbounded duration or undeclared modes/types.
for(const bad of [
  {...envelope({keyframes:[{duration_seconds:.1,targets_rad:{left_knee_joint:.1}}]}),root_pose:[0,0,1]},
  envelope({keyframes:[{duration_seconds:.1,targets_rad:{invented_joint:.1}}]}),
  envelope({stabilization:'magic',keyframes:[{duration_seconds:.1,targets_rad:{left_knee_joint:.1}}]}),
  envelope({motion_intent:{kind:'walk'},keyframes:[{duration_seconds:.1,targets_rad:{left_knee_joint:.1}}]}),
  envelope({keyframes:[{duration_seconds:.1,targets_rad:{left_knee_joint:.1},phase:{label:'bad'}}]}),
  envelope({keyframes:Array(7).fill({duration_seconds:2,targets_rad:{left_knee_joint:.1}})}),
  envelope({keyframes:[]}),
]) assert.throws(()=>parseAsimovWholeBody(bad));

// Assistance is opt-in: an omitted stabilization field means the agent owns the trajectory.
{
  const parsed=parseAsimovWholeBody(envelope({keyframes:[{duration_seconds:.1,targets_rad:{left_knee_joint:.05}}]}));
  assert.equal(parsed.stabilization,'none');
}

// A full-body keyframe is interpolated at 20 Hz and every command remains a 23-joint bounded target.
{
  const f=fixture();
  const program=parseAsimovWholeBody(envelope({motion_intent:'walk',stabilization:'none',keyframes:[{
    phase:'small bilateral bend',duration_seconds:.1,support:'left',
    targets_rad:{left_knee_joint:.1,right_knee_joint:-.1,left_elbow_joint:.9,right_elbow_joint:-.9},
  }]}));
  const result=await executeAsimovWholeBodyMotion(f.sim,program,{dt:.0025,guard,advance:f.advance});
  assert.equal(result.executionStatus,'completed');
  assert.equal(result.motionIntent,'walk');
  assert.equal(result.completedKeyframes,1);
  assert.ok(Math.abs(result.advancedSeconds-.1)<1e-12);
  assert.equal(f.commands.length,2);
  assert.equal(Object.keys(f.commands[0]).length,ASIMOV_SOURCE.joints.length);
  assert.ok(Math.abs(result.horizontalDisplacementM-.01)<1e-12);
  assert.equal(result.samples[0].expectedSupport,'left');
  assert.equal(result.samples[0].observedSupport,'double');
  assert.equal(result.samples[0].supportMatched,false);
  assert.match(result.note,/not proof/i);
}

// The optional ground-truth stabilizer is explicit and bounded at the ankle target layer only.
{
  const f=fixture();
  const program=parseAsimovWholeBody(envelope({stabilization:'ground_truth',keyframes:[{duration_seconds:.05,targets_rad:{left_knee_joint:.05,right_knee_joint:-.05}}]}));
  const result=await executeAsimovWholeBodyMotion(f.sim,program,{dt:.0025,guard,advance:f.advance});
  assert.equal(result.stabilization,'ground_truth');
  assert.equal(f.commands.length,1);
  for(const id of ['left_ankle_pitch_joint','right_ankle_pitch_joint','left_ankle_roll_joint','right_ankle_roll_joint']) {
    const [lo,hi]=ASIMOV_SOURCE.joints.find(j=>j.id===id).rangeRad;
    assert.ok(f.commands[0][id]>=lo&&f.commands[0][id]<=hi);
  }
}

// Supervisor + nominal trajectory cannot bypass any source target-rate bound.
{
  const half=-.1;
  const f=fixture({quaternion:[Math.cos(half),Math.sin(half),0,0]});
  const program=parseAsimovWholeBody(envelope({stabilization:'ground_truth',keyframes:[{duration_seconds:.1,targets_rad:{left_ankle_roll_joint:.09,left_knee_joint:.10}}]}));
  const result=await executeAsimovWholeBodyMotion(f.sim,program,{dt:.0025,guard,advance:f.advance});
  assert.ok(Number.isInteger(result.commandRateLimitEvents)&&result.commandRateLimitEvents>=0);
  let previous=Object.fromEntries(ASIMOV_SOURCE.joints.map(j=>[j.id,j.referenceRad]));
  for(const command of f.commands) {
    for(const joint of ASIMOV_SOURCE.joints) {
      assert.ok(Math.abs(command[joint.id]-previous[joint.id])<=joint.velocityLimitRadS*.05+1e-10,`${joint.id} exceeded final target-rate bound`);
    }
    previous=command;
  }
}

// Velocity-limit validation happens before the first plant mutation.
{
  const f=fixture();
  const program=parseAsimovWholeBody(envelope({stabilization:'none',keyframes:[{duration_seconds:.05,targets_rad:{left_hip_roll_joint:.78}}]}));
  await assert.rejects(executeAsimovWholeBodyMotion(f.sim,program,{dt:.0025,guard,advance:f.advance}),/velocity limit/);
  assert.equal(f.commands.length,0);
  assert.equal(f.counts().time,0);
}

// Passive and disabled-actuation scenes reject before the first target; a drop lab stays a drop lab.
for(const f of [fixture({sceneId:'asimov-drop',controllerMode:'passive'}),fixture({actuationEnabled:false})]) {
  const program=parseAsimovWholeBody(envelope({keyframes:[{duration_seconds:.1,targets_rad:{left_elbow_joint:.1}}]}));
  await assert.rejects(executeAsimovWholeBodyMotion(f.sim,program,{dt:.005,guard,advance:f.advance}),/observation-only|enabled actuation/);
  assert.equal(f.commands.length,0);assert.equal(f.counts().time,0);
}

// A fall-like state aborts rather than being repaired, reset or converted into success.
{
  const f=fixture({dropAfterAdvance:true});
  const program=parseAsimovWholeBody(envelope({motion_intent:'step',stabilization:'none',keyframes:[{duration_seconds:.2,targets_rad:{left_knee_joint:.1,right_knee_joint:-.1}}]}));
  const result=await executeAsimovWholeBodyMotion(f.sim,program,{dt:.0025,guard,advance:f.advance});
  assert.equal(result.executionStatus,'aborted-instability');
  assert.match(result.instabilityReason,/pelvis|tilt/);
  assert.equal(f.commands.length,1);
  assert.equal(f.counts().advances,1);
}

// Mounted scenes reject whole-body locomotion before issuing any command.
{
  const f=fixture({rootMode:'mounted',sceneId:'asimov-mounted'});
  const program=parseAsimovWholeBody(envelope({keyframes:[{duration_seconds:.1,targets_rad:{left_elbow_joint:1}}]}));
  await assert.rejects(executeAsimovWholeBodyMotion(f.sim,program,{dt:.005,guard,advance:f.advance}),/free-base/);
  assert.equal(f.commands.length,0);
}

assert.equal(ASIMOV_WHOLE_BODY_LIMITS.maxDurationSeconds,12);
console.log('Asimov whole-body WebMCP: bounded keyframes, passive gates, target-rate bounds, measured outcomes, support evidence and fall aborts: OK');
