import assert from 'node:assert/strict';
import { PhysicalPythonRuntime } from '../../src/runtime/physical-python-runtime.js';
import { ASIMOV_TASKS, ASIMOV_WORKSPACES, asimovWorkspaceFiles } from '../../src/physics/asimov-workspaces.js';
import { executeAsimovPhysicalControl as execute, WEBMCP_ASIMOV_SCHEMA_VERSION as version } from '../../src/webmcp/asimov-physical-control.js';

// User catalog cleanup must not delete or mutate the three source/reference fixtures.
assert.deepEqual(ASIMOV_TASKS.map(task=>task.id),[
  'asimov-actuator-mounted','asimov-actuator-freebase','asimov-standing',
  'asimov-sensor-standing','asimov-sensor-standing-delay','asimov-sensor-standing-ankle-stress',
]);
assert.equal(ASIMOV_TASKS[0].title,'Asimov 1 — Actuator Lab (experimental)');
assert.equal(ASIMOV_TASKS[1].title,'Asimov 1 — Whole-Body Dynamics (experimental)');
for(const id of ['asimov-mounted','asimov-freebase','asimov-drop']) {
  assert.ok(ASIMOV_WORKSPACES[id],`${id} reference fixture must remain registered`);
  assert.equal(ASIMOV_WORKSPACES[id].catalogVisibility,'internal-reference');
  assert.equal(ASIMOV_TASKS.some(task=>task.id===id),false,`${id} must stay out of the normal task catalog`);
}
for(const task of ASIMOV_TASKS) assert.equal(task.catalogVisibility,'user');

// The longer, declared standing deadline must not change the default or weaken Stop.
let worker;
const runtime = new PhysicalPythonRuntime({
  workerFactory: () => (worker = {postMessage(){},terminate(){this.terminated=true;}}),
  bridgeFactory: () => ({async cancel(){},dispose(){}}),
});
for (const limit of [undefined,120000]) {
  const timers=[]; const original=globalThis.setTimeout;
  let completion;
  try {
    globalThis.setTimeout=(callback,delay)=>{timers.push({callback,delay});return {unref(){}};};
    completion=runtime.start({'main.py':'while True: pass'}, {workspaceEpoch:1, ...(limit==null?{}:{runTimeoutMs:limit})});
  } finally {globalThis.setTimeout=original;}
  assert.equal(timers[0].delay,limit??30000);
  for (const invalid of [0,-1,1.5,NaN,Infinity,120001,'120000']) {
    await assert.rejects(runtime.start({'main.py':'pass'},{workspaceEpoch:2,runTimeoutMs:invalid}),e=>e.code==='INVALID_ARGUMENT');
    assert.equal(runtime.isActive(),true,'invalid deadline must not replace the current run');
  }
  const rejected=assert.rejects(completion,e=>e.code==='OPERATION_CANCELLED');
  await runtime.cancel('STOP',{immediate:true});await rejected;
  assert.equal(worker.terminated,true); assert.equal(runtime.isActive(),false);
}
assert.equal(runtime.runTimeoutMs,30000);
assert.equal(ASIMOV_WORKSPACES['asimov-standing'].executionBudget.pythonWallTimeMs,120000);
assert.equal(ASIMOV_WORKSPACES['asimov-mounted'].executionBudget.pythonWallTimeMs,30000);
const starter=asimovWorkspaceFiles(ASIMOV_WORKSPACES['asimov-standing'])['main.py'];
assert.match(starter,/range\(12\)/);assert.match(starter,/wait_sim\(1\.0\)/);
assert.match(starter,/stand\(controller_id="asimov-stance-feedback-v1"/);

// The real public command dispatcher must carry experimental evidence in its state.
let assessment={status:'running',success:false,validDwellSeconds:0};
const actuator={id:'experimental',continuousOnly:true};
const authority={sessionId:'test',epoch:1,sceneRevision:'v1',robotId:'asimov_1_23dof_physical'};
const facade={
  assertActive(epoch){assert.equal(epoch,9);},
  controlSequence:0,activeControlId:null,
  getRegistrationContext(){return {workspaceStatus:'ready',profileId:'asimov',simulationMode:'physical_mujoco',simulationReady:true,workspaceGeneration:1,simulatorEpoch:1};},
  app:{getExecutionState:()=> 'idle',scenario:{physicalSceneId:'asimov-standing'},sim:{
    getPhysicalAuthorityToken:()=>authority,
    getState:()=>({simulation_time_s:0,controller_mode:'asimov-stance-feedback-v1',actuator_model:actuator,standing_assessment:assessment}),
    getTaskEvaluation:()=>assessment,
    async engageStand(){},
    async stop(){assessment={...assessment,status:'failed'};},
  }},
};
for (const command of ['read_state','engage_stand','stop']) {
  const result=await execute(facade,{schema_version:version,command},null,9);
  assert.equal(result.observedState.actuator_model.continuousOnly,true);
  assert.equal(result.observedState.standing_assessment.status,command==='stop'?'failed':'running');
  assert.notEqual(result.observedState.actuator_model,actuator);
  assert.notEqual(result.observedState.standing_assessment,assessment);
}
console.log('Asimov browser boundaries: six-task user catalog, retained reference fixtures, bounded per-trial deadlines, immediate Stop, coarse Python polling, and WebMCP evidence metadata: OK');
