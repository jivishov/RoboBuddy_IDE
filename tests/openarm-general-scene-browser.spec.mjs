import {test,expect} from '@playwright/test';
import {readFile,writeFile} from 'node:fs/promises';
const scene=JSON.parse(await readFile(new URL('./fixtures/general-scenes/novel-adapter.json',import.meta.url),'utf8'));
const task={schema_version:'robobuddy.lab.task.v2',id:'transfer',type:'dry_transfer',object_id:'novel_adapter',receiver_id:'receiver',target_port:'receiving_top',side:'left',acknowledge_simulation_only:true};
async function setup(page){
  await page.goto('/tests/fixtures/openarm.html');await page.waitForFunction(()=>window.ready,null,{timeout:60000});
  await page.evaluate(async()=>{
    const {createWebMcpRegistration}=await import('/src/webmcp/register-ide-tools.js');
    const tools=new Map();window.activeTools=tools;
    Object.defineProperty(document,'modelContext',{configurable:true,value:{registerTool(tool,{signal}){tools.set(tool.name,tool);signal.addEventListener('abort',()=>tools.delete(tool.name));}}});
    let epoch=0,executionState='idle';window.app={profileId:'openarm',workspaceGeneration:1,runToken:0,
      sim:{backend:sim,getState:()=>sim.getState(),getPhysicalSession:()=>sim.getPhysicalSession(),getPhysicalAuthorityToken:()=>sim.getPhysicalAuthorityToken(),applyPhysicalTargets:(...a)=>sim.applyPhysicalTargets(...a)},
      getExecutionState:()=>executionState,beginExecution(){if(executionState!=='idle')return null;executionState='running';return ++this.runToken;},finishExecution(t){if(t===this.runToken)executionState='idle';},cancelExecution(){this.runToken++;executionState='idle';},resetSimulation:()=>sim.reset(),setStatus(){},renderPanels(){}};
    window.facade={app,controlSequence:0,activeControlId:null,assertActive(e){if(e!==epoch)throw Error('stale agent access');},setRegistrationEpoch(e){epoch=e;},getRegistrationContext:()=>({profileId:app.profileId,workspaceStatus:'ready',simulationReady:sim.isReady(),simulationMode:'physical_mujoco',workspaceGeneration:app.workspaceGeneration,simulatorEpoch:1}),shouldRegisterMicroduckControl:()=>false};
    window.registration=createWebMcpRegistration(facade);await registration.setAccess('assist');
    window.call=(name,input,signal)=>tools.get(name).execute(input,{signal});
    const header=document.createElement('div');header.className='sim-head';document.body.prepend(header);
    const {installGeneralLabBuilder}=await import('/src/ui/general-lab-builder.js');installGeneralLabBuilder(app);
  });
}
const call=(page,name,input)=>page.evaluate(({name,input})=>window.call(name,input),{name,input});
const inspect=page=>call(page,'inspect_openarm_scene',{});
async function stage(page,s){const state=await inspect(page);const result=await call(page,'manage_openarm_scene',{command:'stage',expected_scene_revision:state.authority.sceneRevision,scene:s});expect(result.ok,JSON.stringify(result)).toBe(true);return result.result.id;}
async function apply(page,id){const result=await call(page,'manage_openarm_scene',{command:'apply',stage_id:id,acknowledge_reset:true});expect(result.ok,JSON.stringify(result)).toBe(true);return result;}

test('registered general builder validates novel geometry and completes a causal robot transfer',async({page},info)=>{
  test.setTimeout(240000);const errors=[];page.on('pageerror',e=>errors.push(String(e)));await setup(page);
  const schema=await call(page,'inspect_openarm_scene',{view:'schema'});expect(schema.constructionShapes).toContain('convex_mesh');expect(schema.taskSchema.oneOf).toHaveLength(2);
  const before=await page.evaluate(()=>sim.getState());const id=await stage(page,scene);
  expect(await page.evaluate(()=>sim.getState())).toEqual(before);
  const check=await call(page,'manage_openarm_scene',{command:'check',stage_id:id,settle_seconds:.3});
  expect(check.ok,JSON.stringify(check)).toBe(true);expect(check.result.activeSceneUnchanged).toBe(true);
  expect(await page.evaluate(()=>sim.getState())).toEqual(before);
  await apply(page,id);const built=await inspect(page);
  expect(built.objects.find(x=>x.id==='novel_adapter').kind).toBe('constructed');expect(built.report.imageReconstructionVerified).toBe(false);
  expect(await page.evaluate(()=>sim.getWorkcellState().geometryIds.includes('cell_table'))).toBe(false);
  const definition=await call(page,'manage_openarm_task',{command:'define',expected_scene_revision:built.authority.sceneRevision,task});expect(definition.ok,JSON.stringify(definition)).toBe(true);
  const tool=p=>({side:'left',position_m:p,quaternion_wxyz:[Math.SQRT1_2,0,-Math.SQRT1_2,0]});
  const segments=[{duration_seconds:.2},
    {duration_seconds:4,tool:tool([.554,.1535,1.068])},
    {duration_seconds:3,targets_rad:{openarm_left_finger_joint1:.08}},
    {duration_seconds:4,tool:tool([.554,.1535,1.25])},
    {duration_seconds:4,tool:tool([.67,.1535,1.25])},
    {duration_seconds:4,tool:tool([.67,.1535,1.093])},
    {duration_seconds:3,targets_rad:{openarm_left_finger_joint1:.65}},
    {duration_seconds:4,tool:tool([.67,.1535,1.27])},
    {duration_seconds:.5,wait_for:{type:'authored_task_complete',timeout_seconds:1,dwell_seconds:.2}}];
  const run=await call(page,'run_openarm_program',{schema_version:'robobuddy.openarm.program.v1',expected_scene_revision:built.authority.sceneRevision,segments});
  expect(run.ok,JSON.stringify(run)).toBe(true);
  const evidence=await call(page,'inspect_openarm_scene',{view:'evidence'});
  expect(evidence.task.success,JSON.stringify(evidence.task)).toBe(true);
  expect(Object.values(evidence.task.observedSequence).every(Boolean)).toBe(true);
  expect(evidence.task.hardwareValidated).toBe(false);
  expect(await page.evaluate(()=>sim.canvas.dataset.physicalTaskSuccess)).toBe('true');
  expect(await page.evaluate(()=>sim.canvas.dataset.physicalTaskScope)).toBe('authored');
  expect(await page.evaluate(()=>sim.getContacts().task_success)).toBe(true);
  expect(await page.evaluate(()=>sim.getTelemetry().authored_task_success)).toBe(true);
  expect(await page.evaluate(()=>sim.getContacts().flask_grasp_seen)).toBeUndefined();
  await page.screenshot({path:info.outputPath('novel-adapter-transfer.png')});
  await writeFile(info.outputPath('general-scene-webmcp-evidence.json'),JSON.stringify({check,built,run,evidence},null,2));
  const reset=await call(page,'control_openarm_simulation',{schema_version:'robobuddy.openarm.physical.v1',command:'reset'});expect(reset.ok).toBe(true);
  expect((await inspect(page)).task.success).toBe(false);expect((await inspect(page)).task.status).toBe('invalidated');
  expect(await page.evaluate(()=>sim.canvas.dataset.physicalTaskSuccess)).toBe('false');
  expect(await page.evaluate(()=>sim.canvas.dataset.physicalTaskLift)).toBe('false');
  expect(await page.evaluate(()=>sim.getContacts().task_success)).toBe(false);
  expect(await page.evaluate(()=>sim.getTelemetry().authored_task_success)).toBe(false);
  expect(errors).toEqual([]);
});

test('candidate failures, stale revisions, cancellation and revocation preserve physical authority',async({page})=>{
  await setup(page);await apply(page,await stage(page,scene));
  const before=await page.evaluate(()=>sim.getState());
  const overlap=structuredClone(scene);overlap.objects.push({...overlap.objects[1],id:'collision'});
  const id=await stage(page,overlap);
  for(const command of ['check','apply']){const result=await call(page,'manage_openarm_scene',{command,stage_id:id,...(command==='apply'?{acknowledge_reset:true}:{})});expect(result.ok).toBe(false);expect(await page.evaluate(()=>sim.getState())).toEqual(before);}
  const stale=await call(page,'manage_openarm_scene',{command:'stage',expected_scene_revision:'old',scene});expect(stale.ok).toBe(false);
  const abort=await page.evaluate(async s=>{const b=sim.getState(),e=sim.getGeneralSceneState(),staged=await call('manage_openarm_scene',{command:'stage',expected_scene_revision:e.authority.sceneRevision,scene:s});const a=new AbortController();a.abort();const result=await call('manage_openarm_scene',{command:'check',stage_id:staged.result.id},a.signal);return {result,before:b,after:sim.getState()};},scene);
  expect(abort.result.ok).toBe(false);expect(abort.after).toEqual(abort.before);
  const revoked=await page.evaluate(async()=>{const staleTool=activeTools.get('manage_openarm_scene'),b=sim.getState();await registration.setAccess('off');const result=await staleTool.execute({command:'discard'},{});return {result,before:b,after:sim.getState(),count:activeTools.size};});
  expect(revoked.result.ok).toBe(false);expect(revoked.count).toBe(0);expect(revoked.after).toEqual(revoked.before);
  await page.evaluate(()=>registration.setAccess('assist'));
  const stageId=await stage(page,scene);await call(page,'control_openarm_simulation',{schema_version:'robobuddy.openarm.physical.v1',command:'reset'});
  const bad=await call(page,'manage_openarm_scene',{command:'apply',stage_id:stageId,acknowledge_reset:true});expect(bad.ok).toBe(false);
  const busy=await page.evaluate(async s=>{const token=app.beginExecution();try{return await call('manage_openarm_scene',{command:'stage',expected_scene_revision:sim.getPhysicalAuthorityToken().sceneRevision,scene:s});}finally{app.finishExecution(token);}},scene);expect(busy.ok).toBe(false);
});

test('human review imports only drafts, retains unresolved inventory, exports and toggles collision geometry',async({page},info)=>{
  await setup(page);
  const s=structuredClone(scene);s.reference={mode:'external_reference',label:'Unseen source photograph (test declaration only)'};
  s.inventory=[{id:'visible',status:'approximated',object_ids:s.objects.map(o=>o.id),reason:'Synthetic authored geometry, not an image reconstruction benchmark'},{id:'occluded',label:'Occluded machine',status:'unresolved',reason:'Insufficient visual information'}];
  const body=s.objects[1];body.visual_mesh={vertices_m:[[-.02,-.02,0],[.03,-.02,0],[-.02,.02,0],[-.02,-.02,.085]],triangles:[[0,2,1],[0,1,3],[1,2,3],[2,0,3]]};body.appearance={rgba:[.8,.45,.25,1]};
  await page.locator('#generalLabOpen').click();const before=await page.evaluate(()=>sim.getState());
  await page.locator('#generalLabImport').setInputFiles({name:'novel-scene.json',mimeType:'application/json',buffer:Buffer.from(JSON.stringify(s))});
  await expect(page.locator('#generalLabMessage')).toContainText('Imported into the editor only');expect(await page.evaluate(()=>sim.getState())).toEqual(before);
  await page.locator('#generalLabStage').click();await expect(page.locator('#generalLabReport')).toContainText('Occluded machine');
  expect(await page.evaluate(()=>sim.getState())).toEqual(before);
  await page.locator('#generalLabCheck').click();await expect(page.locator('#generalLabReport')).toContainText('physicalCompilePassed');
  await page.locator('#generalLabApply').click();await expect.poll(()=>page.evaluate(()=>sim.sceneMode)).toBe('authored');
  const physical=await page.evaluate(()=>sim.getState());
  const visual=await page.evaluate(()=>({visible:sim.presentation.visualMeshes.map(m=>m.visible),audit:sim.getPresentationAudit()}));expect(visual.visible.every(Boolean)).toBe(true);expect(visual.audit.sharedCollisionGeometry).toBe(false);
  await page.locator('#generalLabCollision').click();expect(await page.evaluate(()=>sim.presentation.visualMeshes.every(m=>!m.visible))).toBe(true);expect(await page.evaluate(()=>sim.getState())).toEqual(physical);
  const downloaded=page.waitForEvent('download');await page.locator('#generalLabExport').click();const download=await downloaded;const exported=JSON.parse(await readFile(await download.path(),'utf8'));expect(exported.auto_start).toBe(false);expect(exported.scene.inventory[1].status).toBe('unresolved');
  await page.locator('#generalLabImport').setInputFiles({name:'project.json',mimeType:'application/json',buffer:Buffer.from(JSON.stringify({...exported,auto_start:true}))});await expect(page.locator('#generalLabMessage')).toContainText('auto_start:false');expect(await page.evaluate(()=>sim.getState())).toEqual(physical);
  await page.screenshot({path:info.outputPath('general-builder-review.png')});
  await page.locator('#generalLabPanel').press('Escape');await expect(page.locator('#generalLabPanel')).not.toBeVisible();
});

test('photo-guided and novel-equipment examples compile through WebMCP without catalog lookups',async({page},info)=>{
  test.setTimeout(120000);await setup(page);
  for(const name of ['titration-construction','novel-equipment-bench']){
    const s=JSON.parse(await readFile(new URL(`./fixtures/general-scenes/${name}.json`,import.meta.url),'utf8'));
    const id=await stage(page,s),check=await call(page,'manage_openarm_scene',{command:'check',stage_id:id});expect(check.ok,JSON.stringify(check)).toBe(true);
    await apply(page,id);const state=await inspect(page);expect(state.objects.every(o=>o.kind==='constructed')).toBe(true);expect(state.report.imageReconstructionVerified).toBe(false);
    await page.screenshot({path:info.outputPath(`${name}.png`)});
    await writeFile(info.outputPath(`${name}.json`),JSON.stringify({check,state},null,2));
  }
});

// A changed editor draft must never apply an older preview unnoticed.
test('human check/apply requires the current draft to match the staged scene',async({page},info)=>{
  await setup(page);await page.locator('#generalLabOpen').click();
  await page.locator('#generalLabScene').fill(JSON.stringify(scene));
  await page.locator('#generalLabStage').click();await expect(page.locator('#generalLabMessage')).toContainText('Operation completed');
  const before=await page.evaluate(()=>sim.getState());
  const changed=structuredClone(scene);changed.id='edited_after_staging';changed.objects[1].label='Edited sample';
  await page.locator('#generalLabScene').fill(JSON.stringify(changed));
  for(const button of ['#generalLabCheck','#generalLabApply']){
    await page.locator(button).click();await expect(page.locator('#generalLabMessage')).toContainText('Draft differs from the staged scene');
    expect(await page.evaluate(()=>sim.getState())).toEqual(before);
  }
  await page.locator('#generalLabStage').click();await expect(page.locator('#generalLabMessage')).toContainText('Operation completed');
  await page.locator('#generalLabApply').click();await expect.poll(()=>page.evaluate(()=>sim.generalSpec?.id)).toBe(changed.id);
  await page.screenshot({path:info.outputPath('review-draft-match.png')});
});
