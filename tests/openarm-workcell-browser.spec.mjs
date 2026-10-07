import { expect, test } from '@playwright/test';
import { writeFile } from 'node:fs/promises';

test('OpenArm shared geometry and registered WebMCP equipment/program tools use real browser physics', async ({page}, testInfo) => {
  test.setTimeout(240000); const errors=[]; page.on('pageerror',e=>errors.push(String(e)));
  await page.goto('/tests/fixtures/openarm.html'); await page.waitForFunction(()=>window.ready, null, {timeout:60000});
  const alignment = await page.evaluate(() => {
    sim.renderFrame(); const state=sim.getState().observation;
    return [...sim.presentation.bodyGroups].filter(([id])=>id!=='world').map(([id,group])=>({id,actual:group.position.toArray(),expected:[state.bodies[id].positionM[0]*1000,state.bodies[id].positionM[2]*1000,-state.bodies[id].positionM[1]*1000]}));
  });
  for (const a of alignment) a.actual.forEach((v,i)=>expect(v).toBeCloseTo(a.expected[i],6));
  await page.screenshot({path:testInfo.outputPath('contact-aligned-workcell.png')});
  // WebMCP registration is exercised through a test adapter. No claim that CI's
  // Chromium implements the experimental native WebMCP interface is made.
  const names = await page.evaluate(async()=>{
    const {createWebMcpRegistration}=await import('/src/webmcp/register-ide-tools.js');
    window.tools=new Map(); Object.defineProperty(document,'modelContext',{configurable:true,value:{registerTool:async(tool,{signal})=>{tools.set(tool.name,tool);signal.addEventListener('abort',()=>tools.delete(tool.name));}}});
    let executionState='idle',access='assist',epoch=0;
    const app={sim:{backend:sim,getState:()=>sim.getState(),getPhysicalSession:()=>sim.getPhysicalSession(),getPhysicalAuthorityToken:()=>sim.getPhysicalAuthorityToken(),applyPhysicalTargets:(...a)=>sim.applyPhysicalTargets(...a)},runToken:0,
      getExecutionState:()=>executionState,beginExecution(){if(executionState!=='idle')return null;executionState='running';return ++this.runToken;},finishExecution(token){if(token===this.runToken)executionState='idle';},cancelExecution(){this.runToken++;executionState='idle';this.openarmAgentProgramActive=false;},resetSimulation:()=>sim.reset(),setStatus:()=>{},renderPanels:()=>{}};
    const facade={app,controlSequence:0,activeControlId:null,assertActive(e){if(e!==epoch||access!=='assist')throw Error('Agent access inactive');},setRegistrationEpoch(e){epoch=e;},getRegistrationContext:()=>({profileId:'openarm',workspaceStatus:'ready',simulationReady:sim.isReady(),simulationMode:'physical_mujoco',workspaceGeneration:1,simulatorEpoch:1}),shouldRegisterMicroduckControl:()=>false};
    window.testApp=app;window.facade=facade;window.registration=createWebMcpRegistration(facade);await registration.setAccess('assist');
    window.callTool=(name,input,signal)=>tools.get(name).execute(input,{signal});return [...tools.keys()];
  });
  for(const name of ['control_openarm_simulation','inspect_openarm_workcell','manage_openarm_workcell','run_openarm_program'])expect(names).toContain(name);
  const builder=await page.evaluate(async()=>{
    const inspection=await callTool('inspect_openarm_workcell',{});
    const input={schema_version:'robobuddy.openarm.equipment.v1',command:'stage',expected_scene_revision:inspection.authority.sceneRevision,equipment:[{id:'button1',kind:'button',position_m:[.57,.32,1.005]},{id:'tray1',kind:'tray',position_m:[.35,-.38,1.005]},{id:'vial1',kind:'vial',position_m:[.35,-.38,1.08]}]};
    const staged=await callTool('manage_openarm_workcell',input);const during=sim.getPhysicalAuthorityToken();const applied=await callTool('manage_openarm_workcell',{schema_version:input.schema_version,command:'apply',stage_id:staged.result.id,acknowledge_reset:true});
    return {inspection,staged,during,applied,after:sim.getWorkcellState()};
  });
  expect(builder.staged.ok).toBe(true);expect(builder.during).toEqual(builder.inspection.authority);expect(builder.applied.ok).toBe(true);
  expect(builder.applied.result.reset).toBe(true);expect(builder.after.authority.sessionId).not.toBe(builder.during.sessionId);expect(builder.after.equipment).toHaveLength(3);
  await page.screenshot({path:testInfo.outputPath('agent-created-equipment.png')});
  const rollback=await page.evaluate(async()=>{
    const before=sim.getState();const staged=await callTool('manage_openarm_workcell',{schema_version:'robobuddy.openarm.equipment.v1',command:'stage',expected_scene_revision:before.authority.sceneRevision,equipment:[{id:'bad',kind:'block',position_m:[.55,.1535,1.10]}]});
    const result=await callTool('manage_openarm_workcell',{schema_version:'robobuddy.openarm.equipment.v1',command:'apply',stage_id:staged.result.id,acknowledge_reset:true});
    const after=sim.getState();await callTool('manage_openarm_workcell',{schema_version:'robobuddy.openarm.equipment.v1',command:'discard'});return {before,after,result};
  });
  expect(rollback.result.ok).toBe(false);expect(rollback.after).toEqual(rollback.before);
  const result=await page.evaluate(async()=>{
    const tool=position_m=>({side:'left',position_m,quaternion_wxyz:[Math.SQRT1_2,0,-Math.SQRT1_2,0]});
    return callTool('run_openarm_program',{schema_version:'robobuddy.openarm.program.v1',expected_scene_revision:sim.getPhysicalAuthorityToken().sceneRevision,segments:[
      {label:'Lift clear of the reference vessel',tool:tool([.554,.1535,1.25]),duration_seconds:4},
      {label:'Move above the created button',tool:tool([.57,.32,1.23]),duration_seconds:4},
      {label:'Close the empty gripper',targets_rad:{openarm_left_finger_joint1:.02},duration_seconds:3},
      {label:'Approach the button',tool:tool([.57,.32,1.06]),duration_seconds:4},
      {label:'Press the passive spring cap',tool:tool([.57,.32,1.05]),duration_seconds:2,wait_for:{type:'equipment_joint',joint_id:'lab_button1_press',minimum:.004,maximum:.006,timeout_seconds:1,dwell_seconds:.06}},
      {label:'Withdraw and verify spring return',tool:tool([.57,.32,1.15]),duration_seconds:2,wait_for:{type:'equipment_joint',joint_id:'lab_button1_press',minimum:0,maximum:.001,timeout_seconds:1,dwell_seconds:.06}},
      {label:'Verify vial rests inside the tray',duration_seconds:.1,wait_for:{type:'supported',object_id:'lab_vial1',support_geom:'lab_tray1_base',timeout_seconds:1}},
    ]});
  });
  await writeFile(testInfo.outputPath('webmcp-equipment-program.json'),JSON.stringify(result,null,2));
  expect(result.ok,JSON.stringify(result)).toBe(true);expect(result.executionStatus).toBe('completed');expect(result.segments.filter(s=>s.condition).every(s=>s.conditionSatisfied)).toBe(true);
  const outcome=await page.evaluate(()=>({state:sim.getState(),workcell:sim.getWorkcellState(),execution:facade.app.getExecutionState()}));
  expect(outcome.execution).toBe('idle');expect(outcome.workcell.equipmentJoints[0].pressed).toBe(false);
  expect(outcome.state.observation.bodies.flask.positionM[0]).toBeCloseTo(.55,3);
  await page.screenshot({path:testInfo.outputPath('robot-equipment-interaction-completed.png')});
  const fail=await page.evaluate(()=>callTool('run_openarm_program',{schema_version:'robobuddy.openarm.program.v1',expected_scene_revision:sim.getPhysicalAuthorityToken().sceneRevision,segments:[{duration_seconds:.1,wait_for:{type:'equipment_joint',joint_id:'lab_button1_press',minimum:.004,maximum:.006,timeout_seconds:.1}}]}));
  expect(fail.ok).toBe(false);expect(fail.reason).toBe('condition-timeout');
  const cancelled=await page.evaluate(async()=>{const abort=new AbortController();const p=callTool('run_openarm_program',{schema_version:'robobuddy.openarm.program.v1',expected_scene_revision:sim.getPhysicalAuthorityToken().sceneRevision,segments:[{duration_seconds:8}]},abort.signal);setTimeout(()=>abort.abort(),30);const result=await p;return {result,state:facade.app.getExecutionState(),owner:facade.activeControlId};});
  expect(cancelled.result.ok).toBe(false);expect(cancelled.state).toBe('idle');expect(cancelled.owner).toBe(null);
  expect(errors,errors.join('\n')).toEqual([]);
});

test('OpenArm contrast reports visible markers without changing physics, including rebuilt workcells', async ({ page }, testInfo) => {
  const errors = [];
  page.on('pageerror', error => errors.push(String(error)));
  await page.goto('/tests/fixtures/openarm.html');
  await page.waitForFunction(() => window.ready, null, { timeout: 60000 });
  const result = await page.evaluate(async () => {
    const contrast = () => ({
      enabled: sim.isHighContrastSceneEnabled(),
      active: sim.canvas.dataset.highContrastScene,
      count: Number(sim.canvas.dataset.highContrastPerimeterCount),
      markers: sim.targetMarkers.map(marker => ({
        visible: marker.visible,
        presentationOnly: marker.userData.presentationOnly,
        attached: marker.parent === sim.scene,
        opacity: marker.material.opacity,
      })),
    });
    const initial = contrast();
    // Start at nonzero simulation time so an accidental reset cannot pass unnoticed.
    await sim.advanceTime(.02);
    const before = sim.getState();
    const toggles = [false, false, true, true, false].map(enabled => {
      const returned = sim.setHighContrastScene(enabled);
      sim.renderFrame();
      return { requested: enabled, returned, contrast: contrast(), state: sim.getState() };
    });
    const staged = sim.stageEquipment([{ id: 'contrast_tray', kind: 'tray', position_m: [.35, -.38, 1.005] }]);
    const stagedState = sim.getState();
    // Applying equipment is an explicit reset; contrast alone is not.
    const applied = await sim.applyStagedEquipment(staged.id, true);
    const rebuiltOff = contrast();
    const rebuiltState = sim.getState();
    sim.setHighContrastScene(true);
    sim.renderFrame();
    return { initial, before, toggles, stagedState, applied, rebuiltOff, rebuiltOn: contrast(), rebuiltState, finalState: sim.getState() };
  });
  const assertContrast = (value, enabled) => {
    expect(value.enabled).toBe(enabled);
    expect(value.active).toBe(String(enabled));
    expect(value.markers).toHaveLength(2);
    expect(value.count).toBe(enabled ? 2 : 0);
    for (const marker of value.markers) {
      expect(marker.visible).toBe(enabled);
      expect(marker.presentationOnly).toBe(true);
      expect(marker.attached).toBe(true);
      expect(marker.opacity).toBeGreaterThan(0);
    }
  };
  assertContrast(result.initial, true);
  expect(result.before.observation.simulationTimeSeconds).toBeGreaterThan(0);
  for (const toggle of result.toggles) {
    expect(toggle.returned).toBe(toggle.requested);
    assertContrast(toggle.contrast, toggle.requested);
    expect(toggle.state).toEqual(result.before);
  }
  expect(result.stagedState).toEqual(result.before);
  expect(result.applied.status).toBe('applied');
  expect(result.applied.reset).toBe(true);
  expect(result.rebuiltState.authority.sessionId).not.toBe(result.before.authority.sessionId);
  assertContrast(result.rebuiltOff, false);
  assertContrast(result.rebuiltOn, true);
  expect(result.finalState).toEqual(result.rebuiltState);
  await page.screenshot({ path: testInfo.outputPath('openarm-contrast-rebuilt-workcell.png') });
  expect(errors).toEqual([]);
});

test('WebMCP clears baseline fixtures, builds hollow laboratory assets and checks seating without claiming a transfer', async ({page}, testInfo) => {
  test.setTimeout(240000);
  const errors=[];page.on('pageerror',error=>errors.push(String(error)));
  await page.goto('/tests/fixtures/openarm.html');await page.waitForFunction(()=>window.ready,null,{timeout:60000});
  await page.evaluate(async()=>{
    const {createWebMcpRegistration}=await import('/src/webmcp/register-ide-tools.js');
    window.tools=new Map();Object.defineProperty(document,'modelContext',{configurable:true,value:{registerTool:async(tool,{signal})=>{tools.set(tool.name,tool);signal.addEventListener('abort',()=>tools.delete(tool.name));}}});
    let executionState='idle',epoch=0;
    const app={sim:{backend:sim,getState:()=>sim.getState(),getPhysicalSession:()=>sim.getPhysicalSession(),getPhysicalAuthorityToken:()=>sim.getPhysicalAuthorityToken(),applyPhysicalTargets:(...a)=>sim.applyPhysicalTargets(...a)},runToken:0,
      getExecutionState:()=>executionState,beginExecution(){if(executionState!=='idle')return null;executionState='running';return ++this.runToken;},finishExecution(token){if(token===this.runToken)executionState='idle';},cancelExecution(){this.runToken++;executionState='idle';},resetSimulation:()=>sim.reset(),setStatus:()=>{},renderPanels:()=>{}};
    const facade={app,controlSequence:0,activeControlId:null,assertActive(e){if(e!==epoch)throw Error('stale access');},setRegistrationEpoch(e){epoch=e;},getRegistrationContext:()=>({profileId:'openarm',workspaceStatus:'ready',simulationReady:sim.isReady(),simulationMode:'physical_mujoco',workspaceGeneration:1,simulatorEpoch:1}),shouldRegisterMicroduckControl:()=>false};
    window.registration=createWebMcpRegistration(facade);await registration.setAccess('assist');
    window.callTool=(name,input)=>tools.get(name).execute(input,{});
    window.stageLab=(equipment,scene_mode)=>callTool('manage_openarm_workcell',{schema_version:'robobuddy.openarm.equipment.v1',command:'stage',expected_scene_revision:sim.getPhysicalAuthorityToken().sceneRevision,equipment,...(scene_mode?{scene_mode}:{})});
    window.applyLab=staged=>callTool('manage_openarm_workcell',{schema_version:'robobuddy.openarm.equipment.v1',command:'apply',stage_id:staged.result.id,acknowledge_reset:true});
  });
  const clear=await page.evaluate(async()=>{
    const before=sim.getState(), staged=await stageLab([],'blank'), preview=sim.getState();
    const applied=await applyLab(staged);sim.renderFrame();
    const inspection=await callTool('inspect_openarm_workcell',{});
    const reset=await callTool('control_openarm_simulation',{schema_version:'robobuddy.openarm.physical.v1',command:'reset'});
    return {before,preview,staged,applied,inspection,reset,afterReset:sim.getWorkcellState(),markerCount:sim.canvas.dataset.highContrastPerimeterCount};
  });
  expect(clear.preview).toEqual(clear.before);expect(clear.applied.ok,JSON.stringify(clear.applied)).toBe(true);
  expect(clear.inspection.sceneMode).toBe('blank');expect(clear.inspection.bodies.flask).toBeUndefined();expect(clear.inspection.bodies.beaker).toBeUndefined();
  expect(clear.inspection.geometryIds).toContain('cell_table');expect(clear.inspection.geometryIds).not.toContain('left_hotplate');expect(clear.markerCount).toBe('0');
  expect(clear.reset.ok).toBe(true);expect(clear.afterReset.sceneMode).toBe('blank');expect(clear.inspection.assetCatalog.funnel).toBeTruthy();
  await page.screenshot({path:testInfo.outputPath('blank-openarm-workcell.png')});
  const photo=await page.evaluate(async()=>{
    const equipment=[
      {id:'stand',kind:'ring_stand',position_m:[.6,.32,1.005],dimensions_m:[.16,.14,.435/.72]},
      {id:'burette',kind:'burette',position_m:[.6,.32,1.18]},
      {id:'tile',kind:'tile',position_m:[.6,.32,1.017],dimensions_m:[.095,.1,.008]},
      {id:'flask',kind:'erlenmeyer_flask',position_m:[.6,.32,1.025]},
      {id:'bottle',label:'Acetic acid (illustrative)',kind:'bottle',position_m:[.35,.35,1.005]},
      {id:'waste',label:'Waste beaker',kind:'beaker',position_m:[.72,-.32,1.005]},
      {id:'funnel',kind:'funnel',position_m:[.34,-.34,1.04],quaternion_wxyz:[Math.SQRT1_2,0,Math.SQRT1_2,0]},
    ];
    const staged=await stageLab(equipment), applied=await applyLab(staged);sim.renderFrame();
    return {staged,applied,inspection:sim.getWorkcellState(),rendered:[...sim.presentation.geomMeshes.keys()]};
  });
  expect(photo.applied.ok,JSON.stringify(photo.applied)).toBe(true);expect(photo.inspection.sceneMode).toBe('blank');expect(photo.inspection.equipment).toHaveLength(7);
  expect(photo.rendered).toContain('lab_funnel_bowl_0');expect(photo.rendered).toContain('lab_burette_tube_0');expect(photo.rendered).not.toContain('flask_grip_geom');
  await page.screenshot({path:testInfo.outputPath('titration-catalog-workcell.png')});
  const seating=await page.evaluate(async()=>{
    // This setup-only drop tests bore topology. It must never be reported as a robot transfer.
    const staged=await stageLab([{id:'burette',kind:'burette',position_m:[.38,-.35,1.005]},{id:'funnel',kind:'funnel',position_m:[.38,-.35,1.410]}]);
    const applied=await applyLab(staged);
    const result=await callTool('run_openarm_program',{schema_version:'robobuddy.openarm.program.v1',expected_scene_revision:sim.getPhysicalAuthorityToken().sceneRevision,segments:[{duration_seconds:2,wait_for:{type:'funnel_seated',object_id:'lab_funnel',receiver_id:'lab_burette',dwell_seconds:.2,timeout_seconds:2}}]});
    return {applied,result,inspection:sim.getWorkcellState()};
  });
  expect(seating.result.ok,JSON.stringify(seating.result)).toBe(true);expect(seating.inspection.funnelSeating[0].seated).toBe(true);
  expect(seating.inspection.taskEvaluation.success).toBe(false);expect(seating.inspection.taskEvaluation.scope).toContain('does not prove a robot transfer');
  await page.screenshot({path:testInfo.outputPath('physical-funnel-seating.png')});
  const restored=await page.evaluate(async()=>{
    const result=await applyLab(await stageLab([],'baseline'));sim.renderFrame();
    return {result,state:sim.getWorkcellState(),markerCount:sim.canvas.dataset.highContrastPerimeterCount};
  });
  expect(restored.result.ok).toBe(true);expect(restored.state.sceneMode).toBe('baseline');expect(restored.state.bodies.flask).toBeTruthy();expect(restored.markerCount).toBe('2');
  expect(errors).toEqual([]);
  await writeFile(testInfo.outputPath('lab-assets-webmcp.json'),JSON.stringify({clear,photo,seating,restored},null,2));
});
