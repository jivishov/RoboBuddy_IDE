import {test,expect} from '@playwright/test';
import {readFile} from 'node:fs/promises';
import {resolve} from 'node:path';
// Local dependency routing is optional and test-only. Production retains the fleet's pinned CDNs.
async function offlineRoutes(page) {
  if(!process.env.ASIMOV_OFFLINE) return;
  await page.route('https://cdn.jsdelivr.net/npm/three@0.180.0/**',async route=>{
    const path=route.request().url().split('three@0.180.0/')[1];
    await route.fulfill({body:await readFile(resolve('node_modules/three',path)),contentType:'text/javascript'});
  });
}
test('Asimov worker/session/bridge: all variants, stale epoch, pause, command bounds',async({page})=>{
 await page.goto('/physics-slice.html');
 const result=await page.evaluate(async()=>{
  const {PhysicsSession}=await import('/src/physics/session.js');
  const {BrowserMuJoCoBackend}=await import('/src/physics/browser-mujoco-backend.js');
  const {ASIMOV_SCENES}=await import('/src/physics/asimov-scene.js');
  const {LivePythonBridge}=await import('/src/runtime/live-python-bridge.js');
  const s=new PhysicsSession(new BrowserMuJoCoBackend({workerUrl:new URL('/src/physics/asimov-mujoco-worker.js',location.href)}),{observationBatchSteps:4});
  const initial=await s.loadScene(structuredClone(ASIMOV_SCENES.mounted));
  const bridge=new LivePythonBridge(s); await bridge.connect(ASIMOV_SCENES.mounted.robotId);
  await bridge.sendAction({left_elbow_joint:1}); await bridge.advance(1);
  const observation=await bridge.getObservation();
  await s.pause(); await s.advanceSteps(100); const paused=await s.getObservation(); await s.resume();
  await s.reset(); let stale=false; try {await bridge.advance(.1);}catch{stale=true;}
  await s.loadScene(structuredClone(ASIMOV_SCENES.drop)); const drop0=await s.getObservation();await s.advanceSteps(30); const drop1=await s.getObservation();
  await s.loadScene(structuredClone(ASIMOV_SCENES.freebase)); await s.advanceSteps(200);const free=await s.getObservation();
  s.dispose();return {elbow:observation.joints.left_elbow_joint.positionRad,clock:observation.simulationTimeSeconds,pausedClock:paused.simulationTimeSeconds,stale,drop:drop0.root.positionM[2]-drop1.root.positionM[2],freeContacts:free.contactCount};
 });
 expect(Math.abs(result.elbow-1)).toBeLessThan(.1);expect(result.pausedClock).toBe(result.clock);expect(result.stale).toBe(true);expect(result.drop).toBeGreaterThan(.05);
});
test('Asimov actual full STL presentation follows observed bodies; renderer cannot advance clock',async({page})=>{
 await offlineRoutes(page); const errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.goto('/physics-slice.html');
 // This intentionally minimal physics harness does not have the IDE's import map.
 await page.addScriptTag({type:'importmap',content:JSON.stringify({imports:{three:'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js'}})});
 const result=await page.evaluate(async()=>{
  const {AsimovPhysicalSimulator}=await import('/src/physics/asimov-physical-simulator.js');
  const {ASIMOV_WORKSPACES}=await import('/src/physics/asimov-workspaces.js');
  const canvas=document.createElement('canvas');canvas.style='width:850px;height:650px';document.body.replaceChildren(canvas);
  const sim=new AsimovPhysicalSimulator(canvas);window.asimovTestSim=sim;
  await sim.setScenario('asimov',structuredClone(ASIMOV_WORKSPACES['asimov-mounted']));
  const start=sim.getState().simulation_time_s;for(let i=0;i<20;i++)sim.renderFrame();
  const end=sim.getState().simulation_time_s;
  await sim.applyPhysicalTargets({left_elbow_joint:1.2,right_shoulder_pitch_joint:.3},{advanceSeconds:1});sim.renderFrame();
  return {start,end,audit:sim.getPresentationAudit(),alignment:sim.getPresentationAlignment(),elbow:sim.getState().joints.left_elbow_joint};
 });
 expect(result.start).toBe(result.end);expect(result.audit.meshCount).toBe(25);expect(result.audit.bodyCount).toBe(26);
 expect(result.alignment.maxBodyErrorMm).toBeLessThan(.001);expect(errors).toEqual([]);
 await page.screenshot({path:process.env.ASIMOV_SCREENSHOT||'test-results/asimov-mounted.png'});
 const reset=await page.evaluate(async()=>{
  const sim=window.asimovTestSim;await sim.getPhysicalSession().cancelRun('test-stop');
  const wasUnready=!sim.isReady();await sim.reset();
  const result={wasUnready,ready:sim.isReady(),time:sim.getState().simulation_time_s};sim.dispose();return result;
 });
 expect(reset).toEqual({wasUnready:true,ready:true,time:0});
});
test('Asimov IDE selector exposes only user experiments while references remain internal',async({page})=>{
 test.setTimeout(600000);
 await page.addInitScript(()=>{
  localStorage.setItem('rbide.profile','asimov');localStorage.setItem('rbide.task.asimov','asimov-actuator-mounted');
  const registrations=[];
  Object.defineProperty(document,'modelContext',{configurable:true,value:{registerTool(tool){registrations.push(tool);return Promise.resolve();}}});
  window.asimovRegistrations=registrations;
 });
 await page.goto('/?ci=asimov-physical',{waitUntil:'domcontentloaded'});
 await expect(page.locator('#statusMessage')).toContainText('Ready',{timeout:120000});
 await expect(page.locator('#robotSelect')).toHaveValue('asimov');
 const catalog=await page.locator('#taskSelect option').evaluateAll(options=>options.map(o=>({value:o.value,text:o.textContent})));
 expect(catalog).toEqual([
  {value:'asimov-actuator-mounted',text:'Asimov 1 — Actuator Lab (experimental)'},
  {value:'asimov-actuator-freebase',text:'Asimov 1 — Whole-Body Dynamics (experimental)'},
  {value:'asimov-standing',text:'Asimov 1 — Standing Trial (experimental)'},
  {value:'asimov-sensor-standing',text:'Asimov 1 — Sensor Standing (experimental)'},
  {value:'asimov-sensor-standing-delay',text:'Asimov 1 — Delayed Sensor Standing (sensitivity)'},
  {value:'asimov-sensor-standing-ankle-stress',text:'Asimov 1 — Ankle Loss Standing (sensitivity)'},
 ]);
 expect(catalog.map(x=>x.value)).not.toEqual(expect.arrayContaining(['asimov-mounted','asimov-freebase','asimov-drop']));
 await expect(page.locator('#simCanvas')).toHaveAttribute('data-asimov-root-mode','fixed-mounted');
 await page.evaluate(async()=>{const app=window.__robobuddyCi.app; await app.run();});
 const python=await page.evaluate(()=>({out:window.__robobuddyCi.app.console,diagnostics:window.__robobuddyCi.app.sim.backend.getState()}));
 expect(python.out.stderr).toBe('');expect(python.out.stdout).toContain('elbow');expect(python.diagnostics.simulation_time_s).toBeGreaterThan(.9);
 await expect(page.locator('#simActionLabel')).toHaveText('Run complete · observations recorded');
 await expect(page.locator('#physicsBackendBadge')).toContainText('browser-mujoco');
 await expect(page.locator('#simBadge')).toContainText('MUJOCO');
 expect(await page.locator('#telemetryPanel').textContent()).not.toContain('NaN');
 await page.locator('#agentAccessToggle').click();
 const agent=await page.evaluate(async()=>{
  const {agentFacade:facade}=window.__robobuddyCi;
  const {executeAsimovPhysicalControl,WEBMCP_ASIMOV_SCHEMA_VERSION}=await import('/src/webmcp/asimov-physical-control.js');
  const epoch=facade.registrationEpoch;
  const command=await executeAsimovPhysicalControl(facade,{schema_version:WEBMCP_ASIMOV_SCHEMA_VERSION,command:'set_joint_targets',targets_rad:{left_elbow_joint:1.2},advance_seconds:.2},null,epoch);
  let forbidden=false;try{await executeAsimovPhysicalControl(facade,{schema_version:WEBMCP_ASIMOV_SCHEMA_VERSION,command:'walk'},null,epoch);}catch{forbidden=true;}
  return {command,forbidden,registered:window.asimovRegistrations.some(t=>t.name==='control_asimov_physical_simulation')};
 });
 expect(agent.registered).toBe(true);expect(agent.forbidden).toBe(true);expect(agent.command.observedState.joints.left_elbow_joint.requested_target_rad).toBe(1.2);
 await page.screenshot({path:'test-results/asimov-ide.png'});
 await page.locator('#taskSelect').selectOption('asimov-actuator-freebase');
 await expect(page.locator('#statusMessage')).toContainText('Ready',{timeout:120000});
 await expect(page.locator('#simCanvas')).toHaveAttribute('data-asimov-root-mode','free-base');
 await page.locator('#robotSelect').selectOption('unitree');
 await expect(page.locator('#statusMessage')).toContainText('Ready',{timeout:120000});
 await expect(page.locator('#simCanvas')).not.toHaveAttribute('data-asimov-root-mode',/.+/);
});

test('Asimov actuator experiment: sensor isolation, continuous caps and physical standing via Python and WebMCP',async({page})=>{
 test.setTimeout(600000);
 await page.addInitScript(()=>{
  localStorage.setItem('rbide.profile','asimov');localStorage.setItem('rbide.task.asimov','asimov-actuator-mounted');
  Object.defineProperty(document,'modelContext',{configurable:true,value:{registerTool(){return Promise.resolve();}}});
 });
 await page.goto('/?ci=asimov-physical',{waitUntil:'domcontentloaded'});
 await expect(page.locator('#statusMessage')).toContainText('Ready',{timeout:120000});
 await expect(page.locator('#simCanvas')).toHaveAttribute('data-asimov-root-mode','fixed-mounted');
 const lab=await page.evaluate(async()=>{
  const app=window.__robobuddyCi.app;const ok=await app.run();
  const b=app.sim.backend;
  return {ok,problems:app.problems,out:app.console,state:b.getState(),audit:b.getPresentationAlignment(),sensors:b.getSensorObservation()};
 });
 expect(lab.ok,JSON.stringify(lab.problems)).toBe(true);
 expect(lab.state.simulation_time_s).toBeCloseTo(1,8);
 expect(lab.out.stderr).toBe('');expect(lab.out.stdout).toContain('Sensor profile');
 expect(lab.state.actuator_model.continuousOnly).toBe(true);expect(lab.state.actuator_model.configurationSha256).toMatch(/^[0-9a-f]{64}$/);
 expect(lab.sensors.view).toBe('hardware_like');expect(lab.sensors.root).toBeUndefined();expect(lab.sensors.bodies).toBeUndefined();
 expect(lab.audit.maxBodyErrorMm).toBeLessThan(.001);
 await page.screenshot({path:'test-results/asimov-actuator-lab.png'});
 await page.locator('#taskSelect').selectOption('asimov-standing');
 await expect(page.locator('#statusMessage')).toContainText('Ready',{timeout:120000});
 const standing=await page.evaluate(async()=>{
  const app=window.__robobuddyCi.app;const ok=await app.run();return {ok,problems:app.problems,out:app.console,state:app.sim.getState(),evaluation:app.sim.getTaskEvaluation()};
 });
 expect(standing.ok,JSON.stringify(standing.problems)).toBe(true);
 expect(standing.state.simulation_time_s).toBeCloseTo(12,8);expect(standing.evaluation.samples).toBeGreaterThanOrEqual(4800);
 expect(standing.out.stderr).toBe('');expect(standing.state.controller_mode).toBe('asimov-stance-feedback-v1');
 expect(standing.evaluation.status).toBe('passed');expect(standing.evaluation.success).toBe(true);expect(standing.evaluation.validDwellSeconds).toBeGreaterThan(10);
 await expect(page.locator('#simActionLabel')).toHaveText('Physical task complete');
 await page.screenshot({path:'test-results/asimov-standing-trial.png'});
 await page.locator('#agentAccessToggle').click();
 const agent=await page.evaluate(async()=>{
  const {app,agentFacade:f}=window.__robobuddyCi;
  const {executeAsimovPhysicalControl:execute,WEBMCP_ASIMOV_SCHEMA_VERSION:v}=await import('/src/webmcp/asimov-physical-control.js');
  const sensors=await execute(f,{schema_version:v,command:'read_sensors'},null,f.registrationEpoch);
  await execute(f,{schema_version:v,command:'stop'},null,f.registrationEpoch);
  const stopped=app.sim.getTaskEvaluation();
  await execute(f,{schema_version:v,command:'reset'},null,f.registrationEpoch);
  const start=await execute(f,{schema_version:v,command:'engage_stand'},null,f.registrationEpoch);
  await execute(f,{schema_version:v,command:'advance',advance_seconds:.2},null,f.registrationEpoch);
  return {sensors,stopped,start,state:app.sim.getState()};
 });
 expect(agent.sensors.sensorObservation.view).toBe('hardware_like');expect(agent.stopped.status).toBe('failed');
 expect(agent.start.observedState.standing_assessment.status).toBe('running');expect(agent.state.simulation_time_s).toBeCloseTo(.2,8);
 await page.locator('#taskSelect').selectOption('asimov-actuator-mounted');await expect(page.locator('#statusMessage')).toContainText('Ready',{timeout:120000});
 const mountedExperiment=await page.evaluate(async()=>{
  const {app,agentFacade:f}=window.__robobuddyCi;
  const {executeAsimovPhysicalControl:execute,WEBMCP_ASIMOV_SCHEMA_VERSION:v}=await import('/src/webmcp/asimov-physical-control.js');
  let blocked=false;try{await execute(f,{schema_version:v,command:'engage_stand'},null,f.registrationEpoch);}catch{blocked=true;}
  return {blocked,state:app.sim.getState()};
 });
 expect(mountedExperiment.blocked).toBe(true);expect(mountedExperiment.state.actuator_model.continuousOnly).toBe(true);
});

test('Sensor standing and bounded programs are available through the registered WebMCP tool',async({page})=>{
 test.setTimeout(600000);
 await page.addInitScript(()=>{
  localStorage.setItem('rbide.profile','asimov');localStorage.setItem('rbide.task.asimov','asimov-sensor-standing');
  window.asimovProgramTools=[];
  Object.defineProperty(document,'modelContext',{configurable:true,value:{registerTool(tool){window.asimovProgramTools.push(tool);return Promise.resolve();}}});
 });
 await page.goto('/?ci=asimov-physical',{waitUntil:'domcontentloaded'});
 await expect(page.locator('#statusMessage')).toContainText('Ready',{timeout:120000});
 const python=await page.evaluate(async()=>{
  const app=window.__robobuddyCi.app,ok=await app.run();
  return {ok,console:app.console,state:app.sim.getState()};
 });
 expect(python.ok,JSON.stringify(python.console)).toBe(true);
 expect(python.state.standing_assessment.controllerId).toBe('asimov-sensor-stance-v2');
 expect(python.state.standing_assessment.status).toBe('passed');
 expect(python.state.standing_assessment.sensorFeedback.groundTruthFallback).toBe(false);
 await page.locator('#agentAccessToggle').click();
 const result=await page.evaluate(async()=>{
  const {agentFacade:facade,app}=window.__robobuddyCi;
  const v='robobuddy.asimov.physical.v1';
  const exec=async(command,args={})=>{
    const tool=window.asimovProgramTools.findLast(t=>t.name==='control_asimov_physical_simulation');
    const result=await tool.execute({schema_version:v,command,...args},{signal:new AbortController().signal});
    if(!result.ok)throw new Error(JSON.stringify(result));return result;
  };
  const registered=window.asimovProgramTools.findLast(t=>t.name==='control_asimov_physical_simulation');
  const declared=registered.inputSchema.oneOf.map(b=>b.properties.command.const);
  await exec('reset');await exec('engage_stand');
  const sequence=await exec('run_sequence',{segments:[{duration_seconds:1},{targets_rad:{left_elbow_joint:1,right_elbow_joint:-1},preserve_standing:true,duration_seconds:.5},{targets_rad:{left_elbow_joint:.9,right_elbow_joint:-.9},preserve_standing:true,duration_seconds:.5}]});
  const sensors=await exec('read_sensors');
  let rejected=false;const before=app.sim.getState().simulation_time_s;
  try {await exec('run_sequence',{segments:[{targets_rad:{left_elbow_joint:1},duration_seconds:.2},{duration_seconds:.003}]});} catch {rejected=true;}
  const unchanged=app.sim.getState().simulation_time_s===before;
  return {declared,sequence,sensors,rejected,unchanged};
 });
 expect(result.declared).toEqual(expect.arrayContaining(['run_sequence','wait_for_joint','set_standing_targets']));
 expect(result.sequence.executionStatus).toBe('completed');
 expect(result.sequence.observedState.controllerMode).toBe('asimov-sensor-stance-v2');
 expect(result.sensors.sensorObservation.imu.frame).toBe('pelvis_link');
 expect(result.rejected).toBe(true);expect(result.unchanged).toBe(true);
 await page.screenshot({path:'test-results/asimov-sensor-webmcp.png'});
});
