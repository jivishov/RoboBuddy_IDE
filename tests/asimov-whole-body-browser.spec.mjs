import {test,expect} from '@playwright/test';

test('Asimov WebMCP executes bounded agent-generated whole-body keyframes on the free-base MuJoCo plant',async({page})=>{
  test.setTimeout(600000);
  await page.addInitScript(()=>{
    localStorage.setItem('rbide.profile','asimov');
    localStorage.setItem('rbide.task.asimov','asimov-actuator-freebase');
    window.asimovWholeBodyTools=[];
    Object.defineProperty(document,'modelContext',{configurable:true,value:{registerTool(tool){window.asimovWholeBodyTools.push(tool);return Promise.resolve();}}});
  });
  await page.goto('/?ci=asimov-physical',{waitUntil:'domcontentloaded'});
  await expect(page.locator('#statusMessage')).toContainText('Ready',{timeout:120000});
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-asimov-root-mode','free-base');
  await page.locator('#agentAccessToggle').click();
  const result=await page.evaluate(async()=>{
    const {app}=window.__robobuddyCi;
    const tool=window.asimovWholeBodyTools.findLast(t=>t.name==='control_asimov_physical_simulation');
    if(!tool)throw new Error('Asimov WebMCP tool was not registered');
    const schemaVersion='robobuddy.asimov.physical.v1';
    const invoke=async(input)=>{
      const response=await tool.execute({schema_version:schemaVersion,...input},{signal:new AbortController().signal});
      if(!response.ok)throw new Error(JSON.stringify(response));
      return response;
    };
    const commands=tool.inputSchema.oneOf.map(branch=>branch.properties.command.const);
    const capability=await invoke({command:'inspect_capability'});
    const before=app.sim.getState();
    const motion=await invoke({
      command:'run_whole_body_motion',motion_intent:'squat',stabilization:'ground_truth',abort_on_instability:false,
      keyframes:[{phase:'small symmetric leg bend',duration_seconds:.1,support:'any',targets_rad:{left_knee_joint:.05,right_knee_joint:-.05}}],
    });
    const after=app.sim.getState();
    return {commands,capability,before,motion,after};
  });
  expect(result.commands).toContain('run_whole_body_motion');
  expect(result.capability.capabilities.wholeBodyMotion).toBe('experimental-agent-generated-keyframes');
  expect(result.capability.capabilities.walking).toContain('not a trained or validated gait policy');
  expect(result.motion.executionStatus).toBe('completed');
  expect(result.motion.completedKeyframes).toBe(1);
  expect(result.motion.advancedSeconds).toBeCloseTo(.1,8);
  expect(result.motion.stabilization).toBe('ground_truth');
  expect(result.motion.observedState.root.mode).toBe('free-base');
  expect(result.after.simulation_time_s-result.before.simulation_time_s).toBeCloseTo(.1,8);
  for(let i=0;i<3;i++)expect(result.motion.rootDisplacementM[i]).toBeCloseTo(result.after.root.position_m[i]-result.before.root.position_m[i],8);
  expect(result.motion.observedState.joints.left_knee_joint.requested_target_rad).toBeGreaterThan(0);
  expect(result.motion.observedState.joints.right_knee_joint.requested_target_rad).toBeLessThan(0);
  expect(result.motion.note).toContain('not proof');
});
