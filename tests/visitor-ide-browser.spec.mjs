import { test, expect } from '@playwright/test';
import { readFile } from 'node:fs/promises';
const scene=JSON.parse(await readFile(new URL('../guides/examples/adapter-scene.json',import.meta.url)));
const task=JSON.parse(await readFile(new URL('../guides/examples/adapter-task.json',import.meta.url)));
const program=JSON.parse(await readFile(new URL('../guides/examples/adapter-program.json',import.meta.url)));
async function ready(page){
  await page.addInitScript(()=>{
    localStorage.setItem('rbide.profile','so101');
    window.visitorTools=new Map();
    Object.defineProperty(document,'modelContext',{configurable:true,value:{registerTool(tool,{signal}){visitorTools.set(tool.name,tool);signal.addEventListener('abort',()=>visitorTools.delete(tool.name));return Promise.resolve();}}});
  });
  await page.goto('/ide.html?robot=openarm&ci=visitor-manual');
  await expect(page.locator('#statusMessage')).toContainText('Ready',{timeout:60000});
  await expect(page.locator('#robotSelect')).toHaveValue('openarm');
}
const call=(page,name,input={})=>page.evaluate(({name,input})=>visitorTools.get(name).execute(input,{}),{name,input});

test('IDE controls and Help preserve the active OpenArm scene',async({page},info)=>{
  await ready(page);
  await expect(page.locator('#agentAccessControl')).toHaveAttribute('data-access','off');
  const before=await page.evaluate(()=>({authority:__robobuddyCi.app.sim.getPhysicalAuthorityToken(),token:__robobuddyCi.app.runToken}));
  for(const guide of ['openarm','webmcp']){
    await page.locator('[data-menu="help"]').click();
    const [manual]=await Promise.all([page.waitForEvent('popup'),page.locator(`[data-help-guide="${guide}"]`).click()]);
    await manual.waitForLoadState('domcontentloaded');
    await expect(manual.getByRole('heading',{level:1})).toHaveText(guide==='openarm'?'OpenArm manual':'WebMCP manual');
    expect(await manual.evaluate(()=>window.opener)).toBe(null);await manual.close();
  }
  expect(await page.evaluate(()=>({authority:__robobuddyCi.app.sim.getPhysicalAuthorityToken(),token:__robobuddyCi.app.runToken}))).toEqual(before);
  await expect(page.locator('#agentAccessControl')).toHaveAttribute('data-access','off');
  await page.locator('#fitBtn').click();
  await page.locator('#highContrastSceneBtn').click();
  await expect(page.locator('#highContrastSceneBtn')).toHaveAttribute('aria-pressed','false');
  await page.locator('#highContrastSceneBtn').click();
  await expect(page.locator('#highContrastSceneBtn')).toHaveAttribute('aria-pressed','true');
  await page.locator('#panelToggle').click();
  await expect(page.locator('#panelToggle')).toHaveAttribute('aria-expanded','true');
  await page.locator('#bottomClose').click();
  await expect(page.locator('#panelToggle')).toHaveAttribute('aria-expanded','false');
  await page.locator('#generalLabOpen').click();
  await expect(page.locator('#generalLabOpen')).toHaveAttribute('aria-expanded','true');
  await page.locator('#generalLabPanel').getByRole('button',{name:'Close',exact:true}).click();
  await expect(page.locator('#generalLabOpen')).toHaveAttribute('aria-expanded','false');
  const agent=page.getByRole('switch',{name:'Agent Assist'});
  await expect(agent).toHaveAttribute('aria-checked','false');
  // Scripted clicks cannot grant consent; native keyboard activation can.
  await agent.evaluate(button=>button.click());
  await expect(agent).toHaveAttribute('aria-checked','false');
  expect(await page.evaluate(()=>visitorTools.size)).toBe(0);
  await agent.focus();
  await agent.press('Space');
  await expect(agent).toHaveAttribute('aria-checked','true');
  await expect.poll(()=>page.evaluate(()=>visitorTools.has('manage_openarm_scene'))).toBe(true);
  await page.screenshot({path:info.outputPath('openarm-paper-agent-on.png')});
  await agent.press('Enter');
  await expect(agent).toHaveAttribute('aria-checked','false');
  await expect.poll(()=>page.evaluate(()=>visitorTools.size)).toBe(0);
  expect(await page.evaluate(()=>({authority:__robobuddyCi.app.sim.getPhysicalAuthorityToken(),token:__robobuddyCi.app.runToken}))).toEqual(before);
  // Retain visual evidence of both themes on the real, ready workspace.
  await page.evaluate(()=>document.fonts.ready);
  await page.screenshot({path:info.outputPath('openarm-paper.png')});
  await page.locator('[data-menu="theme"]').click();
  await page.locator('[data-theme-id="midnight-teal"]').click();
  await page.screenshot({path:info.outputPath('openarm-graphite.png')});
  await agent.click();
  await expect(page.locator('#agentAccessControl')).toHaveAttribute('data-tools','enabled');
  await page.screenshot({path:info.outputPath('openarm-graphite-agent-on.png')});
  await agent.click();
  await page.setViewportSize({width:390,height:844});
  await page.locator('#mobileSimBtn').click();
  for(const id of ['fitBtn','highContrastSceneBtn','panelToggle','generalLabOpen']){
    const box=await page.locator(`#${id}`).boundingBox();
    expect(box).not.toBeNull();expect(box.x).toBeGreaterThanOrEqual(0);expect(box.x+box.width).toBeLessThanOrEqual(390);
  }
  await page.screenshot({path:info.outputPath('openarm-mobile.png')});
});

test('published manual files execute a full contact-driven transfer through the actual IDE tools',async({page},info)=>{
  test.setTimeout(240000);const errors=[];page.on('pageerror',e=>errors.push(String(e)));await ready(page);
  await page.locator('#agentAccessToggle').click();
  await expect.poll(()=>page.evaluate(()=>visitorTools.has('manage_openarm_scene'))).toBe(true);
  const initial=await call(page,'inspect_openarm_scene',{view:'summary'});
  const stage=await call(page,'manage_openarm_scene',{command:'stage',expected_scene_revision:initial.authority.sceneRevision,scene});expect(stage.ok,JSON.stringify(stage)).toBe(true);
  const checked=await call(page,'manage_openarm_scene',{command:'check',stage_id:stage.result.id,settle_seconds:.3});expect(checked.ok,JSON.stringify(checked)).toBe(true);
  const applied=await call(page,'manage_openarm_scene',{command:'apply',stage_id:stage.result.id,acknowledge_reset:true});expect(applied.ok,JSON.stringify(applied)).toBe(true);
  const current=await call(page,'inspect_openarm_scene',{view:'summary'});
  const defined=await call(page,'manage_openarm_task',{command:'define',expected_scene_revision:current.authority.sceneRevision,task});expect(defined.ok,JSON.stringify(defined)).toBe(true);
  const executed=await call(page,'run_openarm_program',{...program,expected_scene_revision:current.authority.sceneRevision});expect(executed.ok,JSON.stringify(executed)).toBe(true);
  const evidence=await call(page,'inspect_openarm_scene',{view:'evidence'});expect(evidence.task.success,JSON.stringify(evidence)).toBe(true);expect(evidence.task.hardwareValidated).toBe(false);
  expect(Object.values(evidence.task.observedSequence).every(Boolean)).toBe(true);expect(errors).toEqual([]);
  await info.attach('published-manual-transfer',{body:JSON.stringify({checked,applied,executed,evidence},null,2),contentType:'application/json'});
  await page.screenshot({path:info.outputPath('manual-transfer-ide.png')});
});
