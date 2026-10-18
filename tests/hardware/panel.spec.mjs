import {test,expect} from '@playwright/test';
import {readFile} from 'node:fs/promises';
import {resolve} from 'node:path';

test.beforeEach(async({page})=>{
  // Optional, test-only mirrors of the exact pinned production assets for offline runners.
  if(!process.env.HARDWARE_OFFLINE_MESH) return;
  await page.route('https://cdn.jsdelivr.net/npm/three@0.180.0/**',async route=>{
    const path=route.request().url().split('three@0.180.0/')[1];
    await route.fulfill({body:await readFile(resolve('node_modules/three',path)),contentType:'text/javascript'});
  });
  await page.route('https://cdn.jsdelivr.net/gh/jivishov/RoboBuddy_AI@66d18a029a0caeb6a6075e681dbd9ecd6b22affa/simulator/js/robot-mesh-data-so101.js',async route=>{
    await route.fulfill({body:await readFile(process.env.HARDWARE_OFFLINE_MESH),contentType:'text/javascript'});
  });
});

test.afterEach(async({page})=>{
  // Release the demo owner even after an assertion fails; never touches a real bus.
  if(await page.locator('#disconnectBtn').isEnabled()) {
    if(!await page.locator('#disconnectBtn').isVisible()) await page.locator('#setupToggle').click();
    await page.locator('#disconnectBtn').click();
    await expect(page.locator('#stateLabel')).toHaveText('Helper not paired');
  }
});

test('manual panel keeps read-only connection, motor enable and small jogs separate',async({page})=>{
  const commands=[];
  page.on('request',request=>{if(request.url().endsWith('/api/command'))commands.push(request.postDataJSON().operation);});
  const errors=[];page.on('pageerror',error=>errors.push(error.message));
  await page.goto('/hardware.html');
  await expect(page.getByText('DEMO MOTOR BUS · NO PHYSICAL HARDWARE',{exact:true})).toBeVisible();
  await expect(page.locator('#stateLabel')).toHaveText('USB not connected');
  await expect(page.locator('#armBtn')).toBeDisabled();
  await expect(page.getByRole('button',{name:'Jog Shoulder pan positive',exact:true})).toBeDisabled();
  await expect(page.locator('#setupDetails')).toHaveAttribute('open','');
  await page.locator('#setupToggle').click();
  await expect(page.locator('#connectBtn')).toBeHidden();
  await expect(page.getByRole('heading',{name:'Manual joints',exact:true})).toBeInViewport();
  await expect(page.getByRole('heading',{name:'Encoder view',exact:true})).toBeInViewport();
  await page.locator('#setupToggle').press('Enter');
  await expect(page.locator('#connectBtn')).toBeVisible();
  expect(commands).toEqual([]);
  expect(await page.locator('button').evaluateAll(buttons=>buttons.every(button=>button.querySelector('svg')&&button.getAttribute('aria-label')&&button.title))).toBeTruthy();
  await page.getByRole('button',{name:'Connect · read only',exact:true}).click();
  await expect(page.locator('#stateLabel')).toHaveText('Connected · read only');
  await expect(page.locator('#torqueState')).toHaveText('Disabled');
  await expect(page.locator('#setupDetails')).not.toHaveAttribute('open','');
  await expect(page.locator('#setupToggle')).toBeFocused();
  await expect(page.locator('#setupSummary')).toContainText('DEMO · my_so101 · Calibration matched');
  await expect(page.locator('#supportAck')).toBeVisible();
  await expect(page.locator('#disableBtn')).toBeVisible();
  await expect(page.locator('#quickStopBtn')).toBeEnabled();
  await page.locator('#setupToggle').click();
  await page.waitForResponse(response=>response.url().endsWith('/api/heartbeat'));
  await expect(page.locator('#disconnectBtn')).toBeVisible();
  await expect(page.locator('#setupDetails')).toHaveAttribute('open','');
  await page.locator('#setupToggle').press('Space');
  await expect(page.locator('#setupDetails')).not.toHaveAttribute('open','');
  expect(commands).toEqual(['connect']);
  await expect(page.locator('#hardwareCanvas')).toBeVisible({timeout:20000});
  await expect(page.locator('#armBtn')).toBeDisabled();
  await page.locator('#supportAck').check();
  await page.locator('#armBtn').click();
  await expect(page.locator('#stateLabel')).toHaveText('Motors enabled');
  await expect(page.locator('#controlBadge')).toHaveText('READY TO JOG');
  await page.getByRole('button',{name:'Jog Shoulder pan positive',exact:true}).click();
  await expect(page.locator('#stateLabel')).toHaveText('Motors enabled');
  await expect.poll(async()=>Number(await page.locator('[data-joint="shoulder_pan"] input').inputValue())).toBeGreaterThan(.5);
  await page.locator('[data-joint="shoulder_pan"] input').fill('50');
  await page.locator('[data-joint="shoulder_pan"] .apply').click();
  await expect(page.locator('#feedback')).toContainText('increments no larger');
  await page.getByRole('button',{name:'Jog Gripper positive',exact:true}).click();
  await expect(page.locator('#stateLabel')).toHaveText('Motors enabled');
  await page.setViewportSize({width:1366,height:768});
  await page.evaluate(()=>scrollTo(0,0));
  await page.screenshot({path:'test-results/so101-hardware-demo-laptop.png',fullPage:true});
  await expect(page.getByRole('button',{name:'Jog Gripper positive',exact:true})).toBeInViewport({ratio:1});
  await expect(page.locator('#visualHost')).toBeInViewport({ratio:1});
  await page.mouse.move(0,0);
  await page.screenshot({path:'test-results/so101-hardware-demo-laptop.png',fullPage:true});
  await page.locator('#stopBtn').click();
  await expect(page.locator('#stateLabel')).toHaveText('Hold requested');
  await expect(page.locator('#torqueState')).toHaveText('Enabled');
  await page.locator('#disableBtn').click();
  await expect(page.locator('#torqueState')).toHaveText('Disabled');
  await page.setViewportSize({width:1366,height:900});
  await page.evaluate(()=>scrollTo(0,0));
  await page.mouse.move(0,0);
  await page.screenshot({path:'test-results/so101-hardware-demo-desktop.png',fullPage:true});
  for(const width of [320,390,580,640,680,681,768,900,901,1024,1366]) {
    await page.setViewportSize({width,height:844});
    expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),'No horizontal overflow at '+width).toBeTruthy();
  }
  await page.setViewportSize({width:390,height:844});
  await page.evaluate(()=>scrollTo(0,0));
  await expect(page.locator('#quickStopBtn')).toBeInViewport();
  expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBeTruthy();
  await page.screenshot({path:'test-results/so101-hardware-demo-mobile.png',fullPage:true});
  await page.locator('#visualHost').scrollIntoViewIfNeeded();
  await expect(page.locator('#quickStopBtn')).toBeInViewport({ratio:1});
  await page.locator('#setupToggle').click();
  await page.locator('#disconnectBtn').click();
  await expect(page.locator('#stateLabel')).toHaveText('Helper not paired');
  expect(errors).toEqual([]);
});

test('uncalibrated connection keeps setup available and controls locked',async({page})=>{
  // Change only the demo responses to exercise the missing-calibration screen.
  await page.route(/\/api\/(command|state|heartbeat)$/,async route=>{
    const response=await route.fetch();const data=await response.json();
    if(data.connected) data.calibrated=false;
    await route.fulfill({response,json:data});
  });
  await page.goto('/hardware.html');
  await expect(page.locator('#stateLabel')).toHaveText('USB not connected');
  await page.locator('#connectBtn').click();
  await expect(page.locator('#stateLabel')).toHaveText('Connected · read only');
  await expect(page.locator('#setupDetails')).toHaveAttribute('open','');
  await expect(page.locator('#calibrationBadge')).toHaveText('CALIBRATION REQUIRED');
  await expect(page.locator('#controlHint')).toContainText('Calibration required');
  await page.locator('#supportAck').check();
  await expect(page.locator('#armBtn')).toBeDisabled();
  await page.locator('#setupToggle').click();
  await expect(page.locator('#quickStopBtn')).toBeEnabled();
  await page.locator('#setupToggle').click();
  await page.locator('#disconnectBtn').click();
  await expect(page.locator('#stateLabel')).toHaveText('Helper not paired');
});

