import {test,expect} from '@playwright/test';
import {fakeWebcam} from '../helpers/gesture-camera.mjs';
import {openHand,fist} from '../fixtures/gesture-hand.mjs';

test.beforeEach(async({page})=>{await fakeWebcam(page);});
test.afterEach(async({page})=>{
  const {token}=await (await page.request.get('/api/bootstrap')).json();
  if(await page.locator('[data-ui="close"]').isVisible())await page.locator('[data-ui="close"]').click();
  if(await page.locator('#disconnectBtn').isEnabled()) {
    if(!await page.locator('#disconnectBtn').isVisible())await page.locator('#setupToggle').click();
    await page.locator('#disconnectBtn').click();
  }
  await page.goto('about:blank');
  await expect.poll(async()=> (await (await page.request.get('http://127.0.0.1:8766/api/state',{headers:{Authorization:'Bearer '+token}})).json()).ownerActive).toBe(false);
});
async function camera(page) {
  await page.locator('#gestureControlBtn').click();
  await page.getByRole('button',{name:'Start webcam',exact:true}).click();
  await expect(page.getByRole('button',{name:'Set gesture neutral',exact:true})).toBeEnabled();
  await page.getByRole('button',{name:'Set gesture neutral',exact:true}).click();
}
async function enable(page) {
  await page.getByRole('button',{name:'Enable gesture control',exact:true}).click();
  await expect(page.locator('.gesture-panel')).toHaveAttribute('data-active','true');
}
test('preview is read only; enabled gestures jog one joint and fist invalidates capture',async({page})=>{
  const operations=[];page.on('request',r=>{if(r.url().endsWith('/api/command'))operations.push(r.postDataJSON());});
  await page.goto('/hardware.html');
  await expect(page.locator('#stateLabel')).toHaveText('USB not connected');
  await camera(page);
  expect(operations).toEqual([]);
  await page.getByRole('button',{name:'Enable gesture control',exact:true}).click();
  await expect(page.locator('.gesture-status')).toContainText('enable motors');
  expect(operations).toEqual([]);
  await page.locator('[data-ui="close"]').click();
  await page.locator('#connectBtn').click();
  await page.locator('#supportAck').check();await page.locator('#armBtn').click();
  await expect(page.locator('#stateLabel')).toHaveText('Motors enabled');
  await camera(page);await enable(page);
  await page.evaluate(hand=>window.testHand=hand,openHand(-.10));
  await expect.poll(()=>operations.filter(o=>o.operation==='gesture_move'&&o.args.direction>0).length).toBeGreaterThan(1);
  await expect.poll(async()=>Number(await page.locator('[data-joint="shoulder_pan"] .actual').textContent().then(v=>v.replace('°','')))).toBeGreaterThan(.1);
  await page.evaluate(hand=>window.testHand=hand,fist());
  await expect(page.locator('.gesture-panel')).toHaveAttribute('data-active','false');
  await expect(page.locator('.gesture-status')).toContainText('Fist');
  const count=operations.filter(o=>o.operation==='gesture_move').length;
  await page.evaluate(hand=>window.testHand=hand,openHand(-.10));
  await page.waitForTimeout(600);
  expect(operations.filter(o=>o.operation==='gesture_move')).toHaveLength(count);
  expect(operations.filter(o=>o.operation==='gesture_move').every(o=>o.args.joint==='shoulder_pan'&&Math.abs(o.args.direction)<=1)).toBeTruthy();
  await page.locator('[data-ui="close"]').click();
  expect(await page.evaluate(()=>window.testCameraTracks.every(track=>track.readyState==='ended'))).toBeTruthy();
});
test('stalled and stale frames disarm even while helper heartbeat continues',async({page})=>{
  await page.goto('/hardware.html');await expect(page.locator('#stateLabel')).toHaveText('USB not connected');
  await page.locator('#connectBtn').click();await page.locator('#supportAck').check();await page.locator('#armBtn').click();
  await camera(page);await enable(page);
  await page.evaluate(()=>window.testStaleFrames=true);
  await expect(page.locator('.gesture-panel')).toHaveAttribute('data-active','false');
  await expect(page.locator('.gesture-status')).toContainText('Tracking lost');
  await expect(page.locator('#stateLabel')).toHaveText('Hold requested');
  await page.evaluate(()=>window.testStaleFrames=false);
  await expect(page.getByRole('button',{name:'Set gesture neutral',exact:true})).toBeEnabled();
  await page.getByRole('button',{name:'Set gesture neutral',exact:true}).click();await enable(page);
  await page.evaluate(()=>window.testTrackerStalled=true);
  await expect(page.locator('.gesture-panel')).toHaveAttribute('data-active','false');
  await expect(page.locator('#stateLabel')).toHaveText('Hold requested');
});
test('ambiguous hands, Esc, close and denied permission cannot trigger motion',async({page})=>{
  await page.goto('/hardware.html');await expect(page.locator('#stateLabel')).toHaveText('USB not connected');
  await camera(page);
  await page.evaluate(hand=>window.testHand={landmarks:[...hand.landmarks,...hand.landmarks]},openHand());
  await expect(page.locator('.gesture-status')).toContainText('only one hand');
  await expect(page.getByRole('button',{name:'Enable gesture control',exact:true})).toBeDisabled();
  await page.keyboard.press('Escape');
  await page.locator('[data-ui="close"]').click();
  await page.evaluate(()=>window.testCameraDenied=true);
  await page.locator('#gestureControlBtn').click();await page.getByRole('button',{name:'Start webcam',exact:true}).click();
  await expect(page.locator('.gesture-status')).toContainText('permission was denied');
  for (const width of [320,390,768,1366]) {
    await page.setViewportSize({width,height:768});
    expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBeTruthy();
  }
  await page.screenshot({path:'test-results/so101-webcam-gestures.png',fullPage:true});
});
