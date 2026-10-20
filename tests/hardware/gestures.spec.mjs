import {test,expect} from '@playwright/test';
import {fakeWebcam} from '../helpers/gesture-camera.mjs';
import {openHand,fist} from '../fixtures/gesture-hand.mjs';

test.beforeEach(async({page})=>{await fakeWebcam(page);});
test.afterEach(async({page})=>{
  const {token}=await (await page.request.get('/api/bootstrap')).json();
  if(await page.locator('[data-ui="close"]').isVisible())await page.locator('[data-ui="close"]').click();
  await expect(page.locator('.gesture-panel')).toHaveAttribute('data-stopping','false');
  const snapshot=await (await page.request.get('/api/state',{headers:{Authorization:'Bearer '+token}})).json();
  if(snapshot.connected) {
    await expect(page.locator('#disconnectBtn')).toBeEnabled();
    if(!await page.locator('#disconnectBtn').isVisible())await page.locator('#setupToggle').click();
    await page.locator('#disconnectBtn').click();
    await expect(page.locator('#stateLabel')).toHaveText('Helper not paired');
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
  await page.setViewportSize({width:1366,height:768});
  await page.locator('#gestureControlBtn').click();
  await page.mouse.move(700,100);
  await page.screenshot({path:'test-results/so101-gesture-start.png'});
  await page.locator('[data-ui="close"]').click();
  await camera(page);
  expect(operations).toEqual([]);
  await expect(page.getByRole('button',{name:'Enable gesture control',exact:true})).toBeDisabled();
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
  await page.screenshot({path:'test-results/so101-gesture-paused.png'});
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

test('loading can survive joint changes and cancellation releases every camera track',async({page})=>{
  await page.goto('/hardware.html');await expect(page.locator('#stateLabel')).toHaveText('USB not connected');
  await page.evaluate(()=>window.testTrackerInitDelay=900);
  await page.locator('#gestureControlBtn').click();
  await page.getByRole('button',{name:'Start webcam',exact:true}).click();
  await expect(page.locator('.gesture-panel')).toHaveAttribute('data-camera','loading');
  await page.getByRole('combobox',{name:'Gesture control joint'}).selectOption('gripper');
  await expect(page.locator('.gesture-panel')).toHaveAttribute('data-camera','on');
  await expect(page.getByRole('button',{name:'Set gesture neutral',exact:true})).toBeEnabled();
  await page.getByRole('button',{name:'Stop webcam',exact:true}).click();
  await page.getByRole('button',{name:'Start webcam',exact:true}).click();
  await expect(page.locator('.gesture-panel')).toHaveAttribute('data-camera','loading');
  await page.getByRole('button',{name:'Close hand control',exact:true}).click();
  await page.waitForTimeout(1000);
  expect(await page.evaluate(()=>window.testCameraTracks.length)).toBeGreaterThanOrEqual(1);
  expect(await page.evaluate(()=>window.testCameraTracks.every(track=>track.readyState==='ended'))).toBeTruthy();
  await page.locator('#gestureControlBtn').click();
  await expect(page.locator('.gesture-panel')).toHaveAttribute('data-camera','off');
});

test('guided control shows live intent and readback; Pause stays reachable at small sizes',async({page})=>{
  const operations=[];page.on('request',r=>{if(r.url().endsWith('/api/command'))operations.push(r.postDataJSON());});
  await page.goto('/hardware.html');await expect(page.locator('#stateLabel')).toHaveText('USB not connected');
  await page.locator('#connectBtn').click();await page.locator('#supportAck').check();await page.locator('#armBtn').click();
  await camera(page);
  await expect(page.locator('[data-ui="enable"]')).toBeFocused();
  await page.setViewportSize({width:1366,height:768});
  await page.mouse.move(700,100);
  await page.screenshot({path:'test-results/so101-gesture-ready-desktop.png'});
  await enable(page);
  await page.evaluate(hand=>window.testHand=hand,openHand(-.1));
  await expect(page.locator('[data-ui="direction"]')).toContainText('Increasing');
  await expect(page.locator('[data-ui="readback"]')).toContainText('°');
  await expect(page.locator('.gesture-title small')).toContainText('Demo motors');
  await page.screenshot({path:'test-results/so101-gesture-active-desktop.png'});
  for(const [width,height] of [[320,568],[390,660],[568,320],[320,400]]) {
    await page.setViewportSize({width,height});
    await expect(page.getByRole('button',{name:'Pause gesture control',exact:true})).toBeInViewport({ratio:1});
    await expect(page.getByRole('button',{name:'Close hand control',exact:true})).toBeInViewport({ratio:1});
    expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBeTruthy();
  }
  await page.setViewportSize({width:390,height:660});
  await page.screenshot({path:'test-results/so101-gesture-active-mobile.png'});
  await page.getByRole('button',{name:'Pause gesture control',exact:true}).click();
  await expect(page.getByRole('combobox',{name:'Gesture control joint'})).toBeEnabled();
  await page.screenshot({path:'test-results/so101-gesture-paused-mobile.png'});
  await page.getByRole('combobox',{name:'Gesture control joint'}).selectOption('gripper');
  await page.evaluate(hand=>window.testHand=hand,openHand());
  await expect(page.getByRole('button',{name:'Set gesture neutral',exact:true})).toBeEnabled();
  await page.getByRole('button',{name:'Set gesture neutral',exact:true}).click();
  await enable(page);
  const pinch=openHand();pinch.landmarks[0][4]={...pinch.landmarks[0][8],x:pinch.landmarks[0][8].x-.01};
  await page.evaluate(hand=>window.testHand=hand,pinch);
  await expect(page.locator('[data-ui="direction"]')).toContainText('Closing');
  await expect.poll(()=>operations.filter(o=>o.operation==='gesture_move'&&o.args.joint==='gripper'&&o.args.direction<0).length).toBeGreaterThan(0);
  await page.keyboard.press('Escape');
  await expect(page.locator('.gesture-panel')).toHaveAttribute('data-active','false');
  await page.getByText('Options & help',{exact:true}).click();
  await page.getByLabel('Reverse direction',{exact:true}).check();
  await expect(page.locator('.gesture-instructions')).toContainText('Pinch to open');
  await page.evaluate(hand=>window.testHand=hand,openHand());
  await expect(page.getByRole('button',{name:'Set gesture neutral',exact:true})).toBeEnabled();
  await page.getByRole('button',{name:'Set gesture neutral',exact:true}).click();
  await enable(page);
  await page.evaluate(hand=>window.testHand=hand,pinch);
  await expect(page.locator('[data-ui="direction"]')).toContainText('Opening');
  await expect.poll(()=>operations.filter(o=>o.operation==='gesture_move'&&o.args.joint==='gripper'&&o.args.direction>0).length).toBeGreaterThan(0);
});

test('widescreen video keeps the overlay aligned and neutral must be current',async({page})=>{
  await page.goto('/hardware.html');await expect(page.locator('#stateLabel')).toHaveText('USB not connected');
  await page.locator('#connectBtn').click();await page.locator('#supportAck').check();await page.locator('#armBtn').click();
  await page.evaluate(()=>window.testCameraDimensions=[1280,720]);
  await camera(page);
  expect(await page.locator('video').evaluate(video=>[video.videoWidth,video.videoHeight])).toEqual([1280,720]);
  const fit=await page.locator('.gesture-camera').evaluate(el=>({width:el.clientWidth,height:el.clientHeight,canvas:el.querySelector('canvas').width/el.querySelector('canvas').height}));
  expect(Math.abs(fit.width/fit.height-fit.canvas)).toBeLessThan(.02);
  await page.evaluate(hand=>window.testHand=hand,openHand(-.1));
  await expect(page.getByRole('button',{name:'Enable gesture control',exact:true})).toBeDisabled();
  await expect(page.locator('.gesture-status')).toContainText('Return to neutral');
  await page.evaluate(hand=>window.testHand=hand,openHand());
  await expect(page.getByRole('button',{name:'Enable gesture control',exact:true})).toBeEnabled();
  await page.evaluate(()=>window.testTrackerStalled=true);
  await expect(page.locator('.gesture-status')).toContainText('Tracking lost');
  expect(await page.locator('.gesture-camera canvas').evaluate(canvas=>canvas.getContext('2d').getImageData(0,0,canvas.width,canvas.height).data.some(v=>v!==0))).toBe(false);
});
