import {test,expect} from '@playwright/test';
import {openHand} from './fixtures/gesture-hand.mjs';
async function harness(page) {
  await page.goto('/index.html');
  await page.evaluate(async hand=>{
    const {GesturePanel}=await import('/src/gestures/gesture-panel.js');
    const sheet=document.createElement('link');sheet.rel='stylesheet';sheet.href='/gesture-control.css';document.head.append(sheet);
    const button=document.createElement('button');document.body.append(button);
    const deferred=()=>{let resolve;const promise=new Promise(r=>resolve=r);return {promise,resolve};};
    window.beginGate=deferred();window.endGate=deferred();window.ends=0;window.sends=0;window.testHand=hand;
    const adapter={label:'Lifecycle test',begin:()=>window.beginGate.promise,end:()=>{window.ends++;return window.endGate.promise;},send:async()=>{window.sends++;}};
    window.panel=new GesturePanel(button,adapter,{trackerFactory:(_video,{onFrame})=>{
      let timer;
      return {start:async()=>{timer=setInterval(()=>onFrame(window.testHand,performance.now()),40);return true;},stop:()=>clearInterval(timer)};
    }});
    window.panel.open();
  },openHand());
  await page.getByRole('button',{name:'Start webcam',exact:true}).click();
  await expect(page.getByRole('button',{name:'Set gesture neutral',exact:true})).toBeEnabled();
  await page.getByRole('button',{name:'Set gesture neutral',exact:true}).click();
}
test('concurrent pause and close await a late begin and exactly one cleanup',async({page})=>{
  await harness(page);
  await page.getByRole('button',{name:'Enable gesture control',exact:true}).click();
  await page.getByRole('button',{name:'Pause gesture control',exact:true}).click();
  await page.evaluate(()=>{window.reviewClosed=false;window.panel.close().then(()=>window.reviewClosed=true);});
  expect(await page.evaluate(()=>({closed:window.reviewClosed,stopping:window.panel.stopping,ends:window.ends}))).toEqual({closed:false,stopping:true,ends:0});
  await page.evaluate(()=>window.beginGate.resolve());
  await expect.poll(()=>page.evaluate(()=>window.ends)).toBe(1);
  expect(await page.evaluate(()=>window.reviewClosed)).toBe(false);
  await page.evaluate(()=>window.endGate.resolve());
  await expect.poll(()=>page.evaluate(()=>window.reviewClosed)).toBe(true);
  expect(await page.evaluate(()=>({enabled:window.panel.enabled,stopping:window.panel.stopping,sends:window.sends,ends:window.ends}))).toEqual({enabled:false,stopping:false,sends:0,ends:1});
});
test('motion during asynchronous enable cancels takeover before any input packet',async({page})=>{
  await harness(page);
  await page.getByRole('button',{name:'Enable gesture control',exact:true}).click();
  await page.evaluate(hand=>window.testHand=hand,openHand(-.1));
  await expect.poll(()=>page.evaluate(()=>window.panel.centered())).toBe(false);
  await page.evaluate(()=>{window.endGate.resolve();window.beginGate.resolve();});
  await expect(page.locator('.gesture-status')).toContainText('Hand moved while enabling');
  expect(await page.evaluate(()=>({enabled:window.panel.enabled,sends:window.sends,ends:window.ends}))).toEqual({enabled:false,sends:0,ends:1});
});
