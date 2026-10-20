import {test,expect} from '@playwright/test';
import {sandboxAssets} from './helpers/sandbox-assets.mjs';
test('pinned MediaPipe worker loads and runs real inference without camera permission',async({page})=>{
  test.setTimeout(90000);
  await sandboxAssets(page);
  page.on('console',message=>{if(message.type()==='error')console.log(message.text());});
  await page.goto('/index.html');
  const result=await page.evaluate(()=>new Promise((resolve,reject)=>{
    const worker=new Worker('/src/gestures/hand-worker.js');
    const timer=setTimeout(()=>{worker.terminate();reject(new Error('MediaPipe worker did not finish loading/inference.'));},70000);
    worker.onerror=event=>{clearTimeout(timer);worker.terminate();reject(new Error(event.message||'Worker error'));};
    worker.onmessage=async({data})=>{
      if(data.type==='error'){clearTimeout(timer);worker.terminate();reject(new Error(data.message));}
      if(data.type==='ready'){
        const canvas=document.createElement('canvas');canvas.width=640;canvas.height=480;
        const bitmap=await createImageBitmap(canvas);
        worker.postMessage({type:'frame',bitmap,capturedAt:performance.now()},[bitmap]);
      }
      if(data.type==='result'){clearTimeout(timer);worker.terminate();resolve(data.result);}
    };
    worker.postMessage({type:'init'});
  }));
  expect(result.landmarks).toEqual([]);
});
