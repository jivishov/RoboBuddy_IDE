import {openHand} from '../fixtures/gesture-hand.mjs';
import {sandboxAssets} from './sandbox-assets.mjs';
export async function fakeWebcam(page) {
  await sandboxAssets(page);
  await page.addInitScript(initial => {
    window.testHand = initial;
    window.testCameraTracks = [];
    navigator.mediaDevices.getUserMedia = async () => {
      if (window.testCameraDenied) throw new DOMException('Denied', 'NotAllowedError');
      const canvas = document.createElement('canvas'); canvas.width = window.testCameraDimensions?.[0] || 640; canvas.height = window.testCameraDimensions?.[1] || 480;
      const ctx = canvas.getContext('2d');
      const interval = setInterval(() => {ctx.fillStyle='#243a31';ctx.fillRect(0,0,canvas.width,canvas.height);ctx.fillStyle='#cadbd1';ctx.font='20px sans-serif';ctx.fillText('Synthetic camera · browser test',60,60);},60);
      const stream = canvas.captureStream(15); window.testCameraTracks.push(...stream.getTracks());
      stream.getTracks()[0].addEventListener('ended',()=>clearInterval(interval));
      return stream;
    };
    const NativeWorker = window.Worker;
    window.Worker = class {
      constructor(url, options) {
        if (!String(url).endsWith('/gestures/hand-worker.js')) return new NativeWorker(url, options);
        this.active=true;
      }
      postMessage(data) {
        if(data.type === 'init') return setTimeout(()=>this.active&&this.onmessage?.({data:{type:'ready'}}),window.testTrackerInitDelay || 0);
        const imageWidth=data.bitmap?.width, imageHeight=data.bitmap?.height;
        data.bitmap?.close();
        if(window.testTrackerStalled) return;
        setTimeout(()=>this.active&&this.onmessage?.({data:{type:'result', result:{...window.testHand,imageWidth,imageHeight}, capturedAt:data.capturedAt - (window.testStaleFrames ? 700 : 0)}}),5);
      }
      terminate(){this.active=false;}
    };
  }, openHand());
}
