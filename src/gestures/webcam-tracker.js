export class WebcamTracker {
  constructor(video, {onFrame, onError}) { this.video = video; this.onFrame = onFrame; this.onError = onError; this.generation = 0; }
  async start() {
    this.stop();
    const generation = this.generation;
    if (!globalThis.isSecureContext || !navigator.mediaDevices?.getUserMedia || !globalThis.createImageBitmap || !globalThis.Worker || !globalThis.OffscreenCanvas) throw new Error('Webcam control needs a supported browser such as Chrome or Edge on HTTPS or localhost.');
    try {
      const stream = await navigator.mediaDevices.getUserMedia({audio: false, video: {facingMode: 'user', width: {ideal: 640}, height: {ideal: 480}, frameRate: {ideal: 15, max: 20}}});
      if (generation !== this.generation) { stream.getTracks().forEach(t => t.stop()); return false; }
      this.stream = stream;
      stream.getVideoTracks().forEach(track => track.addEventListener('ended', () => { if (generation === this.generation) this.fail('Camera disconnected. Restart the camera.'); }));
      this.video.srcObject = stream;
      await this.video.play();
      if (generation !== this.generation) return false;
      // The pinned MediaPipe WASM loader uses importScripts. A classic worker can
      // dynamically import its ESM API while supporting that loader correctly.
      this.worker = new Worker(new URL('./hand-worker.js', import.meta.url));
      await new Promise((resolve, reject) => {
        this.rejectInit = reject;
        this.initTimer = setTimeout(() => reject(new Error('Hand tracking took too long to load. Restart the camera.')), 25000);
        this.worker.onerror = () => { if (generation === this.generation) this.fail('Hand tracking could not start. Restart the camera.'); };
        this.worker.onmessage = ({data}) => {
          if (generation !== this.generation) return;
          if (data.type === 'ready') { clearTimeout(this.initTimer); this.rejectInit = null; resolve(); }
          if (data.type === 'error') this.fail(data.message);
          if (data.type === 'result') { this.processing = false; this.onFrame(data.result, data.capturedAt); }
        };
        this.worker.postMessage({type: 'init'});
      });
      if (generation !== this.generation) return false;
      this.lastCapture = -Infinity; this.lastVideoTime = -1;
      this.loop(generation);
      return true;
    } catch (error) {
      if (generation !== this.generation) return false;
      this.stop();
      throw new Error(error.name === 'NotAllowedError' ? 'Camera permission was denied. Allow webcam access in your browser and try again.' : error.name === 'NotFoundError' ? 'No webcam was found. Connect a camera and try again.' : error.message);
    }
  }
  loop(generation) {
    if (generation !== this.generation) return;
    this.raf = requestAnimationFrame(() => this.loop(generation));
    const now = performance.now();
    if (this.processing || this.video.readyState < 2 || this.video.currentTime === this.lastVideoTime || now - this.lastCapture < 80) return;
    this.processing = true; this.lastCapture = now; this.lastVideoTime = this.video.currentTime;
    void createImageBitmap(this.video).then(bitmap => {
      if (generation !== this.generation) { bitmap.close(); return; }
      this.worker.postMessage({type: 'frame', bitmap, capturedAt: now}, [bitmap]);
    }).catch(() => { if (generation === this.generation) this.fail('Camera frames could not be read. Restart the camera.'); });
  }
  fail(message) { this.stop(); this.onError(new Error(message)); }
  stop() {
    this.generation++;
    clearTimeout(this.initTimer); cancelAnimationFrame(this.raf);
    this.rejectInit?.(new Error('Camera stopped.')); this.rejectInit = null;
    this.worker?.terminate(); this.worker = null;
    this.stream?.getTracks().forEach(t => t.stop()); this.stream = null;
    this.video.srcObject = null; this.processing = false;
  }
}
