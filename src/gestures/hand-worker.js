// All model inference stays in this worker. Only landmarks leave it; images are closed.
let landmarker;
self.onmessage = async ({data}) => {
  if (data.type === 'init') {
    try {
      const {FilesetResolver, HandLandmarker} = await import('https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.21/vision_bundle.mjs');
      const vision = await FilesetResolver.forVisionTasks('https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.21/wasm');
      landmarker = await HandLandmarker.createFromOptions(vision, {
        canvas: new OffscreenCanvas(640, 480),
        baseOptions: {modelAssetPath: 'https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task', delegate: 'CPU'},
        runningMode: 'VIDEO', numHands: 2,
        minHandDetectionConfidence: .7, minHandPresenceConfidence: .7, minTrackingConfidence: .7,
      });
      self.postMessage({type: 'ready'});
    } catch (error) { console.error('Hand tracker initialization failed:', error); self.postMessage({type: 'error', message: 'Hand tracking could not load. Check access to the MediaPipe model and jsDelivr, then restart the camera.'}); }
  } else if (data.type === 'frame') {
    try {
      if (!landmarker) throw new Error('Hand tracker is not ready.');
      const result = landmarker.detectForVideo(data.bitmap, data.capturedAt);
      self.postMessage({type: 'result', result: {landmarks: result.landmarks, handedness: result.handedness}, capturedAt: data.capturedAt});
    } catch { self.postMessage({type: 'error', message: 'Hand tracking stopped. Restart the camera.'}); }
    finally { data.bitmap?.close(); }
  }
};
