# SO-101 webcam hand control

Hand control is available from the hand icon in the SO-101 simulator header and the local hardware panel header. It controls one selected joint at a time, including all five arm joints and the gripper. It is relative joint jogging, not Cartesian hand-pose imitation or camera-based collision avoidance.

## Use

1. In the IDE, choose **SO-101**. For a physical arm, run `Start-RoboBuddy-Hardware.cmd`, connect read only, verify calibration and explicitly enable the supported arm's motors using the existing hardware controls. Do supervised commissioning before operating hardware; this change does not perform it.
2. Open the **hand icon** and choose **Start webcam**. Allow camera access. A preview alone cannot move either arm. HTTPS or localhost, a browser supporting webcam capture, workers and ImageBitmap, and access to the model/CDN are required.
3. Select a joint. Show one hand fully inside the preview; hold it steady until **Set neutral** becomes available. Set neutral and explicitly **Enable**.
4. With an open palm, move left/right in the mirrored preview to jog the selected joint. Return to the marked neutral band to hold. **Reverse direction** is available while paused, followed by a new neutral capture.
5. For **Gripper**, keep middle/ring/little fingers extended. Set neutral with thumb and index slightly apart. Reduce that gap to close and increase it to open; return to the captured gap to hold.
6. Make a fist, choose **Pause**, or press **Esc** to pause. Tracking loss, multiple hands, a changed hand, a large tracking jump, camera failure or a hidden/unfocused page also disarm input. Set neutral and explicitly enable again. Closing the panel or hiding the tab releases the webcam.

Programs and Agent Assist cannot operate the simulator while gestures own it. Stop, reset and workspace changes end gesture input. Simulation and physical hardware are separate sessions; enabling one does not enable the other. Camera input is not exposed to WebMCP.

## Motion and stop behavior

- The simulator sends small radian targets through its existing MuJoCo PhysicsSession and advances fixed 0.1-second simulation intervals. The existing renderer, contacts and task evaluator consume those observations. No joint/body transforms are assigned by the gesture code. Both joint ranges and actuator ranges bound targets. Pausing holds simulation time; enabling resumes the same session without resetting the scene.
- The local helper requires calibrated fresh encoders, explicit motor enable, a support acknowledgement and an explicit gesture session for one named joint. Manual jogs are rejected during that session. A direction packet targets at most **0.5°**, or **1% gripper opening**, ahead of measured position. Existing **5°/s** and **10%/s** goal-rate limits remain in effect; there is no motion queue.
- Gesture packets arrive at most 10 times/second, must be no more than 250 ms old at the helper, and renew a separate **350 ms** camera lease. Ordinary operator heartbeats cannot renew it. Expiry requests a measured-position hold, clears the pending target and invalidates the gesture token. The watchdog runs every 100 ms; actual stop latency also includes scheduling, motor I/O and mechanical response. This is a software hold with torque remaining enabled, not an independently verified emergency stop.
- Browser results older than 300 ms are discarded. A stale/stalled camera disarms rather than automatically reacquiring motion. Model quality, lighting, occlusion and camera placement affect usability. Keep the independent hardware cutoff accessible; the webcam does not observe obstacles or verify a physical stop.

## Camera and dependencies

Frames are processed locally by a dedicated worker using MediaPipe Tasks Vision **0.10.21**, CPU inference and the version **float16/1** Hand Landmarker model. Frames and landmarks are not uploaded. Only bounded numeric joint-direction commands go to the loopback hardware helper when hardware control is enabled. No AI API key or Python camera package is needed. First use downloads JavaScript/WASM from jsDelivr and the model from Google Storage. Failed downloads produce a recoverable error without enabling motion.

Sources: [MediaPipe Web Hand Landmarker](https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker/web_js), [MediaPipe source and Apache-2.0 license](https://github.com/google-ai-edge/mediapipe), [pinned package](https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.21/).

## Verification

The automated checks use synthetic hand landmarks, an artificial browser webcam and the existing demo motor bus; they do not open USB. They cover mirroring, pinch direction, neutral/deadband, malformed/multiple hands, explicit enable, motor gating, stop invalidation, delayed packets, independent watchdog expiry, bounded targets, gripper units, stream speed, camera permissions and lifecycle, plus integration with the real browser MuJoCo session. A separate real-worker check can load the pinned model and run inference without a camera.

Live-camera usability, physical motor direction/alignment, calibration, collision clearance and stopping response still require supervised testing on the user's actual SO-101. Existing hardware commissioning status remains pending.

### Software checks completed October 20, 2026

- `python3 -m hardware_bridge.check_driver`: released LeRobot 0.6.0 imports/signatures passed without opening USB.
- `python3 -m unittest discover -s tests/hardware -q`: **53 passed**, including 11 gesture-specific controller cases.
- `node tests/gestures-core.mjs`, both existing hardware JavaScript checks, `node tests/validate.mjs` and `node tests/webmcp-core.mjs`: passed.
- `npx playwright test --config playwright.hardware.config.mjs`: **5 passed**, including the existing manual-control and responsive-layout checks.
- `npx playwright test tests/so101-gestures-browser.spec.mjs --repeat-each=3 --workers=1`: **3 passed** against the real MuJoCo worker using synthetic camera landmarks. Joint movement, one authority, program exclusion, pause, re-enable, profile changes and camera release passed.
- `npx playwright test tests/hand-worker-browser.spec.mjs`: **1 passed**, loading the actual pinned MediaPipe JS/WASM/model and running inference on a blank image. No hand was detected, as expected; this verifies the real worker/runtime path rather than live-camera recognition quality.
- Browser layouts were visually inspected at 1366 × 768; narrow panel layouts and hardware controls were checked down to 320 pixels. The camera panel leaves the robot viewport visible on a laptop.

The browser commands in this sandbox used the optional `ROBOBUDDY_TEST_CURL_PROXY=1` transport in `tests/helpers/sandbox-assets.mjs`. It fetches the exact public CDN bytes through the runner's normal network connection; it does not substitute model, mesh or physics behavior. Normal CI uses direct browser downloads.
