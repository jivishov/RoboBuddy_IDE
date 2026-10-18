# SO-101 manual hardware control

## Scope and current evidence

The manual panel is a separate page in the existing RoboBuddy IDE repository. The simulation editor, MuJoCo, Reset, simulation Stop and WebMCP never obtain a hardware handle. The first release supports bounded one-joint jogs, calibrated gripper percent, read-only diagnostics, explicit torque enable/disable, calibration restore and a guided calibration workflow. It does not execute user Python, saved trajectories, Cartesian moves or agent programs on hardware.

All physical commissioning is pending. Tests use a clearly identified fake bus and a released LeRobot class with its wire replaced by a spy. Real driver import/construction checks never open serial. Neither passing test group establishes actual motion accuracy, stopping latency or hardware safety.

## Panel layout

The title, helper status, operation feedback and demo notice share a compact header. Collapsible sections use quiet plus/minus marks instead of arrow glyphs. Connection and calibration share a collapsible setup section. It starts open and folds away when a calibrated arm connects; an uncalibrated connection leaves setup open. Reopening it preserves the selected port and calibration name, and folding it does not send a motor command. Its summary always shows the connection and calibration status.

Motor enable, torque off, the support acknowledgement and control readiness stay beside the manual joints. Compact outline buttons have accessible names and hover/focus tooltips. Enable and torque-off keep visible captions; Stop uses a distinct red octagon and stays in the sticky header. Each row separates the encoder value and raw ticks, the 1° / 2% jog step, and the absolute target with an apply icon. The adjacent encoder view includes motor state, sample age and port. On narrow screens the joint controls wrap into two rows and the monitor follows below.

## Windows preparation

Use this repository checkout, not files under the ChatGPT project's read-only sources folder. One-time setup:

```powershell
.\Setup-RoboBuddy-Hardware.ps1
```

This creates .hardware-venv and installs the released LeRobot 0.6.0 hardware extra, pyserial 3.5 and Feetech SDK 1.0.0. Python 3.12 is preferred; the script can use an installed Python 3.12 or later. Full LeRobot includes core ML dependencies; no training extras, cameras, policy weights or GPU service are requested. The dependency download can therefore be substantial. For an existing verified LeRobot installation, -ReuseInstalledPython creates a venv that can reuse its packages; the driver version is still checked.

Start:

```powershell
.\Start-RoboBuddy-Hardware.ps1
```

Or double-click Start-RoboBuddy-Hardware.cmd. The helper runs hidden, its panel opens visibly, and logs stay under .hardware-logs. It listens only on 127.0.0.1:8765. Nothing opens a COM port until Connect. Quit Helper stops/holds when communication is available and closes the port; it does not automatically remove torque.

Demo without an arm:

```powershell
.\Start-RoboBuddy-Hardware.ps1 -Demo -Port 8766
```

Demo is selected only by an explicit server flag; changing browser parameters cannot switch a real helper into fake mode. Demo never constructs a real motor bus. The entire page identifies synthetic readback.

If PowerShell policy blocks a downloaded script, follow your existing device policy; the launcher does not change execution policy. The equivalent direct command from the checkout is:

```powershell
.\.hardware-venv\Scripts\python.exe -m hardware_bridge.server --open
```

## Panel pairing

The helper's local panel bootstraps its own session without displaying its token. The public hardware page can also pair with the helper using a token supplied locally; public origins cannot retrieve the bootstrap token. The initial supported entry point is the helper's local page, which avoids cross-origin browser permission differences. The public page exposes its address/code fields but does not silently switch to another helper, weaken browser permissions or use a public tunnel.

Only https://robobuddy.dev and the exact local helper origins are allowed. Every mutation needs an approved Origin, bearer token and live operator session. Tokens are kept in memory, never in URLs, source files, local storage or logs. One operator owns the session. A three-second heartbeat lease expires on browser loss. Requests have sequence numbers and a two-second freshness window; commands are not retried automatically.

## Before connecting the actual assembled arm

Record controller type, servo model, firmware, supply voltage and calibration identity. This adapter targets the standard SO-101 follower with six STS3215 motors using IDs 1 through 6. It never assigns IDs, changes baud rates or flashes firmware. An assembled arm with the wrong IDs needs the official individual-motor setup procedure outside the normal Connect operation.

Clamp the arm to a stable work surface. Connect laptop USB to the servo controller, the motor-rated supply to the controller power input, and the assembled servo harness to the controller. Verify voltage and polarity from the actual motor/controller labels; do not infer voltage from the SO-101 name. For the Waveshare board described by the official guide, confirm the B/USB jumper positions. Keep the independent power cutoff accessible and start without a payload.

Official setup and middle-pose guide: https://huggingface.co/docs/lerobot/so101

## Read-only connection and calibration

Select the correct COM port and the same arm name/id previously used by LeRobot. Connect uses robot.bus.connect() and diagnostic reads, not SO101Follower.connect(), configure(), calibrate() or a torque-disabled context. This matters because the high-level connect configures registers and upstream configuration re-enables torque on context exit. Closing a read-only connection passes disable_torque=False; it does not drop a supported arm as a hidden side effect.

A matching saved calibration enables the next step. If calibration is missing, restore a known JSON file through Import & Apply, or use the guided wizard. Both are explicit write operations and require the supported-arm acknowledgement and confirmed disabled torque. Existing calibration is backed up before changes. The wizard has separate Start, Middle Pose, Range Recording and Save steps; no stdin prompts or automatic motor-ID setup run in the background. Cancelling after writing homing offsets does not automatically restore old EEPROM settings: reapply the saved backup or complete calibration before arming.

The standard upstream wrist-roll convention uses the full encoder turn. Verify actual cable routing and usable wrist range during commissioning before allowing repeated wrist motion. The local manual limit of three degrees per command does not certify mechanical clearance.

## Motor enable and manual control

Enable starts only from read-only mode with matching calibration, fresh readings, no preexisting motor torque and the operator acknowledgement. The helper selects position mode, writes the current raw encoder positions as goals, then enables torque. It does not write a home pose, cached target, PID tuning or gripper current limits. Existing motor-controller behavior must be checked on the actual arm.

Arm angles match LeRobot use_degrees=True; raw ticks are also available. Gripper position is calibrated 0–100 percent, not a measured jaw distance. Angular zero is the calibration-range midpoint. Visual alignment is a separate direction/offset mapping and never feeds back into motor commands.

The panel accepts one joint per jog. The maximum requested displacement is three degrees or five gripper percentage points from fresh measured state. Bridge targets ramp at up to five degrees/second or ten gripper points/second, and each jog expires after three seconds. These are software target bounds, not guaranteed measured hardware velocity or deceleration. There is no command queue. Sampling is nominally ten Hz; record actual bus latency before commissioning and adjust only with measured justification.

## Stop and failure semantics

- Stop clears pending movement, reads current encoders and requests a hold. Motors remain enabled. It is not a certified emergency stop and does not prove instantaneous deceleration.
- Disable Motors is separate, requires physical support, writes torque off and checks torque readback.
- Browser loss causes the local lease watchdog to request hold if the helper and USB connection still work. The next operator must explicitly regain control; old commands cannot replay.
- USB loss, a helper/process crash, a blocked driver or power failure may prevent any stop request from reaching motors. The UI reports unknown/unconfirmed physical state. Use the independent physical cutoff as appropriate; do not infer successful stopping from a disabled UI button.
- Disconnect attempts hold, closes the port without automatically removing torque, and reports unmonitored motor state. A failed hold remains recorded as unconfirmed. It does not falsely report disabled motors.

## 3D view

Numeric values and illustrative joint transforms come from encoder readback. The canonical robot mesh is the existing pinned RoboBuddy model. Geometry/zero/direction alignment must be physically checked; the page explicitly identifies it as unverified. Only presentation offsets are editable. Simulated blocks, contact signals and task success are not displayed as actual workcell evidence. External mesh/CDN failure leaves numeric controls available and reports the missing view.

## Software checks

```powershell
python -m unittest discover -s tests/hardware -v
node tests/hardware/joint-map-core.mjs
python -m hardware_bridge.check_driver
npx playwright test --config playwright.hardware.config.mjs
```

The hardware workflow runs driver checks on Windows and Linux, plus a Chromium demo-panel test. It does not open a physical port. The browser screenshots have an explicit DEMO banner. The existing application validation workflow remains separate.

## Final commissioning — intentionally not executed without the arm

1. Record the exact arm/controller/power/firmware/calibration identity.
2. Connect read-only; confirm all six motors, stable raw readings, torque status and actual sampling latency.
3. Support the torque-disabled arm and move it by hand. Check numeric readings, joint direction, gripper normalization and visual alignment.
4. Reuse matching calibration or complete the explicit supported-arm wizard. Check all range endpoints and cable constraints.
5. Enable holding at the supported current pose. Verify no unexpected jump and test Disable Motors while supporting it.
6. Test one small unloaded joint jog, then each remaining joint and the gripper. Measure requested versus actual response.
7. Test software Stop and browser lease expiry with small movements. Measure the response. Check USB loss/reconnection in a supported configuration; never assume a guaranteed motor stop after losing the link.
8. Confirm reconnect never re-arms or replays previous goals. Keep supervised manual-only status until results are recorded.

Do not relabel the arm as hardware-tested based on demo or import checks. Save commissioning evidence before extending physical control to programs, replay or WebMCP.

