# RoboBuddy Robot IDE

A standalone, VS Code-inspired browser IDE for learning robot programming through **visible physical-target Python** and immediate 3D simulation.

## What this build fixes

The site root contains an actual `index.html` application rather than relying on GitHub Pages/Jekyll to render this README. A `.nojekyll` marker is included so Pages serves the static IDE directly.

## Design contract

- Python is the learner-facing source of robot motion; Blockly is absent.
- The editor and 3D simulator are the dominant work areas.
- SO-101, OpenArm bimanual, and LeKiwi starter profiles use their public LeRobot import/configuration/action shapes at pinned revision `7e241bd630a3719a56157a497ce5d08f244784f1`.
- Joint/action envelopes fail closed before simulated motion.
- OpenArm flask attachment is a consequence of modeled bilateral finger contact while closing. Learner code contains no fake `grasp()`, `attach()`, `teleport()`, or Cartesian hardware method.
- Simulated telemetry and contacts are labeled as modeled values.
- Hardware validation remains explicitly **pending**.

## Important fidelity boundary

This standalone IDE is a teaching/reference simulator, **not a hardware-calibrated digital twin**. The displayed robot geometry is deliberately lightweight. It does not claim motor/controller dynamics, force or torque sensing, friction identification, compliance, backlash, payload certification, device-specific calibration transfer, glassware safety certification, wheel slip/odometry accuracy, ZMQ/CAN timing, or hardware validation.

The OpenArm starter pose values are physical-target action dictionaries but are browser-reference poses only. They must not be represented as hardware-tested trajectories.

### Current Python execution mode

The present standalone runtime is intentionally **open-loop command-queue simulation**. Pyodide executes the visible synchronous physical-target Python against compatibility modules, records `connect()`, `send_action()`, `get_observation()`, `disconnect()`, and `time.sleep()` boundaries, and the browser then replays the validated action sequence through the articulated/contact model.

Accordingly, `get_observation()` while Python is constructing that queue reflects compatibility command state; it is **not live hardware feedback and is not a closed-loop readback from the post-collision browser simulation**. Programs that branch on live observations require a future synchronous JSPI/RPC bridge before RoboBuddy can claim closed-loop simulation fidelity. This limitation is documented rather than hidden.

## Controls

- `F5` Run
- `F10` Step Action
- `Ctrl+F10` Run to Cursor
- `Shift+F5` or `Esc` Stop
- `Ctrl+S` Save draft locally
- `Ctrl+B` Toggle Explorer
- `Ctrl+J` Toggle diagnostics
- `Ctrl+Shift+P` Command Palette

## Source provenance

- LeRobot API revision: `7e241bd630a3719a56157a497ce5d08f244784f1`
- Browser Python runtime: Pyodide `0.29.4`
- Editor: CodeMirror `5.65.16`
- 3D rendering: Three.js `0.180.0`
