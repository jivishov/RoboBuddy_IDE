# RoboBuddy Robot IDE

A standalone, VS Code-inspired browser IDE for learning robot programming through **visible physical-target Python** and immediate 3D simulation.

## Design contract

- Python is the learner-facing source of robot motion.
- No Blockly dependency.
- The editor and 3D simulator are the two dominant work areas.
- SO-101, OpenArm bimanual, and LeKiwi starter profiles use the corresponding public LeRobot-style `connect()`, `send_action()`, `get_observation()`, and `disconnect()` structure.
- Joint/action limits fail closed before simulation.
- The OpenArm flask example creates attachment only from modeled bilateral finger contact while the gripper closes. The Python program contains no fake `grasp()`, `attach()`, `teleport()`, or Cartesian hardware method.
- Pyodide records the synchronous physical API calls and `time.sleep()` boundaries, then the browser replays that deterministic command queue through the kinematic/contact simulator. This is intentionally **not** a claim about servo timing or hardware dynamics.
- Simulated telemetry, contacts, gravity/support, and collision-style diagnostics are labeled as modeled values.
- Hardware validation remains explicitly **pending**.

## Controls

- `F5` Run
- `F10` Step Action
- `Ctrl+F10` Run to Cursor
- `Shift+F5` or `Esc` Stop
- `Ctrl+S` Save draft locally
- `Ctrl+B` Toggle Explorer
- `Ctrl+J` Toggle bottom diagnostics panel
- `Ctrl+Shift+P` Command Palette

## Fidelity limits

This standalone IDE is a teaching/reference simulator. It does not claim motor/controller dynamics, torque or force sensing, compliance, backlash, payload certification, physical calibration transfer, glassware safety, wheel slip/odometry accuracy, or hardware validation.

The SO-101 and OpenArm action envelopes were aligned to the limits already pinned in RoboBuddy's compatibility sources. The displayed robot geometry is a lightweight articulated teaching representation rather than the full production mesh set. The IDE uses a lightweight CodeMirror editor rather than Monaco so the static lab remains smaller and simpler to deploy.

## Source provenance

The physical-target API shapes are aligned to the LeRobot revision `7e241bd630a3719a56157a497ce5d08f244784f1` used by RoboBuddy's compatibility catalog. OpenArm left/right joint limits follow that pinned OpenArm follower configuration; SO-101 limits follow the official URDF mechanical envelopes already pinned by RoboBuddy. Browser execution uses Pyodide 0.29.4.
