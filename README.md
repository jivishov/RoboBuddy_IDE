# RoboBuddy Robot IDE

A standalone, VS Code-inspired browser IDE for learning robot programming through **visible physical-target Python** and immediate 3D simulation.

## Design contract

- Python is the learner-facing source of robot motion; Blockly is absent.
- The editor and 3D simulator are the dominant work areas.
- SO-101, OpenArm bimanual, and LeKiwi use their public LeRobot import/configuration/action shapes at pinned revision `7e241bd630a3719a56157a497ce5d08f244784f1`.
- Unitree G1 is a separate browser-only 29-axis kinematic pose workspace. It does not present a Unitree SDK or physical-control API.
- The displayed robot rigs are the canonical generated RoboBuddy_AI visual meshes and articulated joint hierarchy pinned to RoboBuddy_AI revision `66d18a029a0caeb6a6075e681dbd9ecd6b22affa`; the IDE no longer substitutes schematic box/cylinder robots.
- Learner-visible atomic joint, gripper, and mobile-base commands live in `trajectories.py` and are passed to ordinary `robot.send_action(...)` calls from `main.py`.
- No learner-facing fake `grasp()`, `attach()`, `teleport()`, or Cartesian hardware method is introduced.
- Edited commands are revalidated by the pinned RoboBuddy fixed-step collision/contact/support plant; the IDE does not weaken collision rules to make a trajectory pass.
- Simulated telemetry, contact, support, and gravity results are explicitly labeled as modeled values.
- Hardware validation remains explicitly **pending**.

## Reviewed task patch

The IDE pins the reviewed complex-lab task patch from `jivishov/RoboBuddy_AI` revision:

`75fe2669c0ab0b029986de424c69162071174df8` — `feat: add measured SO-101 missions and OpenArm equipment stack`

The task selector currently exposes the missions changed by that patch, plus a LeKiwi reference mission pinned to the same source revision:

- OpenArm — `openarm-04-filtration-workcell`: **Bimanual Heater and Ring-Stand Stack**
- SO-101 — `so101-v2-06-quantitative-transfer`: **Measured Two-Bottle Transfer Workcell**
- SO-101 — `so101-v2-08-burette-initial-reading`: **Burette Receiver Clearance Calibration**
- SO-101 — `so101-v2-09-vacuum-filtration`: **Vacuum Workcell Keep-Clear Preflight**
- LeKiwi — `lekiwi-01-beaker-courier`: **Beaker Courier**

It also exposes a local **Unitree G1 29-DoF Kinematic Pose Inspection** workspace. That workspace is sourced from the canonical mesh asset, not from the reviewed source-plant task patch.

The source scenario id, title, robot id, schema, and reference-action trace are checked before the IDE accepts them. This prevents an upstream branch from silently changing the educational task while retaining the same URL.

## Source-plant simulation

For the pinned tasks, the standalone IDE uses RoboBuddy_AI's `ScenarioV2Engine` / `PortableRobotPlant` at the same task-patch revision rather than the original standalone tool-point collision heuristic. The fixed-step plant runs at 20 ms, validates public robot action fields, advances live modeled joint/base state, checks configured collision geometry, derives contact/attachment consequences, applies support/rest rules, and reports faults at the last valid state.

The browser regression suite constructs all three canonical rigs and fully replays every pinned reference trace through this source plant. A reference trace that collides, penetrates support geometry, uses an invalid physical action field, or triggers another plant fault fails CI rather than being accepted through looser standalone rules.

The Unitree G1 rig is deliberately outside that source-plant path: the IDE has no reviewed G1 ScenarioV2 collision/contact implementation. It validates source-manifest joint ranges and interpolates the canonical mesh through browser-held pose state only.

## Important fidelity boundary

This IDE is a teaching/reference simulator, **not a hardware-calibrated digital twin**. Canonical visual geometry and source-pinned kinematics improve correspondence, but the application does not claim motor/controller dynamics, force or torque sensing, identified friction, compliance, backlash, payload certification, device-specific calibration transfer, glassware safety certification, wheel slip/odometry accuracy, ZMQ/CAN timing equivalence, or hardware validation.

The included SO-101, OpenArm, and LeKiwi reference actions are validated against the pinned browser kinematic/contact model. They are **not hardware-tested trajectories** and must not be represented as such. Unitree G1 reference poses are visual inspections only, not validated robot trajectories.

### Unitree G1 29-DoF rig

The G1 view uses `robot-mesh-data-unitree-g1.js` from the existing canonical RoboBuddy visual revision. The source asset has 29 articulated joints and 36 visual parts, generated from `unitreerobotics/unitree_ros@dd4fa6866e523ad61324f658d63736e4eda3a6e4`, `robots/g1_description/g1_29dof.urdf`. It is BSD-3-Clause; see [the retained notice](licenses/unitree_ros-BSD-3-Clause.txt) and [rig provenance](docs/unitree-g1-rig.md).

Its scope stops at bounded named-joint pose visualization. Dynamic balance, walking, root translation, foot contact, collision, fixed-hand grasping, torque/force control, controller timing, Unitree SDK control, and hardware validation are not simulated.

### Current Python execution mode

The current standalone IDE prepares the visible synchronous physical-target Python into a deterministic sequence of public API boundaries and then advances that sequence through the pinned source plant. This provides faithful visibility for open-loop joint/gripper/base programming, atomic stepping, collision/contact inspection, and numerical tuning.

It is **not yet a closed-loop Python feedback runtime**: branching Python code does not currently suspend at `get_observation()`, receive the post-physics plant observation, and resume the same Python process. RoboBuddy_AI already contains a JSPI/Python-RPC architecture for that stronger behavior; integrating that bridge is the correct future path before claiming closed-loop simulation fidelity.

## Controls

- `F5` Run
- Click `Pause` / `Resume` to hold or continue an in-progress run
- `F10` Step Action
- `Ctrl+F10` Run to Cursor
- `Shift+F5` or `Esc` Stop
- `Ctrl+S` Save draft locally
- `Ctrl+B` Toggle Explorer
- `Ctrl+J` Toggle diagnostics
- `Ctrl+Shift+P` Command Palette

## Validation

CI checks JavaScript syntax, task/source provenance, physical action envelopes, learner-source visibility, canonical robot mesh construction, starter Python syntax/execution shape, HTML parsing, and a headless Chromium runtime suite. The Chromium suite constructs all four canonical rigs, performs a complete source-plant replay of every pinned task, and exercises the IDE `Step Action` path through Pyodide for both a source-plant task and the Unitree pose workspace.

## Source provenance

- Canonical robot visuals: RoboBuddy_AI `66d18a029a0caeb6a6075e681dbd9ecd6b22affa`
- Unitree G1 visual source: `unitreerobotics/unitree_ros@dd4fa6866e523ad61324f658d63736e4eda3a6e4`, BSD-3-Clause
- Reviewed task/source-plant patch: RoboBuddy_AI `75fe2669c0ab0b029986de424c69162071174df8`
- LeRobot API revision: `7e241bd630a3719a56157a497ce5d08f244784f1`
- Browser Python runtime: Pyodide `0.29.4`
- Editor: CodeMirror `5.65.16`
- 3D rendering: Three.js `0.180.0`

## License

Copyright © 2026 Dr. Emil Jivishov. RoboBuddy IDE is licensed under the PolyForm Noncommercial License 1.0.0: it may be used, modified, and distributed for non-commercial purposes under that license. Commercial use requires prior written permission from Dr. Emil Jivishov. See [LICENSE](LICENSE) for the controlling terms.
