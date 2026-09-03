# RoboBuddy Robot IDE

A standalone, VS Code-inspired browser IDE for learning robot programming through **visible physical-target Python** and immediate 3D simulation.

## Design contract

- Python is the learner-facing source of robot motion; Blockly is absent.
- The editor and 3D simulator are the dominant work areas.
- SO-101, OpenArm bimanual, and LeKiwi use their public LeRobot import/configuration/action shapes at pinned revision `7e241bd630a3719a56157a497ce5d08f244784f1`.
- Unitree G1 is a separate browser-only 29-axis kinematic pose workspace. It does not present a Unitree SDK or physical-control API.
- MicroDuck is a separate `policy_sim` workspace using the exact pinned `1x61 -> 1x14` ONNX policies and the official compact Apache-2.0 `robotctl` monitor visual over configured approximate browser dynamics. The configured five-degree movement of the exact official lower-bill part, roller attachments, visual floor alignment, contacts, and dynamics remain approximations; it is not a physics twin or hardware-control SDK.
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

MicroDuck is also outside the source-plant path. Its dedicated policy backend uses the Apache-covered fourteen-joint hierarchy, exact pinned ONNX bytes, and exact compact Apache-2.0 runtime-monitor visual. Triangle winding/normals, the configured lower-bill articulation, rollers, floor alignment, collision/contact model, browser dynamics, Overview/Follow projection and framing, modeled `head_camera` projection, IMU/ToF values, and generated audio remain explicit browser approximations. See [the MicroDuck fidelity and control record](docs/microduck-simulator.md).

## Important fidelity boundary

This IDE is a teaching/reference simulator, **not a hardware-calibrated digital twin**. Canonical visual geometry and source-pinned kinematics improve correspondence, but the application does not claim motor/controller dynamics, force or torque sensing, identified friction, compliance, backlash, payload certification, device-specific calibration transfer, glassware safety certification, wheel slip/odometry accuracy, ZMQ/CAN timing equivalence, or hardware validation.

The included SO-101, OpenArm, and LeKiwi reference actions are validated against the pinned browser kinematic/contact model. They are **not hardware-tested trajectories** and must not be represented as such. Unitree G1 reference poses are visual inspections only, not validated robot trajectories.

### Unitree G1 29-DoF rig

The G1 view uses `robot-mesh-data-unitree-g1.js` from the existing canonical RoboBuddy visual revision. The source asset has 29 articulated joints and 36 visual parts, generated from `unitreerobotics/unitree_ros@dd4fa6866e523ad61324f658d63736e4eda3a6e4`, `robots/g1_description/g1_29dof.urdf`. It is BSD-3-Clause; see [the retained notice](licenses/unitree_ros-BSD-3-Clause.txt) and [rig provenance](docs/unitree-g1-rig.md).

Its scope stops at bounded named-joint pose visualization. Dynamic balance, walking, root translation, foot contact, collision, fixed-hand grasping, torque/force control, controller timing, Unitree SDK control, and hardware validation are not simulated.

### Current Python execution mode

The current standalone IDE prepares the visible synchronous physical-target Python into a deterministic sequence of public API boundaries and then advances that sequence through the pinned source plant. This provides faithful visibility for open-loop joint/gripper/base programming, atomic stepping, collision/contact inspection, and numerical tuning.

For SO-101, OpenArm, LeKiwi, and Unitree, it is **not yet a closed-loop Python feedback runtime**: branching Python code does not currently suspend at `get_observation()`, receive post-simulation state, and resume the same Python process. MicroDuck is the narrow exception: its dedicated worker uses an incremental async bridge so the browser-only `microduck` module can read and command the live approximate policy simulation with cooperative Pause, Step, Run-to-Cursor, Stop, and cancellation. That worker boundary is not a hostile-code sandbox and exposes no physical transport.

## WebMCP agent collaboration

In a WebMCP-capable browser, a person can turn on the session-only **Agent → Assist** control in the toolbar. This exposes a small, explicit tool surface on the top-level page through `document.modelContext.registerTool(...)`; unsupported browsers continue to run the IDE normally.

- `describe_robobuddy_task` describes the selected task and its fidelity boundary.
- `read_robobuddy_workspace` returns one bounded, line-numbered page of the current unsaved Python workspace.
- `inspect_robobuddy_simulation` returns compact modeled state and recent diagnostics.
- `focus_robobuddy_workspace` moves the visible editor to a requested source line for shared review.
- `run_robobuddy_program` resets and runs the visible draft, then reports a compact outcome.
- `draft_robobuddy_cooperative_edit` makes one small, exact-match, **temporary** editor repair: it comments out the selected Python lines, adds an explanation and the proposed working replacement, and focuses that replacement for the person to inspect.

Those six base tools remain registered whenever a person has enabled Assist, including non-MicroDuck and loading/error workspaces. A seventh tool, `control_microduck_simulation`, is registered only for a ready active MicroDuck `policy_sim` workspace. Its 21 catalog commands use 25 strict disjoint schema branches for conditional duration behavior. Continuous control is limited to 20–5000 ms, one-shots have catalog-owned abort/completion behavior and an eight-second application ceiling, and results contain bounded cloned browser state. Audio still requires a trusted human unlock. The tool has no persistent source-write, save/export/publish, hardware/network/admin/shutdown, real-media, BLE, multiplayer, or hidden-data surface.

The tools are cancelled when Agent Assist is switched off or the browser cancels a call. The cooperative-edit tool is deliberately limited to 12 selected lines / 900 source characters and 24 replacement lines / 1,200 characters; it requires an exact `expected_source` match, comments out rather than deletes the original, never invokes Save, and is discarded on page refresh unless the person chooses Save. It cannot edit outside the active four-file workspace, save drafts, export files, publish work, or make a hardware-control claim. Source and console-derived outputs are labeled as untrusted content for the agent.

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

CI is configured to check JavaScript syntax, task/source provenance, physical action envelopes, learner-source visibility, canonical robot mesh construction, the MicroDuck asset/policy/UI/Python/WebMCP contracts, starter Python syntax/execution shape, HTML parsing, and the discovered headless Chromium suite. Local execution evidence is recorded in [the MicroDuck implementation record](docs/microduck-simulator.md); configured CI lanes are not described as executed until a CI host actually runs them.

## Source provenance

- Canonical robot visuals: RoboBuddy_AI `66d18a029a0caeb6a6075e681dbd9ecd6b22affa`
- Unitree G1 visual source: `unitreerobotics/unitree_ros@dd4fa6866e523ad61324f658d63736e4eda3a6e4`, BSD-3-Clause
- Reviewed task/source-plant patch: RoboBuddy_AI `75fe2669c0ab0b029986de424c69162071174df8`
- LeRobot API revision: `7e241bd630a3719a56157a497ce5d08f244784f1`
- Browser Python runtime: Pyodide `0.29.4`
- Editor: CodeMirror `5.65.16`
- 3D rendering: Three.js `0.180.0`
- MicroDuck runtime, policy, MuJoCo/ONNX Runtime Web, and per-file license/hash provenance: [`assets/microduck/manifest.json`](assets/microduck/manifest.json)

## License

Copyright © 2026 Dr. Emil Jivishov. RoboBuddy IDE is licensed under the PolyForm Noncommercial License 1.0.0: it may be used, modified, and distributed for non-commercial purposes under that license. Commercial use requires prior written permission from Dr. Emil Jivishov. See [LICENSE](LICENSE) for the controlling terms.
