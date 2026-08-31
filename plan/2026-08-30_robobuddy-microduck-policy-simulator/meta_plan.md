# RoboBuddy MicroDuck Policy Simulator and WebMCP Meta Plan

## Outcome

Add MicroDuck as the fifth selectable RoboBuddy IDE robot. It must appear as a full articulated 3D rig in the existing simulator viewport and run the exact pinned MicroDuck ONNX policies against an explicitly approximate browser dynamics model. Following the authorized post-cycle visual repair below, the visible rig uses the exact compact Apache-2.0 runtime monitor visual rather than the earlier primitive shells. A user must be able to operate every approved robot-facing and simulation-facing option through the visible control deck, focused keyboard/gamepad input, the browser-only Python SDK, and one bounded WebMCP control tool.

Observable completion means:

- the four existing robot profiles and their source-plant or kinematic behavior remain intact;
- the MicroDuck walking and roller presentations render the pinned Apache-2.0 runtime monitor visual on the same fourteen-joint hierarchy and named frames, with explicitly configured mouth articulation and roller geometry;
- all nine approved policies, movement/head/body/mouth commands, skills, sounds, theremin, local chorale, four appearances, ball interaction, cameras, and modeled sensors are available;
- MicroDuck-specific runtimes and assets continue to load after non-loopback network requests are blocked once the base IDE has loaded;
- Python reads live simulation state and cooperates with Run, Pause/Resume, Step, Run-to-Cursor, Stop, reset, and cancellation;
- Agent Assist remains Off by default, the five current WebMCP tools remain unchanged, and `control_microduck_simulation` appears only for an active ready MicroDuck workspace while Assist is enabled;
- no feature is described as physical hardware validation, real camera/ToF transport, BLE chorale, multiplayer, device administration, or a secure hostile-code sandbox.

## Verified Current State

Planning inspection on 2026-08-30 established:

- Repository root: `C:/Users/EmilJivishov/Projects/RoboBuddy_IDE`.
- Branch and baseline: `codex/MICRODUCK` at `ccc50cb5018e6d532873181538583f1a6e636b5b`; live `origin/main` resolved to the same SHA.
- The checkout is intentionally dirty. Preserve tracked changes in `.github/workflows/validate.yml`, `README.md`, `index.html`, `package.json`, `playwright.config.mjs`, `src/app-v2.js`, `tests/validate.mjs`, and `toolbar-refinement.css`, plus untracked `src/webmcp/agent-facade.js`, `src/webmcp/register-ide-tools.js`, and `tests/webmcp.spec.mjs`. Recheck this list before every cycle and do not assume later changes are disposable.
- The app is a framework-free static ES-module site. It currently loads Three.js 0.180.0 and Pyodide 0.29.4, creates one `SourceRobotSimulator`, and branches between `source_plant` and `kinematic_pose` behavior.
- `CanonicalRobotRig` supplies the common articulated viewport lifecycle for SO-101, OpenArm, LeKiwi, and Unitree. MicroDuck needs the same profile/viewport/camera/reset lifecycle, but a dedicated adapter because its transforms come from MuJoCo rather than the existing action-to-pose mapping.
- `PythonRuntime.compileWorkspace()` currently runs Python first, records `send_action`/`sleep` events, and replays them afterward. That architecture cannot provide live MicroDuck state or state-dependent Python control.
- The dirty WebMCP implementation exposes exactly five session-scoped tools behind trusted human Agent Assist: describe task, read workspace, inspect simulation, focus workspace, and run program. It is cancellation- and workspace-generation-aware and exposes no write/apply/export/publish tool.
- The current authoring machine has Python 3.14 but no NumPy, MuJoCo, or ONNX Runtime Python packages. A one-time CPU-reference fixture therefore requires an isolated source-compatible environment; do not add those packages to the application dependency surface.
- No project implementation tests were run while authoring this package. Only source/repository inspection, upstream revision checks, and package validation are evidence from this planning session.

## Source Baseline and Fidelity Layers

| Authority | Pinned revision | Role |
|---|---|---|
| [MicroDuck product page](https://pollen-robotics.com/microduck/) | Page observed 2026-08-30 | Public product behaviors, morphology, sensors, and colorways |
| [pollen-robotics/microduck](https://github.com/pollen-robotics/microduck) | `590b986bd8c0d50ae02cb3ea2f59c463b6828168` | Authoritative deployed ONNX bytes, runtime command/state contracts, control tuning, gamepad mapping, sounds, theremin, chorale, kinematics, and Apache-2.0 code provenance |
| `pollen-robotics/microduck/robotctl/assets/duck.bin` | same revision | Exact Apache-2.0 compact articulated visual used by the source `robotctl monitor`; `DUCK v1`, 28 meshes, 15 bodies, 58 parts |
| [official MicroDuck browser simulator](https://huggingface.co/spaces/pollen-robotics/microduck-simulator/commit/1261013e7e28ba2a6878bd76ae573751c0e4b457) | `1261013e7e28ba2a6878bd76ae573751c0e4b457` | Known-working browser version/sequencing and behavior oracle; no code, policy, model, audio, or branding byte may be sourced from it unless that exact byte is independently licensed and hash-reconciled |

### 2026-08-30 authorized fidelity revision

The initial package proposed using `microduck_rl` mesh/MJCF assets as distributable geometry and parity evidence. Their model-license language is not sufficiently precise for this release boundary. The revised implementation must not copy, vendor, transform, or use those mesh/MJCF bytes as fidelity evidence. It instead derives the joint hierarchy, transforms, limits, inertials, and named frames solely from Apache-2.0-covered `pollen-robotics/microduck@590b986bd8c0d50ae02cb3ea2f59c463b6828168`, and surrounds them with clean-room primitive geometry and configured browser dynamics. This preserves all runtime, policy, UI, control, peripheral, Python, and WebMCP scope while narrowing physical-fidelity claims.

### 2026-08-30 authorized post-cycle visual repair

The user rejected the primitive render after Cycle 05 and explicitly authorized revisiting the earlier mesh-backed approach. A new source audit found that the pinned Apache-2.0 runtime itself already distributes `robotctl/assets/duck.bin`, the compact 3D visual used by its live monitor. That byte is separate from the RL repository's STLs, its 15 body records match the pinned runtime XML hierarchy/transforms exactly, and its license is covered by the same retained runtime Apache-2.0 file. This post-cycle repair therefore supersedes only the earlier primitive-shell choice: it vendors that exact runtime byte, decodes its 28 meshes and 58 part instances locally, reconstructs consistent triangle winding because the upstream bake sorts each face's indexes, generates browser normals, converts source Z-up to viewport Y-up, and preserves configured lower-bill/roller/floor-alignment presentation. `microduck_rl` STL/MJCF and Hugging Face Space bytes remain excluded. Collision geometry, contacts, dynamics, camera/IMU/ToF/audio, the configured five-degree movement of the exact official lower-bill part, rollers, and floor alignment remain approximate behaviors.

### 2026-08-31 authorized post-cycle learner-readable camera refinement

This refinement does not reopen Cycle 05 or change its historical evidence. The compatible `orbit`, `chase`, and `head` values are visibly named Overview, Follow, and Head POV. Overview uses aspect-aware robot-plus-ball context framing; Follow uses stable time-based robot-root tracking and whole-robot framing; Head POV renders the main viewport at the exact modeled source `head_camera` position and orientation with a configured 100° vertical projection. The redundant inset is removed. Fit, reset, and resize preserve the selected mode, and focused `C` cycles Overview → Follow → Head POV. Immutable Python/WebMCP-visible camera state includes mode, name, purpose, truthful frame, `inset: false`, and an explicit rendered-simulation/no-hardware-video boundary. These presentation choices do not alter policy bytes, dynamics, held motion, contacts, or source colors/materials.

Keep three fidelity layers explicit:

1. **Reference-aligned policy demonstrator:** exact deployed ONNX bytes from the pinned Apache-2.0 runtime, verified only for deterministic `1x61 -> 1x14` CPU/browser inference parity, using MuJoCo WASM 3.11.0 and ONNX Runtime Web 1.27.0 locally. The fourteen-joint hierarchy and named frames come from the pinned runtime `kinematics/assets/alpha/robot_walk.xml`; the visible compact mesh/part payload comes from its exact `robotctl/assets/duck.bin`. Procedural collision shapes, contact parameters, masses used by the browser model, passive rollers, configured articulation of the official lower-bill mesh, visual floor alignment, and all resulting dynamics are original approximations. Neither visual-source fidelity, ONNX inference parity nor use of MuJoCo establishes RL-environment, MJCF, locomotion, contact, or hardware parity.
2. **Runtime-compatible commands/state:** the current `microduck` command vocabulary, limits, tuning, gamepad behavior, skills, and state semantics. Where the official simulator demo differs from the current runtime, the current runtime wins for command behavior.
3. **Modeled peripherals:** the configured Overview and Follow views, the source-frame-aligned but configured-projection Head POV, derived values at the runtime hierarchy's trunk `imu` and `head_imu` frames, synthetic/raycast 8×8 ToF at the `tof` frame, generated audio, and single-tab chorale. The source model supplies named frames, but these browser values are modeled rather than hardware-calibrated measurements.

The coherent policy set is walking, standing/recovery, ground-pick, left kick, right kick, sit/stand, roulade, roller locomotion, and roller-crouch. Cycle 01 records exact filenames and SHA-256 values in one manifest rather than scattering them through source files.

## Decisions and Constraints

- Full browser simulation only; no physical robot connection, JSON-RPC socket, serial/BLE transport, WebRTC, updater, pairing, network configuration, shutdown, or real hardware action.
- Full articulated 3D rig is required. `MicroDuckRigAdapter` must implement the shared rig lifecycle and occupy the same IDE viewport as the other four robots, while reading transforms from MuJoCo state. Do not reduce MicroDuck to a sprite, schematic, canned animation, or kinematic-only pose sequence.
- Vendor the MicroDuck-specific runtime distribution and source-backed assets under `assets/microduck/` as ordinary Git bytes, not Git LFS pointer files. Put the sole machine-readable version/hash/provenance ledger in `assets/microduck/manifest.json`; retain human-readable third-party license and NOTICE text separately in `licenses/` and do not add hidden version checks elsewhere.
- Use `src/microduck/` for the clean-room browser implementation. Do not copy the Hugging Face Space's JS/UI/audio code or any `microduck_rl` mesh/MJCF byte. Decode and preserve the exact pinned `robotctl/assets/duck.bin` payload for visible Three.js geometry, including its 28 meshes, 58 part instances, baked colors, and body mapping. Generate only configured MuJoCo collision shapes and local rendering adaptations such as winding reconstruction, normals, Z-up conversion, optional tinting, and floor alignment; keep lower-bill movement and rollers explicit approximations.
- The repository's PolyForm license does not replace third-party licenses. Classify each vendored file rather than applying one license to an entire directory: pinned runtime code, structural XML, and policies follow their recorded Apache-2.0 provenance; MuJoCo, ONNX Runtime Web, and any transitive runtime files retain their own official license/NOTICE requirements. If exact permission for an indispensable byte cannot be established, Cycle 01 blocks rather than importing the ambiguous RL assets.
- Human control always has priority. A trusted manual Stop/reset cancels Python/WebMCP control and neutralizes continuous intent. Manual focused input preempts lower-priority leases. Python and WebMCP reject a conflicting active lease rather than combining commands. The 250 ms human refresh deadline is an explicit browser focus-safety policy, not a claim that it equals the pinned runtime's default deadman; the five-second Python lease and 20..5000 ms WebMCP durations are likewise application-level safety bounds.
- `robot.stop` means zero commanded movement while remaining enabled; it is not relax, limp, or emergency power-off. Browser `init` is a controlled home-pose ramp and browser `relax` disables actuation under MuJoCo gravity; both must be labeled simulation behavior.
- Ground-pick executes the trained mouth-to-ground trajectory. Never attach the ball to the mouth or claim a successful grasp/lift unless physical contact actually moves it.
- Walking mode performs modeled automatic stand recovery. Roller mode resets after a fall because the pinned bundle has no roller stand policy.
- The current dirty WebMCP/UI work is the integration baseline. Reconcile it in place and preserve its five tools, human activation, access epoch, workspace generation, cancellation, and no-write guarantees.
- Do not stage, commit, push, publish, deploy, create a PR, or clean/reset user files without separate authorization.

## Implementation Changes

### Local source bundle and articulated rig

- Add a validated local asset bundle containing the pinned runtime `kinematics/assets/alpha/robot_walk.xml`, exact `robotctl/assets/duck.bin` visual, deterministic rig/attachment metadata, the exact nine deployed ONNX files, MuJoCo WASM 3.11.0, ONNX Runtime Web 1.27.0, score data, and only traceably licensed/generated audio inputs. Record policy shapes and hashes without claiming policy/model pairing.
- Add a deterministic asset-preparation record or script that extracts the Apache-covered joint hierarchy and named frames, records the exact runtime visual, and adds only clearly identified configured presentation/collision metadata. Preserve source joint names, transforms, axes, signs, and limits; identify lower-bill articulation, roller geometry, visual floor alignment, collisions, contacts, and non-source material/tint behavior as configured approximations.
- Add one `SimulatorHost` around `simCanvas`. It delegates to either the existing `SourceRobotSimulator` family for source-plant/kinematic profiles or `MicroDuckPolicySimulator` for `policy_sim`. Give activation an epoch: dispose the current backend idempotently, cancel its retained animation-frame handle/observer/resources, and allow an asynchronously created backend to publish only if its epoch is still current; a stale backend self-disposes before attaching. Add an explicit disposed/render-loop guard to both backends so repeated profile-family switches leave exactly one WebGL renderer, resize observer, and animation loop. The host contract owns activation/reset/readiness, fit/camera, pause/resume/stop, all simulation-time advancement, command execution, telemetry/contact state, presentation options, and disposal.
- Add MicroDuck to the profile/task/workspace catalogs with `simulationMode: "policy_sim"` and a fidelity notice distinct from both source-plant and kinematic-pose claims.

### Policy engine and command bus

- Run MuJoCo and ONNX on the main thread for the first fidelity release, matching the official browser architecture. `SimulatorHost` supplies the only externally pauseable clock; no policy timer or renderer may advance independently. Never overlap inference calls or run a catch-up spiral. Tag every awaited inference with the active backend/run epoch and discard its result after pause, stop, reset, disposal, profile change, or a newer request. Eager-load the seven walking-mode policies; lazy-load and retain the roller and roller-crouch policies on first mode switch.
- Build the exact 61-value observation as gyro(3), projected gravity(3), home-relative policy-joint position(14), policy-joint velocity(14), last policy action(14), and command(13) ordered as twist(3), head pose(4), and body pose(6). Map the fourteen outputs into the source policy-joint order and home/action-scale target transform; exclude and control the mouth separately as the fifteenth wire/actuated joint.
- Port runtime tuning from the pinned `microduck` source: walking movement limits of ±0.3 m/s and ±1.5 rad/s yaw; roller forward +0.6/back -0.5 m/s, forced zero strafe, and ±0.3 rad/s yaw; body z -0.025..+0.010 m; body roll/pitch ±0.26 rad; mouth 0..1; walking/roller action scale and head/leg low-pass behavior from the source configuration.
- Use one frozen `MICRODUCK_COMMANDS` catalog for UI, keyboard/gamepad, Python, WebMCP schema generation, validation, telemetry labels, and tests. Commands are `move`, `head`, `look`, `stop`, `enable`, `init`, `relax`, `do`, `pose`, `mouth`, `sound`, `theremin`, `chorale`, `get_mode`, `set_mode`, and `get_state`; simulation extensions are `set_color`, `spawn_ball`, `reset`, `set_tof_stimulus`, and `set_camera`.
- Publish one bounded immutable state shape containing time, enabled/lifecycle/mode, requested/applied/limited motion, head/body/mouth, active policy/phase, safety/recovery, loop rate/missed ticks, fourteen joints/targets, simulated pose, contacts/ball, virtual camera, modeled trunk/head IMU, 8×8 ToF summary, audio state, color, and explicit fidelity flags.
- Continuous focused human/gamepad intent expires 250 ms after the last refresh or immediately on focus/capture loss. Python intent lasts until an explicit replacement/stop/disconnect but is neutralized on run cancellation and has a five-second maximum without a new SDK call; `sleep()` does not refresh that lease. WebMCP requires an explicit 20..5000 ms duration for continuous activation and applies catalog-defined expiry behavior: zero movement, neutral head, inactive/neutral body pose, closed mouth, release held `wheee`, and disable theremin/chorale. `theremin(false)` and `chorale(false)` are immediate stop commands and do not require duration.

### Visible controls and modeled peripherals

- Add a compact collapsible MicroDuck control deck adjacent to the simulator. Keep the current laptop layout usable at 1366×768 and preserve the existing mobile Code/Simulator split at 390×740. Prefer a new `microduck-controls.css` over expanding the dirty toolbar stylesheet.
- Expose enable, stop, init, relax, mode, drive/head/body control, look IK, mouth, every skill, seven sounds (`alarm`, `greet`, `inquire`, `peck`, `chirp`, `coo`, `wheee`), theremin, local chorale, four appearances, ball spawn/reset, robot reset, Overview/Follow/Head POV main views over the compatible orbit/chase/head values, perturbation, ToF stimulus, and state telemetry.
- Scope keyboard/gamepad input to simulator capture. Default WASD uses Q/E for kicks; ZQSD uses J/K for kicks so movement does not collide. G is ground-pick/roller-crouch, Y sit-toggle, R roulade, M mode, C camera, and Space reset. Mirror the current runtime gamepad mapping but omit Select-held shutdown.
- Model ToF with a deterministic 8×8 synthetic hand-distance source and an optional scene raycast that writes the same contract. Model camera and IMUs from MuJoCo frames. Do not call them hardware emulators or provide real media transport.
- Require trusted human audio unlock. Calls made before unlock return `AUDIO_LOCKED`. Implement deterministic local synthesis and one-to-four-voice chorale for `wistful` and `duck_strut`; exclude BLE/multiplayer and test-only `outer_wilds`.

### Live browser-only Python SDK

- Keep the existing four workspace files: `main.py`, `trajectories.py`, `robot_config.py`, and `workcell.py`. The starter remains concise; reference helpers and all option examples live across those files instead of running every behavior automatically.
- Add an async simulated `microduck` module with `connect`, `disconnect`, `move`, `head`, `look`, `stop`, `enable`, `init`, `relax`, `do`, `pose`, `mouth`, `sound`, `theremin`, `chorale`, `mode`, `set_mode`, `get_state`, `set_color`, `spawn_ball`, `reset`, `set_tof_stimulus`, `set_camera`, and cooperative `sleep`.
- Execute MicroDuck through an incremental message-correlated Pyodide-to-JavaScript bridge in a dedicated MicroDuck Python worker; MuJoCo and ONNX remain on the main thread. This keeps Stop and UI controls responsive even when user Python fails to await. Existing profiles retain their current main-thread compile/replay execution path.
- Verify Pyodide 0.29.4's async evaluation behavior before choosing the implementation. If top-level `await` needs a compile flag or wrapper, preserve original filename/line attribution and prove it with a fixture rather than assuming a wrapper-free path.
- Run resets the MicroDuck simulation, creates a fresh run/worker epoch, and executes from the top. Pause freezes physics/policy and suspends bridge result resolution. Step advances one bridge command boundary; when paused inside `sleep`, one Step advances one 20 ms control tick and keeps the source line active until the requested duration is consumed. Run-to-Cursor continues until the next bridge boundary originating at or beyond the selected line. Stop/cancel immediately invalidates the run epoch, neutralizes commands/audio, releases ownership, and terminates/recreates the MicroDuck worker if it does not cooperatively settle within the recorded grace period, leaving later runs usable.

### WebMCP integration

- Preserve the existing five tools whenever Agent Assist is On, including non-MicroDuck and loading/error workspaces whose handlers already fail closed on readiness. Register `control_microduck_simulation` only when the current workspace is ready and `simulationMode === "policy_sim"`. Make access epoch plus profile/workspace generation and readiness the registration identity, and trigger re-registration on every relevant loading/ready/error/profile transition. Expected count is zero with Assist Off, five with Assist On outside a ready MicroDuck policy workspace, and six only for an assisted ready MicroDuck workspace.
- Generate a strict disjoint `oneOf` input schema from `MICRODUCK_COMMANDS`, with a command `const` and `additionalProperties: false` in every branch. Continuous `move`, `head`, `pose`, and `mouth`, `sound(tag="wheee", hold=true)`, `theremin(active=true)`, and `chorale(active=true)` branches require `duration_ms` from 20 through 5000; explicit inactive audio branches do not.
- Return `{ ok, command, requested, applied, limitedBy, completed, state, audio }`. Use `INVALID_ARGUMENT`, `PROFILE_MISMATCH`, `SIMULATION_NOT_READY`, `SIMULATION_BUSY`, `COMMAND_CONFLICT`, `AUDIO_LOCKED`, `OPERATION_CANCELLED`, `ASSET_UNAVAILABLE`, and `POLICY_TIMEOUT` consistently.
- On execution abort, Agent Assist disablement, or profile/workspace change, neutralize continuous intent and release audio modes. Give every one-shot a catalog-defined completion, cancellability, and abort handoff. A cancellable one-shot uses its safe cancellation transition; a non-cancellable/unknown one resets the browser simulation. Never guess handoff behavior inside the tool handler.
- Update existing snapshots and inspection output to distinguish `sourcePlantAvailable`, `policySimulationAvailable`, `simulationMode: "policy_sim"`, `stateKind: "browser_policy_sim"`, and `hardwareValidated: false`.

## Development Cycles

1. **Coherent local bundle and articulated-rig gate:** provenance/licenses, local runtimes/assets, deterministic rig derivation, single-host backend seam, fifth profile, and CPU/browser policy fixture.
2. **Policy simulation and command/state core:** MuJoCo/ONNX loop, all policies/modes, command arbitration, state, ball/contact, and recovery.
3. **Control deck, inputs, sensors, and audio:** complete visible/manual/gamepad surface with laptop/compact behavior and modeled peripherals.
4. **Live Python SDK and execution controls:** incremental async bridge, starters, live state, and debugger/cancellation semantics without existing-profile regression.
5. **WebMCP control and integration reconciliation:** sixth conditional tool, strict schemas/lifetimes, final documentation, and the approved targeted/full repository gates.

Cycles are intentionally sequential because they share `src/app-v2.js`, the simulator lifecycle, profile/workspace catalogs, and the dirty WebMCP/UI integration. Do not run them concurrently or let separate sessions race on plan/status files.

## LLM Execution and Per-Cycle Fidelity Review

- Coding model and effort: not preselected. Each fresh implementation session records the actual model and reasoning effort before editing.
- Fidelity-review model and effort: ask the user after implementation and allowed checks at every cycle gate.
- Default when deferred: offer the coding model at two currently supported reasoning-effort levels higher, capped at that model's maximum. The user may approve it for one cycle, apply it to later cycles, or choose another supported setting.
- Completion rule: the selected settings must actually perform one critical fidelity review; correct confirmed in-scope gaps in one refinement pass; rerun only affected checks within the cycle ceiling; then update status.

## Verification Ceiling

Allowed across the package, only when affected by the current cycle:

- source revision, file/hash/license, Git status, schema/JSON, asset-magic, and non-LFS-pointer checks;
- `node --check` for changed JavaScript;
- existing `node tests/validate.mjs`, `node tests/validate_canonical_visuals.mjs`, and `node tests/validate_task_patch.mjs`;
- existing Python starter/reference checks through `py -3` when the changed cycle touches those contracts;
- narrowly targeted new Node tests for manifest, observation/action layout, catalog/state, command arbitration, Python bridge, and WebMCP schema/lifetime behavior;
- an isolated one-time pinned CPU-reference fixture in Cycle 01, without adding its packages to the application;
- directly affected Playwright specs at 1366×768 and one 390×740 compact-layout journey;
- non-loopback request blocking after base IDE load to prove MicroDuck-specific local loading;
- `npm run test:browser` once at the Cycle-05 integration gate;
- focused diff review, `git diff --check` for tracked implementation changes, and explicit trailing-whitespace/file-enumeration checks for newly created owned files.

Do not broaden into a generic visual-regression suite, accessibility audit, performance profile, mobile/browser matrix, hardware test, deployment/Pages check, penetration test, dependency/security scan, or generic cleanup. Stop when the current cycle's acceptance criteria and one fidelity review/refinement record are satisfied.

## Acceptance Criteria

- MicroDuck is visibly the fifth robot and uses the full articulated compact Apache-2.0 runtime monitor visual aligned to the same source hierarchy in the shared simulator viewport.
- All MicroDuck-specific assets/runtimes are manifest-verified and locally served; licenses and derivation are traceable.
- Fixed input fixtures reproduce pinned CPU observations/actions/targets within recorded tolerances, and repeated browser traces remain deterministic within the documented floating-point envelope.
- Walking, roller, all one-shots, head/look/body/mouth, init/relax/enable/stop, recovery/reset, ball/contact, colors, cameras, inputs, sensors, sounds, theremin, and local chorale satisfy the contracts above.
- Ground-pick has no hidden attachment and state/telemetry never conflates policy physics with the existing RoboBuddy source plant or physical hardware.
- Python supports live state-dependent control and all existing profiles retain their current behavior.
- WebMCP has exactly the conditional surface, bounds, cancellation, human gate, and no-write/no-hardware properties described above.
- Current local repository validation and browser tests pass at the designated gates, with executed and unrun evidence recorded truthfully; no CI-host, release, or deployment result is implied.

## Assumptions and Open Decisions

- The approved pins remain fixed even if upstream branches advance. Any upgrade is a separate plan revision requiring a new coherent-bundle and inference-fixture review.
- MuJoCo and ONNX remain main-thread for source parity. The profile-specific Pyodide worker is an execution-safety boundary for live Python, not a physics/inference worker or a general secure sandbox.
- Base IDE dependencies may still need the network during initial page load; the offline claim is limited to MicroDuck-specific runtimes/assets after the base IDE has loaded.
- No product decision remains for implementation. A cycle may block only on changed user-owned checkout state, missing source/license provenance, inability to create the pinned `1x61 -> 1x14` CPU fixture, or failed exact-byte CPU/browser inference parity.

## Unrun Checks / Residual Risk

- No physical MicroDuck, camera, ToF, IMU, speaker, gamepad hardware, BLE chorale, network transport, or hardware safety behavior is validated.
- Main-thread frame pacing and laptop thermals remain unprofiled; functional timing checks do not prove sustained performance.
- A 390×740 targeted journey does not constitute a full responsive/mobile matrix.
- Local mocked WebMCP registration does not prove discovery or cancellation in every WebMCP host.
- A terminable Pyodide worker bounds UI lockup and later-run recovery but does not make arbitrary Python trustworthy or prove memory/resource isolation against hostile code.
- No deployment, GitHub Pages, CI-host, accessibility, penetration, or dependency-security claim is authorized by this package.
