# Continuation Cycle 02 - Policy Simulation and Command-State Core

## Outcome

Turn the reviewed MicroDuck rig/bundle into a deterministic browser policy simulator with the complete approved runtime-compatible command vocabulary, all nine policies, both locomotion modes, truthful state, ball contact, and walking recovery. Leave it controllable through a narrow programmatic command bus, without yet building the final visible deck, Python SDK, or WebMCP tool.

## Preconditions and Evidence

- Depends on: Cycle 01 completed after its fidelity review/refinement gate.
- Verify the manifest, fixture hashes, articulated rig mapping, and current dirty checkout before editing.
- Confirm no other task owns the backend registry, `src/app-v2.js`, MicroDuck modules, profile/task/workspace catalogs, or policy tests.
- Re-read the pinned `microduck` control/observation/state design and tuning. Use the Space commit only to compare browser sequencing; do not use `microduck_rl` mesh/MJCF bytes or claims.

## Implementation Boundary

In scope:

- Implement `MicroDuckPolicySimulator` on the main thread:
  - MuJoCo step `0.005 s`;
  - policy decimation four / 50 Hz control;
  - `requestAnimationFrame` rendering driven only by the externally pauseable `SimulatorHost` clock;
  - one outstanding ONNX inference maximum, no catch-up spiral, and no independent wall-clock policy timer;
  - backend/run epochs that discard an awaited inference result after pause, stop, reset, disposal, profile change, or superseding work before it can apply targets;
  - deterministic reset/keyframes and explicit ready/error states;
  - eager seven-policy walking bundle and retained lazy roller/roller-crouch pair.
- Build the exact 61-float observation from gyro(3), projected gravity(3), home-relative policy-joint positions(14), policy-joint velocities(14), last policy action(14), and command(13) ordered twist(3)/head(4)/body(6). Map fourteen outputs through the source joint order, home pose, active-state action scale, source low-pass behavior, and joint limits; exclude the separately actuated mouth from policy observation/action. Apply source-specific standing/one-shot tuning without silently substituting the Space demo's older constants.
- Add frozen `MICRODUCK_COMMANDS` and one command bus shared by later UI, gamepad, Python, and WebMCP. It validates units/ranges, owns a single controller lease, reports requested/applied/limited values, handles deadlines/cancellation, and never merges conflicting sources. Each command records source authority, mode availability, continuous/one-shot classification, completion/timeout, cancellability, safe abort handoff, and neutral/expiry value so later integrations do not invent behavior independently.
- Implement runtime-compatible commands:
  - continuous `move`, `head`, `pose`, and `mouth`;
  - `look` IK in trunk-frame metres with source clamps and solved head values;
  - `stop`, `enable`, simulated `init`, simulated `relax`;
  - `do` for `ground_pick`, `kick_left`, `kick_right`, `sit_toggle`, and `roulade`;
  - `get_mode`, `set_mode`, and `get_state`;
  - sound/theremin/chorale state hooks that remain silent until Cycle 03 supplies the audio implementation.
- Apply current runtime semantics:
  - walking: vx/vy ±0.3 m/s, yaw ±1.5 rad/s;
  - roller: vx +0.6/-0.5 m/s, vy applied as zero with `limitedBy`, yaw ±0.3 rad/s;
  - body z -0.025..+0.010 m, roll/pitch ±0.26 rad, untrained body x/y/yaw held at zero;
  - mouth 0..1 and independent of policy actions;
  - roller retains sit, kicks, and roulade; ground-pick maps to roller-crouch;
  - one-shot priority and safe handoff follow the current runtime, not the Space UI.
- Record source parity separately from browser safety policy: the focused-human 250 ms refresh deadline, Python five-second no-call maximum, WebMCP 20..5000 ms duration, and WebMCP eight-second one-shot ceiling are application bounds. Catalog neutral values are zero movement, neutral head, inactive/neutral body pose, closed mouth, released held `wheee`, and disabled theremin/chorale.
- Add MuJoCo arena/ground and one ball with explicit spawn/reset. Contacts may move the ball; ground-pick never creates a hidden attachment.
- Implement modeled walking recovery with tip debounce, settle, stand policy, upright hysteresis, and six-second reset fallback. Roller falls reset directly and must not claim use of the stand policy.
- Publish the immutable bounded MicroDuck state defined in `meta_plan.md`, including `simulationMode: "policy_sim"`, `stateKind: "browser_policy_sim"`, `sourcePlantAvailable: false`, `policySimulationAvailable: true`, and `hardwareValidated: false`.
- Keep existing source-plant and kinematic adapters behaviorally unchanged while routing App operations through the backend contract.

Preserve or leave for later:

- No full control-deck layout, keyboard/gamepad capture, camera inset, ToF stimulus UI, audio synthesis, Python bridge, or new WebMCP tool.
- No physics/inference worker migration, network/hardware transport, device admin, multiplayer, learned perception, or fake object grasp.
- Do not expose internal MuJoCo/ORT handles or mutable state through App snapshots.

## Likely Touchpoints

- `src/microduck/command-catalog.js`
- `src/microduck/command-bus.js`
- `src/microduck/policy-simulator.js` and focused observation/policy/state modules
- simulator-backend registry/adapters and minimal `src/app-v2.js` routing
- `src/profiles.js` fidelity/limits metadata
- `tests/microduck-policy-core.mjs` plus targeted browser smoke coverage

## Acceptance Criteria

- Repeated fixed initial-state traces produce the recorded observation/action/target fixtures within tolerance and do not overlap ONNX calls.
- Walking and roller motion, head/look/body/mouth, enable/init/relax/stop, all one-shots, mode switching, limits, priority, and completion/timeout behavior match the pinned contracts.
- The walking and roller procedural geometry visibly follows the approximate browser MuJoCo joint/root state rather than canned animation, while remaining labeled non-parity dynamics.
- Ball movement arises only from simulated contacts; ground-pick has no attachment flag/path, object parenting, pose overwrite, weld/equality constraint, or scripted carry transform.
- Walking face-up/face-down perturbation enters modeled recovery and either returns upright or resets after six seconds; roller perturbation resets without a false recovery claim.
- State includes fourteen policy joints/targets plus mouth separately and distinguishes requested/applied/limited values, policy physics, existing source plant, and hardware.
- A human-priority command preempts lower-priority continuous intent; conflicting Python/WebMCP lease requests reject as `COMMAND_CONFLICT`; any cancellation/expiry applies the catalog-defined neutral/abort behavior only to the owned intent.
- Existing four robot profiles retain their prior backend, reset, apply/time, telemetry/contact, fit, pause, and disposal behavior.

## Verification Ceiling

Allowed checks:

- `node --check` for changed/new JavaScript;
- targeted unit tests for observation layout, joint/action mapping, limits, command arbitration, one-shot transitions/timing, mode mapping, immutable state, and deadman/cancellation;
- Cycle-01 manifest and fixture checks;
- a small consolidated set of deterministic browser traces covering walk/turn/head/body/mouth, all one-shot transitions, roller mode, contact-only ball movement, recovery, stale-inference rejection, and existing-profile spot checks without creating one redundant journey per behavior;
- `node tests/validate.mjs`, `node tests/validate_canonical_visuals.mjs`, and `node tests/validate_task_patch.mjs` after backend routing changes;
- focused diff review and `git diff --check`.

Do not run:

- final UI/responsive/audio/Python/WebMCP journeys, full browser suite, performance profiling, accessibility, hardware, deployment, penetration, dependency-security, or generic security checks.

Stop condition: the programmatic command/state core, deterministic approximate-browser-dynamics criteria, and exact ONNX inference criteria pass and one fidelity review/refinement pass is complete.

## Unrun Checks / Residual Risk

- Main-thread functional timing does not prove sustained laptop frame rate or thermal behavior.
- Modeled contact and recovery do not prove real balance, grasping, friction, or motor safety.
- Audio/peripheral hooks remain intentionally silent/incomplete until Cycle 03.

## Critical Fidelity Review and Refinement Gate

- Coding model and effort: record actual settings at cycle start.
- Review model and effort: ask the user after implementation and allowed checks.
- Default proposal when unspecified: use the coding model two supported reasoning levels higher, capped at its current maximum.
- Review target: observation/action order, current runtime versus older Space constants, 50 Hz/decimation/single-flight and stale-result timing, host-owned pause/time, fourteen-plus-mouth separation, skill priority and roller mappings, init/relax/stop meaning, source parity versus browser safety bounds, catalog-defined ownership/deadman/abort behavior, contact-only ball truthfulness, recovery boundaries, state immutability/capability flags, and preservation of the existing backends.
- Refinement: correct confirmed in-scope gaps and rerun only affected checks above.
- Stop after one review and one refinement pass unless the user requests another.

Do not mark completed until the selected review settings actually perform the review.

## Status Handoff

Record actual model settings, command/state catalog revision, policy/fixture results, findings/refinements, files changed, exact checks/results, unrun checks, and residual risk. Set Cycle 03 to `ready` only after the policy/command core is stable.

## 2026-08-30 Completion Record

### Actual settings and source anchors

- Coding: `gpt-5.6-sol` at `medium` in the implementation task.
- Critical fidelity review and one refinement pass: `gpt-5.6-sol` at `xhigh` in the separately assigned review task.
- Checkout remained `codex/MICRODUCK`; `HEAD` and live `origin/main` both remained `ccc50cb5018e6d532873181538583f1a6e636b5b`.
- Pinned MicroDuck authority remained `pollen-robotics/microduck@590b986bd8c0d50ae02cb3ea2f59c463b6828168`. No Space implementation byte or `microduck_rl` mesh/MJCF byte was added.
- Final command-catalog SHA-256: `98b356c5ec3e24a0ed3ea6bb3c83a4e724e731fce45fd1693ff5cd021568f779`.
- Final immutable-state module SHA-256: `34d63d30a689d84dfba4b567cc2a326cf211090c208f161aa054729972c4bdfc`.
- Cycle-01 manifest SHA-256 remained `efc7357bb708dde5600b5ac670c4fc27a3ea74b7e0176434860a5a5996d11cd8`; inference fixture SHA-256 remained `d717598ffc4809797612bb10df36c3c296437a8a1794a201b9dcbb779e34bafc`.

### Delivered policy and command/state core

- Added the exact 61-float observation layout and fourteen-output mapping with mouth excluded, pinned home pose/joint order, source action scales, head/leg low-pass values, source skill priority/timings, walking/roller limits, and source-compatible trunk-frame gaze IK.
- Added one host-clocked `0.005 s` MuJoCo loop with decimation four, at most four render-frame catch-up steps, 50 Hz single-flight ONNX control, run/gate epoch rejection, eager retention of seven walking policies, and retained lazy loading of the roller pair.
- Added the frozen 21-entry `MICRODUCK_COMMANDS` vocabulary and one command bus. Authority priority, per-source duration bounds, classification, completion/timeout, cancellability, safe abort, neutral/expiry behavior, values, and mode availability are catalog-owned. The bus tracks which intents the active owner wrote so expiry/cancellation cannot neutralize a later unleased intent.
- Added bounded immutable policy-simulation state with explicit approximate-dynamics/non-hardware flags, fourteen joints/targets plus separate mouth, requested/applied/limited motion, policy/actuation distinction, contact-only ball state, modeled sensors, and recovery state.
- Added original approximate MuJoCo motor-force control, explicit zero-force relaxation under gravity, contact-only ball movement with no attachment/weld/parent/pose override, walking stand-policy/reset recovery, and direct roller reset.
- Routed MicroDuck through `SimulatorHost` while retaining the existing source-plant/kinematic backend family and its reset, apply/time, telemetry/contact, fit, resize, pause, disposal, and stale-activation behavior.

### Critical review findings and single refinement pass

1. `robot.stop` incorrectly called App Stop, neutralizing all retained intent instead of only zeroing movement while leaving policy/actuation enabled. Refined the command path so programmatic `stop` changes movement only; trusted App Stop still cancels and neutralizes the owning run.
2. Browser `init` used a one-second ramp and enabled the policy. The pinned runtime uses a two-second home ramp with no policy. Refined `init` to cancel ownership/skills, actuate a two-second current-to-home ramp, report incomplete/`initializing`, and finish with policy disabled. Enabling from a relaxed state uses the same ramp and enables policy only afterward.
3. `relax` still drove position actuators and retained skills/leases. Replaced approximate position actuators with bounded motor/PD forces, made relax write exactly zero actuator force while MuJoCo gravity continues, and clear policy/skills/ownership. State now distinguishes policy `enabled` from `actuationEnabled`.
4. Lease priority/deadline constants were duplicated in the bus, and expiry neutralized every continuous field instead of only owner-written intents. Moved authority/deadline policy into the frozen catalog, added per-intent ownership, preserved source-conflict/preemption behavior, and covered persistent unleased `look` intent.
5. Invalid `set_mode` input was silently coerced to walking, mode loading lacked the catalog timeout, and the inference error path compared unrelated epochs. Refined validation to return `INVALID_ARGUMENT`, added the catalog-defined `POLICY_TIMEOUT`, invalidated the gate across mode/reset changes, and corrected run-epoch error publication.
6. `SimulatorHost` omitted the existing `resize()` surface used by App panel/layout changes. Added the missing backend delegate without changing source backend behavior.
7. Tightened related evidence and truthfulness: finite three-value ball spawn validation, source-style `enable(on/toggle)` handling with original requested values retained, disabled/home-ramp active-policy labels, successful-recovery perturbation clearing, current deadman-applied motion in state, and an expanded single browser trace covering all walking and retained roller one-shots plus real delayed-inference rejection.

No second review/refinement pass or Cycle 03 implementation was performed.

### Files changed for Cycle 02

- Core: `src/microduck/contract.js`, `observation.js`, `policy-director.js`, `policy-runtime.js`, `inference-gate.js`, `look-ik.js`, `command-catalog.js`, `command-bus.js`, `mujoco-dynamics.js`, `state.js`, `rig-adapter.js`, and `policy-simulator.js`.
- Backend/App integration: `src/simulator-host.js`, `src/source-simulator.js`, `src/app-v2.js`, `src/profiles.js`, and `playwright.config.mjs`.
- Focused evidence: `tests/microduck-policy-core.mjs`, `tests/microduck-policy-trace.spec.mjs`, and `tests/microduck-browser-smoke.spec.mjs`.
- Reconciliation only: this continuation file and `_CYCLE_STATUS.json`.

### Exact checks and results

- `node --check` over the 19 changed/new Cycle-02 JavaScript modules/specs: passed.
- `node --test tests/microduck-policy-core.mjs`: passed, 8/8. Covers catalog completeness/authority bounds, exact observation and mouth exclusion, limits/look/body behavior, preemption/conflicts/owned expiry, active audio duration, policy priority/timing/scaling/filtering, inference epochs/single-flight, and immutable bounded state.
- `node tests/validate_microduck_assets.mjs`: passed; 19 files, 9 policies, 14 joints, 2 release-safe scores, and 2 fixed inference outputs.
- `npx playwright test tests/microduck-policy-trace.spec.mjs`: passed, 1/1 after refinement.
- `npx playwright test tests/microduck-browser-smoke.spec.mjs tests/microduck-policy-trace.spec.mjs`: passed, 4/4. The consolidated trace covers walking/turn/head/look/body/mouth, enable/init/relax/stop, all walking one-shots, retained roller crouch/kick/roulade/sit, lazy policy loading, exact walking/roller ORT fixtures, contact-only ball motion, zero-force relax/gravity, walking/roller recovery, pause stale-inference rejection, profile-family switching, and stale backend disposal.
- `node tests/validate.mjs`: passed, `static fidelity checks: OK`.
- `node tests/validate_canonical_visuals.mjs`: passed, `canonical RoboBuddy visual-source checks: OK`.
- `node tests/validate_task_patch.mjs`: passed, `pinned latest task patch + physical action visibility: OK`.
- `git diff --check`: passed; output contained only pre-existing Windows LF-to-CRLF warnings.
- `rg -n "[ \\t]+$"` over Cycle-02 owned source/spec files: no matches; `rg` exited 1 as expected for an empty result.

### Unrun checks and residual risk

- The full browser suite, final UI/responsive/audio/Python/WebMCP journeys, accessibility, sustained frame pacing/thermal profiling, hardware, deployment, penetration, dependency-security, and generic security checks remain unrun by the Cycle-02 ceiling.
- Exact ONNX byte/output evidence does not establish RL-environment, MuJoCo-model, contact, locomotion, balance, grasp, friction, motor, or hardware parity.
- The main-thread functional trace does not prove sustained laptop frame rate. Walking recovery currently proves the allowed stand-policy-to-six-second-reset path; it does not prove physical self-righting.
- Audio, camera/ToF presentation, keyboard/gamepad, Python, and WebMCP control remain intentionally incomplete until their assigned later cycles.

Cycle 02 satisfies the revised acceptance criteria after the required xhigh review and one refinement pass. Cycle 03 is ready; no Cycle 03 code was started here.
