# Continuation Cycle 03 - Control Deck, Inputs, Sensors, and Audio

## Outcome

Expose every approved MicroDuck capability to a person through a compact, simulator-scoped control deck, focused keyboard/gamepad controls, virtual cameras, modeled sensors, four appearances, ball/perturbation tools, licensed local audio, theremin, and local chorale. Preserve the existing laptop and compact IDE workflows.

## Preconditions and Evidence

- Depends on: Cycle 02 completed after its fidelity review/refinement gate.
- Verify the reviewed `MICRODUCK_COMMANDS`, command bus, immutable state, simulator backend, and current dirty UI/WebMCP changes before editing.
- Confirm no other task owns `index.html`, `src/app-v2.js`, the simulator pane, shared CSS, or MicroDuck UI/input/audio modules.
- Inspect current 1366×768 and 390×740 layout behavior before deciding final deck dimensions. Preserve natural wrapping, pane reachability, the mobile Code/Simulator split, themes, Agent Assist control, and high-contrast scene behavior.

## Implementation Boundary

In scope:

- Add a MicroDuck-only control deck in/adjacent to the simulator pane. It is collapsible on laptop and contained within the existing Simulator mobile view. Use a dedicated `microduck-controls.css`; touch dirty toolbar/layout CSS only where a minimal integration rule is unavoidable.
- Provide visible, labeled controls for:
  - Enable, Stop, Init, Relax, Reset;
  - Walk/Roller mode;
  - drive joystick/sliders and requested/applied/limited values;
  - head four-axis command, trunk-frame Look target, body z/roll/pitch, and mouth;
  - ground-pick/roller-crouch, left/right kicks, sit-toggle, and roulade;
  - `alarm`, `greet`, `inquire`, `peck`, `chirp`, `coo`, and held `wheee`;
  - theremin, local chorale, piece/voice count, and audio-unlock status;
  - Cream, Graphite, Lavender, and Sky materials;
  - ball spawn/reset, robot reset, mouse perturbation, orbit/chase/head camera, and ToF stimulus;
  - active policy/phase, safety/recovery, loop rate/missed ticks, fourteen joints/targets, mouth, simulated pose/contact/ball, camera, trunk/head IMU, and 8×8 ToF summary.
- Keep control values and labels generated from `MICRODUCK_COMMANDS` and the immutable state. Do not duplicate range constants in DOM handlers or CSS data attributes.
- Activate manual controls only after explicit simulator focus/capture. Release all human intent on blur, hidden page, capture loss, profile switch, mode switch where a control becomes invalid, gamepad disconnect, or 250 ms without refresh. Label 250 ms as the browser focus-safety lease selected by this app, not the pinned runtime's deadman value.
- Keyboard mapping:
  - selectable WASD or ZQSD movement;
  - Q/E kicks for WASD and J/K kicks for ZQSD;
  - G ground-pick/roller-crouch, Y sit-toggle, R roulade, M mode, C camera, Space reset;
  - do not intercept editor shortcuts when simulator capture is off.
- Gamepad mapping mirrors the pinned runtime: left/right sticks for the current drive/head/body layer, Start policy toggle, Y head mode, B body pose, A ground-pick or roller-crouch, X roulade with held chaining, LB/RB kicks, D-pad down sit-toggle, RT/LT mouth with RT chirp and LT held wheee, and D-pad-up held three seconds for mode. Exclude Select-held shutdown and expose no device-admin action. Build a fixture from the pinned `padd` mapping rather than inferring axes from the Space UI.
- Add orbit and chase rendering plus a head-camera inset derived from the source frame. The view is rendered simulation imagery only; no WebRTC/media transport.
- Add deterministic virtual IMU values at the Apache runtime hierarchy's trunk `imu` and `head_imu` frames and an 8×8 ToF producer anchored at the source `tof` frame. The source names those sites, but the browser derives their values from approximate dynamics and does not expose calibrated hardware sensors. The default ToF source is a hand-distance slider; optional scene raycasts may write the same grid/status contract. Label both modeled.
- Add a trusted `event.isTrusted` audio-unlock action. Use local, traceably licensed/generated synthesis. Before unlock, audio commands fail visibly with `AUDIO_LOCKED` and do not silently claim playback.
- Implement theremin from the modeled ToF nearest usable range with source-like pitch/mouth/dropout hold behavior.
- Implement deterministic single-tab local chorale with one to four generated voices, head/mouth animation, `wistful`, and `duck_strut`. This is a browser-modeled substitute for the runtime's multiplayer behavior, not protocol parity. Exclude BLE discovery/conductor networking, multiplayer, `outer_wilds`, and unlicensed Space recordings.
- Update documentation/fidelity copy and the Problems/Telemetry/Contacts surfaces so modeled peripherals and audio are clearly distinguished from hardware.

Preserve or leave for later:

- No Python API implementation and no new WebMCP control tool.
- No physical gamepad pairing workflow, real camera/ToF/IMU/audio transport, BLE, multiplayer, device admin, network update, or shutdown.
- Do not make a generic redesign of the IDE, toolbar, menus, themes, source editor, bottom panel, or existing robot controls.

## Likely Touchpoints

- new `src/microduck/control-deck.js`, input/gamepad/camera/sensor/audio modules
- new `microduck-controls.css`
- minimal hooks in `index.html` and `src/app-v2.js`
- MicroDuck simulator/rig presentation methods
- `docs/microduck-simulator.md`
- `tests/microduck-ui-core.mjs` and `tests/microduck-browser-smoke.spec.mjs`

## Acceptance Criteria

- Every approved command and simulation option is visible, reachable, range-bound, and connected through the central command bus; unavailable/conflicting states explain why rather than silently doing nothing.
- A complete MicroDuck deck remains usable without clipping or covering essential Run/Stop/status controls at 1366×768 and inside the Simulator view at 390×740.
- Simulator capture prevents IDE/editor shortcut theft, and focus/capture/gamepad loss neutralizes movement within the specified lease.
- Keyboard and gamepad mappings match the contract, including the WASD/ZQSD kick distinction and absence of shutdown.
- Four colors change presentation materials only, without modifying physics, policy observation, contacts, or fidelity state.
- Ball/reset/perturbation, orbit/chase/head camera, frame-derived virtual IMUs, and synthetic/raycast ToF behave deterministically and remain labeled modeled; source frame presence is not presented as sensor calibration evidence.
- Audio cannot unlock through a synthetic click. After trusted unlock, all seven sounds, held wheee, theremin, and both release chorales operate locally; `outer_wilds` and multiplayer remain absent.
- Existing profiles, themes, high-contrast scene, Agent Assist, laptop layout, and mobile Code/Simulator switching remain intact.

## Verification Ceiling

Allowed checks:

- `node --check` for changed/new JavaScript;
- targeted unit tests for control rendering from catalog, input leases, key-layout mapping, gamepad mapping, camera/sensor transforms, ToF stimulus/dropout, audio lock, theremin mapping, and chorale piece allowlist;
- targeted Playwright at 1366×768 for complete deck reachability and behavior;
- one 390×740 Playwright journey for Simulator view/deck reachability and Code view preservation;
- trusted versus programmatic audio-unlock assertions without claiming speaker fidelity;
- Cycle-02 policy traces affected by UI integration and existing profile/layout spot checks;
- `node tests/validate.mjs` after HTML/CSS/integration changes;
- focused diff review and `git diff --check`.

Do not run:

- full browser suite, generic visual regression, full mobile/browser matrix, accessibility audit, audio-quality analysis, performance profiling, hardware/gamepad-device tests, deployment, penetration, dependency-security, or generic security checks.

Stop condition: the complete human control/peripheral surface meets the laptop/compact acceptance criteria and one fidelity review/refinement pass is complete.

## Unrun Checks / Residual Risk

- Synthetic gamepad events do not prove every physical controller's browser mapping.
- Generated browser audio checks do not prove loudness, latency, speaker quality, or physical mouth synchronization.
- Camera/IMU/ToF values are simulation outputs, not calibrated sensor evidence.
- Two viewport checks are targeted reachability evidence, not a responsive or accessibility matrix.

## Critical Fidelity Review and Refinement Gate

- Coding model and effort: record actual settings at cycle start.
- Review model and effort: ask the user after implementation and allowed checks.
- Default proposal when unspecified: use the coding model two supported reasoning levels higher, capped at its current maximum.
- Review target: complete option coverage, catalog-driven ranges, browser safety lease versus runtime semantics, current gamepad semantics/no shutdown, 1366×768 and compact reachability, physics-independent colors, source-frame versus modeled camera/IMU/ToF claims, trusted audio unlock, locally licensed/generated sound/theremin/chorale completeness, release-piece allowlist, and existing UI/profile preservation.
- Refinement: correct confirmed in-scope gaps and rerun only affected checks above.
- Stop after one review and one refinement pass unless the user requests another.

Do not mark completed until the selected review settings actually perform the review.

## Status Handoff

Record actual model settings, control/input/audio decisions implemented, findings/refinements, files changed, exact checks/results, unrun checks, and residual risk. Set Cycle 04 to `ready` only after the human-facing surface is stable.

## Completed Review Handoff - 2026-08-30

### Actual Settings and Result

- Implementation model / effort: `gpt-5.6-sol` / `xhigh`.
- Critical fidelity review and single refinement pass: `gpt-5.6-sol` / `xhigh`.
- Result: **PASS**. Cycle 03 is complete and Cycle 04 may become `ready`. No Cycle 04 implementation was started.
- Baseline rechecked before review: branch `codex/MICRODUCK`; `HEAD` and live `origin/main` were both `ccc50cb5018e6d532873181538583f1a6e636b5b`. The intentional dirty checkout remained in place.

### Delivered Control and Fidelity Decisions

- The complete catalog-backed deck remains inside the existing simulator workbench. All manual commands are visibly present; Stop and Reset remain available as safety actions while other command, range, audio, appearance, camera, sensor-stimulus, and perturbation controls require explicit capture.
- Capture is scoped to the complete simulator workbench rather than only the canvas. Moving focus between the canvas and deck retains capture; leaving the workbench, hiding/blurring the page, switching profile/mode, disconnecting a gamepad, resetting, or manually releasing neutralizes owned human intent. Movement refresh and expiry use the catalog-owned 250 ms browser safety lease.
- WASD/ZQSD differences remain explicit. The pinned browser gamepad adapter now uses the source 0.1 deadzone, 0.3 RT/LT threshold, three-second D-pad-up mode hold, corrected browser-axis signs for drive/head/body, immediate neutralization on layer change, and no Select/shutdown action. Held X uses a cataloged 950 ms near-completion re-trigger so the one-second browser roulade director chains without continually resetting the one-shot policy.
- Repeated held-audio lease refreshes no longer recreate the held `wheee` oscillator or restart an unchanged chorale. Trusted audio unlock, all seven generated sounds, theremin, `wistful`, and `duck_strut` remain local-only; `outer_wilds`, recordings, BLE, and multiplayer remain absent.
- Orbit/chase/head cameras remain deterministic; the source site-frame orientation is converted to Three's camera-forward convention. The head IMU gyro is now derived from successive `head_imu` frame quaternions instead of joint positions, and optional ToF raycast mode writes an actual 8x8 ball-scene sample from the source `tof` frame into the same modeled grid contract.
- Color changes remain presentation-only. A paused browser assertion compared simulated pose, joints, targets, contacts, and ball state before/after Lavender and found them unchanged.

### Critical Review Findings and Single Refinement

1. **Confirmed and corrected:** canvas `blur` released capture when a user clicked the adjacent deck, while deck buttons could execute without capture. Capture/release ownership is now workbench-scoped and all non-safety deck actions are capture-gated.
2. **Confirmed and corrected:** controller timing/deadzone/threshold constants were duplicated in handlers, and several browser-standard head/body axis signs differed from the pinned `padd` mapping. These values now come from `MICRODUCK_COMMANDS`, signs are explicit, RT/LT use the pinned 0.3 analog threshold, and layer changes neutralize the prior lease immediately.
3. **Confirmed and corrected:** held `wheee` and chorale refreshes restarted Web Audio nodes every 100 ms. Refresh is now idempotent for an unchanged held sound/piece/voice count.
4. **Confirmed and corrected:** head IMU output was dimensionally derived from head angles, and the optional ToF `raycast` label used only a planar ball-distance approximation. Head angular velocity is now frame-delta derived and raycast mode performs deterministic source-frame scene sampling.
5. **Reviewed with no further product change required:** complete command/option coverage, catalog ranges, 1366x768 and 390x740 reachability, four-color physics independence, trusted-versus-programmatic audio unlock, existing profiles/themes/high-contrast/Agent-off behavior, and the affected Cycle 02 policy path all passed the allowed checks.

### Files Changed by the Review / Refinement

- `src/microduck/command-catalog.js`
- `src/microduck/input-controller.js`
- `src/microduck/control-deck.js`
- `src/microduck/audio-engine.js`
- `src/microduck/peripherals.js`
- `src/microduck/policy-simulator.js`
- `tests/microduck-ui-core.mjs`
- `tests/microduck-browser-smoke.spec.mjs`
- `plan/2026-08-30_robobuddy-microduck-policy-simulator/CONTINUATION_CYCLE_03.md`
- `plan/2026-08-30_robobuddy-microduck-policy-simulator/_CYCLE_STATUS.json`

### Executed Checks

- `node --check` on the changed Cycle 03 source and test modules: **PASS**.
- `node tests/microduck-ui-core.mjs`: **PASS** (catalog coverage, WASD/ZQSD, pinned gamepad signs/deadzone/trigger threshold, deterministic ToF, frame angular velocity, audio lock and held-node idempotence).
- `node tests/microduck-policy-core.mjs`: **PASS**, 8/8.
- `npx playwright test tests/microduck-policy-trace.spec.mjs`: **PASS**, 1/1 affected Cycle 02 consolidated policy trace.
- `npx playwright test tests/microduck-browser-smoke.spec.mjs`: **PASS**, 5/5. This includes the targeted 1366x768 deck/capture/audio/color/camera/IMU/ToF journey, the 390x740 Simulator-to-Code journey, backend-family preservation, stale activation disposal, and pinned ORT fixtures.
- `node tests/validate.mjs`: **PASS** (`static fidelity checks: OK`).
- `git diff --check`: **PASS**; output contained only pre-existing LF-to-CRLF working-copy warnings on tracked dirty paths, with no whitespace errors.

### Explicitly Unrun / Residual Risk

- Unrun by the Cycle 03 ceiling: full browser suite, generic visual regression, broader responsive/browser matrix, accessibility audit, audio-quality/loudness/latency analysis, performance profiling, physical gamepad/device pairing, calibrated camera/IMU/ToF or other hardware validation, deployment, penetration/dependency-security checks, and all Cycle 04 work.
- Synthetic mapping fixtures and browser Gamepad API logic do not prove every physical controller exposes identical axes/buttons.
- Web Audio assertions prove trusted gating, allowlists, and node behavior, not speaker quality or physical mouth timing.
- Camera, IMU, and ToF remain explicitly modeled outputs over approximate browser dynamics; source frame names and deterministic sampling are not calibration evidence.
- The two targeted viewports prove the required journeys, not a complete responsive or accessibility matrix.
