# MicroDuck learner-readable camera refinement

## Scope and fidelity contract

- **Audience and objective:** learners using the RoboBuddy IDE at representative desktop sizes must be able to inspect the articulated MicroDuck and ball, follow modeled movement, and deliberately enter the modeled source head-camera point of view.
- **Runtime:** the existing static Three.js/MuJoCo/ONNX browser workspace. Preserve the held-by-default state, nine policy bytes, materials, dynamics, command IDs, Python/WebMCP boundaries, and dirty checkout.
- **Fidelity tier:** reference-calibrated camera frame over configured presentation. The `head` mode uses the pinned runtime XML `head_camera` frame; overview direction, follow offset/smoothing, projection FOV, and framing margins are configured browser presentation, not hardware calibration.
- **Units and frames:** source simulation positions remain metres and source Z-up; the renderer remains millimetres and viewport Y-up. `orbit` reports the world/context frame, `chase` reports the modeled robot-root frame, and `head` reports `head_camera`.
- **Not simulated:** hardware video, camera calibration/distortion/exposure, latency, rolling shutter, media transport, autonomous cinematography, and physical stabilization.

## Camera contract

| Compatible command value | Visible name | Learner purpose |
|---|---|---|
| `orbit` | Overview | Inspect the robot and ball together; drag to orbit after the deterministic context fit. |
| `chase` | Follow | Keep a stable, aspect-aware third-person view behind and above the articulated robot. |
| `head` | Head POV | Render the main viewport from the modeled source `head_camera` frame; explicitly not hardware video. |

- Remove the redundant automatic head-camera inset. The viewport label must always identify the current **main** view and its purpose.
- `Fit` reframes the current mode without changing it. Reset preserves the selected camera mode and reconstructs that mode's deterministic baseline framing.
- Resize recomputes meaningful mode-specific framing. Follow smoothing must be time-based; head POV must not be modified by orbit controls.
- Preserve `set_camera("orbit"|"chase"|"head")` in the command catalog, Python, and WebMCP. Enrich returned camera state with visible name, purpose, truthful frame, `inset: false`, and rendered-simulation transport.
- Focused keyboard `C` cycles Overview → Follow → Head POV → Overview.

## Acceptance and focused verification

- The visible camera controls and always-visible viewport label name and explain all three modes.
- Overview keeps both robot and ball inside the viewport with useful robot coverage; Follow keeps the complete robot inside bounded NDC at initial, moved, resized, Fit, and Reset states; Head POV position and forward direction agree with the configured source frame.
- Fit and Reset retain mode identity; resize retains readable framing; keyboard `C` follows the documented order.
- Camera switching does not advance held simulation time or change joints, targets, materials, colors, policies, contacts, or ball state.
- Capture and visually inspect all three modes at 1366×768 or larger. Run syntax, UI/core/static checks and only the focused camera/visual Chromium journeys; do not claim the full browser suite, hardware, deployment, or broad matrices.

