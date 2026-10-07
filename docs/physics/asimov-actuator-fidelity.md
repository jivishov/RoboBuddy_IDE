# Asimov: source-informed actuator and standing experiment

Implemented against RoboBuddy baseline `11230f3e781e7196d42c51ec494429af8887224d`. Source audit date: 2026-09-11. This work is confined to `jivishov/RoboBuddy_IDE_v020`.

## Scope and entry points

The original Mounted Joint Lab, Free-base Dynamics and Passive Gravity Drop are unchanged reference plants. Three **separately selected** workspaces add a model experiment:

| Workspace | Model / capability |
|---|---|
| Actuator Lab (experimental) | Visible pelvis mount; continuous motor caps, estimated speed envelope, friction and delay; joint commands and synthetic observations |
| Actuator Free-base (experimental) | Same actuator experiment with free base; joint hold may fall |
| Standing Trial (experimental) | Same free-base plant plus an explicitly engaged, repository-designed torso-feedback controller and continuous support evaluator |

Select **Asimov 1 — Physical**, select one of the above workspaces, and run its included Python example. The standing example advances 12 simulation seconds, polling from Python once per simulation second. Feedback still runs at 200 Hz and standing assessment at every 2.5 ms physics step. Its explicit 120-second wall deadline accommodates slower clients; the default remains 30 seconds elsewhere. Deadlines are never automatically extended by progress, and human Stop remains available. Neither value equates elapsed wall time with simulation time. Full-resolution presentation redraws only when observations or the camera change; no physical step is omitted. Explicit Reset starts a new trial. The original default workspace remains Mounted Joint Lab.

No hardware connection, trained walking, grasping, calibrated digital twin, or general disturbance-recovery capability is introduced.

## Source reconciliation

The unchanged body model is pinned to Menlo `asimov-1` revision `732cc60dcb8f2b4fd26c3d7346b35f9b89c3cd47`. Its 23 movable joints, fixed neck, collision primitives, link masses and inertias, reflected inertia, contact exclusions and initial keyframes are retained. The new XML files change the model name and integration timestep only. Full-resolution source meshes remain unchanged.

Sources reviewed:

- Full-body mechanical documentation: <https://docs.menlo.ai/asimov/1/overview/system-tour/mechanical>, retrieved 2026-09-11.
- Training environment: <https://docs.menlo.ai/guides/locomotion-training/reinforcement-learning-simulation-training-environment>, retrieved 2026-09-11. Its timing/noise examples are legs-only, not measured full-body calibration.
- System identification: <https://docs.menlo.ai/guides/locomotion-training/reinforcement-learning-deep-dive-system-identification>.
- Older `asimov-mjlab` motor reference: <https://github.com/menloresearch/asimov-mjlab/blob/98870d12f079c0b6313bb0fe459aa591a5e7f251/motor_parameters.md>. Different reflected inertia and torque constants prevent treating this as a compatible full-body calibration. No code or motor settings were copied from that implementation.

`src/physics/asimov-actuator-profile.js` records transcribed scalar data, source pointers and decisions. Its 19 non-ankle motor assignments are matched by joint role and verified against the existing reflected inertia. For example, the source hip-pitch cap of 45 N·m becomes the lower 40 N·m continuous specification; the published 120 N·m peak is **not** enabled. The existing XML armature is not augmented with another motor inertia.

The four ankle axes remain explicitly source-equivalent. Published effective A/B rows differ from the underlying motor-family ratings. Numeric linkage ratios and a compatible motor-space model are not established. `asimov-transmission.js` implements the published small-angle sign convention, Jacobian velocity mapping, power-consistent torque transforms and reflected-inertia matrix, but **is not activated in the physical plant**. It requires explicit ratios; it has no guessed defaults. Round-trip, power and paired saturation tests prepare this boundary without claiming the actual transmission is modeled.

## Actuator law and uncertainty

Profile: `asimov-spec-informed-actuation-v1`. Motor control updates every 5 ms; the selected integration timestep is 2.5 ms. Native checks compare 5, 2.5 and 1.25 ms with the same 5 ms controller cadence.

For the 19 matched motors, the cap is the lower of source effort limit and published continuous rating. Estimated motoring torque falls linearly to zero at the chosen published/source speed boundary; braking retains the continuous cap. This is **not a fitted torque–speed curve** and the speed boundary is not a velocity clamp: external forces can still drive a joint beyond it. There is no arbitrary state correction. Peak duty/thermal operation is disabled because compatible time and temperature data are unavailable. Torque constants are recorded as source evidence but are not used to invent an electrical/thermal model.

A smooth Stribeck-like resisting torque interpolates published static/dynamic magnitudes. Its 0.1 rad/s transition and 0.02 rad/s smoothing are estimates; exact stiction, compliance and backlash are not modeled. Passive friction persists with motors disabled. The worker checks compiled damping/friction arrays to prevent counting the same dissipation twice. Only joint generalized forces receive this passive term; no force is injected at the floating root by the controller.

Commands pass through a fixed 5 ms delay queue before the local PD loop consumes them. Motor feedback remains local at 200 Hz; the synthetic high-level sensor delay is not incorrectly inserted into this inner loop. Requested commands, applied delayed targets, motor effort, passive friction and dynamic torque caps are reported separately. Configuration hashes accompany observations. Reset clears the delay queue, controller/evaluator state and sensor history. Pause/read/render operations do not advance any of those histories.

## Sensor view

Profile: `asimov-hardware-like-stress-v1`. This is an **uncalibrated sensitivity view**, not a faithful CAN/encoder emulator. It samples at 5 ms simulation-time intervals with deterministic seed 1729. Bounded uniform noise uses the cited training examples: ±0.01 rad position, ±0.1 rad/s velocity, ±0.01 rad/s gyro, and ±0.05 projected-gravity components.

The assumed source-order groups have 10/5/0 ms delay. These group assignments are not verified hardware polling order. Every joint carries source sample time, age and validity; unavailable cold-start history is invalid rather than silently replaced with fresh measurements. Repeated reads consume no random samples. Renderer and task evaluator continue to use authoritative ground truth.

Python can read `await robot.get_observation(view="hardware_like")`; the original scenes reject this unsupported view. The synthetic view contains joint and IMU data, **not** exact root position, body poses, contact forces or inferred measured motor torque. The simulator's existing ground-truth API remains available for debugging and evaluation. The standing controller currently uses ground-truth torso orientation/gyro, so passing its tests does not demonstrate robustness to the synthetic sensor profile.

## Limited standing capability

Controller `asimov-stance-feedback-v1` adds bounded pitch/roll ankle feedback to the existing nominal joint controller. Orientation proportional gain is 200 and angular-velocity gain is 20, distributed over both feet with the source joint signs. These are repository-designed gains, not Menlo hardware gains. Floating-base angular velocities are transformed consistently into the heading frame. All actual movement remains the result of joint torques, gravity and contact.

This is a stance regulator for the declared flat-floor initialization, not a whole-body walking or fall-recovery controller. Mounted scenes cannot engage it. It does not reposition the root, weld the robot, change its feet, reset after falling, or enlarge motor caps to make a trial succeed.

Acceptance is evaluated **every physics step**: after 1 s settling, at least 10 s continuously valid dwell; pelvis height at least 0.50 m; tilt no more than 0.12 rad; horizontal drift no more than 0.08 m; at least 20 N normal contact per foot; no other body-ground contact. A later failure invalidates an earlier pass. Motor disable or controller release immediately invalidates the trial. Re-engagement requires explicit reset.

Opt-in WebMCP adds `read_sensors` and `engage_stand` to the existing bounded Asimov tool. Commands retain captured session/epoch ownership. Neither command starts hardware or paid execution. Stop is a motor target operation, not a velocity reset; UI Pause freezes numerical time.

## Executed local checks and release gate

Local tests executed with Node 22.16 and native MuJoCo 3.11.0:

- Existing source Asimov WASM tests and existing physics/WebMCP core checks passed.
- New actuator/transmission/sensor/evaluator core checks passed.
- Shipped WASM ran the new mounted/free-base/standing scenes, including 12 s standing, delay reset, noise reproducibility, invalid command rejection, pause and actuation disable.
- Independent native implementation agreed with 160 WASM snapshots: 20 mounted, 20 free-base, 120 standing. Maximum joint/body differences were below 3.4e-16 rad/m; maximum motor-effort difference below 5.4e-14 N·m. This tests implementation agreement, not physical truth.
- Against the 1.25 ms run, the 2.5 ms stance run differed by at most 0.000550 rad joint position and 0.0000487 m body position at common 0.1 s sample times. Predeclared gates were 0.025 rad and 0.01 m, plus matching pass/fail. No pointwise impact-force convergence claim is made.
- Sixteen 12 s stance trials include timestep, friction/speed/delay variations, four 5 N horizontal force pulses lasting 0.2 s at the pelvis, and three negative controls. Positive trials passed; no-feedback hold, motors off and a 150 N pulse failed. The 5 N pulse is a **small simulation perturbation**, not evidence of robust recovery.
- A separate five-trial mounted elbow command sequence establishes nontrivial speed/delay sensitivity. Quiet standing alone poorly excites those parameters and cannot identify them. Sensitivity sweeps are not statistical confidence intervals.

The updated permanent `.github/workflows/asimov-physics.yml` reproduces the source generation, numerical checks and four Chromium journeys, including real Python and WebMCP integration. The local browser environment blocks page navigation, so **full browser acceptance remains a GitHub Actions gate**, not a claimed local pass. Exact tested commit and browser screenshots are retained in the CI artifact. Merge is contingent on reviewing that result and the existing fleet regressions.

Reproduction:

```sh
python scripts/generate_asimov_models.py --source /path/to/pinned/asimov-1
python scripts/generate_asimov_actuator_models.py
node tests/physics/asimov-actuator-core.mjs
ASIMOV_ACTUATOR_REPORT=/tmp/asimov-actuator-wasm.json node tests/physics/asimov-actuator-worker.mjs
python tests/validate_asimov_actuators.py --wasm-report /tmp/asimov-actuator-wasm.json --report /tmp/asimov-actuator-validation.json
npx playwright test tests/asimov-physical-browser.spec.mjs --workers=1
```

## What still needs evidence

A compatible numeric ankle mapping and motor limits are the next model dependency. Measured joint responses, high-rate timestamped commands/feedback, firmware/gain configuration and held-out trajectories are needed to fit and validate actuator parameters. Safe manufacturer-supervised measurements may be supplied by Menlo; ownership of hardware is not required to analyze them. No real-robot fall or disturbance test is requested here.


## Status update — 2026-09-12

The historical paragraph above records the implementation environment at the time. Its pending browser gate is now closed **for baseline `de97f7c99bcf2ef792c2ccaea3c4b704cf0eff9a`**: GitHub Actions run `34658975401` completed successfully on 2026-09-11, including the four dedicated Chromium journeys. This was CI evidence, not a local browser pass or hardware validation.

Important qualifications: the 19 matched non-ankle motors receive the added friction/speed laws; the old standing regulator acts through four ankles that do not. The original command-delay queue delays nominal targets, not its ground-truth balance feedback. The 32.224913 kg source model differs from the published nominal 35 kg; configuration and mass distribution remain unresolved.

The additive sensor-driven standing and hypothetical ankle-loss profiles are documented in `ASIMOV_WEBMCP_PROGRAMMING.md` and the reviewed `ASIMOV_SENSOR_WEBMCP_PLAN.md`. They do not change the six original workspaces. No guessed A/B transmission, increased peak allowance or rescaled production mass is introduced. New CI evidence must be read for the new tested commit, not inferred from the historical pass.
