# Programming Asimov through WebMCP

Simulation only. This changes only `jivishov/RoboBuddy_IDE_v020`. No physical robot, remote hardware connection or paid simulation service is used.

## Workspaces

Select **Asimov 1 — Physical**. The normal task menu exposes six user-facing workspaces:

| Workspace ID | User-facing purpose | Important qualification |
|---|---|---|
| `asimov-actuator-mounted` | Fixed-base actuator/joint laboratory | Not standing evidence |
| `asimov-actuator-freebase` | **Whole-Body Dynamics**: primary free-base workspace for agent-generated coordinated motion | Not a trained or validated gait policy |
| `asimov-standing` | Ground-truth torso-feedback flat-floor standing trial | Repository-designed controller |
| `asimov-sensor-standing` | Sensor-driven standing | Synthetic body-frame gyro/projected gravity; 10 ms feedback delay |
| `asimov-sensor-standing-delay` | Delayed sensor standing sensitivity | Same sensors with 25 ms feedback delay |
| `asimov-sensor-standing-ankle-stress` | Ankle-loss standing sensitivity | Hypothetical ankle losses and reduced source-joint torque envelope |

The original `asimov-mounted`, `asimov-freebase` and `asimov-drop` scenes remain registered **internally** as reference/validation fixtures. They are intentionally absent from the normal task selector. They preserve the hash-verified source/reference lineage used to derive or validate the experimental actuator models; the passive drop remains a gravity/contact negative control.

The body sensor profile is `asimov-body-sensors-v2`; the regulator is `asimov-sensor-stance-v2`. Gyro and projected gravity share the `pelvis_link` frame. Sensor histories advance only on simulation-time sample ticks. Cold-start history is unavailable; stale/invalid observations after a bounded warmup latch a sensor fault. There is no ground-truth attitude fallback in this regulator. The inner joint PD loop still uses ideal local encoder feedback; this is not a fully calibrated hardware control stack. Projected gravity is synthetic attitude-derived data, not a raw accelerometer or a full IMU estimator.

## Enable agent control

Choose **Agent Assist** in the existing opt-in controls. The registered tool remains `control_asimov_physical_simulation`, with `schema_version: "robobuddy.asimov.physical.v1"`. The bounded program contract is `robobuddy.asimov.program.v1`; agent-generated whole-body trajectories use the additive `robobuddy.asimov.whole-body.v1` contract through the same tool.

Start with `inspect_capability`. It reports all 23 named joint ranges, the selected timestep/controller, unsupported operations, mass discrepancy, limits and evidence. Source effort limits in the joint list are not the instantaneous actuator caps; inspect `actuator_model` for the actual selected experiment.

| Command | Effect |
|---|---|
| `inspect_capability` | Read model/joint/programming metadata; starts nothing |
| `read_state` | Ground-truth simulated state and independent standing assessment |
| `read_sensors` | Synthetic measurements in actuator/sensor experiments |
| `set_joint_targets` | Bounded targets in radians; releases a standing controller |
| `engage_stand` | Engage the controller declared by the selected standing workspace; no time advance |
| `set_standing_targets` | Bounded waist/arm targets while keeping an already active nonfailed standing controller |
| `advance` | At most 2 simulation seconds per call |
| `run_sequence` | 1–32 prevalidated segments, at most 2 s each and 12 s total |
| `run_whole_body_motion` | Execute 1–32 agent-generated whole-body keyframes in an actuated free-base scene, at most 2 s each and 12 s total |
| `wait_for_joint` | Observe existing control until a measured position remains within tolerance, or timeout |
| `stop` | Bounded measured-position hold; not a velocity reset |
| `reset` | New initial condition and trial; not task success |

Waist/arm `set_standing_targets` remains deliberately separate from whole-body programming. The 12 leg joints cannot be overridden while preserving the existing standing regulator through that command. `run_whole_body_motion` instead executes a complete, bounded full-body target trajectory against an actuated free-base MuJoCo plant. It does not retain the existing standing trial. Motion and failure arise from joint actuation, gravity and contact. The internal Passive Gravity Drop fixture remains observation-only and rejects whole-body motion before issuing any target.

## Agent-generated whole-body movement

`run_whole_body_motion` is intended for motions such as stepping, walking attempts, turning, squatting, reaching, gesturing and other coordinated full-body sequences. The `motion_intent` value is descriptive only. Labeling a trajectory `walk` does not make it a successful walk.

Each keyframe contains a duration and one or more named joint targets in radians. Missing joints retain the previous trajectory target. Before the first plant mutation, the complete request is checked for schema validity, source joint ranges, total duration, timestep alignment and source velocity-limit feasibility. The executor then linearly interpolates all 23 joint targets at 20 Hz and sends them through the existing bounded joint controller. The final issued target stream is rate-limited against the source joint velocity limits as well, so optional stabilization cannot bypass the trajectory rate bound. The root pose, root velocity and external forces are never commanded.

Two stabilization modes are available:

- `none` (**default**): execute the agent's full-body target trajectory without an added whole-body supervisor.
- `ground_truth`: apply a small, bounded 20 Hz ankle-target correction from measured pelvis roll/pitch and angular velocity. This must be explicitly requested. It is simulator-only supervisory feedback, not the synthetic hardware-like sensor stack, a trained locomotion policy, or a hardware controller.

The supervisor is intentionally limited to ankle target offsets and remains inside source joint ranges and target-rate limits. It cannot set the root upright, cancel gravity, teleport the robot, create foot contact or reset a failed run. Unless `abort_on_instability` is set to `false`, execution stops when the pelvis becomes too low, tilt becomes excessive, actuation is disabled, or a non-foot body reaches the ground.

The result reports measured root displacement, yaw change, maximum tilt, minimum pelvis height, observed support state at each keyframe, support-state transitions and any final-command target-rate limiting events. A returned `executionStatus: "completed"` means all requested keyframes executed. It does **not** mean a requested step, walk or turn succeeded. Agents should inspect those measured outcomes and revise their next trajectory if needed.

Example exploratory full-body sequence in **Whole-Body Dynamics** (`asimov-actuator-freebase`):

```json
{
  "schema_version": "robobuddy.asimov.physical.v1",
  "command": "run_whole_body_motion",
  "motion_intent": "step",
  "stabilization": "ground_truth",
  "abort_on_instability": true,
  "keyframes": [
    {
      "phase": "load and bend",
      "duration_seconds": 0.5,
      "support": "double",
      "targets_rad": {
        "left_knee_joint": 0.10,
        "right_knee_joint": -0.10
      }
    },
    {
      "phase": "agent-selected transfer",
      "duration_seconds": 0.5,
      "support": "any",
      "targets_rad": {
        "left_hip_pitch_joint": -0.08,
        "right_hip_pitch_joint": 0.05,
        "left_knee_joint": 0.16,
        "right_knee_joint": -0.08
      }
    }
  ]
}
```

This example explicitly opts into the simulator-only stabilizer to demonstrate that option. Omitting `stabilization` uses `none`. The example demonstrates the programming surface, not a validated gait. A model should use the returned measured state/contact evidence to determine whether the attempted motion produced useful translation or a stable support transition before extending it into additional steps.

## Example: stand, then move the elbows without releasing balance

Select `asimov-sensor-standing`, reset explicitly, then send:

```json
{"schema_version":"robobuddy.asimov.physical.v1","command":"engage_stand"}
```

```json
{
  "schema_version": "robobuddy.asimov.physical.v1",
  "command": "run_sequence",
  "segments": [
    {"duration_seconds": 1.0},
    {"targets_rad": {"left_elbow_joint": 1.0, "right_elbow_joint": -1.0}, "preserve_standing": true, "duration_seconds": 0.5},
    {"targets_rad": {"left_elbow_joint": 0.9, "right_elbow_joint": -0.9}, "preserve_standing": true, "duration_seconds": 0.5}
  ]
}
```

A completed sequence means the commands/time segments executed—not that standing passed, a pose was achieved, or a physical robot could do the same. This two-second example is shorter than the 11 seconds needed for the standing gate. Continue with bounded advances and inspect the assessment. Use duration-only segments to leave all targets/controllers unchanged.

## Example: measured-position wait in Actuator Lab

Select `asimov-actuator-mounted` and send:

```json
{"schema_version":"robobuddy.asimov.physical.v1","command":"set_joint_targets","targets_rad":{"left_elbow_joint":1.0},"max_steps":4000}
```

```json
{
  "schema_version": "robobuddy.asimov.physical.v1",
  "command": "wait_for_joint",
  "joint_id": "left_elbow_joint",
  "target_rad": 1.0,
  "tolerance_rad": 0.03,
  "dwell_seconds": 0.1,
  "timeout_seconds": 2.0
}
```

The wait does not send a target. It uses ground-truth joint position, checked every physics step, and returns `achieved` or `timeout`. A successful API response can therefore contain `achieved: false`. The target must be feasible under the actual plant. Closed-loop agents may also read synthetic sensors between bounded operations and choose their next commands. Existing editor/program-run WebMCP functions remain available for async Python programs; no separate unbounded code interpreter was added to this tool.

## Bounds and cancellation

Inputs and the complete sequence or whole-body trajectory are validated before any mutation, including joint ranges and timestep alignment. Whole-body programs reject keyframe target-rate requests that exceed the source joint velocity limits; the interpolated final target stream is independently rate-limited as well. Durations use the selected scene timestep: the internal reference fixtures use 5 ms integration while actuator/sensor scenes use 2.5 ms. A single call has a fixed 120 s wall deadline, never extended by progress. Physics advancement checks cancellation, access, workspace and session ownership between at-most-50-ms simulation batches. An already submitted batch cannot be undone. The normal UI Stop remains available; a second concurrent robot-control call is rejected.

No command sets root pose/velocity, applies external force, changes model mass/contact properties, silently resets after failure, creates synthetic foot contact, or connects to hardware. Agent-generated stepping/walking is expressed only as bounded joint trajectories; whether locomotion occurs is decided by the physical plant. The internal Passive Gravity Drop negative control is excluded rather than silently converted into an actuated scene. Model sensitivity variations are explicit test-runner cases or separately selected immutable user workspaces, not hidden changes from an agent.

## Verification and evidence

Run the original suites plus:

```sh
node tests/physics/asimov-sensor-core.mjs
node tests/physics/asimov-program-core.mjs
node tests/physics/asimov-whole-body-core.mjs
ASIMOV_SENSOR_REPORT=/tmp/asimov-sensor-wasm.json node tests/physics/asimov-sensor-worker.mjs
python tests/validate_asimov_sensors.py --wasm-report /tmp/asimov-sensor-wasm.json --report /tmp/asimov-sensor-validation.json
npx playwright test tests/asimov-physical-browser.spec.mjs tests/asimov-whole-body-browser.spec.mjs --workers=1
```

The native test independently reimplements the sensor/controller and compares with actual WASM traces. Additional cases vary seed, feedback delay, ankle losses, mass, COM, floor/foot friction and contact softness. Uniform mass/inertia scaling to 35 kg is explicitly a hypothetical test, never a correction to the production model. Combined cases and negatives are included. Reports preserve failures rather than tuning every case to pass. CI artifacts are retained 90 days; a compact report should accompany each accepted release.

The browser/catalog gate additionally verifies that only the six user-facing Asimov tasks appear in the normal selector while `asimov-mounted`, `asimov-freebase` and `asimov-drop` remain internally registered for provenance/regression validation.

## Evidence-dependent work deliberately not invented

Actual paired ankle ratios, motor-space limits and consistent reflected inertia still require compatible numeric evidence. So do fitted torque–speed/braking curves, breakaway friction, electrical/thermal peak limits, true encoder/CAN timing and hardware validation. The independent-axis ankle stress profile tests dependence on assumptions but does not solve paired-motor feasibility.

The whole-body WebMCP surface enables an agent to generate and iteratively test physical full-body trajectories, including stepping/walking attempts. It does not supply or claim a trained gait policy, a validated stable walking controller, hardware-feasible locomotion, grasping or general recovery. Those stronger claims remain evidence-dependent.
