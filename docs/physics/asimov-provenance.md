# Asimov 1 physical integration

Source: `menloresearch/asimov-1@732cc60dcb8f2b4fd26c3d7346b35f9b89c3cd47`.
Entry point: choose **Asimov 1 — Physical** in the robot selector. The normal task catalog exposes six experiment/sensitivity workspaces: **Actuator Lab (experimental)**, **Whole-Body Dynamics (experimental)**, **Standing Trial (experimental)**, **Sensor Standing (experimental)**, **Delayed Sensor Standing (sensitivity)** and **Ankle Loss Standing (sensitivity)**. Actuator Lab is the default. The original Mounted Joint Lab, Free-base Dynamics and Passive Gravity Drop remain registered internally as reference/validation fixtures and are deliberately not listed in the normal task menu.
The original RoboBuddy_IDE and RoboBuddy_AI repositories are not modified.

## What is represented

The official MJCF defines 23 hinge joints, a floating pelvis, 26 robot bodies,
25 unique STL files, fixed neck yaw/pitch bodies, and 32.22491296549718 kg of
model mass. The hardware's advertised 25 joints and approximate 35 kg are not
substituted for those source values. Both elbows retain their source reference
offsets. Joint control uses the exact source names, axes and radians.

`generate_asimov_models.py` pins the input MJCF and URDF by Git blob hash. It
retains all source inertials, joint kinematics, armature/damping, contact
primitives, exclusions, sensors, floor friction and solver settings. The timestep
is 0.005 s, implicitfast/Newton, 10 iterations, 20 line-search iterations, with
source noslip/impratio/cone parameters unchanged. The eight original foot spheres
remain. Zero-density/non-colliding visual geoms are removed from the physics XML;
all 25 full-resolution STL files are losslessly gzip-compressed for presentation.
Compressed and decompressed hashes are checked before rendering. No mesh is simplified.

The source defines no actuators. This integration adds one **ideal joint torque
motor** per hinge, bounded in command and effort by the URDF limits. PD gains are
**repository estimates**, not Menlo's training configuration and not measured
motor parameters. Ankle pitch/roll remain the source's ideal coordinates, not the
hardware parallel ankle transmission. The wrist-end meshes have no finger actuators;
no grasping capability is claimed. The two neck bodies stay fixed.

## Authority and controls

One PhysicsSession and the fleet's BrowserMuJoCoBackend own each active workspace.
The Asimov worker uses the existing pinned MuJoCo 3.11.0 WASM. Ordinary control
writes only `data.ctrl`; it never writes root/joint position or velocity. Explicit
reset calls the model's named initial keyframe. An explicit, logged torque-off
setup is available internally/Python but not to the agent control tool.

The renderer consumes all observed body poses in metres/Z-up, with a single
presentation-only conversion to Three.js Y-up. It does not integrate, ground-snap,
set joint state, advance simulation, or claim a requested target was achieved.
Python uses the bounded `robobuddy.sim` bridge. WebMCP
`control_asimov_physical_simulation` remains opt-in with Assist access,
workspace/epoch ownership, cancellation and bounded steps/time. It can inspect
capability/state, issue bounded joint/standing targets, run bounded sequences,
read supported synthetic sensors, and execute bounded agent-generated whole-body
keyframes in actuated free-base scenes. Whole-body execution never writes root pose
or velocity, applies an external locomotion force, creates contact, or silently
resets a failed run. A `walk` intent label is not treated as gait success.

## Internal reference fixtures and user experiments

The three source/reference fixtures are retained because they form part of the provenance and negative-control layer:

- **Mounted reference:** pelvis fixed by the model at 1.05 m; a visible, non-contact mount documents the fixture. The source-adjacent pelvis pairs are excluded to preserve the same adjacent-body collision filtering when MuJoCo welds the root to world. This isolates joint dynamics and presentation alignment; it is not standing evidence.
- **Free-base reference:** source pelvis height 0.630 m and source reference joint positions; estimated bounded PD attempts to hold these positions. It can genuinely fall and is the source-derived parent for the actuator free-base/standing experiments.
- **Passive-drop reference:** initial pelvis height 0.95 m with zero commanded torque. The fall and subsequent contact are physical; reset is not a recovery action. It remains a gravity/contact negative control and rejects whole-body agent motion.

These fixtures remain addressable by internal validation code but are omitted from `ASIMOV_TASKS`, the normal user catalog. This UI cleanup does not delete the model bytes or alter the derivation chain.

The user-facing actuator models are generated from hash-verified mounted/free-base reference XML rather than independently rebuilt from a second plant. **Actuator Lab** uses the fixed-base experimental actuator profile. **Whole-Body Dynamics** uses the corresponding actuated free-base plant and is the primary WebMCP whole-body workspace. Standing and sensor-standing workspaces layer explicit controllers/sensitivity assumptions over the same lineage.

No trained walking policy, verified general balance recovery, hardware connection,
real actuator calibration, backlash, compliance, motor current/thermal dynamics,
or physical robot comparison is supplied. Agent-generated whole-body trajectories
may produce stepping or translation in simulation, but successful execution does
not establish a stable gait or hardware-feasible locomotion.

## Validation

Run from the repository root:

```sh
ASIMOV_WASM_REPORT=/tmp/asimov-wasm.json node tests/physics/asimov-worker.mjs
python tests/validate_asimov_native.py --wasm-report /tmp/asimov-wasm.json
npx playwright test tests/asimov-physical-browser.spec.mjs --workers=1
```

The WASM gate retains all three reference variants, joint tracking, torque caps,
actual contact forces, passive fall, exact reset, pause and invalid requests
(including numeric strings, unknown joints, NaN/Infinity, unknown command fields
and unsupported controller transitions). Malformed commands must preserve the
previous command. Corrupted XML must fail loading and leave no live plant.

Native conformance uses MuJoCo 3.11.0, the same 4-step observation cadence, and
compares every joint, observed body and contact normal force. With `--upstream`,
the full original STL-bearing source is compiled and compared against the stripped
physics model for all body inertias/transforms, joints, armature and damping.
The browser suite additionally verifies that the normal selector contains only the
six user-facing tasks while the three reference fixtures remain registered for
internal direct validation. It also covers session/Python bridge ownership,
rendered source mesh/body alignment, no frame-driven stepping, opt-in agent
controls and switching back to another physical robot workspace.

Local native/shipped-WASM validation on 2026-09-11 passed 208 sample comparisons.
Worst joint difference was 5.14e-16 rad, body difference 2.23e-16 m, contact force
difference 4.46e-12 N. Source compiled physical arrays matched within 1e-12.
These are **numerical checks**, not hardware validation. Later actuator/sensor/
whole-body changes carry their own CI evidence and do not convert these numerical
checks into physical-hardware evidence.

## Source licensing

Preserved verbatim under `models/asimov/source/`:
`HARDWARE-LICENSE.txt` (CERN-OHL-S-2.0), `SOFTWARE-LICENSE.txt` (GPL-2.0), original
MJCF, original URDF and generated source audit. Model/mesh derivations are not
relicensed as MIT. Original RoboBuddy glue/fixtures remain separately scoped by
the repository license. Upstream source and corresponding changes are publicly
available in this repository and in the pinned upstream repository.

## Subsequent actuator, sensor and whole-body experiments

The three source-reference fixtures remain unchanged and internal. User-facing
Actuator Lab, Whole-Body Dynamics and Standing Trial introduce the uncalibrated
profile documented in [asimov-actuator-fidelity.md](asimov-actuator-fidelity.md).
Sensor-standing and sensitivity workspaces add the explicitly documented synthetic
sensor/delay/ankle-loss assumptions. Bounded agent-generated whole-body trajectories
are documented in [ASIMOV_WEBMCP_PROGRAMMING.md](ASIMOV_WEBMCP_PROGRAMMING.md).
None of these experiments supersedes source-reference evidence or establishes
hardware equivalence. In particular, ankle motor-space actuation is still gated
on compatible numeric transmission data.
