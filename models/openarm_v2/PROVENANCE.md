# OpenArm V2 contact and workcell package — revision 3

## Current implementation

The robot kinematics, source inertials, joint axes/ranges, mechanical finger coupling, damping/armature/friction loss and arm effort limits originate from `enactic/openarm_mujoco`, revision `a8c979629f2591ad035d99d338ce114969e6cddc`, `v2/openarm_bimanual.xml` (Git blob `0bd77d3bf7e0a5f3d2361fdf5e8328d00d0b2cc9`). The derived robot remains Apache-2.0; the repository-authored workcell and controller code retain their repository license. See `licenses/openarm-v2-apache-2.0.txt`.

`scripts/prepare_openarm_contact_model.py` replaces the old primitive robot proxies with a separate convex hull for each of 36 upstream collision components. Mirrored source scaling is baked into the vertices. MJCF and Three.js receive identical component vertices and triangles. The geometry manifest records source-file SHA-256 values and the model digest; `src/physics/openarm-generated.js` pins the emitted model and geometry digests. No runtime upstream download is required for OpenArm geometry.

The baseline file `source/primitive-baseline.xml` is a hash-verified historical generation input, not a selectable physical workspace. It must retain SHA-256 `960ecf32c0aa7c8b2b016c6f28a7a8afe8147ce6cb1cdfd9b91f550cd4fc27dc`. The earlier provenance below describes that historical revision and must not be read as current validation evidence.

## Declared changes and assumptions

- Nonadjacent same-arm collisions are enabled; standard MuJoCo connected-body filtering is retained.
- Source and destination X coordinates are now 0.55 and 0.67 m. The shoulder mounting origin remains [0.185, 0, 1.34] m. The tabletop top remains 1.005 m. Added legs, a mounting column/foot and the ring bracket have matching physical/render geometry.
- The 1.2 Nm gripper **operating** cap is an intentionally tighter simulation setting; it is not a source-rated hardware maximum. The source arm caps remain 40/27/7 Nm as applicable.
- Quintic reference speed bounds (0.75 rad/s arm, 0.60 rad/s finger), trajectory timings and the bias-assisted position-servo mapping are repository-designed, uncalibrated settings. Feedforward is delivered through the existing effort-limited actuators, not an external force or object state write.
- Vessel mass, inertia, friction and supports are benchmark assumptions. Flask and beaker are solid dry exterior envelopes, not hollow liquid containers. Grasp success is scoped to this declared task and parameterization.
- Observations follow `mj_forward` after every actual step and do not change with rendering cadence. Contact-force diagnostics come from MuJoCo, not tactile hardware sensors.
- Custom equipment is bounded declarative geometry, compiled into the same MuJoCo world only through a staged, explicit reset. The spring-button cap is a passive joint moved by contact; it is not commanded by an equipment-state setter.

Regenerate with Python, NumPy 2.3.5 and SciPy 1.17.0:

```sh
python scripts/prepare_openarm_contact_model.py --upstream-dir /path/to/pinned/openarm_mujoco/v2
```

Validation commands and supported operations are documented in `docs/physics/OPENARM_REFINEMENT.md`. Software/native-WASM verification is not hardware validation. No real robot connection, calibrated motor dynamics, collision-free trajectory guarantee, fluids, heating, pumping, chemistry or glass deformation is supplied.

---

# Historical revision 2 provenance (retained for lineage)

# OpenArm V2 Phase 5A model provenance

## Governing scope

This package is the Phase 5A OpenArm V2 physical-migration model for RoboBuddy IDE. It replaces the normal OpenArm task's legacy source-plant authority with one browser MuJoCo `PhysicsSession`. The existing task identity remains `openarm-04-filtration-workcell` / **Bimanual Heater and Ring-Stand Stack**.

## Source model

- Upstream repository: `enactic/openarm_mujoco`
- Pinned revision: `a8c979629f2591ad035d99d338ce114969e6cddc`
- Source robot file: `v2/openarm_bimanual.xml`
- Source robot Git blob: `0bd77d3bf7e0a5f3d2361fdf5e8328d00d0b2cc9`
- Source cell mounting reference: `v2/cell/cell.xml`
- Upstream release state at the pin: 2.2.0-era V2 model
- Upstream license: Apache-2.0; a copy is stored at `licenses/openarm-v2-apache-2.0.txt`.

## Source-derived parameters retained

The repository-local model retains the pinned V2 robot's mirrored left/right kinematic tree, joint axes and ranges, link inertial properties, joint damping, armature and friction loss, actuator position gains, actuator control ranges, and actuator force limits. It also retains only the source-defined inner/outer finger mechanical equality coupling for each gripper.

The source cell mounting relationship is preserved: the bimanual arm origin is located at `x=0.185 m`, `z=1.34 m` in the cell reference, with the source cell work surface at `z=1.005 m`. No legacy `base_yaw` pseudo-joint is added to the physical model.

## Browser physical-model adaptation

The upstream visual and collision meshes are not redistributed as MuJoCo collision assets in this Phase 5A package. Robot collision geometry is represented by explicitly declared primitive surrogates so the browser model remains self-contained. Those primitive collision shapes are an approximation and are not equivalent to the upstream mesh contact geometry. The reviewed fingertip capsule surrogate remains at its original 8 mm radius; task success is not obtained by enlarging invisible fingers. Instead, the controller maintains a bounded partial pinch while the vessel/receiver envelopes are aligned to the pinned task definition.

The dry vessel and receiving-surface envelopes are source-informed from the pinned RoboBuddy task definition `jivishov/RoboBuddy_AI@75fe2669c0ab0b029986de424c69162071174df8`: the Erlenmeyer collider follows the declared ~39 mm lower radius / 31 mm shoulder / 15 mm neck and ~114 mm height profile, the small beaker uses the declared 50 mm diameter x 60 mm height envelope, the hotplate top is 116 x 108 mm, and the gauze receiver is 96 mm across. The visible pickup staging supports, vessel mass/inertia, friction and contact material remain controlled benchmark estimates rather than measurements of installed hardware. The unpowered hotplate and ring-stand/gauze support model rigid contact only; no thermal, fluid, vacuum, payload-certification, glass-compliance, tactile, force-sensor, or hardware-safety behavior is claimed. The vessel colliders are validated only for this direct exterior pinch/lift/place task; they are not validated for insertion, pouring, filling, or liquid operations.

## Presentation model and physical authority

The browser presentation reuses the established RoboBuddy OpenArm V2 arm mesh from the pinned canonical visual package (`jivishov/RoboBuddy_AI@66d18a029a0caeb6a6075e681dbd9ecd6b22affa`) so the migrated workspace retains a recognizable OpenArm appearance. That visual package was generated from `enactic/openarm_description@6c7b720f1ba48e8bafa3a3dc752c45f397b42221` and historically also contained RoboBuddy-specific low-stand/turntable geometry and a simulator-only `base_yaw` concept.

For the physical workspace, those nonphysical presentation parts (`turntable_pedestal`, `turntable_bearing`, `turntable_disc`, `turntable_heading`, and `openarm_body_link0_low_stand`) are hidden. The arm mesh is translated onto the source-derived physical mount (`[185, 790, 0]` mm in the Three.js presentation frame), which places the canonical left/right arm mounts at `[185, 1340, -31]` and `[185, 1340, 31]` mm, matching the MuJoCo source mount after the documented basis conversion. The legacy `base_yaw` is fixed at its visual identity and is neither rendered as a turntable nor exposed as a physical/control degree of freedom.

Presentation joints are driven only from observed MuJoCo joint positions. Free vessels are rendered from observed MuJoCo body transforms. Rendering does not advance physics and cannot write back into the model.

## Authority and task semantics

- MuJoCo is the sole state authority during ordinary task execution.
- Robot motion is produced through position-actuator targets and fixed MuJoCo stepping.
- The flask and beaker are true free bodies.
- Ordinary task execution does not write free-body position/velocity, joint qpos/qvel, or object transforms.
- No task-dependent weld, parent, attach, snap, teleport, Cartesian move shortcut, or synthetic success event is used.
- The only equality constraints are the two source-defined gripper finger couplings.
- Rendering consumes MuJoCo observations and does not advance simulation.
- Live Python and bounded WebMCP route to the same `PhysicsSession` used by rendering and evaluation.

## Observation cadence

The evaluator is a consumer of authoritative observations, so the rate at which the browser
samples MuJoCo is part of the software-observation contract rather than an implementation
detail. The browser workspace declares a cadence of **2 physics steps (2 ms at the pinned
0.001 s timestep)** and exposes it in the presentation audit as `observationBatchSteps` and
`observationPeriodSeconds`.

The cadence is chosen from measured physical evidence, not from whichever value happened to
make a test pass. Evaluating the native reference at every 1 ms step shows that the narrowest
causal condition this task depends on — the beaker in intended wire-gauze support contact
while still bilaterally pinched — holds for only 3 ms and 4 ms in its two occurrences. A
2-step period is therefore guaranteed to land inside any window of two or more physics steps.
Coarser cadences were measured to miss it: a 20-step and a 50-step period both observe the
beaker reaching its destination while never observing the support-while-held predecessor, and
the evaluator correctly refuses to infer the unobserved causal transition.

Sampling runs inside the MuJoCo worker: one request advances every requested step and returns
the ordered ground-truth observations captured at the declared cadence. Sampling never
advances physics a second time, never fabricates a state, and never emits task events; each
returned entry is an ordinary observation of the actual MuJoCo state at that step. The command
budget is charged the actual executed step count once per request, derived from the
simulation-time delta.

This cadence is an implementation/observation-conformance parameter. It is not a sensor
sample rate of any assembled OpenArm and carries no hardware-calibration claim.

## Task evaluator evidence

Success is observation-derived and requires, independently for the flask and beaker: simultaneous bilateral finger contact; lift with bilateral contact; physically maintained horizontal transport; intended support contact while the vessel is still bilaterally held; an observed transition from gripper contact to release while intended support contact persists; post-release support-contact, target-region, low-velocity and low-drift settling for the declared simulation-time dwell; and actual post-settle end-effector displacement of at least the declared retreat distance while the vessel remains supported. The two transfers must occur in the required sequential shared-world order.

Historical contact on the two fingers at different times is not accepted as a grasp. A vessel dropped before support contact does not become a valid placement merely because it later lands in the target. Normal geometric distance between the end effector and a stationary vessel is not accepted as retreat evidence.

The evaluator thresholds are synthetic benchmark acceptance criteria, not hardware performance tolerances.

## Native comparison evidence

`native/openarm_v2_reference.py` uses the same model package and controller semantics and emits machine-readable model and controller hashes, MuJoCo version and solver settings, initial state, executed command sequence, relevant contact-pair history, sampled object trajectories, final state, and the task verdict. Nominal, negative-control, and tighter-timestep runs are implementation-comparison evidence only; browser/native agreement is not hardware validation.

## Hardware validation boundary

This Phase 5A model is **numerically verified simulator evidence**, not hardware validation. No installed-robot trajectory error, backlash, compliance, CAN timing, motor temperature, current limit, fingertip material, glass friction, payload capacity, emergency-stop behavior, or workcell registration has been measured. Hardware alignment remains calibration-required.
