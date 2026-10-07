# Unitree G1 Phase 5D physical provenance

This document records the **physical MuJoCo Unitree G1 workspace**. It is a separate workspace from the retained kinematic pose workspace, which stays selectable, keeps its own identity and labels, and is never used as a fallback when the physical backend fails: a physical backend failure surfaces as a failure.

Everything here is implementation evidence. No measurement of an assembled Unitree G1 was used, and no hardware-validation claim is made anywhere in this workspace.

## 1. Robot identity

The workspace models exactly one machine: the **Unitree G1, 29 degrees of freedom, fixed rubber hands**.

It is not the 23-DoF variant, not a dexterous-hand variant, not a dual-arm variant, and not the `rev_1_0` revision. The fixed rubber hands are passive geometry with no actuated fingers; no grasp, dexterous-hand, force-sensor or tactile capability is modelled or claimed.

## 2. Pinned sources and their authority

| Tier | Source | Pin | Authority for |
|---|---|---|---|
| 1 | [`unitreerobotics/unitree_ros`](https://github.com/unitreerobotics/unitree_ros) | `dd4fa6866e523ad61324f658d63736e4eda3a6e4` | model identity, kinematics, inertials, joint ranges, effort/velocity limits, collision meshes, foot contact primitives |
| 2 | [`unitreerobotics/unitree_mujoco`](https://github.com/unitreerobotics/unitree_mujoco) | `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d` | joint armature, damping, friction loss, actuator control range, low-level motor command law |
| 3 | [`unitreerobotics/unitree_rl_mjlab`](https://github.com/unitreerobotics/unitree_rl_mjlab) | `1425b15f73bd4095f0df53709d7c389c3eb9e790` | FixStand standing posture, FixStand gain vectors, FSM cadence |
| 5 | `google-deepmind/mujoco_menagerie` | — | **not adopted.** Audited and excluded: it is a different model revision. Menagerie is never treated as hardware truth |

Vendored copies of the exact bytes these decisions rest on:

| File | SHA-256 |
|---|---|
| `models/unitree_g1/source/g1_29dof.xml` | `599a3f3e55f92543a404944cd0501f8fbcba57a57210d1a1e42ef45839768c84` |
| `models/unitree_g1/source/g1_29dof.urdf` | `ed3b86a04d190b2206c10b44dce685832bc20f72e176d8735aa2d18ffeae1cf0` |
| `models/unitree_g1/source/g1_29dof_unitree_mujoco.xml` | `423e28bd718b19f7a65cda539b6f794ddbb268b4b9bdbd85f4bd982b30729617` |

The Tier 2 model was compared field by field against the Tier 1 model before any of its dynamic parameters were adopted: all compared quantities agree to 0.0 and the referenced meshes are byte-identical. The augmentation is therefore same-model, not a cross-revision transplant.

## 3. Source reconciliation

`src/physics/unitree-g1-source-audit.js` carries the full reconciliation table, and `assertReconciliationCoverage()` fails the build if a required row disappears. Every row records what was adopted, from which pinned file, and under which evidence class. Summary:

| Parameter | Evidence | Notes |
|---|---|---|
| robot variant, joint count and order | source-derived | MJCF joint order is exactly the published Unitree 29-DoF DDS motor index order |
| joint axes, positions, ranges | source-derived | URDF and MJCF agree on all 29 axes and 23 of 29 ranges. The six wrist joints differ only by MJCF five-decimal rounding, at most `2.055e-6` rad. The compiled MJCF value is the authority; the URDF values are retained |
| effort limits (88 / 139 / 50 / 25 / 5 N·m) | source-derived | URDF `<limit effort>` and MJCF `actuatorfrcrange` |
| velocity limits (32 / 20 / 37 / 22 rad/s) | source-derived | URDF `<limit velocity>` |
| masses, inertias, total mass 35.112142 kg | source-derived | MJCF `<inertial>` of all 30 bodies |
| armature 0.01, damping 0.05, friction loss 0.2 | source-derived | Unitree's own MuJoCo simulator at the same model |
| motor control range | source-derived | `ctrlrange` = `actuatorfrcrange` |
| low-level command law | source-derived | `simulate/src/unitree_sdk2_bridge.h` |
| collision geometry | source-derived | convex hulls of the 25 source collision meshes (§4) |
| foot contact primitives | source-derived | four 5 mm spheres per `ankle_roll_link`, named individually |
| self-contact exclusions (4 pairs) | **estimated** | repository-authored |
| timestep 2 ms, Euler, 100/50 solver iterations | source-derived | MuJoCo defaults, which both pinned Unitree models leave unchanged |
| low-level control interval 2 ms | **estimated** | Unitree's FSM runs at 1 kHz; this workspace controls every physics step |
| standing posture, controller structure, waist/arm gains | source-derived | FixStand `qs`, `State_FixStand.h`, FixStand `kp`/`kd` |
| standing controller ankle gains (kp 250, kd 10) | **estimated** | repository-authored, §7 |
| hardware alignment | **calibration-required** | none exists |

## 4. Collision geometry

The source collision meshes are 10.17 MB of STL across 25 files, which is too large to ship to a browser. MuJoCo collides mesh geoms through their convex hull, so each source mesh is replaced by its own convex hull: 2.72 MB, a 73.3% reduction, and **collision-identical** to the source model rather than an approximation of it.

This was verified rather than assumed: a 6 s contact-rich standing trial run against the source meshes and against the hulls agrees to exactly 0.0 on root pose and on every joint.

`models/unitree_g1/source/MESH_MANIFEST.json` records, for each of the 25 meshes, the source SHA-256 and byte count, the hull SHA-256 and byte count, and the hull's vertex count, triangle count and volume. `scripts/prepare_unitree_g1_assets.py` verifies the committed hulls against that manifest offline, and can re-fetch and rebuild them from the pin.

Every collision geom is named. There are no unnamed collision geoms in any variant, so every contact can be reported by named geometry pair rather than as an anonymous count.

## 5. The four model variants

`scripts/generate_unitree_g1_models.py` deterministically generates all four MJCF files from the pinned source, and `--check` fails if a committed file differs by a byte.

| Variant | SHA-256 | Root | Bodies | Mass | Purpose |
|---|---|---|---|---|---|
| `mounted.xml` | `78b99a41889c15f677b723349f4f6f478725bc39a4efa5d7dcf37730d50d0a01` | fixed-mounted | 32 | 35.112142 kg | root-fixed dynamic fixture: joint response and known-pose validation |
| `mounted_blocked.xml` | `5f0af3108fc496fae98a4d1eb2c59d752423668363939a0eb020519d9cc31065` | fixed-mounted | 33 | 35.112142 kg | the same fixture plus a declared `hip_abduction_stop_wall` |
| `freebase.xml` | `3aad0869bf2a83d1f9acd84f5c627745a9a56dc1ab426be8a1f82463a6a76bf3` | free-base | 32 | 35.512142 kg | the interactive workspace: gravity, floor, feet, self-contact, and a declared 0.4 kg free contact-probe block |
| `freebase_drop.xml` | `e222c0b8bf12f51352373f31965752893f5cbb987dc28a172300fcaa25165ab5` | free-base | 31 | 35.112142 kg | the fall scene |

Every variant reports `neq = 0` and `ntendon = 0`: there is no equality constraint, weld, elastic band or tendon anywhere in this workspace. The free-base mass is 0.4 kg above the robot's own mass because the declared external block is part of that scene.

The mounted variants carry three additional contact exclusions the free-base variant does not need. MuJoCo skips its parent-child contact filter when the parent body is welded to the world, so without them the mounted pelvis and hip links graze each other at sub-millimetre distance and consume tens of newton-metres that the free-base model never spends. The exclusions restore free-base behaviour on the fixture rather than adding anything to it.

## 6. Actuation

Actuation follows Unitree's low-level semantics exactly:

```
tau = kp * (q_target - q) + kd * (dq_target - dq) + tau_ff
```

clamped to the joint's source effort limit. The MuJoCo actuators are plain torque motors, so `data.ctrl` is a torque, not a position. **No code path anywhere writes `qpos[joint] = target`.** The worker's only assignment into physical state is:

```js
data.ctrl[actuator.id] = actuationEnabled ? lowLevelTorqueNm(command, measured(index)) : 0;
```

Commands are bounded before they reach the plant: position to the source joint range, velocity to the source velocity limit, feedforward torque to the source effort limit, and `kp`/`kd` to 400 / 60. The observation reports the **requested** target, the **accepted** bounded command and the **measured** joint state as three separate values, so a target can never be read as a measurement.

`tests/validate_unitree_g1_controller_parity.py` checks that the JavaScript and the native Python implementations of this law agree exactly.

## 7. Standing, and the one declared deviation

A rigid inverted-pendulum approximation gives an ankle stiffness scale of `m·g·h`, here **231.3 N·m/rad**. Unitree's FixStand gains give 80 N·m/rad. This comparison is a gain-selection heuristic, not a necessary or sufficient stability proof for an articulated robot with contacts. The source-gain failure and engineering-controller success below are established by the actual physical trials, not by this calculation.

This is not a theoretical objection. Running the exact source FixStand gains on this model is a shipped trial: the robot topples in about 1.5 s, reaching 95° of tilt with seven non-foot ground contacts, at only 12 N·m of peak torque. It is retained as evidence, is reported under its own controller identity `unitree_g1_fixstand_source_v1`, and is deliberately not agent-reachable.

The shipped standing controller `robobuddy_g1_stand_v1` keeps FixStand's structure, posture, ramp and every waist and arm gain, and changes **four numbers**: the ankle pitch and roll gains, to kp 250 / kd 10.

- **kp 250** comes from the stiffness criterion above, with margin.
- **kd 10** is bounded from above by explicit-integration stability on the *unloaded* foot. With the foot's own inertia of about 0.0105 kg·m² the bound is `kd·Δt < 2I`, so `kd < ~10.5` at the nominal 2 ms step. Measured: kd 30 limit-cycles a free foot at 4 rad/s while kd 10 tracks it to 0.9 mrad, and both hold the stand identically.

Both are **repository-authored simulator gains**, recorded as `estimated`, and are never presented as Unitree hardware settings.

## 8. Measured standing evidence

Gate: evaluated from 2 s to 8 s after the controller engages. Pelvis height in [0.7342, 0.8342] m, tilt ≤ 0.15 rad, horizontal drift ≤ 0.10 m, terminal linear speed ≤ 0.05 m/s, terminal angular speed ≤ 0.20 rad/s, both feet supported throughout, no non-foot ground contact, no external or fixture contact, worst actuator effort ≤ 90% of its source limit.

| Trial | Result | Pelvis z (m) | Max tilt | Drift (m) | Non-foot contacts | Worst effort |
|---|---|---|---|---|---|---|
| nominal | **pass** | 0.78088 – 0.78148 | 4.44° | 0.023 | 0 | 13.4% |
| nominal, 1 ms step and 1 kHz control | **pass** | 0.78088 – 0.78148 | 4.44° | 0.023 | 0 | 13.4% |
| low floor friction (0.25) | **pass** | 0.78178 – 0.78210 | 3.63° | 0.016 | 0 | 13.4% |
| forward impulse 0.15 m/s | **pass** | 0.78037 – 0.78190 | 6.05° | 0.059 | 0 | 35.2% |
| forward impulse 0.20 m/s | **fail** | 0.07405 – 0.70625 | 98.01° | 0.939 | 13 | 100% |
| source FixStand gains | **fail** | 0.09065 – 0.09256 | 95.09° | 0.946 | 7 | 100% |
| motors disabled | **fail** | 0.05857 – 0.07318 | 103.30° | 0.265 | 13 | **0%** |

The last three are negative controls and are expected to fail. The motors-disabled trial reports exactly zero actuator effort, which is what makes it a real control rather than a differently-parameterised positive.

The perturbation pair brackets a boundary: the posture hold survives 0.15 m/s and is toppled by 0.20 m/s. That is a measured limit of a posture hold, not a perturbation-recovery capability. Perturbation recovery is unsupported and is not exposed.

Halving the timestep changes the evaluated pelvis height and tilt by exactly 0, and the drift by 3e-6 m.

## 9. Other measured physical evidence

- **Mounted joint response.** Bounded holds on the root-fixed fixture. Every commanded joint moves in the commanded direction, every measured value differs from its command (a gravity-loaded joint settles visibly short at the source hold gains), and no actuator exceeds its source effort limit.
- **Blocked joint.** The identical command is issued in two models. Unobstructed, `left_hip_roll_joint` reaches 1.056 rad. Behind the declared wall it stops at 0.137 rad, the actuator saturates at 88 of 88 N·m, and the obstruction is reported as the named pair `left_foot_toe_lateral ↔ hip_abduction_stop_wall`.
- **Root-fixed known poses.** Commanded poses on the mount leave the welded pelvis frame exactly where it was, so the mount really is root-fixed.
- **Command bounds.** An out-of-range request (6 rad, 500 rad/s, 10 kN·m, kp/kd 1e5) is clamped to the source range, the source velocity limit, the source effort limit and the 400 / 60 gain bound, and the measured state after it is read from MuJoCo rather than echoed.
- **Genuine fall.** From the drop scene with no actuation, the root descends 0.654 m, `uprightZ` changes by −1.81, and eight non-foot geoms reach the ground. The fall is produced by removing actuation through the single declared setup operation, never by writing root state.
- **Self-contact.** Both hip roll joints commanded to their source inner limits cross the shins, producing a `left_knee_link ↔ right_knee_link` contact carrying 52.5 N. This is a non-adjacent pair that MuJoCo's parent-child filter does not remove, so self-contact is demonstrably part of the plant.
- **External object.** A commanded single-leg reach topples the robot; its right knee strikes the declared free block, which moves 77.6 mm. The contact is reported as `contact_probe_block_geom ↔ right_knee_link_hull`. Nothing places, welds or transforms the object.

## 10. Browser and native conformance

The browser worker and `native/unitree_g1_reference.py` run the same pinned models, the same controller tables and the same bounded command law. `tests/unitree-g1-conformance-browser.spec.mjs` compares them, and fails if a configured native report is missing rather than silently skipping the comparison.

| Quantity | Browser | Native | Delta |
|---|---|---|---|
| standing pelvis height minimum | 0.78087899 m | 0.78087900 m | 7.3e-9 |
| standing maximum tilt | 0.07741754 rad | 0.07741400 rad | 3.5e-6 |
| standing maximum drift | 0.02294016 m | 0.02294000 m | 1.6e-7 |
| worst actuator effort fraction | 0.13350096 | 0.13365500 | 1.5e-4 |
| blocked joint, obstructed | 0.13741577 rad | 0.13741600 rad | 2.3e-7 |
| blocked joint, unobstructed | 1.05621102 rad | 1.05621100 rad | 1.9e-8 |
| free-fall height change | −0.654641 m | −0.654641 m | 3.2e-7 |
| external object displacement | 0.077633 m | 0.077633 m | < 1e-6 |
| self-contact pair | `left_knee_link ∥ right_knee_link` | identical | — |

Agreement here is implementation evidence only. Both backends carry the same model and the same approximations, so agreement says the implementations match, not that either matches a robot.

## 11. Display-rate independence and observation timing

The same command sequence over the same simulated time, run with different observation cadences and different rendering activity, lands in the same physical state: simulation time, pelvis height, tilt and worst joint all agree to exactly 0. Rendering never advances physics. 240 rendered frames advance the simulation clock by 0 and move the root by 0.

`mj_step` integrates `qpos`/`qvel` but leaves body poses and contacts at the pre-integration state. Both authorities therefore evaluate forward once before building an observation, so every published quantity comes from the same instant. Without it the reported body poses lagged the joint angles beside them by one timestep, which is invisible in a held stand and reaches 4.94 mm during a fall.

## 12. Presentation

The canonical Three.js rig is presentation only. It is driven from the observed MuJoCo pelvis transform and the 29 measured joint positions, and nothing it computes is ever fed back into physics. The rig's joint groups sit on the source joint pivots, which are the MuJoCo child body origins, so the two must coincide: measured, the worst disagreement across all 29 joints is **0.0006 mm** at rest, while standing, and throughout an unactuated collapse.

The physical simulator withdraws every claim it makes on the canvas when it is disposed, so selecting the kinematic pose workspace cannot inherit a stale free-base, standing, contact, physics-session authority or physical model-package attribute.

## 13. The two workspaces

| | Physical workspace | Kinematic pose workspace |
|---|---|---|
| Task id | `unitree-g1-physical-dynamics` | `unitree-g1-kinematic-pose-inspection` |
| Backend | browser MuJoCo, one `PhysicsSession` | source canonical rig |
| Capability label | `browser-mujoco · numerically-verified` | `legacy · model-derived` |
| Driver label | `robobuddy.sim.v1 · browser MuJoCo` | `RoboBuddy G1 pose rig (kinematic only)` |
| WebMCP tool | `control_unitree_g1_physical_simulation` | `control_unitree_g1_simulation` |
| Contact plant | yes | none |

The physical workspace is the profile default; the pose workspace is retained, selectable and unchanged. The two never share a tool name, and each tool is withdrawn while the other workspace is displayed, so an agent holding one name can never reach the other's command path. A pose write is not a physical command and is never offered as one.

## 14. Capability table

| Capability | Classification |
|---|---|
| kinematic pose inspection | source-derived, retained workspace |
| mounted joint dynamics | physical / verified |
| free-base dynamics | physical / verified |
| fall and contact | physical / verified |
| standing | physical / verified (bounded posture hold) |
| perturbation recovery | **unsupported** |
| walking | **unsupported** |
| dexterous hands | **unsupported** |
| hardware calibration | **unsupported** |

`assertCapabilityAudit()` fails the build if any of the last four stops being `unsupported`. There is no walk command in the model, the worker, the live Python surface, the WebMCP schema or the starter workspace, and a `walk` request is refused by schema validation rather than silently accepted.

## 15. What is not modelled

Motor bandwidth, gearbox friction, joint compliance, backlash, contact material, sensor latency, control-network timing and stability margin are all unmeasured. Contact parameters and the four repository-authored ankle gains are simulator estimates. The fixed rubber hands are passive geometry. No hardware comparison exists, and none is claimed.
