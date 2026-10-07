# MicroDuck Phase 5C physical provenance

This document records the **physical MuJoCo MicroDuck workspace** after the full-plant fidelity migration. It is separate from the legacy policy demonstrator, which is explicitly selectable and is never used as a fallback when the physical backend fails.

## 1. Authority and pinned sources

| Role | Source | Pin |
|---|---|---|
| Deployed controller/runtime and policy bytes | `pollen-robotics/microduck` | `590b986bd8c0d50ae02cb3ea2f59c463b6828168` |
| Physical/RL environment and task collision models | `pollen-robotics/microduck_rl` | `519142b1f5bf59fdfd44d06c205119e7fff8e3cb` |
| BAM actuator equations/identified M6 parameters | `Rhoban/bam` | v1.0.1, `ab81512c44f1f709b99ef332addb5e51568cd51c` |

The deployed source is authoritative for the 61-value observation layout, 14 policy joints, home pose, command encoding, raw previous-action feedback, action scaling/filtering, skill priority, standing threshold, 50 Hz controller cadence, firmware-gain schedule and deployed IMU preprocessing. The RL source is authoritative for the physical robot model, task-specific collision plants, reset pose, ball prop, training-domain randomisation and MicroDuck BAM integration. BAM v1.0.1 is the actuator-equation authority.

## 2. Task-specific collision plants

MicroDuck does **not** use one universal collision approximation. Four registered packages preserve the task distinction in the pinned environment:

| Package key | Model | Physical use |
|---|---|---|
| `walk` | `models/microduck/walk.xml` | normal locomotion; reduced upstream `robot_walk.xml` collision set |
| `lowTraction` | `models/microduck/walk_lowtraction.xml` | declared low-friction locomotion negative-control fixture |
| `groundContact` | `models/microduck/groundcontact.xml` | stand, recovery, sit/stand, ground-pick and conservative roulade mapping; upstream `robot_allcollisions.xml` |
| `kick` | `models/microduck/kick.xml` | all-collision robot plus the source `ball.xml` prop |

A capability requested on the wrong collision plant is rejected with `wrong-plant` before it reaches the physical authority. Roller and roller-crouch have no matched plant at this pin and remain unsupported in physical mode.

The pinned `microduck_rl` revision does not contain the roulade task configuration even though the deployed runtime contains `roulade.onnx`. Roulade is therefore mapped conservatively to the broad all-collision plant and remains **physical / experimental**, not an exact pinned training-task match.

## 3. Collision geometry provenance

The physical packages use the **exact STL collision mesh bytes** from the pinned `microduck_rl` checkout. They are not fitted boxes, ellipsoids or other repository-authored primitive replacements. Source MJCF is retained under `models/microduck/source/`; exact STL bytes under `models/microduck/assets/`; `models/microduck/source/GIT_BLOB_SHA1SUMS` records each imported mesh's Git blob identity. CI compares committed XML/STL bytes against a fresh checkout of the exact pin and checks deterministic regeneration of the task wrappers.

`walk_lowtraction.xml` is intentionally different: sole/floor sliding friction is reduced to 0.02 as an adverse-condition fixture. It is not represented as a measured physical material.

## 4. BAM actuator plant

The source XML position actuator (`kp=0.55`, `kv=0`, `forcerange=±0.96 N·m`) is retained only as fallback/source-model evidence. It is **not** the Phase 5C physical actuator authority.

Browser and native execution use the BAM v1.0.1 XL330/M6 voltage-domain torque model. Requested joint targets stay separate from MuJoCo `ctrl`; `ctrl` is motor torque. BAM supplies voltage/back-EMF behavior, identified armature, directional/Stribeck/load-dependent friction, viscous friction and load-dependent voltage sag. JavaScript equations are independently cross-checked against `better-actuator-models==1.0.1` in CI.

The deployed firmware gain remains part of the physical law: 200 while running and 160 for standing/stand-tuned behavior. Torque-off removes electromagnetic motor torque without overwriting the requested target.

## 5. Deployment-reference observation/control path

Physics timestep is 5 ms and the policy/control interval is 20 ms (50 Hz). The policy receives the deployed 61-value contract: `3 gyro + 3 projected gravity + 14 (q-home) + 14 qdot + 14 previous raw action + 13 command`.

The mouth is excluded from the 14 policy joints. Previous action is the **raw policy output before scaling**. The standing threshold is inclusive (`<= 0.05`). Projected gravity is normalized, then the interactive physical workspace applies the deployed per-axis median-of-three preprocessing to gyro and projected gravity.

## 6. Training-reference and deployment-reference remain separate

The encoded training-reference preserves the pinned training distributions: firmware gain 200; battery voltage 6.5–8.2 V; voltage-sag coefficient 0–0.2 V/Nm with 6.0 V minimum; target delay 3–6 **physics steps**; friction and armature scaling 0.9–1.1; mass/inertia scaling 0.95–1.05; encoder bias ±0.015 rad; and IMU mount randomisation ±6°.

The interactive deployment-reference uses no synthetic training delay or stochastic domain randomisation. Its deterministic 7.4 V and 0.1 V/Nm sag settings are a source-backed CPU regression/reference condition for reproducible rehearsal. They are **not measurements of an assembled robot battery or internal resistance**.

## 7. Physical-authority invariants

MuJoCo is the single state authority. Outside explicit logged setup/reset operations, the physical path does not write root pose, root velocity, joint state or object state directly. There is no synthetic kick impulse, boundary clamp, teleport, hidden weld/grasp, scripted recovery or success-state overwrite.

Locomotion evidence requires real foot contacts. A kick can be credited only after a named kicking-foot/ball contact. Recovery is judged from simulated pose/contact/actuator evidence after a declared initial orientation; reset is never recovery. Live-Python and WebMCP expose bounded commands and skills, not arbitrary joint/root state writes.

## 8. Verification and evidence limits

Permanent gates cover deterministic model regeneration, asset hashes, exact pinned source XML/STL identity, controller/observation compatibility, collision-plant routing, BAM equation conformance, native MuJoCo/BAM positive and negative controls, timestep sensitivity, fleet regressions and a browser physical journey.

This is **software/model fidelity**, not assembled-hardware calibration. No repository evidence establishes exact physical walking speed, recovery probability, battery/internal resistance, real bus latency, thermal behavior, individual servo variation, wear or real contact-material coefficients for a particular MicroDuck. Those remain hardware-validation items.

## 9. Licensing

RoboBuddy original software remains MIT-licensed under `LICENSES/MIT.txt`. Applicable upstream material retains its upstream terms. The exact MicroDuck 3D model/collision assets are outside the RoboBuddy MIT grant; upstream currently declares its 3D model files **Creative Commons BY-SA-NC** and does not state a Creative Commons version. This repository does not invent one. See `LICENSE` and `THIRD_PARTY_NOTICES.md` for component scope.
