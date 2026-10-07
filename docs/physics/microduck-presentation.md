# MicroDuck physical presentation and task retirement

## Corrected frames

The source DUCK v1 hierarchy stores the initial trunk world transform, including 0.12 m height. The physical renderer must replace that transform, not add it to the observed trunk pose. The model-to-view basis is (x, y, z) -> (x, z, -y), with metres scaled to millimetres. Zero quaternion components, including w=0 in a half-turn, are valid.

The model descriptors publish every one of the 15 source robot bodies. The renderer consumes their MuJoCo xpos/xquat world-pose snapshot and derives parent-relative presentation transforms. It does not recompute joint FK from qpos. MuJoCo's solved kinematic/contact fields after mj_step describe its force-evaluation stage, whereas qpos has already been integrated; those stages must not be mixed. Rendering performs no new mj_forward or mj_step and does not change controller/sensor timing, BAM forces, contacts, or any state. See https://mujoco.readthedocs.io/en/stable/computation/index.html#consistency-in-mjdata.

A missing, nonfinite, or degenerate body pose rejects the whole visual update before mutation. The source visual mesh bytes, inertials, collision meshes and physics thresholds are unchanged. The official soles remain; the local cosmetic passive-roller assemblies are not constructed in physical workspaces. The source reset has a small genuine sole clearance; ordinary gravity/contact closes it when simulation is advanced. No floor snap, hidden settle, or synthetic support is introduced.

## Supported learner tasks

Physical MicroDuck Locomotion is the default. Physical MicroDuck Ground Contact and Physical MicroDuck Ball Kick retain their task-specific plants. The articulated policy demonstrator is removed from the selector, scenario resolver and SimulatorHost routing. A saved old task selection migrates to the physical default at UI load; a direct request for the retired scenario is rejected. Its optional roller/audio/peripheral features are not advertised as physical features.

## Regression scope

The presentation browser suite checks all 15 rendered body positions/orientations against MuJoCo at reset, stationary contact and walking, at unchanged 1 micrometre / 10 microradian tolerances. It checks all three tasks, the absence of cosmetic rollers, all 58 official parts, sole clearance, real named floor contact, non-snapping half-turns/tilts, and atomic failure on invalid snapshots. The existing native BAM and contact-based physical gates are retained unchanged. Legacy-demonstrator-only UI browser suites are removed because that product surface is retired, not skipped to disguise a physical failure; their independent ONNX CPU/browser parity check remains active alongside physical Python, WebMCP and cancellation suites. Low-level legacy module unit tests are retained as isolation/provenance tests while shared assets remain in use.

This is simulator/presentation verification, not assembled-hardware calibration.
