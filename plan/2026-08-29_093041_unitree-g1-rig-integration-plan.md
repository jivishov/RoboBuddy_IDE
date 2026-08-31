# Unitree G1 29-DoF Rig Integration Plan

## Scope and fidelity contract

- **Audience and scenario:** RoboBuddy IDE learners inspect and program bounded joint poses on the Unitree G1 29-DoF visual rig.
- **Learning objective:** connect named joint-angle commands to the articulated humanoid mesh while keeping the distinction between visual kinematics and robot control explicit.
- **Runtime and constraints:** static browser IDE, Three.js 0.180.0, current canonical-rig loader, laptop viewport support, and no new build dependency.
- **Fidelity tier:** reference-calibrated visual geometry and joint hierarchy. The mesh, axes, limits, frame conventions, and ground offset are tied to pinned source data; behavior is not a hardware-calibrated digital twin.
- **Canonical units and frames:** joint commands are degrees; mesh geometry is Three.js Y-up with vertices in meters and transforms in millimetres as declared by the source asset. The root uses the source `groundOffsetMm` to place the visual feet on the presentation ground.
- **Source provenance:** `jivishov/RoboBuddy_AI@66d18a029a0caeb6a6075e681dbd9ecd6b22affa`, `simulator/js/robot-mesh-data-unitree-g1.js`; mesh SHA-256 `2d0623dc17ac1026678232d10378cadd310cc0a736b77e91cf00da9ecbad8dcb`. Its source URDF is `unitreerobotics/unitree_ros@dd4fa6866e523ad61324f658d63736e4eda3a6e4`, `robots/g1_description/g1_29dof.urdf`, BSD-3-Clause.

## Authoritative behavior

- The canonical G1 mesh definition drives visual hierarchy, 29 named revolute axes, base quaternions, material keys, bounding box, and ground placement.
- The G1 workspace accepts only bounded, degree-valued named joint commands drawn from the source manifest.
- Its state is a browser-only kinematic pose state. The renderer derives directly from that state; no separate procedural G1 visual is allowed.
- Existing SO-101, LeKiwi, and OpenArm tasks remain source-plant simulation paths. G1 deliberately does not enter that plant because there is no reviewed G1 ScenarioV2 collision/contact implementation in the IDE's task patch.

## Explicitly not simulated

- dynamic balance, foot contact, gait generation, locomotion/root translation, collision avoidance, force/torque control, hand/finger actuation, grasping, payload behavior, controller/network timing, calibration transfer, and physical Unitree hardware execution.

## Acceptance checks

1. The selector exposes Unitree G1 and loads `unitree_g1_29dof` through the same canonical loader as the other robots.
2. A valid named G1 joint command visibly changes the corresponding rig joint; an out-of-range or unknown command is rejected before rendering.
3. G1 UI wording identifies the workspace as kinematic-only and does not claim a fixed-step contact plant.
4. Existing source-plant tasks still load and replay unchanged; static checks and browser smoke coverage pass.

## Verification result

All acceptance checks passed on 2026-08-29. The browser suite verified the G1's source-pinned `unitree_g1_29dof` loader, 29-joint/36-part asset shape, valid and rejected command envelopes, Step Action pose update, and all existing source-plant regressions. A 1366×768 inspection confirmed both the neutral mesh and upper-body pose remain fully visible after the G1-specific fit adjustment.
