# Unitree G1 29-DoF canonical rig

The IDE loads the Unitree G1 visual rig from the RoboBuddy canonical asset pinned to `jivishov/RoboBuddy_AI@66d18a029a0caeb6a6075e681dbd9ecd6b22affa`:

`simulator/js/robot-mesh-data-unitree-g1.js`

The reviewed local source asset has SHA-256:

`2d0623dc17ac1026678232d10378cadd310cc0a736b77e91cf00da9ecbad8dcb`

It contains 29 named revolute joints, 36 visual parts, and 36 unique mesh payloads. The source mesh declares Three.js Y-up geometry, positions in metres, transforms in millimetres, and a ground offset that the shared canonical loader applies.

## Upstream source and license

- Unitree description: [`unitreerobotics/unitree_ros`](https://github.com/unitreerobotics/unitree_ros) at `dd4fa6866e523ad61324f658d63736e4eda3a6e4`
- URDF: `robots/g1_description/g1_29dof.urdf`
- License: BSD-3-Clause; retained in [unitree_ros-BSD-3-Clause.txt](../licenses/unitree_ros-BSD-3-Clause.txt)

## Simulation boundary

The Unitree workspace is a browser-only articulated-pose viewer. It validates named joint angles against the source manifest and updates the canonical mesh hierarchy. It does not implement balance, walking, root translation, feet contact, collision, grasping, force/torque control, physical hardware control, or a Unitree SDK adapter. The fixed rubber hands have no articulated fingers.
