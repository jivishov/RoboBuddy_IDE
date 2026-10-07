# Unitree G1 29-DoF workspaces

The Unitree G1 profile carries **two separate workspaces**. Selecting one never changes or stands in for the other, and a failure in the physical workspace is reported as a failure rather than falling back to the pose workspace.

| | Physical dynamics | Kinematic pose inspection |
|---|---|---|
| Task id | `unitree-g1-physical-dynamics` | `unitree-g1-kinematic-pose-inspection` |
| Backend | browser MuJoCo, one authoritative `PhysicsSession` | canonical pose rig |
| Capability | `browser-mujoco · numerically-verified` | `legacy · model-derived` |
| Contact plant | yes | none |

The physical workspace is the profile default. It is documented in full in [unitree-g1-provenance.md](unitree-g1-provenance.md), including its pinned sources, model variants, actuation law, standing evidence, negative controls and capability table.

This page documents the retained **kinematic pose workspace**.

## Canonical rig

The IDE loads the Unitree G1 visual rig from the RoboBuddy canonical asset pinned to `jivishov/RoboBuddy_AI@66d18a029a0caeb6a6075e681dbd9ecd6b22affa`:

`simulator/js/robot-mesh-data-unitree-g1.js`

The reviewed local source asset has SHA-256:

`2d0623dc17ac1026678232d10378cadd310cc0a736b77e91cf00da9ecbad8dcb`

It contains 29 named revolute joints, 36 visual parts, and 36 unique mesh payloads. The source mesh declares Three.js Y-up geometry, positions in metres, transforms in millimetres, and a ground offset that the shared canonical loader applies.

The same rig is reused as the presentation layer of the physical workspace, where it is driven from observed MuJoCo state and never drives the plant.

## Upstream source and license

- Unitree description: [`unitreerobotics/unitree_ros`](https://github.com/unitreerobotics/unitree_ros) at `dd4fa6866e523ad61324f658d63736e4eda3a6e4`
- URDF: `robots/g1_description/g1_29dof.urdf`
- License: BSD-3-Clause; retained in [unitree_ros-BSD-3-Clause.txt](../../licenses/unitree_ros-BSD-3-Clause.txt)

## Simulation boundary of the pose workspace

The pose workspace is a browser-only articulated-pose viewer. It validates named joint angles against the source manifest and updates the canonical mesh hierarchy. It does **not implement balance**, walking, root translation, feet contact, collision, grasping, force/torque control, physical hardware control, or a Unitree SDK adapter. The fixed rubber hands have no articulated fingers.

Its WebMCP tool is `control_unitree_g1_simulation`, which applies a bounded pose. That tool is withdrawn whenever the physical workspace is displayed, and the physical workspace's own tool, `control_unitree_g1_physical_simulation`, is withdrawn whenever this workspace is displayed. The two never share a name, so a pose write can never be reached through a physical badge.
