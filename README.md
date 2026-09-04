# RoboBuddy IDE

[Open the live demo](https://jivishov.github.io/RoboBuddy_IDE/)

RoboBuddy IDE is a browser-based robotics learning environment where people write Python and watch modeled robots carry out the program in a live 3D scene. It brings an IDE-style workspace, visible simulation controls, and optional WebMCP collaboration into one approachable experience.

## About

RoboBuddy IDE makes robot programming concrete: the source editor, diagnostics, task controls, and 3D scene remain visible together. Learners can explore robot behavior step by step, while builders can use a focused, opt-in agent workflow without giving the agent hidden access to the application.

**Repository description:** Browser-based robotics IDE for Python, live 3D simulation, and bounded WebMCP collaboration.

## What you can explore

- Write and inspect Python across a small workspace containing the main program, trajectories, robot configuration, and workcell.
- Run, pause, step, run to cursor, stop, and reset simulations from the visible control bar.
- Explore modeled SO-101, OpenArm, LeKiwi, Unitree G1, and MicroDuck workspaces.
- Watch modeled robot state, diagnostics, contacts, and task progress in the 3D simulator.
- In a WebMCP-capable browser, turn on **Agent Assist** for a deliberately narrow, session-only collaboration surface.

## WebMCP collaboration

Agent Assist is off by default and requires a person to enable it for the current browser session. When available, it lets an agent inspect the selected task, read the visible draft, inspect modeled simulation state, focus source lines, run the unsaved program, and propose a small cooperative edit for the person to review. The MicroDuck workspace adds bounded commands for its browser-only simulation.

The agent surface cannot save or publish source, operate physical hardware, administer devices or networks, or access hidden application data. Temporary agent edits are discarded on refresh unless the person explicitly saves a local draft.

## Screenshots

### MicroDuck policy simulator

![MicroDuck policy simulator with Python workspace, live 3D scene, and control deck](https://jivishov.github.io/RoboBuddy_IDE/screenshot/duck.png)

The MicroDuck workspace pairs an articulated browser simulation with a compact control deck and an editable Python routine.

### SO-101 follower

![SO-101 measured two-bottle transfer workcell](https://jivishov.github.io/RoboBuddy_IDE/screenshot/so101.png)

The SO-101 workspace shows a measured transfer task beside the learner-facing Python program.

### OpenArm bimanual workcell

![OpenArm bimanual heater and ring-stand stack workcell](https://jivishov.github.io/RoboBuddy_IDE/screenshot/openarm.png)

The OpenArm workspace presents a two-arm workcell for examining coordinated robot actions.

### LeKiwi mobile manipulator

![LeKiwi beaker courier workcell](https://jivishov.github.io/RoboBuddy_IDE/screenshot/lekiwi.png)

The LeKiwi workspace combines mobile manipulation with a visible courier task.

### Unitree G1 pose inspection

![Unitree G1 29 degree-of-freedom pose inspection workspace](https://jivishov.github.io/RoboBuddy_IDE/screenshot/unitree.png)

The Unitree G1 workspace is a bounded joint-pose inspection experience within the same IDE.

## Run locally

RoboBuddy IDE is a static browser application and should be served from a local web server.

On Windows, double-click **Launch RoboBuddy IDE.bat**. It starts a local server at http://127.0.0.1:8765/ and opens the app.

Or start a local server yourself:

~~~powershell
py -3 -m http.server 8765 --bind 127.0.0.1
~~~

Then open http://127.0.0.1:8765/.

## Simulation boundary

RoboBuddy IDE is an educational, browser-based simulation environment. The displayed motion, telemetry, contact behavior, camera views, and task outcomes are modeled values, not evidence of hardware behavior. It is not a hardware-control application or a hardware-calibrated digital twin.

## License

RoboBuddy IDE is available under the [MIT License](LICENSE). Bundled third-party assets and components retain their own notices; see [licenses](licenses/) and the relevant asset manifests.
