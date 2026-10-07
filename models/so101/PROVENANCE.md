# SO-101 physical model and IDE migration provenance

RoboBuddy IDE Physics Preview uses two pinned SO-101 MuJoCo packages for different evidence purposes. Neither is a hardware-calibrated digital twin.

## Pinned source basis

The executed SO-101 models are based on Google DeepMind MuJoCo Menagerie `robotstudio_so101` at revision `8161bba264d7fa7c99ca301e91e7fb44737676ad`. Menagerie records its upstream derivation from The Robot Studio `SO-ARM100` simulation model at revision `aec17bbc256d1a7342d53aaa4950595d4c30b40d`. The SO-101-derived model content is Apache-2.0.

Primary source paths:

- Menagerie: `robotstudio_so101/so101.xml`
- Upstream project: `Simulation/SO101/so101_new_calib.xml`

Menagerie intentionally adapts the project model for MuJoCo simulation, including `implicitfast`, actuator settings, primitive collision geometry, gripper contact settings, and a camera mount. A value described below as source-derived is traceable to the named source layer; it is not automatically a measured manufacturer or installed-hardware parameter.

## Model packages

### Articulation/conformance reference

- Package: `so101-phase2a-menagerie-8161bba`
- Model ID: `robobuddy-so101-phase2a-v1`
- Asset: `models/so101/model.xml`
- SHA-256: `8573559b58eb522ca80c8cd4c88d30e0b2ab20fb222f67c57e3808a4783af085`

This reduced model remains a deterministic articulation/browser-native conformance reference. It deliberately omits source components that matter to manipulation and is not the normal physical SO-101 IDE workspace.

### Manipulation/normal IDE physical package

- Package: `so101-manipulation-menagerie-8161bba-v1`
- Model ID: `robobuddy-so101-manipulation-v1`
- Asset: `models/so101/manipulation.xml`
- SHA-256: `cf0064059a56e2c6748f72733c606b833ebf61131802b27b3645c1f25089aecc`
- MuJoCo timestep: `0.005 s`
- Integrator: `implicitfast`
- Solver iterations: `10`
- Line-search iterations: `20`

This package retains the source-derived six-actuated-joint topology, articulated-body inertials, gripper primitive collision geometry, and the pinned source camera collision boxes. The source camera-mount visual mesh mass of `0.012 kg` is restored by distributing that mass across the two source camera collision boxes; the resulting box-based inertia is an explicit estimate, not the original mesh inertia.

The manipulation model contains a true free-body benchmark block (`benchmark_block_free`). Ordinary execution does not weld, parent, attach, snap, or write the block transform. The target marker is non-colliding. The pickup and target supports are split synthetic benchmark fixtures so the workcell does not place a broad support surface through the retained camera collision swept volume.

## Wrist-roll discrepancy

The source layers disagree in a way that must remain visible:

- The Robot Studio revision recorded by Menagerie gives `wrist_roll` an upper joint range of approximately `2.8412063 rad`.
- The pinned Menagerie model constrains the joint symmetrically to approximately `-2.7438473 .. 2.7438473 rad` while its actuator control range extends to about `2.84121 rad`.

RoboBuddy executes the pinned Menagerie joint constraint. Commands are validated against both the joint and actuator envelopes, so the effective command range is their intersection. This is a simulation-source decision, not a measured hardware limit. Actual assembled-robot usable range must be calibrated separately.

## Evidence classes

### Source-derived

- six-joint hierarchy, transforms, local axes, and pinned Menagerie simulation joint limits;
- retained articulated-body masses and inertia tensors;
- `0.005 s` timestep, integrator, and solver settings;
- position-actuator mappings and source simulation control ranges;
- retained gripper primitive collisions;
- camera collision boxes and the source `0.012 kg` camera-mount mass value.

### Estimated / synthetic benchmark

- Menagerie STS3215 simulation gains/effort behavior when interpreted as physical hardware behavior;
- box-derived inertia distribution used to restore the camera-mount mass;
- benchmark block dimensions, `0.020 kg` mass, inertia, friction, support-pad dimensions/positions, and target region;
- benchmark work-surface contact parameters;
- task-evaluator acceptance thresholds: `0.030 m` lift clearance above the block's nominal support height, `0.050 m` horizontal carried travel, `0.012 m` final support-height tolerance, at least `0.20 s` of post-release target-support dwell, and at most `0.0005 m` positional drift during that dwell.

The evaluator thresholds are declared acceptance criteria for this synthetic benchmark. They are chosen to reject contactless lift, free-body travel after a lost grasp, transient support contact, and visibly continuing translation while remaining comfortably inside the already validated P4 nominal behavior. They are not measured SO-101 hardware performance tolerances.

These parameters support a controlled simulator benchmark. They are not measured laboratory geometry or installed-hardware calibration.

### Calibration-required

Hardware-aligned fidelity requires independent measurement or validation of at least joint zero offsets and usable limits, installed servo configuration/gains/latency, torque-speed and effort limits, backlash/friction/compliance, gripper opening/force/friction/fingertip geometry, mount/tool/camera configuration, payload mass/centre of mass, work-surface properties, and model-to-hardware trajectory error.

## Normal SO-101 physical IDE architecture

The supported SO-101 workspace is `SO-101 Physical Block Transfer` (`so101-physical-block-transfer`). It uses the manipulation package above through one authoritative browser `PhysicsSession` backed by MuJoCo.

- Live Python uses `robobuddy.sim.v1`, SI units, radians, and fixed simulation-time advancement.
- A command acknowledgement means the target was accepted/latched; it is not target achievement.
- WebMCP uses versioned schema `robobuddy.so101.physical.v1` with explicit radian targets and bounded physics-step/time budgets.
- Python and WebMCP route to the same current PhysicsSession and reject stale session/epoch ownership.
- The Three.js canonical SO-101 rig is presentation-only. MuJoCo radians are converted at the rendering boundary; the benchmark block is rendered directly from the MuJoCo body observation.
- Rendering does not advance physics.
- Task evaluation consumes the same MuJoCo observations. Success requires observed gripper contact, lift while gripper contact is present, horizontal carry measured from a continuous lifted-contact anchor, release, target inclusion, sustained post-release target-support contact with bounded positional drift, and final rest. Program completion or a commanded endpoint is not success.
- Reset is deterministic initial-condition creation. It clears task evidence and does not count as recovery of a failed manipulation.

## Supported and deferred SO-101 tasks

### Supported

`so101-physical-block-transfer` is the only task exposed as completed physical SO-101 functionality in this milestone. It is deliberately a rigid-body benchmark, not a hardware-calibrated laboratory operation.

### Deferred

The following legacy RoboBuddy_AI definitions remain read-only provenance/regression fixtures but are not exposed as completed physical SO-101 tasks:

- `so101-v2-06-quantitative-transfer`: current dry two-bottle staging concept is potentially rigid-body defensible, but needs validated bottle geometry, multi-object grasps/releases, and controller evidence. Liquid transfer remains unsupported.
- `so101-v2-08-burette-initial-reading`: current dry bottle/beaker clearance concept is potentially rigid-body defensible, but needs validated glassware/burette fixtures and multi-object manipulation. Filling, dispensing, meniscus reading, and liquid metrology remain unsupported.
- `so101-v2-09-vacuum-filtration`: current dry keep-clear concept is potentially rigid-body defensible, but needs validated workcell geometry and multi-object manipulation. Vacuum, hose coupling, pressure, filtration, and liquids remain unsupported.

No legacy task is preserved by adding fake liquid behavior, hidden grasp attachment, object teleportation, assisted animation semantics, or unjustified contact parameters.

## Validation boundary

Browser/native MuJoCo agreement and the P4 positive/negative/sensitivity tests validate meaningful behavior of the declared simulator package. They do not establish hardware calibration or parity with a specific assembled SO-101. Future hardware claims require separate physical measurements and acceptance criteria.
