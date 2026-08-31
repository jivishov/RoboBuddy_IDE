# RoboBuddy MicroDuck Policy Simulator Development Cycles

## Goal and Plan

Implement the complete approved MicroDuck browser simulation and bounded WebMCP control surface. `meta_plan.md` is authoritative for source pins, decisions, shared contracts, acceptance criteria, and the verification ceiling.

## Startup Protocol

A fresh session starts in `C:/Users/EmilJivishov/Projects/RoboBuddy_IDE` and reads, in order:

1. applicable instructions supplied with the task;
2. `plan/2026-08-30_robobuddy-microduck-policy-simulator/meta_plan.md`;
3. `_CYCLE_STATUS.json`;
4. this cycle index;
5. its assigned `CONTINUATION_CYCLE_NN.md`.

Before editing, recheck `git rev-parse HEAD`, branch, `git status --short`, live `origin/main`, and the dirty-path preservation list. Do not assume the observed baseline is still current. If another task owns an overlapping file or the dirty baseline changed materially without an attributable handoff, pause and report the exact conflict rather than resetting or overwriting it.

## Shared Constraints

- Keep the latest accepted plan intact: full browser simulation, complete approved control mirror, a full articulated 3D rig in the shared viewport, and bounded direct WebMCP control.
- Preserve the four existing profiles, their current source-plant/kinematic distinctions, themes, toolbar refinement, compact layout, task/workspace persistence, and current five WebMCP tools.
- Reuse the IDE simulator/profile lifecycle, but keep MicroDuck behind `policy_sim` and a dedicated MuJoCo backend. Never report it as the RoboBuddy source plant or as physical hardware.
- Centralize pins and per-file provenance in the asset manifest and commands/lifetimes/abort behavior in `MICRODUCK_COMMANDS`; do not scatter string/version/safety conditionals. Deployed ONNX bytes and the fourteen-joint hierarchy come only from the pinned Apache-2.0 runtime. CPU/browser fixtures prove exact `1x61 -> 1x14` inference behavior only; the Space remains a sequencing oracle and `microduck_rl` mesh/MJCF bytes are excluded.
- Keep `SimulatorHost` as the sole owner of the canvas/backend clock. Backend activation, awaited inference, Python runs, and WebMCP registration all use explicit epochs/generations so stale asynchronous work cannot publish into a replacement profile/workspace.
- Preserve the current dirty files and untracked WebMCP modules. No reset, clean, mass reformat, unrelated rewrite, staging, commit, push, publish, deployment, or PR without separate authorization.
- Run only one cycle at a time. These cycles share app, profile, simulator, and WebMCP integration surfaces and are not parallel-ready.

## Verification Ceiling

Use only the checks authorized by `meta_plan.md` and the active continuation file. Target changed modules/specs during implementation. Run `npm run test:browser` only at the final integration gate unless a continuation explicitly names a smaller Playwright invocation. Use `git diff --check` for tracked changes plus explicit file-enumeration/trailing-whitespace checks for new owned files. Record all stronger checks as unrun.

## Per-Cycle Fidelity Review Protocol

Every cycle closes with one critical fidelity review and one refinement pass. Record the actual coding model and effort. If review settings are not already selected, pause after implementation and allowed checks; ask the user; offer the coding model at two currently supported effort levels higher, capped at its maximum; and let the user approve it for the cycle, apply it to later cycles, or replace it. The selected settings must actually perform the review before status becomes `completed`.

The review compares the deliverable with the user goal, `meta_plan.md`, cycle acceptance criteria, local instructions, public interfaces, source pins, licensing/provenance, fidelity boundaries, and dirty-work preservation. Correct only confirmed in-scope gaps and rerun only affected allowed checks.

## Dependency Map

| Cycle | Outcome | Depends on | Initial status | Primary ownership |
|---|---|---|---|---|
| 01 | Local coherent asset/policy bundle and fifth articulated rig | None | ready | Provenance, assets, rig adapter, single-host backend/profile seam |
| 02 | Complete MuJoCo/ONNX policy engine and shared command/state core | 01 | pending | Policy simulator, command catalog/bus, physics/state |
| 03 | Complete human-facing control deck and modeled peripherals | 02 | pending | UI, keyboard/gamepad, cameras/sensors/audio, layout |
| 04 | Live async MicroDuck Python SDK with IDE execution semantics | 03 | pending | Profile-specific Pyodide worker/bridge, starters, Run/Pause/Step/Cursor/Stop |
| 05 | Conditional direct WebMCP control and reconciled final integration | 04 | pending | Existing WebMCP modules, app snapshot/epochs, docs/tests |

## Cycle Summaries

### Cycle 01 - Coherent Local Bundle and Articulated-Rig Gate

Resolve provenance and compatibility first. Vendor and validate the exact local runtime/policy bundle, generate an original procedural browser rig around the pinned Apache-covered hierarchy, introduce a single `SimulatorHost` that never leaks multiple renderers/loops, and render the fifth articulated robot in the shared viewport. Stop if license provenance, joint mapping, policy shape, local serving, lifecycle disposal, or exact CPU/browser inference parity fails.

### Cycle 02 - Policy Simulation and Command-State Core

Implement the fixed-step MuJoCo/ONNX backend, all nine policies and both modes, runtime tuning, command arbitration, state, ball contact, and walking recovery. Stop with a deterministic programmatic API; visible control-deck, Python, and WebMCP work remain later.

### Cycle 03 - Control Deck, Inputs, Sensors, and Audio

Expose every approved option to a person through a compact MicroDuck deck, focused keyboard/gamepad mapping, virtual cameras, modeled IMU/ToF, colors, ball/perturbation, sounds, theremin, and local chorale. Preserve laptop and compact reachability and the trusted audio-unlock boundary.

### Cycle 04 - Live Python SDK and Execution Controls

Add the incremental async MicroDuck Pyodide worker/bridge and four-file starter workspace so Python can command and read the live main-thread simulator. Integrate Run, Pause/Resume, Step, Run-to-Cursor, Stop, cooperative/forced cancellation, and later-run recovery while preserving compile/replay for existing profiles.

### Cycle 05 - WebMCP Control and Integration Reconciliation

Extend the current dirty WebMCP implementation with one conditional strict control tool, correct policy-simulation state/capability output, durations/abort behavior, and no-write/hardware boundaries. Reconcile documentation and run the approved integration gates without expanding into release or hardware claims.

## Completion Protocol

After implementation and allowed checks, complete the cycle fidelity review/refinement gate. Update only that cycle plus the smallest `currentCycle`/`lastCompleted` fields needed for the next session. Record actual model settings, findings, refinements, changed assumptions, files changed, exact checks/results, unrun checks, and remaining risk in the continuation file. Do not rewrite another cycle in advance.
