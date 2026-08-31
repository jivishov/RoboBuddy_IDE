# Continuation Cycle 04 - Live Python SDK and Execution Controls

## Outcome

Deliver a browser-only async MicroDuck Python SDK that commands the same live simulator and reads live state, integrated with the IDE's Run, Pause/Resume, Step, Run-to-Cursor, Stop, reset, diagnostics, and workspace lifecycle. Preserve the existing compile/replay runtime for the other four robots.

## Preconditions and Evidence

- Depends on: Cycle 03 completed after its fidelity review/refinement gate.
- Verify the reviewed command bus, simulator state, control-deck behavior, current Python compile/replay path, four-file workspace generator, and current dirty WebMCP execution changes before editing.
- Confirm no other task owns `src/python-runtime.js`, `src/app-v2.js`, workspace generation, execution controls, starter files/tests, or new MicroDuck Python modules.
- Record current Run/Pause/Step/Run-to-Cursor/Stop behavior for an existing source-plant profile and Unitree before changing shared execution routing.

## Implementation Boundary

In scope:

- Preserve the authorized fidelity boundary in every Python-facing description and state result: the exact pinned ONNX bytes and Apache-covered hierarchy are source facts, while procedural geometry, mouth pivot, rollers, contacts, sensors, and dynamics are modeled approximations. Do not expose or imply a pinned-`microduck_rl` model, mesh, MJCF, locomotion-parity, or hardware-parity claim.

- Add a profile-specific incremental Pyodide bridge for `policy_sim` in a dedicated MicroDuck Python worker. Use message-correlated requests, run/workspace epochs, bounded cloned payloads, and explicit cancellation; MuJoCo, ONNX, the command bus, and state remain on the main thread. Existing profiles continue to use `PythonRuntime.compileWorkspace()` and event replay unchanged.
- Verify Pyodide 0.29.4's async evaluation behavior with a source-attribution fixture before choosing the evaluator. Top-level `await` must work without `asyncio.run`; if a compile flag or generated wrapper is required, preserve original filename/line mapping rather than assuming a wrapper-free path. Register a browser-only Python module named `microduck`. Its `MicroDuck` object exposes async:
  - `connect()` and `disconnect()`;
  - `move(vx, vy, vyaw)`, `head(neck_pitch, head_pitch, head_yaw, head_roll)`, and `look(x, y, z, neck_pitch=0)`;
  - `stop()`, `enable(on=None)`, `init()`, and `relax()`;
  - `do(skill)`, `pose(z=0, roll=0, pitch=0, active=True)`, and `mouth(open)`;
  - `sound(tag, hold=False)`, `theremin(active)`, and `chorale(active, piece=None, voices=1)`;
  - `mode()`, `set_mode(mode)`, and `get_state()`;
  - `set_color(color)`, `spawn_ball(position=None)`, `reset()`, `set_tof_stimulus(distance=None)`, and `set_camera(mode)`;
  - cooperative `sleep(seconds)`.
- Route every method through `MICRODUCK_COMMANDS` and the same command bus as the visible controls. Return deep-cloned bounded data, source clamping/limiting details, and catalog error codes; never expose JS/MuJoCo/ORT objects.
- Use explicit browser-simulation language in module documentation and starters. `connect()` acquires a simulation controller lease; it does not open a socket or discover hardware.
- Keep workspace filenames exactly `main.py`, `trajectories.py`, `robot_config.py`, and `workcell.py` so the existing bounded WebMCP read allowlist remains valid. Put the concise runnable sequence in `main.py`, API helpers/examples in `trajectories.py`, simulator/client creation in `robot_config.py`, and actual calls to the catalog-backed ball/color/ToF/camera/reset methods in `workcell.py`; do not leave those options as uncallable prose presets.
- Support state-dependent Python branching. `get_state()` must reflect the live post-tick simulator state, not a compile-time shadow or recorded request state.
- Define execution behavior:
  - Run resets MicroDuck, starts a fresh worker/run epoch, acquires the Python lease, and executes top-level async code from the beginning;
  - Pause freezes MuJoCo/policy time and suspends bridge result resolution without allowing the worker to enqueue unbounded commands;
  - Step executes one bridge command boundary; if the current boundary is `sleep`, advance one 20 ms control tick and remain on that line until its duration is exhausted;
  - Run-to-Cursor executes until the next bridge call originating at or beyond the selected line;
  - a Python `sleep()` does not refresh the five-second no-SDK-call continuous-intent lease;
  - Stop, profile/workspace change, reset, timeout, or bridge error immediately invalidates pending requests, neutralizes continuous intent/audio modes, and releases the lease; if Python does not cooperatively settle within the recorded grace period, terminate/recreate the MicroDuck worker so a subsequent run remains usable.
- Preserve source-line attribution for Problems, editor highlighting, action labels, and status. Add MicroDuck-specific terminology rather than calling commands source-plant physical actions.
- Ensure a trusted human manual command preempts the Python lease according to the command-bus contract and returns a structured cancellation/conflict to Python.
- Update starter validation to parse/execute the async MicroDuck workspace in the browser bridge context without pretending CPython alone can operate the simulator.

Preserve or leave for later:

- No physical MicroDuck SDK/socket, background hardware daemon, arbitrary JS access, attachment/file/provider work, or hostile-code sandbox claim. The worker is a cancellation/responsiveness boundary only.
- No new WebMCP control tool; the existing `run_robobuddy_program` may use the integrated App run path but must retain its current access/generation/cancellation guards.
- Do not migrate the four existing profiles to the async bridge in this cycle.

## Likely Touchpoints

- `src/python-runtime.js`
- new `src/microduck/python-bridge.js` and Python shim source/module
- `src/app-v2.js` execution routing
- `src/task-workspace.js` and MicroDuck starter content
- focused Python/bridge unit tests and `tests/microduck-browser-smoke.spec.mjs`
- existing starter and browser-smoke tests for preservation

## Acceptance Criteria

- Every SDK method reaches the same command catalog/bus and yields the same ranges, limiting reasons, mode/skill behavior, state, and errors as the visible controls.
- A MicroDuck Python program can branch on a live `get_state()` result after motion/physics ticks.
- Run, Pause/Resume, command Step, 20 ms sleep Step, Run-to-Cursor, Stop, reset, timeout, profile change, and workspace change meet the specified semantics and do not leave an active lease, stale worker request, or inference loop.
- A failed/cancelled run is followed by a successful fresh run without page reload.
- Problems/status/highlighting distinguish Python, command validation, policy/physics, audio-lock, and cancellation failures and do not label MicroDuck as source-plant evidence.
- Manual control preemption stops the Python run safely and returns an explicit cancellation/conflict.
- SO-101, OpenArm, LeKiwi, and Unitree starter validation and compile/replay Run/Step/Cursor behavior remain unchanged.
- The four MicroDuck starter files expose callable examples for every approved robot and simulation method without making the default run excessively long.

## Verification Ceiling

Allowed checks:

- `node --check` for changed/new JavaScript;
- targeted unit tests for worker/bridge request correlation, epoch invalidation, state cloning, leases, cooperative and forced termination, source attribution, method/catalog parity, and error mapping;
- existing `py -3` starter/reference checks for the four unchanged profiles;
- syntax/static inspection of the MicroDuck starter plus browser execution through the real Pyodide bridge;
- a consolidated set of targeted Playwright journeys for live state branching, pause/resume, both Step cases, cursor run, cooperative stop, non-awaiting-code worker termination/later-run recovery, manual preemption, and one existing source-plant plus Unitree preservation path;
- `node tests/validate.mjs` and affected task/workspace validation;
- focused diff review and `git diff --check`.

Do not run:

- full browser suite, provider/file APIs, hostile-code security testing, performance profiling, hardware, deployment, accessibility, penetration, dependency-security, or generic security checks.

Stop condition: live state-dependent Python and all shared execution-control acceptance criteria pass, existing profiles remain on their prior path, and one fidelity review/refinement pass is complete.

## Unrun Checks / Residual Risk

- Browser Pyodide is not a secure sandbox for hostile Python.
- Forced worker termination is abrupt and may discard Python-local state; prove lease/command cleanup and later-run recovery, but do not describe it as graceful interruption or a secure sandbox.
- Browser simulation APIs do not prove compatibility with physical MicroDuck JSON-RPC or hardware timing.

## Critical Fidelity Review and Refinement Gate

- Coding model and effort: record actual settings at cycle start.
- Review model and effort: ask the user after implementation and allowed checks.
- Default proposal when unspecified: use the coding model two supported reasoning levels higher, capped at its current maximum.
- Review target: live versus shadow state, complete robot/simulation catalog parity, worker bridge epochs and cooperative/forced cancellation, lease cleanup/manual priority, exact Pause/Step/sleep/Cursor semantics, verified top-level-await source attribution, truthful simulation language, four-file/read-allowlist compatibility, later-run recovery, and existing-profile compile/replay preservation.
- Refinement: correct confirmed in-scope gaps and rerun only affected checks above.
- Stop after one review and one refinement pass unless the user requests another.

Do not mark completed until the selected review settings actually perform the review.

## Status Handoff

Record actual model settings, Python public API, execution/cancellation behavior, findings/refinements, files changed, exact checks/results, unrun checks, and residual risk. Set Cycle 05 to `ready` only after the live bridge and existing-profile regressions are stable.

## Completed Fidelity Review and Handoff

- Coding model and effort: `gpt-5.6-sol` / `medium`.
- Review model and effort: `gpt-5.6-sol` / `xhigh`.
- Review result: Cycle 04 acceptance criteria pass after one in-scope refinement pass. Cycle 04 is `completed`; Cycle 05 is `ready` and was not implemented.
- Reconciled baseline: repository `C:/Users/EmilJivishov/Projects/RoboBuddy_IDE`, branch `codex/MICRODUCK`, `HEAD` and live `origin/main` both `ccc50cb5018e6d532873181538583f1a6e636b5b`. The pre-existing dirty Cycle 01-03/WebMCP/UI work remains preserved.

### Delivered Python surface

The browser-only `microduck.MicroDuck` async API is: `connect`, `disconnect`, `move`, `head`, `look`, `stop`, `enable`, `init`, `relax`, `do`, `pose`, `mouth`, `sound`, `theremin`, `chorale`, `mode`, `set_mode`, `get_state`, `set_color`, `spawn_ball`, `reset`, `set_tof_stimulus`, `set_camera`, and cooperative `sleep`. Every simulator method is mapped to the frozen `MICRODUCK_COMMANDS` catalog except bridge-only connection, disconnection, and sleep. The workspace is exactly `main.py`, `trajectories.py`, `robot_config.py`, and `workcell.py`, matching the bounded WebMCP read allowlist.

Run creates a fresh worker/run epoch after resetting the simulator. Pause suspends bridge resolution and policy/physics time. Step resolves one bridge boundary; sleep Step advances exactly one 20 ms control slice and awaits inference started by that slice. Run-to-Cursor reports the actual next bridge boundary at or beyond the requested line. Stop, reset, workspace edit, profile change, timeout, bridge error, and trusted human preemption invalidate the run, neutralize the controller/audio state, release its lease, and terminate an unresponsive worker after the bounded grace period. A later run remains usable. Existing source-plant and Unitree workspaces remain on `PythonRuntime.compileWorkspace()` plus main-thread replay.

### Critical findings and one refinement pass

- Enforced the active controller lease for every mutating catalog command, including one-shot reset/stop/mode commands; cross-controller state/mode reads remain non-mutating. Trusted human commands preempt Python, while peer/lower-priority writes receive `COMMAND_CONFLICT`.
- Neutralized continuous intent and stopped generated audio on five-second lease expiry. Every later SDK boundary, including a repeated `connect`, reacquires an expired Python lease; cooperative sleep itself does not refresh it.
- Made paused sleep Step advance one 20 ms physics/control slice, permit that slice's policy tick, and await newly launched inference. Reset now also synchronizes the canvas clock diagnostic to the reset state.
- Bounded worker requests, source attribution, cloned payloads, per-stream output, and worker error text; rejected any workspace other than the exact four-file set in both bridge and worker.
- Corrected Run-to-Cursor status to use the actual bridge source line and report completion-before-boundary truthfully.
- Removed source-plant/LeRobot wording from the MicroDuck task/fidelity presentation and replaced “physical action” Step terminology with a MicroDuck Python boundary label.
- Expanded authority, expiry, reacquisition, workspace/source/output bounds, Step timing/inference completion, audio-lock, cleanup, actual cursor boundary, later-run recovery, and existing-profile preservation evidence.
- Evidence chronology: the new lifecycle journey first reached a successful final rerun but used a nonexistent console selector; that assertion was corrected and passed. The new Step-clock assertion then exposed the stale reset diagnostic described above; the diagnostic was repaired and the affected journey passed on rerun.

### Cycle 04 files changed

- Runtime/routing: `src/microduck/python-bridge.js`, `src/microduck/python-worker.js`, `src/microduck/command-bus.js`, `src/microduck/policy-simulator.js`, `src/simulator-host.js`, `src/app-v2.js`.
- Workspace: `src/task-workspace.js`.
- Evidence: `tests/microduck-python-bridge.mjs`, `tests/microduck-policy-core.mjs`, `tests/microduck-browser-smoke.spec.mjs`, `tests/browser-smoke.spec.mjs`.
- The legacy `src/python-runtime.js` was intentionally not changed.

### Executed checks and results

- `node --check` over 44 source/test/script `.js`/`.mjs` files: PASS.
- `node tests/validate.mjs`: PASS.
- `node tests/validate_canonical_visuals.mjs`: PASS.
- `node tests/validate_task_patch.mjs`: PASS.
- `node tests/validate_microduck_assets.mjs`: PASS; 19 files, 9 policies, 14 joints, 2 release-safe scores, and 2 fixed inference outputs.
- `node tests/microduck-policy-core.mjs`: PASS, 8/8.
- `node tests/microduck-ui-core.mjs`: PASS.
- `node tests/microduck-python-bridge.mjs`: PASS.
- `py -3 tests/validate_starters.py`: PASS.
- `py -3 tests/execute_starters.py`: PASS; OpenArm 7, SO-101 8, and LeKiwi 6 physical-action calls.
- `py -3 tests/validate_openarm_reference.py`: PASS.
- Targeted Playwright final evidence: PASS for seven journeys: MicroDuck profile/backend/fidelity lifecycle; live Python state, Pause/Resume, command and sleep Step, actual cursor boundary, forced Stop recovery, and manual preemption; reset/edit/profile cleanup and later-run recovery; Python/audio source attribution; source-plant plus Unitree compile/replay preservation; explicit human WebMCP Assist guard; and the consolidated MicroDuck policy trace.
- `git diff --check`: PASS, with only Git's existing LF-to-CRLF checkout warnings.
- Explicit trailing-whitespace scan of the owned refinement files: PASS.

### Unrun checks and residual risk

Per the verification ceiling, the full browser suite, provider/file APIs, hostile-code security testing, performance profiling, hardware, deployment, accessibility, penetration, dependency-security, and generic security checks remain UNRUN. Pyodide is not a hostile-code sandbox; the worker is only a responsiveness/cancellation boundary. Forced termination can discard Python-local state. The browser-only API and approximate geometry/contacts/sensors/dynamics do not establish physical JSON-RPC compatibility, locomotion/RL-environment parity, hardware timing, or hardware validation. Pyodide 0.29.4 remains loaded from its declared jsDelivr runtime URL rather than vendored locally.
