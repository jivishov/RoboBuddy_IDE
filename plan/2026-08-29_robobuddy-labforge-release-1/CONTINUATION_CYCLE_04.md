# Continuation Cycle 04 - Structured Runs, Evidence, and PR 2 Integration

## Outcome

Reconcile the reviewed worker core onto the completed PR-1 base and deliver PR 2: terminable shared execution, truthful bounded source-plant/kinematic evidence, and agent run/evidence/reset tools without regressing human controls.

## Preconditions and Evidence

- Depends on: Cycles 02 and 03.
- Verify before editing:
  - Cycle 02/PR 1 is on the target base with its review complete;
  - Cycle 03's exact reviewed commits modify only its owned new paths;
  - no other task owns `src/app-v2.js`, `index.html`, `src/python-runtime.js`, `src/source-simulator.js`, `src/agent/**`, or the PR-2 specs;
  - remote `main` and the target branch have not introduced an architectural conflict.
- Reconciliation order: rebase/replay Cycle 03 after the PR-1 base, then let this single task own all shared integration edits.
- Confirm Cycle 02's ready-snapshot/workspace-generation contract remains the sole App-to-agent read boundary after reconciliation.

## Implementation Boundary

In scope:

- Integrate the worker client behind the production runtime interface, remove the main-page Pyodide loader after proof, and avoid duplicate main-thread/worker initialization.
- Refactor `run`, `step`, and `runToCursor` through one narrow internal execution function while preserving their distinct reset/realtime/stateful behavior. Agent runs always reset first and default to non-realtime.
- Wire Stop, timeout, and WebMCP execution abort to worker termination/fresh-worker recovery.
- Capture the ready workspace generation and immutable profile/task/file snapshot at the start of every agent run/reset. A profile/task/source/import/workspace-reset transition during the call terminates or rejects it as `WORKSPACE_CHANGED`; an ordinary simulation reset does not change workspace generation, and no result may be relabeled as belonging to the new workspace.
- Add bounded source-plant summary and separate bounded kinematic-pose summary paths. Exclude high-contrast markers, theme, DOM, reference actions, full scenario, validation cases, and full grade.
- Capture `beforeCommand`, `afterInitialPlantTick`, optional `afterFollowingSleep`, and `terminal` evidence per learner `send_action()`.
- Identify only the recorded `send_action()` call site. Disable agent evidence's action-index-to-reference-label/trajectory-row inference; use `Learner action N`.
- Keep at most five runs and 1,000 actions per run; paginate/truncate output and collision witnesses.
- Store immutable `profileId`, `taskId`, workspace generation/revision when available, and capability union with each run. Completed evidence remains attributable by `runId` after a workspace change until normal five-run eviction.
- Add bounded `run_simulation`, `read_run_evidence`, and `reset_simulation` tools for Inspect/Repair access with registration-epoch/access/workspace-generation/execution-signal rechecks. Registration abort unregisters tools; the execution signal plus worker termination cancels active work.
- Report `executionMode`, `simulationMode`, `stateKind`, `sourcePlantAvailable`, `hardwareValidated`, and `pythonShimOutput` truthfully. G1 has no contact/plant/grade evidence.
- Capture `firstDetectedFault` from structured Python-runtime or plant results with `faultDomain: "python_runtime" | "source_plant" | null`; report action validation separately with `status: "validation_error"`. Do not parse Problems-panel or exception display strings, and never label G1 errors as plant faults.

Preserve or leave for later:

- Preserve human Run/Pause/Resume/Step/Run-to-Cursor/Stop/Reset behavior, current task/profile persistence, themes, high-contrast toggle, and layout.
- No source staging/apply, benchmark/private evaluator, closed-loop Python, provider, or broad service extraction.
- Do not send scenarios or private data to Python and do not weaken source-plant validation.

## Likely Touchpoints

- `src/runtime/python-runtime-client.js`
- `src/runtime/python-worker.js`
- `src/runtime/python-shim.js`
- `src/python-runtime.js`
- `src/app-v2.js`
- `src/source-simulator.js`
- `src/agent/agent-facade.js`
- `src/agent/webmcp-tools.js`
- `index.html`
- `tests/runtime-safety.spec.mjs`
- `tests/webmcp.spec.mjs`
- narrow unit tests for evidence/run-store logic
- `docs/labforge/IMPLEMENTATION_STATUS.md`

## Acceptance Criteria

- Infinite, timed-out, stopped, or WebMCP-cancelled Python terminates the worker and a later run succeeds with a fresh worker.
- All run paths return structured results and preserve existing human-control semantics.
- Agent operations begun while loading/error return `WORKSPACE_NOT_READY`; a mid-call workspace change returns `WORKSPACE_CHANGED`, terminates active worker work when applicable, and cannot mutate the replacement workspace.
- Source-plant evidence is bounded, phase-linked, and call-site-linked; Python shim output is separate.
- G1 run/state output remains explicitly `kinematic_pose` and makes no contact, collision-plant, grade, or hardware claim.
- Tool output is compact, the run store evicts correctly, and no reference label/trajectory row is assigned by learner action index.
- Retained evidence reports the run's original immutable workspace identity, and structured fault domains are correct without UI-string parsing.
- Existing instructional tasks, pinned reference replay, Unitree/high-contrast behavior, layouts, and source validation pass the PR-2 gate.

## Verification Ceiling

Allowed checks:

- `node --check` for all changed/new JavaScript;
- targeted unit tests for runtime limits, evidence phases, run-store bounds/immutable identity, structured fault domains, workspace readiness/change races, and capability unions;
- `npx playwright test` for the directly changed worker/runtime/WebMCP specs during development;
- the refined plan's complete existing repository validation once at the PR-2 gate, including browser smoke/source-plant replay;
- 1366x768 and 390x740 affected layout assertions;
- `git diff --check`.

Do not run:

- closed-loop Python, provider, benchmark, hardware, broad browser, visual-regression, accessibility, performance, deployment, penetration, dependency-security, or generic security suites.

Stop condition: PR-2 acceptance evidence and one fidelity review/refinement record are complete; source staging has not begun.

## Unrun Checks / Residual Risk

- Worker termination is not graceful Python interruption.
- Browser Pyodide remains unsuitable as a hardened hostile-code sandbox.
- Static/browser tests do not prove hardware or closed-loop control.
- Real WebMCP cancellation remains limited if only mocked tool execution is available; record the manual gap.

## Critical Fidelity Review and Refinement Gate

- Coding model and effort: record actual settings at cycle start.
- Review model and effort: ask the user after implementation and allowed checks.
- Default proposal when unspecified: use the coding model two supported reasoning levels higher, capped at its current maximum.
- Review target: cancellation/recovery, registration-versus-execution cancellation, workspace-change races, immutable run attribution, structured fault capture without display-string parsing, output bounds, state/evidence truthfulness, G1/source-plant distinctions, reference-index leakage, duplicate loaders, Pages worker paths, pinned revisions, and preservation of human controls/layout.
- Refinement: correct confirmed in-scope gaps and rerun only affected allowed checks.
- Stop after one review and one refinement pass unless the user requests another.

Do not mark completed until the selected settings actually perform the review.

## Status Handoff

Record reconciliation commits/order, coding/review settings, findings/refinements, changed files, exact checks/results, unrun manual evidence, and residual risk. Set Cycle 05 to `ready` only after PR-2 integration and review are complete.
