# Continuation Cycle 03 - Terminable Python Worker Core

## Outcome

Build and directly test the real Pyodide worker/client/shim core needed by PR 2, on a separate worktree, without touching Cycle 02 or shared integration files. The result is a reviewed preparation branch for Cycle 04, not a standalone merge-ready PR.

## Preconditions and Evidence

- Depends on: Cycle 01.
- Start from the same exact coordinator-recorded Cycle-01 base SHA as Cycle 02, in a separate worktree on `feat/labforge-02-cancellable-runs`; verify `git rev-parse HEAD` equals that SHA before editing.
- Verify before editing:
  - Cycle-01 measured instructional event/sleep maxima and selected margins;
  - Pyodide remains pinned to 0.29.4 and current `PythonRuntime.compileWorkspace(files)` result semantics are characterized;
  - the controlled Playwright pattern discovers a new direct worker-core spec;
  - no file in this cycle's ownership list is being edited by another task.

## Implementation Boundary

In scope:

- Add a real worker client, worker, and extracted shim implementing `INIT`, `RUN`, `CANCEL`, and `DISPOSE` requests with unique request IDs and `READY`, `RUN_RESULT`, and `RUN_ERROR` responses.
- Pin the worker's Pyodide 0.29.4 URL and construct the worker relative to `import.meta.url` so later integration can work under a Pages subpath.
- Reject or ignore malformed/unknown messages; send no scenario, grader, DOM, storage, or private data to the worker.
- Enforce named source/event/sleep/stdout/stderr/init/run limits derived from Cycle 01. Label stdout/stderr as Python shim output only.
- On timeout, abort, or unresponsive execution, terminate the worker, reject with `cancelled` or `timeout`, and create a fresh worker for the next run. `CANCEL` is advisory; termination is the reliable fallback.
- Directly test success, Python error, event/sleep/output limits, infinite-loop termination, caller abort, fresh-worker recovery, and unknown-message handling by importing the real new core modules through the existing loopback/static Playwright server. Do not duplicate the worker protocol or shim in a test-only implementation; exercise a repository-subpath URL.

Preserve or leave for later:

- Do not edit `src/app-v2.js`, `index.html`, `src/python-runtime.js`, `src/source-simulator.js`, `src/agent/**`, existing CSS, package/test discovery configuration, docs status, `_CYCLE_STATUS.json`, or `_DEVELOPMENT_CYCLES.md`.
- Do not expose WebMCP run tools, remove the main-page loader, integrate UI buttons, or claim source-plant evidence in this cycle.
- Do not add SharedArrayBuffer requirements, new dependencies, a secure-sandbox claim, or benchmark/private data.

## Likely Touchpoints

- `src/runtime/python-runtime-client.js`
- `src/runtime/python-worker.js`
- `src/runtime/python-shim.js`
- `tests/runtime-worker-core.spec.mjs` or an equivalently named new direct core spec

## Acceptance Criteria

- Current instructional workspace inputs compile to the characterized event/stdout/stderr/exception shape in the direct core harness.
- Infinite or cancelled Python cannot hold the page indefinitely; a fresh subsequent run succeeds.
- Resource limits fail with bounded structured results, and unknown/malformed messages do not become executable operations.
- The worker receives learner files only; no scenario, source-plant, validation, UI, or local-storage data enters it.
- Only the new owned paths changed, `git diff --name-only <base-sha>...HEAD` proves that ownership, and the branch commit can be replayed onto the completed PR-1 base without content conflicts.

## Verification Ceiling

Allowed checks:

- `node --check src/runtime/python-runtime-client.js src/runtime/python-worker.js src/runtime/python-shim.js`;
- the new direct worker-core Playwright spec and any narrow Node unit tests that do not require app integration;
- `git diff --check` and `git diff --name-only <base-sha>...HEAD` path-ownership review.

Do not run:

- the full repository/PR-2 suite, because production integration is intentionally deferred to Cycle 04;
- Chrome Inspector or ChatGPT desktop built-in-browser WebMCP journeys, broad browser matrices, visual regression, accessibility, performance, hardware, deployment, or generic security scans.

Stop condition: the worker core and direct tests pass, the owned-path diff is reviewed, and one fidelity review/refinement pass is complete.

## Unrun Checks / Residual Risk

- The main app still uses the old main-thread runtime until Cycle 04.
- The main-page Pyodide loader remains until Cycle 04, so this branch is not merge-ready by itself.
- Pages-subpath behavior is exercised only by the direct spec; full UI/runtime compatibility remains unproven.

## Critical Fidelity Review and Refinement Gate

- Coding model and effort: record actual settings at cycle start.
- Review model and effort: ask the user after implementation and allowed checks.
- Default proposal when unspecified: use the coding model two supported reasoning levels higher, capped at its current maximum.
- Review target: real termination/recovery, protocol validation, correct init-versus-run timeout accounting, no private data, no sandbox overclaim, real-module/repository-subpath coverage, parity with characterized shim semantics, and strict ownership isolation.
- Refinement: correct confirmed in-scope gaps and rerun only the direct core checks.
- Stop after one review and one refinement pass unless the user requests another.

Do not mark completed until the selected settings actually perform the review.

## Status Handoff

Record the shared Cycle-01 base SHA, branch and exact commit(s), `git diff --name-only <base-sha>...HEAD` ownership proof, model settings, findings/refinements, owned files, checks run, unrun integration evidence, and remaining risk in this continuation record. Do not race Cycle 02 on central status files; send the record to the coordinator for serial reconciliation.

## Completion Record - 2026-08-29

- Shared reviewed base: `ca879e9032280dfcb7a0471c46d38285aef13fe6`.
- Branch/commit: `feat/labforge-02-cancellable-runs` at `09fb411aee40153636098f5e2fdcdec5dc17d9d4`.
- Models: implementation `gpt-5.6-sol` at `medium`; fidelity review `gpt-5.6-sol` at `xhigh`.
- Ownership: only `src/runtime/python-runtime-client.js`, `src/runtime/python-worker.js`, `src/runtime/python-shim.js`, and `tests/runtime-worker-core.spec.mjs`.
- Fidelity refinements: enforce the 30-second hard run ceiling; validate learner files before `postMessage` plus worker defense-in-depth; strengthen malformed-response/request-correlation handling; prove init time is outside run timing; add multi-file shim parity, advisory CANCEL evidence, event-payload bounds, and fresh-worker recovery.
- Verification: all four syntax checks passed; real-worker Playwright spec passed 1/1; staged/diff hygiene passed.
- Unrun/residual: production app integration, exact generated-starter matrix through the new worker, the full PR-2 suite, Pages/UI journeys, Chrome Inspector, ChatGPT Site tools, layouts, hardware, deployment, and broad out-of-ceiling checks. The app remains on the legacy main-thread runtime until Cycle 04.
- No shared integration file, coordinator file, dependency installation, push, merge, PR, publication, or deployment occurred.
