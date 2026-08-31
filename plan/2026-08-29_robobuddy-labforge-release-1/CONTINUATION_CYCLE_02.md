# Continuation Cycle 02 - Opt-in Read-only WebMCP Inspection

## Outcome

Deliver PR 1: a human-enabled, session-scoped read-only WebMCP surface that inspects unsaved instructional workspace state through explicit application methods and returns bounded, truthful capability labels.

## Preconditions and Evidence

- Depends on: Cycle 01.
- Start from the exact approved Cycle-01 commit in a dedicated worktree/branch `feat/labforge-01-webmcp-readonly`.
- Verify before editing:
  - PR-0 tests and docs exist and the baseline still contains all preserved Unitree/high-contrast/theme work;
  - `src/app-v2.js` still owns unsaved files and ends with direct `new App()` construction;
  - there is no existing production `document.modelContext` registration;
  - Cycle 03, if active, owns only its new `src/runtime/**` files and direct worker-core spec.

## Implementation Boundary

In scope:

- Retain the `App` instance without exposing it globally in production; permit only a `?ci=1` test hook if required.
- Add one narrow App snapshot method for task summary, workspace pagination, capability labels, workspace status, and Agent access. It returns an immutable, internally consistent snapshot rather than letting handlers read mutable fields one at a time.
- Add a monotonic workspace generation. Increment it at the start of a profile/task load and on each source edit, import, or workspace reset; an async load commits only if its captured generation is still current. Mark the workspace loading before transition work and ready only after scenario, files, simulator, profile, and task agree. Loading/error calls return bounded `WORKSPACE_NOT_READY`; a missing scenario never means source-plant.
- Add a compact `Off | Inspect` control (Repair may remain disabled or absent), default Off on every load and session-only. Change it only from a trusted browser user activation (`event.isTrusted`) on the visible control; preserve pointer, keyboard, and assistive-technology activation while rejecting script-generated `.click()`.
- Register exactly `describe_lab_task` and `read_workspace` through feature-detected top-level imperative WebMCP registration.
- Use the registration signal to unregister tools, but never assume it cancels an in-flight call. Use the per-execution `{ signal }` for call cancellation and recheck access, registration epoch, and workspace generation immediately before every sensitive read and before returning asynchronous results.
- Keep outputs within the refined plan's approximate 1.5 KB limits and return structured domain errors without stacks.
- Add capability fields so source-plant workspaces and Unitree G1 `kinematic_pose` are never conflated. `executionMode` remains `open_loop_event_replay`; `hardwareValidated` remains false.
- Mark task/source content untrusted. `describe_lab_task` returns no action values. `read_workspace` labels explicitly requested starter/reference content as instructional and paginates it.

Preserve or leave for later:

- No agent run/reset, evidence, editing, staging, Apply, benchmark, provider, or worker functionality.
- Do not touch Cycle-03-owned `src/runtime/**` files.
- Do not refactor a general workspace service or scrape/click the DOM from handlers.
- Preserve themes, high-contrast scene controls, G1/source-plant paths, persistence, and 1366x768/390x740 reachability.

## Likely Touchpoints

- `src/agent/agent-facade.js`
- `src/agent/webmcp-tools.js`
- `src/app-v2.js`
- `index.html`
- the smallest relevant CSS file(s)
- `tests/webmcp.spec.mjs`
- `docs/labforge/IMPLEMENTATION_STATUS.md`
- `docs/labforge/WEBMCP_COMPATIBILITY.md`

## Acceptance Criteria

- Access defaults Off, no tools register while Off, trusted human Inspect activation registers exactly two tools, and Off unregisters them. A direct DOM `.click()` test cannot enable access; a real Playwright pointer/keyboard activation can.
- A stale epoch or disabled access returns no task/source content.
- Loading/error workspaces return `WORKSPACE_NOT_READY`; rapid profile/task switches cannot let an older load overwrite or leak into the current snapshot, and `scenario = null` cannot report `sourcePlantAvailable: true`.
- The tools read current unsaved editor state, paginate source, bound output, reject invalid inputs, and return no reference action values in summaries.
- Source-plant and Unitree G1 summaries carry truthful simulation/state capability labels.
- Unsupported browsers load normally and existing UI/simulator behavior remains intact.
- Chrome Inspector checks and ChatGPT desktop built-in-browser site-tool checks are recorded as separate lanes when available; mocks are labeled as mocks and Chrome is not reported as ChatGPT interoperability.

## Verification Ceiling

Allowed checks:

- `node --check` for changed/new modules;
- targeted unit tests for schemas, pagination, trusted activation, access/epoch checks, workspace readiness/generation, stale-load suppression, and missing-scenario capability handling;
- `npx playwright test tests/webmcp.spec.mjs` and affected layout/smoke assertions;
- the refined plan's full repository validation once at the PR-1 gate;
- specific Chrome Inspector checks and separate ChatGPT desktop built-in-browser site-tools checks when available;
- `git diff --check`.

Do not run:

- broad visual regression, accessibility, performance, hardware, deployment, penetration, dependency-security, or generic security suites;
- live provider/API calls.

Stop condition: PR-1 acceptance evidence, manual availability notes, and the fidelity review/refinement record are complete.

## Unrun Checks / Residual Risk

- A mock `document.modelContext` cannot prove Chrome Inspector discovery or ChatGPT built-in-browser tool discovery, and Chrome results do not prove ChatGPT site-tool availability.
- Site-tools rollout may prevent the manual gate; record this without weakening access control or claiming interoperability.
- No code execution or source modification is authorized through the new tools.

## Critical Fidelity Review and Refinement Gate

- Coding model and effort: record actual settings at cycle start.
- Review model and effort: ask the user after implementation and allowed checks.
- Default proposal when unspecified: use the coding model two supported reasoning levels higher, capped at its current maximum.
- Review target: trusted access activation, registration-versus-execution cancellation, access/epoch/workspace-generation races, atomic ready snapshots, bounded and untrusted output, no source/reference leakage in summaries, truthful source-plant/G1 labels including missing-scenario handling, distinct Chrome/ChatGPT claims, unsupported-browser behavior, and preservation of current UI/runtime behavior.
- Refinement: correct confirmed in-scope gaps and rerun only affected checks.
- Stop after one review and one refinement pass unless the user requests another.

Do not mark completed until the selected settings actually perform the review.

## Status Handoff

Record the shared Cycle-01 base SHA, PR-1 branch/commit, `git diff --name-only <base-sha>...HEAD` ownership proof, actual model settings, findings/refinements, files changed, checks and lane-specific manual probes run, unrun checks, and residual risk. During parallel execution, do not edit central status files concurrently with Cycle 03; send the completion record to the coordinator for a serial status update.

## Completion Record - 2026-08-29

- Shared reviewed base: `ca879e9032280dfcb7a0471c46d38285aef13fe6`.
- Branch/commit: `feat/labforge-01-webmcp-readonly` at `1b4b01576b201c74f348f5d996142cac125ddfa7`.
- Models: implementation `gpt-5.6-sol` at `medium`; fidelity review `gpt-5.6-sol` at `xhigh`.
- Fidelity refinements: serialize Reset/profile transitions; deny editor/import/starter mutations while loading/error; cap complete structured tool results by UTF-8 bytes; preserve semantic Off/Inspect state and simultaneous Agent/Run/Stop reachability.
- Verification: affected syntax checks passed; unit 8/8; refined WebMCP 7/7; affected Step/Pause/Reset 3/3; deterministic single-worker browser gate 17/17; `git diff --check` passed. An earlier two-worker gate ended 12/17 when the loopback server exited; the five failures were `ERR_CONNECTION_REFUSED` and remain recorded as infrastructure evidence.
- Unrun/residual: Chrome Inspector, deployed Pages, ChatGPT built-in-browser discovery, actual screen-reader combinations, and CI Node 22/Python 3.12. Browser mocks do not prove Chrome or ChatGPT interoperability.
- No Cycle-03 paths, coordinator files, dependency installation, push, merge, PR, publication, or deployment occurred.
