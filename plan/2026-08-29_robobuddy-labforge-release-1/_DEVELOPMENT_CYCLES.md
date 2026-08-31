# RoboBuddy LabForge Release 1 Development Cycles

## Goal and Plan

Implement the WebMCP Repair Workbench through PR 3. The authoritative decisions and shared contracts are in `meta_plan.md`; the detailed source plan is `ROBOBUDDY_LABFORGE_CODEX_IMPLEMENTATION_PLAN_REFINED.md` at the repository root.

## Startup Protocol

A fresh session reads applicable local instructions; the control package at `C:/Users/EmilJivishov/Projects/RoboBuddy_IDE/plan/2026-08-29_robobuddy-labforge-release-1`; the source plan at `C:/Users/EmilJivishov/Projects/RoboBuddy_IDE/ROBOBUDDY_LABFORGE_CODEX_IMPLEMENTATION_PLAN_REFINED.md`; and its assigned `CONTINUATION_CYCLE_NN.md` before acting. It then verifies the current Git-backed implementation checkout, branch, status, remote `main`, and any in-flight cycle ownership.

The local planning source at `C:/Users/EmilJivishov/Projects/RoboBuddy_IDE` is not its own Git checkout. Do not create implementation branches there and do not use parent-workspace sync/migration commands without explicit authorization.

The planning package is coordinator-owned local state, not feature-branch content. A task writes implementation and repository documentation only in its assigned worktree, then returns its completion record to the coordinator for serial package/status updates.

## Shared Constraints

- Implement PR 0 through PR 3 only; LabForge-Bench and later research tracks remain out of scope.
- Preserve user-owned local Unitree G1/high-contrast work, current themes/Pages behavior, the five source-plant tasks, all pinned revisions, and existing validation strength.
- Preserve static GitHub Pages deployment and framework-free browser ES modules.
- Default Agent access Off; require trusted browser user activation and reject script-generated `.click()`; check access, registration epoch, workspace generation, and the execution abort signal inside handlers.
- Read only one immutable ready workspace snapshot. Loading/error workspaces return `WORKSPACE_NOT_READY`; profile/task/source changes invalidate asynchronous agent work as `WORKSPACE_CHANGED`.
- Keep output bounded and untrusted; expose no hidden validation or reference trajectory through summaries/run evidence.
- Keep `open_loop_event_replay`, source-plant, and kinematic-pose claims distinct. Never report browser-held G1 state as contact-plant evidence.
- Never let a WebMCP tool apply source in Release 1. Human Apply is visible, trusted-activation-gated, revision-checked, all-or-nothing, dirty, and unsaved.
- Do not stage, commit, push, publish, merge, or open a PR unless the user separately authorizes that action.

## Verification Ceiling

Use only the allowed checks in `meta_plan.md` and the current continuation file. Target changed modules/specs during coding; run the refined plan's full repository validation only at the designated PR gate. Record stronger or unavailable checks as unrun.

## Per-Cycle Fidelity Review Protocol

Every cycle must be critically reviewed and refined before completion. If review settings were not selected in `meta_plan.md`, pause after the deliverable and ask the user which model and effort to use. Offer the coding model at two currently supported reasoning levels higher, capped at its maximum, and allow the user to approve it for this cycle, apply it to remaining cycles, or specify another preference. Record only settings that actually performed the review.

## Parallel Execution Contract

After Cycle 01 completes, Cycles 02 and 03 may both become `ready` and run in separate Codex tasks and separate Git worktrees based on the exact same approved Cycle-01 commit. Each task prompt includes the absolute control-package path, assigned continuation, implementation-worktree path, branch, and recorded base SHA. Each task verifies `git rev-parse HEAD` against that SHA before editing and reports `git diff --name-only <base-sha>...HEAD` at handoff.

Cycle 02 exclusively owns:

- `src/agent/agent-facade.js`
- `src/agent/webmcp-tools.js`
- PR-1 Agent access UI/integration changes in `src/app-v2.js`, `index.html`, and the relevant CSS
- `tests/webmcp.spec.mjs`
- PR-1 status/compatibility documentation

Cycle 03 exclusively owns:

- `src/runtime/python-runtime-client.js`
- `src/runtime/python-worker.js`
- `src/runtime/python-shim.js`
- a new direct worker-core test/spec that imports only those modules

Cycle 03 must not edit `src/app-v2.js`, `index.html`, `src/python-runtime.js`, `src/source-simulator.js`, `src/agent/**`, shared CSS, package/test discovery configuration, or central status files. It is a reviewed preparation branch, not a merge-ready PR 2 by itself.

Reconciliation order is fixed:

1. Complete and place Cycle 02/PR 1 on the target base.
2. Rebase or replay Cycle 03's new-file-only commits onto that base.
3. Let one Cycle-04 task take the exclusive shared-file integration turn.
4. Run PR-2 verification and fidelity review after integration.

During parallel work, each task records its base SHA, branch/commit, owned-path diff, checks, model settings, findings, and residual risk in its own continuation record. A single coordinator updates `_CYCLE_STATUS.json` and this index serially; parallel branches must not race on central status files.

## Dependency Map

| Cycle | Outcome | Depends on | Initial status | Ownership |
|---|---|---|---|---|
| 01 | Approved Git-backed baseline and PR-0 characterization | None | ready | Baseline, docs, test discovery, characterization |
| 02 | Opt-in read-only WebMCP inspection / PR 1 | 01 | pending | Agent facade, access UI, read-only tools |
| 03 | Tested terminable worker core for later PR-2 integration | 01 | pending | New `src/runtime/**` modules and direct core spec only |
| 04 | Integrated cancellable runs and bounded evidence / PR 2 | 02, 03 | pending | Exclusive shared runtime/App/simulator/WebMCP integration |
| 05 | Human-controlled revision-locked staged edits / PR 3 | 04 | pending | Staging core, Patch panel, final Release-1 journey |

## Cycle Summaries

### Cycle 01 - Baseline reconciliation and characterization

Resolve the non-Git local-copy hazard without discarding concurrent work, establish an approved branchable baseline, then deliver PR 0's documentation, test discovery, execution-boundary characterization, starter-limit measurements, and compatibility probe. Stop at the PR-0 review/approval boundary.

### Cycle 02 - Opt-in read-only WebMCP inspection

Add trusted, session-scoped Agent access and exactly two bounded read-only tools over one immutable ready App snapshot. Prevent stale async profile loads and preserve source-plant/kinematic capability distinctions; stop before any agent-triggered run or edit support.

### Cycle 03 - Terminable Python worker core

In a separate worktree, implement and directly test the real worker/client/shim protocol and hard-termination recovery without touching Cycle 02 or shared integration files. Stop with a reviewed branch and commit ready for Cycle 04 reconciliation.

### Cycle 04 - Structured runs, evidence, and PR 2 integration

Reconcile Cycle 03 after PR 1, replace main-thread Pyodide execution, add the shared execution path, generation-bound operations, bounded evidence/state unions, immutable run identities, run store, and three run/evidence tools. Stop after the PR-2 gate and review.

### Cycle 05 - Revision-locked staged edits with human Apply

Add UTF-8 workspace hashing, unambiguous exact-match staging, proposal invalidation, safe Patch-panel rendering, and trusted all-or-nothing human Apply/Discard. Demonstrate the full instructional repair journey, complete the Release-1 gate, and stop before benchmark work.

## Completion Protocol

Verify the cycle's acceptance criteria within its ceiling, complete the fidelity-review/refinement gate, and record actual model settings, findings, refinements, changed assumptions, files changed, checks run, unrun checks, and remaining risk. Update only the relevant cycle and the smallest roadmap fields. Never rewrite another in-flight parallel cycle's record.
