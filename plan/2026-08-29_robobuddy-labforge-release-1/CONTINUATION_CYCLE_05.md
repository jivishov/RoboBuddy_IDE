# Continuation Cycle 05 - Revision-locked Staged Edits with Human Apply

## Outcome

Deliver PR 3 and the complete Release-1 instructional repair journey: an agent may stage bounded exact-match edits, but only the human can review and apply them in the visible Patch panel.

## Preconditions and Evidence

- Depends on: Cycle 04.
- Start from the reviewed PR-2 base in a dedicated worktree/branch `feat/labforge-03-staged-edits`.
- Verify before editing:
  - Agent access, worker cancellation, run/evidence tools, capability labels, and current source/presentation paths are stable;
  - no other task owns shared `src/app-v2.js`, `index.html`, CSS, or `src/agent/**` files;
  - the current editor change, profile/task change, reset, import, and access-mode paths that must invalidate proposals.

## Implementation Boundary

In scope:

- Add lowercase SHA-256 workspace revisions over the refined plan's stable serialization encoded as exact UTF-8 bytes with `TextEncoder`; perform no Unicode or newline normalization and exclude UI/presentation/simulator metadata.
- Add exact-match line-range staging for one to eight non-overlapping edits, at most 50 KB total replacement, to `main.py` and `trajectories.py` only. `start_line` and `end_line` are one-based and inclusive over the editor's canonical text; the requested slice includes separators between selected lines but no invented terminal newline.
- Validate base revision, line ranges, exact expected text, overlap, null bytes, and protected files before computing in-memory proposed contents/diff.
- Register `stage_workspace_edits` only in Repair access. The tool changes Patch-panel state, not source.
- Invalidate proposals on human source edits, profile/task change, workspace reset, import, another proposal action, reload, or leaving Repair access.
- Add a Patch bottom tab showing proposal source, base revision, changed files, additions/deletions, safe text-only diff, Apply, Discard, and stale state.
- Apply only from a trusted browser user activation (`event.isTrusted`) on the visible control; pointer, keyboard, and assistive-technology activation remain valid, while script-generated `.click()` is rejected. Recheck the revision and every exact slice before mutation, compute all resulting file contents first, and commit them as one all-or-nothing transaction. Apply approved ranges bottom-up, suppress proposal invalidation only for this transaction's own editor callbacks, mark files dirty, do not save, focus the first changed range, and clear the proposal.
- Demonstrate stale rejection, restaging, human Apply, and rerun on a source-plant instructional fixture/task. Explicitly state that it is not a benchmark-valid no-answer-leak task.

Preserve or leave for later:

- No WebMCP Apply tool, arbitrary file editing, auto-save, benchmark direct-apply adapter, provider integration, or benchmark mode.
- No edits to `robot_config.py`, `workcell.py`, HTML/CSS/JS through the staging tool, pinned revisions, profiles, tasks, limits, or source-plant validation.
- Preserve G1 kinematic-only behavior, high-contrast presentation state, themes, diagnostics layout, and all human run controls.

## Likely Touchpoints

- `src/agent/workspace-revision.js`
- `src/agent/edit-staging.js`
- `src/agent/agent-facade.js`
- `src/agent/webmcp-tools.js`
- `src/app-v2.js`
- `index.html`
- the smallest relevant CSS file(s)
- unit tests for revision/staging/diff logic
- `tests/webmcp.spec.mjs`
- a focused Release-1 repair-flow Playwright spec
- `docs/labforge/IMPLEMENTATION_STATUS.md`

## Acceptance Criteria

- Workspace revision ignores tab/cursor/dirty/theme/high-contrast/simulator changes and changes after source/profile/task content changes.
- Revision hashing is deterministic for non-ASCII content and exact final-newline differences. Staging accepts valid proposals and rejects stale revisions, text mismatches, overlap, invalid ranges, oversize replacements, null bytes, and protected files.
- Source remains unchanged before Apply and after Discard; stale proposals cannot apply.
- Trusted human Apply changes only approved ranges, leaves files dirty/unsaved, and safely renders untrusted diff content without `innerHTML`. A DOM `.click()` cannot apply; a real pointer/keyboard activation can.
- If any second-pass edit check fails, no file changes. The Apply transaction does not invalidate itself mid-commit and does not suppress later genuine human-edit invalidation.
- `stage_workspace_edits` is absent in Inspect access and stale epochs cannot stage.
- The complete repair journey reruns and truthfully reports source-plant instructional/open-loop/hardware-pending limits.
- PR-3/Release-1 deterministic, manual-availability, security/fidelity, layout, and preservation gates are recorded.

## Verification Ceiling

Allowed checks:

- `node --check` for changed/new modules;
- targeted unit tests for UTF-8 revision serialization, non-ASCII/final-newline cases, inclusive line slices, exact edits, overlap, all-or-nothing multi-file apply, transaction-scoped invalidation, and diff rendering;
- focused WebMCP/repair-flow Playwright specs and affected 1366x768/390x740 layout assertions;
- the refined plan's complete repository validation once at the PR-3/Release-1 gate;
- Chrome Inspector repair journey and the separate ChatGPT desktop built-in-browser site-tools repair journey when each feature is available;
- focused source-diff review for leakage/unsupported claims and `git diff --check`.

Do not run:

- benchmark/private evaluator, live provider, closed-loop Python, hardware, broad browser, visual-regression, accessibility, performance, deployment, penetration, dependency-security, or generic security suites.

Stop condition: PR-3 and Release-1 criteria have allowed evidence, one fidelity review/refinement pass is complete, and no benchmark code has begun.

## Unrun Checks / Residual Risk

- If Chrome Inspector or ChatGPT built-in-browser site tools are unavailable, the corresponding natural-language staging/apply lane remains unproven and must be recorded separately. Chrome cannot substitute for the ChatGPT lane.
- The acceptance journey is instructional and contains reviewed starter/reference content; it is not benchmark evidence.
- Browser controls do not prove hostile-code sandboxing, hardware safety, or closed-loop control.

## Critical Fidelity Review and Refinement Gate

- Coding model and effort: record actual settings at cycle start.
- Review model and effort: ask the user after implementation and allowed checks.
- Default proposal when unspecified: use the coding model two supported reasoning levels higher, capped at its current maximum.
- Review target: silent/stale/partial application risk, trusted activation, UTF-8/final-newline hash determinism, inclusive slice semantics, exact second-pass checks, transaction-scoped invalidation, protected paths, safe diff rendering, no auto-save/WebMCP Apply, access/epoch/workspace-generation races, distinct Chrome/ChatGPT claims, leakage/unsupported claims, and preservation of runtime/G1/layout behavior.
- Refinement: correct confirmed in-scope gaps and rerun only affected allowed checks.
- Stop after one review and one refinement pass unless the user requests another.

Do not mark completed until the selected settings actually perform the review.

## Status Handoff

Record the PR-3 branch/commit, actual coding/review settings, fidelity findings/refinements, changed files, exact checks and manual journeys, unrun evidence, and residual risk. Mark Cycle 05 completed only when the Release-1 gate is satisfied; do not create benchmark cycles automatically.

## Completion Record - 2026-08-29

- Release branch/commit: `feat/labforge-03-staged-edits` at `fdd587522e3dc0ab2a6ce2610a74ee633c9b3b95`, directly based on reviewed PR-2 commit `17d086abce09eb01a0b7c15d26e9019d966df52b`.
- Models: implementation `gpt-5.6-sol` at `medium`; fidelity review `gpt-5.6-sol` at `xhigh`.
- Fidelity refinements: use deterministic code-unit filename ordering; reject an existing null byte anywhere in a staged target; require trusted activation to originate from the currently visible Patch controls; remove a machine-local path from implementation documentation.
- Verification: complete Release-1 unit gate 19/19 and Chromium gate 28/28; review-affected revision/staging unit slice 6/6 and repair-flow browser slice 2/2; five affected syntax checks, static/canonical/pinned-task/Python/OpenArm checks, 1366x768 and 390x740 Patch reachability, and `git diff --check` passed.
- The final exact-path audit found no credentials, real `.env`, machine-local paths, runtime attachment values, vendor `file_id`, provider implementation, benchmark implementation/evidence, or coordinator-package changes. The final feature worktree is clean and contains no temporary test artifacts.
- Unrun/residual: Chrome Inspector and ChatGPT desktop built-in-browser journeys were unavailable and remain separate unproven manual lanes; deployed Pages, CI-host execution, benchmark/private evaluator, live providers, closed-loop Python, hardware, broad browser, visual regression, accessibility, performance, penetration, dependency-security, and generic security suites were not run.
- The source-plant repair journey is instructional/open-loop and is not benchmark or hardware evidence. No dependency installation, push, merge, PR, publication, or deployment occurred.
