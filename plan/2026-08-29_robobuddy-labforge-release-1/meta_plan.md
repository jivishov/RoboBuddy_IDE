# RoboBuddy LabForge Release 1 Meta Plan

## Outcome

Deliver PR 0 through PR 3 of the WebMCP Repair Workbench described in `ROBOBUDDY_LABFORGE_CODEX_IMPLEMENTATION_PLAN_REFINED.md`, without beginning LabForge-Bench.

The observable Release-1 journey is:

```text
inspect the active instructional workspace
-> run terminably
-> inspect bounded, call-site-linked evidence
-> stage a revision-locked exact-match edit
-> let the human review and apply it
-> rerun and report whether the first detected modeled fault remains
```

Success preserves the static GitHub Pages application, current instructional workspaces, pinned source/model/task revisions, source-plant validation, Unitree G1's kinematic-only boundary, presentation-only high-contrast scene aids, and direct human control over applying source changes.

## Verified Current State

Evidence was inspected on 2026-08-29. No project test suite was run while creating this planning package.

- The refined plan reviewed `jivishov/RoboBuddy_IDE@b6d29dfeffbb11830ac219a26417c09cebd98df4`. A read-only remote check found `main` at `8a83b5ea57840eb836e5ed3044db989f60c4409e`, with intervening commits `4e4a151` and `8a83b5e` affecting the Pages app, themes, layout, `src/app-v2.js`, `index.html`, `src/source-simulator.js`, and browser coverage. Cycle 01 must recheck this value because it may move again.
- `C:/Users/EmilJivishov/Projects/RoboBuddy_IDE` has no `.git` directory. Git resolves it to the parent `C:/Users/EmilJivishov/Projects` repository with prefix `RoboBuddy_IDE/`, but `git ls-files -- .` and scoped status show that this folder is not tracked there. It also has no entry in the parent `.workspace/projects.json` manifest.
- The local folder contains user-owned work beyond remote `main`. During the moving-source audit, at least these remote-tracked paths differed: `README.md`, `index.html`, `styles.css`, `theme-system.css`, `src/app-v2.js`, `src/canonical-rig.js`, `src/profiles.js`, `src/python-runtime.js`, `src/source-simulator.js`, `src/task-catalog.js`, `src/task-workspace.js`, `tests/browser-smoke.spec.mjs`, `tests/validate.mjs`, and `tests/validate_canonical_visuals.mjs`. Local-only evidence includes `ROBOBUDDY_LABFORGE_CODEX_IMPLEMENTATION_PLAN_REFINED.md`, `docs/unitree-g1-rig.md`, `licenses/unitree_ros-BSD-3-Clause.txt`, and `plan/2026-08-29_093041_unitree-g1-rig-integration-plan.md`. The set expanded while planning was underway, so Cycle 01 must take a fresh diff and must not overwrite it.
- The app remains a static, framework-free ES-module site. `index.html` loads CodeMirror 5.65.16, Three.js 0.180.0, and a main-page Pyodide 0.29.4 loader. `package.json` has only `test:browser`, and `playwright.config.mjs` discovers only `browser-smoke.spec.mjs`.
- `src/app-v2.js` owns workspace files, dirty state, task/profile selection, execution state, UI rendering, and persistence. `prepare()` executes the whole learner program through `PythonRuntime.compileWorkspace()` before `_applyEvent()` replays recorded actions and sleeps. `trajectoryLine()` and `actionLabel()` currently infer reference rows and labels from action order; agent evidence must stop doing that.
- Workspace loading is asynchronous. The constructor begins with `scenario = null`, `loadProfile()` mutates profile/task state before awaiting scenario and simulator setup, and `usesSourcePlant()` currently treats a missing scenario as source-plant. The agent boundary therefore needs an explicit loading/ready/error state and a monotonic workspace generation; it must never expose a half-loaded or stale profile/task/scenario combination.
- `src/python-runtime.js` initializes Pyodide on the main page. Its shim updates `_rb_state` and returns that state from `get_observation()` before source-plant replay, so this output is not post-physics telemetry.
- `src/source-simulator.js` owns the pinned `ScenarioV2Engine` path for OpenArm, SO-101, and LeKiwi. `applyAction()` performs the initial 20 ms plant advance and `advanceTime()` performs later sleep ticks. The module has no bounded agent state-summary API yet.
- The local Unitree G1 workspace uses a distinct `kinematic_pose` path without a source plant. Its browser-held joint state, fixed visual root, and no-contact boundary are intentional. Agent tools therefore require explicit `simulationMode`, `stateKind`, and `sourcePlantAvailable` labels instead of assuming every workspace has modeled contact evidence.
- The local high-contrast scene layer and toggle are presentation-only. They must not enter state evidence, collision claims, grading, or workspace revisions.
- Existing CI uses Node 22 and Python 3.12. The planning host reports Node 23.3.0, npm 11.7.0, Python 3.10.1, and Playwright 1.55.0; execution records must distinguish local results from CI results.
- Current official WebMCP material was rechecked on 2026-08-29. The [WebMCP imperative draft](https://github.com/webmachinelearning/webmcp/blob/main/index.bs) and [Chrome imperative-API documentation](https://developer.chrome.com/docs/ai/webmcp/imperative-api?hl=en) use `document.modelContext.registerTool(...)`, a registration signal for tool lifetime, and the per-execution `{ signal }` for call cancellation. Chrome's unregister behavior is version-dependent, so RoboBuddy must not use registration abort as its in-flight cancellation mechanism. [OpenAI's site-tools guidance](https://help.openai.com/en/articles/20001423-using-site-tools-in-the-chatgpt-desktop-app) says site tools run in the ChatGPT desktop built-in browser, not Chrome, and the [built-in-browser guidance](https://help.openai.com/en/articles/20001277-using-the-built-in-browser-in-the-chatgpt-desktop-app) describes separate browser state; Chrome Inspector and ChatGPT site-tools evidence are distinct manual lanes.

## Decisions and Constraints

- Treat the current local folder as preservation evidence, not as a branchable implementation checkout. Cycle 01 must use an independent Git-backed clone/worktree and reconcile the local delta only after the user approves the authoritative baseline. Do not enroll, migrate, replace, commit, push, or publish through the parent Projects repository without separate authorization.
- Treat this package at `C:/Users/EmilJivishov/Projects/RoboBuddy_IDE/plan/2026-08-29_robobuddy-labforge-release-1` and the refined source plan at `C:/Users/EmilJivishov/Projects/RoboBuddy_IDE/ROBOBUDDY_LABFORGE_CODEX_IMPLEMENTATION_PLAN_REFINED.md` as the local planning control plane. Independent implementation worktrees will not contain them automatically. Fresh-task prompts must provide these absolute paths; do not copy the package into feature branches or let parallel workers edit coordinator-owned status files.
- Implement only PR 0 through PR 3. Public-task sanitization, private evaluation, provider adapters, scene authoring, cross-play, closed-loop Python, and VLA control remain out of scope.
- Preserve a static GitHub Pages deployment. Do not add React, a bundler, a backend, provider keys, or a second grader.
- Preserve the pinned RoboBuddy_AI task/plant revision `75fe2669c0ab0b029986de424c69162071174df8`, canonical visual revision `66d18a029a0caeb6a6075e681dbd9ecd6b22affa`, LeRobot revision `7e241bd630a3719a56157a497ce5d08f244784f1`, and all current task/action/geometry validation unless a separately reviewed upstream change is explicitly authorized.
- Agent access is session-scoped `Off | Inspect | Repair`, defaults to `Off` on every load, and changes only from a trusted browser user activation (`event.isTrusted`) on the visible control. Pointer, keyboard, and assistive-technology activation remain valid; script-generated `.click()` is rejected. Content mode remains `instructional` throughout Release 1.
- WebMCP uses feature-detected top-level imperative registration. Aborting the registration controller unregisters the tool set; it is not relied on to stop an already executing call. Every handler checks its captured registration epoch, current access, and the per-execution abort signal before a sensitive read, before any state mutation, and before returning state derived from a potentially asynchronous operation. Expected domain failures return bounded structured results; unexpected failures do not expose stacks.
- `AgentFacade` reads one immutable application snapshot containing a monotonic internal workspace generation and `workspaceStatus: "loading" | "ready" | "error"`. Loading/error transitions return bounded `WORKSPACE_NOT_READY` results. Capability fields are derived only from a ready scenario; absence of a scenario is never interpreted as source-plant availability.
- `read_workspace` may return explicitly requested instructional starter source, including `trajectories.py`, only with clear `instructional` and reviewed-starter labels. Summary, run, and evidence tools return no reference trajectory or hidden validation structure.
- Worker termination is the reliable timeout/cancel fallback. Do not call browser Pyodide a secure sandbox or claim graceful `KeyboardInterrupt`.
- A WebMCP tool may stage edits only. Only a trusted browser user activation in the visible Patch panel may apply them, and Apply must not save to `localStorage`. Programmatic `.click()` cannot cross this boundary.
- Parallel execution is allowed only for Cycles 02 and 03, after Cycle 01. They must run in separate Git worktrees from the exact same recorded Cycle-01 base SHA and obey the exclusive ownership map in `_DEVELOPMENT_CYCLES.md`. No other cycle pair is parallel-ready.

## Implementation Changes

### Baseline and characterization

- Establish an approved Git-backed baseline that preserves current remote changes and explicitly reconciles the local Unitree/high-contrast work.
- Add `docs/labforge/IMPLEMENTATION_STATUS.md`, `FIDELITY_BOUNDARIES.md`, and `WEBMCP_COMPATIBILITY.md`.
- Make unit and browser-test discovery explicit, characterize the existing pre-replay Python boundary, measure current instructional event/sleep maxima, and add a development-only WebMCP compatibility probe without registering production tools.

### Read-only agent surface

- Add a narrow `AgentFacade` over explicit `App` methods and a top-level WebMCP registration module.
- Add one atomic App snapshot method plus a monotonic load generation. Set status to loading before profile/task transition work, ignore stale async completions, and expose data only after the matching scenario, files, simulator, profile, and task are ready.
- Implement bounded `describe_lab_task` and paginated `read_workspace` tools behind `Inspect` access.
- Add truthful capability labels for both source-plant and kinematic-pose workspaces while preserving the current UI, themes, high-contrast control, and laptop/mobile layouts.

### Terminable runs and bounded evidence

- Move Python compilation to a version-pinned worker client/shim with request IDs, hard timeout/cancel termination, resource limits, and fresh-worker recovery.
- Refactor UI and agent runs through one application execution path. Agent runs reset first and default to non-realtime replay; Step and Run-to-Cursor remain stateful human operations.
- Add bounded source-plant and kinematic-pose summary methods, per-action evidence phases, a five-run store, and `run_simulation`, `read_run_evidence`, and `reset_simulation` tools.
- Bind every agent run/reset to the ready workspace generation captured at call start. A profile/task/source transition cancels or rejects the active operation as `WORKSPACE_CHANGED`; completed run records retain their immutable profile/task/generation identity until normal eviction.
- Remove agent reliance on reference action labels and `trajectoryLine(actionIndex)`. Evidence identifies the recorded `send_action()` call site only.

### Revision-locked staging and human Apply

- Hash the exact stable workspace serialization as UTF-8 bytes and validate one to eight exact-match, non-overlapping edits to `main.py` or `trajectories.py`.
- Stage proposals in memory, invalidate them on any relevant access/task/profile/workspace/source transition, and render their textual diff safely in a new bottom-panel Patch tab.
- Define edit ranges as one-based inclusive lines over the editor's canonical text. The exact slice contains separators between selected lines but no invented terminal newline.
- Apply only after trusted user activation and a second validation of every edit. Compute all resulting files first, then commit them as one guarded transaction with no partial mutation. Suppress proposal invalidation only for that transaction's own editor callbacks, mark edited files dirty, leave them unsaved, focus the first changed range, and clear the proposal.

## Public Interfaces, State, and Data

- Tool set by access/release:
  - PR 1 Inspect: `describe_lab_task`, `read_workspace`.
  - PR 2 Inspect/Repair: add `run_simulation`, `read_run_evidence`, `reset_simulation`.
  - PR 3 Repair only: add `stage_workspace_edits`; there is no WebMCP Apply tool.
- Every task/run summary keeps `executionMode: "open_loop_event_replay"` and `hardwareValidated: false`, and additionally reports:
  - `simulationMode: "source_plant" | "kinematic_pose"`;
  - `stateKind: "modeled_source_plant" | "browser_kinematic_pose"`;
  - `sourcePlantAvailable: boolean`.
- For `kinematic_pose`, contact, collision, grade, and plant-fault evidence is unavailable. Runtime or action-validation errors may still be reported, but they must not be labeled plant faults.
- Workspace reads and operations use `workspaceStatus: "loading" | "ready" | "error"` internally and return structured `WORKSPACE_NOT_READY` or `WORKSPACE_CHANGED` domain results instead of content from a mixed transition state.
- `pythonShimOutput` is bounded stdout/stderr produced before replay. It is never presented as modeled state or telemetry.
- `firstDetectedFault` is the first observed Python-runtime or source-plant fault, not a causal/root-cause conclusion. Pair it with `faultDomain: "python_runtime" | "source_plant" | null`; never derive it by parsing rendered UI strings. Action validation uses the separate `validation_error` status and a bounded validation result, never a plant-fault label. A completed run is not automatically a successful task.
- Every retained run record includes its immutable `profileId`, `taskId`, workspace generation/revision when available, and capability union so evidence remains attributable after the current workspace changes.
- Workspace revisions are lowercase `sha256:` hashes of the recommended serialization encoded with `TextEncoder` UTF-8, without Unicode or newline normalization. Cursor, tab, dirty flags, timestamps, simulator state, theme, and high-contrast presentation state are excluded.
- Staged edits retain the refined plan's exact line-range/expected-text schema, file allowlist, overlap checks, and 50 KB total replacement limit. `start_line` and `end_line` are one-based and inclusive; all edits pass the second revision/text check before any file is mutated.

## Development Cycles

1. Baseline reconciliation and PR 0 characterization.
2. Opt-in read-only WebMCP inspection (PR 1).
3. Terminable Python worker core (parallel preparation for PR 2).
4. Structured runs, evidence, and PR 2 integration.
5. Revision-locked staged edits with human Apply (PR 3).

After Cycle 01, Cycles 02 and 03 may proceed concurrently in separate worktrees. Cycle 04 serially reconciles them after Cycle 02 is on the target base. Cycle 05 is sequential.

## LLM Execution and Per-Cycle Fidelity Review

- Coding model and effort: not selected in this planning session; every cycle records its actual task model and reasoning effort before implementation.
- Fidelity-review model and effort: ask at each cycle gate.
- Default when review settings are deferred: use the coding model two currently supported reasoning levels higher, capped at its maximum; let the user approve it for the current cycle, apply it to remaining cycles, or choose another supported setting.
- Completion rule: the selected settings must actually perform one critical fidelity review and one in-scope refinement pass before a cycle is marked `completed`.

## Verification Ceiling

Allowed during implementation:

- read-only Git/status/log/diff/hash checks needed to establish the baseline;
- `node --check` for changed JavaScript modules;
- changed Node built-in unit tests through `node --test`;
- the directly affected Playwright specification during a code-changing cycle;
- `git diff --check` in the independent Git checkout;
- the refined plan's existing repository validation commands once per PR gate, including the existing static/Python checks and `npm run test:unit` plus `npm run test:browser`;
- the specific `1366x768` and `390x740` layout checks, Chrome WebMCP Inspector check, and ChatGPT site-tools check at the PR/Release-1 gates where the user-selected refined plan requires them. Chrome proves browser API/Inspector compatibility; only the ChatGPT desktop built-in browser can prove ChatGPT site-tool discovery.

Do not add test dependencies or expand into full visual regression, generic accessibility, performance, hardware, deployment, penetration, dependency-security, or broad cyber-safety testing. Record unavailable manual site-tool checks and all heavier evidence as unrun.

Stop condition: collect the narrow evidence required by the current cycle, complete its fidelity review/refinement pass, and stop at its PR or approval boundary.

## Acceptance Criteria

- The approved baseline includes or explicitly preserves all user-owned local work and all current remote `main` changes; no implementation runs from the untracked parent-workspace copy.
- PR 0 through PR 3 satisfy their acceptance criteria in the refined plan and the cycle-specific capability refinements above.
- The five source-plant instructional tasks retain their pinned task/plant behavior, and Unitree G1 remains explicitly kinematic-only.
- Agent access defaults Off, output is bounded and labeled untrusted, cancellation terminates the worker, and no tool silently applies source.
- Tool calls never read a half-loaded workspace, script-generated clicks cannot enable access or apply edits, and a multi-file Apply cannot partially mutate source.
- The Release-1 repair journey is demonstrated on a source-plant instructional task and explicitly does not claim benchmark validity or hardware validation.
- Every cycle records actual model settings, checks run, unrun checks, review findings, refinements, changed assumptions, and residual risk.

## Assumptions and Open Decisions

- Baseline decision: the local Unitree/high-contrast delta is not on remote `main` and was changing during planning. Cycle 01 may inspect and package the choices, but coding must pause until the user identifies or approves the Git-backed authoritative baseline. This plan assumes the work is preserved, never discarded.
- Review settings are intentionally deferred because the coding model and effort for future tasks are unknown.
- Chrome WebMCP support and ChatGPT site-tool availability are separate. Record Chrome/browser version for the Inspector lane; record ChatGPT desktop app version, account/model availability, and built-in-browser result for the site-tools lane. Lack of availability limits the manual claim; it does not authorize bypassing access controls or substituting a mock result.

## Planning-package Fidelity Review - 2026-08-29

The package was reread against all current planning artifacts, the moving local source, the current remote `main` SHA, the refined source plan, and current primary WebMCP/OpenAI documentation. The first pass confirmed the five-cycle dependency shape and the Cycle-02/Cycle-03 ownership split. The focused refinement added explicit workspace readiness/generation semantics, trusted-activation enforcement, registration-versus-execution cancellation separation, immutable run identity, unambiguous edit/hash semantics, atomic Apply behavior, exact parallel-base proof, and distinct Chrome/ChatGPT manual lanes. Cycle count, dependency state, and `_CYCLE_STATUS.json` remain unchanged because no implementation cycle was executed.

## Unrun Checks / Residual Risk

- No project unit, browser, CI, manual WebMCP, mobile, or runtime tests were executed while writing this package.
- The concurrent local Unitree/high-contrast changes were inspected as source but not independently validated here and may continue to move before Cycle 01.
- Remote `main` may advance again; every execution cycle must verify its starting commit.
- Browser mocks cannot prove Chrome Inspector or ChatGPT site-tool interoperability. Chrome Inspector cannot prove ChatGPT site-tool discovery, and neither supports a claim of hostile-code sandboxing.
- No hardware, deployment, accessibility, performance, or generic security audit is authorized by this package.
