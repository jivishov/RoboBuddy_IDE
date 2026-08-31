# RoboBuddy LabForge
## Refined Codex Implementation Plan — Fidelity-First Revision 2

**Repository:** `jivishov/RoboBuddy_IDE`  
**Reviewed branch:** `main`  
**Reviewed commit:** `b6d29dfeffbb11830ac219a26417c09cebd98df4`  
**Reviewed upstream task/plant revision:** `jivishov/RoboBuddy_AI@75fe2669c0ab0b029986de424c69162071174df8`  
**Prepared:** 2026-08-29  
**Supersedes:** `ROBOBUDDY_LABFORGE_CODEX_IMPLEMENTATION_PLAN.md`

---

# 1. Executive decision

Implement RoboBuddy LabForge in two clearly separated programs:

1. **WebMCP Repair Workbench**
   - A human-visible, browser-local collaboration layer over the existing IDE.
   - The agent can inspect code, run the current simulation, inspect bounded evidence, and stage a source edit.
   - The human—not the WebMCP tool—applies the staged edit in the first release.

2. **LabForge-Bench**
   - A provider-neutral, reproducible evaluation system.
   - It must not depend on WebMCP, a live page, or a particular browser agent.
   - It is implemented only after a no-answer-leak public task contract and a private evaluator boundary are proven.

The first Codex handoff is limited to **PR 0 through PR 3**:

```text
PR 0  Baseline, characterization, and compatibility checks
PR 1  Explicitly enabled, read-only WebMCP tools
PR 2  Cancellable Python worker and structured simulation evidence
PR 3  Revision-locked staged edits with human Apply
```

Do not begin benchmark task sanitization, provider adapters, asset authoring, cross-play, closed-loop Python, or VLA control during this first handoff.

---

# 2. Why this revision replaces the earlier plan

The previous plan had the correct product direction but was too broad and too prescriptive before proving one end-to-end workflow. It also contained several fidelity and sequencing errors.

## 2.1 Critical corrections

| Previous plan issue | Refined decision |
|---|---|
| Required a large service architecture before producing user value | Build a thin agent façade over the existing `App` and simulator first; extract services only when repeated use proves the boundary |
| Put renderer-independent plant extraction before WebMCP | Defer it until the private evaluator needs it; the current browser suite already drives `ScenarioV2Engine` directly without using the rendered UI |
| Defaulted WebMCP access on when supported | Default **Off** on every page load; require explicit, session-scoped user enablement |
| Gave the agent an `apply_workspace_patch` tool | The first release provides only `stage_workspace_edits`; the human applies or discards the staged diff in the UI |
| Used whole-file replacement as the primary patch format | Use revision-locked, exact-match text edits against line ranges; reject stale or mismatched edits |
| Mixed agent permissions with instructional/benchmark page modes | Keep two orthogonal concepts: **Agent access** (`Off`, `Inspect`, `Repair`) and **Content mode** (`Instructional`, later `Benchmark`) |
| Claimed the system could diagnose the “first causal failure” | Report the **first detected plant/runtime fault**; causal/root-cause attribution is a model-evaluation metric, not a simulator fact |
| Treated Python stdout/observation as physical evidence | Label it `python_shim_output`; the current shim executes before plant replay and does not receive post-physics observations |
| Treated the recorded source line as the numeric trajectory-definition line | The shim records the `send_action()` **call site**; it does not prove where the command dictionary or incorrect number was defined |
| Made Release 1 demonstrate a no-reference benchmark task | Release 1 demonstrates repair on the current instructional task; no benchmark-validity claim is made until the public task contract exists |
| Planned to strip hidden grading from public scenarios | Add a compatibility gate: the pinned ScenarioV2 validator currently requires hidden grading for portable-Python scenarios |
| Called the browser evaluator “sealed” without a threat model | Call it a **private evaluator**; document that browser Pyodide is not a hardened hostile-code sandbox |
| Treated hidden perturbations as adaptive robustness | Separate **static robustness** of one fixed open-loop program from **adaptive agent robustness** with a fresh agent run |
| Proposed broad “3D asset generation” too early | Later implement **parametric scene authoring** from an audited template inventory; unrestricted mesh/code generation is out of scope |
| Required the full test suite after every small milestone | Run targeted tests per commit; run the complete suite at each PR gate and before merge |

---

# 3. Verified current constraints

Codex must re-check these before editing because `main` may move after this plan is written.

## 3.1 Current application constraints

The reviewed application:

- is a static GitHub Pages-compatible site;
- has no build system or application framework;
- uses CodeMirror 5, Three.js, Pyodide 0.29.4, and browser ES modules;
- keeps workspace state inside `App` in `src/app-v2.js`;
- loads the source task/plant from a commit-pinned RoboBuddy_AI URL;
- exposes full reviewed reference actions in the instructional `trajectories.py`;
- runs learner Python in the main browser thread;
- records Python API events first, then replays them through the source plant;
- records the caller of `send_action()` as a call site, not the provenance line of every value inside the action dictionary;
- has no true post-physics `get_observation()` feedback into the same Python process;
- has a Playwright configuration that currently matches only `browser-smoke.spec.mjs`.

## 3.2 Current source-plant constraints

The pinned `ScenarioV2Engine`:

- creates a `PortableRobotPlant` for SO-101, LeKiwi, and OpenArm;
- provides authoritative `snapshot().grade`;
- validates physical overlaps and task definitions;
- retains hidden grading requirements inside the full scenario;
- allows client definitions to omit `validation` through its current `validationAvailable` mechanism;
- still requires non-empty `hiddenGradingRequirements` for portable-Python scenarios;
- uses full scenario geometry and task semantics to evaluate outcomes.

## 3.3 Current WebMCP constraints

Implement against the current imperative API:

```javascript
document.modelContext.registerTool(...)
```

Requirements for RoboBuddy:

- register tools with JavaScript in the **top-level page** for compatibility with ChatGPT site tools;
- use feature detection;
- use registration `AbortSignal` to manage tool availability;
- use the per-execution `{ signal }` argument for cancellation;
- do not assume unregistering a tool will safely cancel an in-flight operation on every supported browser;
- perform a permission/mode check inside every handler immediately before any state change;
- use `readOnlyHint` accurately;
- use `untrustedContentHint` for code, task text, stdout, stderr, and other user/external content;
- keep each tool name below 30 characters;
- keep ordinary individual tool outputs near or below 1.5 KB;
- paginate code and evidence rather than returning large payloads.

## 3.4 Current Pyodide cancellation constraint

Pyodide can run in a Web Worker. Graceful Python interruption requires a `SharedArrayBuffer` and appropriate cross-origin isolation headers. Do not assume GitHub Pages supplies those headers.

The reliable MVP cancellation mechanism is:

```text
terminate the worker → reject the active run → create a fresh worker
```

Do not claim that termination is equivalent to graceful Python `KeyboardInterrupt`.

---

# 4. Product boundaries

## 4.1 What Release 1 may claim

Release 1 may claim:

- an agent can inspect the active RoboBuddy instructional workspace;
- an agent can run or reset the modeled simulation;
- an agent can retrieve bounded, call-site-linked modeled evidence;
- an agent can stage a revision-locked source edit;
- the human can inspect and apply the staged edit;
- a rerun can verify whether the modeled fault remains.

## 4.2 What Release 1 must not claim

Release 1 must not claim:

- benchmark validity;
- hidden-test evaluation;
- first-principles root-cause diagnosis;
- closed-loop Python control;
- hardware validation;
- safe physical deployment;
- measured force, torque, current, tactile, fluid, powder, heating, or reaction behavior;
- a hardware-calibrated digital twin;
- that learner action N semantically equals reference action N;
- that Python `get_observation()` reflects the replayed physical plant.

## 4.3 Terminology

Use these exact distinctions:

| Term | Meaning |
|---|---|
| `firstDetectedFault` | First runtime or source-plant fault observed during this run |
| `pythonShimOutput` | Python stdout/stderr produced before source-plant event replay |
| `sendActionCallSite` | File and line where `robot.send_action(...)` was invoked; not necessarily where an action constant or incorrect value was defined |
| `modeledState` | State returned by the pinned source plant |
| `open_loop_event_replay` | Current Python execution mode |
| `instructional workspace` | Current workspace containing reviewed starter/reference actions |
| `benchmark workspace` | Later unsolved workspace generated without reference trajectory |
| `static robustness` | The same fixed program evaluated under bounded variants |
| `adaptive robustness` | A fresh agent is allowed to inspect and revise for each variant |
| `private evaluator` | Evaluator whose cases are not provided to the agent; not automatically a hardened sandbox |

---

# 5. Implementation principles

1. **Prove the vertical slice before refactoring broadly.**
2. **Reuse existing application logic.** WebMCP handlers must not click buttons or scrape the DOM.
3. **Preserve current instructional behavior.**
4. **Add no-answer-leak benchmark mode separately.**
5. **Treat every browser/tool result as untrusted content.**
6. **Keep agent editing human-reviewable.**
7. **Use exact revisions and exact-match edits.**
8. **Make every claim observable in a deterministic test.**
9. **Do not weaken the source plant, geometry, action envelopes, or task validation.**
10. **Do not create placeholder architecture files.** Add a module only when a PR uses it.
11. **No framework migration.** Do not introduce React, Vite, Webpack, or a backend for PR 0–3.
12. **No provider API keys or model adapters in the browser.**
13. **Do not modify RoboBuddy_AI inline.** A required upstream change must be made and reviewed in that repository, then pinned by commit.

---

# 6. Branch and PR strategy

Before starting:

```bash
git fetch origin
git checkout main
git pull --ff-only
git rev-parse HEAD
```

If `HEAD` is no longer:

```text
b6d29dfeffbb11830ac219a26417c09cebd98df4
```

Codex must:

1. inspect all intervening commits;
2. update the baseline record;
3. determine whether the plan’s assumptions still hold;
4. stop only if there is a material architectural conflict;
5. otherwise continue from current `main`.

Use one branch per PR:

```text
feat/labforge-00-baseline
feat/labforge-01-webmcp-readonly
feat/labforge-02-cancellable-runs
feat/labforge-03-staged-edits
```

Do not accumulate all work in one long-lived branch.

---

# 7. PR 0 — Baseline, characterization, and compatibility

## Goal

Create a trustworthy baseline and tests around current behavior before refactoring.

## 7.1 Tasks

### A. Record the exact baseline

Create:

```text
docs/labforge/IMPLEMENTATION_STATUS.md
docs/labforge/FIDELITY_BOUNDARIES.md
docs/labforge/WEBMCP_COMPATIBILITY.md
```

Record:

- IDE commit;
- RoboBuddy_AI task/plant commit;
- canonical visual revision;
- LeRobot revision;
- Pyodide version;
- CodeMirror version;
- Three.js version;
- Playwright version;
- Node version used by CI;
- Python version used by CI;
- exact commands and results.

### B. Make browser-test discovery explicit

The current Playwright configuration matches only one file. Change it deliberately to one of:

```javascript
testMatch: ["browser-smoke.spec.mjs", "webmcp.spec.mjs", "runtime-safety.spec.mjs"]
```

or a controlled pattern such as:

```javascript
testMatch: /.*\.spec\.mjs/
```

Do not silently rely on new test files being discovered.

Add scripts:

```json
{
  "scripts": {
    "test:browser": "playwright test",
    "test:smoke": "playwright test tests/browser-smoke.spec.mjs",
    "test:unit": "node --test tests/unit/*.test.mjs",
    "test": "npm run test:unit && npm run test:browser"
  }
}
```

Use Node’s built-in test runner. Do not add a unit-test framework.

### C. Characterize the current execution boundary

Add deterministic tests proving:

1. Python is executed before plant replay.
2. `get_observation()` in the current shim reflects accumulated command state.
3. a `send_action` event carries its invocation call-site file and line, not guaranteed value-definition provenance;
4. `SourceRobotSimulator.applyAction()` advances the plant by one 20 ms step;
5. a following `sleep` advances the plant independently;
6. the current UI catches and displays plant faults;
7. the current task workspace contains reference actions.

These tests document current limitations; they are not endorsements.

### D. Add a WebMCP compatibility probe

Add a development-only diagnostics function that reports:

```json
{
  "modelContextAvailable": true,
  "registerToolAvailable": true,
  "secureContext": true,
  "topLevel": true
}
```

Do not register production tools in PR 0.

### E. Add a manual test matrix

Document three separate checks:

1. **Chrome local development**
   - Enable `chrome://flags/#enable-webmcp-testing`.
   - Use the Model Context Tool Inspector.
2. **Deployed GitHub Pages**
   - Verify secure context and feature detection.
   - Add an origin-trial token only when a valid token for the deployed origin is obtained.
3. **ChatGPT desktop built-in browser**
   - Verify top-level imperative tools appear under Site tools.
   - Record app version, selected model, and rollout availability.

Mock Playwright tests do not replace these checks.

## 7.2 Acceptance criteria

- Existing CI and browser smoke tests pass.
- The new unit-test command succeeds even if no functional WebMCP work exists yet.
- Current execution limitations are covered by tests.
- No UI or simulator behavior changes.
- No new production tool is registered.
- `IMPLEMENTATION_STATUS.md` contains exact test commands and outcomes.

## 7.3 Suggested commit

```text
test/docs: characterize RoboBuddy before WebMCP changes
```

---

# 8. PR 1 — Explicitly enabled read-only WebMCP

## Goal

Prove that an agent can inspect the current live IDE through a small, safe tool surface without a broad application rewrite.

## 8.1 Minimal architecture

Add only:

```text
src/agent/agent-facade.js
src/agent/webmcp-tools.js
tests/webmcp.spec.mjs
```

Modify `app-v2.js` so the app instance is retained:

```javascript
const app = new App();
const agentFacade = new AgentFacade(app);
registerWebMcpTools(agentFacade);
```

Do not expose `app` globally in production. A test-only hook may be enabled through `?ci=1`.

### AgentFacade responsibility

`AgentFacade` may call explicit methods on `App`; it must not:

- query UI text to infer state;
- click buttons;
- dispatch synthetic keyboard events;
- parse CodeMirror DOM;
- read private variables through undocumented globals.

Add narrow methods to `App` only as needed:

```javascript
getAgentTaskSummary()
readAgentWorkspacePage(...)
getAgentAccess()
setAgentAccess(...)
```

Do not extract a general `WorkspaceService` yet.

## 8.2 Agent access control

Add a compact UI control:

```text
Agent access: Off | Inspect | Repair
```

In PR 1:

- `Off` and `Inspect` are functional;
- `Repair` may be visible but disabled with “available after staged-edit support,” or omitted until PR 3.

Rules:

- default `Off` on every page load;
- preference is session-scoped, not persisted in `localStorage`;
- enabling requires a direct human click;
- disabling unregisters tools;
- navigation/reload returns to `Off`.

This control is separate from the later content mode.

## 8.3 Tool-registration safety

Maintain:

```javascript
let registrationEpoch = 0;
let registrationController = null;
```

On access change:

1. increment epoch;
2. abort prior registration controller;
3. register the new tool set with the new controller;
4. capture the epoch in each handler.

Every handler must check:

```javascript
signal.throwIfAborted?.();
facade.assertAccess(requiredLevel, capturedEpoch);
```

immediately before reading sensitive workspace content and again before any state-changing action in later PRs.

Do not use `exposedTo` in the MVP.

## 8.4 PR 1 tool set

### Tool 1 — `describe_lab_task`

Read-only.

Purpose:

- summarize the active robot and instructional task;
- identify accessible files;
- state execution and fidelity limitations;
- return current workspace revision only after PR 3 adds revision hashing.

Input:

```json
{
  "type": "object",
  "properties": {},
  "additionalProperties": false
}
```

Return no reference action values.

Return at most approximately 1.5 KB:

```json
{
  "taskId": "...",
  "title": "...",
  "robot": "...",
  "brief": "...",
  "files": ["main.py", "trajectories.py", "robot_config.py", "workcell.py"],
  "writableInRepair": ["main.py", "trajectories.py"],
  "contentMode": "instructional",
  "reviewedStarterPresent": true,
  "executionMode": "open_loop_event_replay",
  "hardwareValidated": false,
  "limitations": ["..."]
}
```

Annotations:

```javascript
{
  readOnlyHint: true,
  untrustedContentHint: true
}
```

### Tool 2 — `read_workspace`

Read-only.

Input:

```json
{
  "type": "object",
  "properties": {
    "file": {
      "type": "string",
      "enum": ["main.py", "trajectories.py", "robot_config.py", "workcell.py"]
    },
    "start_line": {
      "type": "integer",
      "minimum": 1
    }
  },
  "required": ["file"],
  "additionalProperties": false
}
```

Behavior:

- return line-numbered source;
- explicitly label the content mode as `instructional`;
- indicate that `trajectories.py` may contain reviewed starter/reference content in instructional mode;
- default `start_line` to 1;
- stop before 1,200 output characters or 30 lines, whichever comes first;
- return `next_line` when more content exists;
- return current file length;
- never return all files in one call.

Output:

```json
{
  "file": "main.py",
  "contentMode": "instructional",
  "reviewedStarterContent": false,
  "startLine": 1,
  "endLine": 22,
  "content": "1: ...\n2: ...",
  "nextLine": null,
  "truncated": false
}
```

Annotations:

```javascript
{
  readOnlyHint: true,
  untrustedContentHint: true
}
```

## 8.5 Error behavior

Return a fulfilled, structured result for expected domain errors:

```json
{
  "ok": false,
  "error": {
    "code": "AGENT_ACCESS_DISABLED",
    "message": "Enable Inspect access in RoboBuddy before using this tool.",
    "retryable": true
  }
}
```

Throw only for unexpected programming errors. Do not expose stack traces.

## 8.6 UI requirements

- Show a subtle indicator when site tools are enabled.
- Do not add a modal or persistent banner.
- Do not expose code merely because WebMCP is available.
- Keep the existing 1366×768 and 390×740 layouts usable.

## 8.7 Automated tests

Mock `document.modelContext` with `page.addInitScript()` before app load.

Test:

1. default access is Off;
2. no tools are registered while Off;
3. human activation registers exactly two tools;
4. tools unregister on Off;
5. task output is bounded and contains fidelity labels;
6. source output paginates;
7. invalid files and line numbers are rejected;
8. handler rejects a stale registration epoch;
9. unsupported browsers load normally;
10. no task/source content is returned while Off.

## 8.8 Manual tests

Run the documented Chrome Inspector and ChatGPT desktop checks.

## 8.9 Acceptance criteria

- Read-only tools work against unsaved editor state.
- No tool can modify source or simulator state.
- Tool output stays bounded.
- Current IDE behavior and tests remain intact.
- Site tools are explicitly user-enabled for the current session.

## 8.10 Suggested commit

```text
feat: add opt-in read-only WebMCP tools
```

---

# 9. PR 2 — Cancellable Python and structured simulation evidence

## Goal

Allow an agent to run the current instructional program without freezing the page and retrieve trustworthy, bounded evidence from the modeled plant.

## 9.1 Worker migration must precede agent-run access

Do not expose `run_simulation` until generated or edited Python can be terminated reliably.

Add:

```text
src/runtime/python-runtime-client.js
src/runtime/python-worker.js
src/runtime/python-shim.js
tests/runtime-safety.spec.mjs
```

After migration, keep `src/python-runtime.js` only as a compatibility re-export or remove it after all imports are updated.

Remove the main-page Pyodide loader from `index.html` once the worker is proven.

## 9.2 Worker URL and GitHub Pages compatibility

Create the worker with a URL relative to the current module:

```javascript
new Worker(
  new URL("./python-worker.js", import.meta.url),
  { type: "module" }
);
```

Inside the worker, use an exact pinned Pyodide 0.29.4 URL. If the module-worker CDN path fails in the target browsers, use a classic worker with `importScripts`, but keep the version pinned and document the chosen mechanism.

Test under the repository subpath used by GitHub Pages; do not assume deployment at `/`.

## 9.3 Worker protocol

Only accept these message types:

```text
INIT
RUN
CANCEL
DISPOSE
```

Each request has a unique `requestId`.

Ignore malformed or unknown worker messages.

Return:

```text
READY
RUN_RESULT
RUN_ERROR
```

Do not send scenario definitions, hidden grading, simulator state, local storage, or DOM data to the Python worker.

## 9.4 Reliable cancellation

For each run:

- establish a wall-clock timeout;
- listen to the WebMCP execution signal;
- listen to the Stop button;
- on timeout/cancel, terminate the worker;
- reject the run with `cancelled` or `timeout`;
- create a fresh worker for the next run.

Do not depend on `SharedArrayBuffer`.

## 9.5 Runtime limits

Use named constants and cover them in tests. Do not choose event and simulated-sleep caps without measuring the five current pinned starters first. PR 0 must calculate:

```text
MAX_CURRENT_STARTER_EVENTS
MAX_CURRENT_STARTER_REQUESTED_SLEEP_S
```

Instructional limits must exceed those maxima with a documented margin so the current IDE does not regress. Benchmark limits are defined later per task. A reasonable starting shape is:

```text
MAX_SOURCE_BYTES       200,000 total
MAX_EVENTS             max(1,000, 2 * MAX_CURRENT_STARTER_EVENTS)
MAX_REQUESTED_SLEEP_S  max(180, 2 * MAX_CURRENT_STARTER_REQUESTED_SLEEP_S)
MAX_SINGLE_SLEEP_S     10
WORKER_INIT_TIMEOUT_MS 90,000
RUN_TIMEOUT_MS          30,000
MAX_STDOUT_CHARS         8,000
MAX_STDERR_CHARS         8,000
```

The worker-initialization timeout is separate from the run timeout. Do not start the run timeout while Pyodide is still downloading or initializing.

These are browser protection limits, not physical fidelity limits.

The Python shim should stop recording and raise a clear exception when the event or sleep budget is exceeded. A Python infinite loop that records no events is stopped by worker termination.

## 9.6 Security wording

Do not call browser Pyodide a secure sandbox.

Document:

- it runs in a worker;
- it is isolated from the main DOM by the worker boundary;
- it may still have browser/WebAssembly capabilities;
- evaluator security requires additional isolation and network controls;
- no private evaluator data may enter this worker.

## 9.7 Structured run result

Refactor the existing run path minimally so UI buttons and agent calls share one internal function.

Preferred shape:

```javascript
async executeSimulationRun({
  mode = "full",
  throughAction = null,
  realtime = true,
  signal = null,
  origin = "human"
})
```

The existing `run()`, `step()`, and `runToCursor()` may delegate to it.

For deterministic agent use, `run_simulation` always resets to the task initial state before replay. Its `realtime` default is `false`; the human Run button may preserve the current real-time presentation behavior. Step and Run-to-Cursor remain explicitly stateful UI operations and are not silently conflated with a fresh agent run.

Return:

```json
{
  "schema": "robobuddy.run-result.v1",
  "runId": "run_...",
  "status": "completed",
  "profileId": "so101",
  "taskId": "...",
  "executionMode": "open_loop_event_replay",
  "closedLoopObservation": false,
  "hardwareValidated": false,
  "preparedActions": 12,
  "completedActions": 12,
  "firstDetectedFault": null,
  "pythonShimOutput": {
    "stdoutTruncated": false,
    "stderrTruncated": false
  },
  "finalModeledStateAvailable": true
}
```

Allowed status values:

```text
completed
fault
python_error
validation_error
cancelled
timeout
```

## 9.8 Evidence capture

Do not extract `SourcePlantSession` in this PR. Add a bounded state-summary method to the existing simulator:

```javascript
getModeledStateSummary()
```

It may use `engine.snapshot()` but must return only JSON-serializable bounded data:

```json
{
  "simulationClockS": 1.42,
  "jointState": {},
  "rootPose": {},
  "objects": [
    {
      "id": "receiver",
      "worldPositionMm": [0, 0, 0],
      "attachedTo": "",
      "releasedUnsupported": false
    }
  ],
  "fault": {
    "code": "...",
    "message": "..."
  }
}
```

Do not return the full scenario, validation cases, reference actions, or full grade.

### Evidence phases

For every learner `send_action`, capture:

```text
beforeCommand
afterInitialPlantTick
afterFollowingSleep
terminal
```

Rules:

- `afterFollowingSleep` is null when no sleep occurs before the next action or program end;
- multiple consecutive sleeps may be accumulated, but the field must record total simulated seconds;
- `terminal` records state at next action, program end, cancellation, timeout, or fault;
- a fault record includes the last valid modeled state and collision witness when available;
- `callSite` is the learner file and line where `send_action()` was invoked;
- it must not be described as the definition/provenance line of the command dictionary;
- label is always `Learner action N` in PR 2.

Example:

```json
{
  "actionIndex": 3,
  "callSite": {
    "file": "main.py",
    "line": 17
  },
  "command": {},
  "beforeCommand": {},
  "afterInitialPlantTick": {},
  "afterFollowingSleep": {
    "requestedSeconds": 0.25,
    "state": {}
  },
  "terminal": {
    "reason": "next_action",
    "state": {}
  },
  "fault": null,
  "label": "Learner action 3"
}
```

Do not map by index to the reference action label. Also remove or disable the current `trajectoryLine(actionIndex)` highlighting path for agent and benchmark evidence because it infers a `trajectories.py` row from execution order. Highlight only the recorded `send_action()` call site unless future runtime instrumentation provides explicit value provenance.

## 9.9 Run-store bounds

Keep only:

```text
MAX_RETAINED_RUNS = 5
MAX_ACTIONS_PER_RUN = 1000
```

Truncate stdout/stderr and large collision witnesses.

Long evidence is read through pagination.

## 9.10 PR 2 WebMCP tools

When Agent access is `Inspect` or `Repair`, add:

### `run_simulation`

Input:

```json
{
  "type": "object",
  "properties": {
    "mode": {
      "type": "string",
      "enum": ["full", "until_fault", "through_action"]
    },
    "through_action": {
      "type": "integer",
      "minimum": 1
    },
    "realtime": {
      "type": "boolean"
    }
  },
  "required": ["mode"],
  "additionalProperties": false
}
```

Validation:

- `through_action` is required only for `through_action`;
- it is forbidden for other modes;
- the tool resets the simulator before every run;
- omitted `realtime` means `false`.

Return only a compact summary and `runId`.

### `read_run_evidence`

Input:

```json
{
  "type": "object",
  "properties": {
    "run_id": { "type": "string" },
    "view": {
      "type": "string",
      "enum": ["summary", "fault", "action", "final"]
    },
    "action_index": {
      "type": "integer",
      "minimum": 1
    }
  },
  "required": ["run_id", "view"],
  "additionalProperties": false
}
```

Return at most approximately 1.5 KB.

For `action`, require `action_index`.

### `reset_simulation`

No arguments.

It changes simulator state but not source.

Every state-changing handler must:

1. validate current agent access and registration epoch;
2. check the execution signal;
3. call the shared application method;
4. update the visible IDE;
5. return a compact verification result.

## 9.11 Output truthfulness

- `pythonShimOutput` is not modeled telemetry.
- Modeled telemetry comes only from the source plant.
- `firstDetectedFault` is not automatically root cause.
- A completed run is not automatically task success unless the source engine’s bounded, public outcome summary says so.
- In instructional mode, do not expose private/validation structures through WebMCP.

## 9.12 Automated tests

Add tests for:

1. infinite Python loop times out;
2. Stop terminates the worker;
3. WebMCP cancellation terminates the worker;
4. a subsequent run works with a fresh worker;
5. event budget failure;
6. sleep budget failure;
7. stdout/stderr truncation;
8. `send_action()` call-site file and line evidence;
9. evidence after a following sleep;
10. action with no following sleep;
11. first detected plant fault;
12. run-store eviction;
13. compact tool output;
14. no reference labels attached by action index;
15. current Run, Step, Run to Cursor, Stop, and Reset UI behavior remains functional.

## 9.13 Acceptance criteria

- Agent-triggered code cannot freeze the main page indefinitely.
- All run paths return structured results.
- Modeled evidence is separated from Python shim output.
- Cancellation is reliable through worker termination.
- Existing instructional tasks and source-plant tests pass.

## 9.14 Suggested commits

Prefer two focused commits within the PR:

```text
feat: move Pyodide execution to a terminable worker
feat: add structured call-site-linked simulation evidence
```

---

# 10. PR 3 — Revision-locked staged edits with human Apply

## Goal

Complete the first trustworthy WebMCP repair workflow without giving the agent silent source-write authority.

## 10.1 Agent access modes

At the end of PR 3:

| Access | Available actions |
|---|---|
| Off | No WebMCP tools |
| Inspect | Describe, read, run, inspect evidence, reset |
| Repair | Inspect tools plus stage edits |

Default remains Off.

Content mode remains Instructional; benchmark mode does not exist yet.

## 10.2 Workspace revision

Add:

```text
src/agent/workspace-revision.js
src/agent/edit-staging.js
```

Compute SHA-256 from stable serialization:

```text
profileId
taskId
sorted file names
exact file contents
```

Exclude:

- active tab;
- cursor position;
- dirty flags;
- timestamps;
- simulator state.

Recommended serialization:

```javascript
[
  `profile:${profileId}`,
  `task:${taskId}`,
  ...sortedFiles.map(([name, text]) => `file:${name}\0${text.length}\0${text}`)
].join("\0")
```

Prefix output:

```text
sha256:
```

## 10.3 Edit format

Use surgical exact-match edits:

```json
{
  "file": "trajectories.py",
  "start_line": 42,
  "end_line": 47,
  "expected_text": "exact existing text",
  "replacement": "new text"
}
```

Rules:

- allowed files: `main.py`, `trajectories.py`;
- one to eight edits;
- total replacement size at most 50 KB;
- `expected_text` must exactly equal the current requested line slice;
- no overlapping edits;
- apply from bottom to top within each file;
- line endings are normalized only if the existing editor already normalizes them;
- no null bytes;
- no edits to `robot_config.py`, `workcell.py`, HTML, CSS, JS, task definitions, source revisions, profiles, or limits.

Do not rely only on line numbers; exact text match is mandatory.

## 10.4 Staging behavior

`stage_workspace_edits`:

1. verifies Repair access;
2. verifies `base_revision`;
3. validates all edits;
4. computes the proposed file contents in memory;
5. creates a diff;
6. renders the diff in the visible Patch panel;
7. returns a compact staging ID and summary;
8. makes no source change.

A staged proposal is invalidated by:

- any human source edit;
- task change;
- robot change;
- workspace reset;
- importing a file;
- applying/discarding another proposal;
- page reload;
- agent access changing from Repair.

No arbitrary time expiration is required for the MVP.

## 10.5 Human-controlled apply

Do **not** register `apply_workspace_patch` in PR 3.

The visible Patch panel provides:

```text
Apply staged edits
Discard
```

Apply requires a direct human click.

On Apply:

1. recheck current revision;
2. recheck exact expected text;
3. reject if stale;
4. update files;
5. mark files dirty;
6. do not save to `localStorage`;
7. open the first changed file;
8. visibly highlight or scroll to the first changed range;
9. clear the staged proposal.

Use DOM text nodes or `textContent` for untrusted diff content. Do not inject source through `innerHTML`.

## 10.6 Future automation boundary

A later non-interactive benchmark adapter may apply edits directly through the shared staging core.

A future WebMCP apply tool, if ever added, must require an app-issued one-time approval token produced by an explicit user action. It is not part of this MVP.

## 10.7 WebMCP tool — `stage_workspace_edits`

Register only in Repair access.

Input:

```json
{
  "type": "object",
  "properties": {
    "base_revision": {
      "type": "string"
    },
    "edits": {
      "type": "array",
      "minItems": 1,
      "maxItems": 8,
      "items": {
        "type": "object",
        "properties": {
          "file": {
            "type": "string",
            "enum": ["main.py", "trajectories.py"]
          },
          "start_line": {
            "type": "integer",
            "minimum": 1
          },
          "end_line": {
            "type": "integer",
            "minimum": 1
          },
          "expected_text": {
            "type": "string"
          },
          "replacement": {
            "type": "string"
          }
        },
        "required": [
          "file",
          "start_line",
          "end_line",
          "expected_text",
          "replacement"
        ],
        "additionalProperties": false
      }
    }
  },
  "required": ["base_revision", "edits"],
  "additionalProperties": false
}
```

Return:

```json
{
  "stagingId": "stage_...",
  "baseRevision": "sha256:...",
  "changedFiles": ["trajectories.py"],
  "additions": 2,
  "deletions": 2,
  "status": "awaiting_human_apply"
}
```

Annotations:

```javascript
{
  readOnlyHint: false,
  untrustedContentHint: true
}
```

The tool changes Patch-panel state but not workspace source.

## 10.8 Patch panel

Add a `PATCH` tab to the existing bottom panel rather than creating a new modal.

Show:

- source of proposal: Site tool;
- base revision;
- changed files;
- additions/deletions;
- full textual diff;
- Apply;
- Discard;
- stale warning when invalidated.

Preserve diagnostics layout on desktop and mobile.

## 10.9 Automated tests

Test:

1. revision stable across tab/cursor changes;
2. revision changes after source edit;
3. stage with correct revision;
4. reject stale revision;
5. reject exact-text mismatch;
6. reject overlap;
7. reject protected file;
8. invalidate on human edit;
9. invalidate on task/robot change;
10. no source change before human Apply;
11. human Apply changes only approved ranges;
12. applied files remain dirty and unsaved;
13. source remains unchanged after Discard;
14. diff content is rendered safely;
15. stage tool absent in Inspect access;
16. stale registration epoch cannot stage;
17. repair workflow reruns successfully after human apply.

## 10.10 Release-1 acceptance scenario

Use an instructional fixture or a deterministic test clone of an existing task.

The scenario must demonstrate:

1. Human enables Repair.
2. Agent calls `describe_lab_task`.
3. Agent reads bounded source.
4. Agent runs until the first detected fault.
5. Agent reads fault/action evidence, including the `send_action()` call site without claiming it is the command-definition line.
6. Agent stages an exact-match edit.
7. RoboBuddy shows a visible diff.
8. Human edits source before applying; staged edit becomes stale.
9. Agent stages a new edit against the new revision.
10. Human clicks Apply.
11. Source changes visibly and remains dirty.
12. Rerun verifies whether the first detected fault is resolved.
13. Results explicitly state:
    - instructional workspace;
    - modeled source plant;
    - open-loop event replay;
    - hardware validation pending.

This scenario does **not** prove benchmark validity because the instructional task still contains reviewed reference actions.

## 10.11 Acceptance criteria

PR 3 is complete when the complete workflow works in:

- deterministic Playwright tests;
- Chrome’s WebMCP inspector;
- ChatGPT desktop built-in browser, when site tools are available for the selected account/model.

## 10.12 Suggested commit

```text
feat: stage revision-locked repairs for human approval
```

---

# 11. Release-1 gate

Do not proceed to benchmark work until all of the following are true.

## 11.1 Deterministic gate

- Existing browser smoke tests pass.
- New unit tests pass.
- New WebMCP tests pass.
- Worker timeout/cancellation tests pass.
- Desktop and mobile layout tests pass.
- Current five instructional tasks still load.
- Existing source-plant reference replay remains unchanged.
- No pinned revision was changed.
- `git diff --check` passes.

## 11.2 Manual gate

Record:

- Chrome version and flag/origin-trial state;
- Inspector tool discovery;
- inspector natural-language journey result;
- ChatGPT desktop app version;
- model used;
- tool discovery result;
- successful end-to-end repair journey;
- any rollout limitation.

## 11.3 Security/fidelity gate

Confirm:

- default site-tool access Off;
- no cross-origin exposure;
- no source apply tool;
- no reference action returned by `describe_lab_task`;
- code returned as untrusted content;
- output bounds enforced;
- worker termination on timeout;
- no private/evaluator data in Python worker;
- no hardware claim;
- no closed-loop claim;
- no “causal fault” claim.

---

# 12. Benchmark Program — begins only after Release 1

The following phases are a roadmap, not part of the first Codex handoff.

---

# 13. PR 4 — Public benchmark contract compatibility gate

## Goal

Prove that a public, unsolved task can drive the physical plant without exposing reference trajectories or private evaluation information.

## 13.1 Blocking compatibility issue

The pinned ScenarioV2 schema currently requires:

```text
hiddenGradingRequirements.length > 0
```

for portable-Python scenarios.

Therefore, this earlier instruction is invalid:

```text
“strip hidden grading and load the result through the current engine”
```

Codex must not bypass or weaken the validator.

## 13.2 Required decision

Complete a field-level leakage audit and choose one path.

### Path A — Audited public grading contract

Use only if the audit proves that the retained grading fields:

- merely restate the task’s public success criteria;
- expose no reference joint values;
- expose no accepted alternate implementation details;
- expose no hidden negative-case details;
- expose no private variant values;
- create no meaningful answer leakage.

Then:

- rename/document them as public outcome criteria in the benchmark artifact;
- keep private variants and evaluator-only cases elsewhere;
- never claim these criteria are hidden from the model.

### Path B — Preferred clean separation

Make a reviewed upstream change in `jivishov/RoboBuddy_AI`:

- define a public physics/runtime scenario that can omit evaluator-only grading fields;
- retain strict full ScenarioV2 validation for authored/private definitions;
- let the public engine run the physical plant without authoritative private grading;
- allow the private evaluator to supply or load the full grade definition separately;
- add upstream tests;
- pin the new reviewed RoboBuddy_AI commit in RoboBuddy_IDE.

Do not patch a CDN-loaded module at runtime.

## 13.3 Sanitization pipeline

After the compatibility decision, create a public task generator that:

1. begins from the authoritative source definition;
2. uses the upstream client-strip helper where appropriate;
3. explicitly removes:
   - `portablePython.referenceActions`;
   - `validation.referenceExecutions`;
   - `validation.acceptedAlternates`;
   - `validation.negativeCases`;
   - private variants and seeds;
4. includes only the grading/outcome fields allowed by the selected path;
5. sets a versioned public-task schema;
6. emits a deterministic content hash;
7. validates that the public scenario still loads through the chosen runtime path.

## 13.4 No-answer-leak checks

Static checks must search for:

- reference action field values;
- serialized reference action arrays;
- reference labels paired with action dictionaries;
- validation IDs;
- negative-case IDs;
- private seeds;
- accidental full-source copies.

A numeric fingerprint scan may be used as a warning, not the only proof, because legitimate geometry can share numbers with trajectories.

## 13.5 Benchmark workspace

Generate a separate unsolved workspace.

Do not change instructional starter generation.

Benchmark files:

- `main.py`: minimal scaffold;
- `trajectories.py`: empty or deliberately faulty scaffold;
- `robot_config.py`: read-only physical API setup;
- `workcell.py`: read-only public information.

## 13.6 Track name and claim

Call the initial track:

```text
Open-loop physical-target program synthesis
```

Do not present it as observation-driven control.

## 13.7 Acceptance criteria

- Public task loads and renders.
- Physical plant runs.
- No reference trajectory appears in browser resources or workspace.
- Public and private task hashes differ.
- Instructional mode remains unchanged.
- Benchmark content mode is separate from Agent access.

---

# 14. PR 5 — Private evaluator and reproducible run records

## Goal

Evaluate a submitted open-loop program against private authoritative cases without providing those cases to the agent.

## 14.1 Threat model

The private evaluator is intended to prevent ordinary benchmark answer leakage to a model operating the public IDE.

It is not yet intended to defend against:

- a malicious researcher with repository/server access;
- browser zero-days;
- hostile WebAssembly escape;
- a compromised dependency/CDN;
- a model with access to the evaluator filesystem.

Document this explicitly.

## 14.2 Evaluation order

1. End the model/agent interaction.
2. Serialize and hash the submission.
3. Create a fresh evaluator process/context.
4. Load the private case from a filesystem path not served by the public app.
5. Load only learner source into the Python worker.
6. Compile learner source to the event trace.
7. Replay the trace against the private source plant in the evaluator’s main context.
8. Read `plant.fault` and authoritative `snapshot().grade`.
9. write the run record.
10. dispose the context.

Private scenario/grader data must never enter the Python worker.

## 14.3 Browser controls

For the evaluator:

- block all network requests after required pinned assets are loaded;
- deny unexpected origins;
- ignore unknown worker messages;
- cap source size, events, output, wall time, and simulated time;
- create a fresh context for every case or submission;
- use no persistent storage;
- clear service workers/caches unless a verified local asset cache is explicitly used.

## 14.4 Supply-chain reproducibility

Record:

- IDE commit;
- evaluator commit;
- RoboBuddy_AI commit;
- public task hash;
- private task hash;
- canonical model revision;
- LeRobot revision;
- Pyodide version and asset hash;
- Three.js version if rendering is involved;
- Playwright version;
- bundled Chromium version;
- Node version;
- OS/container image;
- all evaluator limits.

For stronger repeatability:

- prefetch commit-pinned upstream assets;
- verify them against an `asset-lock.json` SHA-256 manifest;
- later vendor the exact evaluator runtime subtree if external availability becomes a problem.

A pinned URL alone protects content identity, not availability.

## 14.5 Source of truth

Do not implement a second grader in RoboBuddy_IDE.

Use:

```text
plant.fault
snapshot().grade
```

from the pinned authoritative engine.

The evaluator may summarize these values but must not reinterpret success through trajectory similarity.

## 14.6 Robustness categories

Report separately:

### Canonical success

One fixed program on the canonical private scene.

### Static robustness

The same fixed open-loop program is tested under small, validator-approved perturbations.

This measures margins and brittleness, not adaptation.

### Adaptive agent robustness

A fresh agent session is allowed to inspect the public variant and revise a program before each evaluation.

Do not label static hidden perturbations as adaptive intelligence.

## 14.7 Outcome record

Use a versioned run record containing:

- task and variant hashes;
- submission hash;
- compile status;
- action validity;
- first detected fault;
- authoritative goal/prohibition summary;
- safe success;
- action count;
- simulated time;
- wall time;
- truncation flags;
- provenance versions.

Do not create a weighted score that lets a safety failure be offset by speed or visual quality.

## 14.8 Naming

Use:

```text
private evaluator
```

until a containerized/hardened threat model is implemented and reviewed.

---

# 15. PR 6 — Provider-neutral agent evaluation

## Goal

Benchmark model behavior without making WebMCP the only interface.

## 15.1 Shared operation layer

Expose the same logical operations through:

- WebMCP;
- direct in-process tools;
- deterministic replay adapter;
- future remote MCP/API adapter.

Do not duplicate schemas per provider.

## 15.2 Required first adapter

Implement a deterministic replay adapter before any live provider:

```text
benchmark/adapters/replay.mjs
```

It replays a fixed sequence of tool calls and edits. This validates the harness independently of model variability.

## 15.3 Provider metadata

Every run records:

- provider;
- exact model ID;
- access date;
- API/preview endpoint;
- system prompt hash;
- user prompt hash;
- tool schema hash;
- reasoning setting;
- temperature/sampling;
- maximum turns;
- token budget;
- simulator-call budget;
- retries;
- interface mode.

## 15.4 Repeated trials

For stochastic model results:

- run multiple independent trials per task/model/interface;
- report pass@1 and pass@k where appropriate;
- report uncertainty or confidence intervals;
- preserve raw tool transcripts;
- avoid drawing conclusions from one run.

## 15.5 Interface ablation

Compare separately:

```text
direct-tools
webmcp
dom-only
```

Do not interpret the result as model-only performance; browser/client behavior is part of the condition.

## 15.6 CI boundary

- no live provider calls in pull-request CI;
- no provider keys in frontend code;
- keys only through environment variables in an opt-in runner;
- deterministic replay remains the CI test.

## 15.7 Public-leaderboard gate

Do not publish a competitive leaderboard until:

- enough tasks exist to reduce overfitting;
- task authorship and evaluator have independent review;
- no-answer-leak audits pass;
- repeated trials are used;
- the versioning and model-access policy is documented;
- benchmark limitations are published.

---

# 16. PR 7+ — Parametric scene authoring and cross-play

## Goal

Evaluate scene/workcell authoring only after the coding benchmark is trustworthy.

## 16.1 First task: inventory, not schema invention

Audit the pinned RoboBuddy_AI source for genuinely supported:

- apparatus profiles;
- renderers;
- physical-rest proxies;
- support surfaces;
- grasp interfaces;
- fixture types;
- articulated components;
- predicate types.

Publish a versioned inventory.

Do not promise beakers, burettes, clamps, or other templates unless the audited renderer and plant support them coherently.

## 16.2 Scope name

Use:

```text
parametric laboratory scene authoring
```

not unrestricted “3D asset generation.”

## 16.3 Draft language

The later draft language may allow:

- audited template ID;
- bounded dimensions;
- pose;
- approved material;
- support surface;
- grasp band/socket;
- semantic label;
- approved primitive composition.

It must not allow:

- arbitrary JavaScript;
- arbitrary Three.js code;
- external URLs;
- shaders;
- custom package imports;
- hidden grading;
- reference actions;
- evaluator seeds;
- robot limits;
- collision-tolerance weakening.

## 16.4 Validation

A trusted compiler must check:

- finite values and units;
- dimension ranges;
- unique IDs;
- visual/collision correspondence;
- initial overlap;
- stable support;
- grasp reachability;
- fixture/object separation;
- complexity budgets;
- task solvability under the selected robot.

## 16.5 Cross-play

Do not score a model only on solving its own scene.

Evaluate:

```text
A authors → A solves
A authors → B solves
B authors → A solves
B authors → B solves
trusted scene → A/B solve
```

Cross-play is required to expose self-serving or undocumented scene assumptions.

---

# 17. Later research tracks

## 17.1 Closed-loop Python

Integrate the upstream Python-RPC architecture only after:

- Python can suspend on `get_observation()`;
- the source plant advances;
- the post-physics observation returns;
- the same Python process resumes;
- cancellation and timeout remain reliable;
- tests distinguish the old and new semantics.

Keep both labels:

```text
open_loop_event_replay
closed_loop_python_rpc
```

Do not silently change existing benchmark semantics.

## 17.2 Gemini Robotics ER-class models

Include embodied-reasoning models only when current API access and terms are verified at implementation time.

Evaluate them in:

- visual task interpretation;
- tool orchestration;
- scene understanding;
- debugging;
- high-level planning.

Do not claim native 3D asset generation when the endpoint returns text/tool calls.

## 17.3 VLA direct control

Keep VLA evaluation separate from code generation.

Define independently:

- observation stream;
- camera views;
- action space;
- control frequency;
- latency;
- embodiment adapter;
- episode reset;
- safety stop;
- success metrics.

Do not place VLA and Python-coding results on one undifferentiated leaderboard.

---

# 18. Cross-cutting security checklist

For every PR:

- [ ] Agent access defaults Off.
- [ ] No cross-origin tool exposure.
- [ ] Every handler checks current access and registration epoch.
- [ ] Long operations receive the per-execution abort signal.
- [ ] Tool outputs are bounded.
- [ ] User code and task text are marked untrusted.
- [ ] No stack traces returned through tools.
- [ ] No `innerHTML` for code/diff/error content.
- [ ] No source mutation without human Apply in Release 1.
- [ ] No automatic `localStorage` save after staged edits.
- [ ] No private evaluator data sent to Python.
- [ ] No provider keys in browser code.
- [ ] Task-summary, run, and evidence tools return no reference trajectory; `read_workspace` may return explicitly requested instructional starter source and labels it as such.
- [ ] Worker/event/output limits are tested.
- [ ] Unknown worker messages are rejected or ignored.
- [ ] Current task/source revisions are not weakened or silently changed.

---

# 19. Cross-cutting fidelity checklist

- [ ] `executionMode` is explicit.
- [ ] `hardwareValidated` is always false.
- [ ] Telemetry is labeled modeled.
- [ ] `pythonShimOutput` is not presented as plant observation.
- [ ] First detected fault is not labeled root cause.
- [ ] Learner action labels are not borrowed from reference indices.
- [ ] A `send_action()` call site is not represented as the definition line of every command value.
- [ ] A completed run is not automatically described as successful.
- [ ] No force/torque/tactile claim.
- [ ] No servo/controller dynamics claim.
- [ ] No friction/compliance/backlash claim.
- [ ] No liquid/powder/process claim beyond configured discrete state.
- [ ] No hardware-safe trajectory claim.
- [ ] Static and adaptive robustness are reported separately.
- [ ] Instructional and benchmark workspaces are visibly distinguished.

---

# 20. Testing strategy

## Per commit

Run targeted checks relevant to the change:

```bash
node --check <changed-js-files>
node --test <changed-unit-tests>
npx playwright test <changed-spec>
git diff --check
```

## Per PR

Run the complete repository validation:

```bash
npm install --ignore-scripts --no-audit --no-fund
node --check src/app.js
node --check src/app-v2.js
node --check src/editor.js
node --check src/profiles.js
node --check src/python-runtime.js
node --check src/canonical-rig.js
node --check src/simulator.js
node --check src/source-simulator.js
node --check src/task-catalog.js
node --check src/task-workspace.js
node tests/validate.mjs
node tests/validate_canonical_visuals.mjs
node tests/validate_task_patch.mjs
python tests/validate_starters.py
python tests/execute_starters.py
python tests/validate_openarm_reference.py
npm run test:unit
npm run test:browser
git diff --check
```

Update the syntax list when modules are added or removed.

## Before merge

In addition to full automated tests:

- run desktop `1366×768`;
- run mobile `390×740`;
- perform Chrome Inspector WebMCP check;
- perform ChatGPT site-tools check when available;
- review source diff for answer leakage and unsupported claims.

---

# 21. Required Codex self-review for PR 0–3

Before opening each PR, Codex must inspect for:

1. accidental source/reference leakage;
2. default-on agent access;
3. stale registration or mode-race behavior;
4. unbounded outputs;
5. silent source application;
6. stale edit application;
7. worker that cannot be terminated;
8. duplicate main-thread and worker Pyodide loads;
9. incorrect GitHub Pages worker path;
10. source rendered with `innerHTML`;
11. fault labeled causal/root cause;
12. shim output labeled telemetry;
13. reference semantic labels assigned by action index;
14. pinned revision changes;
15. weakened physics/validation;
16. desktop/mobile regression;
17. hidden benchmark data in frontend or worker;
18. claims that exceed the existing simulation.

Record findings and exact test results in:

```text
docs/labforge/IMPLEMENTATION_STATUS.md
```

---

# 22. Suggested PR sequence and commits

## PR 0

```text
test/docs: characterize RoboBuddy before WebMCP changes
```

## PR 1

```text
feat: add opt-in read-only WebMCP tools
test: cover WebMCP registration and bounded inspection
```

## PR 2

```text
feat: move Pyodide execution to a terminable worker
feat: add structured call-site-linked simulation evidence
test: cover runtime timeout cancellation and evidence
```

## PR 3

```text
feat: stage revision-locked repairs for human approval
test: cover stale edit rejection and visible apply flow
docs: complete WebMCP repair MVP status
```

Do not squash during development. A reviewed PR may be squash-merged.

---

# 23. Copy-paste prompt for Codex

Attach this refined plan or commit it as:

```text
docs/labforge/CODEX_IMPLEMENTATION_PLAN.md
```

Then use:

```text
Implement the attached RoboBuddy LabForge refined plan in:

https://github.com/jivishov/RoboBuddy_IDE

Before editing, fetch current main and compare it with the reviewed baseline:
b6d29dfeffbb11830ac219a26417c09cebd98df4

If main has moved, inspect the intervening changes and update the baseline record.
Stop only for a material architectural conflict.

Implement PR 0 through PR 3 only. Use separate branches and pull requests:

feat/labforge-00-baseline
feat/labforge-01-webmcp-readonly
feat/labforge-02-cancellable-runs
feat/labforge-03-staged-edits

Do not begin benchmark task sanitization, a private evaluator, model-provider
adapters, parametric scene authoring, cross-play, closed-loop Python, or VLA
integration during this handoff.

Critical requirements:

- preserve the static GitHub Pages deployment;
- do not add React, a bundler, or a backend;
- preserve all current pinned source/model/task revisions;
- do not weaken collision, contact, support, action, or task validation;
- default Agent access to Off on every page load;
- require direct human activation for Inspect or Repair;
- register imperative WebMCP tools in the top-level page behind feature detection;
- pass each tool execution AbortSignal to long-running work;
- check current access and registration epoch inside every handler;
- keep tool names and outputs within current WebMCP guidance;
- mark source/task/output content untrusted;
- first expose only describe_lab_task and read_workspace;
- move Pyodide to a terminable worker before exposing run_simulation;
- use worker termination as the reliable timeout/cancel fallback;
- do not call browser Pyodide a secure sandbox;
- label current execution open_loop_event_replay;
- label Python stdout/stderr pythonShimOutput, not modeled telemetry;
- report firstDetectedFault, not first causal failure or root cause;
- label recorded file/line as the send_action call site, not necessarily the value-definition line;
- do not assign reference action labels or trajectories.py rows to learner actions by index;
- make agent run_simulation reset first and default to non-realtime replay;
- stage edits through exact-match, revision-locked line-range operations;
- allow staged edits only for main.py and trajectories.py;
- do not provide a WebMCP apply tool in Release 1;
- require the human to click Apply in the visible Patch panel;
- do not auto-save applied edits;
- preserve 1366×768 and 390×740 layouts;
- update Playwright test discovery so new specs actually run;
- use targeted tests per commit and the complete suite at each PR gate;
- update docs/labforge/IMPLEMENTATION_STATUS.md with exact commands and results.

PR 1 tools:
- describe_lab_task
- read_workspace

PR 2 adds:
- run_simulation
- read_run_evidence
- reset_simulation

PR 3 adds:
- stage_workspace_edits

The Release-1 acceptance journey must operate on an instructional task and must
state that it is not yet a benchmark-valid no-answer-leak task.

Before opening each PR, perform the self-review checklist in the plan. If a
requirement cannot be implemented without weakening a fidelity or security
boundary, document the blocker and continue with independent work rather than
bypassing the boundary.
```

---

# 24. Final definition of success for this handoff

The first handoff succeeds when RoboBuddy provides a verified, reviewable interaction:

```text
inspect live workspace
→ run terminably
→ observe first detected modeled fault
→ inspect call-site-linked modeled evidence
→ stage a revision-locked edit
→ human reviews and applies
→ rerun and verify
```

while preserving:

- current instructional behavior;
- source-plant fidelity;
- explicit open-loop limitations;
- hardware-validation-pending status;
- static GitHub Pages deployment;
- human control over source modification.

Only after that workflow is stable should LabForge-Bench proceed through the public-task compatibility gate and private evaluator design.
