# Continuation Cycle 05 - WebMCP Control and Integration Reconciliation

## Outcome

Complete the approved integration by extending the current dirty Agent Assist/WebMCP implementation with one strictly bounded MicroDuck simulation-control tool, correct policy-simulation capability/state reporting, cancellation and duration behavior, and final source/fidelity documentation. Reconcile all five cycles through the authorized repository/browser gates without making release, deployment, or hardware claims.

## Preconditions and Evidence

- Depends on: Cycle 04 completed after its fidelity review/refinement gate.
- Verify the current five WebMCP tools, trusted Agent Assist control, access/registration epochs, immutable ready snapshot, workspace generation, run cancellation, and no-write behavior before editing.
- Verify the reviewed `MICRODUCK_COMMANDS`, command bus, immutable state, UI/audio boundaries, Python bridge, and active profile/backend events.
- Confirm no other task owns `src/webmcp/**`, `tests/webmcp.spec.mjs`, `src/app-v2.js`, Agent Assist UI, package/test discovery, or final documentation.
- Treat the dirty WebMCP files as the baseline to extend. Do not replace them with an older planned architecture or rename/migrate them solely for tidiness.

## Implementation Boundary

In scope:

- Preserve the authorized public fidelity contract in WebMCP results and final documentation: exact-byte `1x61 -> 1x14` ONNX inference and the Apache-covered hierarchy are the only exact policy/model evidence. Procedural geometry, mouth pivot, rollers, contacts, modeled sensors, and browser dynamics remain explicit approximations; never claim or surface pinned-`microduck_rl` mesh/MJCF, RL-environment, locomotion, contact, or hardware parity.

- Preserve the exact existing five tool names and behavior:
  - `describe_robobuddy_task`;
  - `read_robobuddy_workspace`;
  - `inspect_robobuddy_simulation`;
  - `focus_robobuddy_workspace`;
  - `run_robobuddy_program`.
- Preserve the current five-tool registration whenever Agent Assist is human-enabled, including loading/error and non-MicroDuck workspaces whose handlers already fail closed on readiness. Add `control_microduck_simulation` only when the current workspace is ready, the active profile is MicroDuck, and `simulationMode` is `policy_sim`. Make access epoch plus profile/workspace generation and readiness the registration identity, and trigger reconciliation on every relevant loading/ready/error/profile transition. The count contract is: zero while Assist is Off; five while Assist is On outside a ready MicroDuck policy workspace; six only for assisted ready MicroDuck. Register each surface as one abortable group, abort any partial failed attempt, and never leave duplicate/stale handlers.
- Generate the control tool's strict disjoint `oneOf` input schema from `MICRODUCK_COMMANDS`. Give every branch a command `const`, set `additionalProperties: false`, and accept only:
  - runtime commands `move`, `head`, `look`, `stop`, `enable`, `init`, `relax`, `do`, `pose`, `mouth`, `sound`, `theremin`, `chorale`, `get_mode`, `set_mode`, `get_state`;
  - simulation commands `set_color`, `spawn_ball`, `reset`, `set_tof_stimulus`, `set_camera`.
- Require `duration_ms` 20..5000 for continuous `move`, `head`, `pose`, and `mouth`; `sound(tag="wheee", hold=true)`; `theremin(active=true)`; and `chorale(active=true)`. Explicit `theremin(false)`/`chorale(false)` and non-held sounds are immediate commands without duration. Expiry applies the catalog-defined neutral value only to the call's owned intent. One-shots use catalog completion/cancellation behavior and wait with an application-level eight-second hard ceiling.
- Route through the same command bus and return `{ ok, command, requested, applied, limitedBy, completed, state, audio }` with bounded cloned state. Use only the error codes in `meta_plan.md`.
- Recheck access, registration epoch, workspace generation/profile/mode readiness, controller lease, and execution `AbortSignal` before command acquisition, during awaited work, and before result publication.
- Abort/Assist Off/profile or workspace change neutralizes continuous intent and audio modes. A cancellable one-shot uses its catalog-defined safe cancellation transition; a non-cancellable or unknown handoff resets the browser simulation. The tool handler must not guess which behavior is safe, and autonomous motion may not outlive the call.
- Preserve trusted audio unlock. WebMCP cannot fabricate a trusted gesture; audio commands before unlock return `AUDIO_LOCKED` without claiming playback.
- Extend App/facade snapshots and existing describe/inspect/run results with `policySimulationAvailable`, `simulationMode: "policy_sim"`, `stateKind: "browser_policy_sim"`, and `hardwareValidated: false`. Keep source-plant, kinematic-pose, and policy-simulation unions distinct.
- Keep all WebMCP tools simulation/source inspection only: no source writes, staging/apply, save/export/publish, hardware/network/device admin, shutdown, real media, BLE, multiplayer, hidden reference data, or unbounded state.
- Reconcile README/docs, validation discovery, and CI syntax/check lists only for files that now exist. Record exact final evidence and all unrun lanes in `docs/microduck-simulator.md` or the existing implementation-status convention.

Preserve or leave for later:

- No release, deployment, Pages, PR, provider, benchmark, physical MicroDuck, generalized robot-control tool, WebMCP source editing, or security-hardening expansion.
- Do not change the five current tool schemas/semantics except for additive truthful policy-simulation capability fields required by this integration.

## Likely Touchpoints

- `src/webmcp/agent-facade.js`
- `src/webmcp/register-ide-tools.js`
- minimal `src/app-v2.js` profile/registration event integration
- `tests/webmcp.spec.mjs`
- `tests/microduck-browser-smoke.spec.mjs`
- `tests/validate.mjs`, README/docs, and CI syntax lists only as required

## Acceptance Criteria

- Agent Assist Off registers zero tools. Assist On registers the unchanged five tools for non-MicroDuck and loading/error workspaces. Assisted ready MicroDuck registers exactly six, including one and only one `control_microduck_simulation`.
- Profile/task/workspace changes never expose a stale MicroDuck tool or publish a result under the replacement workspace identity.
- Every allowed command has one disjoint strict schema branch; unknown commands/fields, conditionally missing/invalid duration, out-of-range values, unavailable assets/audio, command conflicts, and profile mismatch return the specified bounded errors.
- Continuous commands stop at duration/abort/access/profile/workspace change. One-shot abort cannot leave motion active. A later control call remains usable.
- Tool results report requested/applied/limited values and bounded truthful state without MuJoCo/ORT objects, source files, hidden reference data, or mutable shared objects.
- Audio remains human-unlocked; WebMCP and Python cannot create a trusted event.
- Existing five tool behaviors, workspace generation/cancellation guards, human Run, visible controls, Python, all five robots, and layout remain intact.
- The final approved local static, Python, targeted browser, WebMCP, and complete existing browser gates pass, and documentation distinguishes executed evidence from all unrun CI-host/hardware/deployment/performance/host lanes.

## Verification Ceiling

Allowed checks:

- `node --check` for changed/new JavaScript;
- targeted unit tests for schema generation, exact command set, output bounds/cloning, duration/timeout, abort, epoch/profile/workspace races, and error mapping;
- focused `tests/webmcp.spec.mjs` journeys for zero/five/six registration counts across loading/ready/error/profile transitions, all command branches, conditional duration, cancellation/partial-registration failure, audio lock, profile switching, and no-write/no-hardware surface;
- affected MicroDuck UI/Python browser journeys from prior cycles only when the final discovered suite does not already cover them;
- existing `node tests/validate.mjs`, `node tests/validate_canonical_visuals.mjs`, `node tests/validate_task_patch.mjs`, and existing Python starter/reference checks;
- `npm run test:browser` once at the final integration gate;
- one final 1366×768 MicroDuck journey and one 390×740 reachability journey only if not already covered by the complete discovered suite;
- focused public-interface/fidelity/provenance diff review, `git diff --check` for tracked changes, and explicit file-enumeration/trailing-whitespace checks for new owned files.

Do not run:

- live physical hardware, Chrome/host matrices beyond the available mocked/real WebMCP lane, generic visual regression, accessibility audit, performance profile, full mobile/browser matrix, deployment/Pages, penetration, dependency-security, or generic security checks.

Stop condition: the conditional sixth tool and full integration criteria have allowed evidence, one fidelity review/refinement pass is complete, and no release/deployment work has begun.

## Unrun Checks / Residual Risk

- Mocked/local WebMCP discovery and cancellation do not prove every host implementation. Record any real host lane separately if it was actually available.
- Main-thread MuJoCo/ONNX remains unprofiled for sustained performance and thermals.
- Browser policy simulation, virtual sensors, and audio do not prove physical behavior or safety.
- No deployment, CI-host, hardware, accessibility, broad responsive, penetration, or dependency-security claim is established.

## Critical Fidelity Review and Refinement Gate

- Coding model and effort: record actual settings at cycle start.
- Review model and effort: ask the user after implementation and allowed checks.
- Default proposal when unspecified: use the coding model two supported reasoning levels higher, capped at its current maximum.
- Review target: preservation of the current five tools, exact zero/five/six registration lifecycle including loading/error and partial-failure states, disjoint schema/catalog parity, conditional durations and catalog-defined abort/reset, stale epoch/workspace protection, human audio gate, output bounds/cloning, policy/source-plant/kinematic distinctions, no write/hardware/admin surface, complete UI/Python integration, source/license fidelity, dirty-work preservation, and truthful local final evidence.
- Refinement: correct confirmed in-scope gaps and rerun only affected checks above.
- Stop after one review and one refinement pass unless the user requests another.

Do not mark completed until the selected review settings actually perform the review.

## Status Handoff

Record actual coding/review settings, exact tool surface/counts, fidelity findings/refinements, files changed, commands/results, host lanes actually run, unrun checks, and residual risk. Mark Cycle 05 completed only when all package acceptance criteria have authorized evidence. Do not commit, push, publish, deploy, or create successor cycles automatically.

### Medium implementation handoff - 2026-08-30

- Coding model/effort: `gpt-5.6-sol` / `medium`.
- Review model/effort: pending the required separate `gpt-5.6-sol` / `xhigh` critical review and one refinement pass. Cycle 05 remains not completed.
- Git baseline rechecked before edits: branch `codex/MICRODUCK`; HEAD `ccc50cb5018e6d532873181538583f1a6e636b5b`; live `origin/main` the same SHA. The Cycles 01–04 dirty set was preserved as the attributable handoff. No reset, clean, mass-format, stage, commit, push, publish, deploy, PR, or release action was performed.
- Ownership: the only active RoboBuddy IDE task found was the parent coordinator that dispatched this Cycle 05 task. Older same-checkout tasks were not active.
- Tool counts: zero while Assist is Off; the unchanged five base tools while Assist is On outside a ready MicroDuck policy workspace and during MicroDuck loading/error; exactly six for assisted ready MicroDuck, adding only `control_microduck_simulation`.
- Base tool names preserved: `describe_robobuddy_task`, `read_robobuddy_workspace`, `inspect_robobuddy_simulation`, `focus_robobuddy_workspace`, `run_robobuddy_program`.
- Sixth-tool command set: `move`, `head`, `look`, `stop`, `enable`, `init`, `relax`, `do`, `pose`, `mouth`, `sound`, `theremin`, `chorale`, `get_mode`, `set_mode`, `get_state`, `set_color`, `spawn_ball`, `reset`, `set_tof_stimulus`, `set_camera`. These 21 catalog commands generate 25 strict disjoint `additionalProperties: false` branches for the held/inactive audio variants.
- Delivered boundaries: access/workspace/profile/readiness/backend registration identity; abortable whole-group registration; bounded conditional 20..5000 ms control; shared command bus and authority; eight-second ceiling; catalog-owned expiry/abort; bounded cloned `{ok,command,requested,applied,limitedBy,completed,state,audio}`; trusted audio lock; additive policy capability/state fields; no source-write/apply/export/publish, hardware/network/admin/shutdown/media/BLE/multiplayer/hidden/unbounded surface.
- Cycle 05 files changed: `.github/workflows/validate.yml`, `README.md`, `docs/microduck-simulator.md`, `src/app-v2.js`, `src/simulator-host.js`, `src/microduck/command-bus.js`, `src/microduck/policy-simulator.js`, `src/webmcp/agent-facade.js`, `src/webmcp/register-ide-tools.js`, new `src/webmcp/microduck-control.js`, `tests/validate.mjs`, `tests/webmcp.spec.mjs`, and new `tests/webmcp-core.mjs`.
- Passed local checks: changed/new JavaScript syntax; WebMCP core (21 commands, 25 branches); MicroDuck policy core 8/8; UI core; Python bridge; asset gate (19 files, nine policies, fourteen joints, two scores, two inference outputs); repository static/canonical/task-patch checks; Python starter syntax/execution and OpenArm reference invariants; tracked diff and owned-text whitespace checks.
- Final browser gate: `npm run test:browser` executed exactly once, discovering 24 tests. Result was 23 passed and one failed. The failure was a test-only false positive: `/BLE/i` matched the `ble` suffix in valid `enable` before the ready-MicroDuck journey reached command execution. The assertion was corrected to compare exact forbidden command names; the corrected browser test and full suite remain unrun by explicit medium-turn ceiling. The successful 23 include the 1366×768 and 390×740 journeys, existing five-tool WebMCP behavior, loading/error five-tool registration, partial-failure cleanup, unsupported browsers, and the prior policy/UI/Python integration journeys.
- Host lanes run: local Windows Node/static/Python; local Playwright Chromium with mocked `document.modelContext.registerTool`. Real WebMCP host behavior and CI-host execution remain unrun.
- Residual risk/readiness: implementation and non-browser evidence are ready for the separate xhigh review/refinement turn, but the full browser gate is not formally green after the assertion correction and must be treated as unverified rather than inferred. Also unrun: physical hardware, deployment/Pages, accessibility, performance/thermals, broad browser/mobile matrices, penetration, and dependency-security checks. Approximate procedural geometry, contacts, sensors, audio, and dynamics remain outside exact policy/hierarchy evidence.

### Xhigh fidelity review, refinement, and completion - 2026-08-30

- Review model/effort actually used: `gpt-5.6-sol` / `xhigh`. The review reread the cycle/meta contracts, current WebMCP surface, command catalog/bus, policy simulator, App/backend lifecycle, tests, documentation, and final dirty diff. It preserved the medium implementation's `gpt-5.6-sol` / `medium` record.
- Confirmed findings: the active-call guard did not compare the simulator-host epoch; cancellable WebMCP one-shots had no controller-specific completion outcome, so a trusted human Stop/reset or higher-priority command could be mistaken for normal completion; lease cleanup stopped all audio rather than only owned held audio; non-held sound schema did not accept explicit `hold: false`; and the first focused browser review exposed a 20 ms timer race because the facade began measuring after the backend acquired its lease.
- One in-scope refinement pass: added simulator-epoch snapshot checks; controller-specific one-shot active/completed/cancelled tracking with catalog-safe abort and later-call recovery; human/Python preemption of lower-priority WebMCP one-shots; owned-command metadata on lease preemption/expiry and scoped held-audio release; optional explicit `hold: false` for non-held sounds; and dispatch-relative duration timing so normal expiry is not mislabeled as preemption. No command, write, hardware, network, admin, or hidden-data surface was added.
- Xhigh checks passed: syntax for the changed WebMCP/App/policy/test modules; `node tests/webmcp-core.mjs` (21 commands, 25 strict branches, cloning/bounds, epoch/cancellation/error mapping, owned expiry); `node tests/microduck-policy-core.mjs` (8/8); `node tests/microduck-ui-core.mjs`; `node tests/validate_microduck_assets.mjs` (19 files, nine policies, fourteen joints, two scores, two fixed inference outputs); and `node tests/validate.mjs`.
- Focused browser chronology: the first corrected `npx playwright test tests/webmcp.spec.mjs` review run passed 4/5 and found the dispatch/lease timer race in the ready-MicroDuck journey. After the single refinement, the same directly affected spec passed 5/5. The final slice covers the five unchanged base tools, conditional sixth tool, strict 21-command/25-branch schema, zero/five/six lifecycle, loading/error, partial-registration group abort, unsupported fallback, bounded duration, abort, manual Stop during `init`, later-call recovery, audio lock, profile removal, and no write/hardware/admin surface.
- Full-browser reconciliation: `npm run test:browser` was not rerun. Its sole allowed invocation remains 23/24 from the medium turn, with the only failure caused by the corrected test-only `enable`/`BLE` substring assertion. The final source therefore has composite passing evidence for every discovered lane: those 23 passes plus the final corrected WebMCP 5/5 slice, including the previously unexercised ready-MicroDuck journey. This is recorded as composite evidence, not as a second green full-suite invocation.
- Cycle 05 is completed on branch `codex/MICRODUCK` over baseline `ccc50cb5018e6d532873181538583f1a6e636b5b`. No stage, commit, push, PR, publish, deploy, release, cleanup, reset, or mass-format action occurred; the full Cycles 01–05 dirty work remains preserved.
- Unrun/residual: real WebMCP hosts and Chrome/host matrices, CI-host execution, physical hardware, deployment/Pages, sustained performance/thermals, accessibility, broad responsive/browser/mobile matrices, visual regression, penetration, and dependency-security remain unrun. Main-thread timing remains unprofiled, and local mocked Chromium does not prove every host. Geometry, mouth/rollers, contacts, sensors, audio, and dynamics remain approximate; only the pinned ONNX bytes and Apache-covered hierarchy carry exact evidence.
