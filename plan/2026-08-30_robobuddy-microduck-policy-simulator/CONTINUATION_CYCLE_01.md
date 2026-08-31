# Continuation Cycle 01 - Coherent Local Bundle and Articulated-Rig Gate

Status: **completed** on 2026-08-30 after the required critical review and one refinement pass. Cycle 02 is ready; it was not implemented here.

## Outcome

Deliver an independently verifiable fifth MicroDuck profile whose locally served, license-traceable, original procedural articulated rig loads in the existing simulator viewport. Align it to the pinned Apache-2.0 runtime hierarchy and named frames, label all geometry/dynamics approximations, and establish the simulator-backend seam plus deterministic CPU/browser ONNX fixture before implementing locomotion or controls.

## Preconditions and Evidence

- Depends on: none.
- Read the complete package and recheck the current branch, `HEAD`, live `origin/main`, and dirty paths before editing.
- The planning baseline was branch `codex/MICRODUCK`, `HEAD`/`origin/main` `ccc50cb5018e6d532873181538583f1a6e636b5b`, with tracked WebMCP/UI/workflow changes and three untracked WebMCP paths listed in `meta_plan.md`. Preserve them.
- Reverify upstream pins without upgrading them:
  - `pollen-robotics/microduck@590b986bd8c0d50ae02cb3ea2f59c463b6828168`;
  - browser oracle `pollen-robotics/microduck-simulator@1261013e7e28ba2a6878bd76ae573751c0e4b457`.
- Inspect the pinned runtime policy provenance, protocol/control design, `padd` mapping, sounds/score source, Apache-covered `kinematics/assets/alpha/robot_walk.xml`, fourteen-joint hierarchy, and named frames. Treat the Space only as a browser version/sequencing oracle. Do not inspect or use `microduck_rl` meshes/MJCF as distributable bytes, derivation inputs, or fidelity evidence.

## Implementation Boundary

In scope:

- Create `assets/microduck/manifest.json` as the only machine-readable runtime/asset version ledger. Record source repository/revision/path, SHA-256, byte size, per-file license classification, derived/source status, runtime purpose, and selected policy/model pairing for:
  - MuJoCo WASM 3.11.0 distribution;
  - ONNX Runtime Web 1.27.0 distribution;
  - the pinned runtime `kinematics/assets/alpha/robot_walk.xml` structural source;
  - deterministic original procedural rig data, with configured primitives separated from extracted source hierarchy/frame data;
  - kinematics/joint mapping;
  - walking, standing, ground-pick, left/right kick, sit/stand, roulade, roller, and roller-crouch ONNX policies sourced from the pinned `microduck` runtime;
  - release score/audio inputs used later.
- Store actual static bytes under `assets/microduck/` as ordinary Git content. Reject Git LFS pointer text, truncated downloads, hash mismatch, wrong MIME/magic, unlisted files, and network-fallback loaders.
- Classify each byte rather than applying a directory-wide license. Retain Apache-2.0 notices for pinned runtime code, structural XML, and policies; retain official MuJoCo, ONNX Runtime Web, and transitive distribution license/NOTICE requirements. Add `docs/microduck-simulator.md` and the necessary human-readable license/notice files. Do not copy any Space or `microduck_rl` JS, UI, CSS, audio, mesh, MJCF, policy, model, or branding byte.
- Add a deterministic asset preparation record/script that extracts source joint names, parent hierarchy, transforms, axes, signs, limits, and named frames from the pinned runtime XML, then adds original configured primitive shells, mouth pivot, roller visuals, and collision shapes. Generated outputs must be reproducible and must tag extracted versus approximate fields.
- Add `SimulatorHost` as the only App-owned object attached to `simCanvas`. It delegates source-plant/kinematic profiles to `SourceRobotSimulator` and MicroDuck to a minimal `MicroDuckPolicySimulator` shell. Give activation an epoch: idempotently dispose the old renderer, retained animation-frame handle, resize observer, controls, and scene resources; an asynchronously loaded backend may attach/publish only while its epoch remains current, otherwise it self-disposes. Add a disposed/render-loop guard to `SourceRobotSimulator`. The host owns activation/reset/readiness, fit/camera, pause/resume/stop, all time advancement, command execution, telemetry/contacts, presentation options, and disposal.
- Add `MicroDuckRigAdapter` with the shared `root`/bounds/state-application/disposal lifecycle expected by the MicroDuck backend. It loads only local generated procedural geometry aligned to source hierarchy data and later applies browser-model body transforms; do not add MicroDuck to the external-CDN `CANONICAL_VISUALS` table or force its state through the existing action-to-pose converter.
- Add the MicroDuck profile, task descriptor, and four-file starter workspace shell. Select it as the fifth option and render the full rig in the same `simCanvas`, lighting, orbit controls, fit/reset lifecycle, and mobile simulator pane used by the existing robots.
- Present walking and roller as two configured visual modes of the same runtime-derived hierarchy. Cycle 01 may hold both in the source home/keyframe pose; do not fake locomotion or imply exact passive-wheel or mouth-pivot placement.
- Produce deterministic CPU-reference fixtures directly from the exact deployed ONNX bytes in a narrow isolated CPU-compatible environment outside the app. Record environment/package versions, exact command, policy hash, deterministic 61-float input, 14-float output, and tolerance for at least one walking policy and one roller policy. Add a browser-side fixture check using vendored ONNX Runtime Web 1.27.0. This proves `1x61 -> 1x14` inference parity only; it must not record or claim RL/MuJoCo/model parity. Do not add NumPy/onnxruntime to the application dependency surface.

Preserve or leave for later:

- No complete fixed-step simulation, command bus, skills, recovery, ball interaction, UI control deck, audio, Python bridge, or new WebMCP tool.
- Do not modify or remove the current five WebMCP tools or weaken existing source-plant/Unitree validation.
- Do not convert the entire IDE to local dependencies or a bundler. The local-loading requirement is MicroDuck-specific after the base IDE has loaded.
- Do not represent the static rig shell as policy execution, contact physics, or hardware behavior.

## Likely Touchpoints

- `assets/microduck/**`
- `licenses/**` and `docs/microduck-simulator.md`
- new `src/microduck/rig-adapter.js` and asset/manifest loader modules
- new `src/simulator-host.js`, plus the minimal `src/source-simulator.js` lifecycle guard and `src/app-v2.js` integration
- `src/profiles.js`, `src/task-catalog.js`, and `src/task-workspace.js`
- `index.html` only for the fifth profile and local runtime preload/import needs
- `tests/validate_microduck_assets.mjs` and `tests/microduck-browser-smoke.spec.mjs`

## Acceptance Criteria

- MicroDuck is the fifth selectable robot, and its walking and roller forms display as a complete articulated 3D rig in the existing viewport with correct fit/reset and no detached viewer.
- Repeated source/kinematic ↔ MicroDuck profile-family switches, including a delayed stale activation, leave one active WebGL renderer, resize observer, and animation loop; inactive/stale backends are idempotently disposed and cannot later attach.
- All fourteen policy joints and `head_camera`, `tof`, trunk `imu`, `head_imu`, `mouth_tip`, and foot sites map to the pinned source hierarchy with documented units/signs. The manifest/docs distinguish these source facts from the configured approximate mouth pivot, rollers, procedural geometry/collisions, and later modeled browser sensor values.
- The manifest covers every served MicroDuck byte, exact hashes pass, no LFS pointer is served, per-file license/provenance and runtime NOTICE obligations are populated, and no unlicensed Space code/audio/model/policy byte is copied.
- With the base IDE already loaded, blocking non-loopback requests still permits selecting/resetting MicroDuck, switching its rig variant, and loading the policy fixture/runtime bytes.
- At least one walking-policy fixture and one roller-policy fixture use exact pinned runtime ONNX bytes and match deterministic CPU outputs in the browser within an explicit recorded tolerance; policy input/output shapes are exactly `1x61` and `1x14`, with no RL/MuJoCo parity claim.
- Existing profiles still load through their original paths and retain their current fidelity labels.

## Verification Ceiling

Allowed checks:

- read-only Git/revision checks and source/license inspection;
- manifest JSON parse, SHA-256/size/magic/LFS-pointer validation, and deterministic asset-generation hash comparison;
- `node --check` for changed/new JavaScript;
- `node tests/validate_microduck_assets.mjs`;
- the isolated one-time CPU fixture command plus the narrow browser fixture comparison;
- `node tests/validate.mjs` and `node tests/validate_canonical_visuals.mjs` after profile/rig integration;
- targeted `npx playwright test tests/microduck-browser-smoke.spec.mjs` for profile selection, repeated backend-family switching/disposal, walking/roller rig, fit/reset, existing-profile spot checks, and non-loopback request blocking;
- focused diff review, `git diff --check` for tracked implementation changes, and explicit file-enumeration/trailing-whitespace checks for new owned files.

Do not run:

- full policy journeys, complete browser suite, responsive matrix, accessibility, performance, hardware, deployment, penetration, dependency-security, or generic security checks.

Stop condition: provenance/license, per-policy hash/shape records, full hierarchy/frame mapping, local serving, and CPU/browser inference-fixture gates pass. If an indispensable byte lacks traceable permission, a fixture fails exact-byte inference parity, or the dirty checkout has an unresolved owner conflict, stop the cycle as blocked without substituting Space/RL bytes or fabricated source claims.

## Unrun Checks / Residual Risk

- A loaded/static articulated rig and inference fixture do not yet prove fixed-step locomotion, contact dynamics, skills, recovery, controls, or sustained frame pacing.
- The isolated CPU environment is reference generation only and is not an application dependency or CI guarantee.
- Network blocking after base load does not prove a fully offline first page load.

## Critical Fidelity Review and Refinement Gate

- Coding model and effort: record actual settings at cycle start.
- Review model and effort: ask the user after implementation and allowed checks.
- Default proposal when unspecified: use the coding model two supported reasoning levels higher, capped at its current maximum.
- Review target: exact source pins, exact-byte ONNX inference fixtures, full original procedural articulated rig rather than schematic animation, extracted-versus-configured joint/frame/geometry boundaries, activation-epoch and single-host renderer/loop disposal, local bytes/no LFS pointer, deterministic generation, per-file licenses/runtime notices, no copied Space or `microduck_rl` assets, existing-profile preservation, truthful static-rig limits, and no RL/MuJoCo parity claim.
- Refinement: correct confirmed in-scope gaps and rerun only affected checks above.
- Stop after one review and one refinement pass unless the user requests another.

Do not mark completed until the selected review settings actually perform the review.

## Status Handoff

Record actual model settings, source revisions, manifest/fixture hashes, asset-generation command, findings/refinements, files changed, exact checks/results, unrun checks, and residual risk. Set Cycle 02 to `ready` only after all Cycle-01 gates pass; otherwise record the precise blocker.

## Completion Record

### Actual model settings

- Implementation: `gpt-5.6-sol`, `medium` reasoning.
- Critical fidelity review and refinement: `gpt-5.6-sol`, `xhigh` reasoning.
- Review count: one critical review and one in-scope refinement pass, as required.

### Final source and checkout evidence

- Branch: `codex/MICRODUCK`.
- `HEAD`: `ccc50cb5018e6d532873181538583f1a6e636b5b`.
- Live `origin/main`: `ccc50cb5018e6d532873181538583f1a6e636b5b` at final review.
- Pinned MicroDuck runtime: `pollen-robotics/microduck@590b986bd8c0d50ae02cb3ea2f59c463b6828168`.
- Browser sequencing oracle remains `pollen-robotics/microduck-simulator@1261013e7e28ba2a6878bd76ae573751c0e4b457`; no Space byte was distributed.
- Preserved the pre-existing dirty WebMCP/UI/workflow work. The three untracked WebMCP files remained present; final SHA-256 values were `501acbf139ce13df48ecefaf7a3b248ffb816960c4faf6ebc839388bf6259644` (`agent-facade.js`), `f3ef939edd9b51dca8237f8e8e5fa6794158722931683abb4b01bd90fc2a1440` (`register-ide-tools.js`), and `d36dcbaadc30f735fde369e453dccdb040816e2169a38bf5f7cf14dfd48957c4` (`webmcp.spec.mjs`). No reset, clean, stage, commit, push, deploy, or PR action occurred.

### Delivered boundary

- Added the fifth MicroDuck profile and four-file static starter workspace in the shared IDE and shared `simCanvas`.
- Added one `SimulatorHost`, a local `MicroDuckPolicySimulator` Cycle-01 shell, and a `MicroDuckRigAdapter` around the pinned fourteen-joint hierarchy. Walking and roller are visibly distinct configured presentations; policy execution remains disabled in both the UI and programmatic run entrypoints.
- Bundled the exact nine pinned ONNX policies, MuJoCo WASM 3.11.0, ONNX Runtime Web 1.27.0, pinned structural XML, deterministic procedural rig, CPU/browser fixture, and the two upstream-original release-safe score inputs (`wistful.duckscore`, `duck_strut.mid`). The upstream copyrighted/test-only `outer_wilds.mid` is explicitly excluded; no rendered audio is shipped in Cycle 01.
- `assets/microduck/manifest.json` inventories 19 served assets with byte size, SHA-256, source path/revision, purpose, status, and per-file license. Its final SHA-256 is `efc7357bb708dde5600b5ac670c4fc27a3ea74b7e0176434860a5a5996d11cd8`.
- Deterministic generated rig SHA-256: `f5510250976f8f2fe237bd04bd0d0094d7eb9696e7e2bbea8e88773d8a8e5f4b`.
- CPU/browser fixture SHA-256: `d717598ffc4809797612bb10df36c3c296437a8a1794a201b9dcbb779e34bafc`.
- Generation command: `node scripts/prepare_microduck_assets.mjs`.
- CPU fixture command recorded in the fixture: `python scripts/verify_microduck_cpu_fixture.py`.

### Critical review findings and refinement

1. The initial host epoch rejected stale publication but did not track and immediately dispose a still-loading backend, permitting overlapping renderer/observer/animation-loop lifetimes. The refinement now tracks every pending backend, disposes pending and active instances before replacement, publishes a pending-count diagnostic, and makes host disposal clear both sets. The targeted browser race delays the MicroDuck rig response and proves the stale result is `false`, the replacement is ready, the source backend remains active, and the pending count returns to zero.
2. `SourceRobotSimulator` could finish a canonical-rig or scenario-engine await after disposal and attach resources. Post-await disposed guards now dispose the newly resolved rig/plant before returning. Both backends retain and cancel their animation-frame handles, disconnect observers, dispose controls/renderers, and expose idempotent readiness/pause/resume/stop behavior through the host.
3. The mapping needed an explicit contract. Generated data and the manifest now record metres, radians, right-hand positive rotation about the source XML axes, the fifteen-entry wire order, mouth at wire index 9, and the fourteen-entry policy action order with mouth deliberately skipped.
4. Asset checks previously proved hashes/LFS rejection but not enough file-format evidence. The gate now verifies WASM and MIDI magic, rejects truncated/text ONNX responses and truncated runtime modules, checks the exact score inventory/exclusion, and records an exact repeatable CPU verification command.
5. The original score inputs were missing from the local ledger. The two upstream files explicitly described as original are now exact-source bytes with Apache provenance; the upstream commit message's copyrighted test score is excluded. The mixed generated rig is classified `Apache-2.0 AND PolyForm-Noncommercial-1.0.0` because it combines extracted Apache hierarchy data with original local procedural configuration.
6. The static policy shell still exposed Run/Step/Cursor and could report source-plant execution. The refinement disables those controls and rejects the programmatic execution entrypoints for Cycle 01 while preserving fit/reset/inspection and the explicit static-rig label.

### Cycle-owned implementation paths

- `assets/microduck/**`
- `docs/microduck-simulator.md`
- `licenses/microduck-apache-2.0.txt`
- `licenses/microduck-third-party-notices.md`
- `scripts/prepare_microduck_assets.mjs`
- `scripts/verify_microduck_cpu_fixture.py`
- `src/simulator-host.js`
- `src/microduck/policy-simulator.js`
- `src/microduck/rig-adapter.js`
- MicroDuck integration in `index.html`, `src/app-v2.js`, `src/profiles.js`, `src/source-simulator.js`, `src/task-catalog.js`, and `src/task-workspace.js`
- `tests/validate_microduck_assets.mjs`
- `tests/microduck-browser-smoke.spec.mjs`

### Executed verification

- Relevant `node --check` commands: passed.
- `node tests/validate_microduck_assets.mjs`: passed; 19 files, 9 policies, 14 joints, 2 release-safe scores, and 2 fixed inference outputs.
- Deterministic preparation rerun: passed; generated-rig and manifest hashes were unchanged across the rerun.
- Isolated CPython 3.10.1 environment with NumPy 2.2.6 and ONNX Runtime 1.23.2 CPU provider: `alpha_walking` and `roller` both recomputed `1x61 -> 1x14` with `max_error=0.0`.
- `node tests/validate.mjs`: passed.
- `node tests/validate_canonical_visuals.mjs`: passed.
- `npx playwright test tests/microduck-browser-smoke.spec.mjs`: 3 passed. Covered an existing source profile, MicroDuck selection, walking/roller presentation, disabled Cycle-01 execution, fit/reset, Unitree/MicroDuck family switching, non-loopback blocking after base load, delayed stale activation disposal, and vendored-browser-ORT comparison for walking and roller.
- `git diff --check`: passed (Git emitted only existing Windows LF/CRLF conversion warnings).
- Explicit trailing-whitespace scan over new owned text files: none.

### Unrun checks and residual risk

- Unrun by the Cycle-01 ceiling: full policy journeys, complete browser suite, responsive matrix, accessibility, performance, hardware, deployment, penetration, dependency-security, and generic security checks.
- The static articulated rig and exact-byte inference fixture do not prove fixed-step locomotion, contact dynamics, skills, recovery, sustained frame pacing, hardware behavior, or RL/MuJoCo-model parity.
- The isolated CPU environment is verification-only and is not part of the application dependency surface or a CI guarantee.
- Blocking non-loopback traffic after initial IDE load does not prove a fully offline first page load.

All Cycle-01 stop gates passed. Cycle 02 may begin from its own continuation file.
