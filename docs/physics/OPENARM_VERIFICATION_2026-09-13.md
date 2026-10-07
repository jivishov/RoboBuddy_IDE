# OpenArm v3 verification — September 13, 2026

## Reproducible evidence

Application and test revision: `5a050b6b771d911a9947cc36a34ad0e9762cd739`.

Completed read-only GitHub Actions run: [Validate OpenArm Phase 5A / 34776902727](https://github.com/jivishov/RoboBuddy_IDE_v020/actions/runs/34776902727).

Its OpenArm contracts, shared runtime checks, exported JSON Schemas, actual WebAssembly worker tests, native positive/negative/sensitivity gates and eight browser tests passed. This is focused OpenArm and integration evidence, not a claim about the result of every repository-wide workflow.

The next commit only corrects evidence retention in this workflow and adds this report; it does not alter the tested simulator, tools or browser tests. Native reports were previously printed and validated but then removed from the artifact directory by Playwright startup cleanup. The workflow now saves them in runner temporary storage and copies them into the final artifact after the browser tests.

## Verified outcomes

The full IDE's Python reference moved the flask and beaker to their designated supports. Both vessels satisfied sustained bilateral grasp, lift, carry, supported release, settling and retreat criteria. Neither reported a remaining gripper contact at completion. The task evaluator reported no palm contact, no excessive penetration and no order violation. No page errors were recorded.

The browser evidence (`openarm-completed-state.json`) reports maximum vessel-related penetration **0.0002937299486334118 m**, approximately **0.294 mm**, during this nominal simulated run. This measures numerical contact overlap in this model; it is not a millimetre-accuracy claim about physical hardware or arbitrary grasps.

Native MuJoCo 3.11.0 passed the nominal run and the half-timestep run at 0.0005 s. All five explicit negative profiles (`miss-left`, `weak-grip`, `blocked-left`, `outside-left`, `insufficient-budget`) failed the task as required. The validator checks model/controller identity, hashes, free-object/no-weld structure, causal task events, the 2 mm penetration ceiling, and a 2 mm ceiling on nominal-versus-half-timestep final-position differences.

The equipment/program browser test created a tray, vial and passive spring button. OpenArm moved its gripper through the prescribed approach, depressed the cap by contact, withdrew, and observed spring return. It also observed the vial settling onto the tray. Failed scene compilation/initial overlap preserved the old scene; impossible outcome waits failed and cancellation released program ownership.

An additional test used the full IDE and real agent facade—not a fabricated application lease—to stage and apply equipment, run a support-checked program, reject a stale scene revision and reject competing control. Revoking Agent Assist interrupted the running program and subsequent simulation time remained unchanged. The other browser checks retained explicit-human opt-in, synthetic-click rejection, temporary cooperative editing, stale-edit rejection, program cancellation, profile-specific registration, partial-registration cleanup and unsupported-browser behavior.

Only browser WebMCP registration is adapted in these tests. They do not prove native experimental WebMCP support in CI Chromium. Actual schema validation is tested separately using the exported schemas and JSON Schema Draft 2020-12.

## Deliberate limits

This is a rigid-body laboratory manipulation simulator with an estimated servo/contact profile, not a calibrated digital twin. Rendered shapes are the source-derived convex collision components, so visual appearance is less detailed than manufacturer CAD. Per-finger observed transforms replace the old independent presentation mapping.

Cartesian IK is not a collision-free path planner. Custom workcell tasks need explicit outcome predicates and appropriate approach trajectories. The reference flask/beaker evaluator does not grade arbitrary custom equipment tasks. Button mechanics do not implement electrical instruments. No liquids, heating, chemistry, instrument measurement physics, glass compliance, hardware control or hardware-safety certification is included.

For tool schemas, limits and the tested equipment/button program, see [OpenArm usage and refinement](OPENARM_REFINEMENT.md).
