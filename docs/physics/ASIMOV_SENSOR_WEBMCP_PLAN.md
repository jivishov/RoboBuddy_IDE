# Asimov sensor-driven standing and WebMCP programming

Date: 2026-09-12. Target: `jivishov/RoboBuddy_IDE_v020` only.
Baseline: `de97f7c99bcf2ef792c2ccaea3c4b704cf0eff9a`.

## Reviewed implementation plan

1. Preserve all six existing workspaces, model bytes, actuator profiles and reference-controller behavior. Add separately named sensor-driven standing workspaces, reusing the unchanged source physical model.
2. Add a versioned deterministic body-frame sensor profile. Joint IDs, timestamps, availability, latency and estimated noise are explicit. Body gyro and projected gravity share one frame. Reads/rendering must not advance sensor history.
3. Add a high-level standing regulator whose only dynamic input is the synthetic sensor record. Keep the existing local joint PD plant separate. No ground-truth attitude fallback, root writes, or automatic reset. Report sensor warmup, age, filtering and latched sensor faults. Preserve the existing standing criteria.
4. Add fixed, separately selected ankle and feedback-delay sensitivity profiles. Changes are labeled hypothetical, not inferred hardware calibration. Ankle torque reductions and added friction/speed loss must actually reach the plant. Do not activate guessed A/B transmission ratios or duplicate inertia.
5. Extend the existing opt-in WebMCP tool, not a new remote server: complete joint/capability discovery, bounded joint sequences and achieved-position waits. Preserve existing commands. Validate entire programs before mutation, enforce simulation/wall budgets, check captured backend/session ownership and cancellation between short physics batches. Do not expose hardware, forces, arbitrary model writes or root state.
6. Test old/new contracts, source isolation, sensor-only feedback, noise/delay reproducibility, negative cases and actual shipped WASM. Add targeted mass/COM/contact and timestep sensitivity experiments outside the public command surface. Keep outcomes, including failures, in a reproducible report.
7. Run CI/browser journeys, retain compact evidence with the release, update limitations and provide executable WebMCP examples. Merge only a tested coherent change; leave unavailable hardware calibration as an explicit dependency.

## Critical fidelity review and resulting decisions

- **More complex is not necessarily more accurate.** No fitted motor curve, thermal duty cycle, backlash, raw accelerometer model or hardware-specific CAN order is claimed. These remain evidence-dependent.
- **Ankle mapping remains unresolved.** Added independent-axis losses test dependence on an idealized ankle; they do not establish paired-motor feasibility. No guessed transmission is promoted to a default.
- **Sensor-only has a precise boundary.** Only the new balance regulator is restricted to synthetic measurements. Inner joint PD remains the declared ideal local encoder loop; evaluators/rendering use ground truth. Projected gravity is a synthetic attitude-derived signal, not raw accelerometer output or a full estimator.
- **Latency paths differ.** Nominal-command delay, synthetic feedback delay and smoothing are separately reported. The original command-delay sweep did not exercise balance-feedback transport.
- **Sensor noise is not a confidence interval.** Fixed seeds support regression; multiple seed/parameter tests describe only the declared experimental cases.
- **Programmable does not mean unbounded.** Sequence execution is finite and prevalidated; measured-position waits can time out. A command being accepted or a sequence finishing does not establish physical task success.
- **Cancellation is cooperative at a stated bound.** Guard before/after every at-most-50-ms simulation batch; an already submitted batch cannot be undone. Capture the actual backend so a replaced workspace cannot receive old commands.
- **No silent schema break.** Keep the public v1 command envelope for compatible additions, declare a programming extension revision, and retain strict per-command schemas. Remove the erroneous outer additionalProperties restriction that would otherwise reject all oneOf branches in a standards-compliant JSON Schema validator.
- **Mass remains unreconciled.** The pinned model is 32.22491296549718 kg; the public mechanical page says 35 kg. Do not scale the production model. Mass/COM variations belong to declared tests until assembly configuration is established.
- **Evidence must be exact.** Historical CI for baseline passed on September 11, 2026; this is not a local browser pass or a calibration result. New tests must name their tested source and never overwrite historical reports.

## Acceptance gates chosen before new trials

Keep original standing thresholds (1 s settling, 10 s dwell, height >=0.50 m, tilt <=0.12 rad, drift <=0.08 m, >=20 N per foot, no other body-floor contact). Nominal new standing should pass; stress scenarios may fail and must report that honestly. Sensor faults, motor disable, controller release and large adverse model changes must not manufacture success.

Numerical comparison: common 0.1 s samples, <=0.025 rad joint and <=0.01 m body differences and same nominal standing classification at 2.5/1.25 ms. Report velocity, torque, integrated normal impulse and contact-switch diagnostics separately; no universal hardware accuracy or pointwise impact-force claim.

WebMCP: <=32 sequence segments, <=12 simulation seconds total, <=2 seconds per segment; waits <=2 seconds; at-most-50-ms batches; fixed 120-second wall deadline for an entire call. No reset in a sequence. Validate step alignment before sending any target. Human stop/access revocation/epoch replacement must prevent the next batch. Invalid later segments must leave the scene untouched.

## Primary source basis

Uploaded Asimov Actuator Fidelity Notes and original physics implementation plan; inspected baseline code. Rechecked on 2026-09-12:
- https://docs.menlo.ai/asimov/1/overview/system-tour/mechanical
- https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-gyro
- https://webmachinelearning.github.io/webmcp/

These sources support the starting model/interface facts, not the chosen synthetic noise, sensitivity multipliers or controller gains. Those are explicitly repository-designed experiments.

## Programming refinement after code review

Ordinary joint targets release the standing controller. To support useful arm programming without a hidden balance override, add explicit `set_standing_targets` and `preserve_standing` sequence segments. They accept only the 11 modeled waist/arm joints, require an already active nonfailed standing trial, preserve its evaluator and may still cause a fall. Leg targets require ordinary control; no gait or whole-body motion planner is implied. Prevalidate controller transitions across an entire sequence before mutation.
