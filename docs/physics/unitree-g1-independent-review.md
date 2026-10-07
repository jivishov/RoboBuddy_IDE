# Unitree G1 P5D independent fidelity review

Reviewed candidate: `d0711bb0d36376725a372210ccbaff4904b31885` (Claude branch). Canonical main at review: `ac42a42650782a4f0d1d3d2a48f4c16f5f2a1ca0`. The candidate is five commits ahead and includes that main; no newer MicroDuck repairs were discarded. Review date: September 10, 2026 (America/Chicago).

The supplied P5D handoff and attached governing physics plan are the requirements. Neither document is copied into this repository. This work remains confined to `jivishov/RoboBuddy_IDE_v020`; the original repository and all upstream sources are read-only.

## Assessment

The central physical implementation is sound enough to retain, not rewrite: the exact 29-DoF fixed-rubber-hand source, separate pose and physical tasks, mounted fixtures, bounded Unitree-like torque law, free-base gravity/contact, source-gain negative and engineering standing controller. The nominal standing and required failure cases reproduce in the native reference. No controller gains, geometry, physical model hashes, physics timestep, solver settings or force limits were changed in this review.

The report's claim that nothing further needed verification was too strong. Green original tests missed the following boundary/evidence defects.

## Corrections

| Finding | Reproduction | Correction and gate |
|---|---|---|
| Stale observation publication and continuation | Reset a session from its first intermediate-observation subscriber. The old chunked advance completed instead of rejecting and could step the replacement epoch. Already-returned sampled batches could publish old samples after reset. | Capture the starting context for the complete operation; check it after awaits, before each chunk and before/after publication. Late errors cannot clear replacement state. `session-epoch-publication-core.mjs`. |
| Rejected commands changed the active controller | With standing engaged, submit a non-finite joint target or low-level gain. The worker rejected it but first set its controller to null, reporting the joint-hold/low-level profile afterwards. | Build and validate the entire accepted command vector before committing any controller/profile change. Shipped-WASM and Chromium tests check both rejection and unchanged controller/joints. A valid joint-target command still intentionally releases standing. |
| Collision asset identity was not enforced during loading | Change a valid STL header byte while keeping the XML unchanged. The worker accepted the modified mesh because it checked only the XML hash. | Pin the mesh-manifest hash in the model descriptor; check each mesh hash and byte length before compilation. Finish parallel downloads before constructing the VFS, avoiding writes to an already-deleted VFS after a fetch failure. |
| Standing could pass with incomplete evidence | Two good snapshots at 2 and 8 seconds could earn six seconds of standing; missing velocities were converted to zero, and missing joint efforts were skipped. Mounted or unreadable observations were not explicitly excluded. | Fail closed on incomplete physical state; require all 29 effort measurements, a free gravity-loaded root, readable contacts and continuous declared observation cadence. Foot support requires positive normal force from an actual named contact. |
| Angular velocity was mislabeled world-frame | For a pelvis yawed 90 degrees, a local X spin was returned as world X. MuJoCo's independent `mj_objectVelocity(..., flg_local=0)` returns world Y. | Rotate free-joint angular velocity into world coordinates for pelvis and free objects; keep linear velocity world-frame. Expose the frame in live state. Native/JS frame tests cover a nontrivial rotation. |
| Native observation scheduling drifted by a step | Repeated floating-time comparisons produced a 22 ms sample gap at the declared 20 ms cadence with a 2 ms timestep. | Schedule native observations using integer controller ticks, matching the browser's fixed-step discipline. Validate full-window continuity and force-bearing feet in the native evaluator too. |

The bundled WASM's contact-force out parameter is a heap-backed `DoubleBuffer(6)` with `GetView()`, explicitly freed after reading. Passing a plain JavaScript typed array leaves that array unchanged; a zero-filled array is not contact-force evidence. The regression checks positive supporting forces, not merely whether the API call returned.

## Report wording corrected

`m*g*h = 231.3 N m/rad` is a rigid-pendulum gain-selection heuristic, not a necessary/sufficient stability theorem for the entire articulated contact system. The physical trials establish standing success/failure. The API audit now calls the comparison `passesRigidPendulumHeuristic`, not `freeBaseStable`.

The engineering controller changes kp and kd at four ankle joints: eight gain-table entries, with the two shared values kp 250 and kd 10. They are repository-authored simulator gains, not measured Unitree hardware settings.

The fixed rubber hands remain **visual-only**, exactly as in the pinned source: no finger actuation, hand collision geometry, grasping or dexterous capability is implied.

## Verification actually executed during review

Local native MuJoCo 3.11.0: all 18 required reports regenerated; physical/negative/sensitivity evidence validator passed; JavaScript/native controller parity passed. Frame interpretation was checked against MuJoCo's independent object-velocity API. Model regeneration and all 25 collision-mesh hashes passed unchanged.

Local Node 22: all 18 `tests/physics/*core.mjs` scripts passed, including the existing SO-101, OpenArm, LeKiwi, MicroDuck, runtime and session gates. The shipped G1 worker and bundled MuJoCo WASM were executed unchanged apart from adapting worker messaging and repository file delivery to Node. It passed malformed-command atomicity, tampered-mesh rejection and a complete unsupported standing trial with measured positive foot forces. This is WASM execution evidence, not a claim that a local Chromium test ran.

Native nominal stand: six evaluated seconds after the two-second ramp; pelvis 0.780879–0.781479 m, maximum tilt 4.4355 degrees, horizontal drift 0.022940 m, maximum effort approximately 13.37% of the source limit. The 1 ms configuration also passes. Source FixStand, motors disabled (exactly zero actuator effort), and the 0.20 m/s perturbation fail the standing gate. The 0.15 m/s trial passes only its stated test; it does not confer a general recovery capability.

Static fidelity, canonical visual-source contracts and starter syntax/execution checks passed. The source-defined physical model hashes remain unchanged.

Chromium runs in GitHub Actions against the exact candidate, because the local managed browser blocks URL navigation. Existing full IDE, live Python, WebMCP, browser/native and presentation tests are retained. A new browser test exercises worker-level rejected commands and force telemetry. The release must use the actual exact-SHA workflow outcomes, not infer success from a nearby commit. The deployed Pages artifact includes `physics-build.json` identifying its repository and commit.

## Scope and limitations

Standing means a bounded joint-space posture controller maintaining a free-base rigid-body model on its feet over the stated interval. It does not mean robust balance, recovery, walking or hardware calibration. Walking, perturbation recovery, dexterous hands and hardware calibration remain unsupported. Contact materials, actuator bandwidth/friction/compliance/backlash, latency and real stability margin remain unmeasured. The browser and native engines share model assumptions, so their agreement is implementation conformance, not independent physical truth.

No Panda, ASIMOV, Fly hosting, hardware-control backend or final reintegration into the original repository is included.

## Manual demonstration

Open the temporary Pages site and select Unitree G1. Its physical workspace should load without falling back to pose mode. Run the starter and inspect requested, accepted and measured joint values; standing is awarded only after its evaluation window. Select the retained pose task and confirm its fixed-root, nonphysical labels. Agent Assist remains opt-in; only the currently displayed workspace's G1 tool is registered. The automatic browser regressions additionally cover genuine falls, blocked-joint feedback, cancellation and pose/physical presentation switching.
