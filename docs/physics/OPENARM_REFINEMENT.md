# OpenArm contact/workcell and WebMCP revision 3

September 18 laboratory extension: see [blank scenes and laboratory assets](OPENARM_LAB_ASSETS.md). The verified robot/controller baseline remains revision 3; the workcell tools now support optional baseline fixtures and a bounded hollow-equipment catalog.

## What changed

The rendered OpenArm follows authoritative MuJoCo body poses, including each passive and actuated finger individually. The same convex-component surfaces are used for rendering and collision: 36 source robot mesh components and 55 baseline physical geometries. There is no second canonical joint animation, visual-only palm offset, object weld or attachment. The view is a contact-aligned collision representation, not the detailed manufacturer CAD surface.

The old pickup placement overlapped source-matched palms by approximately 20–25 mm. Pickups have been repositioned, the tool approach raised, and the dry transfer reference rebuilt. Source supports, destination supports, table legs and robot mount form a physically represented workcell. Shoulder height is preserved rather than lowering the robot to hide a frame error.

The worker refreshes derived state after every integration step, independently of sampling. Observations distinguish requested target, moving reference, actual actuator input and measured joint state. MuJoCo contact forces and penetration are available for inspection. Grasp evaluation requires sustained bilateral contact; excessive vessel penetration or palm contact invalidates the reference task. The vessels must lift, carry, reach their supports, release, settle and be cleared by the robot.

## Start and control

Select OpenArm and the bimanual workcell. Run the visible Python reference to perform the dry transfer. The 17-stage reference is 26.3 simulation seconds; slow computers may take longer in wall time. The physical Python limit for this workspace is **120 seconds**. This is separate from the bounded WebMCP program limit of 180 seconds. Existing project drafts may need the normal workspace reload to receive the revised reference; preserve personal edits first.

Turn **Agent Assist** on to register the OpenArm tools. Controls are simulation-only. Angle targets are radians; position is metres in a right-handed Z-up world. Quaternion order is WXYZ. The renderer converts to the IDE's millimetre/Y-up display once; agents should not perform that conversion themselves.

`control_openarm_simulation` keeps schema `robobuddy.openarm.physical.v1` and supports `set_joint_targets`, `move_tool`, `advance` and explicit `reset`. The published JSON Schema accepts valid inputs and rejects unknown fields inside each command branch. A command being accepted does not mean its target was attained.

Reference speed limits are 0.75 rad/s for arm joints and 0.60 rad/s for finger reference motion, using quintic interpolation. An explicitly requested duration that is too short is rejected before latching a target. These are controller design choices, not guarantees about actual joint velocity or calibrated hardware limits. Gripper joint1 is positive on the left and negative on the right; magnitude 0.65 rad is the reference open setting. The mechanically coupled fingers are not independently commanded.

`move_tool` and program `tool` targets address the **pinch reference** at local EE point [-0.00143, 0, -0.153] m. Read the observed reference first. This is not an object socket, contact guarantee or attachment. Position-only IK leaves orientation unconstrained. A normalized optional quaternion constrains EE orientation. IK operates on separate scratch data and never overwrites the plant state. It is not a collision-free path planner.

## Equipment construction

`inspect_openarm_workcell` returns scene revision, body/geometry IDs, observed pinch poses, contacts, equipment affordances, passive equipment joints, transient staging and program progress. These are simulator ground-truth observations, not sensor-realistic hardware data.

`manage_openarm_workcell` uses `robobuddy.openarm.equipment.v1`:

1. `stage` supplies `expected_scene_revision` from inspection and a complete custom `equipment` array. A visible wireframe preview is shown; physics is unchanged.
2. `apply` supplies the returned `stage_id` and `acknowledge_reset:true`. The candidate must compile and pass initial collision checks before replacing the active session. Failed application preserves the old world. Applying is a setup/reset, never credited as a task outcome.
3. `discard` removes a preview. Staging/applying an empty list removes all custom equipment. Set `scene_mode:"blank"` on `stage` to also remove the baseline task fixtures and vessels, preserving robot, mount, table and floor. `scene_mode:"baseline"` restores the reference fixtures. Omitting the mode preserves the current mode. Reset retains the selected workcell; it is not a synonym for clearing it.

Each item has an identifier, kind and `position_m` describing **bottom-centre**. `dimensions_m` contains full X width, Y width and Z height, not half-sizes. Optional yaw is in radians about world Z; alternatively, normalized `quaternion_wxyz` sets the setup orientation about the local bottom-centre origin. Do not supply both. Supported kinds: platform, tray, rack, vial, block, button, assembly, funnel, burette, ring_stand, beaker, erlenmeyer_flask, bottle and tile. Inspect `assetCatalog` for bounded dimensions, defaults and scope. Catalog glassware uses separate convex wall sectors to preserve openings. Catalog geometry and parameters are illustrative, not measured from the image. Assemblies accept at most 12 box/sphere/cylinder parts with local positions and bounded dimensions. Trays/racks use separated components so cavities are not solid convex hulls. Vials are solid dry exterior models.

Limits: 12 equipment items, 256 compiled geometries, 24 kB equipment input, 0.005–2 kg dynamic item mass, and a bounded table workspace. Raw XML, arbitrary scripts, remote asset URLs and physics disabling are rejected. The scene compiler revalidates on the worker side.

Example staging payload (replace the revision with the latest inspection value):

```json
{
  "schema_version": "robobuddy.openarm.equipment.v1",
  "command": "stage",
  "expected_scene_revision": "phase5a-openarm-v2-bimanual-stack-v3",
  "equipment": [
    {"id":"button1","kind":"button","position_m":[0.57,0.32,1.005]},
    {"id":"tray1","kind":"tray","position_m":[0.35,-0.38,1.005]},
    {"id":"vial1","kind":"vial","position_m":[0.35,-0.38,1.08]}
  ]
}
```

The vial begins above the tray and settles through gravity/contact after time advances. The button is a passive spring-cap mechanism, not an electrical or thermal instrument.

## Bounded programs and outcome checks

`run_openarm_program` uses `robobuddy.openarm.program.v1`. Supply the current `expected_scene_revision` and 1–32 `segments`. It executes the current scene **without an implicit reset**. Worst-case simulated duration, including waits, is limited to 60 seconds; wall time to 180 seconds. Each segment has `duration_seconds` and optionally `targets_rad` OR `tool`. With neither, the segment is a bounded dwell.

Optional `wait_for` predicates: `bilateral_grasp`, `released`, `supported`, `in_region`, `tool_reached`, `equipment_joint`, `funnel_seated`. The new seating predicate requires a dynamic funnel, fixed burette, geometric fit, receiver contacts, release from both grippers, no other support and settled motion; its dwell is at least 0.20 s. It establishes seating, not the preceding transfer history. Other predicates use named observations and sustained dwell (default 0.06 s), with a finite additional timeout (default 3 s; at most 5 s). A timeout stops the sequence and returns its failed stage instead of success. A program without predicates reports execution completion only. Reference flask/beaker results remain separate and do not certify custom equipment tasks.

For the example button, this tested approach starts from the initial scene. All tool targets are left-sided with orientation `[0.7071067811865476,0,-0.7071067811865476,0]`:

| Stage | Target or operation | Duration |
|---|---|---:|
| Lift clear of the flask before moving laterally | Tool [0.554,0.1535,1.25] | 4 s |
| Move above the created button | Tool [0.57,0.32,1.23] | 4 s |
| Close the empty gripper | `openarm_left_finger_joint1:0.02` | 3 s |
| Approach | Tool [0.57,0.32,1.060] | 4 s |
| Press | Tool [0.57,0.32,1.050] | 2 s |
| Withdraw | Tool [0.57,0.32,1.150] | 2 s |

On the press stage, wait for `equipment_joint`, `joint_id:"lab_button1_press"`, `minimum:0.004`, `maximum:0.006`. On withdrawal require 0–0.001 m travel. Motion must come from finger contact and passive spring return. `supported` for object `lab_vial1` and geometry `lab_tray1_base` verifies tray support. This exact approach is in the worker/browser regression tests; changing equipment positions requires new reach/clearance checks.

Human Stop, workspace switching, cancellation or revoking Agent Assist prevents subsequent program steps from moving a replacement session. Controls and programs acquire the real UI execution lease to prevent simultaneous Python ownership. Pause freezes the simulation scheduler, not physical velocities.

## Verification and limits

The OpenArm workflow runs the exact checked-out commit with read-only repository permissions. It does not edit source or push new commits from CI. Its native gates compare the v3 identifiers and model/controller hashes, enforce nominal and half-timestep success, reject all five negative profiles, and retain penetration/contact and final-position evidence.

```sh
node tests/physics/openarm-phase5a-core.mjs
node tests/physics/openarm-worker.mjs
node tests/physics/openarm-lab-assets.mjs
python tests/validate_openarm_schemas.py
# Generate the seven native reports as specified in the OpenArm workflow, then:
python tests/validate_openarm_evidence.py test-results/native
npx playwright test tests/openarm-phase5a-browser.spec.mjs tests/openarm-workcell-browser.spec.mjs tests/webmcp.spec.mjs
```

Negative native profiles: `miss-left`, `weak-grip`, `blocked-left`, `outside-left`, `insufficient-budget`. These are explicit test-only interventions, not hidden assists.

The worker test uses the bundled MuJoCo 3.11.0 WebAssembly runtime, not mocked dynamics. It covers command atomicity, observation purity, sampling independence, IK without plant writes, nominal transfer, initial overlap rejection, robot-driven passive button motion, spring return and vial/tray contact. Native runs provide implementation comparison, not hardware validation. Browser tests exercise the full IDE/Python path, equipment/program tools, and real-app execution ownership. A registration adapter stands in for experimental WebMCP; CI Chromium is not claimed to implement the native interface.

Passing this reference does not establish calibrated hardware fidelity or universal grasp reliability. Meshes remain per-component convex approximations. Friction, vessel mass/inertia, grip operating limits, servo bias assistance and equipment parameters are estimates/design choices. No fluids, heat, chemistry, tactile-sensor realism, glass compliance, instrument measurement physics, hardware transport or guaranteed collision-free trajectories are included. Asset and source lineage are in `models/openarm_v2/PROVENANCE.md`.
