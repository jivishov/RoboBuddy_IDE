# Blank OpenArm workcells and hollow laboratory assets

September 18, 2026. This is a focused extension of the deployed OpenArm workcell API, not a merge of the separate `lab-builder-v2` branch.

## Clear and restore through WebMCP

Enable Agent Assist in a ready OpenArm physical workspace. First call `inspect_openarm_workcell` and read `authority.sceneRevision`. To clear the task equipment, call `manage_openarm_workcell` with the following, replacing the revision placeholder:

```json
{
  "schema_version": "robobuddy.openarm.equipment.v1",
  "command": "stage",
  "expected_scene_revision": "COPY_FROM_INSPECTION",
  "scene_mode": "blank",
  "equipment": []
}
```

Then apply the returned stage ID:

```json
{
  "schema_version": "robobuddy.openarm.equipment.v1",
  "command": "apply",
  "stage_id": "COPY_FROM_STAGE_RESULT",
  "acknowledge_reset": true
}
```

Staging is a preview, not physical movement. Applying explicitly resets into the checked scene. A failed compile or overlap check preserves the old physical session. Blank mode removes the reference flask, beaker, pickup supports, hotplate and ring support from both physics and rendering; robot, mount, tabletop and floor remain. Reset preserves this mode. Stage/apply an empty list with `scene_mode:"baseline"` to restore the reference setup. Omitted mode preserves the selected mode. Each stage supplies the complete replacement custom equipment list.

## Equipment and coordinate conventions

The catalog adds `funnel`, `burette`, `ring_stand`, `beaker`, `erlenmeyer_flask`, `bottle` and `tile`. Existing kinds remain supported. `inspect_openarm_workcell` returns catalog defaults, actual bodies, geometry names and world-space affordance points.

All input lengths are metres in the Z-up MuJoCo world. `dimensions_m` gives full dimensions. `position_m` locates the local bottom-centre origin, not the centre of mass. For a funnel it is the stem tip. Use either `yaw_rad` or a normalized WXYZ quaternion, never both. With a rotated object, the origin still refers to its local bottom-centre; all rotated bounding-box corners must fit above the table. The tabletop is at Z = 1.005 m.

A loose sideways funnel can be staged with `kind:"funnel"`, `position_m:[0.34,-0.34,1.04]` and `quaternion_wxyz:[0.7071067811865476,0,0.7071067811865476,0]`. It is dynamic by default. Objects move only after physics advances.

Default funnel dimensions are 60 x 60 x 85 mm, with a 10 mm outside-diameter hollow stem. The burette is 400 mm high with an open bore and closed stopcock. The bore and conical bowl use separate convex wall components, not a single convex hull closing the opening. Rendered and physical components share the same vertices.

Burettes, stands and tiles are fixed fixtures in this catalog. A burette's fixed/clamped mounting is an explicit boundary condition, not a simulation of a robot tightening a clamp. The stand includes a base, rod, bracket and split clamp; align its `clampReferenceM` with the burette's clamp point. Capped bottles and old `vial` objects are solid exterior models; the new beaker and Erlenmeyer are open vessels. No fluid process is simulated.

The catalog permits bounded scaling (0.6–1.5 times each default dimension), at most 12 items, 256 compiled geometries and 24 kB input. Arbitrary meshes, remote asset URLs, caller XML, scripts and physics disabling remain unsupported. The current rectangular setup envelope is X 0–0.82 m, Y -0.55–0.55 m, Z 1.005–1.90 m. Being inside it does not establish arm reachability or a collision-free path.

## Programming and evidence

Use existing bounded joint/Cartesian programs. Read the observed pinch pose, asset grasp reference and mouth position; check reach and clearance before carrying the funnel. Scene editing is setup, never an alternative to robot manipulation.

A final program segment can wait for:

```json
{
  "type": "funnel_seated",
  "object_id": "lab_funnel",
  "receiver_id": "lab_burette",
  "dwell_seconds": 0.2,
  "timeout_seconds": 3
}
```

This requires bore fit, receiver contact, release from both grippers, no other active support, acceptable penetration and low object velocity for a sustained dwell. `funnelSeating` in inspection is instantaneous evidence only. To establish a robot transfer, separately observe the initial free object, bilateral grasp, lift, carry, release and final seating. A funnel dropped into a burette at setup does not prove that the robot put it there. Blank scenes do not claim success for the unrelated baseline flask/beaker task.

## Reproduction and scope

```sh
node tests/physics/openarm-phase5a-core.mjs
node tests/physics/openarm-worker.mjs
node tests/physics/openarm-lab-assets.mjs
npx playwright test tests/openarm-workcell-browser.spec.mjs tests/webmcp.spec.mjs
```

The new real-WASM tests cover blank/reset/restoration, every catalog asset, orientation/envelope rejection, physical passive funnel seating, invalid-contact/held/offset/moving negative controls, initial-overlap rejection, and a separate robot-driven funnel grasp/lift with an open-gripper no-lift control. The browser test covers the registered tool flow, a seven-item titration-inspired scene, seating and restoration. CI retains screenshots and JSON evidence.

These tests do **not** establish a complete robot pick-and-place from the supplied photograph. Geometry, friction, mass and contact parameters are illustrative estimates; no image-derived dimensional calibration, photorealistic reconstruction, hardware validation, glass deformation, chemistry, automatic motion planning or fluid flow is claimed. Browser tests adapt experimental WebMCP registration, while exercising the actual application handlers and MuJoCo worker. Native in-app WebMCP host compatibility is a separate integration check.
