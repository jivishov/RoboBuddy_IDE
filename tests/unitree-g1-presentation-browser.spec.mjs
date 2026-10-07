import { expect, test } from '@playwright/test';

// Unitree G1 Phase 5D presentation regression.
//
// The canonical Three.js rig is presentation only. It must follow the MuJoCo observation and must
// never drive it, must never be mistaken for the physical authority, and must not carry a physical
// claim into the retained kinematic pose workspace. These are the visible-behaviour checks: a fall
// that is really rendered, a stand that is really rendered, joints that show measured state rather
// than requested targets, and a renderer that cannot advance physics.

const PHYSICAL_TASK = 'unitree-g1-physical-dynamics';
const POSE_TASK = 'unitree-g1-kinematic-pose-inspection';
const RAD_TO_DEG = 180 / Math.PI;

async function openUnitree(page) {
  await page.goto('/?ci=unitree-g1-presentation', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
  await page.locator('#robotSelect').selectOption('unitree');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 120_000 });
  await expect(page.locator('#taskSelect')).toHaveValue(PHYSICAL_TASK);
}

// One rendered frame, then everything the renderer and the authority each claim about that frame.
async function frame(page) {
  return page.evaluate(() => {
    const sim = window.__robobuddyCi.app.sim.backend;
    sim.renderFrame();
    const alignment = sim.getPresentationAlignment();
    const state = sim.getState();
    const canvas = document.getElementById('simCanvas');
    return {
      alignment,
      audit: sim.getPresentationAudit(),
      simulationTimeSeconds: state.simulation_time_s,
      pelvisZM: state.root.position_m[2],
      tiltRad: state.root.tilt_rad,
      controller: state.controller_mode,
      actuationEnabled: state.actuation_enabled,
      footContacts: { left: state.foot_contacts.left.length, right: state.foot_contacts.right.length },
      nonFootGround: state.contacts.non_foot_ground.length,
      evaluation: window.__robobuddyCi.app.sim.getTaskEvaluation?.() ?? null,
      dataset: {
        rootMode: canvas.dataset.unitreeG1RootMode ?? null,
        standing: canvas.dataset.unitreeG1Standing ?? null,
        fell: canvas.dataset.unitreeG1Fell ?? null,
        pelvisZM: canvas.dataset.unitreeG1PelvisZM ?? null,
        actuationEnabled: canvas.dataset.unitreeG1ActuationEnabled ?? null,
        renderedFrames: canvas.dataset.renderedFrames ?? null,
      },
    };
  });
}

// The rig's joint groups sit on the source joint pivots, which are the MuJoCo child body origins,
// so a frame, sign or joint-order mistake shows up here as a millimetre error.
function expectRegistered(value, label) {
  expect(value.alignment, label).not.toBeNull();
  expect(value.alignment.comparedBodies, label).toBe(29);
  // 0.6 um is the float/rounding floor of this comparison. Anything larger is a real registration
  // error: a joint-order, sign or frame mistake, or an observation whose body poses and joint
  // angles come from different instants.
  expect(value.alignment.maxBodyErrorMm, `${label}: worst body ${value.alignment.worstBody}`).toBeLessThan(0.01);
  expect(value.alignment.objectErrorMm, label).toBeLessThan(1e-6);
  expect(value.audit.presentationAuthority, label).toBe('observed MuJoCo pelvis transform and 29 measured joint positions only');
  expect(value.audit.drivesFromObservation, label).toBe(true);
  expect(value.audit.jointCount, label).toBe(29);
}

test('the rendered Unitree G1 follows measured MuJoCo state and cannot drive it', async ({ page }, testInfo) => {
  test.setTimeout(600_000);
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await openUnitree(page);

  const initial = await frame(page);
  expectRegistered(initial, 'initial');
  expect(initial.simulationTimeSeconds).toBe(0);
  expect(initial.dataset.rootMode).toBe('free-base');

  // Rendering many frames must not advance the plant by one step. This is the render-frame physics
  // prohibition, checked against the authority's own clock rather than a visual.
  const rendered = await page.evaluate(async () => {
    const sim = window.__robobuddyCi.app.sim.backend;
    const before = { time: sim.getState().simulation_time_s, frames: sim.renderedFrames, pelvis: sim.getState().root.position_m[2] };
    for (let index = 0; index < 240; index += 1) sim.renderFrame();
    await new Promise((resolve) => requestAnimationFrame(() => requestAnimationFrame(resolve)));
    const after = { time: sim.getState().simulation_time_s, frames: sim.renderedFrames, pelvis: sim.getState().root.position_m[2] };
    return { before, after };
  });
  expect(rendered.after.frames - rendered.before.frames, 'the render loop must really have run').toBeGreaterThanOrEqual(240);
  expect(rendered.after.time, 'rendering must not advance simulated time').toBe(rendered.before.time);
  expect(rendered.after.pelvis, 'rendering must not move the physical root').toBe(rendered.before.pelvis);

  // The verified controller really holds, and the rendered rig tracks that hold.
  await page.evaluate(async () => {
    const sim = window.__robobuddyCi.app.sim.backend;
    await sim.engageStand({ holdSeconds: 0 });
    await sim.advanceTime(2);
    for (let index = 0; index < 4; index += 1) await sim.advanceTime(1.6);
  });
  const standing = await frame(page);
  expectRegistered(standing, 'standing');
  expect(standing.controller).toBe('robobuddy_g1_stand_v1');
  expect(standing.evaluation.standing).toBe(true);
  expect(standing.dataset.standing).toBe('true');
  expect(standing.dataset.fell).toBe('false');
  expect(Number(standing.dataset.pelvisZM)).toBeCloseTo(standing.pelvisZM, 4);
  expect(standing.nonFootGround).toBe(0);
  expect(standing.footContacts.left).toBeGreaterThan(0);
  expect(standing.footContacts.right).toBeGreaterThan(0);

  // A bounded low-level target is a request. What is displayed is the measured joint angle, so the
  // two must be visibly different while the joint is still travelling toward the target. This
  // replaces the stand controller with the low-level motor law, so it runs last.
  const divergence = await page.evaluate(async () => {
    const host = window.__robobuddyCi.app.sim;
    const sim = host.backend;
    const requestedRad = 0.6;
    await host.applyLowLevelCommands({ waist_yaw_joint: { positionRad: requestedRad, velocityRadS: 0, kp: 200, kd: 5, feedforwardTorqueNm: 0 } }, { advanceSeconds: 0.06 });
    sim.renderFrame();
    const alignment = sim.getPresentationAlignment();
    const joint = sim.getState().joints.waist_yaw_joint;
    return {
      requestedRad,
      requested: joint.requested_target_rad,
      accepted: joint.accepted_target_rad,
      measured: joint.position_rad,
      displayedDeg: alignment.jointDeg.waist_yaw_joint,
      alignmentMeasuredRad: alignment.measuredJointRad.waist_yaw_joint,
      alignmentAcceptedRad: alignment.acceptedTargetRad.waist_yaw_joint,
      maxBodyErrorMm: alignment.maxBodyErrorMm,
      worstBody: alignment.worstBody,
      comparedBodies: alignment.comparedBodies,
      controller: sim.getState().controller_mode,
    };
  });
  expect(divergence.requested).toBeCloseTo(divergence.requestedRad, 12);
  expect(divergence.accepted).toBeCloseTo(divergence.requestedRad, 12);
  expect(divergence.alignmentMeasuredRad).toBeCloseTo(divergence.measured, 12);
  // The visual shows measured radians converted to the rig's degrees, never the request.
  expect(divergence.displayedDeg).toBeCloseTo(divergence.measured * RAD_TO_DEG, 4);
  expect(
    Math.abs(divergence.measured - divergence.accepted),
    'a target reached instantly would leave nothing to distinguish a request from a measurement',
  ).toBeGreaterThan(0.02);
  expect(divergence.maxBodyErrorMm, `worst body ${divergence.worstBody}`).toBeLessThan(0.01);
  expect(divergence.comparedBodies).toBe(29);
  expect(divergence.controller, 'a low-level command is not the standing controller').toBe('unitree_g1_lowlevel_motor');

  await page.locator('#simCanvas').screenshot({ path: testInfo.outputPath('unitree-g1-standing.png') });
  await testInfo.attach('standing-presentation', { body: JSON.stringify({ initial, rendered, divergence, standing }, null, 2), contentType: 'application/json' });
  expect(pageErrors, `page errors: ${pageErrors.join('\n')}`).toEqual([]);
});

test('a genuine fall and a verified stand are both visible in the rendered scene', async ({ page }, testInfo) => {
  test.setTimeout(600_000);
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await openUnitree(page);

  // The fall is produced by removing actuation through the one declared setup operation, not by
  // writing root state, so the rendered collapse is a real contact-and-gravity outcome.
  const fall = await page.evaluate(async () => {
    const host = window.__robobuddyCi.app.sim;
    const sim = host.backend;
    const samples = [];
    await host.setActuationEnabled(false);
    for (let index = 0; index < 8; index += 1) {
      await sim.advanceTime(0.25);
      sim.renderFrame();
      const alignment = sim.getPresentationAlignment();
      const state = sim.getState();
      samples.push({
        timeS: state.simulation_time_s,
        pelvisZM: state.root.position_m[2],
        pelvisVisualMm: alignment.pelvisVisualMm,
        tiltRad: state.root.tilt_rad,
        maxBodyErrorMm: alignment.maxBodyErrorMm,
        nonFootGround: state.contacts.non_foot_ground.length,
      });
    }
    return { samples, evaluation: host.getTaskEvaluation(), fell: document.getElementById('simCanvas').dataset.unitreeG1Fell };
  });
  await page.locator('#simCanvas').screenshot({ path: testInfo.outputPath('unitree-g1-fallen.png') });

  const first = fall.samples[0];
  const last = fall.samples.at(-1);
  expect(last.pelvisZM, 'the pelvis must really fall').toBeLessThan(first.pelvisZM - 0.4);
  expect(last.tiltRad, 'a fall is not a vertical drop').toBeGreaterThan(1);
  expect(last.nonFootGround, 'something other than a foot must reach the ground').toBeGreaterThan(0);
  expect(fall.fell).toBe('true');
  // The rendered pelvis must track the physical pelvis through the whole fall, in millimetres.
  for (const sample of fall.samples) {
    expect(sample.maxBodyErrorMm, `t=${sample.timeS}`).toBeLessThan(0.01);
    expect(sample.pelvisVisualMm[1] / 1000, `t=${sample.timeS}`).toBeCloseTo(sample.pelvisZM, 3);
  }

  // Reset is a scene reload, not a recovery, and the verified controller then really holds.
  const recovered = await page.evaluate(async () => {
    const host = window.__robobuddyCi.app.sim;
    const sim = host.backend;
    await host.setActuationEnabled(true);
    await sim.reset();
    const afterReset = { timeS: sim.getState().simulation_time_s, pelvisZM: sim.getState().root.position_m[2], fell: document.getElementById('simCanvas').dataset.unitreeG1Fell };
    await sim.advanceTime(0.2);
    await sim.engageStand({ holdSeconds: 0 });
    await sim.advanceTime(2);
    for (let index = 0; index < 4; index += 1) await sim.advanceTime(1.6);
    sim.renderFrame();
    const alignment = sim.getPresentationAlignment();
    const state = sim.getState();
    return {
      afterReset,
      held: {
        timeS: state.simulation_time_s, pelvisZM: state.root.position_m[2], tiltRad: state.root.tilt_rad,
        pelvisVisualMm: alignment.pelvisVisualMm, maxBodyErrorMm: alignment.maxBodyErrorMm,
        nonFootGround: state.contacts.non_foot_ground.length, controller: state.controller_mode,
      },
      evaluation: host.getTaskEvaluation(),
      dataset: { standing: document.getElementById('simCanvas').dataset.unitreeG1Standing, fell: document.getElementById('simCanvas').dataset.unitreeG1Fell },
    };
  });
  await page.locator('#simCanvas').screenshot({ path: testInfo.outputPath('unitree-g1-stood-after-reset.png') });

  expect(recovered.afterReset.timeS).toBe(0);
  expect(recovered.afterReset.pelvisZM).toBeGreaterThan(0.73);
  expect(recovered.held.controller).toBe('robobuddy_g1_stand_v1');
  expect(recovered.held.pelvisZM).toBeGreaterThan(0.73);
  expect(recovered.held.pelvisZM).toBeLessThan(0.84);
  expect(recovered.held.nonFootGround).toBe(0);
  expect(recovered.held.maxBodyErrorMm).toBeLessThan(0.01);
  expect(recovered.held.pelvisVisualMm[1] / 1000).toBeCloseTo(recovered.held.pelvisZM, 3);
  expect(recovered.evaluation.standing).toBe(true);
  expect(recovered.dataset).toEqual({ standing: 'true', fell: 'false' });
  await testInfo.attach('fall-and-stand-presentation', { body: JSON.stringify({ fall, recovered }, null, 2), contentType: 'application/json' });
  expect(pageErrors, `page errors: ${pageErrors.join('\n')}`).toEqual([]);
});

test('switching between the two Unitree workspaces switches the presentation and its labels', async ({ page }, testInfo) => {
  test.setTimeout(600_000);
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await openUnitree(page);

  const labels = () => page.evaluate(() => {
    const canvas = document.getElementById('simCanvas');
    const app = window.__robobuddyCi.app;
    return {
      capabilityBadge: document.getElementById('physicsBackendBadge')?.textContent ?? null,
      driverLabel: document.getElementById('driverLabel')?.textContent ?? null,
      fidelityText: document.getElementById('fidelityText')?.textContent ?? null,
      simulatorBackend: canvas.dataset.simulatorBackend ?? null,
      simulationAuthority: canvas.dataset.simulationAuthority ?? null,
      physicalSceneId: canvas.dataset.physicalSceneId ?? null,
      physicalSceneRevision: canvas.dataset.physicalSceneRevision ?? null,
      modelPackageId: canvas.dataset.modelPackageId ?? null,
      unitreeKeys: Object.keys(canvas.dataset).filter((key) => key.startsWith('unitreeG1')).sort(),
      physical: app.isPhysicalWorkspace(),
      kinematic: app.isKinematicPoseWorkspace(),
      hasPhysicalSession: Boolean(app.sim.getPhysicalSession()),
      alignment: Boolean(app.sim.getPresentationAlignment()),
    };
  });

  const physical = await labels();
  expect(physical.capabilityBadge).toBe('browser-mujoco · numerically-verified');
  expect(physical.driverLabel).toContain('browser MuJoCo');
  expect(physical.fidelityText).toContain('Walking is unsupported');
  expect(physical.simulatorBackend).toBe('browser-mujoco');
  expect(physical.simulationAuthority).toBe('physics-session');
  expect(physical.physicalSceneId).toBeTruthy();
  expect(physical.modelPackageId).toBeTruthy();
  expect(physical.unitreeKeys).toContain('unitreeG1RootMode');
  expect(physical).toMatchObject({ physical: true, kinematic: false, hasPhysicalSession: true, alignment: true });

  // Pose the physical plant so the two workspaces cannot be confused by a coincidentally equal
  // rendered posture, then switch.
  await page.evaluate(async () => {
    const sim = window.__robobuddyCi.app.sim.backend;
    await sim.engageStand({ holdSeconds: 0 });
    await sim.advanceTime(2);
  });
  await page.locator('#simCanvas').screenshot({ path: testInfo.outputPath('unitree-g1-physical-workspace.png') });

  await page.locator('#taskSelect').selectOption(POSE_TASK);
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 120_000 });
  const pose = await labels();
  expect(pose.capabilityBadge).toBe('legacy · model-derived');
  expect(pose.driverLabel).not.toContain('browser MuJoCo');
  expect(pose.fidelityText).toContain('No fixed-step contact plant');
  expect(pose.simulatorBackend).toBe('source-robot');
  // Nothing physical may survive the switch: no authority, no scene, no model package, and none of
  // the physical workspace's own free-base/standing/contact claims.
  expect(pose.simulationAuthority, 'the pose workspace has no PhysicsSession to be the authority').toBeNull();
  expect(pose.physicalSceneId).toBeNull();
  expect(pose.physicalSceneRevision).toBeNull();
  expect(pose.modelPackageId).toBeNull();
  expect(pose.unitreeKeys).toEqual([]);
  expect(pose).toMatchObject({ physical: false, kinematic: true, hasPhysicalSession: false, alignment: false });
  await page.locator('#simCanvas').screenshot({ path: testInfo.outputPath('unitree-g1-pose-workspace.png') });

  // The pose workspace poses a kinematic chain. Its root never moves: there is no free base, so a
  // pose write can never be mistaken for standing, falling or locomotion.
  const kinematicPose = await page.evaluate(async () => {
    const sim = window.__robobuddyCi.app.sim.backend;
    const rootBefore = sim.rig.root.position.toArray();
    const before = sim.rig.groups.left_knee_joint.quaternion.toArray();
    await sim.applyAction({ left_knee_joint: 40 });
    const after = sim.rig.groups.left_knee_joint.quaternion.toArray();
    return { rootBefore, rootAfter: sim.rig.root.position.toArray(), before, after };
  });
  expect(kinematicPose.rootAfter, 'a kinematic pose write must never move the root').toEqual(kinematicPose.rootBefore);
  expect(kinematicPose.after, 'the posed joint must really move').not.toEqual(kinematicPose.before);

  // Switching back restores the physical claims, so the labels track the workspace both ways.
  await page.locator('#taskSelect').selectOption(PHYSICAL_TASK);
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 120_000 });
  const returned = await labels();
  expect(returned.capabilityBadge).toBe('browser-mujoco · numerically-verified');
  expect(returned.simulationAuthority).toBe('physics-session');
  expect(returned.physicalSceneId).toBe(physical.physicalSceneId);
  expect(returned.physicalSceneRevision).toBe(physical.physicalSceneRevision);
  expect(returned.modelPackageId).toBe(physical.modelPackageId);
  expect(returned).toMatchObject({ physical: true, kinematic: false, hasPhysicalSession: true, alignment: true });
  const restored = await frame(page);
  expectRegistered(restored, 'after returning to the physical workspace');
  expect(restored.simulationTimeSeconds, 'the returned physical workspace starts from its own reset state').toBe(0);
  expect(restored.evaluation.standing, 'a fresh physical workspace makes no standing claim').toBe(false);

  await testInfo.attach('workspace-switch-labels', { body: JSON.stringify({ physical, pose, kinematicPose, returned }, null, 2), contentType: 'application/json' });
  expect(pageErrors, `page errors: ${pageErrors.join('\n')}`).toEqual([]);
});
