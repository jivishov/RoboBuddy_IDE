import { expect, test } from '@playwright/test';

// The visible browser journey for the MicroDuck Phase 5C physical workspace.
//
// It proves the things that cannot be proved in Node: that the workspace loads a real browser
// MuJoCo session, that locomotion comes out of foot-floor contact rather than a movement
// command, that removing actuation or traction removes the propulsion, that the renderer
// follows the authoritative trunk instead of re-seating the robot on the floor, and that the
// unsupported roller capabilities are refused rather than routed anywhere.

const CI_URL = '/?ci=microduck-phase5c';

async function selectPhysicalMicroDuck(page) {
  await page.goto(CI_URL, { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
  await page.locator('#robotSelect').selectOption('microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 120_000 });
  await page.locator('#taskSelect').selectOption('microduck-physical-locomotion');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 120_000 });
}

test('MicroDuck physical workspace walks through contact and loses propulsion without actuation or traction', async ({ page }) => {
  test.setTimeout(360_000);
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await selectPhysicalMicroDuck(page);

  // --- the workspace is a real physical session, labelled as one -----------------
  const identity = await page.evaluate(() => {
    const sim = window.__robobuddyCi.app.sim.backend;
    const canvas = document.querySelector('canvas');
    return {
      backend: canvas.dataset.simulatorBackend,
      authorityDataset: canvas.dataset.simulationAuthority,
      sceneId: canvas.dataset.physicalSceneId,
      modelPackageId: canvas.dataset.modelPackageId,
      workspace: canvas.dataset.microduckWorkspace,
      controllerHz: canvas.dataset.microduckControllerHz,
      rootAuthority: canvas.dataset.microduckRootAuthority,
      unsupported: canvas.dataset.microduckUnsupportedPhysicalSkills,
      authority: sim.getPhysicalAuthorityToken(),
      audit: sim.getPresentationAudit(),
      state: sim.getState(),
    };
  });
  expect(identity.backend).toBe('browser-mujoco');
  expect(identity.authorityDataset).toBe('physics-session');
  expect(identity.sceneId).toBe('microduck-physical-walk');
  expect(identity.modelPackageId).toBe('microduck-walk-519142b-v1');
  expect(identity.workspace).toBe('physical');
  expect(identity.controllerHz).toBe('50');
  expect(identity.rootAuthority).toBe('mujoco-free-body');
  expect(identity.unsupported.split(',').sort()).toEqual(['roller', 'roller_crouch']);
  expect(identity.authority?.sessionId).toBeTruthy();
  expect(identity.audit.rendererFloorSnapping).toBe(false);
  expect(identity.audit.rendererIntegratesRoot).toBe(false);
  expect(identity.audit.rootTransformSource).toContain('trunk_base free-body pose');
  expect(identity.state.capability.backend).toBe('browser-mujoco');
  expect(identity.state.capability.evidence).toBe('numerically-verified');
  expect(identity.state.hardwareValidated).toBe(false);
  // It stands on its own feet before anything is commanded.
  expect(identity.state.actual.trunkPositionM[2]).toBeGreaterThan(0.10);
  expect(identity.state.actual.trunkTiltDeg).toBeLessThan(5);

  // --- nominal walking is contact-driven ----------------------------------------
  const walk = await page.evaluate(async () => {
    const sim = window.__robobuddyCi.app.sim.backend;
    await sim.reset();
    sim.setCommand({ vx: 0.35 });
    for (let i = 0; i < 3; i += 1) await sim.advanceSeconds(2);
    sim.renderFrame();
    const rig = sim.rig;
    return {
      verdict: sim.locomotionVerdict(),
      state: sim.getState(),
      rigRootMm: rig ? [rig.root.position.x, rig.root.position.y, rig.root.position.z] : null,
    };
  });
  expect(walk.verdict.walked, `locomotion verdict: ${JSON.stringify(walk.verdict.reasons)}`).toBe(true);
  expect(walk.verdict.report.commandedAxisDistanceM).toBeGreaterThan(0.2);
  // Alternating foot contact, not a slide.
  expect(walk.verdict.report.contacts.leftContactTransitions).toBeGreaterThan(6);
  expect(walk.verdict.report.contacts.rightContactTransitions).toBeGreaterThan(6);
  expect(walk.verdict.report.fallen).toBe(false);
  // The controller ran the matched walking policy at the deployed walking scale.
  expect(walk.state.controller.policyId).toBe('walking');
  expect(walk.state.controller.actionScale).toBeCloseTo(0.9, 6);
  // The renderer followed the authoritative trunk: MuJoCo metres, Three.js millimetres.
  expect(walk.rigRootMm).not.toBeNull();
  expect(walk.rigRootMm[0] / 1000).toBeCloseTo(walk.state.actual.trunkPositionM[0], 3);
  expect(walk.rigRootMm[1] / 1000).toBeCloseTo(walk.state.actual.trunkPositionM[2], 3);
  expect(walk.rigRootMm[2] / 1000).toBeCloseTo(-walk.state.actual.trunkPositionM[1], 3);
  const nominal = walk.verdict.report.commandedAxisDistanceM;

  // The deployed servo gain reached the physics, and the servos were working. `controller`
  // reports the gain that was asked for; `actual` reports what MuJoCo applied. If the model
  // write were a silent no-op in the WASM build, these would disagree and the force below
  // would be unchanged by any gain at all.
  expect(walk.state.controller.firmwareGain).toBe(200);
  expect(walk.state.actual.firmwareGain).toBe(200);
  expect(walk.state.actual.appliedServoKp).toBeCloseTo(0.55, 10);
  expect(walk.state.actual.actuatorForceTotalNm).toBeGreaterThan(0);

  // --- actuation disabled: the propulsion mechanism is gone ---------------------
  const noActuation = await page.evaluate(async () => {
    const sim = window.__robobuddyCi.app.sim.backend;
    await sim.reset();
    await sim.setActuationEnabled(false);
    sim.setCommand({ vx: 0.35 });
    for (let i = 0; i < 3; i += 1) await sim.advanceSeconds(2);
    return { report: sim.report(), state: sim.getState() };
  });
  expect(noActuation.state.actual.actuationEnabled).toBe(false);
  // The declared torque-off condition is logged as setup, so a run can never look nominal.
  expect(noActuation.state.actual.setupLog.some((item) => item.event === 'setup_actuation')).toBe(true);
  expect(noActuation.report.fallen).toBe(true);
  expect(noActuation.report.finalTrunkHeightM).toBeLessThan(0.07);
  expect(Math.abs(noActuation.report.commandedAxisDistanceM)).toBeLessThan(nominal * 0.4);

  // A torque-off must produce NO actuator force. The controller still emits position
  // targets, but the BAM motor law is disabled and the MuJoCo motor torque is zero. Reporting
  // zero effective firmware gain and zero force makes this a true negative control rather
  // than a differently-commanded positive run.
  expect(noActuation.state.actual.firmwareGain).toBe(0);
  expect(noActuation.state.actual.appliedServoKp).toBe(0);
  expect(noActuation.state.actual.actuatorForceTotalNm).toBe(0);
  // The controller kept asking for targets throughout: a disabled servo is not a disabled policy.
  expect(Object.values(noActuation.state.actual.jointTargetRad).some((value) => value !== 0)).toBe(true);

  await expect.poll(() => pageErrors.length, { timeout: 1_000 }).toBe(0);
});

test('MicroDuck physical traction control, unsupported roller rejection, kick contact and honest recovery failure', async ({ page }) => {
  test.setTimeout(420_000);
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await selectPhysicalMicroDuck(page);

  // --- traction reduced: commanded propulsion materially degrades ---------------
  const traction = await page.evaluate(async () => {
    const sim = window.__robobuddyCi.app.sim.backend;
    await sim.reset();
    sim.setCommand({ vx: 0.35 });
    for (let i = 0; i < 3; i += 1) await sim.advanceSeconds(2);
    const nominal = sim.report();
    await sim.load('lowTraction');
    sim.setCommand({ vx: 0.35 });
    for (let i = 0; i < 3; i += 1) await sim.advanceSeconds(2);
    const degraded = sim.report();
    await sim.load('walk');
    return { nominal, degraded };
  });
  expect(traction.nominal.commandedAxisDistanceM).toBeGreaterThan(0.2);
  // At least a 60% loss of commanded-axis progress on the declared degraded surface.
  expect(traction.degraded.commandedAxisDistanceM).toBeLessThan(traction.nominal.commandedAxisDistanceM * 0.4);

  // --- unsupported capabilities are refused, not routed anywhere ----------------
  const capabilities = await page.evaluate(() => {
    const sim = window.__robobuddyCi.app.sim.backend;
    return {
      roller: sim.requestSkill('roller'),
      rollerCrouch: sim.requestSkill('roller_crouch'),
      kickRight: sim.requestSkill('kick_right'),
      table: sim.getState().capabilities,
    };
  });
  expect(capabilities.roller.accepted).toBe(false);
  expect(capabilities.roller.status).toBe('unsupported');
  expect(capabilities.rollerCrouch.accepted).toBe(false);
  expect(capabilities.kickRight.accepted).toBe(false);
  expect(capabilities.kickRight.status).toBe('wrong-plant');
  expect(capabilities.kickRight.requiredPackageKeys).toContain('kick');
  expect(capabilities.table.find((item) => item.id === 'roller').physical).toBe(false);
  expect(capabilities.table.find((item) => item.id === 'walk').physical).toBe(true);

  // --- a kick needs actual foot-ball contact, and a miss stays a miss ------------
  const kick = await page.evaluate(async () => {
    const sim = window.__robobuddyCi.app.sim.backend;
    await sim.load('kick');
    await sim.placeBall([0.09, -0.042, 0.035], 'declared kick placement');
    sim.requestSkill('kick_right');
    for (let i = 0; i < 2; i += 1) await sim.advanceSeconds(1.5);
    const hit = sim.report();
    await sim.reset();
    await sim.placeBall([0.55, -0.042, 0.035], 'declared out-of-reach placement');
    sim.requestSkill('kick_right');
    for (let i = 0; i < 2; i += 1) await sim.advanceSeconds(1.5);
    const miss = sim.report();
    await sim.load('walk');
    return { hit, miss };
  });
  expect(kick.hit.contacts.footBallContacts.length).toBeGreaterThan(0);
  expect(kick.hit.contacts.footBallContacts[0].geoms.join(' ')).toContain('microduck_ball_geom');
  expect(kick.hit.ball.movedByFootContact).toBe(true);
  expect(kick.hit.ball.planarDistanceM).toBeGreaterThan(0.3);
  // The miss: no contact, and therefore no ball motion attributed to a kick.
  expect(kick.miss.contacts.footBallContacts.length).toBe(0);
  expect(kick.miss.ball.planarDistanceM).toBeLessThan(1e-3);
  expect(kick.miss.ball.movedByFootContact).toBe(false);

  // --- recovery is physical, and can fail honestly ------------------------------
  const recovery = await page.evaluate(async () => {
    const sim = window.__robobuddyCi.app.sim.backend;
    await sim.load('groundContact');
    await sim.reset();
    await sim.applyPerturbation('face_down');
    await sim.settle(1.5);
    const settled = sim.getState().actual;
    // Zero velocity on the broad ground-contact plant selects the deployed stand policy,
    // which is the pinned fall-recovery reference. Do not substitute the sit/stand rise alias.
    for (let i = 0; i < 4; i += 1) await sim.advanceSeconds(2);
    const recovered = sim.report();
    const recoveryPolicy = sim.getState().controller?.policyId ?? null;

    await sim.reset();
    await sim.applyPerturbation('face_down');
    await sim.settle(1.5);
    await sim.setActuationEnabled(false);
    for (let i = 0; i < 4; i += 1) await sim.advanceSeconds(2);
    const failed = sim.report();
    return { settled, recovered, recoveryPolicy, failed };
  });
  // The perturbation is declared setup, visible in the observation, and put the robot down.
  expect(recovery.settled.setupLog.some((item) => item.event === 'setup_trunk_orientation')).toBe(true);
  expect(recovery.settled.trunkPositionM[2]).toBeLessThan(0.07);
  expect(recovery.settled.trunkTiltDeg).toBeGreaterThan(60);
  // Physical recovery, through contact, using the pinned stand-policy route.
  expect(recovery.recoveryPolicy).toBe('stand');
  expect(recovery.recovered.upright).toBe(true);
  expect(recovery.recovered.finalTrunkHeightM).toBeGreaterThan(0.10);
  expect(recovery.recovered.everFallen).toBe(true);
  // And an honest failure: no reset is counted as a recovery.
  expect(recovery.failed.upright).toBe(false);
  expect(recovery.failed.finalTrunkHeightM).toBeLessThan(0.08);

  await expect.poll(() => pageErrors.length, { timeout: 1_000 }).toBe(0);
});
