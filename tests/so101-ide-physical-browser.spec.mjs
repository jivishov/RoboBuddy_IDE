import { expect, test } from '@playwright/test';

test('normal SO-101 IDE workspace uses one MuJoCo authority for live Python, renderer, evaluator, and WebMCP', async ({ page }) => {
  test.setTimeout(240_000);
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await page.goto('/?ci=so101-physical-ide', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
  await page.locator('#robotSelect').selectOption('so101');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
  await expect(page.locator('#taskSelect')).toHaveValue('so101-physical-block-transfer');
  await expect(page.locator('#taskPanel')).toContainText('SO-101 Physical Block Transfer');
  await expect(page.locator('#modeChip')).toContainText('MUJOCO AUTHORITY');
  await expect(page.locator('#simBadge')).toContainText('ACTUAL CONTACT/GRAVITY');
  await expect(page.locator('#physicsBackendBadge')).toContainText('browser-mujoco');
  await expect(page.locator('#stepBtn')).toBeDisabled();
  await expect(page.locator('#cursorBtn')).toBeDisabled();

  const initial = await page.evaluate(() => {
    const app = window.__robobuddyCi.app;
    return {
      backend: app.sim.backend?.constructor?.name,
      authority: app.sim.getPhysicalAuthorityToken(),
      source: app.files['main.py'],
      prepared: app.prepared,
    };
  });
  expect(initial.backend).toBe('So101PhysicalSimulator');
  expect(initial.authority).toMatchObject({ robotId: 'so101_follower', sceneRevision: 'p4-so101-benchmark-transfer-v3' });
  expect(initial.source).toContain('from robobuddy.sim import connect');
  expect(initial.source).toContain('await robot.send_action');
  expect(initial.source).toContain('await robot.advance');
  expect(initial.source).not.toContain('time.sleep(');
  expect(initial.prepared).toBeNull();

  await page.locator('#runBtn').click();
  await expect(page.locator('#statusMessage')).toHaveText('Run complete · SO-101 physical task succeeded', { timeout: 150_000 });

  const completed = await page.evaluate(() => {
    const app = window.__robobuddyCi.app;
    const backend = app.sim.backend;
    const state = app.sim.getState();
    const observation = state.observation;
    const p = observation.bodies.benchmark_block.positionM;
    const expectedBlockMm = [p[0] * 1000, p[2] * 1000, -p[1] * 1000];
    const actualBlockMm = backend.blockMesh.position.toArray();
    const beforeRender = observation.simulationTimeSeconds;
    for (let index = 0; index < 12; index += 1) backend.renderFrame(index * 16.7);
    const afterRender = app.sim.getState().observation.simulationTimeSeconds;
    return {
      evaluation: app.sim.getTaskEvaluation(),
      authority: app.sim.getPhysicalAuthorityToken(),
      actualBlockMm,
      expectedBlockMm,
      beforeRender,
      afterRender,
      physicalRuntimeActive: app.physicalRuntime.isActive(),
      prepared: app.prepared,
      canvas: {
        authority: document.querySelector('#simCanvas').dataset.simulationAuthority,
        success: document.querySelector('#simCanvas').dataset.physicalTaskSuccess,
      },
    };
  });
  expect(completed.evaluation).toMatchObject({
    success: true, contactSeen: true, liftSeen: true, carrySeen: true,
    releaseSeen: true, settleSeen: true, inTarget: true,
    currentGripperContact: false, currentTargetSupportContact: true,
  });
  expect(completed.evaluation.maxBlockZM).toBeGreaterThan(0.264);
  expect(completed.evaluation.maxHorizontalTravelM).toBeGreaterThan(0.05);
  expect(completed.evaluation.maxHeldHorizontalTravelM).toBeGreaterThan(0.05);
  expect(completed.evaluation.carriedContactObservationCount).toBeGreaterThanOrEqual(2);
  expect(completed.evaluation.settleEvidenceDurationSeconds).toBeGreaterThanOrEqual(0.20);
  expect(completed.evaluation.settlePositionDriftM).toBeLessThanOrEqual(0.0005);
  expect(completed.evaluation.targetSupportContactObservationCount).toBeGreaterThan(1);
  expect(completed.canvas).toEqual({ authority: 'physics-session', success: 'true' });
  expect(completed.physicalRuntimeActive).toBe(false);
  expect(completed.prepared).toBeNull();
  expect(completed.afterRender).toBeCloseTo(completed.beforeRender, 12);
  completed.actualBlockMm.forEach((value, index) => expect(value).toBeCloseTo(completed.expectedBlockMm[index], 6));

  const webmcp = await page.evaluate(async () => {
    const app = window.__robobuddyCi.app;
    const realFacade = window.__robobuddyCi.agentFacade;
    const { executeProfileControl, WEBMCP_SO101_PHYSICAL_SCHEMA_VERSION } = await import('/src/webmcp/robot-controls.js');
    const facade = {
      app,
      activeControlId: null,
      controlSequence: 0,
      assertActive: () => {},
      getRegistrationContext: () => app.getAgentRegistrationContext(),
      inspectSimulation: (snapshot) => realFacade.inspectSimulation(snapshot),
    };
    const before = app.sim.getPhysicalAuthorityToken();
    const result = await executeProfileControl(facade, 'so101', {
      schema_version: WEBMCP_SO101_PHYSICAL_SCHEMA_VERSION,
      command: 'set_joint_targets',
      targets_rad: { shoulder_pan: 0.2 },
      max_steps: 20,
      advance_seconds: 0,
    }, new AbortController().signal, 1);
    const after = app.sim.getPhysicalAuthorityToken();
    return { before, after, result };
  });
  expect(webmcp.result.commandStatus).toBe('accepted');
  expect(webmcp.result.acceptedTargetsRad).toEqual({ shoulder_pan: 0.2 });
  expect(Math.abs(webmcp.result.observedState.jointsRad.shoulder_pan - 0.2)).toBeGreaterThan(0.02);
  expect(webmcp.after.sessionId).toBe(webmcp.before.sessionId);
  expect(webmcp.after.epoch).toBe(webmcp.before.epoch);
  expect(webmcp.result.physicalAuthority.sessionId).toBe(webmcp.before.sessionId);

  const staleReset = await page.evaluate(async () => {
    const app = window.__robobuddyCi.app;
    const { LivePythonBridge } = await import('/src/runtime/live-python-bridge.js');
    const session = app.sim.getPhysicalSession();
    const bridge = new LivePythonBridge(session);
    await bridge.connect('so101_follower');
    const before = app.sim.getPhysicalAuthorityToken();
    await app.resetSimulation();
    const after = app.sim.getPhysicalAuthorityToken();
    let staleCode = null;
    try { await bridge.sendAction({ shoulder_pan: 0.3 }, { maxSteps: 10 }); }
    catch (error) { staleCode = error.code || null; }
    const state = app.sim.getState();
    return { before, after, staleCode, evaluation: app.sim.getTaskEvaluation(), block: state.observation.bodies.benchmark_block.positionM };
  });
  expect(staleReset.after.sessionId).toBe(staleReset.before.sessionId);
  expect(staleReset.after.epoch).toBeGreaterThan(staleReset.before.epoch);
  expect(staleReset.staleCode).toBe('STALE_LIVE_SESSION');
  expect(staleReset.evaluation.success).toBe(false);
  expect(staleReset.block[0]).toBeCloseTo(0.39416, 4);
  expect(staleReset.block[1]).toBeCloseTo(-0.00169, 4);

  const cancellation = await page.evaluate(async () => {
    const app = window.__robobuddyCi.app;
    const { LivePythonBridge } = await import('/src/runtime/live-python-bridge.js');
    const oldSession = app.sim.getPhysicalSession();
    const bridge = new LivePythonBridge(oldSession);
    await bridge.connect('so101_follower');
    const before = app.sim.getPhysicalAuthorityToken();
    await bridge.cancel('integration-cancel');
    const readyAfterCancel = app.sim.isReady();
    const authorityAfterCancel = app.sim.getPhysicalAuthorityToken();
    await app.resetSimulation();
    const afterReset = app.sim.getPhysicalAuthorityToken();
    let lateCode = null;
    try { await oldSession.sendCommand({ type: 'set_joint_targets', targetsRad: { shoulder_pan: 0.4 } }, { maxSteps: 10 }); }
    catch (error) { lateCode = String(error.message || error); }
    return { before, readyAfterCancel, authorityAfterCancel, afterReset, lateCode, evaluation: app.sim.getTaskEvaluation() };
  });
  expect(cancellation.readyAfterCancel).toBe(false);
  expect(cancellation.authorityAfterCancel).toBeNull();
  expect(cancellation.afterReset.sessionId).not.toBe(cancellation.before.sessionId);
  expect(cancellation.evaluation.success).toBe(false);
  expect(cancellation.lateCode).toContain('disposed');
  expect(pageErrors, pageErrors.join('\n\n')).toEqual([]);
});
