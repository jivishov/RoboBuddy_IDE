import { expect, test } from '@playwright/test';
import { OPENARM_MODEL_SHA256 } from '../src/physics/openarm-generated.js';
import { writeFile } from 'node:fs/promises';

test('OpenArm V2 Phase 5A uses one MuJoCo authority for both arms, free vessels, Python, renderer, evaluator, and WebMCP', async ({ page }, testInfo) => {
  test.setTimeout(240_000);
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await page.goto('/?ci=openarm-phase5a', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
  await page.locator('#robotSelect').selectOption('openarm');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });

  const startupUi = {
    taskValue: await page.locator('#taskSelect').inputValue(),
    taskPanelText: await page.locator('#taskPanel').textContent(),
    modeChipText: await page.locator('#modeChip').textContent(),
    physicsBadgeText: await page.locator('#physicsBackendBadge').textContent(),
    stepDisabled: await page.locator('#stepBtn').isDisabled(),
    cursorDisabled: await page.locator('#cursorBtn').isDisabled(),
    visualSource: await page.locator('#simCanvas').getAttribute('data-openarm-visual-source'),
    legacyBaseYawRendered: await page.locator('#simCanvas').getAttribute('data-openarm-legacy-base-yaw-rendered'),
    canvasBackend: await page.locator('#simCanvas').getAttribute('data-simulator-backend'),
    canvasAuthority: await page.locator('#simCanvas').getAttribute('data-simulation-authority'),
    runtime: await page.evaluate(() => {
      const app = window.__robobuddyCi?.app;
      const backend = app?.sim?.backend;
      return {
        appPresent: Boolean(app),
        backendName: backend?.constructor?.name || null,
        hasPresentationAudit: typeof backend?.getPresentationAudit === 'function',
        authority: app?.sim?.getPhysicalAuthorityToken?.() || null,
        model: app?.sim?.getState?.()?.observation?.model || null,
      };
    }),
    pageErrors: [...pageErrors],
  };
  const startupPath = testInfo.outputPath('openarm-startup-audit.json');
  await writeFile(startupPath, JSON.stringify(startupUi, null, 2));
  await testInfo.attach('openarm-startup-audit.json', { path: startupPath, contentType: 'application/json' });

  await expect(page.locator('#taskSelect')).toHaveValue('openarm-04-filtration-workcell');
  await expect(page.locator('#taskPanel')).toContainText('Bimanual Heater and Ring-Stand Stack');
  await expect(page.locator('#modeChip')).toContainText('OPENARM V2 PHYSICAL WORKSPACE');
  await expect(page.locator('#physicsBackendBadge')).toContainText('browser-mujoco');
  await expect(page.locator('#stepBtn')).toBeDisabled();
  await expect(page.locator('#cursorBtn')).toBeDisabled();
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-openarm-visual-source', 'shared-source-collision-geometry-v3');
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-openarm-legacy-base-yaw-rendered', 'false');

  const initial = await page.evaluate(() => {
    const app = window.__robobuddyCi.app;
    const state = app.sim.getState();
    return {
      backend: app.sim.backend?.constructor?.name,
      authority: app.sim.getPhysicalAuthorityToken(),
      presentation: app.sim.backend.getPresentationAudit(),
      source: app.files['main.py'],
      trajectories: app.files['trajectories.py'],
      model: state?.observation?.model,
      flask: state?.observation?.bodies?.flask,
      beaker: state?.observation?.bodies?.beaker,
      leftBase: state?.observation?.bodies?.openarm_left_base_link?.positionM,
      rightBase: state?.observation?.bodies?.openarm_right_base_link?.positionM,
      leftEe: state?.observation?.bodies?.openarm_left_ee_base_link?.positionM,
      rightEe: state?.observation?.bodies?.openarm_right_ee_base_link?.positionM,
    };
  });
  const initialPath = testInfo.outputPath('openarm-initial-state.json');
  await writeFile(initialPath, JSON.stringify({ ...initial, pageErrors }, null, 2));
  await testInfo.attach('openarm-initial-state.json', { path: initialPath, contentType: 'application/json' });
  expect(initial.backend).toBe('OpenArmPhysicalSimulator');
  expect(initial.authority).toMatchObject({ robotId: 'openarm_v2_bimanual', sceneRevision: 'phase5a-openarm-v2-bimanual-stack-v3' });
  expect(initial.model).toMatchObject({ id: 'robobuddy-openarm-v2-phase5a-v3', asset: 'models/openarm_v2/manipulation.xml' });
  expect(initial.model.sha256).toBe(OPENARM_MODEL_SHA256);
  expect(initial.presentation).toMatchObject({
    physicalAuthority: 'MuJoCo PhysicsSession only',
    jointPresentationSource: 'observed MuJoCo body transforms, including each passive finger',
    sharedCollisionGeometry: true,
    legacyBaseYawControlled: false,
    legacyBaseYawRendered: false,
    observationBatchSteps: 2,
    observationPeriodSeconds: 0.002,
  });
  expect(initial.presentation.geometryCount).toBeGreaterThan(50);
  expect(initial.leftBase).toEqual([0.185, 0.031, 1.34]);
  expect(initial.rightBase).toEqual([0.185, -0.031, 1.34]);
  expect(initial.source).toContain('from robobuddy.sim import connect');
  expect(initial.source).toContain('await robot.send_action');
  expect(initial.source).toContain('await robot.advance');
  expect(initial.source).not.toContain('time.sleep(');
  expect(initial.trajectories).not.toMatch(/\.(?:attach|teleport|move_to|grasp)\s*\(/i);
  expect(initial.flask.linearVelocityMS).toHaveLength(3);
  expect(initial.beaker.angularVelocityRadS).toHaveLength(3);
  expect(initial.flask.positionM[2]).toBeCloseTo(1.092, 3);
  expect(initial.beaker.positionM[2]).toBeCloseTo(1.105, 3);
  expect(initial.leftEe[0]).toBeCloseTo(0.401, 3);
  expect(initial.leftEe[1]).toBeCloseTo(0.1535, 3);
  expect(initial.leftEe[2]).toBeCloseTo(1.14, 3);
  expect(initial.rightEe[0]).toBeCloseTo(0.401, 3);
  expect(initial.rightEe[1]).toBeCloseTo(-0.1535, 3);
  expect(initial.rightEe[2]).toBeCloseTo(1.14, 3);

  await page.screenshot({path:testInfo.outputPath('openarm-workcell.png')});
  await page.locator('#runBtn').click();
  await page.waitForFunction(() => /Run complete|run failed/i.test(document.querySelector('#statusMessage').textContent), null, { timeout: 180_000 });
  const runStatus = await page.locator('#statusMessage').textContent();
  if (!runStatus.includes('Run complete')) await writeFile(testInfo.outputPath('python-run-failure.txt'), await page.locator('body').innerText());
  expect(runStatus).toContain('Run complete');

  const completed = await page.evaluate(() => {
    const app = window.__robobuddyCi.app;
    const backend = app.sim.backend;
    const state = app.sim.getState();
    const beforeRender = state.observation.simulationTimeSeconds;
    for (let index = 0; index < 8; index += 1) backend.renderFrame(index * 16.7);
    const afterRender = app.sim.getState().observation.simulationTimeSeconds;
    const flaskExpected = state.observation.bodies.flask.positionM;
    const flaskVisual = backend.objectMeshes.get('flask').position.toArray();
    return {
      evaluation: app.sim.getTaskEvaluation(),
      authority: app.sim.getPhysicalAuthorityToken(),
      presentation: backend.getPresentationAudit(),
      observation: state.observation,
      contacts: app.sim.getContacts(),
      beforeRender, afterRender,
      flaskExpectedMm: [flaskExpected[0] * 1000, flaskExpected[2] * 1000, -flaskExpected[1] * 1000],
      flaskVisual,
      runtimeActive: app.physicalRuntime.isActive(),
      canvasAuthority: document.querySelector('#simCanvas').dataset.simulationAuthority,
      canvasTaskSuccess: document.querySelector('#simCanvas').dataset.physicalTaskSuccess,
      statusText: document.querySelector('#statusMessage').textContent,
    };
  });
  const completedPath = testInfo.outputPath('openarm-completed-state.json');
  await writeFile(completedPath, JSON.stringify({ ...completed, pageErrors }, null, 2));
  await testInfo.attach('openarm-completed-state.json', { path: completedPath, contentType: 'application/json' });

  await page.screenshot({path:testInfo.outputPath('openarm-transfer-completed.png')});
  expect(completed.evaluation.maximumPenetrationM).toBeLessThan(.002);
  expect(completed.evaluation.palmContactSeen).toBe(false);
  expect(completed.canvasTaskSuccess).toBe('true');
  expect(completed.evaluation.success).toBe(true);
  expect(completed.evaluation.orderViolation).toBe(false);
  for (const object of [completed.evaluation.flask, completed.evaluation.beaker]) {
    expect(object).toMatchObject({ graspSeen: true, liftSeen: true, carrySeen: true, supportWhileHeldSeen: true, releaseSeen: true, settled: true, retreated: true, currentGripperContact: false, currentSupportContact: true });
    expect(object.bilateralContactObservationCount).toBeGreaterThan(0);
    expect(object.maxHeldHorizontalTravelM).toBeGreaterThanOrEqual(0.06);
    expect(object.settleEvidenceDurationSeconds).toBeGreaterThanOrEqual(0.20);
    expect(object.settleDriftM).toBeLessThanOrEqual(0.001);
    expect(object.supportWhileHeldTimeSeconds).toBeLessThan(object.releaseTimeSeconds);
    expect(object.releaseTimeSeconds).toBeLessThan(object.settleTimeSeconds);
    expect(object.settleTimeSeconds).toBeLessThan(object.retreatTimeSeconds);
  }
  expect(completed.presentation.legacyBaseYawControlled).toBe(false);
  expect(completed.canvasAuthority).toBe('physics-session');
  expect(completed.runtimeActive).toBe(false);
  expect(completed.afterRender).toBeCloseTo(completed.beforeRender, 12);
  completed.flaskVisual.forEach((value, index) => expect(value).toBeCloseTo(completed.flaskExpectedMm[index], 5));

  const webmcp = await page.evaluate(async () => {
    const app = window.__robobuddyCi.app;
    const { executeOpenArmPhysicalControl, WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION } = await import('/src/webmcp/openarm-physical-control.js');
    const facade = { app, activeControlId: null, controlSequence: 0, assertActive: () => {}, getRegistrationContext: () => app.getAgentRegistrationContext() };
    const before = app.sim.getPhysicalAuthorityToken();
    const result = await executeOpenArmPhysicalControl(facade, {
      schema_version: WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION,
      command: 'set_joint_targets',
      targets_rad: { openarm_left_joint1: -0.2 },
      max_steps: 20,
      advance_seconds: 0,
    }, new AbortController().signal, 1);
    return { before, after: app.sim.getPhysicalAuthorityToken(), result };
  });
  expect(webmcp.result.commandStatus).toBe('accepted');
  expect(webmcp.result.acceptedTargetsRad).toEqual({ openarm_left_joint1: -0.2 });
  expect(Math.abs(webmcp.result.observedState.jointsRad.openarm_left_joint1 + 0.2)).toBeGreaterThan(0.01);
  expect(webmcp.after.sessionId).toBe(webmcp.before.sessionId);
  expect(webmcp.after.epoch).toBe(webmcp.before.epoch);

  const stale = await page.evaluate(async () => {
    const app = window.__robobuddyCi.app;
    const { LivePythonBridge } = await import('/src/runtime/live-python-bridge.js');
    const bridge = new LivePythonBridge(app.sim.getPhysicalSession());
    await bridge.connect('openarm_v2_bimanual');
    const before = app.sim.getPhysicalAuthorityToken();
    await app.resetSimulation();
    let code = null;
    try { await bridge.sendAction({ openarm_left_joint1: -0.3 }, { maxSteps: 10 }); } catch (error) { code = error.code || null; }
    return { before, after: app.sim.getPhysicalAuthorityToken(), code, evaluation: app.sim.getTaskEvaluation() };
  });
  expect(stale.after.epoch).toBeGreaterThan(stale.before.epoch);
  expect(stale.code).toBe('STALE_LIVE_SESSION');
  expect(stale.evaluation.success).toBe(false);
  expect(pageErrors, pageErrors.join('\n\n')).toEqual([]);
});
