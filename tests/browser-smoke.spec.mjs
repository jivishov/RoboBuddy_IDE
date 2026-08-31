import { test, expect } from '@playwright/test';

const TASK_PATCH = '75fe2669c0ab0b029986de424c69162071174df8';
const SOURCE_REPLAY_TIMEOUT = 180_000;

test('IDE loads canonical robots and pinned reviewed tasks without runtime exceptions', async ({ page }) => {
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));

  await page.goto('/?ci=1', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
  await expect(page.locator('#taskSelect')).toHaveValue('openarm-04-filtration-workcell');
  await expect(page.locator('#taskPanel')).toContainText('Bimanual Heater and Ring-Stand Stack');

  for (const profile of ['so101', 'lekiwi', 'openarm', 'unitree']) {
    await page.locator('#robotSelect').selectOption(profile);
    await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
    await expect(page.locator('#simActionLabel')).toHaveText('Ready');
  }

  expect(pageErrors, pageErrors.join('\n\n')).toEqual([]);
});

test('Help opens and closes the accessible About dialog with the non-commercial license notice', async ({ page }) => {
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));

  await page.goto('/?ci=about', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
  await page.locator('[data-menu="help"]').click();
  await expect(page.locator('#helpMenu')).toBeVisible();
  await page.locator('[data-action="about"]').click();

  const dialog = page.locator('#aboutDialog');
  await expect(dialog).toBeVisible();
  await expect(dialog).toHaveAttribute('open', '');
  await expect(dialog).toContainText('RoboBuddy IDE');
  await expect(dialog).toContainText('© 2026 Dr. Emil Jivishov');
  await expect(dialog).toContainText('PolyForm Noncommercial 1.0.0');
  await expect(dialog).toContainText('Commercial use requires prior written permission.');
  await expect(page.locator('#aboutCloseBtn')).toBeFocused();

  await page.keyboard.press('Escape');
  await expect(dialog).not.toHaveAttribute('open', '');
  expect(pageErrors).toEqual([]);
});

test('high-contrast scene boundaries are default-on presentation aids for every pinned workcell', async ({ page }) => {
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await page.goto('/?ci=contrast', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });

  for (const profile of ['openarm', 'so101', 'lekiwi']) {
    await page.locator('#robotSelect').selectOption(profile);
    await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
    const on = await page.evaluate(() => ({
      pressed: document.getElementById('highContrastSceneBtn').getAttribute('aria-pressed'),
      active: document.getElementById('simCanvas').dataset.highContrastScene,
      floor: document.getElementById('simCanvas').dataset.presentationGroundColor,
      perimeters: Number(document.getElementById('simCanvas').dataset.highContrastPerimeterCount),
    }));
    expect(on).toEqual({ pressed: 'true', active: 'true', floor: '#687378', perimeters: expect.any(Number) });
    expect(on.perimeters).toBeGreaterThan(0);

    const clockBefore = await page.locator('#simCanvas').getAttribute('data-simulation-clock-s');
    await page.locator('#highContrastSceneBtn').click();
    await expect(page.locator('#highContrastSceneBtn')).toHaveAttribute('aria-pressed', 'false');
    await expect(page.locator('#simCanvas')).toHaveAttribute('data-high-contrast-scene', 'false');
    await expect(page.locator('#simCanvas')).toHaveAttribute('data-presentation-ground-color', '#687378');
    await expect(page.locator('#simCanvas')).toHaveAttribute('data-simulation-clock-s', clockBefore || '0');
    await page.locator('#highContrastSceneBtn').click();
    await expect(page.locator('#highContrastSceneBtn')).toHaveAttribute('aria-pressed', 'true');
    await expect(page.locator('#simCanvas')).toHaveAttribute('data-high-contrast-scene', 'true');
  }

  expect(pageErrors, pageErrors.join('\n\n')).toEqual([]);
});

test('all pinned reference traces run through the source fixed-step plant collision-free', async ({ page }) => {
  await page.goto('/?ci=1', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });

  const report = await page.evaluate(async ({ revision }) => {
    const catalog = await import('/src/task-catalog.js');
    const { CanonicalRobotRig } = await import('/src/canonical-rig.js');
    const { ScenarioV2Engine } = await import(`https://cdn.jsdelivr.net/gh/jivishov/RoboBuddy_AI@${revision}/lab/v2/scenario-engine.js`);
    const results = [];

    // Explicitly construct all four canonical rigs in-browser. This protects
    // against constructor-order errors that static syntax tests cannot catch.
    for (const profileId of ['openarm', 'so101', 'lekiwi', 'unitree']) {
      const rig = await CanonicalRobotRig.load(profileId);
      if (!rig?.root?.isObject3D) throw new Error(`${profileId}: canonical root was not constructed`);
      rig.root.updateMatrixWorld(true);
      rig.dispose();
    }

    for (const [profileId, descriptors] of Object.entries(catalog.PATCH_TASKS)) {
      for (const descriptor of descriptors) {
        const scenario = await catalog.loadPatchedScenario(profileId, descriptor.id);
        const engine = await ScenarioV2Engine.create(scenario, { autoStartPlant: false });
        const instanceId = `browser-${profileId}`;
        const connectionConfig = profileId === 'openarm'
          ? { kind: 'bimanual', side: 'bimanual', cameras: {} }
          : { cameras: {} };
        engine.plant.connect(instanceId, connectionConfig);
        let ticks = 0;
        for (const [index, record] of scenario.portablePython.referenceActions.entries()) {
          engine.plant.sendAction(instanceId, record.action, {});
          const count = Math.max(1, Math.ceil(Number(record.hold_seconds || 0) / engine.plant.tickSeconds));
          for (let tick = 0; tick < count; tick += 1) {
            engine.plant.tick();
            ticks += 1;
            if (engine.plant.fault) {
              throw new Error(`${scenario.id} action ${index + 1} (${record.label}): ${JSON.stringify(engine.plant.fault)}`);
            }
          }
        }
        results.push({ profileId, scenarioId: scenario.id, actions: scenario.portablePython.referenceActions.length, ticks });
        engine.plant.dispose();
      }
    }
    return results;
  }, { revision: TASK_PATCH });

  expect(report.map((item) => item.scenarioId)).toEqual([
    'openarm-04-filtration-workcell',
    'so101-v2-06-quantitative-transfer',
    'so101-v2-08-burette-initial-reading',
    'so101-v2-09-vacuum-filtration',
    'lekiwi-01-beaker-courier',
  ]);
  expect(report.every((item) => item.actions > 1 && item.ticks >= item.actions)).toBeTruthy();
});

test('Unitree G1 loads the source-pinned 29-joint mesh as a truthful kinematic pose workspace', async ({ page }) => {
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await page.goto('/?ci=unitree', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
  await page.locator('#robotSelect').selectOption('unitree');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
  await expect(page.locator('#taskSelect')).toHaveValue('unitree-g1-kinematic-pose-inspection');
  await expect(page.locator('#taskPanel')).toContainText('Unitree G1 29-DoF Kinematic Pose Inspection');
  await expect(page.locator('#modeChip')).toContainText('KINEMATIC POSE RIG');
  await expect(page.locator('#simBadge')).toContainText('NO CONTACT PLANT');

  const rig = await page.evaluate(async () => {
    const { CanonicalRobotRig } = await import('/src/canonical-rig.js');
    const { validateAction } = await import('/src/profiles.js');
    const instance = await CanonicalRobotRig.load('unitree');
    const result = {
      robotId: instance.meshData.robotId,
      joints: instance.meshData.chain.length,
      parts: instance.meshData.parts.length,
      groundOffsetMm: instance.meshData.groundOffsetMm,
      valid: validateAction('unitree', { left_knee_joint: 20 }).left_knee_joint,
      rejected: false,
    };
    try { validateAction('unitree', { left_knee_joint: 999 }); }
    catch { result.rejected = true; }
    instance.dispose();
    return result;
  });
  expect(rig).toEqual({ robotId: 'unitree_g1_29dof', joints: 29, parts: 36, groundOffsetMm: 792.266, valid: 20, rejected: true });

  await page.locator('#stepBtn').click();
  await expect(page.locator('#statusMessage')).toContainText('Stepped A01', { timeout: 90_000 });
  await page.locator('[data-side-view="robot"]').click();
  await expect(page.locator('#robotSideView')).toBeVisible();
  await page.locator('[data-side-action="telemetry"]').click();
  await expect(page.locator('#telemetryPanel')).toContainText('BROWSER-HELD KINEMATIC G1 JOINT STATE');
  await expect(page.locator('#telemetryPanel')).toContainText('waist_pitch_joint');
  await page.locator('[data-side-action="contacts"]').click();
  await expect(page.locator('#contactsPanel')).toContainText('NOT SIMULATED');
  expect(pageErrors, pageErrors.join('\n\n')).toEqual([]);
});

test('learner Python reaches the first physical action through the IDE Step Action path', async ({ page }) => {
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await page.goto('/?ci=1', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
  await page.locator('#stepBtn').click();
  await expect(page.locator('#statusMessage')).toContainText('Stepped A01', { timeout: 90_000 });
  await expect(page.locator('#simActionLabel')).toContainText('A01');
  await expect(page.locator('#problemsPanel')).not.toContainText('COLLISION');
  expect(pageErrors, pageErrors.join('\n\n')).toEqual([]);
});


test('Pause holds an active source-plant run and resumes it in place', async ({ page }) => {
  test.setTimeout(240_000);
  await page.goto('/?ci=pause', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
  expect(await page.locator('#runBtn').evaluate((button) => button.nextElementSibling?.id)).toBe('pauseBtn');

  await page.locator('#runBtn').click();
  await expect(page.locator('#pauseBtn')).toBeEnabled();
  await expect(page.locator('#simActionLabel')).toContainText('A01', { timeout: 90_000 });
  await page.locator('#pauseBtn').click();
  await expect(page.locator('#statusMessage')).toHaveText('Simulation paused');
  await expect(page.locator('#pauseBtn')).toHaveText('▶ Resume');
  await expect(page.locator('#pauseBtn')).toHaveAttribute('aria-pressed', 'true');

  const pausedAction = await page.locator('#simActionLabel').textContent();
  const pausedClock = await page.locator('#simCanvas').getAttribute('data-simulation-clock-s');
  await page.waitForTimeout(250);
  await expect(page.locator('#simActionLabel')).toHaveText(pausedAction || '');
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-simulation-clock-s', pausedClock || '0');

  await page.locator('#pauseBtn').click();
  await expect(page.locator('#pauseBtn')).toHaveText('⏸ Pause');
  await expect(page.locator('#pauseBtn')).toHaveAttribute('aria-pressed', 'false');
  await expect(page.locator('#statusMessage')).toHaveText('Run complete', { timeout: SOURCE_REPLAY_TIMEOUT });
  await expect(page.locator('#pauseBtn')).toBeDisabled();
});

test('source-plant and Unitree keep their main-thread compile/replay Run and Run-to-Cursor paths', async ({ page }) => {
  test.setTimeout(240_000);
  await page.goto('/?ci=cycle04-preservation', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
  const setFirstActionCursor = () => page.evaluate(() => {
    const app = window.__robobuddyCi.app;
    const lines = app.files['main.py'].split('\n');
    const index = lines.findIndex((line) => line.includes('robot.send_action('));
    app.openFile('main.py');
    app.editor.cm.setCursor({ line: Math.max(0, index), ch: 0 });
    return index + 1;
  });

  const sourceLine = await setFirstActionCursor();
  await page.click('#cursorBtn');
  await expect(page.locator('#statusMessage')).toContainText(`main.py:${sourceLine}`, { timeout: SOURCE_REPLAY_TIMEOUT });
  expect(await page.evaluate(() => ({ policyWorker: window.__robobuddyCi.app.microduckRuntime.isActive(), prepared: window.__robobuddyCi.app.prepared?.events?.length > 0 }))).toEqual({ policyWorker: false, prepared: true });

  await page.selectOption('#robotSelect', 'unitree');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
  await page.click('#runBtn');
  await expect(page.locator('#statusMessage')).toHaveText('Run complete', { timeout: SOURCE_REPLAY_TIMEOUT });
  const unitreeLine = await setFirstActionCursor();
  await page.click('#cursorBtn');
  await expect(page.locator('#statusMessage')).toContainText(`main.py:${unitreeLine}`, { timeout: SOURCE_REPLAY_TIMEOUT });
  expect(await page.evaluate(() => ({ policyWorker: window.__robobuddyCi.app.microduckRuntime.isActive(), prepared: window.__robobuddyCi.app.prepared?.events?.length > 0 }))).toEqual({ policyWorker: false, prepared: true });
});

test('diagnostics panel owns a full-width grid row and activity rail buttons are functional', async ({ page }) => {
  await page.setViewportSize({ width: 1366, height: 768 });
  await page.goto('/?ci=ux', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });

  await page.locator('[data-side-view="task"]').click();
  await expect(page.locator('#taskSideView')).toBeVisible();
  await page.locator('[data-side-view="robot"]').click();
  await expect(page.locator('#robotSideView')).toBeVisible();
  await page.locator('[data-side-action="telemetry"]').click();
  await expect(page.locator('#telemetryPanel')).toBeVisible();

  const layout = await page.evaluate(() => {
    const panel = document.getElementById('bottomPanel').getBoundingClientRect();
    const workspace = document.getElementById('workspace').getBoundingClientRect();
    const editor = document.querySelector('.editor-pane').getBoundingClientRect();
    return { panelLeft: panel.left, panelRight: panel.right, panelTop: panel.top, workspaceBottom: workspace.bottom, editorBottom: editor.bottom, viewportWidth: innerWidth };
  });
  expect(layout.panelLeft).toBeLessThanOrEqual(1);
  expect(layout.panelRight).toBeGreaterThanOrEqual(layout.viewportWidth - 1);
  expect(layout.editorBottom).toBeLessThanOrEqual(layout.panelTop + 1);
  expect(Math.abs(layout.workspaceBottom - layout.panelTop)).toBeLessThanOrEqual(1);

  await page.setViewportSize({ width: 390, height: 740 });
  await page.locator('[data-side-view="task"]').click();
  await expect(page.locator('#taskSideView')).toBeVisible();
  await page.locator('#bottomClose').click();
  await page.keyboard.press('Control+J');
  await expect(page.locator('#problemsPanel')).toBeVisible();
  const mobile = await page.evaluate(() => {
    const panel = document.getElementById('bottomPanel').getBoundingClientRect();
    const editor = document.querySelector('.editor-pane').getBoundingClientRect();
    return { panelLeft: panel.left, panelRight: panel.right, panelTop: panel.top, editorBottom: editor.bottom, viewportWidth: innerWidth };
  });
  expect(mobile.panelLeft).toBeLessThanOrEqual(1);
  expect(mobile.panelRight).toBeGreaterThanOrEqual(mobile.viewportWidth - 1);
  expect(mobile.editorBottom).toBeLessThanOrEqual(mobile.panelTop + 1);
});

test('load, Fit, and Reset use pinned canonical front-view directions for every robot', async ({ page }) => {
  await page.goto('/?ci=front', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
  const presets = await page.evaluate(async () => {
    const module = await import('/src/source-simulator.js');
    return module.FRONT_CAMERA_PRESETS;
  });
  expect(presets.so101).toEqual({ position: [540, 410, 720], target: [140, 105, -35] });
  expect(presets.lekiwi).toEqual({ position: [500, 350, -150], target: [-60, 125, 45] });
  expect(presets.lekiwi.position[0]).toBeGreaterThan(presets.lekiwi.target[0]);
  expect(presets.lekiwi.position[2]).toBeLessThan(presets.lekiwi.target[2]);
  expect(presets.openarm).toEqual({ position: [1830, 820, 0], target: [140, 365, 0] });
  expect(presets.openarm.position[0]).toBeGreaterThan(presets.openarm.target[0]);
  expect(presets.unitree).toEqual({ position: [1950, 1180, 1650], target: [150, 660, 0] });
  for (const profile of ['openarm', 'so101', 'lekiwi', 'unitree']) {
    await page.locator('#robotSelect').selectOption(profile);
    await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
    await expect(page.locator('#simCanvas')).toHaveAttribute('data-camera-view', 'front');
    await page.locator('#fitBtn').click();
    await expect(page.locator('#simCanvas')).toHaveAttribute('data-camera-view', 'front');
    await page.locator('#resetBtn').click();
    await expect(page.locator('#statusMessage')).toContainText('Simulation reset', { timeout: 45_000 });
    await expect(page.locator('#simCanvas')).toHaveAttribute('data-camera-view', 'front');
  }
});
