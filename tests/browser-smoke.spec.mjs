import { test, expect } from '@playwright/test';

const TASK_PATCH = '75fe2669c0ab0b029986de424c69162071174df8';

test('IDE loads canonical robots and pinned reviewed tasks without runtime exceptions', async ({ page }) => {
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));

  await page.goto('/?ci=1', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
  await expect(page.locator('#taskSelect')).toHaveValue('openarm-04-filtration-workcell');
  await expect(page.locator('#taskPanel')).toContainText('Bimanual Heater and Ring-Stand Stack');

  for (const profile of ['so101', 'lekiwi', 'openarm']) {
    await page.locator('#robotSelect').selectOption(profile);
    await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
    await expect(page.locator('#simActionLabel')).toHaveText('Ready');
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

    // Explicitly construct all three canonical rigs in-browser. This protects
    // against constructor-order errors that static syntax tests cannot catch.
    for (const profileId of ['openarm', 'so101', 'lekiwi']) {
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
  for (const profile of ['openarm', 'so101', 'lekiwi']) {
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