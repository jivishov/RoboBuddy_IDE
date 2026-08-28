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
