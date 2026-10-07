import { expect, test } from '@playwright/test';

const TASKS = ['microduck-physical-locomotion', 'microduck-physical-groundcontact', 'microduck-physical-kick'];
async function openMicroDuck(page) {
  await page.goto('/?ci=microduck-presentation', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
  await page.locator('#robotSelect').selectOption('microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 120_000 });
}
// Measure the actual body nodes, not their wrapper: the old wrapper already matched
// MuJoCo, while a second baked world transform lifted all visible feet another 120 mm.
async function evidence(page) {
  return page.evaluate(() => {
    const sim = window.__robobuddyCi.app.sim.backend;
    const { THREE, rig, lastObservation: obs } = sim;
    sim.renderFrame();
    const basis = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
    const bodyErrors = [];
    for (const [id, body] of Object.entries(obs.bodies)) {
      const meshBody = rig.bodies.get(id);
      if (!meshBody) continue;
      const actual = meshBody.getWorldPosition(new THREE.Vector3()).multiplyScalar(0.001);
      const [x, y, z] = body.positionM;
      const [w, qx, qy, qz] = body.quaternionWxyz;
      const expectedQ = basis.clone().multiply(new THREE.Quaternion(qx, qy, qz, w));
      bodyErrors.push({ id, positionErrorM: actual.distanceTo(new THREE.Vector3(x, z, -y)),
        angleErrorRad: meshBody.getWorldQuaternion(new THREE.Quaternion()).angleTo(expectedQ) });
    }
    const soleMinYM = {};
    // Pinned DUCK v1 sole parts, not configured rollers or invented collision boxes.
    for (const [side, index] of [['left', 20], ['right', 55]]) {
      const part = rig.officialParts[index];
      const vertices = part.geometry.getAttribute('position');
      const point = new THREE.Vector3();
      let minY = Infinity;
      for (let i = 0; i < vertices.count; i += 1) {
        point.fromBufferAttribute(vertices, i).applyMatrix4(part.matrixWorld);
        minY = Math.min(minY, point.y * 0.001);
      }
      soleMinYM[side] = minY;
    }
    let configuredRollerMeshes = 0;
    rig.root.traverse((node) => { if (node.name === 'configured-passive-roller-assembly') configuredRollerMeshes += 1; });
    return { bodyErrors, soleMinYM, configuredRollerMeshes, officialParts: rig.officialParts.length,
      simulationTimeSeconds: obs.simulationTimeSeconds, floor: obs.footContacts?.floor,
      rootPositionM: obs.bodies.trunk_base.positionM, audit: sim.getPresentationAudit() };
  });
}
function expectRegisteredFrames(value) {
  expect(value.bodyErrors.map((row) => row.id)).toEqual(expect.arrayContaining(['trunk_base', 'ankle_left', 'ankle_right']));
  for (const row of value.bodyErrors) {
    expect(row.positionErrorM, `${row.id}: ${JSON.stringify(value)}`).toBeLessThan(1e-6);
    expect(row.angleErrorRad, row.id).toBeLessThan(1e-5);
  }
  expect(value.bodyErrors).toHaveLength(15);
  expect(value.officialParts).toBe(58);
  expect(value.configuredRollerMeshes).toBe(0);
  expect(value.audit.rendererFloorSnapping).toBe(false);
}

test('physical MicroDuck rendered feet coincide with the contact plant in all three tasks', async ({ page }, testInfo) => {
  test.setTimeout(300_000);
  const errors = [];
  page.on('pageerror', (error) => errors.push(String(error)));
  await openMicroDuck(page);
  expect(await page.locator('#taskSelect option').evaluateAll((items) => items.map((item) => item.value))).toEqual(TASKS);
  await expect(page.locator('#taskSelect')).toHaveValue(TASKS[0]);
  await expect(page.locator('#microduckControlDeck')).toBeHidden();
  const records = [];
  for (const task of TASKS) {
    if (task !== TASKS[0]) {
      await page.locator('#taskSelect').selectOption(task);
      await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 120_000 });
    }
    const initial = await evidence(page);
    expectRegisteredFrames(initial);
    expect(initial.simulationTimeSeconds).toBe(0);
    // Keep the source reset's real ~4 mm clearance; gravity closes it on advancement.
    for (const minY of Object.values(initial.soleMinYM)) {
      expect(minY).toBeGreaterThan(0); expect(minY).toBeLessThan(0.006);
    }
    await page.evaluate(async () => {
      const sim = window.__robobuddyCi.app.sim.backend;
      sim.setCommand({}); await sim.advanceSeconds(2);
    });
    const settled = await evidence(page);
    console.log('SETTLED', task, JSON.stringify(settled));
    await page.locator('#simCanvas').screenshot({ path: testInfo.outputPath(`${task}-grounded.png`) });
    expectRegisteredFrames(settled);
    expect(settled.floor).toEqual({ left_foot_collision: true, right_foot_collision: true });
    for (const minY of Object.values(settled.soleMinYM)) expect(Math.abs(minY)).toBeLessThan(0.001);
    const repeated = await evidence(page);
    expect(repeated.simulationTimeSeconds).toBe(settled.simulationTimeSeconds);
    expect(repeated.rootPositionM).toEqual(settled.rootPositionM);
    records.push({ task, initial, settled });
    await page.evaluate(async () => { await window.__robobuddyCi.app.sim.backend.reset(); });
    const reset = await evidence(page);
    expectRegisteredFrames(reset); expect(reset.simulationTimeSeconds).toBe(0);
  }
  const retired = await page.evaluate(async () => {
    const host = window.__robobuddyCi.app.sim; const old = host.backend;
    try { await host.setScenario('microduck', { simulationMode: 'policy_sim' }); return { rejected: false }; }
    catch (error) { return { rejected: true, sameBackend: host.backend === old, message: error.message }; }
  });
  expect(retired).toMatchObject({ rejected: true, sameBackend: true });
  await testInfo.attach('physical-presentation-registration', { body: JSON.stringify(records, null, 2), contentType: 'application/json' });
  expect(errors).toEqual([]);
});

test('physical MicroDuck transforms preserve tilted and half-turn poses without floor snapping', async ({ page }, testInfo) => {
  test.setTimeout(180_000);
  await openMicroDuck(page);
  // Isolated renderer inputs: no command, setup or physical-state write is made.
  const checks = await page.evaluate(() => {
    const sim = window.__robobuddyCi.app.sim.backend;
    const { rig, THREE } = sim;
    const original = JSON.stringify(sim.lastObservation);
    const basis = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
    const records = [];
    for (const quaternionWxyz of [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1], [0.5, 0.5, 0.5, 0.5]]) {
      rig.applyPhysicalRootPose([0.23, -0.18, 0.45], quaternionWxyz);
      const trunk = rig.bodies.get('trunk_base');
      const p = trunk.getWorldPosition(new THREE.Vector3()).multiplyScalar(0.001);
      const [w, x, y, z] = quaternionWxyz;
      const expectedQ = basis.clone().multiply(new THREE.Quaternion(x, y, z, w));
      records.push({ quaternionWxyz, positionErrorM: p.distanceTo(new THREE.Vector3(0.23, 0.45, 0.18)),
        angleErrorRad: trunk.getWorldQuaternion(new THREE.Quaternion()).angleTo(expectedQ) });
    }
    const after = JSON.stringify(sim.lastObservation);
    rig.applyPhysicalBodyPoses(sim.lastObservation.bodies);
    return { records, physicalStateUnchanged: original === after };
  });
  expect(checks.physicalStateUnchanged).toBe(true);
  for (const row of checks.records) { expect(row.positionErrorM).toBeLessThan(1e-9); expect(row.angleErrorRad).toBeLessThan(1e-6); }
  await page.evaluate(async () => {
    const sim = window.__robobuddyCi.app.sim.backend;
    sim.setCommand({ vx: 0.35 }); await sim.advanceSeconds(2);
  });
  const moving = await evidence(page);
  console.log('MOVING', JSON.stringify(moving));
  expectRegisteredFrames(moving);
  await testInfo.attach('half-turn-registration', { body: JSON.stringify(checks, null, 2), contentType: 'application/json' });
});

test('physical MicroDuck rejects incomplete pose snapshots without partially moving its visual', async ({ page }) => {
  await openMicroDuck(page);
  const result = await page.evaluate(() => {
    const sim = window.__robobuddyCi.app.sim.backend;
    const { rig } = sim;
    const transforms = () => rig.bodyList.map((body) => [body.position.toArray(), body.quaternion.toArray()]);
    const original = JSON.stringify(transforms());
    const physicalState = JSON.stringify(sim.lastObservation);
    const records = [];
    for (const kind of ['missing-body', 'non-finite', 'zero-quaternion']) {
      const snapshot = structuredClone(sim.lastObservation.bodies);
      snapshot.trunk_base.positionM[2] += 1;
      if (kind === 'missing-body') delete snapshot.ankle_right;
      if (kind === 'non-finite') snapshot.ankle_right.positionM[0] = NaN;
      if (kind === 'zero-quaternion') snapshot.ankle_right.quaternionWxyz = [0, 0, 0, 0];
      let rejected = false;
      try { rig.applyPhysicalBodyPoses(snapshot); } catch { rejected = true; }
      records.push({ kind, rejected, unchanged: original === JSON.stringify(transforms()) });
    }
    return { records, physicalUnchanged: physicalState === JSON.stringify(sim.lastObservation) };
  });
  expect(result.physicalUnchanged).toBe(true);
  for (const row of result.records) expect(row).toMatchObject({ rejected: true, unchanged: true });
});
