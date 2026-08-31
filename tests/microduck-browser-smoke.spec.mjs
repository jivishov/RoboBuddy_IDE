import { test, expect } from '@playwright/test';
import { mkdir } from 'node:fs/promises';
import { resolve } from 'node:path';

test('MicroDuck profile uses the shared canvas and survives backend-family switches', async ({ page }) => {
  await page.goto('/?ci=1');
  await page.selectOption('#robotSelect', 'openarm');
  await expect(page.locator('#statusMessage')).toContainText('Ready');
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-simulator-backend', 'source-robot');
  await page.selectOption('#robotSelect', 'microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready');
  const canvas = page.locator('#simCanvas');
  await expect(canvas).toHaveAttribute('data-simulator-backend', 'microduck-policy-sim');
  await expect(page.locator('#simBadge')).toContainText('EXACT PINNED ONNX');
  await expect(page.locator('#simBadge')).not.toContainText('SOURCE PLANT');
  await expect(page.locator('#taskPanel')).toContainText('Pinned runtime hierarchy');
  await expect(page.locator('#taskPanel')).not.toContainText('Pinned task source');
  await expect(page.locator('#fidelityText')).not.toContainText('LeRobot');
  await expect(page.locator('#runBtn')).toBeEnabled();
  await expect(page.locator('#stepBtn')).toBeEnabled();
  await expect(page.locator('#cursorBtn')).toBeEnabled();
  await page.evaluate(() => window.__robobuddyCi.app.sim.setVariant('roller'));
  await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.getState().mode)).toBe('roller');
  await expect(canvas).toHaveAttribute('data-microduck-variant', 'roller');
  await page.click('#fitBtn');
  await page.click('#resetBtn');
  await page.selectOption('#robotSelect', 'unitree');
  await expect(page.locator('#statusMessage')).toContainText('Ready');
  await page.selectOption('#robotSelect', 'microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready');
  await expect(canvas).toHaveAttribute('data-simulator-backend', 'microduck-policy-sim');
  await page.route('**/*', (route) => {
    const url = new URL(route.request().url());
    if (url.hostname !== '127.0.0.1' && url.hostname !== 'localhost') return route.abort();
    return route.continue();
  });
  await page.click('#resetBtn');
  await expect(page.locator('#statusMessage')).toContainText('Simulation reset');
  const epoch = Number(await canvas.getAttribute('data-simulator-host-epoch'));
  expect(epoch).toBeGreaterThanOrEqual(3);
});

test('MicroDuck doubles the configured field and stops both free bodies at its edge', async ({ page }) => {
  test.setTimeout(90_000);
  await page.goto('/?ci=field-edge');
  await page.selectOption('#robotSelect', 'microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });

  const boundary = await page.evaluate(async () => {
    const host = window.__robobuddyCi.app.sim;
    const backend = host.backend;
    const context = { source: 'python', controllerId: 'field-edge', durationMs: 5000 };
    await host.executeCommand('enable', { enabled: true }, { source: 'human' });
    backend.dynamics.data.qpos[0] = 3.92;
    await host.executeCommand('move', { vx: 0.3, vy: 0, yaw: 0 }, context);
    await host.advanceTime(0.2);
    const duck = { state: host.getState(), velocity: Array.from(backend.dynamics.data.qvel.slice(0, 3)) };

    await host.executeCommand('reset', {}, { source: 'human' });
    backend.dynamics.data.qpos[21] = 3.96;
    backend.dynamics.data.qvel[20] = 1;
    await host.advanceTime(0.1);
    const ballAtEdge = host.getState();
    await host.advanceTime(0.2);
    const ballAfterWait = host.getState();
    return {
      floor: [backend.floor.geometry.parameters.width, backend.floor.geometry.parameters.height],
      duck: { position: duck.state.simulatedPose.position, velocity: duck.velocity },
      ballAtEdge: { position: ballAtEdge.ball.position, velocity: ballAtEdge.ball.velocity },
      ballAfterWait: { position: ballAfterWait.ball.position, velocity: ballAfterWait.ball.velocity },
    };
  });

  expect(boundary.floor).toEqual([8000, 8000]);
  expect(boundary.duck.position[0]).toBeCloseTo(3.93, 6);
  expect(boundary.duck.velocity.slice(0, 2)).toEqual([0, 0]);
  expect(boundary.ballAtEdge.position[0]).toBeCloseTo(3.965, 6);
  expect(boundary.ballAtEdge.velocity.slice(0, 2)).toEqual([0, 0]);
  expect(boundary.ballAfterWait.position[0]).toBeCloseTo(boundary.ballAtEdge.position[0], 6);
  expect(boundary.ballAfterWait.position[1]).toBeCloseTo(boundary.ballAtEdge.position[1], 6);
  expect(boundary.ballAfterWait.velocity.slice(0, 2)).toEqual([0, 0]);
});

test('MicroDuck kicks a nearby ball, lets it coast to rest, and rejects distant kick contact', async ({ page }) => {
  test.setTimeout(90_000);
  await page.goto('/?ci=ball-motion');
  await page.selectOption('#robotSelect', 'microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });

  const motion = await page.evaluate(async () => {
    const host = window.__robobuddyCi.app.sim;
    await host.executeCommand('enable', { enabled: true }, { source: 'human' });
    const start = host.getState().ball;
    await host.executeCommand('do', { skill: 'kick_left' }, { source: 'human' });
    await host.advanceTime(0.3);
    const afterKick = host.getState().ball;
    await host.advanceTime(3);
    const atRest = host.getState().ball;

    await host.executeCommand('reset', {}, { source: 'human' });
    await host.executeCommand('spawn_ball', { position: [1, 0, 0.035] }, { source: 'human' });
    await host.executeCommand('do', { skill: 'kick_right' }, { source: 'human' });
    await host.advanceTime(0.3);
    const distant = host.getState().ball;
    return { start, afterKick, atRest, distant };
  });

  expect(motion.afterKick.position[0]).toBeGreaterThan(motion.start.position[0] + 0.03);
  expect(motion.afterKick.velocity[0]).toBeGreaterThan(0.1);
  expect(Math.hypot(...motion.atRest.velocity.slice(0, 2))).toBeLessThanOrEqual(0.015);
  expect(motion.atRest.position[0]).toBeGreaterThan(motion.afterKick.position[0]);
  expect(motion.distant.position[0]).toBeCloseTo(1, 6);
  expect(motion.distant.velocity.slice(0, 2)).toEqual([0, 0]);
});

test('official MicroDuck visual is coherent, grounded, articulated, framed and motion-bounded', async ({ page }) => {
  await page.goto('/?ci=visual-repair');
  await page.selectOption('#robotSelect', 'microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
  const canvas = page.locator('#simCanvas');
  await expect(canvas).toHaveAttribute('data-microduck-motion-state', 'holding');

  const idleBefore = await page.evaluate(() => window.__robobuddyCi.app.sim.getState());
  await page.waitForTimeout(300);
  const idleAfter = await page.evaluate(() => window.__robobuddyCi.app.sim.getState());
  expect(idleBefore.enabled).toBe(false);
  expect(idleAfter.time).toBe(idleBefore.time);
  expect(idleAfter.joints).toEqual(idleBefore.joints);
  expect(idleAfter.simulatedPose.quaternion).toEqual(idleBefore.simulatedPose.quaternion);

  const visual = await page.evaluate(() => {
    const backend = window.__robobuddyCi.app.sim.backend;
    const rig = backend.rig;
    const bounds = rig.getBounds();
    const expectedBasis = [-Math.SQRT1_2, 0, 0, Math.SQRT1_2];
    const basis = rig.modelRoot.quaternion.toArray();
    const officialParts = [];
    const configuredMeshes = [];
    rig.root.traverse((node) => {
      if (node.isMesh && node.userData?.provenance === 'exact_apache_runtime_monitor_visual') officialParts.push(node.name);
      if (node.isMesh && node.userData?.configuredApproximation) configuredMeshes.push(node.name);
    });
    const materials = [...rig.visualMaterials.values()].map((material) => ({
      color: material.color.getHex(), metalness: material.metalness, roughness: material.roughness, side: material.side,
    }));
    const winding = [...rig.geometries]
      .filter((geometry) => geometry.userData?.sourceWinding)
      .map((geometry) => geometry.userData);

    const knee = rig.joints.get('left_knee');
    rig.applyState({ left_knee: 0 });
    const kneeRest = knee.body.quaternion.clone();
    rig.applyState({ left_knee: 0.2 });
    const kneeTravel = kneeRest.angleTo(knee.body.quaternion);

    const bill = rig.officialParts[40];
    rig.applyState({ mouth: 0 });
    rig.root.updateWorldMatrix(true, true);
    const billClosed = bill.getWorldPosition(rig.modelRoot.position.clone());
    rig.applyState({ mouth: 1 });
    rig.root.updateWorldMatrix(true, true);
    const billTravel = billClosed.distanceTo(bill.getWorldPosition(rig.modelRoot.position.clone()));

    rig.applyRootPose([0.1, 0.2, 0.3], [1, 0, 0, 0]);
    const converted = { x: rig.root.position.x, z: rig.root.position.z, floor: rig.getBounds().min.y };
    backend.syncVisuals(backend.dynamics.snapshot());
    backend.fit();

    const projectedExtent = () => {
      const box = rig.getBounds();
      const xs = [box.min.x, box.max.x];
      const ys = [box.min.y, box.max.y];
      const zs = [box.min.z, box.max.z];
      let maxX = 0; let maxY = 0;
      for (const x of xs) for (const y of ys) for (const z of zs) {
        const point = box.min.clone().set(x, y, z).project(backend.camera);
        maxX = Math.max(maxX, Math.abs(point.x));
        maxY = Math.max(maxY, Math.abs(point.y));
      }
      return { maxX, maxY };
    };
    return {
      bounds: { min: bounds.min.toArray(), max: bounds.max.toArray() },
      basisError: Math.max(...basis.map((value, index) => Math.abs(value - expectedBasis[index]))),
      officialParts,
      configuredMeshes,
      materials,
      winding,
      kneeTravel,
      billTravel,
      converted,
      aspect: backend.camera.aspect,
      projected: projectedExtent(),
    };
  });
  expect(visual.officialParts).toHaveLength(58);
  expect(visual.configuredMeshes).not.toContain('configured-articulated-lower-bill');
  expect(visual.basisError).toBeLessThan(1e-12);
  expect(Math.abs(visual.bounds.min[1])).toBeLessThan(1e-6);
  expect(visual.materials.every((material) => material.metalness === 0 && material.roughness >= 0.8 && material.side === 2)).toBe(true);
  expect(new Set(visual.materials.map((material) => material.color)).size).toBeGreaterThanOrEqual(12);
  expect(visual.winding.every((item) => item.browserWinding === 'centroid-oriented')).toBe(true);
  expect(visual.winding.reduce((sum, item) => sum + item.flippedFaces, 0)).toBeGreaterThan(100);
  expect(visual.kneeTravel).toBeCloseTo(0.2, 5);
  expect(visual.billTravel).toBeGreaterThan(0.1);
  expect(visual.billTravel).toBeLessThan(15);
  expect(visual.converted.x).toBeCloseTo(100, 6);
  expect(visual.converted.z).toBeCloseTo(-200, 6);
  expect(Math.abs(visual.converted.floor)).toBeLessThan(1e-6);
  expect(visual.projected.maxX).toBeLessThan(0.9);
  expect(visual.projected.maxY).toBeLessThan(0.9);

  const initialAspect = visual.aspect;
  await page.setViewportSize({ width: 1180, height: 800 });
  await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.backend.camera.aspect)).not.toBeCloseTo(initialAspect, 2);
  const resized = await page.evaluate(() => {
    const backend = window.__robobuddyCi.app.sim.backend;
    const box = backend.rig.getBounds();
    let maxX = 0; let maxY = 0;
    for (const x of [box.min.x, box.max.x]) for (const y of [box.min.y, box.max.y]) for (const z of [box.min.z, box.max.z]) {
      const point = box.min.clone().set(x, y, z).project(backend.camera);
      maxX = Math.max(maxX, Math.abs(point.x)); maxY = Math.max(maxY, Math.abs(point.y));
    }
    return { maxX, maxY };
  });
  expect(resized.maxX).toBeLessThan(0.9);
  expect(resized.maxY).toBeLessThan(0.9);

  const targetDeltas = await page.evaluate(async () => {
    const backend = window.__robobuddyCi.app.sim.backend;
    backend.pause();
    await backend.executeCommand('enable', { enabled: true }, { source: 'human' });
    const targets = [backend.getState().targets];
    for (let index = 0; index < 12; index += 1) {
      await backend.advanceTime(0.02, { controlStep: true });
      targets.push(backend.getState().targets);
    }
    let maximum = 0;
    for (let sample = 1; sample < targets.length; sample += 1) {
      for (let joint = 0; joint < 14; joint += 1) maximum = Math.max(maximum, Math.abs(targets[sample][joint] - targets[sample - 1][joint]));
    }
    backend.stop();
    return maximum;
  });
  expect(targetDeltas).toBeLessThanOrEqual(0.060001);
  await expect(canvas).toHaveAttribute('data-microduck-motion-state', 'holding');

  const contrast = await page.evaluate(() => {
    const channel = (value) => { const scaled = value / 255; return scaled <= 0.04045 ? scaled / 12.92 : ((scaled + 0.055) / 1.055) ** 2.4; };
    const rgb = (value) => (value.match(/[\d.]+/g) || []).slice(0, 3).map(Number);
    const ratio = (element) => {
      const style = getComputedStyle(element); const foreground = rgb(style.color); const background = rgb(style.backgroundColor);
      const fg = 0.2126 * channel(foreground[0]) + 0.7152 * channel(foreground[1]) + 0.0722 * channel(foreground[2]);
      const bg = 0.2126 * channel(background[0]) + 0.7152 * channel(background[1]) + 0.0722 * channel(background[2]);
      return (Math.max(fg, bg) + 0.05) / (Math.min(fg, bg) + 0.05);
    };
    const deck = document.querySelector('#microduckControlDeck');
    const selectors = ['[data-md-command="enable"]', '[data-md-command="set_color"]', '[data-md-command="set_mode"][data-mode="walking"]', '.md-collapse', '.md-boundary'];
    return selectors.map((selector) => ({ selector, ratio: ratio(deck.querySelector(selector)) }));
  });
  expect(contrast.every((item) => item.ratio >= 4.5)).toBe(true);
  await page.click('#highContrastSceneBtn');
  await expect(canvas).toHaveAttribute('data-high-contrast-scene', 'false');

  await page.evaluate(async () => {
    const backend = window.__robobuddyCi.app.sim.backend;
    backend.camera.position.set(2000, 2000, 2000);
    backend.controls.target.set(900, 900, 900);
    await backend.reset('microduck', backend.scenario);
  });
  await expect(canvas).toHaveAttribute('data-microduck-motion-state', 'holding');
  expect((await page.evaluate(() => window.__robobuddyCi.app.sim.getState())).enabled).toBe(false);

  const screenshotDirectory = process.env.MICRODUCK_SCREENSHOT_DIR;
  if (screenshotDirectory) {
    await mkdir(screenshotDirectory, { recursive: true });
    await page.setViewportSize({ width: 1366, height: 768 });
    await page.click('#highContrastSceneBtn');
    await page.waitForTimeout(150);
    await page.screenshot({ path: resolve(screenshotDirectory, 'microduck-visual-repair-desktop.png') });
  }
});

test('official runtime visual is upright, framed, articulated, and resize-safe', async ({ page }) => {
  const readVisualMetrics = () => page.evaluate(() => {
    const backend = window.__robobuddyCi.app.sim.backend;
    backend.updateCameras();
    const box = backend.rig.getBounds();
    const size = box.getSize(box.min.clone());
    const center = box.getCenter(box.min.clone());
    const corners = [];
    for (const x of [box.min.x, box.max.x]) {
      for (const y of [box.min.y, box.max.y]) {
        for (const z of [box.min.z, box.max.z]) corners.push(box.min.clone().set(x, y, z).project(backend.camera));
      }
    }
    let officialParts = 0;
    let configuredParts = 0;
    let articulatedOfficialParts = 0;
    backend.rig.root.traverse((node) => {
      if (node.isMesh && node.userData?.provenance === 'exact_apache_runtime_monitor_visual') officialParts += 1;
      if (node.isMesh && node.userData?.configuredApproximation) configuredParts += 1;
      if (node.isMesh && node.userData?.articulatedByConfiguredPivot) articulatedOfficialParts += 1;
    });
    const canvasRect = backend.canvas.getBoundingClientRect();
    return {
      backend: backend.canvas.dataset.simulatorBackend,
      bodyCount: backend.rig.bodyList.length,
      meshCount: backend.rig.visual.meshes.length,
      officialParts,
      configuredParts,
      articulatedOfficialParts,
      bounds: {
        minY: box.min.y,
        maxY: box.max.y,
        size: size.toArray(),
        center: center.toArray(),
      },
      ndc: {
        minX: Math.min(...corners.map((corner) => corner.x)),
        maxX: Math.max(...corners.map((corner) => corner.x)),
        minY: Math.min(...corners.map((corner) => corner.y)),
        maxY: Math.max(...corners.map((corner) => corner.y)),
      },
      cameraMode: backend.cameraMode,
      canvas: { width: canvasRect.width, height: canvasRect.height, aspect: backend.camera.aspect },
    };
  });

  await page.goto('/?ci=visual-repair');
  await page.selectOption('#robotSelect', 'microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready');
  await page.click('#fitBtn');
  await page.waitForTimeout(200);

  const initial = await readVisualMetrics();
  console.log(`MICRODUCK_VISUAL_1366=${JSON.stringify(initial)}`);
  const evidenceDir = process.env.MICRODUCK_VISUAL_EVIDENCE_DIR;
  if (evidenceDir) await page.screenshot({ path: `${evidenceDir}/microduck-visual-repair-1366x768.png`, fullPage: false });
  expect(initial).toMatchObject({ backend: 'microduck-policy-sim', bodyCount: 15, meshCount: 28, officialParts: 58, configuredParts: 4, articulatedOfficialParts: 1, cameraMode: 'orbit' });
  expect(initial.bounds.minY).toBeGreaterThanOrEqual(-0.5);
  expect(initial.bounds.size[0]).toBeGreaterThan(80);
  expect(initial.bounds.size[1]).toBeGreaterThan(150);
  expect(initial.bounds.size[1]).toBeGreaterThan(initial.bounds.size[2]);
  expect(initial.ndc.minX).toBeGreaterThanOrEqual(-0.98);
  expect(initial.ndc.maxX).toBeLessThanOrEqual(0.98);
  expect(initial.ndc.minY).toBeGreaterThanOrEqual(-0.98);
  expect(initial.ndc.maxY).toBeLessThanOrEqual(0.98);

  const articulation = await page.evaluate(() => {
    const backend = window.__robobuddyCi.app.sim.backend;
    backend.pause();
    const head = backend.rig.bodies.get('bottom_head_shell');
    const beforeHead = head.getWorldQuaternion(backend.rig.root.quaternion.clone());
    const beforeMouth = backend.rig.mouthPivot.quaternion.clone();
    backend.rig.applyState({ head_yaw: 0.7, mouth: 1 });
    backend.rig.root.updateWorldMatrix(true, true);
    const afterHead = head.getWorldQuaternion(backend.rig.root.quaternion.clone());
    const afterMouth = backend.rig.mouthPivot.quaternion.clone();
    const result = {
      headAngleRad: beforeHead.angleTo(afterHead),
      mouthAngleRad: beforeMouth.angleTo(afterMouth),
    };
    backend.syncVisuals(backend.dynamics.snapshot());
    backend.resume();
    return result;
  });
  expect(articulation.headAngleRad).toBeGreaterThan(0.6);
  expect(articulation.mouthAngleRad).toBeGreaterThan(0.08);
  expect(articulation.mouthAngleRad).toBeLessThan(0.1);

  await page.evaluate(() => window.__robobuddyCi.app.sim.executeCommand('set_camera', { camera: 'head' }, { source: 'human' }));
  await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.getState().virtualCamera.mode)).toBe('head');
  await page.click('#fitBtn');
  await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.getState().virtualCamera.mode)).toBe('head');
  await page.click('#resetBtn');
  await expect(page.locator('#statusMessage')).toContainText('Simulation reset');
  await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.getState().virtualCamera.mode)).toBe('head');

  await page.evaluate(() => window.__robobuddyCi.app.sim.executeCommand('set_camera', { camera: 'orbit' }, { source: 'human' }));
  await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.getState().virtualCamera.mode)).toBe('orbit');

  await page.setViewportSize({ width: 1440, height: 900 });
  await page.waitForTimeout(250);
  const resized = await readVisualMetrics();
  console.log(`MICRODUCK_VISUAL_1440=${JSON.stringify(resized)}`);
  expect(resized.cameraMode).toBe('orbit');
  expect(resized.ndc.minX).toBeGreaterThanOrEqual(-0.98);
  expect(resized.ndc.maxX).toBeLessThanOrEqual(0.98);
  expect(resized.ndc.minY).toBeGreaterThanOrEqual(-0.98);
  expect(resized.ndc.maxY).toBeLessThanOrEqual(0.98);
  if (evidenceDir) await page.screenshot({ path: `${evidenceDir}/microduck-visual-repair-1440x900.png`, fullPage: false });
});

test('learner camera views are named, framed, mode-aware, and source-aligned', async ({ page }) => {
  const readCamera = () => page.evaluate(() => {
    const backend = window.__robobuddyCi.app.sim.backend;
    backend.updateCameras(performance.now());
    backend.camera.updateMatrixWorld(true);
    const bounds = backend.rig.getBounds().clone();
    const corners = [];
    for (const x of [bounds.min.x, bounds.max.x]) {
      for (const y of [bounds.min.y, bounds.max.y]) {
        for (const z of [bounds.min.z, bounds.max.z]) corners.push(bounds.min.clone().set(x, y, z).project(backend.camera));
      }
    }
    const ball = backend.ballMesh.position.clone().project(backend.camera);
    const direction = backend.camera.getWorldDirection(backend.camera.position.clone()).normalize();
    const cameraUp = backend.camera.up.clone().applyQuaternion(backend.camera.quaternion).normalize();
    const site = backend.rig.getSiteWorldPose('head_camera');
    const sourceBasis = backend.camera.quaternion.clone().set(-0.5, 0.5, 0.5, -0.5).normalize();
    const expectedQuaternion = site?.quaternion.clone().multiply(sourceBasis);
    const expectedForward = backend.camera.up.clone().set(0, 0, -1).applyQuaternion(expectedQuaternion).normalize();
    const expectedUp = backend.camera.up.clone().set(0, 1, 0).applyQuaternion(expectedQuaternion).normalize();
    const state = backend.getState();
    return {
      mode: state.virtualCamera.mode,
      name: state.virtualCamera.name,
      purpose: state.virtualCamera.purpose,
      frame: state.virtualCamera.frame,
      inset: state.virtualCamera.inset,
      transport: state.virtualCamera.transport,
      fov: backend.camera.fov,
      robot: {
        minX: Math.min(...corners.map((point) => point.x)),
        maxX: Math.max(...corners.map((point) => point.x)),
        minY: Math.min(...corners.map((point) => point.y)),
        maxY: Math.max(...corners.map((point) => point.y)),
      },
      ball: [ball.x, ball.y, ball.z],
      position: backend.camera.position.toArray(),
      quaternion: backend.camera.quaternion.toArray(),
      head: {
        positionError: backend.camera.position.distanceTo(site.position),
        quaternionError: backend.camera.quaternion.angleTo(expectedQuaternion),
        forwardDot: direction.dot(expectedForward),
        upDot: cameraUp.dot(expectedUp),
      },
      overlay: document.querySelector('#cameraModeLabel')?.textContent || '',
      fitLabel: document.querySelector('#fitBtn')?.textContent || '',
      canvasName: backend.canvas.dataset.cameraName,
      canvasPurpose: backend.canvas.dataset.cameraPurpose,
      hasInsetRenderer: typeof backend.renderHeadInset === 'function' || Boolean(backend.headCamera),
    };
  });
  const assertRobotFramed = (view) => {
    expect(view.robot.minX).toBeGreaterThanOrEqual(-0.96);
    expect(view.robot.maxX).toBeLessThanOrEqual(0.96);
    expect(view.robot.minY).toBeGreaterThanOrEqual(-0.96);
    expect(view.robot.maxY).toBeLessThanOrEqual(0.96);
    expect(view.robot.maxY - view.robot.minY).toBeGreaterThan(0.35);
  };
  const cameraState = () => page.evaluate(() => {
    const state = window.__robobuddyCi.app.sim.getState();
    const officialMaterials = [];
    window.__robobuddyCi.app.sim.backend.rig.root.traverse((node) => {
      if (node.isMesh && node.userData?.provenance === 'exact_apache_runtime_monitor_visual') {
        const materials = Array.isArray(node.material) ? node.material : [node.material];
        for (const material of materials) officialMaterials.push({ color: material.color.getHex(), metalness: material.metalness, roughness: material.roughness });
      }
    });
    return { time: state.time, enabled: state.enabled, joints: state.joints, targets: state.targets, ball: state.ball, contacts: state.contacts, officialMaterials };
  });

  await page.goto('/?ci=camera-readability');
  await page.selectOption('#robotSelect', 'microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready');
  const deck = page.locator('#microduckControlDeck');
  const presentation = deck.locator('details').filter({ hasText: 'Presentation & modeled peripherals' });
  await presentation.locator('summary').click();
  const cameraButtons = deck.locator('[data-md-command="set_camera"]');
  await expect(cameraButtons).toHaveCount(3);
  await expect(cameraButtons.nth(0)).toContainText('Overview');
  await expect(cameraButtons.nth(0)).toContainText('robot and ball together');
  await expect(cameraButtons.nth(1)).toContainText('Follow');
  await expect(cameraButtons.nth(1)).toContainText('behind and above');
  await expect(cameraButtons.nth(2)).toContainText('Head POV');
  await expect(cameraButtons.nth(2)).toContainText('not hardware video');
  for (let index = 0; index < 3; index += 1) await expect(cameraButtons.nth(index)).toBeEnabled();
  await expect(deck).toContainText('C cycles Overview → Follow → Head POV');
  const beforeSwitching = await cameraState();

  await cameraButtons.nth(0).click();
  await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.getState().virtualCamera.mode)).toBe('orbit');
  const overview = await readCamera();
  console.log(`MICRODUCK_CAMERA_OVERVIEW=${JSON.stringify({ robot: overview.robot, ball: overview.ball, fov: overview.fov, frame: overview.frame })}`);
  expect(overview).toMatchObject({ mode: 'orbit', name: 'Overview', frame: 'world', inset: false, fov: 44, canvasName: 'Overview', hasInsetRenderer: false, fitLabel: 'Refit Overview' });
  expect(overview.purpose).toContain('robot and ball together');
  expect(overview.overlay).toBe('OVERVIEW · ORBIT ROBOT + BALL');
  expect(overview.ball[0]).toBeGreaterThanOrEqual(-0.94);
  expect(overview.ball[0]).toBeLessThanOrEqual(0.94);
  expect(overview.ball[1]).toBeGreaterThanOrEqual(-0.94);
  expect(overview.ball[1]).toBeLessThanOrEqual(0.94);
  assertRobotFramed(overview);
  await page.click('#fitBtn');
  const overviewFitA = await readCamera();
  await page.click('#fitBtn');
  const overviewFitB = await readCamera();
  expect(overviewFitA.mode).toBe('orbit');
  expect(overviewFitA.position).toEqual(overviewFitB.position);
  expect(overviewFitA.quaternion).toEqual(overviewFitB.quaternion);

  await cameraButtons.nth(1).click();
  await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.getState().virtualCamera.mode)).toBe('chase');
  const follow = await readCamera();
  console.log(`MICRODUCK_CAMERA_FOLLOW=${JSON.stringify({ robot: follow.robot, ball: follow.ball, fov: follow.fov, frame: follow.frame })}`);
  expect(follow).toMatchObject({ mode: 'chase', name: 'Follow', frame: 'robot_root', inset: false, fov: 44, canvasName: 'Follow', fitLabel: 'Refit Follow' });
  expect(follow.overlay).toBe('FOLLOW · STABLE THIRD-PERSON TRACKING');
  assertRobotFramed(follow);
  const tracking = await page.evaluate(() => {
    const backend = window.__robobuddyCi.app.sim.backend;
    backend.fit('chase');
    const start = backend.camera.position.clone();
    backend.rig.root.position.x += 160;
    backend.rig.root.updateWorldMatrix(true, true);
    backend.lastCameraUpdateTime = 0;
    const stepTravel = [];
    let previous = backend.camera.position.clone();
    for (let frame = 1; frame <= 90; frame += 1) {
      backend.updateCameras(frame * (1000 / 60));
      stepTravel.push(backend.camera.position.distanceTo(previous));
      previous = backend.camera.position.clone();
    }
    const bounds = backend.rig.getBounds().clone();
    const points = [];
    for (const x of [bounds.min.x, bounds.max.x]) for (const y of [bounds.min.y, bounds.max.y]) for (const z of [bounds.min.z, bounds.max.z]) points.push(bounds.min.clone().set(x, y, z).project(backend.camera));
    const result = {
      cameraTravel: backend.camera.position.distanceTo(start),
      firstStep: stepTravel[0],
      lastStep: stepTravel.at(-1),
      maxX: Math.max(...points.map((point) => Math.abs(point.x))),
      maxY: Math.max(...points.map((point) => Math.abs(point.y))),
    };
    backend.syncVisuals(backend.dynamics.snapshot());
    backend.fit('chase');
    return result;
  });
  expect(tracking.cameraTravel).toBeGreaterThan(150);
  expect(tracking.firstStep).toBeGreaterThan(tracking.lastStep * 100);
  expect(tracking.maxX).toBeLessThanOrEqual(0.96);
  expect(tracking.maxY).toBeLessThanOrEqual(0.96);
  console.log(`MICRODUCK_CAMERA_TRACKING=${JSON.stringify(tracking)}`);
  await page.click('#fitBtn');
  await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.getState().virtualCamera.mode)).toBe('chase');
  await page.setViewportSize({ width: 1440, height: 900 });
  await page.waitForTimeout(150);
  const followResized = await readCamera();
  expect(followResized.mode).toBe('chase');
  assertRobotFramed(followResized);

  await cameraButtons.nth(2).click();
  await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.getState().virtualCamera.mode)).toBe('head');
  const head = await readCamera();
  console.log(`MICRODUCK_CAMERA_HEAD=${JSON.stringify({ ball: head.ball, fov: head.fov, frame: head.frame, head: head.head })}`);
  expect(head).toMatchObject({ mode: 'head', name: 'Head POV', frame: 'head_camera', inset: false, fov: 100, canvasName: 'Head POV', fitLabel: 'Align Head POV' });
  expect(head.transport).toContain('no hardware video');
  expect(head.overlay).toBe('HEAD POV · MODELED RENDERED VIEW · NO HARDWARE VIDEO');
  expect(head.head.positionError).toBeLessThan(1e-8);
  expect(head.head.quaternionError).toBeLessThan(1e-8);
  expect(head.head.forwardDot).toBeGreaterThan(0.99999999);
  expect(head.head.upDot).toBeGreaterThan(0.99999999);
  expect(head.ball[0]).toBeGreaterThanOrEqual(-0.96);
  expect(head.ball[0]).toBeLessThanOrEqual(0.96);
  expect(head.ball[1]).toBeGreaterThanOrEqual(-0.96);
  expect(head.ball[1]).toBeLessThanOrEqual(0.96);
  await page.click('#fitBtn');
  await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.getState().virtualCamera.mode)).toBe('head');
  await page.setViewportSize({ width: 1180, height: 800 });
  await page.waitForTimeout(150);
  const headResized = await readCamera();
  expect(headResized.mode).toBe('head');
  expect(headResized.head.positionError).toBeLessThan(1e-8);
  expect(headResized.head.quaternionError).toBeLessThan(1e-8);
  await page.click('#resetBtn');
  await expect(page.locator('#statusMessage')).toContainText('Simulation reset');
  await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.getState().virtualCamera.mode)).toBe('head');
  const headReset = await readCamera();
  expect(headReset.head.positionError).toBeLessThan(1e-8);
  expect(headReset.head.quaternionError).toBeLessThan(1e-8);
  expect((await cameraState()).enabled).toBe(false);

  const afterSwitching = await cameraState();
  expect(afterSwitching).toEqual(beforeSwitching);
  await page.setViewportSize({ width: 1366, height: 768 });
  await cameraButtons.nth(0).click();
  await deck.locator('.md-capture-button').click();
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-capture', 'true');
  const cycle = [];
  for (let index = 0; index < 3; index += 1) {
    await page.keyboard.press('c');
    const expectedMode = ['chase', 'head', 'orbit'][index];
    await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.getState().virtualCamera.mode)).toBe(expectedMode);
    cycle.push((await readCamera()).mode);
  }
  expect(cycle).toEqual(['chase', 'head', 'orbit']);
  await deck.locator('.md-capture-button').click();
  const afterKeyboardCycle = await cameraState();
  expect(afterKeyboardCycle).toEqual(afterSwitching);

  const contrast = await cameraButtons.nth(0).evaluate((button) => {
    const channel = (value) => { const scaled = value / 255; return scaled <= 0.04045 ? scaled / 12.92 : ((scaled + 0.055) / 1.055) ** 2.4; };
    const rgb = (value) => (value.match(/[\d.]+/g) || []).slice(0, 3).map(Number);
    const style = getComputedStyle(button);
    const foreground = rgb(style.color); const background = rgb(style.backgroundColor);
    const luminance = (color) => 0.2126 * channel(color[0]) + 0.7152 * channel(color[1]) + 0.0722 * channel(color[2]);
    return (Math.max(luminance(foreground), luminance(background)) + 0.05) / (Math.min(luminance(foreground), luminance(background)) + 0.05);
  });
  expect(contrast).toBeGreaterThanOrEqual(4.5);

  const evidenceDir = process.env.MICRODUCK_CAMERA_EVIDENCE_DIR;
  if (evidenceDir) {
    await mkdir(evidenceDir, { recursive: true });
    await presentation.scrollIntoViewIfNeeded();
    for (const [mode, name] of [['orbit', 'overview'], ['chase', 'follow'], ['head', 'head-pov']]) {
      await page.evaluate((camera) => window.__robobuddyCi.app.sim.executeCommand('set_camera', { camera }, { source: 'human' }), mode);
      await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.getState().virtualCamera.mode)).toBe(mode);
      await page.waitForTimeout(120);
      await page.screenshot({ path: resolve(evidenceDir, `microduck-camera-${name}-1366x768.png`), fullPage: false });
    }
  }
});

test('a delayed stale activation is disposed before the replacement backend starts', async ({ page }) => {
  await page.route('**/assets/microduck/generated/procedural-rig.json', async (route) => {
    await new Promise((resolve) => setTimeout(resolve, 300));
    await route.continue();
  });
  await page.goto('/?ci=1');
  await expect(page.locator('#statusMessage')).toContainText('Ready');
  const result = await page.evaluate(async () => {
    const host = window.__robobuddyCi.app.sim;
    const stale = host.setScenario('microduck', { variant: 'walking' }, {});
    await new Promise((resolve) => setTimeout(resolve, 0));
    const replacement = host.setScenario('unitree', null, {});
    const [staleResult, replacementResult] = await Promise.all([stale, replacement]);
    return {
      staleResult,
      replacementResult,
      ready: host.isReady(),
      backend: document.querySelector('#simCanvas').dataset.simulatorBackend,
      pending: document.querySelector('#simCanvas').dataset.simulatorHostPendingCount,
    };
  });
  expect(result).toEqual({ staleResult: false, replacementResult: true, ready: true, backend: 'source-robot', pending: '0' });
});

test('vendored ORT matches walking and roller CPU fixtures', async ({ page }) => {
  await page.goto('/');
  const result = await page.evaluate(async () => {
    const fixture = await (await fetch('./assets/microduck/fixtures/inference-parity.json')).json();
    const ort = await import('./assets/microduck/runtime/onnx/ort.wasm.min.mjs');
    ort.env.wasm.wasmPaths = `${location.origin}/assets/microduck/runtime/onnx/`;
    ort.env.wasm.numThreads = 1;
    const errors = {};
    for (const [name, item] of Object.entries(fixture.policies)) {
      const session = await ort.InferenceSession.create(`./assets/microduck/${item.path}`, { executionProviders: ['wasm'] });
      const output = await session.run({ [item.inputName]: new ort.Tensor('float32', Float32Array.from(fixture.input), [1,61]) });
      const values = Array.from(output[session.outputNames[0]].data);
      errors[name] = Math.max(...values.map((value, index) => Math.abs(value - item.output[index])));
      await session.release();
    }
    return errors;
  });
  expect(result.alpha_walking).toBeLessThanOrEqual(1e-5);
  expect(result.roller).toBeLessThanOrEqual(1e-5);
});

test('MicroDuck control deck is complete, reachable, and trusted-audio gated at 1366x768', async ({ page }) => {
  await page.goto('/?ci=1');
  await page.selectOption('#robotSelect', 'microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready');
  const deck = page.locator('#microduckControlDeck');
  await expect(deck).toBeVisible();
  await expect(page.locator('#agentAccessControl')).toHaveAttribute('data-access', 'off');
  await expect(page.locator('#highContrastSceneBtn')).toHaveAttribute('aria-pressed', 'true');
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-high-contrast-scene', 'true');
  await expect(page.locator('html')).toHaveAttribute('data-theme', 'calm-light');
  await expect(deck.locator('[data-md-command="sound"]')).toHaveCount(7);
  await expect(deck.locator('[data-md-command="set_color"]')).toHaveCount(4);
  await expect(deck.locator('[data-md-command="set_camera"]')).toHaveCount(3);
  await expect(deck).toContainText('250 ms browser focus-safety lease');
  await expect(deck).toContainText('not hardware validation');
  await expect(deck.locator('[data-md-command="set_color"]').first()).toBeDisabled();
  await expect(deck.locator('[data-md-command="stop"]')).toBeEnabled();
  await expect(deck.locator('[data-md-command="reset"]')).toBeEnabled();
  expect(await page.evaluate(() => window.__robobuddyCi.app.sim.unlockAudio(new Event('click')))).toBe(false);
  await deck.locator('details').filter({ hasText: 'Generated local audio' }).locator('summary').click();
  await deck.locator('.md-audio-unlock').click();
  await expect(deck.locator('[data-md-value="audio.status"]')).toContainText('UNLOCKED');
  await deck.locator('.md-capture-button').click();
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-capture', 'true');
  await deck.locator('[data-md-command="set_mode"][data-mode="roller"]').click();
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-capture', 'false');
  await expect(deck.locator('[data-md-range="move"][data-field="vx"]')).toHaveAttribute('max', '0.6');
  await expect(deck.locator('[data-md-range="move"][data-field="vy"]')).toBeDisabled();
  const bounds = await deck.boundingBox();
  expect(bounds.x).toBeGreaterThanOrEqual(0);
  expect(bounds.y).toBeGreaterThanOrEqual(0);
  expect(bounds.x + bounds.width).toBeLessThanOrEqual(1366);
  expect(bounds.y + bounds.height).toBeLessThanOrEqual(768);
  await deck.locator('.md-capture-button').click();
  await expect(deck.locator('[data-md-range="move"][data-field="vy"]')).toBeDisabled();
  await deck.locator('[data-md-command="sound"][data-tag="chirp"]').click();
  await deck.locator('details').filter({ hasText: 'Presentation & modeled peripherals' }).locator('summary').click();
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-capture', 'true');
  await page.evaluate(() => window.__robobuddyCi.app.sim.pause());
  const beforeColor = await page.evaluate(() => window.__robobuddyCi.app.sim.getState());
  await deck.locator('[data-md-command="set_color"][data-color="lavender"]').click();
  await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.getState().color)).toBe('lavender');
  const afterColor = await page.evaluate(() => window.__robobuddyCi.app.sim.getState());
  expect({ position: afterColor.simulatedPose, joints: afterColor.joints, targets: afterColor.targets, contacts: afterColor.contacts, ball: afterColor.ball }).toEqual({ position: beforeColor.simulatedPose, joints: beforeColor.joints, targets: beforeColor.targets, contacts: beforeColor.contacts, ball: beforeColor.ball });
  await page.evaluate(() => window.__robobuddyCi.app.sim.resume());
  await deck.locator('[data-md-command="set_camera"][data-camera="chase"]').click();
  await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.getState().virtualCamera.mode)).toBe('chase');
  await deck.locator('[data-md-command="set_camera"][data-camera="head"]').click();
  const headCamera = await page.evaluate(() => {
    const backend = window.__robobuddyCi.app.sim.backend;
    backend.updateCameras();
    const site = backend.rig.getSiteWorldPose('head_camera');
    const direction = backend.camera.getWorldDirection(site.position.clone());
    return { positionDistance: backend.camera.position.distanceTo(site.position), directionLength: direction.length(), frame: backend.getState().virtualCamera.frame };
  });
  expect(headCamera.positionDistance).toBeLessThan(1e-6);
  expect(headCamera.directionLength).toBeCloseTo(1, 8);
  expect(headCamera.frame).toBe('head_camera');
  await deck.locator('[data-md-command="set_camera"][data-camera="orbit"]').click();
  await deck.locator('[data-md-tof-source="raycast"]').check();
  await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.getState().tof.source)).toBe('raycast');
  const modeledFrames = await page.evaluate(async () => {
    await window.__robobuddyCi.app.sim.advanceTime(0.004);
    const state = window.__robobuddyCi.app.sim.getState();
    return { tofLength: state.tof.valuesM.length, tofFrame: state.tof.frame, imuFrames: [state.imu.trunk.frame, state.imu.head.frame] };
  });
  expect(modeledFrames).toEqual({ tofLength: 64, tofFrame: 'tof', imuFrames: ['imu', 'head_imu'] });
  await page.keyboard.down('w');
  await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.getState().movement.applied[0])).toBeGreaterThan(0);
  await page.keyboard.up('w');
  await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.getState().movement.applied[0])).toBe(0);
  await page.locator('#robotSelect').focus();
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-capture', 'false');
  await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.sim.getState().movement.applied[0])).toBe(0);
});

test('390x740 preserves Simulator deck reachability and returns to Code', async ({ page }) => {
  await page.setViewportSize({ width: 390, height: 740 });
  await page.goto('/?ci=1');
  await page.selectOption('#robotSelect', 'microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready');
  await page.locator('#mobileSimBtn').click();
  const deck = page.locator('#microduckControlDeck');
  await expect(deck).toBeVisible();
  const deckBounds = await deck.boundingBox();
  const simBounds = await page.locator('.sim-pane').boundingBox();
  expect(deckBounds.x).toBeGreaterThanOrEqual(simBounds.x);
  expect(deckBounds.y).toBeGreaterThanOrEqual(simBounds.y);
  expect(deckBounds.x + deckBounds.width).toBeLessThanOrEqual(simBounds.x + simBounds.width + 1);
  expect(deckBounds.y + deckBounds.height).toBeLessThanOrEqual(simBounds.y + simBounds.height + 1);
  await deck.locator('.md-collapse').click();
  await expect(deck.locator('.md-collapse')).toHaveAttribute('aria-expanded', 'false');
  await deck.locator('.md-collapse').click();
  await expect(deck.locator('[data-md-command="reset"]')).toBeVisible();
  for (const details of await deck.locator('details').all()) {
    if (!(await details.evaluate((element) => element.open))) await details.locator('summary').click();
  }
  const deckBody = deck.locator('.md-deck-body');
  const finalControl = deck.locator('[data-md-tof-source="raycast"]');
  await finalControl.scrollIntoViewIfNeeded();
  const [bodyBounds, controlBounds] = await Promise.all([deckBody.boundingBox(), finalControl.boundingBox()]);
  expect(controlBounds.y).toBeGreaterThanOrEqual(bodyBounds.y);
  expect(controlBounds.y + controlBounds.height).toBeLessThanOrEqual(bodyBounds.y + bodyBounds.height + 1);
  await page.locator('#mobileCodeBtn').click();
  await expect(page.locator('.editor-pane')).toBeVisible();
  await expect(page.locator('#editor')).toBeVisible();
});

test('live MicroDuck Python supports top-level await, live state, pause, both Step boundaries, cursor, cancellation recovery, and manual preemption', async ({ page }) => {
  await page.goto('/?ci=python-cycle04');
  await page.selectOption('#robotSelect', 'microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
  await expect(page.locator('#runBtn')).toBeEnabled();
  await expect(page.locator('#stepBtn')).toBeEnabled();
  await expect(page.locator('#cursorBtn')).toBeEnabled();

  await page.click('#runBtn');
  await expect(page.locator('#statusMessage')).toHaveText('MicroDuck Python run complete', { timeout: 90_000 });
  await expect(page.locator('#problemsPanel')).toContainText('live modeled time');
  expect(await page.evaluate(() => window.__robobuddyCi.app.sim.getState().time)).toBeGreaterThan(0);

  const stepProgram = `from robot_config import create_robot\nrobot = create_robot()\nawait robot.connect()\nawait robot.move(0.2, 0, 0)\nawait robot.sleep(0.06)\nstate = await robot.get_state()\nprint("stepped", state["time"])\nawait robot.disconnect()\n`;
  await page.evaluate((source) => {
    const app = window.__robobuddyCi.app;
    app.files['main.py'] = source;
    app.openFile('main.py');
    app.editor.setFile('main.py', source);
    app.workspaceGeneration += 1;
  }, stepProgram);
  await page.click('#stepBtn');
  await expect(page.locator('#statusMessage')).toContainText('Stepped main.py:3 · connect', { timeout: 90_000 });
  await page.click('#stepBtn');
  await expect(page.locator('#statusMessage')).toContainText('Stepped main.py:4 · move');
  const beforeSleepStep = await page.evaluate(() => window.__robobuddyCi.app.sim.getState().time);
  await page.click('#stepBtn');
  await expect(page.locator('#statusMessage')).toContainText('Stepped 20 ms of sleep at main.py:5');
  const afterSleepStep = await page.evaluate(() => window.__robobuddyCi.app.sim.getState().time);
  expect(afterSleepStep - beforeSleepStep).toBeCloseTo(0.02, 3);
  expect(await page.evaluate(() => window.__robobuddyCi.app.sim.getState().loop.inferenceInFlight)).toBe(false);
  const sleepLine = await page.locator('#simActionLabel').textContent();
  await page.click('#stepBtn');
  await expect(page.locator('#simActionLabel')).toHaveText(sleepLine || '');
  await page.click('#stepBtn');
  await expect(page.locator('#statusMessage')).toContainText('Stepped main.py:5 · sleep');
  await page.click('#stopBtn');

  const pauseProgram = `from robot_config import create_robot\nrobot = create_robot()\nawait robot.connect()\nawait robot.move(0.2, 0, 0)\nawait robot.sleep(0.8)\nawait robot.stop()\nawait robot.disconnect()\n`;
  await page.evaluate((source) => {
    const app = window.__robobuddyCi.app;
    app.files['main.py'] = source;
    app.openFile('main.py');
    app.editor.setFile('main.py', source);
    app.workspaceGeneration += 1;
  }, pauseProgram);
  await page.click('#runBtn');
  await expect(page.locator('#simActionLabel')).toContainText('sleep()', { timeout: 90_000 });
  await page.click('#pauseBtn');
  const pausedClock = await page.locator('#simCanvas').getAttribute('data-simulation-clock-s');
  await page.waitForTimeout(120);
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-simulation-clock-s', pausedClock || '0');
  await page.click('#pauseBtn');
  await expect(page.locator('#statusMessage')).toHaveText('MicroDuck Python run complete', { timeout: 90_000 });

  const cursorProgram = `from robot_config import create_robot\nrobot = create_robot()\nawait robot.connect()\nawait robot.enable(True)\n# Run-to-Cursor should report the next bridge boundary, not this comment.\nawait robot.move(0.1, 0, 0)\nawait robot.stop()\nawait robot.disconnect()\n`;
  await page.evaluate((source) => {
    const app = window.__robobuddyCi.app;
    app.files['main.py'] = source;
    app.openFile('main.py');
    app.editor.setFile('main.py', source);
    app.editor.cm.setCursor({ line: 4, ch: 0 });
    app.workspaceGeneration += 1;
  }, cursorProgram);
  await page.click('#cursorBtn');
  await expect(page.locator('#statusMessage')).toHaveText('Stopped at main.py:6', { timeout: 90_000 });

  const stuckProgram = `from robot_config import create_robot\nrobot = create_robot()\nawait robot.connect()\nwhile True:\n    pass\n`;
  await page.evaluate((source) => {
    const app = window.__robobuddyCi.app;
    app.files['main.py'] = source;
    app.openFile('main.py');
    app.editor.setFile('main.py', source);
    app.workspaceGeneration += 1;
  }, stuckProgram);
  await page.click('#runBtn');
  await expect(page.locator('#simActionLabel')).toContainText('connect()', { timeout: 90_000 });
  await page.click('#stopBtn');
  await expect(page.locator('#statusMessage')).toHaveText('Simulation stopped');

  await page.evaluate((source) => {
    const app = window.__robobuddyCi.app;
    app.files['main.py'] = source;
    app.openFile('main.py');
    app.editor.setFile('main.py', source);
    app.workspaceGeneration += 1;
  }, pauseProgram);
  await page.click('#runBtn');
  await expect(page.locator('#simActionLabel')).toContainText('sleep()', { timeout: 90_000 });
  await page.evaluate(() => window.__robobuddyCi.app.sim.executeCommand('move', { vx: -0.1 }, { source: 'human', controllerId: 'cycle04-test', durationMs: 250 }));
  await expect(page.locator('#problemsPanel')).toContainText('trusted manual simulator command preempted', { timeout: 20_000 });
  await expect(page.locator('#runBtn')).toBeEnabled();

  await page.evaluate((source) => {
    const app = window.__robobuddyCi.app;
    app.files['main.py'] = source;
    app.openFile('main.py');
    app.editor.setFile('main.py', source);
    app.workspaceGeneration += 1;
  }, cursorProgram);
  await page.click('#runBtn');
  await expect(page.locator('#statusMessage')).toHaveText('MicroDuck Python run complete', { timeout: 90_000 });
});

test('MicroDuck Python reset, workspace edit, and profile switch cancel cleanly and preserve later runs', async ({ page }) => {
  await page.goto('/?ci=python-lifecycle');
  await page.selectOption('#robotSelect', 'microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
  const longProgram = `from robot_config import create_robot\nrobot = create_robot()\nawait robot.connect()\nawait robot.move(0.2, 0, 0)\nawait robot.sleep(2)\nawait robot.disconnect()\n`;
  const shortProgram = `from robot_config import create_robot\nrobot = create_robot()\nawait robot.connect()\nprint((await robot.get_state())["simulationMode"])\nawait robot.disconnect()\n`;
  const installProgram = async (source) => {
    await page.evaluate((program) => {
      const app = window.__robobuddyCi.app;
      app.files['main.py'] = program;
      app.openFile('main.py');
      app.editor.setFile('main.py', program);
      app.workspaceGeneration += 1;
    }, source);
  };
  const expectRuntimeReleased = async () => {
    await expect.poll(() => page.evaluate(() => ({
      active: window.__robobuddyCi.app.microduckRuntime.isActive(),
      lease: window.__robobuddyCi.app.sim.backend.commandBus.snapshot().lease,
      audio: window.__robobuddyCi.app.sim.getState().audio,
    }))).toMatchObject({ active: false, lease: null, audio: { sound: null, theremin: false, chorale: false } });
  };

  await installProgram(longProgram);
  await page.click('#runBtn');
  await expect(page.locator('#simActionLabel')).toContainText('sleep()', { timeout: 90_000 });
  await page.click('#resetBtn');
  await expect(page.locator('#statusMessage')).toHaveText('Simulation reset', { timeout: 30_000 });
  await expectRuntimeReleased();

  await installProgram(longProgram);
  await page.click('#runBtn');
  await expect(page.locator('#simActionLabel')).toContainText('sleep()', { timeout: 90_000 });
  await page.evaluate((source) => window.__robobuddyCi.app.onEdit('main.py', `${source}\n# workspace generation changed\n`), longProgram);
  await expectRuntimeReleased();

  await installProgram(longProgram);
  await page.click('#runBtn');
  await expect(page.locator('#simActionLabel')).toContainText('sleep()', { timeout: 90_000 });
  await page.selectOption('#robotSelect', 'unitree');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
  expect(await page.evaluate(() => window.__robobuddyCi.app.microduckRuntime.isActive())).toBe(false);
  await page.selectOption('#robotSelect', 'microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
  await installProgram(shortProgram);
  await page.click('#runBtn');
  await expect(page.locator('#statusMessage')).toHaveText('MicroDuck Python run complete', { timeout: 90_000 });
  await expect(page.locator('#problemsPanel')).toContainText('policy_sim');
});

test('MicroDuck Python errors retain original main.py line attribution', async ({ page }) => {
  await page.goto('/?ci=python-source');
  await page.selectOption('#robotSelect', 'microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });
  const audioSource = `from robot_config import create_robot\nrobot = create_robot()\nawait robot.connect()\nawait robot.sound("chirp")\n`;
  await page.evaluate((program) => {
    const app = window.__robobuddyCi.app;
    app.files['main.py'] = program;
    app.openFile('main.py');
    app.editor.setFile('main.py', program);
    app.workspaceGeneration += 1;
  }, audioSource);
  await page.click('#runBtn');
  await expect(page.locator('#problemsPanel')).toContainText('AUDIO_LOCKED', { timeout: 90_000 });
  await expect(page.locator('#problemsPanel')).toContainText('main.py", line 4');
  await expect(page.locator('#statusMessage')).toHaveText('MicroDuck audio is locked');

  const source = `from robot_config import create_robot\nrobot = create_robot()\nawait robot.connect()\nawait robot.set_mode("not-a-mode")\n`;
  await page.evaluate((program) => {
    const app = window.__robobuddyCi.app;
    app.files['main.py'] = program;
    app.openFile('main.py');
    app.editor.setFile('main.py', program);
    app.workspaceGeneration += 1;
  }, source);
  await page.click('#runBtn');
  await expect(page.locator('#problemsPanel')).toContainText('INVALID_ARGUMENT', { timeout: 90_000 });
  await expect(page.locator('#problemsPanel')).toContainText('main.py", line 4');
});
