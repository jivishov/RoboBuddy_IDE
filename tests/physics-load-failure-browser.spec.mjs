import { test, expect } from '@playwright/test';

test('physical model reload failure invalidates the previous browser-mujoco authority without legacy fallback', async ({ page }) => {
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await page.goto('/physics-slice.html', { waitUntil: 'domcontentloaded' });

  const result = await page.evaluate(async () => {
    const [{ PhysicsSession }, { BrowserMuJoCoBackend }, { registerModelPackage }, { PHYSICS_BACKEND_API_VERSION }, { PHASE1_SCENE }] = await Promise.all([
      import('/src/physics/session.js'),
      import('/src/physics/browser-mujoco-backend.js'),
      import('/src/physics/model-registry.js'),
      import('/src/physics/backend-contract.js'),
      import('/src/physics/phase1-scene.js'),
    ]);

    const packageId = 'phase2a-intentionally-missing-model';
    registerModelPackage({
      id: packageId,
      robotId: 'phase2a_failure_fixture',
      modelId: 'phase2a-intentionally-missing-model-v1',
      source: { url: 'https://example.invalid/controlled-test-source', revision: 'test-only' },
      license: 'test-only',
      asset: 'models/phase2a-intentionally-missing/model.xml',
      sha256: '0000000000000000000000000000000000000000000000000000000000000000',
      physics: { timestepSeconds: 0.002, integrator: 'RK4' },
      controllers: ['test_position'],
      joints: [{ id: 'test_joint' }],
      actuators: [{ id: 'test_actuator', jointId: 'test_joint', controllerId: 'test_position', command: 'position-rad' }],
      bodies: [{ id: 'test_body' }],
      sceneConstraints: { fixtures: [], objects: [] },
    });

    const scene = {
      schemaVersion: PHYSICS_BACKEND_API_VERSION,
      id: 'phase2a-load-failure-fixture',
      revision: 'phase2a-load-failure-fixture-v1',
      robotId: 'phase2a_failure_fixture',
      modelPackage: packageId,
      legacyTaskId: null,
      physics: { timestepSeconds: 0.002, integrator: 'RK4' },
      fixtures: [],
      objects: [],
      controllers: ['test_position'],
      taskGoal: null,
    };

    const backend = new BrowserMuJoCoBackend();
    const session = new PhysicsSession(backend, { sessionId: 'phase2a-load-failure-browser' });
    const valid = await session.loadScene(structuredClone(PHASE1_SCENE));
    const beforeFailure = await session.getDiagnostics();

    let error = null;
    try {
      await session.loadScene(scene);
    } catch (failure) {
      error = String(failure?.message || failure);
    }

    let staleCommandError = null;
    try {
      await session.sendCommand(
        { type: 'set_joint_target', targetRad: 0.2 },
        { commandId: 'must-not-reach-stale-phase1', maxSteps: 10 },
      );
    } catch (failure) {
      staleCommandError = String(failure?.message || failure);
    }

    const diagnostics = await session.getDiagnostics();
    session.dispose();
    return { valid, beforeFailure, error, staleCommandError, diagnostics };
  });

  expect(result.valid.robotId).toBe('phase1_articulated_joint');
  expect(result.beforeFailure.backend).toBe('browser-mujoco');
  expect(result.beforeFailure.loaded).toBe(true);
  expect(result.beforeFailure.modelPackageId).toBe('phase1-vertical-slice');

  expect(result.error).toContain('HTTP 404');
  expect(result.staleCommandError).toContain('No physical scene loaded');
  expect(result.diagnostics.backend).toBe('browser-mujoco');
  expect(result.diagnostics.state).toBe('failed');
  expect(result.diagnostics.loaded).toBe(false);
  expect(result.diagnostics.workerActive).toBe(false);
  expect(result.diagnostics.sceneRevision).toBeNull();
  expect(result.diagnostics.robotId).toBeNull();
  expect(result.diagnostics.modelPackageId).toBeNull();
  expect(result.diagnostics.model).toBeNull();
  expect(pageErrors, pageErrors.join('\n\n')).toEqual([]);
});
