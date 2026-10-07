import { test, expect } from '@playwright/test';

test('SO-101 live physical bridge uses model timestep, actual feedback, pause, and stale ownership', async ({ page }) => {
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await page.goto('/physics-slice.html', { waitUntil: 'domcontentloaded' });

  const result = await page.evaluate(async () => {
    const [{ PhysicsSession }, { BrowserMuJoCoBackend }, { SO101_PHASE2A_SCENE }, { LivePythonBridge }] = await Promise.all([
      import('/src/physics/session.js'),
      import('/src/physics/browser-mujoco-backend.js'),
      import('/src/physics/so101-scene.js'),
      import('/src/runtime/live-python-bridge.js'),
    ]);
    const session = new PhysicsSession(new BrowserMuJoCoBackend(), { sessionId: 'p3-live-python-browser' });
    try {
      await session.loadScene(structuredClone(SO101_PHASE2A_SCENE));
      const bridge = new LivePythonBridge(session);
      const connection = await bridge.connect('so101_follower');
      const accepted = await bridge.sendAction({ shoulder_pan: 0.4, elbow_flex: 0.2 }, { commandId: 'p3-multi', maxSteps: 100 });
      const acceptedActual = accepted.observation.joints.shoulder_pan.positionRad;
      const cadence = await bridge.advanceControllerInterval(0.02);
      const afterCadence = await bridge.getObservation();
      const goal = await bridge.waitForGoal({ shoulder_pan: 0.4 }, { toleranceRad: 1e-6, timeoutSeconds: 0.02, controllerPeriodSeconds: 0.02 });
      const beforePause = await bridge.getObservation();
      await bridge.pause();
      const paused = await bridge.advance(0.02);
      await bridge.resume();
      const afterResume = await bridge.advance(0.02);
      await session.reset();
      let staleCode = null;
      try { await bridge.getObservation(); } catch (error) { staleCode = error.code || null; }
      return { connection, accepted, acceptedActual, cadence, afterCadence, goal, beforePause, paused, afterResume, staleCode };
    } finally {
      session.dispose();
    }
  });

  expect(result.connection.apiVersion).toBe('robobuddy.sim.v1');
  expect(result.connection.timestepSeconds).toBeCloseTo(0.005, 12);
  expect(result.accepted.status).toBe('accepted');
  expect(result.accepted.acceptedTargetsRad).toEqual({ shoulder_pan: 0.4, elbow_flex: 0.2 });
  expect(Math.abs(result.acceptedActual - 0.4)).toBeGreaterThan(0.05);
  expect(result.cadence.physicsSteps).toBe(4);
  expect(result.cadence.controllerPeriodSeconds).toBeCloseTo(0.02, 12);
  expect(result.cadence.observation.simulationTimeSeconds).toBeCloseTo(0.02, 9);
  expect(result.afterCadence.joints.shoulder_pan.positionRad).not.toBeCloseTo(0.4, 5);
  expect(result.goal.status).toBe('timeout');
  expect(Math.abs(result.goal.errorsRad.shoulder_pan)).toBeGreaterThan(1e-6);
  expect(result.paused.simulationTimeSeconds).toBeCloseTo(result.beforePause.simulationTimeSeconds, 12);
  expect(result.afterResume.simulationTimeSeconds - result.beforePause.simulationTimeSeconds).toBeCloseTo(0.02, 9);
  expect(result.staleCode).toBe('STALE_LIVE_SESSION');
  expect(pageErrors, pageErrors.join('\n\n')).toEqual([]);
});
