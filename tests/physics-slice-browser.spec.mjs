import { existsSync, readFileSync } from 'node:fs';
import { test, expect } from '@playwright/test';

const NATIVE_REFERENCE_PATH = '/tmp/robobuddy-native-reference.json';
const PHASE1_CONFORMANCE = Object.freeze({
  simulationTimeSeconds: 1e-9,
  jointPositionRad: 1e-5,
  jointVelocityRadS: 1e-4,
  bodyPositionM: 1e-5,
});

function expectNear(label, actual, expected, tolerance) {
  const delta = Math.abs(Number(actual) - Number(expected));
  expect(Number.isFinite(delta), `${label} must be finite`).toBe(true);
  expect(delta, `${label} delta ${delta} exceeded Phase 1 tolerance ${tolerance}`).toBeLessThanOrEqual(tolerance);
  return delta;
}

test('Phase 1 browser MuJoCo slice is fixed-step, paused-safe, bounded, and native-conformant', async ({ page }) => {
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));

  await page.goto('/physics-slice.html', { waitUntil: 'domcontentloaded' });
  await page.locator('#load').click();
  await expect(page.locator('#status')).toHaveText('Load scene complete', { timeout: 30_000 });
  await expect(page.locator('#schemaVersion')).toHaveText('0.2.0-alpha.1');
  await expect(page.locator('#view')).toHaveText('ground_truth');
  await expect(page.locator('#worldFrame')).toContainText('mujoco_world · right-handed · +Z');
  await expect(page.locator('#modelSha')).toHaveText(/^[0-9a-f]{64}$/);
  await expect(page.locator('#engineVersion')).toHaveText('3.11.0');
  await expect(page.locator('#timestep')).toHaveText('0.002000 s');
  await expect(page.locator('#time')).toHaveText('0.0000 s');

  // Wall-clock/render time is not physics time: waiting alone must not advance MuJoCo.
  await page.waitForTimeout(250);
  await page.locator('#observe').click();
  await expect(page.locator('#status')).toHaveText('Read observation complete');
  await expect(page.locator('#time')).toHaveText('0.0000 s');

  await page.locator('#right').click();
  await expect(page.locator('#status')).toHaveText('Set target +0.8 rad complete');
  await expect(page.locator('#target')).toHaveText('0.80000 rad');
  let diagnostics = await page.evaluate(() => structuredClone(window.__phase1Diagnostics));
  expect(diagnostics.remainingCommandSteps).toBe(500);

  await page.waitForTimeout(250);
  await page.locator('#observe').click();
  await expect(page.locator('#time')).toHaveText('0.0000 s');

  // A paused step request is a no-op: no simulated time and no command budget are consumed.
  await page.locator('#pause').click();
  await expect(page.locator('#status')).toHaveText('Pause complete');
  await page.locator('#step100').click();
  await expect(page.locator('#status')).toHaveText('Advance 100 steps complete');
  await expect(page.locator('#time')).toHaveText('0.0000 s');
  diagnostics = await page.evaluate(() => structuredClone(window.__phase1Diagnostics));
  expect(diagnostics.state).toBe('paused');
  expect(diagnostics.remainingCommandSteps).toBe(500);

  await page.locator('#resume').click();
  await expect(page.locator('#status')).toHaveText('Resume complete');
  await page.locator('#step500').click();
  await expect(page.locator('#status')).toHaveText('Advance 500 steps complete', { timeout: 30_000 });
  await expect(page.locator('#time')).toHaveText('1.0000 s');
  await expect(page.locator('#contactsReadable')).toHaveText('yes');

  const browser = await page.evaluate(() => structuredClone(window.__phase1Observation));
  expect(browser.schemaVersion).toBe('0.2.0-alpha.1');
  expect(browser.sessionId).toBe('phase1-browser-slice');
  expect(browser.epoch).toBe(1);
  expect(browser.view).toBe('ground_truth');
  expect(browser.robotId).toBe('phase1_articulated_joint');
  expect(browser.frames.world).toEqual({
    id: 'mujoco_world', handedness: 'right-handed', upAxis: '+Z', linearUnit: 'm', angularUnit: 'rad',
  });
  expect(browser.model.id).toBe('phase1-vertical-slice-v1');
  expect(browser.model.asset).toBe('models/vertical-slice/model.xml');
  expect(browser.model.sha256).toMatch(/^[0-9a-f]{64}$/);

  const boxZ = Number(browser.bodies.free_box.positionM[2]);
  expect(Number.isFinite(boxZ)).toBe(true);
  expect(boxZ).toBeLessThan(1.0);
  expect(boxZ).toBeGreaterThan(0.38);
  expect(browser.contactCount).toBeGreaterThan(0);
  expect(Math.abs(Number(browser.joints.hinge.positionRad))).toBeGreaterThan(0.01);

  diagnostics = await page.evaluate(() => structuredClone(window.__phase1Diagnostics));
  expect(diagnostics.remainingCommandSteps).toBe(0);
  expect(diagnostics.model.sha256).toBe(browser.model.sha256);

  // Wall time after a completed step batch still cannot advance simulation by itself.
  await page.waitForTimeout(250);
  await page.locator('#observe').click();
  await expect(page.locator('#time')).toHaveText('1.0000 s');

  await page.locator('#step100').click();
  await expect(page.locator('#status')).toContainText('exceeds remaining command budget 0');

  if (existsSync(NATIVE_REFERENCE_PATH)) {
    const native = JSON.parse(readFileSync(NATIVE_REFERENCE_PATH, 'utf8'));
    expect(native.engine.version).toBe('3.11.0');
    expect(native.model.sha256).toBe(browser.model.sha256);
    expect(native.frames.world).toEqual(browser.frames.world);
    const deltas = {
      simulationTimeSeconds: expectNear('simulation time', browser.simulationTimeSeconds, native.simulationTimeSeconds, PHASE1_CONFORMANCE.simulationTimeSeconds),
      jointPositionRad: expectNear('hinge position', browser.joints.hinge.positionRad, native.joints.hinge.positionRad, PHASE1_CONFORMANCE.jointPositionRad),
      jointVelocityRadS: expectNear('hinge velocity', browser.joints.hinge.velocityRadS, native.joints.hinge.velocityRadS, PHASE1_CONFORMANCE.jointVelocityRadS),
      freeBoxX: expectNear('free box x', browser.bodies.free_box.positionM[0], native.bodies.free_box.positionM[0], PHASE1_CONFORMANCE.bodyPositionM),
      freeBoxY: expectNear('free box y', browser.bodies.free_box.positionM[1], native.bodies.free_box.positionM[1], PHASE1_CONFORMANCE.bodyPositionM),
      freeBoxZ: expectNear('free box z', browser.bodies.free_box.positionM[2], native.bodies.free_box.positionM[2], PHASE1_CONFORMANCE.bodyPositionM),
    };
    console.log('Phase 1 browser/native deltas', deltas);
  }

  expect(pageErrors, pageErrors.join('\n\n')).toEqual([]);
});
