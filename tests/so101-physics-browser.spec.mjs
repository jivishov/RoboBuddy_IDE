import { existsSync, readFileSync } from 'node:fs';
import { test, expect } from '@playwright/test';

const NATIVE_REFERENCE_PATH = '/tmp/robobuddy-native-so101.json';
const JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'];
// These are browser/native numerical-conformance tolerances, not hardware-fidelity tolerances.
// Position tolerance 2e-4 rad is about 0.0115 degrees; velocity tolerance is 2e-3 rad/s.
const PARITY = Object.freeze({ simulationTimeSeconds: 1e-9, jointPositionRad: 2e-4, jointVelocityRadS: 2e-3 });

function expectNear(label, actual, expected, tolerance) {
  const delta = Math.abs(Number(actual) - Number(expected));
  expect(Number.isFinite(delta), `${label} must be finite`).toBe(true);
  expect(delta, `${label} delta ${delta} exceeded SO-101 Phase 2A tolerance ${tolerance}`).toBeLessThanOrEqual(tolerance);
  return delta;
}

test('SO-101 Phase 2A loads, resets deterministically, and moves through bounded actuators', async ({ page }) => {
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await page.goto('/physics-slice.html', { waitUntil: 'domcontentloaded' });

  const result = await page.evaluate(async () => {
    const [{ PhysicsSession }, { BrowserMuJoCoBackend }, { SO101_PHASE2A_SCENE }] = await Promise.all([
      import('/src/physics/session.js'),
      import('/src/physics/browser-mujoco-backend.js'),
      import('/src/physics/so101-scene.js'),
    ]);
    const backend = new BrowserMuJoCoBackend();
    const session = new PhysicsSession(backend, { sessionId: 'so101-phase2a-browser' });
    const finiteJointState = (observation) => Object.values(observation.joints).every((joint) =>
      Number.isFinite(joint.positionRad) && Number.isFinite(joint.velocityRadS));

    try {
      const loaded = await session.loadScene(structuredClone(SO101_PHASE2A_SCENE));
      const initial = structuredClone(loaded.observation);
      const diagnostics = await session.getDiagnostics();

      const accepted = await session.sendCommand(
        { type: 'set_joint_target', jointId: 'shoulder_pan', targetRad: 0.4 },
        { commandId: 'so101-single', maxSteps: 100 },
      );
      const acceptancePosition = accepted.observation.joints.shoulder_pan.positionRad;
      const acceptanceTarget = accepted.observation.joints.shoulder_pan.targetRad;
      const singleJointTrajectory = [];
      let moved = null;
      for (let checkpoint = 1; checkpoint <= 4; checkpoint += 1) {
        moved = await session.advanceSteps(25);
        singleJointTrajectory.push({
          step: checkpoint * 25,
          simulationTimeSeconds: moved.simulationTimeSeconds,
          positionRad: moved.joints.shoulder_pan.positionRad,
          velocityRadS: moved.joints.shoulder_pan.velocityRadS,
        });
      }

      let jointRangeError = null;
      try {
        await session.sendCommand(
          { type: 'set_joint_target', jointId: 'wrist_roll', targetRad: 2.8 },
          { commandId: 'so101-joint-out-of-range', maxSteps: 10 },
        );
      } catch (error) {
        jointRangeError = String(error?.message || error);
      }

      let controlRangeError = null;
      try {
        await session.sendCommand(
          { type: 'set_joint_target', jointId: 'wrist_roll', targetRad: 3.0 },
          { commandId: 'so101-control-out-of-range', maxSteps: 10 },
        );
      } catch (error) {
        controlRangeError = String(error?.message || error);
      }

      const reset1 = (await session.reset()).result;
      const reset2 = (await session.reset()).result;

      const targets = {
        shoulder_pan: 0.18,
        shoulder_lift: -0.22,
        elbow_flex: 0.24,
        wrist_flex: -0.16,
        wrist_roll: 0.12,
        gripper: 0.20,
      };
      for (const [jointId, targetRad] of Object.entries(targets)) {
        await session.sendCommand(
          { type: 'set_joint_target', jointId, targetRad },
          { commandId: `multi-${jointId}`, maxSteps: 200 },
        );
      }
      const multi1 = await session.advanceSteps(200);

      await session.reset();
      for (const [jointId, targetRad] of Object.entries(targets)) {
        await session.sendCommand(
          { type: 'set_joint_target', jointId, targetRad },
          { commandId: `repeat-${jointId}`, maxSteps: 200 },
        );
      }
      const multi2 = await session.advanceSteps(200);

      return {
        loaded: initial,
        diagnostics,
        acceptancePosition,
        acceptanceTarget,
        singleJointTrajectory,
        moved,
        jointRangeError,
        controlRangeError,
        reset1,
        reset2,
        multi1,
        multi2,
        finiteMoved: finiteJointState(moved),
        finiteMulti1: finiteJointState(multi1),
        finiteMulti2: finiteJointState(multi2),
      };
    } finally {
      session.dispose();
    }
  });

  expect(result.loaded.robotId).toBe('so101_follower');
  expect(result.loaded.model.id).toBe('robobuddy-so101-phase2a-v1');
  expect(result.loaded.model.asset).toBe('models/so101/model.xml');
  expect(result.loaded.model.sha256).toMatch(/^[0-9a-f]{64}$/);
  expect(result.loaded.engine.version).toBe('3.11.0');
  expect(result.loaded.engine.timestepSeconds).toBeCloseTo(0.005, 12);
  expect(result.diagnostics.backend).toBe('browser-mujoco');
  expect(result.diagnostics.modelPackageId).toBe('so101-phase2a-menagerie-8161bba');
  expect(Object.keys(result.loaded.joints)).toEqual(JOINTS);
  expect(result.acceptanceTarget).toBeCloseTo(0.4, 12);
  expect(Math.abs(result.acceptancePosition - 0.4)).toBeGreaterThan(0.05);
  expect(result.singleJointTrajectory.map(({ step }) => step)).toEqual([25, 50, 75, 100]);
  expect(result.singleJointTrajectory.some(({ velocityRadS }) => Math.abs(velocityRadS) > 1e-4)).toBe(true);
  expect(result.moved.simulationTimeSeconds).toBeCloseTo(0.5, 9);
  expect(Math.abs(result.moved.joints.shoulder_pan.positionRad - result.acceptancePosition)).toBeGreaterThan(1e-4);
  expect(Number.isFinite(result.moved.joints.shoulder_pan.velocityRadS)).toBe(true);
  expect(result.finiteMoved).toBe(true);
  expect(result.jointRangeError).toContain('outside joint wrist_roll range');
  expect(result.controlRangeError).toContain('outside the actuator control range');

  for (const jointId of JOINTS) {
    expectNear(`${jointId} deterministic reset qpos`, result.reset1.joints[jointId].positionRad, result.reset2.joints[jointId].positionRad, 1e-12);
    expectNear(`${jointId} deterministic reset qvel`, result.reset1.joints[jointId].velocityRadS, result.reset2.joints[jointId].velocityRadS, 1e-12);
    const [min, max] = result.multi1.joints[jointId].jointRangeRad;
    expect(result.multi1.joints[jointId].positionRad).toBeGreaterThanOrEqual(min - 1e-8);
    expect(result.multi1.joints[jointId].positionRad).toBeLessThanOrEqual(max + 1e-8);
    expectNear(`${jointId} repeated multi-joint qpos`, result.multi1.joints[jointId].positionRad, result.multi2.joints[jointId].positionRad, 1e-10);
    expectNear(`${jointId} repeated multi-joint qvel`, result.multi1.joints[jointId].velocityRadS, result.multi2.joints[jointId].velocityRadS, 1e-10);
  }
  expect(result.finiteMulti1).toBe(true);
  expect(result.finiteMulti2).toBe(true);
  expect(result.multi1.simulationTimeSeconds).toBeCloseTo(1.0, 9);

  if (existsSync(NATIVE_REFERENCE_PATH)) {
    const native = JSON.parse(readFileSync(NATIVE_REFERENCE_PATH, 'utf8'));
    expect(native.model.sha256).toBe(result.loaded.model.sha256);
    expect(native.model.id).toBe(result.loaded.model.id);
    expect(native.engine.version).toBe('3.11.0');
    expect(native.engine.integrator).toBe('implicitfast');
    expect(native.engine.iterations).toBe(10);
    expect(native.engine.lsIterations).toBe(20);
    expect(native.resetMatchesInitial).toBe(true);
    expect(native.trajectory).toHaveLength(result.singleJointTrajectory.length);
    const trajectoryDeltas = result.singleJointTrajectory.map((browserPoint, index) => {
      const nativePoint = native.trajectory[index];
      expect(nativePoint.step).toBe(browserPoint.step);
      return {
        step: browserPoint.step,
        simulationTimeSeconds: expectNear(`SO-101 trajectory time at step ${browserPoint.step}`, browserPoint.simulationTimeSeconds, nativePoint.simulationTimeSeconds, PARITY.simulationTimeSeconds),
        shoulderPanPosition: expectNear(`SO-101 shoulder_pan position at step ${browserPoint.step}`, browserPoint.positionRad, nativePoint.positionRad, PARITY.jointPositionRad),
        shoulderPanVelocity: expectNear(`SO-101 shoulder_pan velocity at step ${browserPoint.step}`, browserPoint.velocityRadS, nativePoint.velocityRadS, PARITY.jointVelocityRadS),
      };
    });
    expectNear('SO-101 final simulation time', result.moved.simulationTimeSeconds, native.simulationTimeSeconds, PARITY.simulationTimeSeconds);
    expectNear('SO-101 final shoulder_pan position', result.moved.joints.shoulder_pan.positionRad, native.joints.shoulder_pan.positionRad, PARITY.jointPositionRad);
    expectNear('SO-101 final shoulder_pan velocity', result.moved.joints.shoulder_pan.velocityRadS, native.joints.shoulder_pan.velocityRadS, PARITY.jointVelocityRadS);
    console.log('SO-101 Phase 2A browser/native trajectory deltas', trajectoryDeltas);
  }

  expect(pageErrors, pageErrors.join('\n\n')).toEqual([]);
});
