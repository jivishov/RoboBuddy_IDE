import { test, expect } from '@playwright/test';

test('consolidated policy-core trace covers commands, modes, contact truth and recovery', async ({ page }) => {
  test.setTimeout(90_000);
  await page.goto('/?ci=1');
  await page.selectOption('#robotSelect', 'microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });

  const trace = await page.evaluate(async () => {
    const host = window.__robobuddyCi.app.sim;
    const context = { source: 'python', controllerId: 'cycle-02-trace', durationMs: 5000 };
    const advancePolicy = async (seconds) => {
      const ticks = Math.ceil(seconds / 0.02);
      for (let index = 0; index < ticks; index += 1) {
        await host.advanceTime(0.02);
        await new Promise((resolve) => setTimeout(resolve, 0));
      }
    };
    const runSkill = async (skill, seconds) => {
      const result = await host.executeCommand('do', { skill }, { source: 'human' });
      const active = host.getState().activePolicy;
      await advancePolicy(seconds);
      return { completedAtStart: result.completed, active, after: host.getState().activePolicy };
    };
    const move = await host.executeCommand('move', { vx: 1, vy: -1, yaw: 3 }, context);
    const head = await host.executeCommand('head', { neckPitch: 0.2, headPitch: -0.3, headYaw: 0.4, headRoll: 0.1 }, context);
    const look = await host.executeCommand('look', { x: 0.25, y: 0.1, z: 0.2, neckPitch: 0.15 }, { source: 'human' });
    const pose = await host.executeCommand('pose', { x: 1, y: 1, z: -1, roll: 1, pitch: -1, yaw: 1 }, context);
    await host.executeCommand('mouth', { open: 2 }, context);
    await host.advanceTime(0.12);
    const walking = host.getState();
    await host.executeCommand('move', { vx: 0.2 }, context);
    await host.executeCommand('head', { headYaw: 0.3 }, context);
    await host.executeCommand('stop', {}, context);
    const stoppedCommand = host.getState();
    await host.executeCommand('enable', { enabled: false }, { source: 'human' });
    const disabledPolicy = host.getState();
    const initResult = await host.executeCommand('init', {}, { source: 'human' });
    const initializing = host.getState();
    await host.advanceTime(2.05);
    const initialized = host.getState();
    await host.executeCommand('enable', { on: true }, { source: 'human' });
    const eagerPolicies = host.backend.policyRuntime.sessions.size;

    const walkingSkills = {
      groundPick: await runSkill('ground_pick', 2.9),
      kickLeft: await runSkill('kick_left', 0.6),
      kickRight: await runSkill('kick_right', 0.6),
      roulade: await runSkill('roulade', 1.1),
    };
    const sitDown = await runSkill('sit_toggle', 0.04);
    const sitUp = await runSkill('sit_toggle', 1.1);
    walkingSkills.sit = { down: sitDown, up: sitUp };

    await host.executeCommand('spawn_ball', { position: [0.055, 0, 0.035] }, { source: 'human' });
    const ballBefore = host.getState().ball.position;
    await host.executeCommand('move', { vx: 0.3, vy: 0, yaw: 0 }, context);
    await host.advanceTime(0.4);
    const ballAfter = host.getState().ball.position;

    await host.executeCommand('relax', {}, { source: 'human' });
    const relaxed = host.getState();
    await host.advanceTime(0.1);
    const relaxControlMax = Math.max(...host.backend.dynamics.data.ctrl.map(Math.abs));
    const relaxedAfterGravity = host.getState();
    const enableFromRelax = await host.executeCommand('enable', { on: true }, { source: 'human' });
    const reinitializing = host.getState();
    await host.advanceTime(2.05);
    const enabledAfterRamp = host.getState();

    let invalidModeCode = null;
    try { await host.executeCommand('set_mode', { mode: 'invalid' }, { source: 'human' }); }
    catch (error) { invalidModeCode = error.code; }
    await host.executeCommand('set_mode', { mode: 'roller' }, { source: 'human' });
    const lazyPolicies = host.backend.policyRuntime.sessions.size;
    const rollerSkills = {
      crouch: await runSkill('ground_pick', 2.2),
      kick: await runSkill('kick_left', 0.6),
      roulade: await runSkill('roulade', 1.1),
    };
    const rollerSitDown = await runSkill('sit_toggle', 0.04);
    const rollerSitUp = await runSkill('sit_toggle', 1.1);
    rollerSkills.sit = { down: rollerSitDown, up: rollerSitUp };
    const rollerLimit = await host.executeCommand('move', { vx: 2, vy: 1, yaw: 2 }, context);
    host.perturb('face_down');
    await host.advanceTime(0.25);
    const rollerRecovery = host.getState();

    await host.executeCommand('set_mode', { mode: 'walking' }, { source: 'human' });
    host.perturb('face_up');
    await host.advanceTime(6.3);
    const walkingRecovery = host.getState();

    while (host.backend.inferenceGate.inFlight) await new Promise((resolve) => setTimeout(resolve, 0));
    const originalInfer = host.backend.policyRuntime.infer.bind(host.backend.policyRuntime);
    let releaseInference;
    host.backend.policyRuntime.infer = () => new Promise((resolve) => { releaseInference = resolve; });
    for (let index = 0; index < 4 && !host.backend.inferenceGate.inFlight; index += 1) await host.advanceTime(0.005);
    const targetsBeforePause = [...host.getState().targets];
    host.pause();
    releaseInference(new Float32Array(14).fill(1));
    await new Promise((resolve) => setTimeout(resolve, 0));
    const staleInferenceRejected = JSON.stringify(targetsBeforePause) === JSON.stringify(host.getState().targets);
    host.backend.policyRuntime.infer = originalInfer;
    host.resume();
    const frozen = Object.isFrozen(walkingRecovery) && Object.isFrozen(walkingRecovery.ball.position);
    return {
      move: move.applied,
      moveLimitedBy: move.limitedBy,
      head: head.applied,
      look: look.applied,
      pose: pose.applied,
      walking,
      stoppedCommand,
      disabledPolicy,
      initCompletedAtStart: initResult.completed,
      initializing,
      initialized,
      walkingSkills,
      relaxed,
      relaxControlMax,
      relaxedAfterGravity,
      enableFromRelaxCompleted: enableFromRelax.completed,
      reinitializing,
      enabledAfterRamp,
      invalidModeCode,
      eagerPolicies,
      lazyPolicies,
      rollerSkills,
      ballBefore,
      ballAfter,
      rollerLimit: rollerLimit.applied,
      rollerLimitedBy: rollerLimit.limitedBy,
      rollerRecovery,
      walkingRecovery,
      staleInferenceRejected,
      frozen,
    };
  });

  expect(trace.move).toEqual({ vx: 0.3, vy: -0.3, yaw: 1.5 });
  expect(trace.moveLimitedBy).toEqual(expect.arrayContaining(['vx_limit', 'vy_limit', 'yaw_limit']));
  expect(trace.pose).toMatchObject({ x: 0, y: 0, yaw: 0, z: -0.025, roll: 0.26, pitch: -0.26 });
  expect(trace.look.solvedHead).toMatchObject({ neckPitch: 0.15, headRoll: 0 });
  expect(Object.values(trace.look.solvedHead).every(Number.isFinite)).toBe(true);
  expect(trace.walking).toMatchObject({ simulationMode: 'policy_sim', stateKind: 'browser_policy_sim', hardwareValidated: false, policySimulationAvailable: true, mouth: 1 });
  expect(trace.stoppedCommand).toMatchObject({ enabled: true, movement: { applied: [0, 0, 0] }, head: [0, 0, 0.3, 0] });
  expect(trace.disabledPolicy).toMatchObject({ enabled: false, actuationEnabled: true });
  expect(trace.initCompletedAtStart).toBe(false);
  expect(trace.initializing).toMatchObject({ enabled: false, actuationEnabled: true, phase: 'initializing' });
  expect(trace.initialized).toMatchObject({ enabled: false, actuationEnabled: true });
  expect(trace.initialized.targets.every(Number.isFinite)).toBe(true);
  expect(trace.walkingSkills).toMatchObject({
    groundPick: { completedAtStart: false, active: 'ground_pick', after: 'stand' },
    kickLeft: { completedAtStart: false, active: 'kick_left', after: 'stand' },
    kickRight: { completedAtStart: false, active: 'kick_right', after: 'stand' },
    roulade: { completedAtStart: false, active: 'roulade', after: 'stand' },
    sit: { down: { active: 'sitstand' }, up: { active: 'sitstand', after: 'stand' } },
  });
  expect(trace.relaxed).toMatchObject({ enabled: false, actuationEnabled: false, activePolicy: 'disabled', movement: { applied: [0, 0, 0] } });
  expect(trace.relaxControlMax).toBe(0);
  expect(trace.relaxedAfterGravity.simulatedPose.position[2]).toBeLessThan(trace.relaxed.simulatedPose.position[2]);
  expect(trace.enableFromRelaxCompleted).toBe(false);
  expect(trace.reinitializing.phase).toBe('initializing');
  expect(trace.enabledAfterRamp).toMatchObject({ enabled: true, actuationEnabled: true });
  expect(trace.invalidModeCode).toBe('INVALID_ARGUMENT');
  expect(trace.walking.joints).toHaveLength(14); expect(trace.walking.targets).toHaveLength(14);
  expect(trace.eagerPolicies).toBe(7); expect(trace.lazyPolicies).toBe(9);
  expect(trace.rollerSkills).toMatchObject({
    crouch: { active: 'roller_crouch', after: 'roller' },
    kick: { active: 'kick_left', after: 'roller' },
    roulade: { active: 'roulade', after: 'roller' },
    sit: { down: { active: 'sitstand' }, up: { active: 'sitstand', after: 'roller' } },
  });
  expect(trace.ballAfter).not.toEqual(trace.ballBefore);
  expect(trace.walking.ball.attached).toBe(false);
  expect(trace.rollerLimit).toEqual({ vx: 0.6, vy: 0, yaw: 0.3 });
  expect(trace.rollerLimitedBy).toContain('roller_no_strafe');
  expect(trace.rollerRecovery.safety.resetFallback).toBe(true);
  expect(trace.walkingRecovery.safety.resetFallback).toBe(true);
  expect(trace.staleInferenceRejected).toBe(true);
  expect(trace.frozen).toBe(true);

  await page.click('#pauseBtn');
  await expect(page.locator('#statusMessage')).toContainText('paused');
  await page.click('#pauseBtn');
  await expect(page.locator('#statusMessage')).toContainText('resumed');
  await page.click('#stopBtn');
  await expect(page.locator('#statusMessage')).toContainText('stopped');
});
