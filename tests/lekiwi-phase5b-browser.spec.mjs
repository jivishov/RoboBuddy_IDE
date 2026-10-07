import { expect, test } from '@playwright/test';
import { writeFile } from 'node:fs/promises';

const attach = async (testInfo, name, value) => {
  const path = testInfo.outputPath(name);
  await writeFile(path, JSON.stringify(value, null, 2));
  await testInfo.attach(name, { path, contentType: 'application/json' });
};

test('LeKiwi Phase 5B drives, grasps, carries, delivers and returns through one browser MuJoCo authority', async ({ page }, testInfo) => {
  test.setTimeout(600_000);
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  // Headless Chromium has no WebMCP implementation, so stand in a minimal document.modelContext.
  // Registration still goes through the app's real gating, epoch and abort-signal path.
  await page.addInitScript(() => {
    const registrations = [];
    Object.defineProperty(document, 'modelContext', {
      configurable: true,
      value: {
        registerTool(tool, options = {}) {
          registrations.push({ tool, signal: options.signal || null });
          return Promise.resolve();
        },
      },
    });
    window.__webMcpRegistrations = registrations;
  });
  await page.goto('/?ci=lekiwi-phase5b', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
  await page.locator('#robotSelect').selectOption('lekiwi');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 90_000 });

  // 1-3. The physical LeKiwi workspace loads, browser MuJoCo is the declared authority, and the
  // loaded model identity matches the registered package.
  const startup = await page.evaluate(async () => {
    const app = window.__robobuddyCi?.app;
    const backend = app?.sim?.backend;
    const state = backend?.getState?.();
    const { LEKIWI_COURIER_PACKAGE } = await import('/src/physics/lekiwi-model-package.js');
    return {
      profileId: app?.profileId,
      taskId: app?.taskId,
      simulationMode: app?.scenario?.simulationMode,
      physicalSceneId: app?.scenario?.physicalSceneId,
      backendName: backend?.constructor?.name || null,
      authority: backend?.getPhysicalAuthorityToken?.() || null,
      presentation: backend?.getPresentationAudit?.() || null,
      model: state?.observation?.model || null,
      expectedModelId: LEKIWI_COURIER_PACKAGE.modelId,
      expectedSha256: LEKIWI_COURIER_PACKAGE.sha256,
      engine: state?.observation?.engine || null,
      basePose: backend?.getBasePose?.() || null,
      beaker: state?.observation?.bodies?.empty_beaker || null,
      wheelJoints: Object.fromEntries(['base_left_wheel', 'base_back_wheel', 'base_right_wheel']
        .map((id) => [id, state?.observation?.joints?.[id] || null])),
      contactCount: state?.observation?.contactCount ?? null,
      contactsReadable: state?.observation?.contactsReadable ?? null,
      mainPy: app?.files?.['main.py'] || '',
    };
  });
  await attach(testInfo, 'lekiwi-startup-audit.json', { ...startup, pageErrors: [...pageErrors] });

  expect(startup.profileId).toBe('lekiwi');
  expect(startup.taskId).toBe('lekiwi-physical-beaker-courier');
  expect(startup.simulationMode).toBe('physical_mujoco');
  expect(startup.physicalSceneId).toBe('p5b-lekiwi-beaker-courier');
  expect(startup.backendName).toBe('LeKiwiPhysicalSimulator');
  expect(startup.model.id).toBe(startup.expectedModelId);
  expect(startup.model.sha256).toBe(startup.expectedSha256);
  expect(startup.engine.version).toBe('3.11.0');
  expect(startup.engine.timestepSeconds).toBeCloseTo(0.002, 12);
  expect(startup.authority?.sessionId).toBeTruthy();
  expect(startup.presentation.rendererIntegratesBase).toBe(false);
  expect(startup.presentation.baseTransformSource).toContain('observed MuJoCo');
  expect(startup.presentation.observationPeriodSeconds).toBeCloseTo(0.01, 9);
  // The declared initial state places the wheels exactly tangent to the floor, so MuJoCo correctly
  // reports no contact until the plant runs. Settling briefly proves the base is then held up by
  // real roller/floor contacts rather than by a scripted pose: contacts appear and it does not sink.
  expect(startup.contactCount).toBe(0);
  expect(startup.contactsReadable).toBe(true);
  const settle = await page.evaluate(async () => {
    const sim = window.__robobuddyCi.app.sim.backend;
    const beforeZ = sim.getBasePose().zM;
    await sim.advanceTime(0.2);
    const pose = sim.getBasePose();
    return {
      beforeZM: beforeZ,
      afterZM: pose.zM,
      speedMS: pose.speedMS,
      contactCount: sim.getState().observation.contactCount,
      contacts: (sim.getState().observation.contacts || []).map((c) => `${c.geom1Name}|${c.geom2Name}`),
    };
  });
  await attach(testInfo, 'lekiwi-startup-settle.json', settle);
  expect(settle.contactCount).toBeGreaterThan(0);
  // One floor contact per wheel, on a roller barrel rather than on the hub or the chassis.
  const floorRollerContacts = settle.contacts.filter((pair) => pair.includes('lekiwi_floor') && pair.includes('_roller_'));
  expect(floorRollerContacts.length).toBe(3);
  expect(settle.contacts.some((pair) => pair.includes('lekiwi_chassis'))).toBe(false);
  expect(settle.beforeZM - settle.afterZM).toBeLessThan(0.002);
  expect(settle.speedMS).toBeLessThan(0.01);
  expect(Math.abs(startup.basePose.xM)).toBeLessThan(0.01);
  expect(startup.beaker.positionM[2]).toBeGreaterThan(0.2);
  for (const joint of Object.values(startup.wheelJoints)) {
    expect(joint.controlRangeRadS[1]).toBeCloseTo(4.60061, 6);
    expect(joint.forceRangeNm[1]).toBeCloseTo(2.94, 9);
    expect(joint.targetVelocityRadS).toBe(0);
  }
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-simulator-backend', 'browser-mujoco');
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-simulation-authority', 'physics-session');
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-lekiwi-base-integration', 'mujoco-free-body');
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-lekiwi-wheel-contact', 'explicit-passive-rollers');
  expect(startup.mainPy).toContain('x.vel');
  expect(startup.mainPy).toContain('await robot.get_observation()');

  // 13. Rendering must not advance physics: many render frames, no simulation-time change.
  const renderProbe = await page.evaluate(async () => {
    const app = window.__robobuddyCi.app;
    const before = app.sim.backend.getState().observation.simulationTimeSeconds;
    for (let i = 0; i < 45; i += 1) {
      app.sim.backend.renderFrame();
      await new Promise((resolve) => requestAnimationFrame(resolve));
    }
    return { before, after: app.sim.backend.getState().observation.simulationTimeSeconds };
  });
  expect(renderProbe.after).toBe(renderProbe.before);

  // 5. Base motion comes from the physical plant: forward, lateral and yaw all arrive through
  // bounded wheel actuation, and a wheels-stopped command halts the base through contact.
  const baseMotion = await page.evaluate(async () => {
    const app = window.__robobuddyCi.app;
    const sim = app.sim.backend;
    const { bodyFrameDisplacement } = await import('/src/physics/lekiwi-task-evaluator.js');
    const runs = {};
    const drive = async (label, action, seconds) => {
      const start = sim.getBasePose();
      const accepted = await sim.applyChassisVelocity(action, { maxSteps: 12000, advanceSeconds: 0 });
      const steps = Math.round(seconds / 0.002);
      await sim.advanceTime(steps * 0.002);
      const end = sim.getBasePose();
      runs[label] = {
        requested: accepted.requestedChassisVelocity,
        wheelTargets: accepted.acceptedWheelTargetsRadS,
        wheelActual: Object.fromEntries(['base_left_wheel', 'base_back_wheel', 'base_right_wheel']
          .map((id) => [id, sim.getState().observation.joints[id].velocityRadS])),
        displacement: bodyFrameDisplacement(start, end),
        endSpeedMS: end.speedMS,
      };
      await sim.applyChassisVelocity({ 'x.vel': 0, 'y.vel': 0, 'theta.vel': 0 }, { maxSteps: 4000 });
      await sim.advanceTime(1.0);
      runs[label].stoppedSpeedMS = sim.getBasePose().speedMS;
      await sim.reset();
      await sim.advanceTime(0.4);
    };
    await sim.advanceTime(0.4);
    await drive('forward', { 'x.vel': 0.15, 'y.vel': 0, 'theta.vel': 0 }, 2.0);
    await drive('lateralLeft', { 'x.vel': 0, 'y.vel': 0.15, 'theta.vel': 0 }, 2.0);
    await drive('yawPositive', { 'x.vel': 0, 'y.vel': 0, 'theta.vel': 45 }, 2.0);
    return runs;
  });
  await attach(testInfo, 'lekiwi-base-motion.json', baseMotion);

  expect(baseMotion.forward.displacement.forwardM).toBeGreaterThan(0.20);
  expect(Math.abs(baseMotion.forward.displacement.leftM)).toBeLessThan(0.02);
  expect(baseMotion.forward.wheelActual.base_left_wheel).toBeLessThan(-1);
  expect(baseMotion.forward.wheelActual.base_right_wheel).toBeGreaterThan(1);
  expect(baseMotion.forward.stoppedSpeedMS).toBeLessThan(0.02);
  expect(baseMotion.lateralLeft.displacement.leftM).toBeGreaterThan(0.20);
  expect(Math.abs(baseMotion.lateralLeft.displacement.forwardM)).toBeLessThan(0.02);
  expect(baseMotion.lateralLeft.wheelActual.base_back_wheel).toBeLessThan(-1);
  expect(baseMotion.yawPositive.displacement.yawRad).toBeGreaterThan(1.0);
  expect(Math.hypot(baseMotion.yawPositive.displacement.forwardM, baseMotion.yawPositive.displacement.leftM)).toBeLessThan(0.02);

  // 4. Live Python owns the same physical session, and a chassis request there is translated into
  // bounded wheel targets rather than reported back as if it were motion.
  const livePython = await page.evaluate(async () => {
    const app = window.__robobuddyCi.app;
    await app.sim.backend.reset();
    app.files['main.py'] = [
      'from robobuddy.sim import connect',
      'robot = await connect("lekiwi_v1_mobile_manipulator")',
      'try:',
      '    before = await robot.get_observation()',
      '    accepted = await robot.send_action({"x.vel": 0.15, "y.vel": 0.0, "theta.vel": 0.0}, max_steps=6000)',
      '    print("wheel targets", accepted["acceptedWheelTargetsRadS"])',
      '    print("requested", accepted["requestedChassisVelocity"])',
      '    after = await robot.advance(2.0)',
      '    print("before x", before["bodies"]["lekiwi_base"]["positionM"][0])',
      '    print("after x", after["bodies"]["lekiwi_base"]["positionM"][0])',
      '    print("wheel actual", after["joints"]["base_left_wheel"]["velocityRadS"])',
      'finally:',
      '    await robot.disconnect()',
      '',
    ].join('\n');
    const ok = await app.run();
    return {
      ok: Boolean(ok),
      stdout: app.console?.stdout || '',
      stderr: app.console?.stderr || '',
      basePose: app.sim.backend.getBasePose(),
      authority: app.sim.backend.getPhysicalAuthorityToken(),
    };
  });
  await attach(testInfo, 'lekiwi-live-python.json', livePython);
  expect(livePython.stderr).toBe('');
  expect(livePython.stdout).toContain('wheel targets');
  expect(livePython.stdout).toContain('requested');
  const beforeX = Number(/before x (-?[\d.eE+]+)/.exec(livePython.stdout)?.[1]);
  const afterX = Number(/after x (-?[\d.eE+]+)/.exec(livePython.stdout)?.[1]);
  expect(Number.isFinite(beforeX) && Number.isFinite(afterX)).toBe(true);
  // The base actually moved, and it moved less than an ideal-command integration would predict,
  // because the motion had to come through wheel/ground contact.
  expect(afterX - beforeX).toBeGreaterThan(0.15);
  expect(afterX - beforeX).toBeLessThan(0.30);

  // 6-10. The full physical courier: drive, rim pinch, lift, carry, supported release, settle,
  // return home, and an evaluator verdict derived only from observations.
  const courier = await page.evaluate(async () => {
    const app = window.__robobuddyCi.app;
    const sim = app.sim.backend;
    await sim.reset();
    const { LEKIWI_COURIER_CONTROLLER } = await import('/src/physics/lekiwi-scene.js');
    const period = LEKIWI_COURIER_CONTROLLER.controllerPeriodSeconds;
    const marks = [];
    for (const stage of LEKIWI_COURIER_CONTROLLER.stages) {
      const targets = { ...stage.armTargetsRad, arm_gripper: stage.gripperRad };
      if (stage.kind === 'drive') {
        for (const waypoint of stage.waypoints) {
          let elapsed = 0;
          for (;;) {
            const step = await sim.driveTowards(waypoint);
            if (step.arrived) break;
            elapsed += period;
            if (elapsed >= stage.timeoutSeconds) break;
          }
          await sim.applyChassisVelocity({ 'x.vel': 0, 'y.vel': 0, 'theta.vel': 0 }, { maxSteps: 4000 });
          await sim.applyArmTargets(targets, { maxSteps: 4000, advanceSeconds: 0.2 });
        }
      } else {
        await sim.applyChassisVelocity({ 'x.vel': 0, 'y.vel': 0, 'theta.vel': 0 }, { maxSteps: 4000 });
        await sim.applyArmTargets(targets, { maxSteps: 12000, advanceSeconds: 0 });
        await sim.advanceTime(Math.round(stage.durationSeconds / 0.002) * 0.002);
      }
      const evaluation = sim.getTaskEvaluation();
      marks.push({
        stage: stage.name,
        simulationTimeSeconds: sim.getState().observation.simulationTimeSeconds,
        basePose: sim.getBasePose(),
        beakerM: sim.getState().observation.bodies.empty_beaker.positionM,
        graspSeen: evaluation.graspSeen,
        liftSeen: evaluation.liftSeen,
        carrySeen: evaluation.carrySeen,
        supportWhileHeldSeen: evaluation.supportWhileHeldSeen,
        releaseSeen: evaluation.releaseSeen,
        settled: evaluation.settled,
        homeReturned: evaluation.homeReturned,
      });
    }
    return { marks, evaluation: sim.getTaskEvaluation(), telemetry: sim.getTelemetry(), contacts: sim.getContacts() };
  });
  await attach(testInfo, 'lekiwi-courier-run.json', courier);

  const verdict = courier.evaluation;
  expect(verdict.serviceStopReached).toBe(true);
  expect(verdict.graspSeen).toBe(true);
  expect(verdict.liftSeen).toBe(true);
  expect(verdict.carrySeen).toBe(true);
  expect(verdict.supportWhileHeldSeen).toBe(true);
  expect(verdict.releaseSeen).toBe(true);
  expect(verdict.settled).toBe(true);
  expect(verdict.homeReturned).toBe(true);
  expect(verdict.restrictedViolation).toBe(false);
  expect(verdict.payloadLost).toBe(false);
  expect(verdict.causalOrder).toBe(true);
  expect(verdict.basePathLengthM).toBeGreaterThan(0.2);
  expect(verdict.restXYErrorM).toBeLessThan(0.02);
  expect(verdict.success).toBe(true);
  expect(verdict.evidence).toContain('No weld, parenting, attachment flag, snap, teleport, upright lock, timer, command echo, or program-reported success');
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-physical-task-success', 'true');
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-physical-task-home', 'true');

  // 11. WebMCP reaches the same session, and it reports a request separately from achieved state.
  await expect(page.locator('#agentAccessControl')).toHaveAttribute('data-available', 'true');
  await page.locator('#agentAccessToggle').click();
  await expect(page.locator('#agentAccessControl')).toHaveAttribute('data-access', 'assist');
  await expect(page.locator('#agentAccessControl')).toHaveAttribute('data-tools', 'enabled');
  // The physical tool is the one that actually reaches the agent surface, exactly once.
  const activeLekiwiTools = () => page.evaluate(() => window.__webMcpRegistrations
    .filter(({ signal }) => !signal?.aborted)
    .map(({ tool }) => tool.name)
    .filter((name) => name.includes('lekiwi')));
  await expect.poll(activeLekiwiTools).toEqual(['control_lekiwi_simulation']);
  const registeredSchema = await page.evaluate(() => {
    const entry = window.__webMcpRegistrations
      .findLast(({ tool, signal }) => tool.name === 'control_lekiwi_simulation' && !signal?.aborted);
    const branches = entry?.tool?.inputSchema?.oneOf || [];
    return {
      commands: branches.map((branch) => branch.properties?.command?.const),
      schemaVersions: [...new Set(branches.map((branch) => branch.properties?.schema_version?.const))],
    };
  });
  expect(registeredSchema.schemaVersions).toEqual(['robobuddy.lekiwi.physical.v1']);
  expect(registeredSchema.commands).toEqual(['set_chassis_velocity', 'set_arm_targets', 'stop', 'reset']);
  const webmcp = await page.evaluate(async () => {
    const { app, agentFacade } = window.__robobuddyCi;
    const { getLeKiwiPhysicalControlDefinition, executeLeKiwiPhysicalControl } = await import('/src/webmcp/lekiwi-physical-control.js');
    const { getProfileControlDefinition } = await import('/src/webmcp/robot-controls.js');
    const definition = getLeKiwiPhysicalControlDefinition(agentFacade);
    const legacy = getProfileControlDefinition(agentFacade);
    const before = app.sim.backend.getPhysicalAuthorityToken();
    const result = await executeLeKiwiPhysicalControl(agentFacade, {
      schema_version: 'robobuddy.lekiwi.physical.v1',
      command: 'set_chassis_velocity',
      chassis_velocity: { 'x.vel': 0.1 },
      advance_seconds: 0.5,
      max_steps: 4000,
    }, null, agentFacade.registrationEpoch);
    const after = app.sim.backend.getPhysicalAuthorityToken();
    let rejected = null;
    try {
      await executeLeKiwiPhysicalControl(agentFacade, {
        schema_version: 'robobuddy.lekiwi.physical.v1',
        command: 'set_chassis_velocity',
        chassis_velocity: { 'x.vel': 99 },
      }, null, agentFacade.registrationEpoch);
    } catch (error) { rejected = { code: error?.code, message: String(error?.message || error) }; }
    return { definitionName: definition?.name || null, legacyToolPresent: Boolean(legacy), result, before, after, rejected };
  });
  await attach(testInfo, 'lekiwi-webmcp.json', webmcp);
  expect(webmcp.definitionName).toBe('control_lekiwi_simulation');
  // The legacy source-plant LeKiwi tool must not shadow the physical one.
  expect(webmcp.legacyToolPresent).toBe(false);
  expect(webmcp.result.ok).toBe(true);
  expect(webmcp.result.hardwareValidated).toBe(false);
  expect(webmcp.result.requestedChassisVelocity['x.vel']).toBeCloseTo(0.1, 9);
  expect(Object.keys(webmcp.result.acceptedWheelTargetsRadS)).toHaveLength(3);
  expect(webmcp.result.observedState.basePose).toBeTruthy();
  expect(webmcp.result.note).toContain('not a measurement');
  expect(webmcp.result.physicalAuthority.sessionId).toBe(webmcp.before.sessionId);
  expect(webmcp.after.sessionId).toBe(webmcp.before.sessionId);
  expect(webmcp.rejected?.code).toBe('INVALID_ARGUMENT');

  // 12. Reset advances the session epoch and invalidates a bridge bound to the previous epoch.
  const staleness = await page.evaluate(async () => {
    const app = window.__robobuddyCi.app;
    const { LivePythonBridge } = await import('/src/runtime/live-python-bridge.js');
    const session = app.sim.backend.getPhysicalSession();
    const bridge = new LivePythonBridge(session);
    const connected = await bridge.connect('lekiwi_v1_mobile_manipulator');
    const beforeEpoch = session.epoch;
    await app.sim.backend.reset();
    const afterEpoch = session.epoch;
    let stale = null;
    try { await bridge.sendAction({ 'x.vel': 0.05 }, { maxSteps: 100 }); }
    catch (error) { stale = { code: error?.code, message: String(error?.message || error) }; }
    bridge.dispose();
    return { connectedRobotId: connected.robotId, beforeEpoch, afterEpoch, stale };
  });
  await attach(testInfo, 'lekiwi-staleness.json', staleness);
  expect(staleness.connectedRobotId).toBe('lekiwi_v1_mobile_manipulator');
  expect(staleness.afterEpoch).toBeGreaterThan(staleness.beforeEpoch);
  expect(staleness.stale?.code).toBe('STALE_LIVE_SESSION');

  // 14. No page errors anywhere in the chain.
  expect(pageErrors).toEqual([]);
});
