import { expect, test } from '@playwright/test';
import { existsSync, readFileSync } from 'node:fs';
import { writeFile } from 'node:fs/promises';

// Browser/native conformance for the LeKiwi Phase 5B physical gates.
//
// This spec drives the browser MuJoCo worker directly through PhysicsSession, with no Three.js
// presentation involved, so it isolates the physics from the renderer. When native reference
// reports are available (LEKIWI_NATIVE_REPORT_DIR, produced by native/lekiwi_reference.py) the
// same quantities are compared across the two backends within declared tolerances.
//
// Agreement here is implementation evidence only: both backends run the same pinned model and
// the same approximations, so it is not hardware validation.

const NATIVE_DIR = process.env.LEKIWI_NATIVE_REPORT_DIR || '';
const nativeReport = (trial) => {
  if (!NATIVE_DIR) return null;
  const path = `${NATIVE_DIR}/lekiwi-${trial}.json`;
  return existsSync(path) ? JSON.parse(readFileSync(path, 'utf8')) : null;
};

const TOLERANCE = Object.freeze({
  displacementM: 0.020,
  yawRad: 0.10,
  wheelRadS: 0.15,
});

test('LeKiwi Phase 5B browser MuJoCo reproduces the native physical base gates', async ({ page }, testInfo) => {
  test.setTimeout(300_000);
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await page.goto('/physics-slice.html?ci=lekiwi-conformance', { waitUntil: 'domcontentloaded' });

  const results = await page.evaluate(async () => {
    const { PhysicsSession } = await import('/src/physics/session.js');
    const { BrowserMuJoCoBackend } = await import('/src/physics/browser-mujoco-backend.js');
    const { LEKIWI_BASE_SCENE, LEKIWI_BASE_STAND_SCENE, LEKIWI_BASE_LOWTRACTION_SCENE } = await import('/src/physics/lekiwi-scene.js');
    const { bodyToWheelRadS, WHEEL_ORDER } = await import('/src/physics/lekiwi-kinematics.js');
    const { basePoseFromObservation, bodyFrameDisplacement } = await import('/src/physics/lekiwi-task-evaluator.js');

    const workerUrl = new URL('/src/physics/lekiwi-mujoco-worker.js', window.location.origin);
    const openSession = async (scene, id) => {
      const session = new PhysicsSession(new BrowserMuJoCoBackend({ workerUrl }), { sessionId: id });
      await session.loadScene(structuredClone(scene));
      return session;
    };
    const advance = async (session, seconds) => session.advanceSteps(Math.round(seconds / 0.002));
    const wheelState = (observation) => Object.fromEntries(WHEEL_ORDER.map((wheel) => [wheel, {
      velocityRadS: observation.joints[wheel].velocityRadS,
      targetVelocityRadS: observation.joints[wheel].targetVelocityRadS,
      effortNm: observation.joints[wheel].effortNm,
    }]));

    const run = async (scene, id, command, seconds, { settle = 1.0 } = {}) => {
      const session = await openSession(scene, id);
      try {
        let observation = await advance(session, settle);
        const start = basePoseFromObservation(observation);
        const { targetsRadS, saturated } = bodyToWheelRadS(command);
        await session.sendCommand({ type: 'set_wheel_velocity_targets', targetsRadS }, { maxSteps: 40000 });
        observation = await advance(session, seconds);
        const end = basePoseFromObservation(observation);
        const displacement = bodyFrameDisplacement(start, end);
        // A commanded stop must bring the base to rest through wheel/ground contact.
        await session.sendCommand({ type: 'set_wheel_velocity_targets', targetsRadS: Object.fromEntries(WHEEL_ORDER.map((w) => [w, 0])) }, { maxSteps: 20000 });
        const stopped = basePoseFromObservation(await advance(session, 1.5));
        return {
          command, saturated,
          wheelTargetsRadS: targetsRadS,
          wheels: wheelState(observation),
          forwardM: displacement.forwardM,
          lateralM: displacement.leftM,
          yawRad: displacement.yawRad,
          endSpeedMS: end.speedMS,
          stoppedSpeedMS: stopped.speedMS,
          stopTravelM: Math.hypot(stopped.xM - end.xM, stopped.yM - end.yM),
          contactCount: observation.contactCount,
          baseHeightM: end.zM,
          model: observation.model,
          engine: observation.engine,
        };
      } finally { session.dispose(); }
    };

    const out = { engine: null, model: null, cases: {} };
    out.cases.forward = await run(LEKIWI_BASE_SCENE, 'conf-forward', { x: 0.15 }, 3.0);
    out.cases.reverse = await run(LEKIWI_BASE_SCENE, 'conf-reverse', { x: -0.15 }, 3.0);
    out.cases.lateralLeft = await run(LEKIWI_BASE_SCENE, 'conf-lateral-left', { y: 0.15 }, 3.0);
    out.cases.lateralRight = await run(LEKIWI_BASE_SCENE, 'conf-lateral-right', { y: -0.15 }, 3.0);
    out.cases.yawPositive = await run(LEKIWI_BASE_SCENE, 'conf-yaw-pos', { thetaRadS: 0.8 }, 3.0);
    out.cases.yawNegative = await run(LEKIWI_BASE_SCENE, 'conf-yaw-neg', { thetaRadS: -0.8 }, 3.0);
    out.cases.mixed = await run(LEKIWI_BASE_SCENE, 'conf-mixed', { x: 0.12, y: 0.08, thetaRadS: 0.4 }, 3.0);
    out.cases.zeroCommand = await run(LEKIWI_BASE_SCENE, 'conf-zero', {}, 3.0);
    // Adverse conditions, both as declared visible/pinned assets rather than runtime mutation.
    out.cases.lifted = await run(LEKIWI_BASE_STAND_SCENE, 'conf-lifted', { x: 0.15 }, 3.0);
    out.cases.lowTractionShort = await run(LEKIWI_BASE_LOWTRACTION_SCENE, 'conf-lowtraction-short', { x: 0.26 }, 0.5);
    out.cases.nominalShort = await run(LEKIWI_BASE_SCENE, 'conf-nominal-short', { x: 0.26 }, 0.5);
    out.engine = out.cases.forward.engine;
    out.model = out.cases.forward.model;
    return out;
  });

  const path = testInfo.outputPath('lekiwi-browser-conformance.json');
  await writeFile(path, JSON.stringify({ ...results, pageErrors }, null, 2));
  await testInfo.attach('lekiwi-browser-conformance.json', { path, contentType: 'application/json' });

  expect(pageErrors).toEqual([]);
  expect(results.engine.version).toBe('3.11.0');
  expect(results.engine.timestepSeconds).toBeCloseTo(0.002, 12);
  expect(results.model.id).toBe('robobuddy-lekiwi-base-v1');

  const c = results.cases;
  // Forward, reverse, lateral and yaw all arise from bounded wheel actuation and contact.
  expect(c.forward.forwardM).toBeGreaterThan(0.38);
  expect(Math.abs(c.forward.lateralM)).toBeLessThan(0.02);
  expect(c.reverse.forwardM).toBeLessThan(-0.38);
  expect(c.lateralLeft.lateralM).toBeGreaterThan(0.38);
  expect(Math.abs(c.lateralLeft.forwardM)).toBeLessThan(0.02);
  expect(c.lateralRight.lateralM).toBeLessThan(-0.38);
  expect(c.yawPositive.yawRad).toBeGreaterThan(2.0);
  expect(c.yawNegative.yawRad).toBeLessThan(-2.0);
  expect(Math.hypot(c.yawPositive.forwardM, c.yawPositive.lateralM)).toBeLessThan(0.02);
  expect(c.mixed.forwardM).toBeGreaterThan(0.05);
  expect(c.mixed.lateralM).toBeGreaterThan(0.20);
  expect(c.mixed.yawRad).toBeGreaterThan(0.8);
  // A zero chassis request produces no motion at all.
  expect(Math.hypot(c.zeroCommand.forwardM, c.zeroCommand.lateralM)).toBeLessThan(0.005);
  expect(Math.abs(c.zeroCommand.yawRad)).toBeLessThan(0.02);
  // Commanded stop halts the base through wheel/ground contact, over a finite distance.
  for (const label of ['forward', 'lateralLeft', 'yawPositive']) {
    expect(c[label].stoppedSpeedMS).toBeLessThan(0.02);
    expect(c[label].stopTravelM).toBeLessThan(0.05);
  }
  // Lifted base: the wheels spin, nothing touches the floor, and the base does not propel itself.
  expect(Math.abs(c.lifted.wheels.base_left_wheel.velocityRadS)).toBeGreaterThan(1.0);
  expect(Math.abs(c.lifted.wheels.base_right_wheel.velocityRadS)).toBeGreaterThan(1.0);
  expect(Math.hypot(c.lifted.forwardM, c.lifted.lateralM)).toBeLessThan(0.002);
  expect(Math.abs(c.lifted.yawRad)).toBeLessThan(0.01);
  expect(c.lifted.baseHeightM).toBeGreaterThan(0.015);
  // Reduced traction materially degrades propulsion for the same bounded wheel command.
  expect(Math.abs(c.lowTractionShort.wheels.base_left_wheel.velocityRadS)).toBeGreaterThan(3.0);
  expect(c.lowTractionShort.forwardM).toBeLessThan(c.nominalShort.forwardM * 0.5);
  // Bounded effort: every wheel stays inside the declared torque limit.
  for (const label of Object.keys(c)) {
    for (const wheel of Object.values(c[label].wheels)) {
      expect(Math.abs(wheel.effortNm)).toBeLessThanOrEqual(2.94 + 1e-6);
      expect(Math.abs(wheel.targetVelocityRadS)).toBeLessThanOrEqual(4.60061 + 1e-9);
    }
  }

  // Browser/native conformance, when the native reports are available.
  const nativeBase = nativeReport('base-motion');
  const nativeLifted = nativeReport('lifted-base');
  const nativeTraction = nativeReport('traction-reduced');
  const comparison = { available: Boolean(nativeBase), rows: [] };
  if (nativeBase) {
    const pairs = [
      ['forward', nativeBase.metrics.forward], ['reverse', nativeBase.metrics.reverse],
      ['lateralLeft', nativeBase.metrics.lateralLeft], ['lateralRight', nativeBase.metrics.lateralRight],
      ['yawPositive', nativeBase.metrics.yawPositive], ['yawNegative', nativeBase.metrics.yawNegative],
      ['mixed', nativeBase.metrics.mixed], ['zeroCommand', nativeBase.metrics.zeroCommand],
    ];
    for (const [label, native] of pairs) {
      const browser = c[label];
      const row = {
        label,
        forwardDeltaM: browser.forwardM - native.forwardM,
        lateralDeltaM: browser.lateralM - native.lateralM,
        yawDeltaRad: browser.yawRad - native.yawRad,
        wheelDeltaRadS: Object.fromEntries(Object.keys(browser.wheels).map((wheel, index) => [wheel, browser.wheels[wheel].velocityRadS - native.actualWheelRadS[index]])),
      };
      comparison.rows.push(row);
      expect(Math.abs(row.forwardDeltaM)).toBeLessThan(TOLERANCE.displacementM);
      expect(Math.abs(row.lateralDeltaM)).toBeLessThan(TOLERANCE.displacementM);
      expect(Math.abs(row.yawDeltaRad)).toBeLessThan(TOLERANCE.yawRad);
      for (const delta of Object.values(row.wheelDeltaRadS)) expect(Math.abs(delta)).toBeLessThan(TOLERANCE.wheelRadS);
    }
  }
  if (nativeLifted) {
    comparison.liftedNativeForwardM = nativeLifted.metrics.lifted.forwardM;
    expect(Math.abs(c.lifted.forwardM - nativeLifted.metrics.lifted.forwardM)).toBeLessThan(0.002);
  }
  if (nativeTraction) {
    comparison.tractionNativeForwardM = nativeTraction.metrics['reduced_0.5s'].forwardM;
    expect(Math.abs(c.lowTractionShort.forwardM - nativeTraction.metrics['reduced_0.5s'].forwardM)).toBeLessThan(TOLERANCE.displacementM);
  }
  const comparePath = testInfo.outputPath('lekiwi-browser-native-comparison.json');
  await writeFile(comparePath, JSON.stringify(comparison, null, 2));
  await testInfo.attach('lekiwi-browser-native-comparison.json', { path: comparePath, contentType: 'application/json' });
});

test('LeKiwi Phase 5B browser MuJoCo reproduces the native manipulation, lateral-carry and courier gates', async ({ page }, testInfo) => {
  test.setTimeout(420_000);
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await page.goto('/physics-slice.html?ci=lekiwi-courier-conformance', { waitUntil: 'domcontentloaded' });

  const results = await page.evaluate(async () => {
    const { PhysicsSession } = await import('/src/physics/session.js');
    const { BrowserMuJoCoBackend } = await import('/src/physics/browser-mujoco-backend.js');
    const { LEKIWI_COURIER_CONTROLLER, LEKIWI_COURIER_SCENE, LEKIWI_WORKCELL, chassisVelocityCommand, waypointReached, wheelTargetsForChassis } = await import('/src/physics/lekiwi-scene.js');
    const { WHEEL_ORDER } = await import('/src/physics/lekiwi-kinematics.js');
    const { LeKiwiCourierEvaluator, basePoseFromObservation } = await import('/src/physics/lekiwi-task-evaluator.js');

    const workerUrl = new URL('/src/physics/lekiwi-mujoco-worker.js', window.location.origin);
    const OBSERVATION_BATCH_STEPS = 5;
    const STOP = Object.fromEntries(WHEEL_ORDER.map((wheel) => [wheel, 0]));

    const makeSession = async (id) => {
      const evaluator = new LeKiwiCourierEvaluator();
      const session = new PhysicsSession(new BrowserMuJoCoBackend({ workerUrl }), { sessionId: id, observationBatchSteps: OBSERVATION_BATCH_STEPS });
      let latest = null;
      session.subscribe(({ observation }) => { latest = observation; evaluator.observe(observation); });
      const load = await session.loadScene(structuredClone(LEKIWI_COURIER_SCENE));
      latest = load.observation;
      return {
        session, evaluator,
        observation: () => latest,
        pose: () => basePoseFromObservation(latest),
        advance: (seconds) => session.advanceSteps(Math.round(seconds / 0.002)),
        arm: (targets) => session.sendCommand({ type: 'set_joint_targets', targetsRad: targets }, { maxSteps: 40000 }),
        wheels: (targetsRadS) => session.sendCommand({ type: 'set_wheel_velocity_targets', targetsRadS }, { maxSteps: 40000 }),
      };
    };

    const runStages = async (ctx, stages) => {
      const marks = [];
      for (const stage of stages) {
        const targets = { ...stage.armTargetsRad, arm_gripper: stage.gripperRad };
        if (stage.kind === 'drive') {
          for (const waypoint of stage.waypoints) {
            let elapsed = 0;
            while (elapsed < stage.timeoutSeconds) {
              const pose = ctx.pose();
              if (waypointReached(pose, waypoint)) break;
              const command = chassisVelocityCommand(pose, waypoint);
              await ctx.wheels(wheelTargetsForChassis(command).targetsRadS);
              await ctx.arm(targets);
              await ctx.advance(LEKIWI_COURIER_CONTROLLER.controllerPeriodSeconds);
              elapsed += LEKIWI_COURIER_CONTROLLER.controllerPeriodSeconds;
            }
            await ctx.wheels(STOP);
            await ctx.advance(0.2);
          }
        } else {
          await ctx.wheels(STOP);
          await ctx.arm(targets);
          await ctx.advance(Math.round(stage.durationSeconds / 0.002) * 0.002);
        }
        const snapshot = ctx.evaluator.snapshot();
        marks.push({
          stage: stage.name,
          t: ctx.observation().simulationTimeSeconds,
          basePose: ctx.pose(),
          beakerM: ctx.observation().bodies.empty_beaker.positionM,
          graspSeen: snapshot.graspSeen, liftSeen: snapshot.liftSeen, carrySeen: snapshot.carrySeen,
          supportWhileHeldSeen: snapshot.supportWhileHeldSeen, releaseSeen: snapshot.releaseSeen,
          settled: snapshot.settled, homeReturned: snapshot.homeReturned,
        });
      }
      return marks;
    };

    const out = {};
    // Full physical courier: drive, rim pinch, lift, carry, supported release, settle, drive home.
    {
      const ctx = await makeSession('conf-courier');
      try {
        const marks = await runStages(ctx, LEKIWI_COURIER_CONTROLLER.stages);
        out.courier = { marks, evaluation: ctx.evaluator.snapshot() };
      } finally { ctx.session.dispose(); }
    }
    // Manipulation gate only, with the base commanded to the service stop first, so payload
    // handling is tested separately from route planning.
    {
      const ctx = await makeSession('conf-manipulation');
      try {
        const driveStage = LEKIWI_COURIER_CONTROLLER.stages.find((stage) => stage.name === 'drive_to_service');
        const manipulation = LEKIWI_COURIER_CONTROLLER.stages.filter((stage) => stage.kind !== 'drive'
          && !['settle', 'stow_arm', 'stop_home'].includes(stage.name));
        await runStages(ctx, [driveStage]);
        await runStages(ctx, manipulation);
        out.manipulation = { evaluation: ctx.evaluator.snapshot() };
      } finally { ctx.session.dispose(); }
    }
    // Lateral carry: a holonomic strafe in both directions while the payload stays gripped.
    {
      const ctx = await makeSession('conf-lateral-carry');
      try {
        const upTo = ['stow_arm', 'drive_to_service', 'stop_at_service', 'approach_beaker', 'reach_rim', 'close_gripper', 'lift_beaker'];
        await runStages(ctx, LEKIWI_COURIER_CONTROLLER.stages.filter((stage) => upTo.includes(stage.name)));
        const held = ctx.evaluator.snapshot();
        const start = ctx.pose();
        const beakerStart = [...ctx.observation().bodies.empty_beaker.positionM];
        const segments = [];
        for (const direction of [1, -1]) {
          await ctx.wheels(wheelTargetsForChassis({ x: 0, y: direction * 0.12, thetaRadS: 0 }).targetsRadS);
          await ctx.advance(1.6);
          await ctx.wheels(STOP);
          await ctx.advance(0.6);
          const pose = ctx.pose();
          const dx = pose.xM - start.xM;
          const dy = pose.yM - start.yM;
          const cos = Math.cos(start.yawRad);
          const sin = Math.sin(start.yawRad);
          const snapshot = ctx.evaluator.snapshot();
          segments.push({
            direction,
            forwardM: dx * cos + dy * sin,
            lateralM: -dx * sin + dy * cos,
            stillGrasped: snapshot.currentGrasp,
            supportContact: snapshot.currentSupportContact,
            beakerM: [...ctx.observation().bodies.empty_beaker.positionM],
          });
        }
        out.lateralCarry = {
          heldBeforeLateral: held.liftSeen,
          segments,
          beakerTravelWithBaseM: Math.hypot(
            ctx.observation().bodies.empty_beaker.positionM[0] - beakerStart[0],
            ctx.observation().bodies.empty_beaker.positionM[1] - beakerStart[1],
          ),
          evaluation: ctx.evaluator.snapshot(),
        };
      } finally { ctx.session.dispose(); }
    }
    return out;
  });

  const path = testInfo.outputPath('lekiwi-browser-courier.json');
  await writeFile(path, JSON.stringify({ ...results, pageErrors }, null, 2));
  await testInfo.attach('lekiwi-browser-courier.json', { path, contentType: 'application/json' });
  expect(pageErrors).toEqual([]);

  const verdict = results.courier.evaluation;
  expect(verdict.serviceStopReached).toBe(true);
  expect(verdict.graspSeen).toBe(true);
  expect(verdict.liftSeen).toBe(true);
  expect(verdict.carrySeen).toBe(true);
  expect(verdict.supportWhileHeldSeen).toBe(true);
  expect(verdict.releaseSeen).toBe(true);
  expect(verdict.settled).toBe(true);
  expect(verdict.homeReturned).toBe(true);
  expect(verdict.causalOrder).toBe(true);
  expect(verdict.restrictedViolation).toBe(false);
  expect(verdict.payloadLost).toBe(false);
  expect(verdict.restXYErrorM).toBeLessThan(0.02);
  expect(verdict.basePathLengthM).toBeGreaterThan(0.2);
  expect(verdict.success).toBe(true);

  const manipulation = results.manipulation.evaluation;
  expect(manipulation.graspSeen).toBe(true);
  expect(manipulation.liftSeen).toBe(true);
  expect(manipulation.carrySeen).toBe(true);
  expect(manipulation.supportWhileHeldSeen).toBe(true);
  expect(manipulation.releaseSeen).toBe(true);
  expect(manipulation.settled).toBe(true);

  // Lateral holonomic carry: the base strafes both ways and the payload stays physically held.
  const lateral = results.lateralCarry;
  expect(lateral.heldBeforeLateral).toBe(true);
  expect(Math.max(...lateral.segments.map((s) => Math.abs(s.lateralM)))).toBeGreaterThan(0.10);
  expect(Math.max(...lateral.segments.map((s) => Math.abs(s.forwardM)))).toBeLessThan(0.02);
  for (const segment of lateral.segments) {
    expect(segment.stillGrasped).toBe(true);
    expect(segment.supportContact).toBe(false);
  }
  expect(lateral.evaluation.payloadLost).toBe(false);

  const nativeCourier = nativeReport('courier');
  const nativeManipulation = nativeReport('manipulation');
  const nativeLateral = nativeReport('lateral-carry');
  const comparison = { available: Boolean(nativeCourier) };
  if (nativeCourier) {
    comparison.courier = {
      browserFinalBeakerM: verdict.finalBeakerM,
      nativeFinalBeakerM: nativeCourier.metrics.finalBeakerM,
      deltaM: Math.hypot(...verdict.finalBeakerM.map((value, index) => value - nativeCourier.metrics.finalBeakerM[index])),
      browserBasePathM: verdict.basePathLengthM,
      nativeBasePathM: nativeCourier.metrics.basePathLengthM,
    };
    expect(nativeCourier.metrics.success).toBe(true);
    expect(comparison.courier.deltaM).toBeLessThan(0.02);
    expect(Math.abs(verdict.basePathLengthM - nativeCourier.metrics.basePathLengthM)).toBeLessThan(0.05);
  }
  if (nativeManipulation) expect(nativeManipulation.metrics.manipulationSuccess).toBe(true);
  if (nativeLateral) expect(nativeLateral.metrics.lateralCarrySuccess).toBe(true);
  const comparePath = testInfo.outputPath('lekiwi-browser-native-courier-comparison.json');
  await writeFile(comparePath, JSON.stringify(comparison, null, 2));
  await testInfo.attach('lekiwi-browser-native-courier-comparison.json', { path: comparePath, contentType: 'application/json' });
});
