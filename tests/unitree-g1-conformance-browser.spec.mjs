import { expect, test } from '@playwright/test';
import { existsSync, readFileSync } from 'node:fs';
import { writeFile } from 'node:fs/promises';

// Browser/native conformance and P7 physical gates for the Unitree G1 Phase 5D workspace.
//
// The spec drives the browser MuJoCo worker directly through PhysicsSession, with no Three.js
// presentation involved, so it isolates the physics from the renderer. When native reference
// reports are available (UNITREE_G1_NATIVE_REPORT_DIR, produced by native/unitree_g1_reference.py)
// the same physical quantities are compared across the two backends.
//
// Agreement here is implementation evidence only: both backends run the same pinned model, the
// same controller and the same approximations, so it is not hardware validation.

const NATIVE_DIR = process.env.UNITREE_G1_NATIVE_REPORT_DIR || '';
const NATIVE_TRIALS = Object.freeze(['stand-nominal', 'stand-source-fixstand', 'stand-motors-disabled', 'blocked-joint', 'free-fall', 'external-object', 'self-contact']);
let nativeComparisons = 0;
// When a native report directory is configured, every comparison it promises must actually run.
// A missing file is a failure, not a silently skipped check: CI would otherwise report a green
// cross-backend conformance run that compared nothing.
const nativeReport = (trial) => {
  if (!NATIVE_DIR) return null;
  if (!NATIVE_TRIALS.includes(trial)) throw new Error(`Undeclared native conformance trial: ${trial}`);
  const path = `${NATIVE_DIR}/unitree-g1-${trial}.json`;
  if (!existsSync(path)) throw new Error(`UNITREE_G1_NATIVE_REPORT_DIR is set but ${path} is missing; run native/unitree_g1_reference.py --trial ${trial}`);
  nativeComparisons += 1;
  return JSON.parse(readFileSync(path, 'utf8'));
};

const TOLERANCE = Object.freeze({
  jointRad: 0.02,
  pelvisHeightM: 0.010,
  tiltRad: 0.05,
  driftM: 0.030,
  objectDisplacementM: 0.040,
});

test('Unitree G1 Phase 5D browser MuJoCo reproduces the native physical gates', async ({ page }, testInfo) => {
  test.setTimeout(900_000);
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await page.goto('/physics-slice.html?ci=unitree-g1-conformance', { waitUntil: 'domcontentloaded' });

  const results = await page.evaluate(async () => {
    const { PhysicsSession } = await import('/src/physics/session.js');
    const { BrowserMuJoCoBackend } = await import('/src/physics/browser-mujoco-backend.js');
    const { UNITREE_G1_SCENES, UNITREE_G1_STAND_GATE, UnitreeG1StandEvaluator } = await import('/src/physics/unitree-g1-scene.js');
    const { G1_CONTROLLERS, G1_PHYSICS_TIMESTEP_SECONDS, G1_STAND_CONTROLLER_PROFILES, G1_STAND_POSE_RAD } = await import('/src/physics/unitree-g1-controller.js');
    const { G1_JOINT_ORDER } = await import('/src/physics/unitree-g1-source-audit.js');

    const workerUrl = new URL('/src/physics/unitree-g1-mujoco-worker.js', window.location.origin);
    let sequence = 0;
    const open = async (scene, { batch = null } = {}) => {
      const session = new PhysicsSession(
        new BrowserMuJoCoBackend({ workerUrl, setupOperations: ['set_actuation'] }),
        { sessionId: `g1-conformance-${++sequence}`, observationBatchSteps: batch },
      );
      await session.loadScene(structuredClone(scene));
      return session;
    };
    const advance = (session, seconds) => session.advanceSteps(Math.round(seconds / G1_PHYSICS_TIMESTEP_SECONDS));
    const classes = (observation) => Object.fromEntries(Object.entries(observation.contactClasses || {}).map(([key, value]) => [key, value.length]));
    const geomPairs = (observation, key) => (observation.contactClasses?.[key] || []).map((entry) => [...entry.geoms].sort().join(' | '));

    const out = {};

    // --- model audit ------------------------------------------------------------------------
    out.modelAudit = {};
    for (const [key, scene] of Object.entries(UNITREE_G1_SCENES)) {
      const session = await open(scene);
      try {
        const observation = await session.getObservation();
        out.modelAudit[key] = {
          sceneId: scene.id,
          modelSha256: observation.model.sha256,
          rootMode: observation.root.mode,
          rootFree: observation.root.free,
          gravityZ: observation.engine.gravity[2],
          jointCount: Object.keys(observation.joints).length,
          jointOrder: G1_JOINT_ORDER.every((jointId) => jointId in observation.joints),
          controllerId: observation.controller.id,
          timestepSeconds: observation.engine.timestepSeconds,
          effortLimits: Object.fromEntries(G1_JOINT_ORDER.slice(0, 6).map((jointId) => [jointId, observation.joints[jointId].effortLimitNm])),
          velocityLimits: Object.fromEntries(G1_JOINT_ORDER.slice(0, 6).map((jointId) => [jointId, observation.joints[jointId].velocityLimitRadS])),
        };
      } finally { session.dispose(); }
    }

    // --- bounded command: requested, accepted and measured are three things -------------------
    {
      const session = await open(UNITREE_G1_SCENES.mounted);
      try {
        await session.sendCommand({ type: 'set_joint_targets', targetsRad: { left_knee_joint: 1.1, left_ankle_pitch_joint: -0.5, waist_yaw_joint: 0.6 } }, { maxSteps: 4000 });
        const observation = await advance(session, 2.5);
        out.mountedJointResponse = Object.fromEntries(['left_knee_joint', 'left_ankle_pitch_joint', 'waist_yaw_joint'].map((jointId) => [jointId, {
          requestedTargetRad: observation.joints[jointId].requestedTargetRad,
          acceptedTargetRad: observation.joints[jointId].acceptedTargetRad,
          positionRad: observation.joints[jointId].positionRad,
          effortNm: observation.joints[jointId].effortNm,
          effortLimitNm: observation.joints[jointId].effortLimitNm,
        }]));
        // An out-of-range request must be clamped to the source range and must not be echoed back.
        await session.sendCommand({ type: 'set_joint_targets', targetsRad: { left_knee_joint: 6 } }, { maxSteps: 4000 });
        const clamped = await advance(session, 2.0);
        out.commandBounds = {
          requestedTargetRad: clamped.joints.left_knee_joint.requestedTargetRad,
          acceptedTargetRad: clamped.joints.left_knee_joint.acceptedTargetRad,
          positionRad: clamped.joints.left_knee_joint.positionRad,
          jointRangeRad: clamped.joints.left_knee_joint.jointRangeRad,
          commandBounded: clamped.joints.left_knee_joint.commandBounded,
          worstEffortFraction: Math.max(...G1_JOINT_ORDER.map((jointId) => Math.abs(clamped.joints[jointId].effortNm) / clamped.joints[jointId].effortLimitNm)),
        };
      } finally { session.dispose(); }
    }

    // --- blocked joint ------------------------------------------------------------------------
    {
      const run = async (scene) => {
        const session = await open(scene);
        try {
          await session.sendCommand({ type: 'set_joint_targets', targetsRad: { left_hip_roll_joint: 1.2 } }, { maxSteps: 4000 });
          const observation = await advance(session, 3.0);
          return {
            requestedTargetRad: observation.joints.left_hip_roll_joint.requestedTargetRad,
            acceptedTargetRad: observation.joints.left_hip_roll_joint.acceptedTargetRad,
            positionRad: observation.joints.left_hip_roll_joint.positionRad,
            effortNm: observation.joints.left_hip_roll_joint.effortNm,
            effortLimitNm: observation.joints.left_hip_roll_joint.effortLimitNm,
            fixtureContacts: geomPairs(observation, 'robotFixture'),
          };
        } finally { session.dispose(); }
      };
      out.blockedJoint = { unobstructed: await run(UNITREE_G1_SCENES.mounted), blocked: await run(UNITREE_G1_SCENES.mountedBlocked) };
    }

    // --- self-contact ---------------------------------------------------------------------------
    {
      const session = await open(UNITREE_G1_SCENES.mounted);
      try {
        await session.sendCommand({ type: 'set_joint_targets', targetsRad: { left_hip_roll_joint: -0.5236, right_hip_roll_joint: 0.5236 } }, { maxSteps: 4000 });
        const observation = await advance(session, 3.0);
        out.selfContact = {
          pairs: [...new Set(geomPairs(observation, 'robotSelf'))],
          bodies: [...new Set((observation.contactClasses.robotSelf || []).map((entry) => [...entry.bodies].sort().join(' | ')))],
          leftMeasuredRad: observation.joints.left_hip_roll_joint.positionRad,
          leftAcceptedRad: observation.joints.left_hip_roll_joint.acceptedTargetRad,
          rightMeasuredRad: observation.joints.right_hip_roll_joint.positionRad,
        };
      } finally { session.dispose(); }
    }

    // --- gravity and fall -------------------------------------------------------------------------
    {
      const session = await open(UNITREE_G1_SCENES.freebaseDrop);
      try {
        const initial = await session.getObservation();
        const final = await advance(session, 3.0);
        out.freeFall = {
          initialHeightM: initial.root.positionM[2],
          finalHeightM: final.root.positionM[2],
          heightChangeM: final.root.positionM[2] - initial.root.positionM[2],
          initialUprightZ: initial.root.uprightZ,
          finalUprightZ: final.root.uprightZ,
          contacts: classes(final),
        };
      } finally { session.dispose(); }
    }

    // --- standing: nominal, the source controller, and the motors-disabled negative ---------------
    const stand = async (controllerId, { disableActuation = false } = {}) => {
      const session = await open(UNITREE_G1_SCENES.freebase, { batch: 10 });
      const evaluator = new UnitreeG1StandEvaluator();
      const unsubscribe = session.subscribe(({ observation }) => evaluator.observe(observation));
      try {
        if (disableActuation) await session.applySetup({ type: 'set_actuation', enabled: false });
        await session.sendCommand({ type: 'engage_stand', controllerId }, { maxSteps: 200000 });
        const total = UNITREE_G1_STAND_GATE.evaluationStartSeconds + UNITREE_G1_STAND_GATE.evaluationSeconds;
        let remaining = total;
        let observation = null;
        while (remaining > 1e-9) {
          const chunk = Math.min(2, remaining);
          observation = await advance(session, chunk);
          remaining -= chunk;
        }
        const snapshot = evaluator.snapshot();
        return {
          controllerId,
          standing: snapshot.standing,
          checks: snapshot.checks,
          measured: snapshot.measured,
          terminal: snapshot.terminal,
          actuationEnabled: observation.actuationEnabled,
          engagedControllerId: observation.controller.id,
          footContacts: { left: geomPairs(observation, 'leftFootFloor').length, right: geomPairs(observation, 'rightFootFloor').length },
          setupLog: observation.setupLog,
        };
      } finally { unsubscribe(); session.dispose(); }
    };
    out.standNominal = await stand(G1_CONTROLLERS.STAND);
    out.standSourceFixStand = await stand(G1_CONTROLLERS.SOURCE_FIXSTAND);
    out.standMotorsDisabled = await stand(G1_CONTROLLERS.STAND, { disableActuation: true });

    // --- external object ----------------------------------------------------------------------------
    {
      const session = await open(UNITREE_G1_SCENES.freebase, { batch: 10 });
      const seen = new Set();
      const unsubscribe = session.subscribe(({ observation }) => {
        for (const entry of observation.contactClasses?.robotExternalObject || []) seen.add([...entry.geoms].sort().join(' | '));
      });
      try {
        await session.sendCommand({ type: 'engage_stand', controllerId: G1_CONTROLLERS.STAND }, { maxSteps: 200000 });
        await advance(session, 2.0);
        const before = (await session.getObservation()).bodies.contact_probe_block.positionM;
        // The reach is held at the standing controller's own gains, which is exactly what the native
        // trial does, so the two backends run the same experiment. It is a large asymmetric
        // single-leg motion: the robot topples, its right knee strikes the declared free block, and
        // the block's response is contact physics rather than any placement or transform write.
        const profile = G1_STAND_CONTROLLER_PROFILES[G1_CONTROLLERS.STAND];
        const reachRad = [...G1_STAND_POSE_RAD];
        reachRad[G1_JOINT_ORDER.indexOf('right_hip_pitch_joint')] = -0.95;
        reachRad[G1_JOINT_ORDER.indexOf('right_knee_joint')] = 0.45;
        reachRad[G1_JOINT_ORDER.indexOf('right_ankle_pitch_joint')] = -0.25;
        const reach = Object.fromEntries(G1_JOINT_ORDER.map((jointId, index) => [jointId, {
          positionRad: reachRad[index], velocityRadS: 0, feedforwardTorqueNm: 0,
          kp: profile.kp[index], kd: profile.kd[index],
        }]));
        await session.sendCommand({ type: 'set_lowlevel_targets', commands: reach }, { maxSteps: 40000 });
        const final = await advance(session, 1.6);
        const after = final.bodies.contact_probe_block.positionM;
        out.externalObject = {
          before, after,
          displacementM: Math.hypot(after[0] - before[0], after[1] - before[1], after[2] - before[2]),
          contactPairs: [...seen],
          rootAfter: final.root.positionM,
          rootUprightZ: final.root.uprightZ,
        };
      } finally { unsubscribe(); session.dispose(); }
    }

    // --- display-rate independence ---------------------------------------------------------------
    // The same physical command sequence over the same simulated time, executed with different
    // observation cadences and different rendering activity, must land in the same physical state.
    {
      const trial = async ({ batch, spinFrames }) => {
        const session = await open(UNITREE_G1_SCENES.freebase, { batch });
        try {
          await session.sendCommand({ type: 'engage_stand', controllerId: G1_CONTROLLERS.STAND }, { maxSteps: 200000 });
          for (let index = 0; index < 3; index += 1) {
            await advance(session, 1.0);
            // Simulated renderer pressure between advances. Rendering must not advance physics.
            for (let frame = 0; frame < spinFrames; frame += 1) await new Promise((resolve) => requestAnimationFrame(resolve));
          }
          const observation = await session.getObservation();
          return {
            simulationTimeSeconds: observation.simulationTimeSeconds,
            pelvisHeightM: observation.root.positionM[2],
            tiltRad: observation.root.tiltRad,
            driftM: Math.hypot(observation.root.positionM[0], observation.root.positionM[1]),
            joints: Object.fromEntries(G1_JOINT_ORDER.map((jointId) => [jointId, observation.joints[jointId].positionRad])),
          };
        } finally { session.dispose(); }
      };
      const dense = await trial({ batch: 5, spinFrames: 0 });
      const sparse = await trial({ batch: 250, spinFrames: 12 });
      out.displayRate = {
        dense, sparse,
        simulationTimeDelta: Math.abs(dense.simulationTimeSeconds - sparse.simulationTimeSeconds),
        pelvisHeightDelta: Math.abs(dense.pelvisHeightM - sparse.pelvisHeightM),
        tiltDelta: Math.abs(dense.tiltRad - sparse.tiltRad),
        maxJointDelta: Math.max(...G1_JOINT_ORDER.map((jointId) => Math.abs(dense.joints[jointId] - sparse.joints[jointId]))),
      };
    }

    // --- lifecycle: a stale operation cannot touch the replacement run --------------------------
    {
      const session = await open(UNITREE_G1_SCENES.freebase);
      const errors = [];
      try {
        const stale = { sessionId: session.sessionId, epoch: session.epoch };
        await session.reset({ reason: 'lifecycle-test' });
        try { await session.backend.advanceSteps(10, stale); } catch (error) { errors.push(String(error.message)); }
        try { await session.backend.acceptCommand({ schemaVersion: '0.2.0-alpha.1', sessionId: stale.sessionId, epoch: stale.epoch, commandId: 'stale', sceneRevision: session.sceneRevision, robotId: session.robotId, command: { type: 'set_joint_targets', targetsRad: { left_knee_joint: 1 } }, maxSteps: 10 }); } catch (error) { errors.push(String(error.message)); }
        const cancelled = await session.cancelRun('lifecycle-test');
        let afterCancel = null;
        try { await session.getObservation(); } catch (error) { afterCancel = String(error.message); }
        out.lifecycle = { staleRejections: errors, cancelled: cancelled.cancelled, reloadRequired: cancelled.reloadRequired, afterCancel };
      } finally { session.dispose(); }
    }

    // --- the authority refuses what it does not declare -------------------------------------------
    {
      const session = await open(UNITREE_G1_SCENES.mounted);
      const refusals = {};
      try {
        for (const [label, command] of Object.entries({
          walk: { type: 'walk', velocity: 0.5 },
          setRootPose: { type: 'set_root_pose', positionM: [0, 0, 1] },
          standOnMount: { type: 'engage_stand', controllerId: G1_CONTROLLERS.STAND },
          unknownJoint: { type: 'set_joint_targets', targetsRad: { not_a_joint: 0 } },
        })) {
          try { await session.sendCommand(command, { maxSteps: 10 }); refusals[label] = null; } catch (error) { refusals[label] = String(error.message); }
        }
        let setupRefusal = null;
        try { await session.applySetup({ type: 'place_robot', positionM: [0, 0, 1] }); } catch (error) { setupRefusal = String(error.message); }
        out.refusals = { ...refusals, undeclaredSetup: setupRefusal };
      } finally { session.dispose(); }
    }

    return out;
  });

  await writeFile(testInfo.outputPath('unitree-g1-browser-conformance.json'), JSON.stringify(results, null, 1));
  await testInfo.attach('unitree-g1-browser-conformance.json', { path: testInfo.outputPath('unitree-g1-browser-conformance.json'), contentType: 'application/json' });
  expect(pageErrors, `page errors: ${pageErrors.join('\n')}`).toEqual([]);

  // --- model audit ------------------------------------------------------------------------------
  expect(Object.keys(results.modelAudit).sort()).toEqual(['freebase', 'freebaseDrop', 'mounted', 'mountedBlocked']);
  for (const [key, audit] of Object.entries(results.modelAudit)) {
    expect(audit.jointCount, `${key} joint count`).toBe(29);
    expect(audit.jointOrder, `${key} joint identity`).toBe(true);
    expect(audit.gravityZ, `${key} gravity`).toBeLessThan(-9);
    expect(audit.timestepSeconds, `${key} timestep`).toBeCloseTo(0.002, 9);
    expect(audit.effortLimits.left_knee_joint).toBe(139);
    expect(audit.velocityLimits.left_knee_joint).toBe(20);
  }
  // Root-fixed and free-base are genuinely distinct modes.
  expect(results.modelAudit.mounted.rootMode).toBe('fixed-mounted');
  expect(results.modelAudit.mounted.rootFree).toBe(false);
  expect(results.modelAudit.freebase.rootMode).toBe('free-base');
  expect(results.modelAudit.freebase.rootFree).toBe(true);
  expect(results.modelAudit.mounted.modelSha256).not.toBe(results.modelAudit.freebase.modelSha256);

  // --- requested / accepted / measured -----------------------------------------------------------
  for (const [jointId, record] of Object.entries(results.mountedJointResponse)) {
    expect(record.positionRad, `${jointId} must not echo its command`).not.toBe(record.acceptedTargetRad);
    expect(Math.abs(record.effortNm), `${jointId} effort`).toBeLessThanOrEqual(record.effortLimitNm + 1e-9);
    expect(Math.abs(record.positionRad - record.acceptedTargetRad), `${jointId} tracking`).toBeLessThan(0.35);
  }
  expect(results.commandBounds.requestedTargetRad).toBe(6);
  expect(results.commandBounds.acceptedTargetRad).toBeCloseTo(results.commandBounds.jointRangeRad[1], 9);
  expect(results.commandBounds.commandBounded).toBe(true);
  expect(results.commandBounds.positionRad).not.toBe(results.commandBounds.acceptedTargetRad);
  expect(results.commandBounds.worstEffortFraction).toBeLessThanOrEqual(1 + 1e-9);

  // --- blocked joint -------------------------------------------------------------------------------
  const blocked = results.blockedJoint;
  expect(blocked.unobstructed.fixtureContacts).toEqual([]);
  expect(blocked.unobstructed.positionRad).toBeGreaterThan(0.9);
  expect(blocked.blocked.positionRad, 'the declared wall must stop the joint well short of its target').toBeLessThan(0.4);
  expect(blocked.blocked.acceptedTargetRad, 'the command is still accepted; only the plant refuses it').toBeCloseTo(1.2, 9);
  expect(blocked.blocked.fixtureContacts.length, 'the obstruction must be reported by named geometry pair').toBeGreaterThan(0);
  expect(blocked.blocked.fixtureContacts.join(' ')).toContain('hip_abduction_stop_wall');
  expect(Math.abs(blocked.blocked.effortNm)).toBeCloseTo(blocked.blocked.effortLimitNm, 3);

  // --- self-contact ----------------------------------------------------------------------------------
  expect(results.selfContact.pairs.length, 'a source-legal crossed-leg pose must produce self-contact').toBeGreaterThan(0);
  expect(results.selfContact.bodies.join(' ')).toContain('knee_link');
  expect(Math.abs(results.selfContact.leftMeasuredRad), 'self-contact must stop the commanded motion short').toBeLessThan(Math.abs(results.selfContact.leftAcceptedRad) * 0.7);

  // --- gravity and fall ---------------------------------------------------------------------------------
  expect(results.freeFall.heightChangeM, 'a released robot must fall').toBeLessThan(-0.2);
  expect(results.freeFall.finalUprightZ, 'the fall must change root orientation').toBeLessThan(results.freeFall.initialUprightZ - 0.5);
  expect(results.freeFall.contacts.otherBodyFloor, 'a fallen robot must contact the floor with something other than its feet').toBeGreaterThan(0);

  // --- standing ---------------------------------------------------------------------------------------
  const nominal = results.standNominal;
  expect(nominal.standing, `nominal standing failed: ${JSON.stringify(nominal.checks)}`).toBe(true);
  expect(nominal.engagedControllerId).toBe('robobuddy_g1_stand_v1');
  expect(nominal.actuationEnabled).toBe(true);
  // Nominal standing performs no setup operation at all. The only logged entry is the disclosure
  // of the model package's declared initial command, which is itself a bounded low-level joint
  // hold, not a state write: no qpos/qvel assignment, no weld, no band, no upright correction.
  expect(nominal.setupLog).toEqual([{
    simulationTimeSeconds: 0,
    event: 'initialCommand',
    initialCommand: 'joint-hold',
    detail: 'bounded joint hold at the declared initial joint positions, at the Unitree source hold gains',
  }]);
  expect(JSON.stringify(nominal.setupLog)).not.toContain('set_actuation');
  expect(nominal.measured.maxNonFootGroundContacts).toBe(0);
  expect(nominal.measured.maxExternalOrFixtureContacts, 'no external object or fixture may support the stand').toBe(0);
  expect(nominal.footContacts.left).toBeGreaterThan(0);
  expect(nominal.footContacts.right).toBeGreaterThan(0);
  expect(nominal.measured.worstActuatorEffortFraction).toBeLessThan(0.9);

  // The exact Unitree source controller was tried and is measured not to hold this posture.
  expect(results.standSourceFixStand.standing, 'source FixStand must not be reported as standing').toBe(false);
  expect(results.standSourceFixStand.measured.pelvisHeightMinM).toBeLessThan(0.5);

  // The mandatory negative: with actuation removed the same controller holds nothing up.
  const motorsOff = results.standMotorsDisabled;
  expect(motorsOff.standing, 'a motors-disabled run must never pass the standing gate').toBe(false);
  expect(motorsOff.actuationEnabled).toBe(false);
  expect(motorsOff.measured.pelvisHeightMinM).toBeLessThan(0.5);
  expect(motorsOff.measured.worstActuatorEffortFraction, 'a disabled actuator must produce no force at all').toBeLessThan(1e-9);
  expect(motorsOff.setupLog.length, 'the actuation-disabled condition must be logged').toBeGreaterThan(0);
  expect(JSON.stringify(motorsOff.setupLog)).toContain('set_actuation');

  // --- external object -----------------------------------------------------------------------------------
  expect(results.externalObject.contactPairs.length, 'the robot must contact the declared external object').toBeGreaterThan(0);
  expect(results.externalObject.contactPairs.join(' ')).toContain('contact_probe_block_geom');
  expect(results.externalObject.displacementM, 'the free object must respond physically').toBeGreaterThan(0.02);

  // --- display-rate independence -----------------------------------------------------------------------------
  expect(results.displayRate.simulationTimeDelta).toBeLessThan(1e-9);
  expect(results.displayRate.pelvisHeightDelta, 'observation cadence and rendering must not change the physics').toBeLessThan(1e-9);
  expect(results.displayRate.tiltDelta).toBeLessThan(1e-9);
  expect(results.displayRate.maxJointDelta).toBeLessThan(1e-9);

  // --- lifecycle and refusals ------------------------------------------------------------------------------------
  expect(results.lifecycle.staleRejections.length).toBe(2);
  for (const message of results.lifecycle.staleRejections) expect(message).toMatch(/Stale|epoch/i);
  expect(results.lifecycle.cancelled).toBe(true);
  expect(results.lifecycle.reloadRequired).toBe(true);
  expect(results.lifecycle.afterCancel).toMatch(/No physical scene loaded|No MuJoCo scene is loaded/);
  expect(results.refusals.walk, 'there is no walk operation in the authority').toMatch(/Unsupported physical command/);
  expect(results.refusals.setRootPose, 'there is no root-pose write in the authority').toMatch(/Unsupported physical command/);
  expect(results.refusals.standOnMount, 'a mounted fixture must refuse a standing controller').toMatch(/free-base|does not declare/);
  expect(results.refusals.unknownJoint).toMatch(/Unknown declared joint/);
  expect(results.refusals.undeclaredSetup).toMatch(/not declared by this backend/);

  // --- browser / native conformance -----------------------------------------------------------------------------
  const nativeStand = nativeReport('stand-nominal');
  if (nativeStand) {
    expect(nativeStand.evaluation.passed).toBe(true);
    expect(Math.abs(nominal.measured.pelvisHeightMinM - nativeStand.evaluation.measured.pelvisHeightMinM)).toBeLessThan(TOLERANCE.pelvisHeightM);
    expect(Math.abs(nominal.measured.maxTiltRad - nativeStand.evaluation.measured.maxTiltRad)).toBeLessThan(TOLERANCE.tiltRad);
    expect(Math.abs(nominal.measured.maxHorizontalDriftM - nativeStand.evaluation.measured.maxHorizontalDriftM)).toBeLessThan(TOLERANCE.driftM);
  }
  const nativeSource = nativeReport('stand-source-fixstand');
  if (nativeSource) expect(nativeSource.evaluation.passed, 'native and browser must agree that source FixStand fails').toBe(false);
  const nativeMotors = nativeReport('stand-motors-disabled');
  if (nativeMotors) {
    expect(nativeMotors.evaluation.passed).toBe(false);
    expect(nativeMotors.evaluation.measured.worstActuatorEffortFraction).toBe(0);
  }
  const nativeBlocked = nativeReport('blocked-joint');
  if (nativeBlocked) {
    expect(Math.abs(blocked.blocked.positionRad - nativeBlocked.comparison.blocked.measuredPositionRad)).toBeLessThan(TOLERANCE.jointRad);
    expect(Math.abs(blocked.unobstructed.positionRad - nativeBlocked.comparison.unobstructed.measuredPositionRad)).toBeLessThan(TOLERANCE.jointRad);
  }
  const nativeFall = nativeReport('free-fall');
  if (nativeFall) {
    expect(Math.abs(results.freeFall.heightChangeM - nativeFall.rootHeightChangeM)).toBeLessThan(0.25);
    expect(Math.sign(results.freeFall.finalUprightZ)).toBe(Math.sign(nativeFall.final.root.uprightZ));
  }
  const nativeObject = nativeReport('external-object');
  if (nativeObject) {
    expect(nativeObject.objectDisplacementM).toBeGreaterThan(0.02);
    expect(
      Math.abs(results.externalObject.displacementM - nativeObject.objectDisplacementM),
      'the two backends must move the free object by the same amount, not merely both move it',
    ).toBeLessThan(TOLERANCE.objectDisplacementM);
    expect(results.externalObject.contactPairs.map((pair) => pair.split(' | ').sort().join(' | ')).sort())
      .toEqual(nativeObject.contactGeomPairs.map((pair) => [...pair].sort().join(' | ')).sort());
  }
  const nativeSelf = nativeReport('self-contact');
  if (nativeSelf) expect(nativeSelf.selfContactPairs.flat().join(' ')).toContain('knee_link');
  expect(nativeComparisons, NATIVE_DIR
    ? 'every declared native conformance report must have been compared'
    : 'no native report directory is configured, so no native comparison may claim to have run',
  ).toBe(NATIVE_DIR ? NATIVE_TRIALS.length : 0);
  await testInfo.attach('unitree-g1-native-conformance', {
    body: JSON.stringify({
      nativeReportDir: NATIVE_DIR || null,
      nativeComparisons,
      tolerance: TOLERANCE,
      browser: {
        standNominal: nominal.measured,
        blocked: { blockedRad: blocked.blocked.positionRad, unobstructedRad: blocked.unobstructed.positionRad },
        freeFall: results.freeFall,
        externalObject: results.externalObject,
      },
      native: NATIVE_DIR ? {
        standNominal: nativeStand.evaluation.measured,
        blocked: { blockedRad: nativeBlocked.comparison.blocked.measuredPositionRad, unobstructedRad: nativeBlocked.comparison.unobstructed.measuredPositionRad },
        freeFall: { heightChangeM: nativeFall.rootHeightChangeM, finalUprightZ: nativeFall.final.root.uprightZ },
        externalObject: { displacementM: nativeObject.objectDisplacementM },
      } : null,
    }, null, 2),
    contentType: 'application/json',
  });
});


// Independent review: reject malformed requests atomically at the actual worker boundary,
// rather than relying only on the UI/Python validators to protect an engaged controller.
test('G1 rejected commands preserve the active controller and expose real support forces', async ({ page }) => {
  await page.goto('/physics-slice.html', { waitUntil: 'domcontentloaded' });
  const result = await page.evaluate(async () => {
    const { PhysicsSession } = await import('/src/physics/session.js');
    const { BrowserMuJoCoBackend } = await import('/src/physics/browser-mujoco-backend.js');
    const { UNITREE_G1_FREEBASE_SCENE } = await import('/src/physics/unitree-g1-scene.js');
    const session = new PhysicsSession(new BrowserMuJoCoBackend({ workerUrl: new URL('/src/physics/unitree-g1-mujoco-worker.js', location.href) }));
    try {
      await session.loadScene(UNITREE_G1_FREEBASE_SCENE);
      await session.sendCommand({ type: 'engage_stand' }, { maxSteps: 500 });
      const before = await session.getObservation();
      const errors = [];
      for (const command of [
        { type: 'set_joint_targets', targetsRad: { left_knee_joint: NaN } },
        { type: 'set_lowlevel_targets', commands: { left_knee_joint: { kp: Infinity } } },
      ]) {
        try { await session.sendCommand(command, { maxSteps: 500 }); errors.push(null); }
        catch (error) { errors.push(String(error)); }
      }
      const after = await session.getObservation();
      const settled = await session.advanceSteps(500);
      return { errors, before: before.controller, after: after.controller,
        sameJoints: JSON.stringify(before.joints) === JSON.stringify(after.joints),
        worldFrame: settled.root.frame,
        supportN: ['leftFootFloor', 'rightFootFloor'].map((side) => settled.contactClasses[side].reduce((sum, c) => sum + c.normalForceN, 0)) };
    } finally { session.dispose(); }
  });
  expect(result.errors.every((error) => error?.includes('finite'))).toBe(true);
  expect(result.after).toEqual(result.before);
  expect(result.sameJoints).toBe(true);
  expect(result.worldFrame).toBe('mujoco_world');
  for (const force of result.supportN) expect(force).toBeGreaterThan(1);
});
