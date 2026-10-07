import { expect, test } from '@playwright/test';

// Unitree G1 Phase 5D end-to-end IDE journey.
//
// It exercises what a learner and an agent actually reach: the two Unitree workspaces side by side,
// the physical workspace's live async Python, and the bounded WebMCP surface - all against the one
// PhysicsSession that the renderer and the evaluator observe.

test('The Unitree profile exposes a physical workspace beside the source pose workspace', async ({ page }) => {
  test.setTimeout(600_000);
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  // Agent Assist is only offerable when a WebMCP interface exists, so the spec provides one and
  // then grants access exactly the way a person does: by clicking the opt-in control.
  await page.addInitScript(() => {
    const registrations = [];
    Object.defineProperty(document, 'modelContext', {
      configurable: true,
      value: { registerTool(tool, options = {}) { registrations.push({ tool, signal: options.signal || null }); return Promise.resolve(); } },
    });
    window.__webMcpRegistrations = registrations;
  });
  await page.goto('/?ci=unitree-g1-phase5d', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
  await page.locator('#robotSelect').selectOption('unitree');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 120_000 });

  // Both workspaces remain selectable, and the physical one is the default.
  const options = await page.locator('#taskSelect option').evaluateAll((nodes) => nodes.map((node) => ({ value: node.value, text: node.textContent })));
  expect(options).toEqual([
    { value: 'unitree-g1-physical-dynamics', text: 'Unitree G1 29-DoF Physical Dynamics' },
    { value: 'unitree-g1-kinematic-pose-inspection', text: 'Unitree G1 29-DoF Kinematic Pose Inspection' },
  ]);
  await expect(page.locator('#driverLabel')).toContainText('browser MuJoCo');
  await expect(page.locator('#fidelityText')).toContainText('Walking is unsupported');

  const canvas = page.locator('#simCanvas');
  await expect(canvas).toHaveAttribute('data-simulator-backend', 'browser-mujoco');
  await expect(canvas).toHaveAttribute('data-simulation-authority', 'physics-session');
  await expect(canvas).toHaveAttribute('data-unitree-g1-root-mode', 'free-base');
  await expect(canvas).toHaveAttribute('data-unitree-g1-walking', 'unsupported');
  await expect(canvas).toHaveAttribute('data-unitree-g1-hands', 'fixed-rubber-passive');

  // The generated starter must teach the honest API surface and must not invent a walk.
  const starter = await page.evaluate(() => ({ ...window.__robobuddyCi.app.files }));
  expect(starter['main.py']).toContain('await robot.stand()');
  expect(starter['main.py']).toContain('await robot.get_state()');
  expect(starter['main.py']).toContain('requested');
  expect(starter['main.py']).toContain('measured');
  expect(starter['main.py']).not.toMatch(/robot\.walk|set_root|force_upright/);
  expect(starter['robot_config.py']).toContain('WALKING = "unsupported"');
  expect(starter['robot_config.py']).toContain('LOWLEVEL_CONTROL_INTERVAL_S = 0.002');

  // --- the physical plant, driven through the simulator the IDE actually holds -----------------
  const physical = await page.evaluate(async () => {
    const sim = window.__robobuddyCi.app.sim.backend;
    await sim.advanceTime(0.2);
    const settled = sim.getState();
    // Engage the controller and then run the whole declared evaluation window: two seconds of
    // source ramp followed by six evaluated seconds, so the gate is really satisfied rather than
    // sampled early.
    await sim.engageStand({ holdSeconds: 2 });
    for (let index = 0; index < 4; index += 1) await sim.advanceTime(1.6);
    const standing = sim.getState();
    const evaluation = sim.getTaskEvaluation();
    // Commanding a joint releases the standing controller. That is stated in the starter and it
    // is observable here.
    await sim.applyPhysicalTargets({ waist_yaw_joint: 0.4 });
    await sim.advanceTime(0.5);
    const afterCommand = sim.getState();
    return { settled, standing, evaluation, afterCommand, telemetry: sim.getTelemetry(), contacts: sim.getContacts(), audit: sim.getPresentationAudit() };
  });

  // Gravity, real foot contact, and a genuine free root.
  expect(physical.settled.root.mode).toBe('free-base');
  expect(physical.settled.foot_contacts.left.length).toBeGreaterThan(0);
  expect(physical.settled.foot_contacts.right.length).toBeGreaterThan(0);
  expect(physical.settled.foot_contacts.left.flat().join(' ')).toContain('left_foot_');
  expect(physical.settled.foot_contacts.left.flat().join(' ')).toContain('floor');

  // The verified standing controller earns an upright posture with no support of any kind.
  expect(physical.standing.controller_mode).toBe('robobuddy_g1_stand_v1');
  expect(physical.standing.root.position_m[2]).toBeGreaterThan(0.73);
  expect(physical.standing.root.position_m[2]).toBeLessThan(0.84);
  expect(physical.standing.root.tilt_rad).toBeLessThan(0.15);
  expect(physical.standing.contacts.non_foot_ground).toEqual([]);
  expect(physical.standing.contacts.external_object).toEqual([]);
  expect(physical.evaluation.standing).toBe(true);
  expect(physical.evaluation.checks.noExternalOrFixtureSupport).toBe(true);
  expect(physical.telemetry.walking).toBe('unsupported');

  // Requested, accepted and measured are separately reported for the commanded joint.
  const waist = physical.afterCommand.joints.waist_yaw_joint;
  expect(waist.requested_target_rad).toBeCloseTo(0.4, 9);
  expect(waist.accepted_target_rad).toBeCloseTo(0.4, 9);
  expect(waist.position_rad).not.toBe(waist.accepted_target_rad);
  expect(Math.abs(waist.effort_nm)).toBeLessThanOrEqual(waist.effort_limit_nm + 1e-9);
  expect(physical.afterCommand.controller_mode).toBe('unitree_g1_joint_hold_v1');
  expect(physical.afterCommand.walking).toBe('unsupported');
  expect(physical.audit.presentationAuthority).toContain('observed MuJoCo pelvis transform');

  // --- live async Python runs against the same session ------------------------------------------
  const program = [
    'from robobuddy.sim import connect',
    'robot = await connect("unitree_g1_29dof_physical")',
    'try:',
    '    await robot.wait_sim(0.2)',
    '    await robot.stand()',
    '    for _ in range(4):',
    '        await robot.wait_sim(1.6)',
    '    state = await robot.get_state()',
    '    print("MODE", state["controller_mode"])',
    '    print("PELVIS_Z", round(state["root"]["position_m"][2], 4))',
    '    print("TILT", round(state["root"]["tilt_rad"], 4))',
    '    print("LEFT_FOOT", len(state["foot_contacts"]["left"]))',
    '    print("RIGHT_FOOT", len(state["foot_contacts"]["right"]))',
    '    print("NON_FOOT", len(state["contacts"]["non_foot_ground"]))',
    '    await robot.set_joint_targets({"left_elbow_joint": 1.4})',
    '    await robot.wait_sim(1.0)',
    '    after = await robot.get_state()',
    '    elbow = after["joints"]["left_elbow_joint"]',
    '    print("REQUESTED", round(elbow["requested_target_rad"], 4))',
    '    print("MEASURED", round(elbow["position_rad"], 4))',
    '    print("DIFFERS", elbow["requested_target_rad"] != elbow["position_rad"])',
    '    print("HAS_WALK", hasattr(robot, "walk"))',
    'finally:',
    '    await robot.disconnect()',
    '',
  ].join('\n');
  await page.evaluate(async (source) => {
    const app = window.__robobuddyCi.app;
    await app.sim.backend.reset();
    app.files['main.py'] = source;
    app.openFile('main.py');
    await app.run();
  }, program);
  const output = await page.evaluate(() => `${window.__robobuddyCi.app.console.stdout}\n${window.__robobuddyCi.app.console.stderr}`);
  expect(output).toContain('MODE robobuddy_g1_stand_v1');
  expect(output).toContain('DIFFERS True');
  expect(output).toContain('HAS_WALK False');
  const pelvisZ = Number(/PELVIS_Z ([\d.]+)/.exec(output)?.[1]);
  expect(pelvisZ).toBeGreaterThan(0.73);
  expect(pelvisZ).toBeLessThan(0.84);
  expect(Number(/TILT ([\d.]+)/.exec(output)?.[1])).toBeLessThan(0.15);
  expect(Number(/LEFT_FOOT (\d+)/.exec(output)?.[1])).toBeGreaterThan(0);
  expect(Number(/RIGHT_FOOT (\d+)/.exec(output)?.[1])).toBeGreaterThan(0);
  expect(Number(/NON_FOOT (\d+)/.exec(output)?.[1])).toBe(0);

  // --- bounded WebMCP over the same session -------------------------------------------------------
  // Agent Assist stays opt-in and is granted here the way a person grants it: by clicking.
  await page.locator('#agentAccessToggle').click();
  await expect(page.locator('#agentAccessToggle')).toHaveAttribute('aria-checked', 'true');

  // The tool must actually be registered with the browser interface, not merely definable.
  const registered = await page.evaluate(() => window.__webMcpRegistrations.filter(({ signal }) => !signal?.aborted).map(({ tool }) => tool.name));
  expect(registered).toContain('control_unitree_g1_physical_simulation');
  // The two workspaces must never share a tool name: the kinematic pose tool writes joint angles
  // straight into the rig, so it must not be reachable while the physical plant is displayed.
  expect(registered).not.toContain('control_unitree_g1_simulation');
  expect(registered.filter((name) => name === 'control_unitree_g1_physical_simulation')).toHaveLength(1);

  const webmcp = await page.evaluate(async () => {
    const { executeUnitreeG1PhysicalControl, getUnitreeG1PhysicalControlDefinition, WEBMCP_UNITREE_G1_SCHEMA_VERSION } = await import('/src/webmcp/unitree-g1-physical-control.js');
    const facade = window.__robobuddyCi.agentFacade;
    const epoch = facade.registrationEpoch;
    const definition = getUnitreeG1PhysicalControlDefinition(facade);
    const call = async (input) => {
      try { return { ok: true, result: await executeUnitreeG1PhysicalControl(facade, { schema_version: WEBMCP_UNITREE_G1_SCHEMA_VERSION, ...input }, null, epoch) }; }
      catch (error) { return { ok: false, code: error?.code ?? null, message: String(error?.message || error) }; }
    };
    const inspect = await call({ command: 'inspect_capability' });
    const reset = await call({ command: 'reset' });
    const nextEpoch = facade.registrationEpoch;
    const call2 = async (input) => {
      try { return { ok: true, result: await executeUnitreeG1PhysicalControl(facade, { schema_version: WEBMCP_UNITREE_G1_SCHEMA_VERSION, ...input }, null, nextEpoch) }; }
      catch (error) { return { ok: false, code: error?.code ?? null, message: String(error?.message || error) }; }
    };
    const advance = await call2({ command: 'advance', advance_seconds: 0.2 });
    const stand = await call2({ command: 'stand', hold_seconds: 2 });
    let hold2 = null;
    for (let index = 0; index < 4; index += 1) hold2 = await call2({ command: 'advance', advance_seconds: 1.6 });
    const read = await call2({ command: 'read_state' });
    const targets = await call2({ command: 'set_joint_targets', targets_rad: { waist_yaw_joint: 0.3 }, advance_seconds: 0.5 });
    const badWalk = await call2({ command: 'walk', velocity: 0.5 });
    const badRoot = await call2({ command: 'set_root_pose', position_m: [0, 0, 1] });
    const outOfRange = await call2({ command: 'set_joint_targets', targets_rad: { left_knee_joint: 6 } });
    const badActuation = await call2({ command: 'set_actuation', enabled: false });
    return { definition, inspect, reset, advance, stand, hold2, read, targets, badWalk, badRoot, outOfRange, badActuation };
  });

  expect(webmcp.definition.name).toBe('control_unitree_g1_physical_simulation');
  expect(webmcp.definition.description).toContain('Walking is unsupported');
  expect(webmcp.inspect.ok).toBe(true);
  expect(webmcp.inspect.result.rootMode).toBe('free-base');
  expect(webmcp.inspect.result.capabilities.find((item) => item.id === 'walking').capability).toBe('unsupported');
  expect(webmcp.inspect.result.capabilities.find((item) => item.id === 'perturbation_recovery').capability).toBe('unsupported');
  expect(webmcp.inspect.result.capabilities.find((item) => item.id === 'dexterous_hands').capability).toBe('unsupported');
  expect(webmcp.inspect.result.capabilities.find((item) => item.id === 'standing').capability).toBe('physical / verified');
  expect(webmcp.inspect.result.hardwareValidated).toBe(false);
  expect(webmcp.reset.ok).toBe(true);
  expect(webmcp.reset.result.note).toContain('not a recovery');
  expect(webmcp.stand.ok).toBe(true);
  expect(webmcp.stand.result.claim).toContain('Not dynamic balance');
  expect(webmcp.hold2.ok).toBe(true);
  expect(webmcp.read.ok).toBe(true);
  expect(webmcp.read.result.observedState.root.position_m[2]).toBeGreaterThan(0.73);
  expect(webmcp.read.result.taskEvaluation.standing).toBe(true);
  expect(webmcp.targets.ok).toBe(true);
  expect(webmcp.targets.result.note).toContain('not a measurement');
  // Unsupported and forbidden operations are refused by schema validation, not silently accepted.
  for (const refused of [webmcp.badWalk, webmcp.badRoot, webmcp.outOfRange, webmcp.badActuation]) {
    expect(refused.ok).toBe(false);
    expect(refused.code).toBe('INVALID_ARGUMENT');
  }

  // --- switching to the source pose workspace changes the labels, not just the scene -------------
  await page.locator('#taskSelect').selectOption('unitree-g1-kinematic-pose-inspection');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 120_000 });
  await expect(page.locator('#fidelityText')).toContainText('No fixed-step contact plant');
  const kinematic = await page.evaluate(() => ({
    backend: document.getElementById('simCanvas').dataset.simulatorBackend ?? null,
    rootMode: document.getElementById('simCanvas').dataset.unitreeG1RootMode ?? null,
    physical: window.__robobuddyCi.app.isPhysicalWorkspace(),
    kinematic: window.__robobuddyCi.app.isKinematicPoseWorkspace(),
    hasPhysicalSession: Boolean(window.__robobuddyCi.app.sim.getPhysicalSession()),
  }));
  expect(kinematic.physical).toBe(false);
  expect(kinematic.kinematic).toBe(true);
  expect(kinematic.hasPhysicalSession, 'the pose workspace must have no physical session at all').toBe(false);
  expect(kinematic.rootMode, 'the pose workspace must not advertise a free base').not.toBe('free-base');

  // The physical WebMCP tool must not be offered while the pose workspace is selected.
  const definitionGone = await page.evaluate(async () => {
    const { getUnitreeG1PhysicalControlDefinition } = await import('/src/webmcp/unitree-g1-physical-control.js');
    return getUnitreeG1PhysicalControlDefinition(window.__robobuddyCi.agentFacade);
  });
  expect(definitionGone).toBeNull();
  // Re-registration is asynchronous, so poll for the withdrawal rather than sampling once.
  const activeTools = () => page.evaluate(() => window.__webMcpRegistrations.filter(({ signal }) => !signal?.aborted).map(({ tool }) => tool.name));
  await expect.poll(
    activeTools,
    { message: 'the physical tool must be withdrawn when the pose workspace is selected', timeout: 30_000 },
  ).not.toContain('control_unitree_g1_physical_simulation');
  // The retained pose workspace keeps its own preserved P6 tool, under its own separate name.
  expect(await activeTools()).toContain('control_unitree_g1_simulation');

  expect(pageErrors, `page errors: ${pageErrors.join('\n')}`).toEqual([]);
});
