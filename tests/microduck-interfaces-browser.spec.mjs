import { expect, test } from '@playwright/test';

async function openPhysical(page, task = 'microduck-physical-locomotion') {
  await page.addInitScript(() => {
    const registrations = [];
    Object.defineProperty(document, 'modelContext', { configurable: true, value: {
      registerTool(tool, options = {}) { registrations.push({ tool, signal: options.signal }); return Promise.resolve(); },
    } });
    window.__interfaceRegistrations = registrations;
  });
  await page.goto('/?ci=microduck-interfaces', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
  await page.locator('#robotSelect').selectOption('microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 120_000 });
  await page.locator('#taskSelect').selectOption(task);
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 120_000 });
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-workspace', 'physical');
}
async function invoke(page, input) {
  return page.evaluate(async (request) => {
    const entry = window.__interfaceRegistrations.findLast(({ tool, signal }) => tool.name === 'control_microduck_physical_simulation' && !signal?.aborted);
    if (!entry) throw new Error('The physical MicroDuck WebMCP tool was not registered');
    return entry.tool.execute({ schema_version: 'robobuddy.microduck.physical.v1', ...request }, { signal: new AbortController().signal });
  }, input);
}

test('registered MicroDuck WebMCP commands reach the real controller through the IDE', async ({ page }) => {
  test.setTimeout(240_000);
  const errors = [];
  page.on('pageerror', (error) => errors.push(String(error)));
  await openPhysical(page);
  await page.locator('#agentAccessToggle').click();
  await expect.poll(() => page.evaluate(() => window.__interfaceRegistrations.some(({ tool, signal }) => tool.name === 'control_microduck_physical_simulation' && !signal?.aborted))).toBe(true);
  const move = await invoke(page, { command: 'set_command', request: { vx: 0.35 }, advance_seconds: 0.1 });
  expect(move.ok, JSON.stringify(move)).toBe(true);
  expect(move.controller.policyId).toBe('walking');
  expect(move.actual.simulationTimeSeconds).toBeCloseTo(0.1, 8);
  const zero = await invoke(page, { command: 'advance', advance_seconds: 0 });
  expect(zero.ok, JSON.stringify(zero)).toBe(true);
  expect(zero.executedTicks).toBe(0);
  expect(zero.actual.simulationTimeSeconds).toBe(move.actual.simulationTimeSeconds);
  const bad = await invoke(page, { command: 'set_command', request: { vx: null } });
  expect(bad).toMatchObject({ ok: false, error: { code: 'INVALID_ARGUMENT' } });
  const mismatch = await invoke(page, { command: 'request_skill', skill: 'kick_right' });
  expect(mismatch).toMatchObject({ ok: false, error: { code: 'PLANT_MISMATCH' } });
  await page.locator('#taskSelect').selectOption('microduck-physical-groundcontact');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 120_000 });
  const sit = await invoke(page, { command: 'request_skill', skill: 'sit', advance_seconds: 0.1 });
  expect(sit.ok, JSON.stringify(sit)).toBe(true);
  expect(sit.controller.policyId).toBe('sitstand');
  const rise = await invoke(page, { command: 'request_skill', skill: 'stand_up', advance_seconds: 0.1 });
  expect(rise.ok, JSON.stringify(rise)).toBe(true);
  expect(rise.controller.policyId).toBe('sitstand');
  const roller = await invoke(page, { command: 'request_skill', skill: 'roller' });
  expect(roller).toMatchObject({ ok: false, error: { code: 'CAPABILITY_UNSUPPORTED' } });
  expect(errors).toEqual([]);
});

for (const task of ['microduck-physical-locomotion', 'microduck-physical-groundcontact', 'microduck-physical-kick']) {
  test(`${task} default Python starter executes through the real IDE bridge`, async ({ page }) => {
    test.setTimeout(240_000);
    const errors = [];
    page.on('pageerror', (error) => errors.push(String(error)));
    await openPhysical(page, task);
    const result = await page.evaluate(async () => {
      const app = window.__robobuddyCi.app;
      const source = app.editor.cm.getValue();
      await app.run();
      return { source, console: app.console, problems: app.problems, state: app.sim.getState(), execution: app.getExecutionState() };
    });
    expect(result.source).toContain('MicroDuck physical MuJoCo workspace');
    expect(result.source).not.toContain('benchmark_block');
    expect(result.console.stderr, JSON.stringify(result.problems)).toBe('');
    expect(result.console.stdout, JSON.stringify(result.problems)).toContain('MicroDuck physical observation');
    expect(result.console.stdout).toContain('Measured task report');
    expect(result.execution).toBe('idle');
    expect(result.state.actual.simulationTimeSeconds).toBeGreaterThan(0);
    expect(result.state.hardwareValidated).toBe(false);
    expect(errors).toEqual([]);
  });
}

for (const mode of ['abort', 'access-off']) {
  test(`MicroDuck discards in-flight inference after ${mode}`, async ({ page }) => {
    test.setTimeout(240_000);
    const errors = [];
    page.on('pageerror', (error) => errors.push(String(error)));
    await openPhysical(page);
    await page.locator('#agentAccessToggle').click();
    await expect.poll(() => page.evaluate(() => window.__interfaceRegistrations.some(({ tool, signal }) => tool.name === 'control_microduck_physical_simulation' && !signal?.aborted))).toBe(true);
    await page.evaluate(() => {
      const sim = window.__robobuddyCi.app.sim.backend;
      const entry = window.__interfaceRegistrations.findLast(({ tool, signal }) => tool.name === 'control_microduck_physical_simulation' && !signal?.aborted);
      const probe = { inferences: 0, commands: 0, steps: 0, pending: false, abort: new AbortController() };
      const infer = sim.policyRuntime.infer.bind(sim.policyRuntime);
      const send = sim.session.sendCommand.bind(sim.session);
      const advanceSteps = sim.session.advanceSteps.bind(sim.session);
      const advance = sim.advanceSeconds.bind(sim);
      // Real ONNX inference and MuJoCo still execute. Delay only the return of the
      // second inference so opt-out happens at a deterministic await boundary.
      sim.policyRuntime.infer = async (...args) => {
        const action = await infer(...args);
        probe.inferences += 1;
        if (probe.inferences === 2) {
          probe.before = sim.lastObservation.simulationTimeSeconds;
          probe.pending = true;
          await new Promise((resolve) => { probe.release = resolve; });
        }
        return action;
      };
      sim.session.sendCommand = (...args) => { probe.commands += 1; return send(...args); };
      sim.session.advanceSteps = (...args) => { probe.steps += args[0]; return advanceSteps(...args); };
      sim.advanceSeconds = (...args) => {
        const run = advance(...args);
        probe.backendDone = run.then(() => null, (error) => error.code);
        return run;
      };
      probe.restore = () => {
        sim.policyRuntime.infer = infer;
        sim.session.sendCommand = send;
        sim.session.advanceSteps = advanceSteps;
        sim.advanceSeconds = advance;
      };
      window.__cancellationProbe = probe;
      probe.result = entry.tool.execute({ schema_version: 'robobuddy.microduck.physical.v1', command: 'set_command', request: { vx: 0.35 }, advance_seconds: 2 }, { signal: probe.abort.signal });
    });
    await expect.poll(() => page.evaluate(() => window.__cancellationProbe.pending)).toBe(true);
    if (mode === 'abort') await page.evaluate(() => window.__cancellationProbe.abort.abort());
    else {
      await page.locator('#agentAccessToggle').click();
      await expect(page.locator('#agentAccessControl')).toHaveAttribute('data-access', 'off');
    }
    const result = await page.evaluate(async () => {
      const probe = window.__cancellationProbe;
      probe.release();
      try {
        const response = await probe.result;
        const backendError = await probe.backendDone;
        return { response, backendError, inferences: probe.inferences, commands: probe.commands, steps: probe.steps,
          before: probe.before, after: window.__robobuddyCi.app.sim.backend.lastObservation.simulationTimeSeconds };
      } finally { probe.restore(); }
    });
    // setAccess rotates the registration epoch; AgentFacade.assertActive reports
    // that revocation as STALE_REGISTRATION. A call-signal abort has its separate
    // OPERATION_CANCELLED code. Both must stop the real backend, not just its reply.
    const expectedError = mode === 'abort' ? 'OPERATION_CANCELLED' : 'STALE_REGISTRATION';
    console.log(`MicroDuck ${mode} cancellation evidence: ${JSON.stringify(result)}`);
    expect(result.response).toMatchObject({ ok: false, error: { code: expectedError } });
    expect(result.backendError).toBe(expectedError);
    expect(result.inferences).toBe(2);
    expect(result.commands).toBe(1);
    expect(result.steps).toBe(4);
    expect(result.before).toBeCloseTo(0.02, 8);
    expect(result.after).toBe(result.before);
    // Cancellation is scoped to the tool call, not a permanent simulator latch.
    // A call abort leaves consent enabled; only access-off needs a fresh opt-in.
    await expect(page.locator('#agentAccessToggle')).toHaveAttribute('aria-checked', mode === 'access-off' ? 'false' : 'true');
    if (mode === 'access-off') await page.locator('#agentAccessToggle').click();
    await expect(page.locator('#agentAccessToggle')).toHaveAttribute('aria-checked', 'true');
    const resumed = await invoke(page, { command: 'advance', advance_seconds: 0.02 });
    expect(resumed.ok, JSON.stringify(resumed)).toBe(true);
    expect(resumed.actual.simulationTimeSeconds).toBeCloseTo(0.04, 8);
    expect(errors).toEqual([]);
  });
}
