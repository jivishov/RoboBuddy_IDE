import { expect, test } from '@playwright/test';
import { writeFile } from 'node:fs/promises';

// Only browser tool registration is adapted. The IDE, facade, session, worker,
// Python bridge, model compiler and MuJoCo dynamics are the shipped code.
async function installWebMcpMock(page, { failAt = 0 } = {}) {
  await page.addInitScript(({ registrationFailureAt }) => {
    const registrations = [];
    Object.defineProperty(document, 'modelContext', {
      configurable: true,
      value: {
        registerTool(tool, options = {}) {
          registrations.push({ tool, signal: options.signal || null });
          if (registrationFailureAt && registrations.length === registrationFailureAt) return Promise.reject(new Error('mock partial registration failure'));
          return Promise.resolve();
        },
      },
    });
    window.__webMcpRegistrations = registrations;
  }, { registrationFailureAt: failAt });
}

async function openReadyApp(page) {
  await installWebMcpMock(page);
  await page.goto('/?ci=webmcp', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
}

async function callTool(page, name, input = {}, { aborted = false, abortAfterMs = 0 } = {}) {
  return page.evaluate(async ({ toolName, toolInput, shouldAbort, delayedAbort }) => {
    const entry = window.__webMcpRegistrations.findLast((candidate) => candidate.tool.name === toolName && !candidate.signal?.aborted);
    if (!entry) throw new Error(`Missing WebMCP tool: ${toolName}`);
    const controller = new AbortController();
    if (shouldAbort) controller.abort();
    else if (delayedAbort) setTimeout(() => controller.abort(), delayedAbort);
    return entry.tool.execute(toolInput, { signal: controller.signal });
  }, { toolName: name, toolInput: input, shouldAbort: aborted, delayedAbort: abortAfterMs });
}

async function activeTools(page) {
  return page.evaluate(() => window.__webMcpRegistrations.filter(({ signal }) => !signal?.aborted).map(({ tool }) => tool.name));
}

const BASE_TOOLS = [
  'describe_robobuddy_task', 'read_robobuddy_workspace', 'inspect_robobuddy_simulation',
  'focus_robobuddy_workspace', 'run_robobuddy_program', 'draft_robobuddy_cooperative_edit',
];
const OPENARM_TOOLS = [...BASE_TOOLS, 'control_openarm_simulation', 'inspect_openarm_workcell', 'manage_openarm_workcell', 'run_openarm_program', 'inspect_openarm_scene', 'manage_openarm_scene', 'manage_openarm_task'];
const READ_ONLY_TOOLS = new Set([...BASE_TOOLS.slice(0, 3), 'inspect_openarm_workcell', 'inspect_openarm_scene']);

test('explicit human Agent Assist registers a bounded, cancellation-aware RoboBuddy WebMCP surface', async ({ page }) => {
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await openReadyApp(page);
  const control = page.locator('#agentAccessControl');
  const toggle = page.getByRole('switch', { name: 'Agent Assist' });
  await expect(control).toHaveAttribute('data-access', 'off');
  await expect(control).toHaveAttribute('data-available', 'true');
  await expect(control).toHaveAttribute('data-tools', 'disabled');
  await expect(toggle).toHaveAttribute('aria-checked', 'false');
  await expect(control.locator('.agent-access-state')).toHaveText('OFF');
  expect(await page.evaluate(() => window.__webMcpRegistrations.length)).toBe(0);
  await page.locator('#agentAccessToggle').evaluate((button) => button.click());
  await expect(control).toHaveAttribute('data-access', 'off');
  expect(await page.evaluate(() => window.__webMcpRegistrations.length)).toBe(0);
  await page.locator('#agentAccessToggle').click();
  await expect(control).toHaveAttribute('data-access', 'assist');
  await expect(control).toHaveAttribute('data-tools', 'enabled');
  await expect(toggle).toHaveAttribute('aria-checked', 'true');
  await expect(control.locator('.agent-access-state')).toHaveText('ON');
  await expect.poll(() => activeTools(page)).toEqual(OPENARM_TOOLS);
  const registered = await page.evaluate(() => window.__webMcpRegistrations.filter(({ signal }) => !signal?.aborted).map(({ tool }) => ({ name: tool.name, annotations: tool.annotations, inputSchema: tool.inputSchema })));
  for (const tool of registered) {
    expect(tool.annotations.untrustedContentHint).toBe(true);
    expect(tool.annotations.readOnlyHint, tool.name).toBe(READ_ONLY_TOOLS.has(tool.name));
    expect(tool.inputSchema.type).toBe('object');
    // oneOf branches close their own property scope. Closing an empty root
    // rejects ALL valid branch properties; that was the original schema bug.
    if (tool.inputSchema.oneOf) {
      expect(tool.inputSchema.oneOf.every(branch => branch.additionalProperties === false)).toBe(true);
      expect(tool.inputSchema.additionalProperties).not.toBe(false);
    } else expect(tool.inputSchema.additionalProperties).toBe(false);
  }
  const task = await callTool(page, 'describe_robobuddy_task');
  expect(task).toMatchObject({ taskId: 'openarm-04-filtration-workcell', simulationMode: 'physical_mujoco', sourcePlantAvailable: false, physicalSimulationAvailable: true, hardwareValidated: false });
  expect(JSON.stringify(task)).not.toContain('referenceActions');
  const source = await callTool(page, 'read_robobuddy_workspace', { file: 'trajectories.py' });
  expect(source).toMatchObject({ file: 'trajectories.py', startLine: 1, contentClassification: 'untrusted_user_authored_source' });
  expect(source.content.length).toBeLessThanOrEqual(1_200);
  expect(source.endLine - source.startLine + 1).toBeLessThanOrEqual(32);
  const state = await callTool(page, 'inspect_robobuddy_simulation');
  expect(state).toMatchObject({ executionState: 'idle', simulationMode: 'physical_mujoco', stateKind: 'mujoco_physical_state', physicalSimulationAvailable: true, untrustedContent: true });
  const focus = await callTool(page, 'focus_robobuddy_workspace', { file: 'main.py', line: 1 });
  expect(focus).toMatchObject({ focused: true, file: 'main.py', line: 1, sourceChanged: false });
  await expect(page.locator('#statusMessage')).toContainText('Agent focused main.py:1');
  await page.evaluate(() => window.__robobuddyCi.app.editor.cm.setValue('async def demo_move():\n    await robot.move(3.0, 0.0, 0.0)\n'));
  const temporaryEdit = await callTool(page, 'draft_robobuddy_cooperative_edit', {
    file: 'main.py', start_line: 2, end_line: 2, expected_source: '    await robot.move(3.0, 0.0, 0.0)', replacement_code: '    await robot.move(0.30, 0.0, 0.0)', explanation: 'Use the bounded teaching speed for a visible, controlled move.',
  });
  expect(temporaryEdit).toMatchObject({ sourceChanged: true, temporary: true, persistence: 'not_saved_refresh_reloads_workspace', file: 'main.py', disabledOriginal: true, startLine: 2, endLine: 2, workingStartLine: 7 });
  const draftedSource = await page.evaluate(() => window.__robobuddyCi.app.editor.cm.getValue());
  expect(draftedSource).toContain('    # Agent-disabled: await robot.move(3.0, 0.0, 0.0)');
  expect(draftedSource).toContain('    # Explanation: Use the bounded teaching speed for a visible, controlled move.');
  expect(draftedSource).toContain('\n    await robot.move(0.30, 0.0, 0.0)');
  expect(draftedSource).not.toContain('\n        await robot.move(0.30, 0.0, 0.0)');
  await expect(page.locator('#statusMessage')).toContainText('temporary cooperative edit');
  expect(await page.locator('#dirtyDot').isHidden()).toBe(false);
  const staleEdit = await callTool(page, 'draft_robobuddy_cooperative_edit', {
    file: 'main.py', start_line: 2, end_line: 2, expected_source: '    await robot.move(3.0, 0.0, 0.0)', replacement_code: '    await robot.move(0.20, 0.0, 0.0)', explanation: 'This must not overwrite a changed draft.',
  });
  expect(staleEdit).toMatchObject({ ok: false, error: { code: 'SOURCE_MISMATCH', retryable: true } });
  expect(await page.evaluate(() => window.__robobuddyCi.app.editor.cm.getValue())).toBe(draftedSource);
  await page.evaluate(() => window.__robobuddyCi.app.editor.cm.setValue('print("webmcp run smoke")\n'));
  const run = await callTool(page, 'run_robobuddy_program');
  expect(run).toMatchObject({ completed: true, simulation: { executionState: 'idle' } });
  expect(run.simulation.status).toContain('physical task criteria');
  const cancelled = await callTool(page, 'run_robobuddy_program', {}, { aborted: true });
  expect(cancelled).toMatchObject({ ok: false, error: { code: 'OPERATION_CANCELLED', retryable: true } });
  await page.locator('#agentAccessToggle').click();
  await expect(control).toHaveAttribute('data-access', 'off');
  await expect(control).toHaveAttribute('data-tools', 'disabled');
  expect(await page.evaluate(() => window.__webMcpRegistrations.every(({ signal }) => signal.aborted))).toBe(true);
  await page.reload({ waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
  const reloadedSource = await page.evaluate(() => window.__robobuddyCi.app.editor.cm.getValue());
  expect(reloadedSource).not.toContain('Agent cooperative edit');
  expect(reloadedSource).not.toContain('Agent-disabled: await robot.move(3.0, 0.0, 0.0)');
  expect(pageErrors, pageErrors.join('\n\n')).toEqual([]);
});

test('ready physical MicroDuck registers only its bounded physical tool and removes it on profile changes', async ({ page }) => {
  test.setTimeout(180_000);
  await openReadyApp(page);
  await page.locator('#agentAccessToggle').click();
  await expect.poll(() => activeTools(page)).toEqual(OPENARM_TOOLS);
  await page.locator('#robotSelect').selectOption('microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 120_000 });
  await expect.poll(() => activeTools(page)).toEqual([...BASE_TOOLS, 'control_microduck_physical_simulation']);
  const task = await callTool(page, 'describe_robobuddy_task');
  expect(task).toMatchObject({ simulationMode: 'physical_mujoco', hardwareValidated: false });
  const schema = 'robobuddy.microduck.physical.v1';
  const advance = await callTool(page, 'control_microduck_physical_simulation', { schema_version: schema, command: 'advance', advance_seconds: 0.02 });
  expect(advance).toMatchObject({ ok: true, command: 'advance', executedTicks: 1 });
  const invalid = await callTool(page, 'control_microduck_physical_simulation', { schema_version: schema, command: 'set_command', request: { vx: null } });
  expect(invalid).toMatchObject({ ok: false, error: { code: 'INVALID_ARGUMENT' } });
  const cancelled = await callTool(page, 'control_microduck_physical_simulation', { schema_version: schema, command: 'advance', advance_seconds: 2 }, { aborted: true });
  expect(cancelled).toMatchObject({ ok: false, error: { code: 'OPERATION_CANCELLED' } });
  await page.locator('#robotSelect').selectOption('openarm');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
  await expect.poll(() => activeTools(page)).toEqual(OPENARM_TOOLS);
  expect(await page.evaluate(() => window.__webMcpRegistrations.filter(({ tool, signal }) => tool.name === 'control_microduck_physical_simulation' && !signal?.aborted).length)).toBe(0);
});

test('loading and failed MicroDuck workspaces keep only the six base tools', async ({ page }) => {
  await openReadyApp(page);
  await page.locator('#agentAccessToggle').click();
  await expect.poll(() => activeTools(page)).toEqual(OPENARM_TOOLS);
  await page.route('**/assets/microduck/generated/procedural-rig.json', async (route) => { await new Promise(resolve => setTimeout(resolve, 250)); await route.abort('failed'); });
  await page.locator('#robotSelect').selectOption('microduck');
  await expect(page.locator('#statusMessage')).toContainText('Loading MicroDuck MuJoCo physical');
  await expect.poll(() => activeTools(page)).toEqual(BASE_TOOLS);
  await expect(page.locator('#statusMessage')).toContainText('unavailable', { timeout: 60_000 });
  await expect.poll(() => activeTools(page)).toEqual(BASE_TOOLS);
  expect(await page.evaluate(() => window.__webMcpRegistrations.filter(({ tool, signal }) => tool.name === 'control_microduck_physical_simulation' && !signal?.aborted).length)).toBe(0);
});

test('partial WebMCP registration failure aborts the entire attempted group', async ({ page }) => {
  await installWebMcpMock(page, { failAt: 3 });
  await page.goto('/?ci=webmcp-partial-failure', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
  await page.locator('#agentAccessToggle').click();
  await expect(page.locator('#agentAccessControl')).toHaveAttribute('data-error', 'true');
  expect(await activeTools(page)).toEqual([]);
  expect(await page.evaluate(() => window.__webMcpRegistrations.every(({ signal }) => signal.aborted))).toBe(true);
});

test('unsupported browsers keep the IDE usable without registering WebMCP tools', async ({ page }) => {
  await page.goto('/?ci=webmcp-unsupported', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
  await expect(page.locator('#agentAccessControl')).toHaveAttribute('data-available', 'false');
  await expect(page.locator('#agentAccessToggle')).toBeDisabled();
  await expect(page.locator('#simCanvas')).toBeVisible();
});

test('full IDE OpenArm equipment and program tools share the real application lease and respect revocation', async ({ page }, testInfo) => {
  test.setTimeout(180_000);
  const errors = [];
  page.on('pageerror', error => errors.push(String(error.stack || error)));
  await openReadyApp(page);
  await page.locator('#agentAccessToggle').click();
  await expect.poll(() => activeTools(page)).toEqual(OPENARM_TOOLS);
  const before = await callTool(page, 'inspect_openarm_workcell');
  expect(before.ok).toBe(true);
  const staged = await callTool(page, 'manage_openarm_workcell', {
    schema_version: 'robobuddy.openarm.equipment.v1', command: 'stage', expected_scene_revision: before.authority.sceneRevision,
    equipment: [{ id: 'tray1', kind: 'tray', position_m: [.35, -.38, 1.005] }, { id: 'vial1', kind: 'vial', position_m: [.35, -.38, 1.08] }],
  });
  expect(staged.ok, JSON.stringify(staged)).toBe(true);
  const preview = await callTool(page, 'inspect_openarm_workcell');
  expect(preview.authority).toEqual(before.authority);
  expect(preview.bodies).toEqual(before.bodies);
  expect(preview.staged.equipment).toHaveLength(2);
  const applied = await callTool(page, 'manage_openarm_workcell', { schema_version: 'robobuddy.openarm.equipment.v1', command: 'apply', stage_id: staged.result.id, acknowledge_reset: true });
  expect(applied.ok, JSON.stringify(applied)).toBe(true);
  expect(applied.result.reset).toBe(true);
  const current = await callTool(page, 'inspect_openarm_workcell');
  expect(current.authority.sessionId).not.toBe(before.authority.sessionId);
  expect(current.equipment).toHaveLength(2);
  expect(current.bodies.flask).toBeTruthy();
  expect(current.bodies.beaker).toBeTruthy();
  const program = { schema_version: 'robobuddy.openarm.program.v1', expected_scene_revision: current.authority.sceneRevision, segments: [{ duration_seconds: 1, wait_for: { type: 'supported', object_id: 'lab_vial1', support_geom: 'lab_tray1_base', timeout_seconds: 1, dwell_seconds: .06 } }] };
  const result = await callTool(page, 'run_openarm_program', program);
  expect(result.ok, JSON.stringify(result)).toBe(true);
  expect(result.executionStatus).toBe('completed');
  expect(result.segments[0].conditionSatisfied).toBe(true);
  expect(await page.evaluate(() => window.__robobuddyCi.app.getExecutionState())).toBe('idle');
  const settled = await callTool(page, 'inspect_openarm_workcell');
  expect(settled.bodies.lab_vial1.positionM[2]).toBeLessThan(current.bodies.lab_vial1.positionM[2]);
  const stale = await callTool(page, 'run_openarm_program', { ...program, expected_scene_revision: before.authority.sceneRevision });
  expect(stale).toMatchObject({ ok: false, error: { code: 'INVALID_ARGUMENT' } });
  expect((await callTool(page, 'inspect_openarm_workcell')).authority).toEqual(settled.authority);
  await page.screenshot({ path: testInfo.outputPath('full-ide-agent-equipment.png') });
  await writeFile(testInfo.outputPath('full-ide-equipment-evidence.json'), JSON.stringify({ before: before.authority, applied, program: result, settled, stale }, null, 2));
  // Start a bounded program and revoke access through the actual UI while it
  // owns the real app. No mock execution state/facade is used here.
  const acquired = await page.evaluate(() => {
    const app = window.__robobuddyCi.app;
    const entry = window.__webMcpRegistrations.findLast(c => c.tool.name === 'run_openarm_program' && !c.signal?.aborted);
    window.__revokedOpenArmRun = entry.tool.execute({ schema_version: 'robobuddy.openarm.program.v1', expected_scene_revision: app.sim.getPhysicalAuthorityToken().sceneRevision, segments: Array.from({ length: 5 }, () => ({ duration_seconds: 10 })) }, { signal: new AbortController().signal });
    return app.getExecutionState();
  });
  expect(acquired).toBe('running');
  const competing = await callTool(page, 'control_openarm_simulation', { schema_version: 'robobuddy.openarm.physical.v1', command: 'advance', seconds: .1 });
  expect(competing).toMatchObject({ ok: false, error: { code: 'SIMULATION_BUSY' } });
  await page.locator('#agentAccessToggle').click();
  const revoked = await page.evaluate(() => window.__revokedOpenArmRun);
  expect(revoked.ok, JSON.stringify(revoked)).toBe(false);
  await expect.poll(() => page.evaluate(() => window.__robobuddyCi.app.getExecutionState())).toBe('idle');
  expect(await activeTools(page)).toEqual([]);
  const stoppedTime = await page.evaluate(() => window.__robobuddyCi.app.sim.getState().observation.simulationTimeSeconds);
  await page.waitForTimeout(150);
  expect(await page.evaluate(() => window.__robobuddyCi.app.sim.getState().observation.simulationTimeSeconds)).toBe(stoppedTime);
  expect(errors, errors.join('\n')).toEqual([]);
});
