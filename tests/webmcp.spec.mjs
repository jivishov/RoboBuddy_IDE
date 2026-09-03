import { expect, test } from '@playwright/test';

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

test('explicit human Agent Assist registers a bounded, cancellation-aware RoboBuddy WebMCP surface', async ({ page }) => {
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await openReadyApp(page);

  const control = page.locator('#agentAccessControl');
  const indicator = control.locator('.agent-access-dot');
  await expect(control).toHaveAttribute('data-access', 'off');
  await expect(control).toHaveAttribute('data-available', 'true');
  await expect(control).toHaveAttribute('data-tools', 'disabled');
  await expect(indicator).toHaveCSS('background-color', 'rgb(220, 90, 100)');
  await expect(indicator).toHaveCSS('animation-name', 'none');
  expect(await page.evaluate(() => window.__webMcpRegistrations.length)).toBe(0);

  await page.locator('[data-agent-access="assist"]').evaluate((button) => button.click());
  await expect(control).toHaveAttribute('data-access', 'off');
  expect(await page.evaluate(() => window.__webMcpRegistrations.length)).toBe(0);

  await page.locator('[data-agent-access="assist"]').click();
  await expect(control).toHaveAttribute('data-access', 'assist');
  await expect(control).toHaveAttribute('data-tools', 'enabled');
  await expect(indicator).toHaveCSS('background-color', 'rgb(70, 209, 124)');
  await expect(indicator).toHaveCSS('animation-name', 'agent-assist-blink');

  const registered = await page.evaluate(() => window.__webMcpRegistrations.filter(({ signal }) => !signal?.aborted).map(({ tool }) => ({
    name: tool.name,
    annotations: tool.annotations,
    inputSchema: tool.inputSchema,
  })));
  expect(registered.map(({ name }) => name)).toEqual([
    'describe_robobuddy_task',
    'read_robobuddy_workspace',
    'inspect_robobuddy_simulation',
    'focus_robobuddy_workspace',
    'run_robobuddy_program',
    'draft_robobuddy_cooperative_edit',
  ]);
  expect(registered.every(({ annotations }) => annotations.untrustedContentHint)).toBe(true);
  expect(registered.slice(0, 3).every(({ annotations }) => annotations.readOnlyHint)).toBe(true);
  expect(registered.slice(3).every(({ annotations }) => !annotations.readOnlyHint)).toBe(true);
  expect(registered.every(({ inputSchema }) => inputSchema.type === 'object' && inputSchema.additionalProperties === false)).toBe(true);

  const task = await callTool(page, 'describe_robobuddy_task');
  expect(task).toMatchObject({
    taskId: 'openarm-04-filtration-workcell',
    simulationMode: 'source_plant',
    sourcePlantAvailable: true,
    hardwareValidated: false,
  });
  expect(JSON.stringify(task)).not.toContain('referenceActions');

  const source = await callTool(page, 'read_robobuddy_workspace', { file: 'trajectories.py' });
  expect(source).toMatchObject({
    file: 'trajectories.py',
    startLine: 1,
    contentClassification: 'untrusted_user_authored_source',
  });
  expect(source.content.length).toBeLessThanOrEqual(1_200);
  expect(source.endLine - source.startLine + 1).toBeLessThanOrEqual(32);

  const state = await callTool(page, 'inspect_robobuddy_simulation');
  expect(state).toMatchObject({
    executionState: 'idle',
    simulationMode: 'source_plant',
    stateKind: 'modeled_source_plant',
    untrustedContent: true,
  });

  const focus = await callTool(page, 'focus_robobuddy_workspace', { file: 'main.py', line: 1 });
  expect(focus).toMatchObject({ focused: true, file: 'main.py', line: 1, sourceChanged: false });
  await expect(page.locator('#statusMessage')).toContainText('Agent focused main.py:1');

  await page.evaluate(() => window.__robobuddyCi.app.editor.cm.setValue('async def demo_move():\n    await robot.move(3.0, 0.0, 0.0)\n'));
  const temporaryEdit = await callTool(page, 'draft_robobuddy_cooperative_edit', {
    file: 'main.py',
    start_line: 2,
    end_line: 2,
    expected_source: '    await robot.move(3.0, 0.0, 0.0)',
    replacement_code: '    await robot.move(0.30, 0.0, 0.0)',
    explanation: 'Use the bounded teaching speed for a visible, controlled move.',
  });
  expect(temporaryEdit).toMatchObject({
    sourceChanged: true,
    temporary: true,
    persistence: 'not_saved_refresh_reloads_workspace',
    file: 'main.py',
    disabledOriginal: true,
    startLine: 2,
    endLine: 2,
    workingStartLine: 7,
  });
  const draftedSource = await page.evaluate(() => window.__robobuddyCi.app.editor.cm.getValue());
  expect(draftedSource).toContain('    # Agent-disabled: await robot.move(3.0, 0.0, 0.0)');
  expect(draftedSource).toContain('    # Explanation: Use the bounded teaching speed for a visible, controlled move.');
  expect(draftedSource).toContain('\n    await robot.move(0.30, 0.0, 0.0)');
  expect(draftedSource).not.toContain('\n        await robot.move(0.30, 0.0, 0.0)');
  await expect(page.locator('#statusMessage')).toContainText('temporary cooperative edit');
  expect(await page.locator('#dirtyDot').isHidden()).toBe(false);

  const staleEdit = await callTool(page, 'draft_robobuddy_cooperative_edit', {
    file: 'main.py',
    start_line: 2,
    end_line: 2,
    expected_source: '    await robot.move(3.0, 0.0, 0.0)',
    replacement_code: '    await robot.move(0.20, 0.0, 0.0)',
    explanation: 'This must not overwrite a changed draft.',
  });
  expect(staleEdit).toMatchObject({ ok: false, error: { code: 'SOURCE_MISMATCH', retryable: true } });
  expect(await page.evaluate(() => window.__robobuddyCi.app.editor.cm.getValue())).toBe(draftedSource);

  await page.evaluate(() => window.__robobuddyCi.app.editor.cm.setValue('print("webmcp run smoke")\n'));
  const run = await callTool(page, 'run_robobuddy_program');
  expect(run).toMatchObject({ completed: true, simulation: { executionState: 'idle', status: 'Run complete' } });

  const cancelled = await callTool(page, 'run_robobuddy_program', {}, { aborted: true });
  expect(cancelled).toMatchObject({ ok: false, error: { code: 'OPERATION_CANCELLED', retryable: true } });

  await page.locator('[data-agent-access="off"]').click();
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

test('ready MicroDuck adds one strict bounded control tool and removes it across profile changes', async ({ page }) => {
  await openReadyApp(page);
  await page.locator('[data-agent-access="assist"]').click();
  await expect.poll(() => activeTools(page)).toEqual([
    'describe_robobuddy_task',
    'read_robobuddy_workspace',
    'inspect_robobuddy_simulation',
    'focus_robobuddy_workspace',
    'run_robobuddy_program',
    'draft_robobuddy_cooperative_edit',
  ]);

  await page.locator('#robotSelect').selectOption('microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
  await expect.poll(() => activeTools(page)).toEqual([
    'describe_robobuddy_task',
    'read_robobuddy_workspace',
    'inspect_robobuddy_simulation',
    'focus_robobuddy_workspace',
    'run_robobuddy_program',
    'draft_robobuddy_cooperative_edit',
    'control_microduck_simulation',
    'manage_microduck_visual_cues',
  ]);

  const controlSurface = await page.evaluate(() => {
    const entry = window.__webMcpRegistrations.findLast(({ tool, signal }) => tool.name === 'control_microduck_simulation' && !signal?.aborted);
    return { schema: entry.tool.inputSchema, description: entry.tool.description };
  });
  const commands = [...new Set(controlSurface.schema.oneOf.map((branch) => branch.properties.command.const))].sort();
  expect(commands).toEqual([
    'chorale', 'do', 'enable', 'get_mode', 'get_state', 'head', 'init', 'look', 'mouth', 'move', 'pose',
    'relax', 'reset', 'set_camera', 'set_color', 'set_mode', 'set_tof_stimulus', 'sound', 'spawn_ball', 'stop', 'theremin',
  ]);
  expect(controlSurface.schema.oneOf).toHaveLength(25);
  expect(controlSurface.schema.oneOf.every((branch) => branch.additionalProperties === false && branch.required.includes('command'))).toBe(true);
  expect(commands).not.toEqual(expect.arrayContaining(['write', 'apply', 'export', 'publish', 'hardware', 'network', 'shutdown', 'ble', 'multiplayer', 'hidden']));
  expect(controlSurface.description).toContain('approximate browser dynamics only');

  const task = await callTool(page, 'describe_robobuddy_task');
  expect(task).toMatchObject({ simulationMode: 'policy_sim', stateKind: 'browser_policy_sim', sourcePlantAvailable: false, policySimulationAvailable: true, hardwareValidated: false });
  const state = await callTool(page, 'control_microduck_simulation', { command: 'get_state' });
  expect(state).toMatchObject({ ok: true, command: 'get_state', completed: true, state: { simulationMode: 'policy_sim', stateKind: 'browser_policy_sim', hardwareValidated: false } });
  expect(JSON.stringify(state).length).toBeLessThan(32_000);
  expect(JSON.stringify(state)).not.toMatch(/file_id|sha256|referenceActions|localPath/i);

  const visualCueSurface = await page.evaluate(() => {
    const entry = window.__webMcpRegistrations.findLast(({ tool, signal }) => tool.name === 'manage_microduck_visual_cues' && !signal?.aborted);
    return { schema: entry.tool.inputSchema, description: entry.tool.description };
  });
  expect(visualCueSurface.schema.oneOf).toHaveLength(4);
  expect(visualCueSurface.schema.oneOf.every((branch) => branch.additionalProperties === false)).toBe(true);
  expect(visualCueSurface.description).toContain('never executes caller code');
  const initialCues = await callTool(page, 'manage_microduck_visual_cues', { operation: 'list' });
  expect(initialCues).toEqual({ ok: true, operation: 'list', cues: [], cueCount: 0 });
  const addedLabel = await callTool(page, 'manage_microduck_visual_cues', { operation: 'upsert', cue: { id: 'pose-note', kind: 'label', text: 'modeled pose', anchor: 'duck', offset_m: [0, 0, 0.18], color: '#5ed6bc' } });
  expect(addedLabel).toMatchObject({ ok: true, operation: 'upsert', created: true, cue: { id: 'pose-note', kind: 'label', anchor: 'duck' }, cueCount: 1 });
  const addedRuler = await callTool(page, 'manage_microduck_visual_cues', { operation: 'upsert', cue: { id: 'reference-span', kind: 'ruler', start: [0, 0, 0.02], end: [1.2, 0, 0.02], title: 'reference', color: '#ff9f7a' } });
  expect(addedRuler).toMatchObject({ ok: true, operation: 'upsert', created: true, cue: { id: 'reference-span', kind: 'ruler' }, cueCount: 2 });
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-visual-cue-count', '2');
  const invalidCue = await callTool(page, 'manage_microduck_visual_cues', { operation: 'upsert', cue: { id: 'invalid', kind: 'line', start: [0, 0, 0], end: [0, 0, 0] } });
  expect(invalidCue).toMatchObject({ ok: false, error: { code: 'INVALID_ARGUMENT' } });
  const removedCue = await callTool(page, 'manage_microduck_visual_cues', { operation: 'remove', id: 'pose-note' });
  expect(removedCue).toEqual({ ok: true, operation: 'remove', removed: true, id: 'pose-note', cueCount: 1 });
  const clearedCues = await callTool(page, 'manage_microduck_visual_cues', { operation: 'clear' });
  expect(clearedCues).toEqual({ ok: true, operation: 'clear', removed: 1, cueCount: 0 });

  const invalidDuration = await callTool(page, 'control_microduck_simulation', { command: 'move', vx: 0.1 });
  expect(invalidDuration).toMatchObject({ ok: false, error: { code: 'INVALID_ARGUMENT' } });
  const moved = await callTool(page, 'control_microduck_simulation', { command: 'move', vx: 0.1, duration_ms: 20 });
  expect(moved).toMatchObject({ ok: true, command: 'move', completed: true, state: { movement: { applied: [0, 0, 0] } } });
  const cancelledMove = await callTool(page, 'control_microduck_simulation', { command: 'move', vx: 0.1, duration_ms: 5000 }, { abortAfterMs: 40 });
  expect(cancelledMove).toMatchObject({ ok: false, error: { code: 'OPERATION_CANCELLED' } });
  const afterCancellation = await callTool(page, 'control_microduck_simulation', { command: 'get_state' });
  expect(afterCancellation).toMatchObject({ ok: true, state: { movement: { applied: [0, 0, 0] } } });
  const manuallyStoppedInit = await page.evaluate(async () => {
    const entry = window.__webMcpRegistrations.findLast(({ tool, signal }) => tool.name === 'control_microduck_simulation' && !signal?.aborted);
    setTimeout(() => window.__robobuddyCi.app.stop(), 40);
    return entry.tool.execute({ command: 'init' }, { signal: new AbortController().signal });
  });
  expect(manuallyStoppedInit).toMatchObject({ ok: false, error: { code: 'OPERATION_CANCELLED' } });
  const afterManualStop = await callTool(page, 'control_microduck_simulation', { command: 'get_state' });
  expect(afterManualStop).toMatchObject({ ok: true, command: 'get_state', completed: true });
  const audioLocked = await callTool(page, 'control_microduck_simulation', { command: 'sound', tag: 'chirp' });
  expect(audioLocked).toMatchObject({ ok: false, error: { code: 'AUDIO_LOCKED' } });

  await page.locator('#robotSelect').selectOption('openarm');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
  await expect.poll(() => activeTools(page)).toHaveLength(6);
  expect(await page.evaluate(() => window.__webMcpRegistrations.filter(({ tool, signal }) => tool.name === 'control_microduck_simulation' && !signal?.aborted).length)).toBe(0);
});

test('loading and failed MicroDuck workspaces keep only the six base tools', async ({ page }) => {
  await openReadyApp(page);
  await page.locator('[data-agent-access="assist"]').click();
  await expect.poll(() => activeTools(page)).toHaveLength(6);
  await page.route('**/assets/microduck/generated/procedural-rig.json', async (route) => {
    await new Promise((resolve) => setTimeout(resolve, 250));
    await route.abort('failed');
  });
  await page.locator('#robotSelect').selectOption('microduck');
  await expect(page.locator('#statusMessage')).toContainText('Loading local MicroDuck');
  await expect.poll(() => activeTools(page)).toHaveLength(6);
  await expect(page.locator('#statusMessage')).toContainText('unavailable', { timeout: 60_000 });
  await expect.poll(() => activeTools(page)).toHaveLength(6);
  expect(await page.evaluate(() => window.__webMcpRegistrations.filter(({ tool, signal }) => tool.name === 'control_microduck_simulation' && !signal?.aborted).length)).toBe(0);
});

test('partial WebMCP registration failure aborts the entire attempted group', async ({ page }) => {
  await installWebMcpMock(page, { failAt: 3 });
  await page.goto('/?ci=webmcp-partial-failure', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
  await page.locator('[data-agent-access="assist"]').click();
  await expect(page.locator('#agentAccessControl')).toHaveAttribute('data-error', 'true');
  expect(await activeTools(page)).toEqual([]);
  expect(await page.evaluate(() => window.__webMcpRegistrations.every(({ signal }) => signal.aborted))).toBe(true);
});

test('unsupported browsers keep the IDE usable without registering WebMCP tools', async ({ page }) => {
  await page.goto('/?ci=webmcp-unsupported', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
  await expect(page.locator('#agentAccessControl')).toHaveAttribute('data-available', 'false');
  await expect(page.locator('[data-agent-access="assist"]')).toBeDisabled();
  await expect(page.locator('#simCanvas')).toBeVisible();
});
