import { test, expect } from '@playwright/test';

async function openControlDeckAudit(page, tag, { gamepad = false } = {}) {
  if (gamepad) {
    await page.addInitScript(() => {
      const buttons = Array.from({ length: 16 }, () => ({ pressed: false, touched: false, value: 0 }));
      const pad = { axes: [0, 0, 0, 0], buttons, connected: true, id: 'MicroDuck standard-browser audit pad', index: 0, mapping: 'standard', timestamp: 0 };
      const fixture = { connected: false, gamepad: pad };
      Object.defineProperty(window, '__microduckGamepadFixture', { configurable: true, value: fixture });
      Object.defineProperty(navigator, 'getGamepads', { configurable: true, value: () => fixture.connected ? [fixture.gamepad] : [] });
    });
  }

  await page.goto(`/?ci=control-deck-audit-${tag}`);
  await page.selectOption('#robotSelect', 'microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });

  const deck = page.locator('#microduckControlDeck');
  const capture = deck.locator('.md-capture-button');
  const state = () => page.evaluate(() => window.__robobuddyCi.app.sim.getState());
  const stateAt = (...path) => page.evaluate((segments) => segments.reduce((value, key) => value[key], window.__robobuddyCi.app.sim.getState()), path);
  const captureInput = async () => {
    if ((await capture.textContent())?.trim() !== 'Release simulator input') await capture.click();
    await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-capture', 'true');
  };
  const expand = async (text) => {
    const details = deck.locator('details').filter({ hasText: text }).first();
    if (!(await details.evaluate((element) => element.open))) await details.locator('summary').click();
    return details;
  };
  const clickRangeEdge = async (locator, edge) => {
    await locator.scrollIntoViewIfNeeded();
    const box = await locator.boundingBox();
    expect(box).not.toBeNull();
    await page.mouse.click(edge === 'min' ? box.x + 2 : box.x + box.width - 2, box.y + box.height / 2);
  };
  const expectLeaseExpiry = async (...path) => {
    await expect.poll(() => stateAt(...path), { timeout: 1_000, intervals: [50, 100, 150] }).toBe(0);
  };

  return { deck, capture, state, stateAt, captureInput, expand, clickRangeEdge, expectLeaseExpiry };
}

test('control-deck-audit STOP and Reset neutralize retained human input', async ({ page }) => {
  const { deck, capture, state, stateAt, captureInput, expand } = await openControlDeckAudit(page, 'safety');
  const initial = await state();
  const runtime = await expand('Runtime');

  await expect(capture).toHaveAttribute('aria-pressed', 'false');
  await expect(runtime.locator('[data-md-command="enable"]')).toBeDisabled();
  await expect(runtime.locator('[data-md-command="stop"]')).toBeEnabled();
  await expect(runtime.locator('[data-md-command="reset"]')).toBeEnabled();
  await expect(deck.locator('.md-capture')).toContainText('250 ms browser focus-safety lease');

  await captureInput();
  await runtime.locator('[data-md-command="enable"]').click();
  await expect.poll(() => stateAt('enabled')).toBe(true);

  await page.keyboard.down('w');
  await expect.poll(() => stateAt('movement', 'applied', 0)).toBeGreaterThan(0);
  await runtime.locator('[data-md-command="stop"]').click();
  await page.waitForTimeout(350);
  expect((await state()).movement.applied).toEqual([0, 0, 0]);
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-capture', 'false');
  await expect(capture).toContainText('Capture simulator input');
  await page.keyboard.up('w');
  expect((await state()).enabled).toBe(true);

  await captureInput();
  await page.keyboard.down('w');
  await expect.poll(() => stateAt('movement', 'applied', 0)).toBeGreaterThan(0);
  await runtime.locator('[data-md-command="reset"]').click();
  await page.waitForTimeout(350);
  const reset = await state();
  expect(reset.movement.applied).toEqual([0, 0, 0]);
  expect(reset).toMatchObject({ enabled: false, activePolicy: 'disabled', phase: 'up' });
  expect(reset.simulatedPose.position[0]).toBeCloseTo(initial.simulatedPose.position[0], 6);
  expect(reset.simulatedPose.position[1]).toBeCloseTo(initial.simulatedPose.position[1], 6);
  expect(reset.ball.position[0]).toBeCloseTo(initial.ball.position[0], 6);
  expect(reset.ball.position[1]).toBeCloseTo(initial.ball.position[1], 6);
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-capture', 'false');
  await page.keyboard.up('w');
});

test('control-deck-audit runtime movement pose and skills cover every command family', async ({ page }) => {
  test.setTimeout(120_000);
  const { deck, state, stateAt, captureInput, expand, clickRangeEdge, expectLeaseExpiry } = await openControlDeckAudit(page, 'runtime-pose-skills');
  const initial = await state();
  expect(initial).toMatchObject({ enabled: false, actuationEnabled: true, mode: 'walking', activePolicy: 'disabled', phase: 'up' });
  expect(initial.joints).toHaveLength(14);
  expect(await page.evaluate(() => {
    const snapshot = window.__robobuddyCi.app.sim.getState();
    return Object.isFrozen(snapshot) && Object.isFrozen(snapshot.joints) && Object.isFrozen(snapshot.movement);
  })).toBe(true);

  await captureInput();
  const runtime = await expand('Runtime');
  await runtime.locator('[data-md-command="enable"]').click();
  await expect.poll(() => stateAt('enabled')).toBe(true);
  await expect.poll(() => stateAt('activePolicy')).toBe('stand');

  const drive = await expand('Drive & intent');
  const movement = [
    { field: 'vx', index: 0, min: -0.3, max: 0.3 },
    { field: 'vy', index: 1, min: -0.3, max: 0.3 },
    { field: 'yaw', index: 2, min: -1.5, max: 1.5 },
  ];
  for (const item of movement) {
    const slider = drive.locator(`[data-md-range="move"][data-field="${item.field}"]`);
    for (const edge of ['max', 'min']) {
      const expected = item[edge];
      await slider.fill(String(expected));
      await expect.poll(() => stateAt('movement', 'applied', item.index), { timeout: 300, intervals: [10, 20, 40] }).toBeCloseTo(expected, 6);
      await expect(drive.locator('[data-md-value="movement.requested"]')).toContainText(expected.toFixed(2));
      await expectLeaseExpiry('movement', 'applied', item.index);
    }
  }

  const exerciseKeyIntent = async (key, index, expected) => {
    await page.locator('#simCanvas').focus();
    await page.keyboard.down(key);
    await expect.poll(() => stateAt('movement', 'applied', index)).toBeCloseTo(expected, 6);
    await page.keyboard.up(key);
    await expectLeaseExpiry('movement', 'applied', index);
  };
  for (const [key, index, expected] of [
    ['w', 0, 0.3], ['s', 0, -0.3], ['a', 1, 0.3], ['d', 1, -0.3], ['ArrowLeft', 2, 1.5], ['ArrowRight', 2, -1.5],
  ]) await exerciseKeyIntent(key, index, expected);
  const exerciseKeyboardSkill = async (key, phase, timeout = 5_000) => {
    await page.keyboard.press(key);
    await expect.poll(() => stateAt('phase')).toBe(phase);
    await expect.poll(() => stateAt('phase'), { timeout }).toBe('up');
  };
  await exerciseKeyboardSkill('q', 'kick_left');
  await exerciseKeyboardSkill('e', 'kick_right');
  await deck.locator('.md-key-layout').selectOption('zqsd');
  for (const [key, index, expected] of [
    ['z', 0, 0.3], ['s', 0, -0.3], ['q', 1, 0.3], ['d', 1, -0.3],
  ]) await exerciseKeyIntent(key, index, expected);
  await exerciseKeyboardSkill('j', 'kick_left');
  await exerciseKeyboardSkill('k', 'kick_right');
  await deck.locator('.md-key-layout').selectOption('wasd');
  await exerciseKeyboardSkill('g', 'ground_pick');
  await exerciseKeyboardSkill('r', 'roulade');
  await page.keyboard.press('y');
  await expect.poll(() => stateAt('phase')).toBe('sitting');
  await page.locator('#pauseBtn').click();
  await captureInput();
  await page.keyboard.press('y');
  await expect.poll(() => stateAt('phase')).toBe('rise');
  await page.locator('#pauseBtn').click();
  await expect.poll(() => stateAt('phase'), { timeout: 4_000 }).toBe('up');
  await captureInput();
  await page.keyboard.press('m');
  await expect.poll(() => stateAt('mode'), { timeout: 10_000 }).toBe('roller');
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-capture', 'false');
  await captureInput();
  await page.keyboard.press('m');
  await expect.poll(() => stateAt('mode'), { timeout: 10_000 }).toBe('walking');
  await captureInput();
  await page.keyboard.press(' ');
  await expect.poll(() => stateAt('enabled')).toBe(false);
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-capture', 'false');
  await captureInput();
  await runtime.locator('[data-md-command="enable"]').click();
  await expect.poll(() => stateAt('enabled')).toBe(true);
  for (const camera of ['chase', 'head', 'orbit']) {
    await page.keyboard.press('c');
    await expect.poll(() => stateAt('virtualCamera', 'mode')).toBe(camera);
  }

  await page.waitForTimeout(320);
  await runtime.locator('[data-md-command="enable"]').click();
  await expect.poll(() => stateAt('enabled')).toBe(false);
  const heldA = await state();
  await page.waitForTimeout(250);
  const heldB = await state();
  expect(heldB.time).toBeCloseTo(heldA.time, 8);
  expect(heldB.joints).toEqual(heldA.joints);
  await runtime.locator('[data-md-command="init"]').click();
  await expect.poll(() => stateAt('phase')).toBe('initializing');
  await expect.poll(() => stateAt('phase'), { timeout: 5_000 }).toBe('up');
  await runtime.locator('[data-md-command="relax"]').click();
  await expect.poll(() => stateAt('actuationEnabled')).toBe(false);
  await runtime.locator('[data-md-command="enable"]').click();
  await expect.poll(() => stateAt('phase')).toBe('initializing');
  await expect.poll(() => stateAt('enabled'), { timeout: 5_000 }).toBe(true);

  const pose = await expand('Head, look, body & mouth');
  const headRanges = pose.locator('[data-md-range="head"]');
  for (let index = 0; index < 4; index += 1) {
    await clickRangeEdge(headRanges.nth(index), 'max');
    await expect.poll(() => stateAt('head', index), { timeout: 300, intervals: [10, 20, 40] }).toBeGreaterThan(0);
    await clickRangeEdge(headRanges.nth(index), 'min');
    await expect.poll(() => stateAt('head', index), { timeout: 300, intervals: [10, 20, 40] }).toBeLessThan(0);
    await expectLeaseExpiry('head', index);
  }
  const look = pose.locator('[data-md-number="look"]');
  await look.nth(0).fill('0.05');
  await look.nth(1).fill('0.5');
  await look.nth(2).fill('0.6');
  await pose.locator('[data-md-command="look"]').click();
  expect((await state()).head.some((value) => Math.abs(value) > 0.01)).toBe(true);
  await look.nth(0).fill('1');
  await look.nth(1).fill('-0.5');
  await look.nth(2).fill('-0.25');
  await pose.locator('[data-md-command="look"]').click();
  expect((await state()).head.every(Number.isFinite)).toBe(true);

  const bodyRanges = pose.locator('[data-md-range="pose"]');
  for (let index = 0; index < 3; index += 1) {
    const stateIndex = index + 2;
    await clickRangeEdge(bodyRanges.nth(index), 'max');
    await expect.poll(() => stateAt('body', stateIndex), { timeout: 300, intervals: [10, 20, 40] }).toBeGreaterThan(0);
    await clickRangeEdge(bodyRanges.nth(index), 'min');
    await expect.poll(() => stateAt('body', stateIndex), { timeout: 300, intervals: [10, 20, 40] }).toBeLessThan(0);
    await expectLeaseExpiry('body', stateIndex);
  }
  const mouth = pose.locator('[data-md-range="mouth"]');
  await clickRangeEdge(mouth, 'max');
  await expect.poll(() => stateAt('mouth'), { timeout: 300, intervals: [10, 20, 40] }).toBeGreaterThan(0.9);
  await clickRangeEdge(mouth, 'min');
  await expect.poll(() => stateAt('mouth')).toBe(0);

  const skills = await expand('Skills');
  const exerciseSkill = async (skill, phase, timeout = 4_000) => {
    await skills.locator(`[data-md-command="do"][data-skill="${skill}"]`).click();
    await expect.poll(() => stateAt('phase')).toBe(phase);
    await expect.poll(() => stateAt('phase'), { timeout }).toBe('up');
  };
  await exerciseSkill('ground_pick', 'ground_pick', 5_000);
  await exerciseSkill('kick_left', 'kick_left');
  await exerciseSkill('kick_right', 'kick_right');
  await exerciseSkill('roulade', 'roulade');
  await skills.locator('[data-md-command="do"][data-skill="sit_toggle"]').click();
  await expect.poll(() => stateAt('phase')).toBe('sitting');
  await page.locator('#pauseBtn').click();
  await captureInput();
  await skills.locator('[data-md-command="do"][data-skill="sit_toggle"]').click();
  await expect.poll(() => stateAt('phase')).toBe('rise');
  await page.locator('#pauseBtn').click();
  await expect.poll(() => stateAt('phase'), { timeout: 4_000 }).toBe('up');

  await captureInput();
  await runtime.locator('[data-md-command="set_mode"][data-mode="roller"]').click();
  await expect.poll(() => stateAt('mode'), { timeout: 10_000 }).toBe('roller');
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-capture', 'false');
  await expect(drive.locator('[data-md-range="move"][data-field="vx"]')).toHaveAttribute('min', '-0.5');
  await expect(drive.locator('[data-md-range="move"][data-field="vx"]')).toHaveAttribute('max', '0.6');
  await expect(drive.locator('[data-md-range="move"][data-field="vy"]')).toBeDisabled();
  await expect(drive.locator('[data-md-range="move"][data-field="yaw"]')).toHaveAttribute('min', '-0.3');
  await expect(drive.locator('[data-md-range="move"][data-field="yaw"]')).toHaveAttribute('max', '0.3');
  await captureInput();
  await drive.locator('[data-md-range="move"][data-field="vx"]').fill('0.6');
  await expect.poll(() => stateAt('movement', 'applied', 0), { timeout: 300, intervals: [10, 20, 40] }).toBeCloseTo(0.6, 6);
  await skills.locator('[data-md-command="do"][data-skill="ground_pick"]').click();
  await expect.poll(() => stateAt('phase')).toBe('roller_crouch');
  await expect.poll(() => stateAt('phase'), { timeout: 5_000 }).toBe('up');
  await runtime.locator('[data-md-command="set_mode"][data-mode="walking"]').click();
  await expect.poll(() => stateAt('mode'), { timeout: 10_000 }).toBe('walking');
});

test('control-deck-audit audio gating held audio and cleanup are complete', async ({ page }) => {
  const { state, stateAt, captureInput, expand } = await openControlDeckAudit(page, 'audio');
  await captureInput();
  const runtime = await expand('Runtime');
  const audio = await expand('Generated local audio');
  await audio.locator('[data-md-command="sound"][data-tag="chirp"]').click();
  await expect(page.locator('#statusMessage')).toContainText('AUDIO_LOCKED');
  await audio.locator('.md-audio-unlock').click();
  await expect(audio.locator('[data-md-value="audio.status"]')).toContainText('UNLOCKED');

  for (const tag of ['alarm', 'greet', 'inquire', 'peck', 'chirp', 'coo']) {
    await audio.locator(`[data-md-command="sound"][data-tag="${tag}"]`).click();
    await expect.poll(() => stateAt('audio', 'sound')).toBe(tag);
  }
  const wheee = audio.locator('[data-md-command="sound"][data-tag="wheee"]');
  await wheee.scrollIntoViewIfNeeded();
  const box = await wheee.boundingBox();
  expect(box).not.toBeNull();
  await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
  await page.mouse.down();
  await expect.poll(() => stateAt('audio', 'sound')).toBe('wheee');
  await page.mouse.up();
  await expect.poll(() => stateAt('audio', 'sound')).toBeNull();

  const presentation = await expand('Presentation & modeled peripherals');
  const tof = presentation.locator('[data-md-range="set_tof_stimulus"]');
  await tof.fill('0.1');
  await audio.locator('[data-md-toggle="theremin"]').check();
  await expect.poll(() => stateAt('audio', 'theremin')).toBe(true);
  const nearFrequency = (await state()).audio.thereminFrequencyHz;
  await tof.fill('0.7');
  await expect.poll(() => stateAt('audio', 'thereminFrequencyHz')).not.toBe(nearFrequency);
  await audio.locator('[data-md-toggle="theremin"]').uncheck();

  const piece = audio.locator('[data-md-chorale="piece"]');
  const voices = audio.locator('[data-md-chorale="voices"]');
  await expect(voices).toHaveAttribute('min', '1');
  await expect(voices).toHaveAttribute('max', '4');
  await piece.selectOption('wistful');
  await voices.fill('1');
  await audio.locator('[data-md-chorale="start"]').click();
  await expect.poll(() => stateAt('audio', 'chorale')).toBe(true);
  expect((await state()).audio).toMatchObject({ piece: 'wistful', voices: 1 });
  await audio.locator('[data-md-chorale="stop"]').click();
  await piece.selectOption('duck_strut');
  await voices.fill('4');
  await audio.locator('[data-md-chorale="start"]').click();
  await audio.locator('[data-md-toggle="theremin"]').check();
  await runtime.locator('[data-md-command="stop"]').click();
  await page.waitForTimeout(350);
  expect((await state()).audio).toMatchObject({ sound: null, theremin: false, chorale: false });
  await expect(audio.locator('[data-md-toggle="theremin"]')).not.toBeChecked();
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-capture', 'false');

  await captureInput();
  await audio.locator('[data-md-chorale="start"]').click();
  await expect.poll(() => stateAt('audio', 'chorale')).toBe(true);
  await runtime.locator('[data-md-command="reset"]').click();
  expect((await state()).audio).toMatchObject({ sound: null, theremin: false, chorale: false });
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-capture', 'false');

  await captureInput();
  await audio.locator('[data-md-chorale="start"]').click();
  await page.selectOption('#robotSelect', 'openarm');
  await expect(page.locator('#statusMessage')).toContainText('Ready');
  await page.selectOption('#robotSelect', 'microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready');
  expect((await state()).audio).toMatchObject({ unlocked: false, sound: null, theremin: false, chorale: false });
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-capture', 'false');
});

test('control-deck-audit presentation peripherals gamepad accessibility and reachability are complete', async ({ page }) => {
  const { deck, state, stateAt, captureInput, expand } = await openControlDeckAudit(page, 'presentation', { gamepad: true });
  const initial = await state();
  const runtime = await expand('Runtime');
  const presentation = await expand('Presentation & modeled peripherals');
  await captureInput();

  const physics = (current) => ({ time: current.time, pose: current.simulatedPose, joints: current.joints, targets: current.targets, contacts: current.contacts, ball: current.ball });
  await page.locator('#pauseBtn').click();
  await captureInput();
  const beforeColors = physics(await state());
  for (const color of ['cream', 'graphite', 'lavender', 'sky']) {
    await presentation.locator(`[data-md-command="set_color"][data-color="${color}"]`).click();
    await expect.poll(() => stateAt('color')).toBe(color);
    expect(physics(await state())).toEqual(beforeColors);
  }
  await page.locator('#pauseBtn').click();

  for (const camera of ['orbit', 'chase', 'head']) {
    await presentation.locator(`[data-md-command="set_camera"][data-camera="${camera}"]`).click();
    await expect.poll(() => stateAt('virtualCamera', 'mode')).toBe(camera);
    await page.locator('#fitBtn').click();
    expect((await state()).virtualCamera.mode).toBe(camera);
  }
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-camera-name', 'Head POV');
  await presentation.locator('[data-md-command="set_camera"][data-camera="orbit"]').click();

  await captureInput();
  await presentation.locator('[data-md-command="spawn_ball"]').click();
  const spawned = (await state()).ball.position;
  expect(spawned[0]).toBeCloseTo(0.28, 6);
  expect(spawned[1]).toBeCloseTo(0, 6);
  expect(spawned[2]).toBeCloseTo(0.035, 2);
  const leftBefore = (await state()).simulatedPose.quaternion;
  await presentation.locator('[data-md-perturb="left"]').click();
  await expect.poll(async () => (await state()).simulatedPose.quaternion).not.toEqual(leftBefore);
  await runtime.locator('[data-md-command="reset"]').click();
  expect((await state()).ball.position[0]).toBeCloseTo(initial.ball.position[0], 6);
  await captureInput();
  const rightBefore = (await state()).simulatedPose.quaternion;
  await presentation.locator('[data-md-perturb="right"]').click();
  await expect.poll(async () => (await state()).simulatedPose.quaternion).not.toEqual(rightBefore);
  await runtime.locator('[data-md-command="reset"]').click();
  expect((await state()).ball.position[0]).toBeCloseTo(initial.ball.position[0], 6);

  await captureInput();
  await presentation.locator('[data-md-tof-source="raycast"]').check();
  await expect.poll(() => stateAt('tof', 'source')).toBe('raycast');
  expect((await state()).tof.valuesM).toHaveLength(64);
  await expect(deck.locator('.md-telemetry')).toContainText('Trunk IMU (modeled)');
  await expect(deck.locator('.md-telemetry')).toContainText('Head IMU (modeled)');
  await expect(deck.locator('.md-telemetry')).toContainText('8×8');

  const audio = await expand('Generated local audio');
  await audio.locator('.md-audio-unlock').click();
  await expect(audio.locator('[data-md-value="audio.status"]')).toContainText('UNLOCKED');
  await page.evaluate(() => {
    const fixture = window.__microduckGamepadFixture;
    fixture.connected = true;
    fixture.gamepad.axes.fill(0);
    window.dispatchEvent(new Event('gamepadconnected'));
  });
  const setGamepadButton = (index, pressed, value = pressed ? 1 : 0) => page.evaluate(({ index: buttonIndex, pressed: isPressed, value: buttonValue }) => {
    Object.assign(window.__microduckGamepadFixture.gamepad.buttons[buttonIndex], { pressed: isPressed, touched: isPressed, value: buttonValue });
  }, { index, pressed, value });
  const pulseGamepadButton = async (index, holdMs = 100) => {
    await setGamepadButton(index, true);
    await page.waitForTimeout(holdMs);
    await setGamepadButton(index, false);
    await page.waitForTimeout(100);
  };
  await pulseGamepadButton(9);
  await expect.poll(() => stateAt('enabled')).toBe(true);
  await page.evaluate(() => { window.__microduckGamepadFixture.gamepad.axes.splice(0, 4, 0.5, -1, -0.5, 0.25); });
  await expect.poll(() => stateAt('movement', 'applied', 0)).toBeCloseTo(0.3, 6);
  await expect.poll(() => stateAt('movement', 'applied', 1)).toBeCloseTo(-0.15, 6);
  await expect.poll(() => stateAt('movement', 'applied', 2)).toBeCloseTo(0.75, 6);
  await pulseGamepadButton(3);
  await expect.poll(() => stateAt('head')).not.toEqual([0, 0, 0, 0]);
  await pulseGamepadButton(3);
  await expect.poll(() => stateAt('movement', 'applied', 0)).toBeCloseTo(0.3, 6);
  await pulseGamepadButton(1);
  await expect.poll(() => stateAt('body')).not.toEqual([0, 0, 0, 0, 0, 0]);
  await pulseGamepadButton(1);
  await page.evaluate(() => { window.__microduckGamepadFixture.gamepad.axes.fill(0); });

  const exerciseGamepadSkill = async (buttonIndex, phase, timeout = 5_000) => {
    await setGamepadButton(buttonIndex, true);
    await expect.poll(() => stateAt('phase')).toBe(phase);
    await setGamepadButton(buttonIndex, false);
    await expect.poll(() => stateAt('phase'), { timeout }).toBe('up');
    await page.waitForTimeout(100);
  };
  await exerciseGamepadSkill(0, 'ground_pick');
  await exerciseGamepadSkill(4, 'kick_left');
  await exerciseGamepadSkill(5, 'kick_right');
  await exerciseGamepadSkill(2, 'roulade');
  await pulseGamepadButton(13);
  await expect.poll(() => stateAt('phase')).toBe('sitting');
  await setGamepadButton(13, true);
  await expect.poll(() => stateAt('phase')).toBe('rise');
  await setGamepadButton(13, false);
  await expect.poll(() => stateAt('phase'), { timeout: 4_000 }).toBe('up');

  await setGamepadButton(7, true);
  await expect.poll(() => stateAt('audio', 'sound')).toBe('chirp');
  await expect.poll(() => stateAt('mouth')).toBeGreaterThan(0.9);
  await setGamepadButton(7, false);
  await expect.poll(() => stateAt('mouth')).toBe(0);
  await setGamepadButton(6, true);
  await expect.poll(() => stateAt('audio', 'sound')).toBe('wheee');
  await expect.poll(() => stateAt('mouth')).toBeGreaterThan(0.9);
  await setGamepadButton(6, false);
  await expect.poll(() => stateAt('audio', 'sound')).toBeNull();
  await expect.poll(() => stateAt('mouth')).toBe(0);

  await setGamepadButton(12, true);
  await expect.poll(() => stateAt('mode'), { timeout: 5_000 }).toBe('roller');
  await setGamepadButton(12, false);
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-capture', 'false');
  await captureInput();
  await setGamepadButton(12, true);
  await expect.poll(() => stateAt('mode'), { timeout: 5_000 }).toBe('walking');
  await setGamepadButton(12, false);
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-capture', 'false');

  await page.evaluate(() => {
    const fixture = window.__microduckGamepadFixture;
    fixture.connected = false;
    fixture.gamepad.axes.fill(0);
    window.dispatchEvent(new Event('gamepaddisconnected'));
  });
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-capture', 'false');
  await page.waitForTimeout(350);
  expect((await state()).movement.applied).toEqual([0, 0, 0]);
  expect((await state()).head).toEqual([0, 0, 0, 0]);
  expect((await state()).body).toEqual([0, 0, 0, 0, 0, 0]);

  for (const details of await deck.locator('details').all()) {
    if (!(await details.evaluate((element) => element.open))) await details.locator('summary').click();
  }
  const controls = deck.locator('button, input, select');
  for (let index = 0; index < await controls.count(); index += 1) await expect(controls.nth(index)).toHaveAccessibleName(/\S/);
  const body = deck.locator('.md-deck-body');
  const lastControl = presentation.locator('[data-md-tof-source="raycast"]');
  await lastControl.scrollIntoViewIfNeeded();
  const [bodyBox, controlBox] = await Promise.all([body.boundingBox(), lastControl.boundingBox()]);
  expect(controlBox.y).toBeGreaterThanOrEqual(bodyBox.y);
  expect(controlBox.y + controlBox.height).toBeLessThanOrEqual(bodyBox.y + bodyBox.height + 1);
  await deck.locator('.md-collapse').click();
  await expect(deck.locator('.md-collapse')).toHaveAttribute('aria-expanded', 'false');
  await deck.locator('.md-collapse').click();
  await expect(body).toBeVisible();

  await captureInput();
  await page.locator('#robotSelect').focus();
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-capture', 'false');
  expect((await state()).movement.applied).toEqual([0, 0, 0]);
});
