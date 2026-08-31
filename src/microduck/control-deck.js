import { MICRODUCK_COMMANDS } from './command-catalog.js';
import { KEY_LAYOUTS, MicroDuckInputController } from './input-controller.js';

const label = (value) => String(value).replaceAll('_', ' ').replace(/\b\w/g, (letter) => letter.toUpperCase());
const fieldMarkup = (command, name, spec, value = 0) => `<label class="md-field"><span>${spec.label} <small>${spec.unit}</small></span><output data-output-for="${command}.${name}">${Number(value).toFixed(3)}</output><input type="range" aria-label="${spec.label} (${spec.unit})" data-md-range="${command}" data-field="${name}" min="${spec.range[0]}" max="${spec.range[1]}" step="${spec.step}" value="${value}"></label>`;
const buttonGroup = (command, values, key = 'value') => values.map((value) => `<button type="button" data-md-command="${command}" data-${key}="${value}">${label(value)}</button>`).join('');

export class MicroDuckControlDeck {
  constructor(root, { simulator, canvas, onStatus = () => {}, onState = () => {} } = {}) {
    this.root = root;
    this.simulator = simulator;
    this.canvas = canvas;
    this.onStatus = onStatus;
    this.onState = onState;
    this.active = false;
    this.collapsed = false;
    this.heldAudio = { wheee: false, theremin: false, chorale: null };
    this.captureRoot = root.closest('.sim-workbench') || root;
    this.render();
    this.input = new MicroDuckInputController({
      canvas,
      captureRoot: this.captureRoot,
      execute: (...args) => simulator.executeCommand(...args),
      releaseIntent: () => simulator.releaseHumanIntent(),
      getState: () => simulator.getState(),
      onCapture: (captured, reason) => this.renderCapture(captured, reason),
      onError: (error) => this.showError(error),
    });
    this.bind();
    this.timer = setInterval(() => this.refresh(), 100);
  }

  render() {
    const walkingMove = MICRODUCK_COMMANDS.move.ui.fieldsByMode.walking;
    const head = MICRODUCK_COMMANDS.head.ui.fields;
    const body = MICRODUCK_COMMANDS.pose.ui.fields;
    const look = MICRODUCK_COMMANDS.look.ui.fields;
    const mouth = MICRODUCK_COMMANDS.mouth.ui.fields.open;
    const tof = MICRODUCK_COMMANDS.set_tof_stimulus.ui.fields.distanceM;
    const cameras = MICRODUCK_COMMANDS.set_camera.ui;
    this.root.innerHTML = `<div class="md-deck-head"><div><strong>MICRODUCK CONTROL DECK</strong><small>Modeled browser policy simulator</small></div><button type="button" class="md-collapse" aria-expanded="true" aria-controls="microduckDeckBody">Collapse</button></div>
      <div id="microduckDeckBody" class="md-deck-body">
        <div class="md-boundary" role="note">Approximate dynamics · modeled camera/IMU/ToF/audio · not hardware validation</div>
        <section class="md-capture"><button type="button" class="md-capture-button">Capture simulator input</button><select aria-label="Keyboard layout" class="md-key-layout">${Object.keys(KEY_LAYOUTS).map((item) => `<option value="${item}">${item.toUpperCase()}</option>`).join('')}</select><span class="md-capture-state">Released</span><small>Focused human intent uses this app's ${MICRODUCK_COMMANDS.move.ui.input.captureLeaseMs} ms browser focus-safety lease. Editor shortcuts remain available while released.</small></section>
        <details open><summary>Runtime <span data-md-summary="runtime">ready</span></summary><div class="md-section-body"><div class="md-button-grid"><button type="button" data-md-command="enable" data-toggle="true">Enable / hold</button><button type="button" data-md-command="stop">Stop motion</button><button type="button" data-md-command="init">Init home ramp</button><button type="button" data-md-command="relax">Relax modeled gravity</button><button type="button" data-md-command="reset">Reset robot</button></div><div class="md-segmented" role="group" aria-label="Policy mode">${buttonGroup('set_mode', ['walking', 'roller'], 'mode')}</div></div></details>
        <details open><summary>Drive & intent <span data-md-summary="drive">0 m/s</span></summary><div class="md-section-body"><div class="md-ranges" data-range-group="move">${Object.entries(walkingMove).map(([name, spec]) => fieldMarkup('move', name, spec)).join('')}</div><div class="md-readout"><span>Requested <code data-md-value="movement.requested">0 / 0 / 0</code></span><span>Applied <code data-md-value="movement.applied">0 / 0 / 0</code></span><span>Limited <code data-md-value="movement.limitedBy">none</code></span></div></div></details>
        <details><summary>Head, look, body & mouth</summary><div class="md-section-body"><h4>Head command</h4><div class="md-ranges" data-range-group="head">${Object.entries(head).map(([name, spec]) => fieldMarkup('head', name, spec)).join('')}</div><h4>Trunk-frame look target</h4><div class="md-inline-fields">${Object.entries(look).map(([name, spec]) => `<label>${spec.label} <small>${spec.unit}</small><input type="number" data-md-number="look" data-field="${name}" min="${spec.range[0]}" max="${spec.range[1]}" step="${spec.step}" value="${name === 'x' ? 0.35 : name === 'z' ? 0.1 : 0}"></label>`).join('')}<button type="button" data-md-command="look">Apply look</button></div><h4>Body pose</h4><div class="md-ranges" data-range-group="pose">${Object.entries(body).map(([name, spec]) => fieldMarkup('pose', name, spec)).join('')}</div><h4>Mouth</h4><div class="md-ranges" data-range-group="mouth">${fieldMarkup('mouth', 'open', mouth)}</div></div></details>
        <details><summary>Skills <span data-md-summary="skill">idle</span></summary><div class="md-section-body"><div class="md-button-grid">${MICRODUCK_COMMANDS.do.values.map((skill) => `<button type="button" data-md-command="do" data-skill="${skill}">${skill === 'ground_pick' ? 'Ground Pick / Roller Crouch' : label(skill)}</button>`).join('')}</div><small>Ground pick is the trained mouth-to-ground trajectory; no hidden ball attachment or grasp claim.</small></div></details>
        <details><summary>Generated local audio <span data-md-summary="audio">locked</span></summary><div class="md-section-body"><div class="md-audio-status"><button type="button" class="md-audio-unlock">Unlock audio</button><strong data-md-value="audio.status">AUDIO_LOCKED</strong></div><div class="md-button-grid md-sounds">${MICRODUCK_COMMANDS.sound.values.map((sound) => `<button type="button" data-md-command="sound" data-tag="${sound}"${sound === 'wheee' ? ' data-held="true"' : ''}>${label(sound)}${sound === 'wheee' ? ' (hold)' : ''}</button>`).join('')}</div><label class="md-toggle"><input type="checkbox" data-md-toggle="theremin"> Modeled ToF theremin</label><div class="md-chorale"><label>Local chorale <select data-md-chorale="piece">${MICRODUCK_COMMANDS.chorale.ui.pieces.map((piece) => `<option value="${piece}">${label(piece)}</option>`).join('')}</select></label><label>Voices <input type="number" data-md-chorale="voices" min="${MICRODUCK_COMMANDS.chorale.ui.voices[0]}" max="${MICRODUCK_COMMANDS.chorale.ui.voices[1]}" value="1"></label><button type="button" data-md-chorale="start">Sing locally</button><button type="button" data-md-chorale="stop">Stop chorale</button></div><small>Deterministic Web Audio synthesis only. Single-tab, one to four voices; no recordings, BLE, multiplayer, or outer_wilds.</small></div></details>
        <details><summary>Presentation & modeled peripherals</summary><div class="md-section-body"><h4>Appearance (presentation only)</h4><div class="md-segmented">${buttonGroup('set_color', MICRODUCK_COMMANDS.set_color.values, 'color')}</div><h4>${cameras.label}</h4><div class="md-camera-grid" role="group" aria-label="MicroDuck main camera view">${MICRODUCK_COMMANDS.set_camera.values.map((mode) => { const option = cameras.options[mode]; return `<button type="button" class="md-camera-option" data-md-command="set_camera" data-camera="${mode}" title="${option.purpose}"><strong>${option.label}</strong><small>${option.purpose}</small></button>`; }).join('')}</div><div class="md-camera-purpose" role="note" aria-live="polite"><strong data-md-camera-name>Overview</strong><span data-md-camera-purpose>${cameras.options.orbit.purpose}</span><small>${cameras.shortcut}. Refit keeps the selected view.</small></div><div class="md-button-grid"><button type="button" data-md-command="spawn_ball">Spawn / reset ball</button><button type="button" data-md-perturb="left">Mouse perturb left</button><button type="button" data-md-perturb="right">Mouse perturb right</button></div><h4>8×8 modeled ToF at source frame</h4>${fieldMarkup('set_tof_stimulus', 'distanceM', tof, 0.4)}<label class="md-toggle"><input type="checkbox" data-md-tof-source="raycast"> Optional scene raycast sampling</label></div></details>
        <details open><summary>Immutable state <span data-md-summary="state">14 joints</span></summary><div class="md-section-body"><div class="md-telemetry" aria-live="polite"></div></div></details>
      </div>`;
  }

  bind() {
    this.root.querySelector('.md-collapse').addEventListener('click', () => {
      this.collapsed = !this.collapsed;
      const button = this.root.querySelector('.md-collapse');
      button.setAttribute('aria-expanded', String(!this.collapsed));
      button.textContent = this.collapsed ? 'Expand' : 'Collapse';
      this.root.classList.toggle('collapsed', this.collapsed);
    });
    this.root.querySelector('.md-capture-button').addEventListener('click', () => this.input.captured ? this.input.release('manual release') : this.input.capture());
    this.root.querySelector('.md-key-layout').addEventListener('change', (event) => this.input.setLayout(event.target.value));
    this.root.querySelector('.md-audio-unlock').addEventListener('click', async (event) => {
      try { const unlocked = await this.simulator.unlockAudio(event); this.onStatus(unlocked ? 'MicroDuck local audio unlocked by trusted input' : 'Audio unlock requires a trusted human event'); this.refresh(); }
      catch (error) { this.showError(error); }
    });
    this.root.querySelectorAll('[data-md-command]').forEach((button) => {
      if (button.dataset.held) {
        button.addEventListener('pointerdown', () => { this.heldAudio.wheee = true; void this.execute('sound', { tag: button.dataset.tag, hold: true }, MICRODUCK_COMMANDS.move.ui.input.captureLeaseMs); });
        for (const eventName of ['pointerup', 'pointercancel', 'pointerleave']) button.addEventListener(eventName, () => { this.heldAudio.wheee = false; void this.execute('sound', { tag: button.dataset.tag, hold: false }); });
      } else button.addEventListener('click', () => void this.executeButton(button));
    });
    this.root.querySelectorAll('[data-md-range]').forEach((range) => range.addEventListener('input', () => {
      const command = range.dataset.mdRange;
      const args = this.collectRanges(command);
      this.root.querySelector(`[data-output-for="${command}.${range.dataset.field}"]`).textContent = Number(range.value).toFixed(3);
      void this.execute(command, args, MICRODUCK_COMMANDS.move.ui.input.captureLeaseMs);
    }));
    this.root.querySelector('[data-md-toggle="theremin"]').addEventListener('change', (event) => { this.heldAudio.theremin = event.target.checked; void this.execute('theremin', { active: event.target.checked }, MICRODUCK_COMMANDS.move.ui.input.captureLeaseMs); });
    this.root.querySelector('[data-md-chorale="start"]').addEventListener('click', () => { this.heldAudio.chorale = { active: true, piece: this.root.querySelector('[data-md-chorale="piece"]').value, voices: Number(this.root.querySelector('[data-md-chorale="voices"]').value) }; void this.execute('chorale', this.heldAudio.chorale, MICRODUCK_COMMANDS.move.ui.input.captureLeaseMs); });
    this.root.querySelector('[data-md-chorale="stop"]').addEventListener('click', () => { this.heldAudio.chorale = null; void this.execute('chorale', { active: false }); });
    this.root.querySelector('[data-md-tof-source="raycast"]').addEventListener('change', (event) => void this.execute('set_tof_stimulus', { distanceM: Number(this.root.querySelector('[data-md-range="set_tof_stimulus"]').value), source: event.target.checked ? 'raycast' : 'synthetic' }));
    this.root.querySelectorAll('[data-md-perturb]').forEach((button) => button.addEventListener('click', () => { if (!this.requireCapture()) return; this.simulator.perturb(button.dataset.mdPerturb === 'left' ? [0.35, 0, 0.2] : [-0.35, 0, -0.2]); this.onStatus('Applied modeled mouse perturbation'); }));
    let drag = null;
    this.canvas.addEventListener('pointerdown', (event) => { if (this.active && this.input.captured && event.shiftKey) { drag = { x: event.clientX, y: event.clientY }; event.preventDefault(); } });
    this.canvas.addEventListener('pointerup', (event) => { if (!drag) return; this.simulator.perturb([(event.clientY - drag.y) / 120, 0, (event.clientX - drag.x) / 120]); drag = null; this.onStatus('Applied modeled Shift-drag perturbation'); });
  }

  async executeButton(button) {
    const command = button.dataset.mdCommand;
    if (command === 'stop' || command === 'reset') this.input.release(command);
    if (command === 'enable') return this.execute(command, { toggle: true });
    if (command === 'set_mode') { const result = await this.execute(command, { mode: button.dataset.mode }); if (result) this.input.release('mode switch'); return result; }
    if (command === 'do') return this.execute(command, { skill: button.dataset.skill });
    if (command === 'sound') return this.execute(command, { tag: button.dataset.tag });
    if (command === 'set_color') return this.execute(command, { color: button.dataset.color });
    if (command === 'set_camera') return this.execute(command, { camera: button.dataset.camera });
    if (command === 'look') return this.execute(command, Object.fromEntries(Array.from(this.root.querySelectorAll('[data-md-number="look"]'), (input) => [input.dataset.field, Number(input.value)])));
    return this.execute(command, {});
  }

  collectRanges(command) { return Object.fromEntries(Array.from(this.root.querySelectorAll(`[data-md-range="${command}"]`), (input) => [input.dataset.field, Number(input.value)])); }
  async execute(command, args, durationMs) {
    if (!['stop', 'reset', 'set_camera'].includes(command) && !this.requireCapture()) return null;
    try {
      const result = await this.simulator.executeCommand(command, args, { source: 'human', controllerId: 'control-deck', durationMs });
      this.onStatus(`MicroDuck ${label(command)} applied${result.limitedBy?.length ? ` · limited: ${result.limitedBy.join(', ')}` : ''}`);
      this.refresh();
      return result;
    } catch (error) { this.showError(error); return null; }
  }

  requireCapture() {
    if (this.input.captured) return true;
    this.showError({ code: 'SIMULATOR_CAPTURE_REQUIRED', message: 'Capture simulator input before applying a manual MicroDuck command.' });
    return false;
  }

  showError(error) { const code = error?.code || 'SIMULATION_ERROR'; this.onStatus(`${code}: ${error?.message || error}`); this.root.dataset.error = code; }
  setActive(active) { this.active = Boolean(active); this.root.hidden = !this.active; this.input.setActive(this.active); if (!this.active) { this.clearHeldAudio(); this.root.dataset.error = ''; } else { this.renderCapture(this.input.captured); this.refresh(); } }
  renderCapture(captured, reason = '') {
    const leaseMs = MICRODUCK_COMMANDS.move.ui.input.captureLeaseMs;
    const button = this.root.querySelector('.md-capture-button');
    button.textContent = captured ? 'Release simulator input' : 'Capture simulator input';
    button.setAttribute('aria-pressed', String(captured));
    this.root.querySelector('.md-capture-state').textContent = captured ? `Captured · ${leaseMs} ms lease` : `Released${reason ? ` · ${reason}` : ''}`;
    if (!captured) this.clearHeldAudio();
    this.syncManualControlAvailability();
  }

  clearHeldAudio() {
    this.heldAudio = { wheee: false, theremin: false, chorale: null };
    const theremin = this.root.querySelector('[data-md-toggle="theremin"]');
    if (theremin) theremin.checked = false;
  }

  syncManualControlAvailability() {
    const captured = this.input.captured;
    this.root.querySelectorAll('[data-md-command], [data-md-range], [data-md-toggle], [data-md-chorale], [data-md-tof-source], [data-md-perturb]').forEach((control) => {
      const safety = ['stop', 'reset', 'set_camera'].includes(control.dataset.mdCommand);
      const fixedRange = control.dataset.mdRange && Number(control.min) === Number(control.max);
      control.disabled = !safety && (!captured || fixedRange);
    });
  }

  refresh() {
    if (!this.active || !this.simulator.isReady()) return;
    const at = performance.now();
    if (!this.lastAudioRefresh || at - this.lastAudioRefresh >= 100) {
      this.lastAudioRefresh = at;
      if (this.heldAudio.wheee) void this.refreshHeld('sound', { tag: 'wheee', hold: true });
      if (this.heldAudio.theremin) void this.refreshHeld('theremin', { active: true });
      if (this.heldAudio.chorale) void this.refreshHeld('chorale', this.heldAudio.chorale);
    }
    const state = this.simulator.getState();
    if (!state) return;
    const set = (selector, text) => { const node = this.root.querySelector(selector); if (node) node.textContent = text; };
    set('[data-md-summary="runtime"]', `${state.mode} · ${state.enabled ? 'enabled' : 'held'}`);
    set('[data-md-summary="drive"]', `${state.movement.applied[0].toFixed(2)} m/s`);
    set('[data-md-summary="skill"]', state.phase);
    set('[data-md-summary="audio"]', state.audio.unlocked ? 'local synthesis ready' : 'locked');
    set('[data-md-value="movement.requested"]', state.movement.requested.map((item) => item.toFixed(2)).join(' / '));
    set('[data-md-value="movement.applied"]', state.movement.applied.map((item) => item.toFixed(2)).join(' / '));
    set('[data-md-value="movement.limitedBy"]', state.movement.limitedBy.join(', ') || 'none');
    set('[data-md-value="audio.status"]', state.audio.unlocked ? 'UNLOCKED · GENERATED LOCAL AUDIO' : 'AUDIO_LOCKED');
    set('[data-md-camera-name]', state.virtualCamera.name);
    set('[data-md-camera-purpose]', state.virtualCamera.purpose);
    const fitButton = document.querySelector('#fitBtn');
    if (fitButton) {
      const camera = MICRODUCK_COMMANDS.set_camera.ui.options[state.virtualCamera.mode];
      fitButton.textContent = camera.fitLabel;
      fitButton.title = `${camera.fitLabel}. ${camera.purpose}`;
      fitButton.setAttribute('aria-label', fitButton.title);
    }
    const moveFields = MICRODUCK_COMMANDS.move.ui.fieldsByMode[state.mode];
    this.root.querySelectorAll('[data-md-range="move"]').forEach((input) => {
      const spec = moveFields[input.dataset.field];
      input.min = String(spec.range[0]);
      input.max = String(spec.range[1]);
      input.step = String(spec.step);
      input.disabled = !this.input.captured || spec.range[0] === spec.range[1];
      if (Number(input.value) < spec.range[0] || Number(input.value) > spec.range[1]) input.value = '0';
    });
    this.root.querySelectorAll('[data-md-command="set_mode"]').forEach((button) => button.classList.toggle('active', button.dataset.mode === state.mode));
    this.root.querySelectorAll('[data-md-command="set_color"]').forEach((button) => button.classList.toggle('active', button.dataset.color === state.color));
    this.root.querySelectorAll('[data-md-command="set_camera"]').forEach((button) => button.classList.toggle('active', button.dataset.camera === state.virtualCamera.mode));
    const telemetry = [
      ['Policy / phase', `${state.activePolicy} / ${state.phase}`], ['Safety / recovery', `${state.safety.fallen ? 'fallen · ' : ''}${state.safety.recovery}`], ['Loop', `${state.loop.rateHz.toFixed(1)} Hz · ${state.loop.missedTicks} missed`],
      ['Pose (modeled)', state.simulatedPose.position.map((item) => item.toFixed(3)).join(', ') + ' m'], ['Contact / ball', `${state.contacts.count} / ${state.contacts.ballContact ? 'contact' : 'clear'} · unattached`], ['Camera', `${state.virtualCamera.name} · ${state.virtualCamera.frame} · main rendered view`],
      ['Trunk IMU (modeled)', state.imu.trunk.gyro.map((item) => item.toFixed(3)).join(', ') + ' rad/s'], ['Head IMU (modeled)', state.imu.head.gyro.map((item) => item.toFixed(3)).join(', ') + ' rad/s'], ['ToF (modeled)', `${state.tof.rows}×${state.tof.cols} · ${state.tof.minimumM.toFixed(3)} m · ${state.tof.source}`],
      ['Mouth', state.mouth.toFixed(3)], ['Joints / targets (rad)', state.joints.map((item, index) => `${item.toFixed(2)}/${state.targets[index].toFixed(2)}`).join('  ')], ['Audio', `${state.audio.sound || 'none'} · theremin ${state.audio.theremin ? `${state.audio.thereminFrequencyHz} Hz` : 'off'} · chorale ${state.audio.chorale ? `${state.audio.piece}/${state.audio.voices}` : 'off'}`],
    ];
    this.root.querySelector('.md-telemetry').innerHTML = telemetry.map(([name, value]) => `<span>${name}</span><code>${value}</code>`).join('');
    this.syncManualControlAvailability();
    this.onState(state);
  }

  async refreshHeld(command, args) {
    if (!this.input.captured) return;
    try { await this.simulator.executeCommand(command, args, { source: 'human', controllerId: 'control-deck', durationMs: MICRODUCK_COMMANDS.move.ui.input.captureLeaseMs }); }
    catch (error) { this.heldAudio = { wheee: false, theremin: false, chorale: null }; this.showError(error); }
  }

  dispose() { clearInterval(this.timer); this.input.dispose(); }
}
