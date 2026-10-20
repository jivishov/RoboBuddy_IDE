import {icon, iconButton} from '../hardware/icons.js';
import {GESTURE_JOINTS, GESTURE_MAX_AGE_MS, analyzeHand, gestureDirection, containedImageRect} from './gesture-core.js';
import {WebcamTracker} from './webcam-tracker.js';

export class GesturePanel {
  constructor(button, adapter, {trackerFactory = (video, callbacks) => new WebcamTracker(video, callbacks)} = {}) {
    this.button = button;
    this.adapter = adapter;
    this.epoch = 0;
    this.cameraEpoch = 0;
    this.enabled = false;
    this.camera = false;
    this.starting = false;
    this.direction = 0;
    this.panel = document.createElement('aside');
    this.panel.className = 'gesture-panel';
    this.panel.hidden = true;
    this.panel.id = 'webcamGesturePanel';
    this.panel.setAttribute('aria-label', 'SO-101 webcam gesture control');
    this.panel.innerHTML = `
      <div class="gesture-title"><div><strong>Hand control</strong><small></small></div><button type="button" data-ui="close"></button></div>
      <div class="gesture-content">
        <div class="gesture-camera">
          <video autoplay playsinline muted aria-label="Mirrored webcam preview"></video>
          <canvas width="640" height="400" aria-hidden="true"></canvas>
          <div class="gesture-empty">${icon('camera')}<strong>Start your webcam</strong><span>Preview first. You choose when to move.</span></div>
          <span class="gesture-camera-label">Camera off</span><span class="gesture-mirror">Mirrored view</span>
        </div>
        <div class="gesture-body">
          <label class="gesture-joint-label">Control joint<select aria-label="Gesture control joint" aria-describedby="gestureInstructions"></select></label>
          <p class="gesture-instructions" id="gestureInstructions"></p>
          <div class="gesture-command">
            <div class="gesture-command-heading"><span data-ui="input-label">Preview input</span><output data-ui="direction" aria-live="off">Set neutral first</output></div>
            <div class="gesture-meter" role="meter" aria-label="Joint command input" aria-valuemin="-100" aria-valuemax="100" aria-valuenow="0"><span></span></div>
            <div class="gesture-meter-labels"><span data-ui="negative">− Decrease</span><span>Hold</span><span data-ui="positive">Increase +</span></div>
            <div class="gesture-readback"><span data-ui="readback-label">Readback</span><output data-ui="readback" aria-live="off">—</output></div>
          </div>
          <details class="gesture-help"><summary>Options &amp; help</summary><label class="gesture-reverse"><input type="checkbox"> Reverse direction</label><p>Keep your whole hand in view. Move gently; only the selected joint responds. Return to neutral to hold.</p><p>Video stays on this device. First use downloads the hand tracker.</p></details>
        </div>
      </div>
      <div class="gesture-footer">
        <p class="gesture-status" role="status" aria-live="polite">Start the camera to begin.</p>
        <div class="gesture-actions">
          <div data-step="camera"><button type="button" data-ui="camera"></button><span>1 · Camera</span></div>
          <div data-step="neutral"><button type="button" data-ui="neutral"></button><span>2 · Neutral</span></div>
          <div data-step="enable"><button type="button" data-ui="enable" aria-pressed="false"></button><span>3 · Enable</span></div>
          <div data-step="pause"><button type="button" data-ui="pause"></button><span>Pause <kbd>Esc</kbd></span></div>
        </div>
        <p class="gesture-stop-hint">A fist or lost tracking pauses control.</p>
      </div>`;
    document.body.append(this.panel);
    this.ui = key => this.panel.querySelector('[data-ui="' + key + '"]');
    this.status = this.panel.querySelector('.gesture-status');
    this.video = this.panel.querySelector('video');
    this.canvas = this.panel.querySelector('canvas');
    this.select = this.panel.querySelector('select');
    this.reverse = this.panel.querySelector('input');
    for (const [value, label] of Object.entries(GESTURE_JOINTS)) this.select.add(new Option(label, value));
    for (const [key, name, label] of [['close','cancel','Close hand control'], ['camera','camera','Start webcam'], ['neutral','calibrate','Set gesture neutral'], ['enable','hand','Enable gesture control'], ['pause','stop','Pause gesture control']]) iconButton(this.ui(key), name, label);
    iconButton(button, 'hand', 'SO-101 hand gesture control');
    button.setAttribute('aria-controls', this.panel.id);
    button.setAttribute('aria-expanded', 'false');
    button.addEventListener('click', () => this.panel.hidden ? this.open() : this.close());
    this.ui('close').onclick = () => this.close();
    this.ui('camera').onclick = () => this.camera || this.starting ? this.stopCamera() : void this.startCamera();
    this.ui('neutral').onclick = () => this.setNeutral();
    this.ui('enable').onclick = event => { if (event.isTrusted) void this.enable(); };
    this.ui('pause').onclick = () => { void this.pause('Paused. Set neutral and enable to continue.'); };
    this.select.onchange = this.reverse.onchange = () => {
      this.resetStability();
      void this.pause('Joint or direction changed. Hold steady, then set neutral.');
    };
    this.tracker = trackerFactory(this.video, {
      onFrame: (result, at) => this.frame(result, at),
      onError: error => this.stopCamera(error.message),
    });
    document.addEventListener('keydown', event => {
      if (event.key === 'Escape' && !this.panel.hidden) {
        event.preventDefault();
        void this.pause('Paused with Esc. Set neutral again.');
      }
    });
    document.addEventListener('visibilitychange', () => { if (document.hidden) this.stopCamera('Tab hidden. Restart the camera to continue.'); });
    window.addEventListener('pagehide', () => this.stopCamera());
    window.addEventListener('blur', () => { if (this.enabled || this.enabling) void this.pause('Window lost focus. Set neutral again.'); });
    this.timer = setInterval(() => this.tick(), 100);
    this.render();
  }
  open() {
    this.panel.hidden = false;
    this.button.setAttribute('aria-expanded', 'true');
    this.render();
    this.ui('camera').focus({preventScroll: true});
  }
  close({restoreFocus = true} = {}) {
    const pending = this.stopCamera();
    this.panel.hidden = true;
    this.button.setAttribute('aria-expanded', 'false');
    if (restoreFocus) this.button.focus({preventScroll: true});
    return pending;
  }
  say(message) { if (this.status.textContent !== message) this.status.textContent = message; }
  fresh() { return this.camera && this.hand?.valid && Number.isFinite(this.lastFrameAt) && performance.now() - this.lastFrameAt <= GESTURE_MAX_AGE_MS; }
  isPoseReady() { return this.select.value === 'gripper' ? this.hand?.gripReady : this.hand?.open; }
  centered() { return this.fresh() && this.isPoseReady() && this.neutral && this.hand.hand === this.neutral.hand && gestureDirection(this.hand, this.neutral, this.select.value) === 0; }
  resetStability() { this.stable = false; this.stableSince = null; this.stableAnchor = null; }
  render() {
    const busy = this.enabled || this.enabling || this.stopping;
    const ready = this.fresh() && this.isPoseReady() && this.stable;
    const grip = this.select.value === 'gripper';
    const prerequisite = this.adapter.availability?.() || '';
    iconButton(this.ui('camera'), this.camera || this.starting ? 'powerOff' : 'camera', this.camera || this.starting ? 'Stop webcam' : 'Start webcam');
    this.ui('neutral').disabled = !ready || busy;
    this.ui('enable').disabled = !this.centered() || busy || !!prerequisite;
    this.ui('enable').setAttribute('aria-pressed', String(this.enabled));
    this.ui('pause').disabled = !this.enabled && !this.enabling;
    this.select.disabled = busy;
    this.reverse.disabled = busy;
    this.panel.dataset.active = String(this.enabled);
    this.panel.dataset.stopping = String(!!this.stopping);
    this.panel.dataset.camera = this.starting ? 'loading' : this.camera ? 'on' : 'off';
    this.panel.dataset.next = !this.camera ? 'camera' : !this.neutral ? 'neutral' : 'enable';
    this.button.dataset.active = String(this.enabled);
    this.panel.querySelector('.gesture-title small').textContent = this.adapter.context?.() || this.adapter.label;
    this.panel.querySelector('.gesture-instructions').textContent = grip
      ? `Three fingers extended. Pinch to ${this.reverse.checked ? 'open' : 'close'}; spread to ${this.reverse.checked ? 'close' : 'open'}.`
      : `Open palm. Move ${this.reverse.checked ? 'left' : 'right'} to increase the joint angle; return to center to hold.`;
    const phase = this.stopping ? 'Pausing' : this.enabling ? 'Enabling' : this.enabled ? 'Control active' : this.starting ? 'Loading tracker' : this.camera ? 'Preview · no motion' : 'Camera off';
    this.panel.querySelector('.gesture-camera-label').textContent = phase;
    this.panel.querySelector('.gesture-empty strong').textContent = this.starting ? 'Loading hand tracker…' : 'Start your webcam';
    this.panel.querySelector('.gesture-empty span').textContent = this.starting ? 'First use can take a moment.' : 'Preview first. You choose when to move.';
    this.panel.querySelector('.gesture-mirror').hidden = !this.camera;
    const input = this.fresh() && this.neutral ? (this.enabled ? this.direction : gestureDirection(this.hand, this.neutral, this.select.value)) * (this.reverse.checked ? -1 : 1) : 0;
    const percent = Math.round(input * 100);
    this.ui('input-label').textContent = this.enabled ? 'Command input' : 'Preview input';
    const intent = !this.neutral ? 'Set neutral first' : input === 0 ? 'Hold · neutral' : (grip ? input > 0 ? 'Opening' : 'Closing' : input > 0 ? 'Increasing' : 'Decreasing') + ' · ' + (Math.abs(percent) || '<1') + '%';
    this.ui('direction').textContent = intent;
    const meter = this.panel.querySelector('.gesture-meter');
    meter.setAttribute('aria-valuenow', String(percent));
    meter.setAttribute('aria-valuetext', intent);
    meter.style.setProperty('--input-position', (50 + percent * .46) + '%');
    this.ui('negative').textContent = grip ? '− Close' : '− Decrease';
    this.ui('positive').textContent = grip ? 'Open +' : 'Increase +';
    const readback = this.adapter.readback?.(this.select.value);
    this.ui('readback-label').textContent = readback?.label || 'Readback';
    this.ui('readback').textContent = readback?.value || '—';
    if (this.stopping) this.say('Pausing control…');
    else if (this.enabling) this.say('Enabling at the measured pose…');
    else if (this.enabled) this.say('Control active. Center to hold; fist or Esc to pause.');
    else if (this.notice) this.say(this.notice);
    else if (this.starting) this.say('Allow the camera, then wait for the tracker to load.');
    else if (!this.camera) this.say('1 · Start the camera. Preview cannot move the arm.');
    else if (prerequisite) this.say(prerequisite);
    else if (this.neutral) this.say(this.centered() ? '3 · Neutral set. Enable when ready.' : 'Return to neutral before enabling, or set a new neutral.');
    else if (ready) this.say('2 · Hand ready. Set your comfortable neutral position.');
    else this.say(this.hand?.valid ? this.isPoseReady() ? 'Hold your hand steady…' : grip ? 'Extend middle, ring and little fingers; keep thumb and index free.' : 'Open your palm with all four fingers extended.' : this.hand?.reason || 'Show one open hand inside the preview.');
    this.draw();
    this.adapter.changed?.();
  }
  async startCamera() {
    if (this.camera || this.starting) return;
    const epoch = ++this.cameraEpoch;
    this.starting = true;
    this.notice = null;
    this.hand = null;
    this.lastFrameAt = null;
    this.resetStability();
    this.render();
    try {
      if (await this.tracker.start() && epoch === this.cameraEpoch) this.camera = true;
    } catch (error) { if (epoch === this.cameraEpoch) this.notice = error.message; }
    finally { if (epoch === this.cameraEpoch) { this.starting = false; this.render(); } }
  }
  stopCamera(message = 'Camera off. Start the camera to begin.') {
    ++this.cameraEpoch;
    this.camera = false;
    this.starting = false;
    this.tracker.stop();
    this.hand = null;
    this.lastFrameAt = null;
    this.resetStability();
    return this.pause(message);
  }
  frame(result, capturedAt) {
    const now = performance.now();
    if (!this.camera || !Number.isFinite(capturedAt) || capturedAt > now || now - capturedAt > GESTURE_MAX_AGE_MS || capturedAt <= (this.lastFrameAt ?? -Infinity)) return;
    const previous = this.hand;
    const gap = this.lastFrameAt !== null && capturedAt - this.lastFrameAt > GESTURE_MAX_AGE_MS;
    this.lastFrameAt = capturedAt;
    this.imageWidth = result.imageWidth || this.video.videoWidth || 640;
    this.imageHeight = result.imageHeight || this.video.videoHeight || 480;
    this.hand = analyzeHand(result, this.imageWidth / this.imageHeight);
    if (gap || !this.hand.valid || this.hand.fist || !this.isPoseReady() || (this.neutral && this.hand.hand !== this.neutral.hand) || (previous?.valid && (previous.hand !== this.hand.hand || Math.abs(previous.x - this.hand.x) > .18))) {
      const message = gap ? 'Tracking lost · paused. Set neutral again.' : this.hand.fist ? 'Fist detected · paused. Open your hand and set neutral again.' : !this.hand.valid ? this.hand.reason : 'Hand pose changed · paused. Set neutral again.';
      this.resetStability();
      if (this.enabled || this.enabling || this.neutral) void this.pause(message);
      else if (!this.notice) this.say(message);
    } else {
      if (this.stableSince === null || (!this.enabled && !this.neutral && (Math.abs(this.hand.x - this.stableAnchor.x) > .02 || (this.select.value === 'gripper' && Math.abs(this.hand.pinch - this.stableAnchor.pinch) > .15)))) {
        this.stableSince = now;
        this.stableAnchor = this.hand;
      }
      this.stable = now - this.stableSince >= 500;
      if (this.enabled) this.direction = gestureDirection(this.hand, this.neutral, this.select.value, this.direction);
    }
    this.render();
  }
  setNeutral() {
    if (this.enabled || this.enabling || this.stopping || !this.fresh() || !this.stable || !this.isPoseReady()) return;
    this.neutral = {...this.hand};
    this.direction = 0;
    this.notice = null;
    this.render();
    // Make the next keyboard action explicit without changing focus every frame.
    if (!this.ui('enable').disabled) this.ui('enable').focus({preventScroll: true});
  }
  async enable() {
    if (this.enabled || this.enabling || this.stopping || !this.centered()) return;
    const epoch = ++this.epoch;
    this.enabling = true;
    this.notice = null;
    this.beginTask = Promise.resolve().then(() => this.adapter.begin(this.select.value));
    this.render();
    try {
      await this.beginTask;
      if (epoch !== this.epoch) return; // The cancelling pause owns cleanup.
      if (!this.centered()) { await this.pause('Hand moved while enabling. Set neutral and try again.'); return; }
      this.enabled = true;
    } catch (error) { if (epoch === this.epoch) await this.pause(error.message); }
    finally {
      if (epoch === this.epoch) { this.enabling = false; this.render(); }
    }
  }
  pause(message = 'Paused. Set neutral again.') {
    const active = this.enabled || this.enabling;
    ++this.epoch;
    this.enabled = false;
    this.enabling = false;
    this.neutral = null;
    this.direction = 0;
    this.notice = message;
    this.resetStability();
    if (active && !this.stopPromise) {
      this.stopping = true;
      const beginTask = this.beginTask;
      this.stopPromise = (async () => {
        // A late begin result must be cleaned up before another owner can start.
        await beginTask?.catch(() => {});
        await this.adapter.end();
      })().catch(error => { this.notice = error.message; }).finally(() => {
        this.beginTask = null;
        this.stopPromise = null;
        this.stopping = false;
        this.render();
      });
    }
    this.render();
    return this.stopPromise || Promise.resolve();
  }
  async tick() {
    if (this.panel.hidden && !this.camera && !this.starting) return;
    if (this.camera && (!Number.isFinite(this.lastFrameAt) || performance.now() - this.lastFrameAt > GESTURE_MAX_AGE_MS)) {
      const hadFrame = this.hand !== null;
      this.resetStability();
      this.hand = null;
      if (this.enabled || this.enabling || this.neutral) await this.pause('Tracking lost · paused. Show one hand and set neutral again.');
      else if (hadFrame) this.notice = 'Tracking lost. Show one hand and set neutral again.';
    }
    this.render();
    if (!this.enabled || this.sending) return;
    const epoch = this.epoch;
    this.sending = true;
    try { await this.adapter.send(this.select.value, this.direction * (this.reverse.checked ? -1 : 1)); }
    catch (error) { if (epoch === this.epoch) await this.pause(error.message); }
    finally { this.sending = false; }
  }
  draw() {
    const canvas = this.canvas, ctx = canvas.getContext('2d');
    const area = this.panel.querySelector('.gesture-camera');
    if (area.clientWidth && area.clientHeight) {
      const height = Math.round(canvas.width * area.clientHeight / area.clientWidth);
      if (canvas.height !== height) canvas.height = height;
    }
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    if (!this.camera || !this.hand?.valid) return;
    const rect = containedImageRect(this.imageWidth, this.imageHeight, canvas.width, canvas.height);
    const px = x => rect.x + x * rect.width;
    const py = y => rect.y + y * rect.height;
    ctx.strokeStyle = this.enabled ? '#9bf1c2' : '#e8efed';
    ctx.fillStyle = ctx.strokeStyle;
    ctx.lineWidth = 2;
    const p = this.hand.points;
    if (this.neutral && this.select.value !== 'gripper') {
      ctx.fillStyle = '#9bf1c218';
      ctx.fillRect(px(this.neutral.x - .045), rect.y, .09 * rect.width, rect.height);
      ctx.setLineDash([5,6]);
      for (const offset of [-.045,.045]) { const x = px(this.neutral.x + offset); ctx.beginPath(); ctx.moveTo(x, rect.y); ctx.lineTo(x, rect.y + rect.height); ctx.stroke(); }
      ctx.setLineDash([]);
      ctx.fillStyle = ctx.strokeStyle;
    }
    for (const chain of [[0,1,2,3,4],[0,5,6,7,8],[5,9,10,11,12],[9,13,14,15,16],[13,17,18,19,20],[0,17]]) {
      ctx.beginPath();
      chain.forEach((id, i) => { const x = px(1-p[id].x), y = py(p[id].y); i ? ctx.lineTo(x,y) : ctx.moveTo(x,y); });
      ctx.stroke();
    }
    for (const point of p) { ctx.beginPath(); ctx.arc(px(1-point.x), py(point.y), 3, 0, Math.PI*2); ctx.fill(); }
  }
}
