import {iconButton} from '../hardware/icons.js';
import {GESTURE_JOINTS, GESTURE_MAX_AGE_MS, analyzeHand, gestureDirection} from './gesture-core.js';
import {WebcamTracker} from './webcam-tracker.js';

export class GesturePanel {
  constructor(button, adapter, {trackerFactory = (video, callbacks) => new WebcamTracker(video, callbacks)} = {}) {
    this.button = button; this.adapter = adapter; this.epoch = 0; this.enabled = false; this.camera = false; this.starting = false; this.direction = 0;
    this.panel = document.createElement('aside'); this.panel.className = 'gesture-panel'; this.panel.hidden = true;
    this.panel.id = 'webcamGesturePanel'; this.panel.setAttribute('aria-label', 'SO-101 webcam gesture control');
    this.panel.innerHTML = '<div class="gesture-title"><div><strong>Hand control</strong><small></small></div><button type="button" data-ui="close"></button></div><div class="gesture-camera"><video autoplay playsinline muted aria-label="Mirrored webcam preview"></video><canvas width="640" height="480" aria-hidden="true"></canvas><span class="gesture-camera-label">Camera off</span></div><div class="gesture-body"><p class="gesture-status" role="status" aria-live="polite">Start the camera to begin.</p><label class="gesture-joint-label">Control joint<select aria-label="Gesture control joint"></select></label><p class="gesture-instructions"></p><div class="gesture-actions"><div><button type="button" data-ui="camera"></button><span>Camera</span></div><div><button type="button" data-ui="neutral"></button><span>Set neutral</span></div><div><button type="button" data-ui="enable" aria-pressed="false"></button><span>Enable</span></div><div><button type="button" data-ui="pause"></button><span>Pause</span></div></div><label class="gesture-reverse"><input type="checkbox"> Reverse direction</label><p class="gesture-privacy">Video stays on this device. First use downloads the hand tracker. A fist, lost tracking or leaving this tab pauses control.</p></div>';
    document.body.append(this.panel);
    this.ui = key => this.panel.querySelector('[data-ui="' + key + '"]');
    this.status = this.panel.querySelector('.gesture-status'); this.video = this.panel.querySelector('video');
    this.select = this.panel.querySelector('select'); this.reverse = this.panel.querySelector('input');
    this.panel.querySelector('.gesture-title small').textContent = adapter.label;
    for (const [value, label] of Object.entries(GESTURE_JOINTS)) this.select.add(new Option(label, value));
    for (const [key, name, label] of [['close','cancel','Close hand control'], ['camera','camera','Start webcam'], ['neutral','calibrate','Set gesture neutral'], ['enable','hand','Enable gesture control'], ['pause','stop','Pause gesture control']]) iconButton(this.ui(key), name, label);
    iconButton(button, 'hand', 'SO-101 hand gesture control'); button.setAttribute('aria-controls', this.panel.id); button.setAttribute('aria-expanded', 'false');
    button.addEventListener('click', () => this.panel.hidden ? this.open() : this.close());
    this.ui('close').onclick = () => this.close();
    this.ui('camera').onclick = () => { if (this.camera || this.starting) this.stopCamera(); else void this.startCamera(); };
    this.ui('neutral').onclick = () => this.setNeutral();
    this.ui('enable').onclick = event => { if (event.isTrusted) void this.enable(); };
    this.ui('pause').onclick = () => { void this.pause('Paused. Set neutral and enable to continue.'); };
    this.select.onchange = this.reverse.onchange = () => { void this.pause('Joint or direction changed. Set neutral again.'); this.render(); };
    this.tracker = trackerFactory(this.video, {onFrame: (result, at) => this.frame(result, at), onError: error => { this.camera = false; this.starting = false; void this.pause(error.message); this.render(); }});
    document.addEventListener('keydown', event => { if (event.key === 'Escape' && !this.panel.hidden) void this.pause('Paused with Esc. Set neutral again.'); });
    document.addEventListener('visibilitychange', () => { if (document.hidden) this.stopCamera('Tab hidden. Restart the camera to continue.'); });
    window.addEventListener('pagehide', () => this.stopCamera());
    window.addEventListener('blur', () => { if (this.enabled || this.enabling) void this.pause('Window lost focus. Set neutral again.'); });
    this.timer = setInterval(() => this.tick(), 100);
    this.render();
  }
  open() { this.panel.hidden = false; this.button.setAttribute('aria-expanded', 'true'); this.ui('camera').focus(); }
  close({restoreFocus = true} = {}) { const pending = this.stopCamera(); this.panel.hidden = true; this.button.setAttribute('aria-expanded', 'false'); if (restoreFocus) this.button.focus(); return pending; }
  say(message) { if (this.status.textContent !== message) this.status.textContent = message; }
  render() {
    iconButton(this.ui('camera'), this.camera || this.starting ? 'powerOff' : 'camera', this.camera || this.starting ? 'Stop webcam' : 'Start webcam');
    this.ui('neutral').disabled = !this.camera || !this.hand?.valid || !this.isPoseReady() || !this.stable || this.enabled || this.enabling;
    this.ui('enable').disabled = !this.neutral || !this.camera || this.enabled || this.enabling || this.stopping;
    this.ui('enable').setAttribute('aria-pressed', String(this.enabled));
    this.ui('pause').disabled = !this.enabled && !this.enabling;
    this.select.disabled = this.enabled || this.enabling; this.reverse.disabled = this.enabled || this.enabling;
    this.panel.dataset.active = String(this.enabled);
    this.panel.querySelector('.gesture-instructions').textContent = this.select.value === 'gripper'
      ? 'Keep three fingers extended. Set neutral with thumb and index slightly apart. Pinch to close; spread to open. Return to neutral to hold.'
      : 'Show an open palm. Set neutral, then enable. Move left or right in the mirrored preview to jog this joint; return to center to hold.';
    this.panel.querySelector('.gesture-camera-label').textContent = this.enabled ? 'GESTURES ENABLED' : this.camera ? 'PREVIEW · NO MOTION' : 'Camera off';
    this.adapter.changed?.();
  }
  isPoseReady() { return this.select.value === 'gripper' ? this.hand?.gripReady : this.hand?.open; }
  async startCamera() {
    const epoch = ++this.epoch; this.starting = true; this.notice = null; this.say('Opening webcam and loading hand tracker…'); this.render();
    try {
      if (await this.tracker.start() && epoch === this.epoch) { this.camera = true; this.say('Show one open hand and hold it steady, then Set neutral.'); }
    } catch (error) { if (epoch === this.epoch) this.say(error.message); }
    finally { if (epoch === this.epoch) { this.starting = false; this.render(); } }
  }
  stopCamera(message = 'Camera off. Start the camera to begin.') {
    this.camera = false; this.starting = false; this.tracker.stop(); this.hand = null; this.stable = false; this.stableSince = null;
    this.panel.querySelector('canvas').getContext('2d').clearRect(0, 0, 640, 480);
    return this.pause(message);
  }
  frame(result, capturedAt) {
    const now = performance.now();
    if (!this.camera || !Number.isFinite(capturedAt) || capturedAt > now || now - capturedAt > GESTURE_MAX_AGE_MS || capturedAt <= (this.lastFrameAt ?? -Infinity)) return;
    const previous = this.hand; this.lastFrameAt = capturedAt; this.hand = analyzeHand(result);
    if (!this.hand.valid || this.hand.fist || !this.isPoseReady() || (this.neutral && this.hand.hand !== this.neutral.hand) || (previous?.valid && (previous.hand !== this.hand.hand || Math.abs(previous.x - this.hand.x) > .18))) {
      const message = this.hand.fist ? 'Fist detected · paused. Open your hand and set neutral again.' : !this.hand.valid ? this.hand.reason : 'Hand pose changed · paused. Set neutral again.';
      this.stableSince = null; this.stable = false;
      if (this.enabled || this.enabling || this.neutral) void this.pause(message); else if (!this.notice) this.say(message);
    } else {
      if (!this.stableSince || (!this.enabled && !this.neutral && (Math.abs(this.hand.x - this.stableAnchor.x) > .02 || (this.select.value === 'gripper' && Math.abs(this.hand.pinch - this.stableAnchor.pinch) > .15)))) { this.stableSince = now; this.stableAnchor = this.hand; }
      this.stable = now - this.stableSince >= 500;
      if (this.enabled) this.direction = gestureDirection(this.hand, this.neutral, this.select.value, this.direction);
      else if (!this.neutral && !this.notice) this.say(this.stable ? 'Hand ready. Set neutral, then Enable.' : 'Hold your hand steady…');
    }
    this.draw(); this.render();
  }
  setNeutral() {
    if (!this.camera || !this.stable || !this.hand?.valid || !this.isPoseReady() || performance.now() - this.lastFrameAt > GESTURE_MAX_AGE_MS) return;
    this.neutral = {...this.hand}; this.direction = 0; this.notice = null; this.say('Neutral set. Enable when ready.'); this.render();
  }
  async enable() {
    if (this.enabled || this.enabling || this.stopping || !this.neutral || !this.camera || performance.now() - this.lastFrameAt > GESTURE_MAX_AGE_MS) return;
    if (gestureDirection(this.hand, this.neutral, this.select.value) !== 0) { this.notice = 'Move back to neutral or set a new neutral before enabling.'; this.say(this.notice); return; }
    const epoch = ++this.epoch; this.enabling = true; this.render();
    try {
      await this.adapter.begin(this.select.value);
      if (epoch !== this.epoch) { await this.adapter.end(); return; }
      this.enabled = true; this.say('Enabled · center to hold, fist to pause.');
    } catch (error) { if (epoch === this.epoch) { this.neutral = null; this.notice = error.message; this.say(error.message); } }
    finally { if (epoch === this.epoch) { this.enabling = false; this.render(); } }
  }
  async pause(message = 'Paused. Set neutral again.') {
    const active = this.enabled || this.enabling;
    ++this.epoch; this.enabled = false; this.enabling = false; this.neutral = null; this.direction = 0;
    this.notice = message; this.say(message); this.render();
    if (active) {
      this.stopping = true; this.render();
      try { await this.adapter.end(); }
      catch (error) { this.say(error.message); }
      finally { this.stopping = false; this.render(); }
    }
  }
  async tick() {
    if (this.camera && (!this.lastFrameAt || performance.now() - this.lastFrameAt > GESTURE_MAX_AGE_MS)) {
      this.stable = false; this.stableSince = null; this.hand = null;
      if (this.enabled || this.enabling || this.neutral) await this.pause('Tracking lost · paused. Show one hand and set neutral again.');
      this.render();
    }
    if (!this.enabled || this.sending) return;
    this.sending = true;
    try { await this.adapter.send(this.select.value, this.direction * (this.reverse.checked ? -1 : 1)); }
    catch (error) { await this.pause(error.message); }
    finally { this.sending = false; }
  }
  draw() {
    const canvas = this.panel.querySelector('canvas'), ctx = canvas.getContext('2d'); ctx.clearRect(0, 0, 640, 480);
    if (!this.hand.valid) return;
    ctx.strokeStyle = this.enabled ? '#9bf1c2' : '#e8efed'; ctx.fillStyle = ctx.strokeStyle; ctx.lineWidth = 3;
    const p = this.hand.points;
    for (const chain of [[0,1,2,3,4],[0,5,6,7,8],[5,9,10,11,12],[9,13,14,15,16],[13,17,18,19,20],[0,17]]) {
      ctx.beginPath(); chain.forEach((id, i) => { const x = (1-p[id].x)*640, y = p[id].y*480; i ? ctx.lineTo(x,y) : ctx.moveTo(x,y); }); ctx.stroke();
    }
    for (const point of p) { ctx.beginPath(); ctx.arc((1-point.x)*640,point.y*480,4,0,Math.PI*2); ctx.fill(); }
    if (this.neutral && this.select.value !== 'gripper') { ctx.setLineDash([5,6]); for (const offset of [-.045,.045]) { const x=(this.neutral.x+offset)*640;ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,480);ctx.stroke(); } ctx.setLineDash([]); }
  }
}
