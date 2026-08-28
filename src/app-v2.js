import { IdeEditor } from './editor.js';
import { PythonRuntime } from './python-runtime.js';
import { SourceRobotSimulator } from './source-simulator.js';
import { PROFILES, validateAction, FIDELITY_NOTICE, LEROBOT_REVISION } from './profiles.js';
import { TASK_PATCH_REVISION, defaultTaskId, loadPatchedScenario, taskDescriptor, tasksForProfile } from './task-catalog.js';
import { buildPatchedWorkspace } from './task-workspace.js';

const $ = (id) => document.getElementById(id);
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

class App {
  constructor() {
    this.profileId = localStorage.getItem('rbide.profile') || 'openarm';
    if (!PROFILES[this.profileId]) this.profileId = 'openarm';
    this.taskId = localStorage.getItem(`rbide.task.${this.profileId}`) || defaultTaskId(this.profileId);
    this.scenario = null;
    this.files = {};
    this.currentFile = 'main.py';
    this.dirty = new Set();
    this.prepared = null;
    this.stepIndex = 0;
    this.runToken = 0;
    this.problems = [];
    this.commands = [];
    this.console = { stdout: '', stderr: '' };
    this.runtime = new PythonRuntime();
    this.sim = new SourceRobotSimulator($('simCanvas'));
    this.editor = new IdeEditor($('editor'), {
      onChange: (f, v) => this.onEdit(f, v),
      onSave: () => this.save(),
      onRun: () => this.run(),
      onCommandPalette: () => this.openPalette(),
      onCursor: (f, l, c) => $('statusCursor').textContent = `${f}:${l}:${c}`,
    });
    this.bind();
    this.setStatus('Loading pinned task…');
    void this.loadProfile(this.profileId, { preserve: false, taskId: this.taskId });
  }

  storageKey() {
    return `rbide.workspace.${TASK_PATCH_REVISION.slice(0, 12)}.${this.profileId}.${this.taskId}`;
  }

  loadStored() {
    try { const raw = localStorage.getItem(this.storageKey()); return raw ? JSON.parse(raw) : null; }
    catch { return null; }
  }

  async loadProfile(id, { preserve = true, taskId = '' } = {}) {
    if (preserve && Object.keys(this.files).length) this.save(false);
    this.runToken++;
    this.profileId = id;
    localStorage.setItem('rbide.profile', id);
    const options = tasksForProfile(id);
    this.taskId = taskId && options.some((item) => item.id === taskId) ? taskId : (localStorage.getItem(`rbide.task.${id}`) || defaultTaskId(id));
    if (!options.some((item) => item.id === this.taskId)) this.taskId = defaultTaskId(id);
    localStorage.setItem(`rbide.task.${id}`, this.taskId);
    this.renderTaskSelector();
    this.problems = [];
    this.commands = [];
    this.console = { stdout: '', stderr: '' };
    this.prepared = null;
    this.stepIndex = 0;
    this.dirty.clear();
    $('robotSelect').value = id;
    const p = PROFILES[id];
    $('robotLabel').textContent = p.label;
    $('driverLabel').textContent = p.driver;
    $('driverStatus').textContent = p.driver;
    this.setStatus('Loading reviewed mission and source plant…');
    try {
      this.scenario = await loadPatchedScenario(id, this.taskId);
      if (!this.scenario) throw new Error(`No pinned source task is configured for ${id}.`);
      const starter = buildPatchedWorkspace(id, this.scenario);
      this.files = this.loadStored() || starter;
      this.currentFile = 'main.py';
      await this.sim.setScenario(id, this.scenario, p.rest);
      this.renderFiles();
      this.openFile('main.py');
      this.renderTask();
      this.renderPanels();
      this.setStatus(`Ready · ${this.scenario.title} · source ${TASK_PATCH_REVISION.slice(0, 12)}`);
    } catch (error) {
      this.scenario = null;
      this.files = { 'main.py': `# Pinned RoboBuddy task failed to load.\n# ${String(error.message || error)}\n` };
      this.renderFiles();
      this.openFile('main.py');
      this.problem('error', 'SOURCE_TASK', String(error.message || error));
      this.setStatus('Pinned source task unavailable');
    }
  }

  renderTaskSelector() {
    const select = $('taskSelect');
    select.innerHTML = '';
    for (const item of tasksForProfile(this.profileId)) {
      const option = document.createElement('option');
      option.value = item.id;
      option.textContent = item.title;
      select.appendChild(option);
    }
    select.value = this.taskId;
  }

  onEdit(file, value) {
    this.files[file] = value;
    this.dirty.add(file);
    this.prepared = null;
    $('dirtyDot').hidden = false;
    this.renderFiles();
  }

  openFile(name) {
    if (!Object.hasOwn(this.files, name)) return;
    this.currentFile = name;
    this.editor.setFile(name, this.files[name]);
    this.renderFiles();
    this.renderTabs();
  }

  renderFiles() {
    const el = $('filesList');
    el.innerHTML = '';
    Object.keys(this.files).forEach((name) => {
      const b = document.createElement('button');
      b.className = `file-item${name === this.currentFile ? ' active' : ''}`;
      b.innerHTML = `<span class="py-icon">PY</span><span>${name}</span>${this.dirty.has(name) ? '<span>●</span>' : ''}`;
      b.onclick = () => this.openFile(name);
      el.appendChild(b);
    });
    this.renderTabs();
  }

  renderTabs() {
    const el = $('editorTabs');
    el.innerHTML = '';
    Object.keys(this.files).forEach((name) => {
      const b = document.createElement('button');
      b.className = `editor-tab${name === this.currentFile ? ' active' : ''}`;
      b.textContent = `${name}${this.dirty.has(name) ? ' ●' : ''}`;
      b.onclick = () => this.openFile(name);
      el.appendChild(b);
    });
  }

  renderTask() {
    const p = PROFILES[this.profileId];
    const scenario = this.scenario;
    const labels = [];
    for (const item of scenario?.portablePython?.referenceActions || []) {
      const label = String(item.label || 'physical action');
      if (!labels.includes(label)) labels.push(label);
      if (labels.length >= 12) break;
    }
    $('taskPanel').innerHTML = `<h2>${escapeHtml(scenario?.title || p.task.title)}</h2><p>${escapeHtml(scenario?.brief || p.source)}</p><p><strong>Pinned task source:</strong> RoboBuddy_AI@${TASK_PATCH_REVISION.slice(0, 12)}</p><ol>${labels.map((label, index) => `<li class="${index === 0 ? 'task-current' : ''}">${escapeHtml(label)}</li>`).join('')}</ol><details><summary>Fidelity boundary</summary><p>${escapeHtml(p.task.limitations)}</p></details>`;
    $('fidelityText').textContent = `${FIDELITY_NOTICE} LeRobot revision ${LEROBOT_REVISION}. Task definitions, reference actions, collision/contact plant, and support rules are pinned to RoboBuddy_AI revision ${TASK_PATCH_REVISION}. ${p.task.limitations}`;
  }

  save(show = true) {
    if (!this.taskId) return;
    localStorage.setItem(this.storageKey(), JSON.stringify(this.files));
    this.dirty.clear();
    $('dirtyDot').hidden = true;
    this.renderFiles();
    if (show) this.setStatus('Draft saved locally');
  }

  async resetWorkspace() {
    if (!confirm('Reset all files for this task to its pinned reviewed physical-Python starter?')) return;
    localStorage.removeItem(this.storageKey());
    if (!this.scenario) return;
    this.files = buildPatchedWorkspace(this.profileId, this.scenario);
    this.dirty.clear();
    this.openFile('main.py');
    this.prepared = null;
    await this.resetSimulation();
    this.renderFiles();
  }

  async prepare() {
    this.setStatus('Preparing Python…');
    this.problems = [];
    this.commands = [];
    this.renderPanels();
    let result;
    try { result = await this.runtime.compileWorkspace(this.files); }
    catch (error) {
      this.problem('error', 'PYODIDE', error.message);
      this.setStatus('Python runtime error');
      throw error;
    }
    this.console = { stdout: result.stdout || '', stderr: result.stderr || '' };
    if (result.exception) {
      this.problem('error', 'PYTHON', result.exception);
      this.setStatus('Python error');
      this.renderPanels();
      throw new Error('Python execution failed');
    }
    const events = [];
    let actionIndex = 0;
    for (const event of result.events || []) {
      if (event.kind === 'send_action') {
        try { event.action = validateAction(this.profileId, event.action); }
        catch (error) {
          this.problem('error', 'ACTION', `${event.file}:${event.line} — ${error.message}`);
          this.setStatus('Action rejected');
          this.renderPanels();
          throw error;
        }
        event.actionIndex = actionIndex++;
      }
      events.push(event);
    }
    this.prepared = { events };
    this.stepIndex = 0;
    this.commands = events.filter((event) => event.kind === 'send_action');
    this.renderPanels();
    this.setStatus(`${this.commands.length} physical actions prepared`);
    return this.prepared;
  }

  async resetSimulation({ cancel = true } = {}) {
    if (cancel) this.runToken++;
    if (this.scenario) await this.sim.reset(this.profileId, this.scenario, PROFILES[this.profileId].rest);
    this.editor.highlightLine(null);
    this.stepIndex = 0;
    $('simActionLabel').textContent = 'Ready';
    this.renderPanels();
    this.setStatus('Simulation reset');
  }

  trajectoryLine(index) {
    const lines = String(this.files['trajectories.py'] || '').split('\n');
    const needle = `"index": ${index + 1},`;
    const found = lines.findIndex((line) => line.includes(needle));
    return found >= 0 ? found + 1 : null;
  }

  actionLabel(index) {
    return String(this.scenario?.portablePython?.referenceActions?.[index]?.label || `physical action ${index + 1}`);
  }

  async _applyEvent(event, token, { honorSleep = true } = {}) {
    if (token !== this.runToken) return false;
    if (event.kind === 'send_action') {
      const label = this.actionLabel(event.actionIndex || 0);
      if (this.currentFile === event.file) this.editor.highlightLine(event.line);
      else if (this.currentFile === 'trajectories.py') this.editor.highlightLine(this.trajectoryLine(event.actionIndex || 0));
      $('simActionLabel').textContent = `A${String((event.actionIndex || 0) + 1).padStart(2, '0')} · ${label}`;
      try { await this.sim.applyAction(event.action); }
      catch (error) {
        this.problem('error', 'COLLISION', `${event.file}:${event.line} — ${error.message}`);
        this.setStatus('Source plant rejected modeled motion');
        throw error;
      }
      this.renderPanels();
    } else if (event.kind === 'sleep' && honorSleep) {
      try { await this.sim.advanceTime(event.seconds, { realtime: true }); }
      catch (error) {
        this.problem('error', 'COLLISION', `${event.file}:${event.line} — ${error.message}`);
        this.setStatus('Source plant stopped at last valid state');
        throw error;
      }
      this.renderPanels();
    }
    return token === this.runToken;
  }

  async run() {
    const token = ++this.runToken;
    await this.resetSimulation({ cancel: false });
    let prep;
    try { prep = await this.prepare(); } catch { return; }
    this.stepIndex = 0;
    this.setStatus('Running pinned source-plant simulation…');
    try {
      for (let i = 0; i < prep.events.length; i += 1) {
        if (!(await this._applyEvent(prep.events[i], token))) return;
        this.stepIndex = i + 1;
      }
    } catch { return; }
    this.editor.highlightLine(null);
    $('simActionLabel').textContent = 'Run complete';
    this.setStatus('Run complete');
    this.renderPanels();
  }

  async step() {
    if (!this.prepared) {
      await this.resetSimulation();
      try { await this.prepare(); } catch { return; }
    }
    const events = this.prepared.events;
    while (this.stepIndex < events.length && events[this.stepIndex].kind !== 'send_action') this.stepIndex += 1;
    if (this.stepIndex >= events.length) { this.setStatus('No more physical actions'); return; }
    const token = this.runToken;
    const action = events[this.stepIndex++];
    try {
      await this._applyEvent(action, token, { honorSleep: false });
      while (this.stepIndex < events.length && events[this.stepIndex].kind !== 'send_action') {
        const event = events[this.stepIndex++];
        await this._applyEvent(event, token, { honorSleep: true });
      }
    } catch { return; }
    this.setStatus(`Stepped A${String((action.actionIndex || 0) + 1).padStart(2, '0')} · ${this.actionLabel(action.actionIndex || 0)}`);
  }

  async runToCursor() {
    const file = this.currentFile;
    const line = this.editor.getCursorLine();
    const token = ++this.runToken;
    await this.resetSimulation({ cancel: false });
    let prep;
    try { prep = await this.prepare(); } catch { return; }
    let hit = false;
    try {
      for (let i = 0; i < prep.events.length; i += 1) {
        const event = prep.events[i];
        if (event.kind === 'send_action' && event.file === file && event.line > line) break;
        if (!(await this._applyEvent(event, token))) return;
        this.stepIndex = i + 1;
        if (event.kind === 'send_action' && event.file === file && event.line === line) hit = true;
      }
    } catch { return; }
    this.setStatus(hit ? `Stopped at ${file}:${line}` : `Ran commands through ${file}:${line}`);
  }

  stop() {
    this.runToken++;
    this.editor.highlightLine(null);
    $('simActionLabel').textContent = 'Stopped';
    this.setStatus('Simulation stopped');
  }

  problem(level, code, message) {
    this.problems.push({ level, code, message });
    this.openBottom('problems');
  }

  renderPanels() {
    const problems = $('problemsPanel');
    problems.innerHTML = this.problems.length ? this.problems.map((item) => `<div class="problem ${item.level}"><strong>${item.code}</strong><div>${escapeHtml(item.message).replace(/\n/g, '<br>')}</div></div>`).join('') : '<div class="empty-state">No problems.</div>';
    if (this.console.stdout || this.console.stderr) problems.innerHTML += `<div class="console-block">${this.console.stdout.split('\n').filter(Boolean).map((line) => `<div class="console-line">${escapeHtml(line)}</div>`).join('')}${this.console.stderr.split('\n').filter(Boolean).map((line) => `<div class="console-line stderr">${escapeHtml(line)}</div>`).join('')}</div>`;
    const telemetry = this.sim.getTelemetry();
    $('telemetryPanel').innerHTML = `<div class="panel-note">SIMULATED ACTUAL STATE FROM THE PINNED ROBObUDDY FIXED-STEP PLANT — not measured hardware telemetry.</div><table><tr><th>Field</th><th>Modeled value</th></tr>${Object.entries(telemetry).map(([key, value]) => `<tr><td>${escapeHtml(key)}</td><td>${Number(value).toFixed(3)}</td></tr>`).join('')}</table>`;
    $('commandsPanel').innerHTML = this.commands.length ? this.commands.map((command, index) => `<div class="command-row ${index === this.stepIndex - 1 ? 'active' : ''}"><span>${index + 1}</span><span>${escapeHtml(this.actionLabel(index))}</span><code>${escapeHtml(JSON.stringify(command.action))}</code><span>physical target</span></div>`).join('') : '<div class="empty-state">Run or Step Action to prepare the physical command queue.</div>';
    const contacts = this.sim.getContacts();
    $('contactsPanel').innerHTML = `<div class="panel-note">MODELED CONTACT / SUPPORT STATE FROM THE PINNED SOURCE PLANT — no force, torque, current, or tactile sensor data.</div><div class="metric-grid">${Object.entries(contacts).map(([key, value]) => `<span>${escapeHtml(key)}</span><strong>${typeof value === 'number' ? value.toFixed(3) : escapeHtml(String(value))}</strong>`).join('')}</div>`;
  }

  openBottom(name) {
    $('bottomPanel').classList.remove('collapsed');
    document.querySelectorAll('.bottom-tab').forEach((button) => button.classList.toggle('active', button.dataset.panel === name));
    document.querySelectorAll('.panel-view').forEach((view) => { view.hidden = true; });
    const map = { problems: 'problemsPanel', telemetry: 'telemetryPanel', commands: 'commandsPanel', contacts: 'contactsPanel', task: 'taskBottomPanel' };
    $(map[name]).hidden = false;
    this.renderPanels();
  }

  toggleSidebar() { $('workspace').classList.toggle('sidebar-collapsed'); setTimeout(() => this.editor.refresh(), 30); }
  togglePanel() { const b = $('bottomPanel'); if (b.classList.contains('collapsed')) this.openBottom('problems'); else b.classList.add('collapsed'); setTimeout(() => this.editor.refresh(), 30); }
  setStatus(text) { $('statusMessage').textContent = text; }

  download(name, text, type = 'text/plain') {
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([text], { type }));
    a.download = name;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  }

  exportWorkspace() {
    const text = Object.entries(this.files).map(([name, content]) => `# ===== ${name} =====\n${content}`).join('\n\n');
    this.download(`${this.profileId}-${this.taskId}-workspace.py`, text);
  }

  importFile(file) {
    const reader = new FileReader();
    reader.onload = () => {
      const name = file.name.endsWith('.py') ? file.name : 'main.py';
      this.files[name] = String(reader.result);
      this.dirty.add(name);
      this.openFile(name);
      this.renderFiles();
    };
    reader.readAsText(file);
  }

  openPalette() { const box = $('commandPalette'); box.hidden = false; $('commandInput').value = ''; this.renderPalette(''); setTimeout(() => $('commandInput').focus(), 0); }
  renderPalette(q) { const commands = this.commandsList().filter((c) => c.label.toLowerCase().includes(q.toLowerCase())); $('commandList').innerHTML = ''; commands.forEach((c) => { const b = document.createElement('button'); b.textContent = c.label; b.onclick = () => { $('commandPalette').hidden = true; c.run(); }; $('commandList').appendChild(b); }); }
  commandsList() { return [{ label: 'Run: Run simulation', run: () => this.run() }, { label: 'Run: Step physical action', run: () => this.step() }, { label: 'Run: Run to cursor', run: () => this.runToCursor() }, { label: 'View: Toggle Explorer', run: () => this.toggleSidebar() }, { label: 'View: Toggle diagnostics panel', run: () => this.togglePanel() }, { label: 'View: Fit simulator', run: () => this.sim.fit() }, { label: 'Robot: Contact diagnostics', run: () => this.openBottom('contacts') }, { label: 'Robot: Simulated telemetry', run: () => this.openBottom('telemetry') }, { label: 'File: Save draft', run: () => this.save() }, { label: 'File: Export workspace', run: () => this.exportWorkspace() }]; }

  dispatch(action) {
    const map = { import: () => $('importFile').click(), save: () => this.save(), exportMain: () => this.download('main.py', this.files['main.py']), exportWorkspace: () => this.exportWorkspace(), resetWorkspace: () => this.resetWorkspace(), undo: () => this.editor.undo(), redo: () => this.editor.redo(), find: () => this.editor.find(), replace: () => this.editor.replace(), toggleComment: () => this.editor.toggleComment(), palette: () => this.openPalette(), run: () => this.run(), step: () => this.step(), cursor: () => this.runToCursor(), stop: () => this.stop(), reset: () => this.resetSimulation(), sidebar: () => this.toggleSidebar(), panel: () => this.togglePanel(), editorFocus: () => this.editor.focus(), simulatorFocus: () => $('simCanvas').focus(), fit: () => this.sim.fit(), contacts: () => this.openBottom('contacts'), telemetry: () => this.openBottom('telemetry'), api: () => this.openBottom('task'), shortcuts: () => this.openBottom('task'), fidelity: () => this.openBottom('task') };
    map[action]?.();
  }

  bind() {
    $('robotSelect').onchange = (event) => void this.loadProfile(event.target.value);
    $('taskSelect').onchange = (event) => { localStorage.setItem(`rbide.task.${this.profileId}`, event.target.value); void this.loadProfile(this.profileId, { taskId: event.target.value }); };
    $('runBtn').onclick = () => void this.run(); $('stepBtn').onclick = () => void this.step(); $('cursorBtn').onclick = () => void this.runToCursor(); $('stopBtn').onclick = () => this.stop(); $('resetBtn').onclick = () => void this.resetSimulation(); $('fitBtn').onclick = () => this.sim.fit(); $('panelToggle').onclick = () => this.togglePanel(); $('sidebarToggle').onclick = () => this.toggleSidebar(); $('bottomClose').onclick = () => $('bottomPanel').classList.add('collapsed');
    $('mobileCodeBtn').onclick = () => { $('workspace').classList.remove('show-sim'); $('mobileCodeBtn').classList.add('active'); $('mobileSimBtn').classList.remove('active'); setTimeout(() => this.editor.refresh(), 20); };
    $('mobileSimBtn').onclick = () => { $('workspace').classList.add('show-sim'); $('mobileCodeBtn').classList.remove('active'); $('mobileSimBtn').classList.add('active'); };
    document.querySelectorAll('.bottom-tab').forEach((b) => b.onclick = () => this.openBottom(b.dataset.panel));
    document.querySelectorAll('.menu-button').forEach((b) => b.onclick = (event) => { event.stopPropagation(); const id = `${b.dataset.menu}Menu`; document.querySelectorAll('.menu-popover').forEach((m) => { if (m.id !== id) m.hidden = true; }); $(id).hidden = !$(id).hidden; });
    document.querySelectorAll('[data-action]').forEach((b) => b.onclick = (event) => { event.stopPropagation(); document.querySelectorAll('.menu-popover').forEach((m) => { m.hidden = true; }); this.dispatch(b.dataset.action); });
    document.addEventListener('click', () => document.querySelectorAll('.menu-popover').forEach((m) => { m.hidden = true; }));
    $('commandClose').onclick = () => $('commandPalette').hidden = true; $('commandInput').oninput = (event) => this.renderPalette(event.target.value); $('commandPalette').onclick = (event) => { if (event.target === $('commandPalette')) $('commandPalette').hidden = true; };
    $('importFile').onchange = (event) => { const file = event.target.files?.[0]; if (file) this.importFile(file); event.target.value = ''; };
    document.addEventListener('keydown', (event) => {
      if (event.key === 'Escape') { if (!$('commandPalette').hidden) { $('commandPalette').hidden = true; return; } this.stop(); return; }
      if (event.key === 'F10' && !event.ctrlKey) { event.preventDefault(); void this.step(); }
      if (event.key === 'F5') { event.preventDefault(); if (event.shiftKey) this.stop(); else void this.run(); }
      if (event.ctrlKey && event.key === 'F10') { event.preventDefault(); void this.runToCursor(); }
      if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === 'b') { event.preventDefault(); this.toggleSidebar(); }
      if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === 'j') { event.preventDefault(); this.togglePanel(); }
    });
    let start = null;
    $('splitter').addEventListener('pointerdown', (event) => { start = { x: event.clientX, pct: parseFloat(getComputedStyle(document.documentElement).getPropertyValue('--editor-pct')) || 52 }; document.body.classList.add('resizing'); $('splitter').setPointerCapture(event.pointerId); });
    $('splitter').addEventListener('pointermove', (event) => { if (!start) return; const rect = $('mainPanes').getBoundingClientRect(); const pct = clamp(start.pct + ((event.clientX - start.x) / Math.max(1, rect.width)) * 100, 30, 75); document.documentElement.style.setProperty('--editor-pct', `${pct}%`); this.editor.refresh(); });
    const end = () => { start = null; document.body.classList.remove('resizing'); };
    $('splitter').addEventListener('pointerup', end); $('splitter').addEventListener('pointercancel', end);
  }
}

function escapeHtml(value) { return String(value ?? '').replace(/[&<>"']/g, (char) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[char])); }

new App();
