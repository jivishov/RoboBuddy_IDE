import { IdeEditor } from './editor.js';
import { PythonRuntime } from './python-runtime.js';
import { SourceRobotSimulator } from './source-simulator.js';
import { PROFILES, validateAction, fidelityNoticeFor, LEROBOT_REVISION } from './profiles.js';
import { TASK_PATCH_REVISION, defaultTaskId, isKinematicRigScenario, loadPatchedScenario, taskDescriptor, tasksForProfile } from './task-catalog.js';
import { buildPatchedWorkspace } from './task-workspace.js';
import { applyTheme, readStoredTheme, THEMES } from './themes.js';

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
    this.executionState = 'idle';
    this.pauseWaiter = null;
    this.problems = [];
    this.commands = [];
    this.console = { stdout: '', stderr: '' };
    this.theme = applyTheme(readStoredTheme(), { persist: false });
    this.runtime = new PythonRuntime();
    this.highContrastScene = true;
    this.sim = new SourceRobotSimulator($('simCanvas'));
    this.sim.setHighContrastScene(this.highContrastScene);
    this.renderHighContrastSceneControl();
    this.editor = new IdeEditor($('editor'), {
      onChange: (f, v) => this.onEdit(f, v),
      onSave: () => this.save(),
      onRun: () => this.run(),
      onCommandPalette: () => this.openPalette(),
      onCursor: (f, l, c) => $('statusCursor').textContent = `${f}:${l}:${c}`,
      theme: this.theme.editorTheme,
    });
    this.renderThemeOptions();
    this.bind();
    this.updateExecutionControls();
    this.setStatus('Loading robot workspace…');
    void this.loadProfile(this.profileId, { preserve: false, taskId: this.taskId });
  }

  storageKey() {
    const revision = this.scenario?.workspaceRevision || TASK_PATCH_REVISION.slice(0, 12);
    return `rbide.workspace.${revision}.${this.profileId}.${this.taskId}`;
  }

  usesSourcePlant() { return !isKinematicRigScenario(this.scenario); }

  isKinematicPoseWorkspace() { return isKinematicRigScenario(this.scenario); }

  updateSimulationPresentation(profile) {
    const kinematic = profile?.simulationMode === 'kinematic_pose';
    $('modeChip').textContent = kinematic
      ? 'KINEMATIC POSE RIG · NO CONTACT PLANT · HW VALIDATION PENDING'
      : 'SOURCE-PLANT SIMULATION · HW VALIDATION PENDING';
    $('simBadge').textContent = kinematic
      ? 'PINNED UNITREE G1 MESH / BOUNDED JOINT-POSE VIEW · NO CONTACT PLANT · CONFIGURED FLOOR · NOT HARDWARE VALIDATION'
      : 'PINNED ROBOBUDDY KINEMATICS / CONTACT PLANT · CONFIGURED FLOOR · NOT HARDWARE VALIDATION';
  }

  loadStored() {
    try { const raw = localStorage.getItem(this.storageKey()); return raw ? JSON.parse(raw) : null; }
    catch { return null; }
  }

  async loadProfile(id, { preserve = true, taskId = '' } = {}) {
    if (preserve && Object.keys(this.files).length) this.save(false);
    this.cancelExecution();
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
    this.updateSimulationPresentation(p);
    this.setStatus(p.simulationMode === 'kinematic_pose' ? 'Loading Unitree canonical pose workspace…' : 'Loading reviewed mission and source plant…');
    try {
      this.scenario = await loadPatchedScenario(id, this.taskId);
      if (!this.scenario) throw new Error(`No pinned source task is configured for ${id}.`);
      const starter = buildPatchedWorkspace(id, this.scenario);
      this.files = this.loadStored() || starter;
      this.currentFile = 'main.py';
      await this.sim.setScenario(id, this.isKinematicPoseWorkspace() ? null : this.scenario, p.rest);
      this.renderFiles();
      this.openFile('main.py');
      this.renderTask();
      this.renderPanels();
      const source = this.isKinematicPoseWorkspace()
        ? `canonical mesh ${this.scenario.canonicalModel.revision.slice(0, 12)}`
        : `source ${TASK_PATCH_REVISION.slice(0, 12)}`;
      this.setStatus(`Ready · ${this.scenario.title} · ${source}`);
    } catch (error) {
      this.scenario = null;
      this.files = { 'main.py': `# RoboBuddy workspace failed to load.\n# ${String(error.message || error)}\n` };
      this.renderFiles();
      this.openFile('main.py');
      this.problem('error', p.simulationMode === 'kinematic_pose' ? 'RIG_WORKSPACE' : 'SOURCE_TASK', String(error.message || error));
      this.setStatus(p.simulationMode === 'kinematic_pose' ? 'Unitree rig workspace unavailable' : 'Pinned source task unavailable');
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
    const kinematic = this.isKinematicPoseWorkspace();
    const labels = [];
    for (const item of scenario?.portablePython?.referenceActions || []) {
      const label = String(item.label || 'physical action');
      if (!labels.includes(label)) labels.push(label);
      if (labels.length >= 12) break;
    }
    const sourceLabel = kinematic ? 'Canonical mesh source' : 'Pinned task source';
    const sourceText = kinematic
      ? `RoboBuddy_AI@${scenario.canonicalModel.revision.slice(0, 12)} · Unitree URDF ${scenario.canonicalModel.sourceRevision.slice(0, 12)} · ${scenario.canonicalModel.license}`
      : `RoboBuddy_AI@${TASK_PATCH_REVISION.slice(0, 12)}`;
    $('taskPanel').innerHTML = `<h2>${escapeHtml(scenario?.title || p.task.title)}</h2><p>${escapeHtml(scenario?.brief || p.source)}</p><p><strong>${sourceLabel}:</strong> ${escapeHtml(sourceText)}</p><ol>${labels.map((label, index) => `<li class="${index === 0 ? 'task-current' : ''}">${escapeHtml(label)}</li>`).join('')}</ol><details><summary>Fidelity boundary</summary><p>${escapeHtml(p.task.limitations)}</p></details>`;
    $('fidelityText').textContent = kinematic
      ? `${fidelityNoticeFor(this.profileId)} ${p.task.limitations}`
      : `${fidelityNoticeFor(this.profileId)} LeRobot revision ${LEROBOT_REVISION}. Task definitions, reference actions, collision/contact plant, and support rules are pinned to RoboBuddy_AI revision ${TASK_PATCH_REVISION}. ${p.task.limitations}`;
    $('sideRobotSummary').textContent = kinematic
      ? `${p.label}. Canonical 29-joint mesh pose view; telemetry is browser-held joint state and contact values are intentionally unavailable.`
      : `${p.label}. Canonical RoboBuddy view; telemetry and contacts are modeled source-plant values, not hardware measurements.`;
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
    const prompt = this.isKinematicPoseWorkspace()
      ? 'Reset all files for this Unitree workspace to its browser-only kinematic-pose starter?'
      : 'Reset all files for this task to its pinned reviewed physical-Python starter?';
    if (!confirm(prompt)) return;
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
    if (cancel) this.cancelExecution();
    if (this.scenario) await this.sim.reset(this.profileId, this.isKinematicPoseWorkspace() ? null : this.scenario, PROFILES[this.profileId].rest);
    this.editor.highlightLine(null);
    this.stepIndex = 0;
    $('simActionLabel').textContent = 'Ready';
    this.renderPanels();
    this.setStatus('Simulation reset');
  }

  updateExecutionControls() {
    const active = this.executionState !== 'idle';
    const paused = this.executionState === 'paused';
    const pauseButton = $('pauseBtn');
    pauseButton.disabled = !active;
    pauseButton.textContent = paused ? '▶ Resume' : '⏸ Pause';
    pauseButton.title = paused ? 'Resume simulation' : 'Pause simulation';
    pauseButton.setAttribute('aria-label', paused ? 'Resume simulation' : 'Pause simulation');
    pauseButton.setAttribute('aria-pressed', String(paused));
    $('runBtn').disabled = active;
    $('stepBtn').disabled = active;
    $('cursorBtn').disabled = active;
  }

  beginExecution() {
    if (this.executionState !== 'idle') return null;
    const token = ++this.runToken;
    this.executionState = 'running';
    this.updateExecutionControls();
    return token;
  }

  finishExecution(token) {
    if (token !== this.runToken) return;
    this.executionState = 'idle';
    const waiter = this.pauseWaiter;
    this.pauseWaiter = null;
    waiter?.();
    this.updateExecutionControls();
  }

  cancelExecution() {
    this.runToken++;
    this.executionState = 'idle';
    const waiter = this.pauseWaiter;
    this.pauseWaiter = null;
    waiter?.();
    this.updateExecutionControls();
  }

  async waitForResume(token) {
    while (token === this.runToken && this.executionState === 'paused') {
      this.setStatus('Simulation paused');
      await new Promise((resolve) => { this.pauseWaiter = resolve; });
    }
    return token === this.runToken;
  }

  togglePause() {
    if (this.executionState === 'running') {
      this.executionState = 'paused';
      this.updateExecutionControls();
      this.setStatus('Simulation paused');
      return;
    }
    if (this.executionState !== 'paused') return;
    this.executionState = 'running';
    this.updateExecutionControls();
    this.setStatus('Simulation resumed');
    const waiter = this.pauseWaiter;
    this.pauseWaiter = null;
    waiter?.();
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
    if (!(await this.waitForResume(token))) return false;
    const beforeTick = () => this.waitForResume(token);
    if (event.kind === 'send_action') {
      const label = this.actionLabel(event.actionIndex || 0);
      if (this.currentFile === event.file) this.editor.highlightLine(event.line);
      else if (this.currentFile === 'trajectories.py') this.editor.highlightLine(this.trajectoryLine(event.actionIndex || 0));
      $('simActionLabel').textContent = `A${String((event.actionIndex || 0) + 1).padStart(2, '0')} · ${label}`;
      let applied;
      try { applied = await this.sim.applyAction(event.action, { beforeTick }); }
      catch (error) {
        this.problem('error', this.usesSourcePlant() ? 'COLLISION' : 'KINEMATIC_RIG', `${event.file}:${event.line} — ${error.message}`);
        this.setStatus(this.usesSourcePlant() ? 'Source plant rejected modeled motion' : 'Kinematic pose update failed');
        throw error;
      }
      if (applied === false) return false;
      this.renderPanels();
    } else if (event.kind === 'sleep' && honorSleep) {
      let advanced;
      try { advanced = await this.sim.advanceTime(event.seconds, { realtime: true, beforeTick }); }
      catch (error) {
        this.problem('error', this.usesSourcePlant() ? 'COLLISION' : 'KINEMATIC_RIG', `${event.file}:${event.line} — ${error.message}`);
        this.setStatus(this.usesSourcePlant() ? 'Source plant stopped at last valid state' : 'Kinematic pose timeline failed');
        throw error;
      }
      if (advanced === false) return false;
      this.renderPanels();
    }
    return token === this.runToken;
  }

  async run() {
    const token = this.beginExecution();
    if (token === null) return;
    let completed = false;
    try {
      await this.resetSimulation({ cancel: false });
      if (!(await this.waitForResume(token))) return;
      const prep = await this.prepare();
      if (!(await this.waitForResume(token))) return;
      this.stepIndex = 0;
      this.setStatus(this.usesSourcePlant() ? 'Running pinned source-plant simulation…' : 'Running Unitree kinematic pose sequence…');
      for (let i = 0; i < prep.events.length; i += 1) {
        if (!(await this._applyEvent(prep.events[i], token))) return;
        this.stepIndex = i + 1;
      }
      completed = true;
    } catch {
      return;
    } finally {
      if (completed && token === this.runToken) {
        this.editor.highlightLine(null);
        $('simActionLabel').textContent = 'Run complete';
        this.setStatus('Run complete');
        this.renderPanels();
      }
      this.finishExecution(token);
    }
  }

  async step() {
    if (this.executionState !== 'idle') return;
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
      if (!(await this._applyEvent(action, token, { honorSleep: false }))) return;
      while (this.stepIndex < events.length && events[this.stepIndex].kind !== 'send_action') {
        const event = events[this.stepIndex++];
        if (!(await this._applyEvent(event, token, { honorSleep: true }))) return;
      }
    } catch { return; }
    this.setStatus(`Stepped A${String((action.actionIndex || 0) + 1).padStart(2, '0')} · ${this.actionLabel(action.actionIndex || 0)}`);
  }

  async runToCursor() {
    const token = this.beginExecution();
    if (token === null) return;
    const file = this.currentFile;
    const line = this.editor.getCursorLine();
    let hit = false;
    try {
      await this.resetSimulation({ cancel: false });
      if (!(await this.waitForResume(token))) return;
      const prep = await this.prepare();
      if (!(await this.waitForResume(token))) return;
      this.setStatus(this.usesSourcePlant() ? 'Running pinned source-plant simulation to cursor…' : 'Running Unitree kinematic pose sequence to cursor…');
      for (let i = 0; i < prep.events.length; i += 1) {
        const event = prep.events[i];
        if (event.kind === 'send_action' && event.file === file && event.line > line) break;
        if (!(await this._applyEvent(event, token))) return;
        this.stepIndex = i + 1;
        if (event.kind === 'send_action' && event.file === file && event.line === line) hit = true;
      }
      if (token === this.runToken) this.setStatus(hit ? `Stopped at ${file}:${line}` : `Ran commands through ${file}:${line}`);
    } catch {
      return;
    } finally {
      this.finishExecution(token);
    }
  }

  stop() {
    this.cancelExecution();
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
    const kinematic = this.isKinematicPoseWorkspace();
    const telemetry = this.sim.getTelemetry();
    const telemetryNote = kinematic
      ? 'BROWSER-HELD KINEMATIC G1 JOINT STATE — not measured telemetry, controller state, or a physical robot observation.'
      : 'SIMULATED ACTUAL STATE FROM THE PINNED ROBObUDDY FIXED-STEP PLANT — not measured hardware telemetry.';
    $('telemetryPanel').innerHTML = `<div class="panel-note">${telemetryNote}</div><table><tr><th>Field</th><th>Modeled value</th></tr>${Object.entries(telemetry).map(([key, value]) => `<tr><td>${escapeHtml(key)}</td><td>${Number(value).toFixed(3)}</td></tr>`).join('')}</table>`;
    $('commandsPanel').innerHTML = this.commands.length ? this.commands.map((command, index) => `<div class="command-row ${index === this.stepIndex - 1 ? 'active' : ''}"><span>${index + 1}</span><span>${escapeHtml(this.actionLabel(index))}</span><code>${escapeHtml(JSON.stringify(command.action))}</code><span>${kinematic ? 'kinematic pose' : 'physical target'}</span></div>`).join('') : `<div class="empty-state">Run or Step Action to prepare the ${kinematic ? 'kinematic pose' : 'physical command'} queue.</div>`;
    const contacts = this.sim.getContacts();
    const contactsNote = kinematic
      ? 'G1 CONTACT / SUPPORT IS NOT SIMULATED. This panel reports the explicit kinematic boundary, not physical contact data.'
      : 'MODELED CONTACT / SUPPORT STATE FROM THE PINNED SOURCE PLANT — no force, torque, current, or tactile sensor data.';
    $('contactsPanel').innerHTML = `<div class="panel-note">${contactsNote}</div><div class="metric-grid">${Object.entries(contacts).map(([key, value]) => `<span>${escapeHtml(key)}</span><strong>${typeof value === 'number' ? value.toFixed(3) : escapeHtml(String(value))}</strong>`).join('')}</div>`;
  }

  openBottom(name) {
    $('app').classList.add('bottom-open');
    $('bottomPanel').classList.remove('collapsed');
    document.querySelectorAll('.bottom-tab').forEach((button) => button.classList.toggle('active', button.dataset.panel === name));
    document.querySelectorAll('.panel-view').forEach((view) => { view.hidden = true; });
    const map = { problems: 'problemsPanel', telemetry: 'telemetryPanel', commands: 'commandsPanel', contacts: 'contactsPanel', task: 'taskBottomPanel' };
    $(map[name]).hidden = false;
    this.renderPanels();
    setTimeout(() => { this.editor.refresh(); this.sim.resize(); }, 30);
  }

  closeBottom() {
    $('bottomPanel').classList.add('collapsed');
    $('app').classList.remove('bottom-open');
    setTimeout(() => { this.editor.refresh(); this.sim.resize(); }, 30);
  }

  openSideView(name) {
    const valid = ['explorer', 'task', 'robot'];
    if (!valid.includes(name)) return;
    const workspace = $('workspace');
    workspace.classList.remove('sidebar-collapsed');
    if (window.matchMedia('(max-width:800px)').matches) workspace.classList.add('side-drawer-open');
    document.querySelectorAll('[data-side-view]').forEach((button) => button.classList.toggle('active', button.dataset.sideView === name));
    for (const view of valid) $(`${view}SideView`).hidden = view !== name;
    setTimeout(() => this.editor.refresh(), 30);
  }

  toggleSidebar() {
    const workspace = $('workspace');
    if (window.matchMedia('(max-width:800px)').matches) workspace.classList.toggle('side-drawer-open');
    else workspace.classList.toggle('sidebar-collapsed');
    setTimeout(() => this.editor.refresh(), 30);
  }
  togglePanel() { const b = $('bottomPanel'); if (b.classList.contains('collapsed')) this.openBottom('problems'); else this.closeBottom(); }
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

  setTheme(themeId) {
    this.theme = applyTheme(themeId);
    this.editor.setTheme(this.theme.editorTheme);
    this.renderThemeOptions();
  }

  renderHighContrastSceneControl() {
    const button = $('highContrastSceneBtn');
    if (!button) return;
    button.classList.toggle('active', this.highContrastScene);
    button.setAttribute('aria-pressed', String(this.highContrastScene));
    button.setAttribute('aria-label', this.highContrastScene ? 'Disable high-contrast scene boundaries' : 'Enable high-contrast scene boundaries');
    button.title = this.highContrastScene
      ? 'High-contrast scene is on. Its configured floor-contact boundaries do not alter collision, kinematics, or canonical mesh data.'
      : 'High-contrast scene is off. Enable configured floor-contact boundaries without changing collision, kinematics, or canonical mesh data.';
  }

  toggleHighContrastScene() {
    this.highContrastScene = this.sim.setHighContrastScene(!this.highContrastScene);
    this.renderHighContrastSceneControl();
    this.setStatus(this.highContrastScene ? 'High-contrast scene enabled (presentation only)' : 'High-contrast scene disabled');
  }

  renderThemeOptions() {
    document.querySelectorAll('[data-theme-id]').forEach((button) => {
      const active = button.dataset.themeId === this.theme.id;
      button.classList.toggle('active', active);
      button.setAttribute('aria-checked', String(active));
    });
  }

  closeMenus() {
    document.querySelectorAll('.menu-popover').forEach((menu) => { menu.hidden = true; });
    document.querySelectorAll('.menu-button').forEach((button) => button.setAttribute('aria-expanded', 'false'));
  }

  openPalette() { const box = $('commandPalette'); box.hidden = false; $('commandInput').value = ''; this.renderPalette(''); setTimeout(() => $('commandInput').focus(), 0); }
  openAbout() { this.closeMenus(); const dialog = $('aboutDialog'); if (!dialog?.open) dialog?.showModal(); setTimeout(() => $('aboutCloseBtn')?.focus(), 0); }
  renderPalette(q) { const commands = this.commandsList().filter((c) => c.label.toLowerCase().includes(q.toLowerCase())); $('commandList').innerHTML = ''; commands.forEach((c) => { const b = document.createElement('button'); b.textContent = c.label; b.onclick = () => { $('commandPalette').hidden = true; c.run(); }; $('commandList').appendChild(b); }); }
  commandsList() {
    const themeCommands = Object.values(THEMES).map((theme) => ({ label: `Preferences: Color Theme — ${theme.label}`, run: () => this.setTheme(theme.id) }));
    return [{ label: 'Run: Run simulation', run: () => this.run() }, { label: 'Run: Step physical action', run: () => this.step() }, { label: 'Run: Run to cursor', run: () => this.runToCursor() }, { label: 'View: Toggle Explorer', run: () => this.toggleSidebar() }, { label: 'View: Toggle diagnostics panel', run: () => this.togglePanel() }, { label: 'View: Toggle high-contrast scene', run: () => this.toggleHighContrastScene() }, { label: 'View: Fit simulator', run: () => this.sim.fit() }, { label: 'Robot: Contact diagnostics', run: () => this.openBottom('contacts') }, { label: 'Robot: Simulated telemetry', run: () => this.openBottom('telemetry') }, { label: 'Help: About RoboBuddy IDE', run: () => this.openAbout() }, { label: 'File: Save draft', run: () => this.save() }, { label: 'File: Export workspace', run: () => this.exportWorkspace() }, ...themeCommands];
  }

  dispatch(action) {
    const map = { import: () => $('importFile').click(), save: () => this.save(), exportMain: () => this.download('main.py', this.files['main.py']), exportWorkspace: () => this.exportWorkspace(), resetWorkspace: () => this.resetWorkspace(), undo: () => this.editor.undo(), redo: () => this.editor.redo(), find: () => this.editor.find(), replace: () => this.editor.replace(), toggleComment: () => this.editor.toggleComment(), palette: () => this.openPalette(), run: () => this.run(), step: () => this.step(), cursor: () => this.runToCursor(), stop: () => this.stop(), reset: () => this.resetSimulation(), sidebar: () => this.toggleSidebar(), panel: () => this.togglePanel(), highContrastScene: () => this.toggleHighContrastScene(), editorFocus: () => this.editor.focus(), simulatorFocus: () => $('simCanvas').focus(), fit: () => this.sim.fit(), contacts: () => this.openBottom('contacts'), telemetry: () => this.openBottom('telemetry'), api: () => this.openBottom('task'), shortcuts: () => this.openBottom('task'), fidelity: () => this.openBottom('task'), about: () => this.openAbout() };
    map[action]?.();
  }

  bind() {
    $('robotSelect').onchange = (event) => void this.loadProfile(event.target.value);
    $('taskSelect').onchange = (event) => { localStorage.setItem(`rbide.task.${this.profileId}`, event.target.value); void this.loadProfile(this.profileId, { taskId: event.target.value }); };
    $('runBtn').onclick = () => void this.run(); $('pauseBtn').onclick = () => this.togglePause(); $('stepBtn').onclick = () => void this.step(); $('cursorBtn').onclick = () => void this.runToCursor(); $('stopBtn').onclick = () => this.stop(); $('resetBtn').onclick = () => void this.resetSimulation(); $('fitBtn').onclick = () => this.sim.fit(); $('highContrastSceneBtn').onclick = () => this.toggleHighContrastScene(); $('panelToggle').onclick = () => this.togglePanel(); $('sidebarToggle').onclick = () => this.toggleSidebar(); $('bottomClose').onclick = () => this.closeBottom();
    document.querySelectorAll('[data-side-view]').forEach((button) => button.onclick = () => this.openSideView(button.dataset.sideView));
    document.querySelectorAll('[data-side-action]').forEach((button) => button.onclick = () => { const action = button.dataset.sideAction; if (action === 'front') this.sim.fit(); else if (action === 'telemetry') this.openBottom('telemetry'); else if (action === 'contacts') this.openBottom('contacts'); });
    $('mobileCodeBtn').onclick = () => { $('workspace').classList.remove('show-sim'); $('mobileCodeBtn').classList.add('active'); $('mobileSimBtn').classList.remove('active'); setTimeout(() => this.editor.refresh(), 20); };
    $('mobileSimBtn').onclick = () => { $('workspace').classList.add('show-sim'); $('mobileCodeBtn').classList.remove('active'); $('mobileSimBtn').classList.add('active'); };
    document.querySelectorAll('.bottom-tab').forEach((b) => b.onclick = () => this.openBottom(b.dataset.panel));
    document.querySelectorAll('.menu-button').forEach((button) => button.onclick = (event) => { event.stopPropagation(); const menu = $(`${button.dataset.menu}Menu`); const open = menu.hidden; this.closeMenus(); menu.hidden = !open; button.setAttribute('aria-expanded', String(open)); });
    document.querySelectorAll('[data-action]').forEach((button) => button.onclick = (event) => { event.stopPropagation(); this.closeMenus(); this.dispatch(button.dataset.action); });
    document.querySelectorAll('[data-theme-id]').forEach((button) => button.onclick = (event) => { event.stopPropagation(); this.closeMenus(); this.setTheme(button.dataset.themeId); });
    document.addEventListener('click', () => this.closeMenus());
    $('commandClose').onclick = () => $('commandPalette').hidden = true; $('commandInput').oninput = (event) => this.renderPalette(event.target.value); $('commandPalette').onclick = (event) => { if (event.target === $('commandPalette')) $('commandPalette').hidden = true; };
    $('aboutCloseBtn').onclick = () => $('aboutDialog').close();
    $('importFile').onchange = (event) => { const file = event.target.files?.[0]; if (file) this.importFile(file); event.target.value = ''; };
    document.addEventListener('keydown', (event) => {
      if (event.key === 'Escape') { const aboutDialog = $('aboutDialog'); if (aboutDialog?.open) { event.preventDefault(); aboutDialog.close(); return; } if (!$('commandPalette').hidden) { $('commandPalette').hidden = true; return; } if (document.querySelector('.menu-popover:not([hidden])')) { this.closeMenus(); return; } this.stop(); return; }
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
