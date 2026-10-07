import { IdeEditor } from './editor.js';
import { PythonRuntime } from './python-runtime.js';
import { SimulatorHost } from './simulator-host.js';
import { PROFILES, validateAction, fidelityNoticeFor, LEROBOT_REVISION } from './profiles.js';
import { TASK_PATCH_REVISION, defaultTaskId, isKinematicRigScenario, isPhysicalMujocoScenario, loadPatchedScenario, taskDescriptor, tasksForProfile } from './task-catalog.js';
import { buildPatchedWorkspace } from './task-workspace.js';
import { applyTheme, readStoredTheme, THEMES } from './themes.js';
import { AgentFacade } from './webmcp/agent-facade.js';
import { createWebMcpRegistration } from './webmcp/register-ide-tools.js';
import { MicroDuckControlDeck } from './microduck/control-deck.js';
import { MicroDuckPythonBridge } from './microduck/python-bridge.js';
import { PhysicalPythonRuntime } from './runtime/physical-python-runtime.js';
import { LivePythonBridge } from './runtime/live-python-bridge.js';
import { MicroDuckPhysicalBridge } from './runtime/microduck-physical-bridge.js';
import { applyPhysicsPreviewStatus } from './physics/ui-status.js';
import { installPhysicalAgentFacade } from './webmcp/physical-agent-facade.js';

const $ = (id) => document.getElementById(id);
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

class App {
  constructor() {
    const requestedProfile = new URLSearchParams(window.location.search).get('robot');
    this.profileId = requestedProfile && Object.hasOwn(PROFILES, requestedProfile)
      ? requestedProfile : (localStorage.getItem('rbide.profile') || 'openarm');
    if (!PROFILES[this.profileId]) this.profileId = 'openarm';
    this.taskId = localStorage.getItem(`rbide.task.${this.profileId}`) || defaultTaskId(this.profileId);
    this.scenario = null;
    this.files = {};
    this.workspaceGeneration = 0;
    this.workspaceStatus = 'loading';
    this.workspaceLoadQueue = Promise.resolve();
    this.workspaceMutationEnabled = false;
    this.agentAccess = 'off';
    this.agentAccessEpoch = 0;
    this.agentAccessListeners = new Set();
    this.agentContextListeners = new Set();
    this.agentRegistrationState = { available: false, registered: false, error: false, pending: false };
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
    this.statusText = 'Starting…';
    this.theme = applyTheme(readStoredTheme(), { persist: false });
    this.runtime = new PythonRuntime();
    this.highContrastScene = true;
    this.sim = new SimulatorHost($('simCanvas'));
    this.physicalRuntime = new PhysicalPythonRuntime({
      bridgeFactory: () => {
        const session = this.sim.getPhysicalSession();
        if (!session) throw new Error('The active workspace has no authoritative physical session.');
        return this.sim.profileId === 'microduck'
          ? new MicroDuckPhysicalBridge(this.sim.backend)
          : new LivePythonBridge(session);
      },
      onBoundary: (source, method) => this.showPhysicalBoundary(source, method),
      onOutput: (output) => { this.console = output; this.renderPanels(); },
      onState: ({ state }) => { $('simCanvas').dataset.physicalPythonState = state; },
    });
    this.physicalExecutionToken = null;
    this.microduckRuntime = new MicroDuckPythonBridge({
      simulator: this.sim,
      onBoundary: (source, method) => this.showMicroDuckBoundary(source, method),
      onOutput: (output) => { this.console = output; this.renderPanels(); },
      onState: ({ state }) => { $('simCanvas').dataset.microduckPythonState = state; },
    });
    this.microduckExecutionToken = null;
    this.sim.setControllerPreemptHandler((previous) => {
      if (previous?.source !== 'python' || !this.microduckRuntime.isActive()) return;
      this.problem('warning', 'OPERATION_CANCELLED', 'A trusted manual simulator command preempted the active Python lease.');
      void this.microduckRuntime.cancel('MANUAL_PREEMPTION');
    });
    this.sim.setHighContrastScene(this.highContrastScene);
    this.microduckDeck = new MicroDuckControlDeck($('microduckControlDeck'), {
      simulator: this.sim,
      canvas: $('simCanvas'),
      onStatus: (message) => this.setStatus(message),
      onState: () => { if (!$('bottomPanel').classList.contains('collapsed')) this.renderPanels(); },
    });
    this.microduckDeck.setActive(false);
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
    this.setWorkspaceMutationEnabled(false);
    this.updateExecutionControls();
    this.setStatus('Loading robot workspace…');
    void this.loadProfile(this.profileId, { preserve: false, taskId: this.taskId });
  }

  storageKey({ scenario = this.scenario, profileId = this.profileId, taskId = this.taskId } = {}) {
    const revision = scenario?.workspaceRevision || TASK_PATCH_REVISION.slice(0, 12);
    return `rbide.workspace.${revision}.${profileId}.${taskId}`;
  }

  isPolicyWorkspace() { return this.scenario?.simulationMode === 'policy_sim'; }
  isPhysicalWorkspace() { return isPhysicalMujocoScenario(this.scenario); }
  usesSourcePlant() { return Boolean(this.scenario) && !this.isPhysicalWorkspace() && !isKinematicRigScenario(this.scenario) && !this.isPolicyWorkspace(); }

  isKinematicPoseWorkspace() { return isKinematicRigScenario(this.scenario); }

  updateSimulationPresentation(profile, selectedMode = profile?.simulationMode) {
    // The selected workspace decides the labels, not the profile. MicroDuck's profile mode is the
    // demonstrator's, so reading it here would badge the physical workspace APPROXIMATE DYNAMICS.
    if (profile?.id === 'asimov') {
      $('modeChip').textContent = 'MUJOCO PHYSICS · ESTIMATED PD · HW VALIDATION PENDING';
      $('simBadge').textContent = 'ASIMOV 1 · 23 SOURCE JOINTS · FIXED NECK · OBSERVED BODY MESHES · NO VERIFIED WALKING';
      return;
    }
    const kinematic = selectedMode === 'kinematic_pose';
    const policy = selectedMode === 'policy_sim';
    $('modeChip').textContent = policy
      ? 'POLICY DEMONSTRATOR · APPROXIMATE DYNAMICS · HW VALIDATION PENDING'
      : kinematic
      ? 'KINEMATIC POSE RIG · NO CONTACT PLANT · HW VALIDATION PENDING'
      : 'SOURCE-PLANT SIMULATION · HW VALIDATION PENDING';
    $('simBadge').textContent = policy
      ? 'MICRODUCK 50 HZ POLICY SIM · OFFICIAL RUNTIME VISUAL · EXACT PINNED ONNX · APPROXIMATE DYNAMICS · NOT HARDWARE PARITY'
      : kinematic
      ? 'PINNED UNITREE G1 MESH / BOUNDED JOINT-POSE VIEW · NO CONTACT PLANT · CONFIGURED FLOOR · NOT HARDWARE VALIDATION'
      : 'PINNED ROBOBUDDY KINEMATICS / CONTACT PLANT · CONFIGURED FLOOR · NOT HARDWARE VALIDATION';
  }

  loadStored(options = {}) {
    try { const raw = localStorage.getItem(this.storageKey(options)); return raw ? JSON.parse(raw) : null; }
    catch { return null; }
  }

  async loadProfile(id, { preserve = true, taskId = '' } = {}) {
    if (preserve && Object.keys(this.files).length) this.save(false);
    this.cancelExecution('PROFILE_CHANGED');
    const generation = ++this.workspaceGeneration;
    this.workspaceStatus = 'loading';
    this.microduckDeck.setActive(false);
    $('cameraModeLabel').hidden = true;
    $('fitBtn').textContent = 'Fit';
    $('fitBtn').title = 'Fit the active robot in the simulator viewport.';
    $('fitBtn').setAttribute('aria-label', $('fitBtn').title);
    $('simCanvas').setAttribute('aria-label', 'Robot simulator');
    this.setWorkspaceMutationEnabled(false);
    this.profileId = id;
    this.emitAgentContextChange();
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
    // LeKiwi and MicroDuck each expose a physical workspace beside a non-physical one, so the
    // driver label follows the selected task rather than the profile. A legacy workspace is never
    // presented as physical mode, and a physical workspace never inherits the profile's
    // demonstrator label.
    const selectedMode = taskDescriptor(id, this.taskId)?.simulationMode || p.simulationMode;
    const migratedPhysical = id === 'so101' || id === 'openarm'
      || ((id === 'asimov' || id === 'lekiwi' || id === 'microduck' || id === 'unitree') && selectedMode === 'physical_mujoco');
    const visibleDriver = migratedPhysical ? 'robobuddy.sim.v1 · browser MuJoCo' : p.driver;
    $('driverLabel').textContent = visibleDriver;
    $('driverStatus').textContent = visibleDriver;
    this.updateSimulationPresentation(p, selectedMode);
    this.updateExecutionControls();
    this.setStatus(migratedPhysical ? `Loading ${p.shortLabel} MuJoCo physical workspace…` : selectedMode === 'policy_sim' ? 'Loading local MicroDuck runtime visual…' : selectedMode === 'kinematic_pose' ? 'Loading Unitree canonical pose workspace…' : 'Loading reviewed mission and source plant…');
    try {
      const selectedTaskId = this.taskId;
      const scenario = await loadPatchedScenario(id, selectedTaskId);
      if (generation !== this.workspaceGeneration) return;
      if (!scenario) throw new Error(`No configured task is available for ${id}.`);
      const starter = buildPatchedWorkspace(id, scenario);
      const files = this.loadStored({ scenario, profileId: id, taskId: selectedTaskId }) || starter;
      const simulatorReady = await this.queueSimulatorTransition(generation, () => (
        this.sim.setScenario(id, isKinematicRigScenario(scenario) ? null : scenario, p.rest)
      ));
      if (!simulatorReady) return;
      this.scenario = scenario;
      this.files = files;
      this.currentFile = 'main.py';
      this.renderFiles();
      this.openFile('main.py');
      this.renderTask();
      this.renderPanels();
      this.workspaceStatus = 'ready';
      this.microduckDeck.setActive(this.isPolicyWorkspace());
      $('cameraModeLabel').hidden = !this.isPolicyWorkspace();
      this.setWorkspaceMutationEnabled(true);
      this.updateExecutionControls();
      applyPhysicsPreviewStatus(id, { physical: this.isPhysicalWorkspace() });
      this.emitAgentContextChange();
      const source = this.isPhysicalWorkspace()
        ? `MuJoCo ${scenario.modelPackage} · ${scenario.physicalApi.version}`
        : p.simulationMode === 'policy_sim'
        ? 'pinned runtime monitor visual · 50 Hz policy simulation'
        : this.isKinematicPoseWorkspace()
        ? `canonical mesh ${this.scenario.canonicalModel.revision.slice(0, 12)}`
        : `source ${TASK_PATCH_REVISION.slice(0, 12)}`;
      this.setStatus(`Ready · ${this.scenario.title} · ${source}`);
    } catch (error) {
      if (generation !== this.workspaceGeneration) return;
      this.scenario = null;
      this.workspaceStatus = 'error';
      this.setWorkspaceMutationEnabled(false);
      this.files = { 'main.py': `# RoboBuddy workspace failed to load.\n# ${String(error.message || error)}\n` };
      this.renderFiles();
      this.openFile('main.py');
      this.problem('error', migratedPhysical ? 'PHYSICAL_WORKSPACE' : p.simulationMode === 'policy_sim' ? 'MICRODUCK_RIG' : p.simulationMode === 'kinematic_pose' ? 'RIG_WORKSPACE' : 'SOURCE_TASK', String(error.message || error));
      this.setStatus(migratedPhysical ? `${p.shortLabel} physical workspace unavailable` : p.simulationMode === 'policy_sim' ? 'MicroDuck runtime visual unavailable' : p.simulationMode === 'kinematic_pose' ? 'Unitree rig workspace unavailable' : 'Pinned source task unavailable');
      this.updateExecutionControls();
      this.emitAgentContextChange();
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
    if (!this.workspaceMutationEnabled || this.workspaceStatus !== 'ready') return;
    if (this.microduckRuntime.isActive() || this.physicalRuntime.isActive()) this.cancelExecution('WORKSPACE_CHANGED');
    this.workspaceGeneration += 1;
    this.files[file] = value;
    this.dirty.add(file);
    this.prepared = null;
    $('dirtyDot').hidden = false;
    this.renderFiles();
    this.emitAgentContextChange();
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
    const policy = this.isPolicyWorkspace();
    const physical = this.isPhysicalWorkspace();
    // The Unitree profile carries two workspaces. The pose workspace keeps its own task text and
    // fidelity notice, so selecting it can never surface the physical workspace's claims.
    const task = kinematic && p.kinematicTask ? p.kinematicTask : p.task;
    const labels = [];
    for (const item of scenario?.portablePython?.referenceActions || []) {
      const label = String(item.label || 'physical action');
      if (!labels.includes(label)) labels.push(label);
      if (labels.length >= 12) break;
    }
    const sourceLabel = physical ? 'Physical model / benchmark' : policy ? 'Pinned runtime hierarchy' : kinematic ? 'Canonical mesh source' : 'Pinned task source';
    const sourceText = physical
      ? `${scenario.modelPackage} · ${scenario.modelId} · ${scenario.physicalApi.version}`
      : policy
      ? `${scenario.canonicalModel.repository}@${scenario.canonicalModel.revision.slice(0, 12)} · ${scenario.canonicalModel.sourcePath} · ${scenario.canonicalModel.geometry}`
      : kinematic
      ? `RoboBuddy_AI@${scenario.canonicalModel.revision.slice(0, 12)} · Unitree URDF ${scenario.canonicalModel.sourceRevision.slice(0, 12)} · ${scenario.canonicalModel.license}`
      : `RoboBuddy_AI@${TASK_PATCH_REVISION.slice(0, 12)}`;
    $('taskPanel').innerHTML = `<h2>${escapeHtml(scenario?.title || task.title)}</h2><p>${escapeHtml(scenario?.brief || p.source)}</p><p><strong>${sourceLabel}:</strong> ${escapeHtml(sourceText)}</p><ol>${labels.map((label, index) => `<li class="${index === 0 ? 'task-current' : ''}">${escapeHtml(label)}</li>`).join('')}</ol><details><summary>Fidelity boundary</summary><p>${escapeHtml(physical ? scenario.limitations.join(' ') : task.limitations)}</p></details>`;
    $('fidelityText').textContent = physical
      ? `${p.shortLabel} uses one authoritative browser MuJoCo PhysicsSession. Rendering, live Python, WebMCP, and task evaluation consume that same state. ${scenario.limitations.join(' ')}`
      : policy
      ? `${fidelityNoticeFor(this.profileId)} ${p.task.limitations}`
      : kinematic
      ? `${fidelityNoticeFor(this.profileId, { physical: false })} ${task.limitations}`
      : `${fidelityNoticeFor(this.profileId)} LeRobot revision ${LEROBOT_REVISION}. Task definitions, reference actions, collision/contact plant, and support rules are pinned to RoboBuddy_AI revision ${TASK_PATCH_REVISION}. ${p.task.limitations}`;
    $('sideRobotSummary').textContent = physical
      ? `${p.label}. Browser MuJoCo is the single physical authority for the rigid-body benchmark. The canonical mesh is presentation-only; actual joint/block state and task evidence come from MuJoCo observations. Hardware validation remains pending.`
      : policy
      ? `${p.label}. Exact pinned ONNX policies and the official Apache-covered runtime visual; local winding/normals, configured lower-bill movement, rollers, contacts, sensors, and dynamics do not establish RL-environment or hardware parity.`
      : kinematic
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
    if (!this.workspaceMutationEnabled || this.workspaceStatus !== 'ready') return;
    const prompt = this.isPhysicalWorkspace()
      ? 'Reset all files for this physical MuJoCo workspace to the live async starter?'
      : this.isKinematicPoseWorkspace()
      ? 'Reset all files for this Unitree workspace to its browser-only kinematic-pose starter?'
      : this.isPolicyWorkspace()
      ? 'Reset all files for this MicroDuck workspace to its live browser-simulation Python starter?'
      : 'Reset all files for this task to its pinned reviewed physical-Python starter?';
    if (!confirm(prompt)) return;
    localStorage.removeItem(this.storageKey());
    if (!this.scenario) return;
    this.workspaceGeneration += 1;
    this.files = buildPatchedWorkspace(this.profileId, this.scenario);
    this.dirty.clear();
    this.openFile('main.py');
    this.prepared = null;
    this.emitAgentContextChange();
    await this.resetSimulation();
    this.renderFiles();
  }

  async prepare() {
    if (this.isPhysicalWorkspace()) throw new Error('Physical MuJoCo workspaces execute live async Python and do not compile to replay events.');
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
    if (this.workspaceStatus !== 'ready') return false;
    if (cancel) this.cancelExecution('RESET');
    const generation = this.workspaceGeneration;
    const profileId = this.profileId;
    const scenario = this.scenario;
    const profile = PROFILES[profileId];
    const resetReady = await this.queueSimulatorTransition(generation, () => (
      scenario ? this.sim.reset(profileId, isKinematicRigScenario(scenario) ? null : scenario, profile.rest) : Promise.resolve()
    ));
    if (!resetReady || this.workspaceStatus !== 'ready') return false;
    this.editor.highlightLine(null);
    this.stepIndex = 0;
    $('simActionLabel').textContent = 'Ready';
    this.renderPanels();
    this.setStatus('Simulation reset');
    return true;
  }

  updateExecutionControls() {
    const active = this.executionState !== 'idle';
    const workspaceReady = this.workspaceStatus === 'ready';
    const executableWorkspace = workspaceReady;
    const paused = this.executionState === 'paused';
    const pauseButton = $('pauseBtn');
    const policyReady = workspaceReady && this.isPolicyWorkspace();
    const physical = workspaceReady && this.isPhysicalWorkspace();
    pauseButton.disabled = !active && !policyReady;
    pauseButton.textContent = paused ? '▶ Resume' : '⏸ Pause';
    pauseButton.title = paused ? 'Resume simulation' : 'Pause simulation';
    pauseButton.setAttribute('aria-label', paused ? 'Resume simulation' : 'Pause simulation');
    pauseButton.setAttribute('aria-pressed', String(paused));
    $('runBtn').disabled = active || !executableWorkspace;
    $('stepBtn').disabled = physical || (!this.isPolicyWorkspace() && active) || !executableWorkspace || (this.isPolicyWorkspace() && active && !paused);
    $('stepBtn').title = physical ? 'Physical MuJoCo Python uses live async execution; edit code and use Run.' : 'Step physical action (F10)';
    $('cursorBtn').disabled = physical || active || !executableWorkspace;
    $('cursorBtn').title = physical ? 'Run to Cursor is not exposed for the live physical worker; use Run.' : 'Run to Cursor (Ctrl+F10)';
    $('resetBtn').disabled = !workspaceReady;
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

  cancelExecution(reason = 'OPERATION_CANCELLED') {
    this.openarmAgentProgramActive = false;
    this.runToken++;
    if (this.microduckRuntime.isActive()) void this.microduckRuntime.cancel(reason);
    if (this.physicalRuntime.isActive()) void this.physicalRuntime.cancel(reason);
    this.microduckExecutionToken = null;
    this.physicalExecutionToken = null;
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
    if (this.workspaceStatus === 'ready' && this.isPhysicalWorkspace() && !this.openarmAgentProgramActive) {
      if (!this.physicalRuntime.isActive()) return;
      if (this.executionState === 'paused') {
        this.executionState = 'running';
        void this.physicalRuntime.resume().catch((error) => this.problem('error', error.code || 'PHYSICAL_PAUSE', error.message));
        this.updateExecutionControls();
        this.setStatus(`${PROFILES[this.profileId].shortLabel} live Python and MuJoCo resumed`);
      } else if (this.executionState === 'running') {
        this.executionState = 'paused';
        void this.physicalRuntime.pause().catch((error) => this.problem('error', error.code || 'PHYSICAL_PAUSE', error.message));
        this.updateExecutionControls();
        this.setStatus(`${PROFILES[this.profileId].shortLabel} live Python and MuJoCo paused`);
      }
      return;
    }
    if (this.workspaceStatus === 'ready' && this.isPolicyWorkspace()) {
      if (this.microduckRuntime.isActive()) {
        if (this.executionState === 'paused') {
          this.executionState = 'running';
          this.microduckRuntime.resume();
          this.updateExecutionControls();
          this.setStatus('MicroDuck Python and policy simulation resumed');
        } else {
          this.executionState = 'paused';
          this.microduckRuntime.pause();
          this.updateExecutionControls();
          this.setStatus('MicroDuck Python and policy simulation paused');
        }
      } else if (this.executionState === 'paused') {
        this.executionState = 'idle';
        this.sim.resume();
        this.updateExecutionControls();
        this.setStatus('MicroDuck policy simulation resumed');
      } else {
        this.executionState = 'paused';
        this.sim.pause();
        this.updateExecutionControls();
        this.setStatus('MicroDuck policy simulation paused');
      }
      return;
    }
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
    if (this.workspaceStatus !== 'ready') return false;
    if (this.isPhysicalWorkspace()) return this.runPhysicalMujoco();
    if (this.isPolicyWorkspace()) return this.runMicroDuck('run');
    const token = this.beginExecution();
    if (token === null) return false;
    let completed = false;
    try {
      if (!(await this.resetSimulation({ cancel: false }))) return false;
      if (!(await this.waitForResume(token))) return false;
      const prep = await this.prepare();
      if (!(await this.waitForResume(token))) return false;
      this.stepIndex = 0;
      this.setStatus(this.usesSourcePlant() ? 'Running pinned source-plant simulation…' : 'Running Unitree kinematic pose sequence…');
      for (let i = 0; i < prep.events.length; i += 1) {
        if (!(await this._applyEvent(prep.events[i], token))) return false;
        this.stepIndex = i + 1;
      }
      completed = true;
    } catch {
      return false;
    } finally {
      if (completed && token === this.runToken) {
        this.editor.highlightLine(null);
        $('simActionLabel').textContent = 'Run complete';
        this.setStatus('Run complete');
        this.renderPanels();
      }
      this.finishExecution(token);
    }
    return completed;
  }

  async runPhysicalMujoco() {
    const token = this.beginExecution();
    if (token === null) return false;
    this.physicalExecutionToken = token;
    this.problems = [];
    this.commands = [];
    this.console = { stdout: '', stderr: '' };
    let completed = false;
    try {
      if (!(await this.resetSimulation({ cancel: false }))) return false;
      this.setStatus(`Running live ${PROFILES[this.profileId].shortLabel} physical Python against the authoritative MuJoCo session…`);
      const result = await this.physicalRuntime.start(this.files, {
        workspaceEpoch: this.workspaceGeneration,
        robotId: this.scenario.robotId,
        ...(this.scenario.executionBudget?.pythonWallTimeMs == null ? {} : { runTimeoutMs: this.scenario.executionBudget.pythonWallTimeMs }),
      });
      if (token !== this.runToken) return false;
      this.console = { stdout: result.stdout || '', stderr: result.stderr || '' };
      const evaluation = this.sim.getTaskEvaluation();
      this.editor.highlightLine(null);
      $('simActionLabel').textContent = evaluation?.status === 'observation-only' ? 'Run complete · observations recorded' : evaluation?.success ? 'Physical task complete' : 'Run complete · task incomplete';
      this.setStatus(evaluation?.status === 'observation-only' ? 'Run complete · physical observations recorded' : evaluation?.success ? `Run complete · ${PROFILES[this.profileId].shortLabel} physical task succeeded` : 'Run complete · physical task criteria not yet satisfied');
      this.renderPanels();
      completed = true;
      return true;
    } catch (error) {
      if (token === this.runToken && error.code !== 'OPERATION_CANCELLED') {
        this.problem('error', error.code || 'PYTHON', error.message);
        this.setStatus(`${PROFILES[this.profileId].shortLabel} live physical Python run failed`);
      }
      return false;
    } finally {
      this.physicalExecutionToken = null;
      if (!completed && token === this.runToken) this.renderPanels();
      this.finishExecution(token);
    }
  }

  async step() {
    if (this.workspaceStatus !== 'ready') return false;
    if (this.isPhysicalWorkspace()) { this.setStatus(`${PROFILES[this.profileId].shortLabel} physical Python is live async; use Run after editing the program.`); return false; }
    if (this.isPolicyWorkspace()) return this.stepMicroDuck();
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
    if (this.workspaceStatus !== 'ready') return false;
    if (this.isPhysicalWorkspace()) { this.setStatus('Run to Cursor is disabled for the live physical Python worker; use Run.'); return false; }
    if (this.isPolicyWorkspace()) return this.runMicroDuck('cursor', { file: this.currentFile, line: this.editor.getCursorLine() });
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
    const physicalPythonActive = this.physicalRuntime.isActive();
    this.cancelExecution('STOP');
    if (this.isPolicyWorkspace() || (this.isPhysicalWorkspace() && !physicalPythonActive)) void this.sim.stop();
    this.editor.highlightLine(null);
    $('simActionLabel').textContent = 'Stopped';
    this.setStatus('Simulation stopped');
  }

  showPhysicalBoundary(source, method) {
    if (this.currentFile === source?.file) this.editor.highlightLine(source.line);
    $('simActionLabel').textContent = `${source?.file || 'main.py'}:${source?.line || 1} · await ${method}()`;
    this.commands.push({ file: source?.file || 'main.py', line: Number(source?.line) || 1, command: method });
    if (this.commands.length > 100) this.commands.shift();
    this.renderPanels();
  }

  showMicroDuckBoundary(source, method) {
    if (this.currentFile === source?.file) this.editor.highlightLine(source.line);
    $('simActionLabel').textContent = `${source?.file || 'main.py'}:${source?.line || 1} · await robot.${method}()`;
    this.commands.push({ file: source?.file || 'main.py', line: Number(source?.line) || 1, command: method });
    if (this.commands.length > 100) this.commands.shift();
    this.renderPanels();
  }

  async beginMicroDuckRun(mode, cursor = null) {
    const token = this.beginExecution();
    if (token === null) return null;
    this.microduckExecutionToken = token;
    this.problems = [];
    this.commands = [];
    this.console = { stdout: '', stderr: '' };
    if (!(await this.resetSimulation({ cancel: false }))) { this.finishExecution(token); return null; }
    const completion = this.microduckRuntime.start(this.files, { workspaceEpoch: this.workspaceGeneration, mode, cursor });
    if (mode === 'step') {
      this.executionState = 'paused';
      this.sim.pause();
      this.updateExecutionControls();
    }
    return { token, completion };
  }

  async runMicroDuck(mode, cursor = null) {
    const started = await this.beginMicroDuckRun(mode, cursor);
    if (!started) return false;
    this.setStatus(mode === 'cursor' ? `Running live MicroDuck Python to ${cursor.file}:${cursor.line}…` : 'Running live MicroDuck browser-simulation Python…');
    try {
      const result = await started.completion;
      if (started.token !== this.runToken) return false;
      this.console = { stdout: result.stdout || '', stderr: result.stderr || '' };
      this.editor.highlightLine(null);
      const cursorBoundary = mode === 'cursor' ? result.cursor : null;
      $('simActionLabel').textContent = cursorBoundary ? `Stopped at ${cursorBoundary.file}:${cursorBoundary.line}` : mode === 'cursor' ? 'Run complete before cursor boundary' : 'Run complete';
      this.setStatus(cursorBoundary ? `Stopped at ${cursorBoundary.file}:${cursorBoundary.line}` : mode === 'cursor' ? `Run completed without a bridge boundary at or after ${cursor.file}:${cursor.line}` : 'MicroDuck Python run complete');
      this.renderPanels();
      return true;
    } catch (error) {
      if (error.code !== 'OPERATION_CANCELLED') {
        this.problem('error', error.code || 'PYTHON', error.message);
        this.setStatus(error.code === 'AUDIO_LOCKED' ? 'MicroDuck audio is locked' : 'MicroDuck Python run failed');
      }
      return false;
    } finally {
      this.microduckExecutionToken = null;
      this.finishExecution(started.token);
    }
  }

  async stepMicroDuck() {
    let started = null;
    if (!this.microduckRuntime.isActive()) {
      started = await this.beginMicroDuckRun('step');
      if (!started) return false;
      void started.completion.then((result) => {
        if (started.token !== this.runToken) return;
        this.console = { stdout: result.stdout || '', stderr: result.stderr || '' };
        this.setStatus('MicroDuck Python run complete');
        this.finishExecution(started.token);
      }).catch((error) => {
        if (error.code !== 'OPERATION_CANCELLED' && started.token === this.runToken) this.problem('error', error.code || 'PYTHON', error.message);
        this.finishExecution(started.token);
      });
    }
    try {
      const boundary = await this.microduckRuntime.step();
      this.executionState = this.microduckRuntime.isActive() ? 'paused' : 'idle';
      this.updateExecutionControls();
      if (boundary?.method === 'sleep' && boundary.remainingSeconds > 0) this.setStatus(`Stepped 20 ms of sleep at ${boundary.source.file}:${boundary.source.line} · ${boundary.remainingSeconds.toFixed(3)} s remains`);
      else if (boundary?.source) this.setStatus(`Stepped ${boundary.source.file}:${boundary.source.line} · ${boundary.method}`);
      return true;
    } catch (error) {
      if (error.code !== 'OPERATION_CANCELLED') this.problem('error', error.code || 'PYTHON', error.message);
      return false;
    }
  }

  problem(level, code, message) {
    this.problems.push({ level, code, message });
    this.openBottom('problems');
  }

  renderPanels() {
    const problems = $('problemsPanel');
    problems.innerHTML = this.problems.length ? this.problems.map((item) => `<div class="problem ${item.level}"><strong>${item.code}</strong><div>${escapeHtml(item.message).replace(/\n/g, '<br>')}</div></div>`).join('') : this.isPhysicalWorkspace() ? '<div class="problem info"><strong>PHYSICAL</strong><div>Physical state, contacts, object motion, and task evidence are read from the authoritative MuJoCo PhysicsSession. This is simulator evidence, not hardware calibration.</div></div>' : this.isPolicyWorkspace() ? '<div class="problem info"><strong>MODELED</strong><div>No simulator faults. Camera imagery, frame-derived IMUs, 8×8 ToF, contacts, dynamics, and generated audio are browser models—not calibrated hardware signals.</div></div>' : '<div class="empty-state">No problems.</div>';
    if (this.console.stdout || this.console.stderr) problems.innerHTML += `<div class="console-block">${this.console.stdout.split('\n').filter(Boolean).map((line) => `<div class="console-line">${escapeHtml(line)}</div>`).join('')}${this.console.stderr.split('\n').filter(Boolean).map((line) => `<div class="console-line stderr">${escapeHtml(line)}</div>`).join('')}</div>`;
    const kinematic = this.isKinematicPoseWorkspace();
    const physical = this.isPhysicalWorkspace();
    const telemetry = this.sim.getTelemetry();
    const telemetryNote = physical
      ? 'ACTUAL MUJOCO GROUND-TRUTH STATE — SI units/radians from the single authoritative PhysicsSession; not commanded targets and not hardware telemetry.'
      : this.isPolicyWorkspace()
      ? 'MODELED MICRODUCK POLICY-SIM STATE — exact pinned ONNX inference over original approximate browser dynamics; not hardware telemetry or RL-environment parity.'
      : kinematic
      ? 'BROWSER-HELD KINEMATIC G1 JOINT STATE — not measured telemetry, controller state, or a physical robot observation.'
      : 'SIMULATED ACTUAL STATE FROM THE PINNED ROBObUDDY FIXED-STEP PLANT — not measured hardware telemetry.';
    $('telemetryPanel').innerHTML = `<div class="panel-note">${telemetryNote}</div><table><tr><th>Field</th><th>Modeled value</th></tr>${Object.entries(telemetry).map(([key, value]) => `<tr><td>${escapeHtml(key)}</td><td>${typeof value === 'number' ? value.toFixed(3) : escapeHtml(value == null ? '—' : typeof value === 'object' ? JSON.stringify(value) : String(value))}</td></tr>`).join('')}</table>`;
    $('commandsPanel').innerHTML = this.commands.length ? this.commands.map((command, index) => physical
      ? `<div class="command-row ${index === this.commands.length - 1 ? 'active' : ''}"><span>${index + 1}</span><span>${escapeHtml(`${command.file}:${command.line}`)}</span><code>${escapeHtml(command.command)}</code><span>live robobuddy.sim.v1</span></div>`
      : this.isPolicyWorkspace()
      ? `<div class="command-row ${index === this.commands.length - 1 ? 'active' : ''}"><span>${index + 1}</span><span>${escapeHtml(`${command.file}:${command.line}`)}</span><code>${escapeHtml(command.command)}</code><span>browser simulation</span></div>`
      : `<div class="command-row ${index === this.stepIndex - 1 ? 'active' : ''}"><span>${index + 1}</span><span>${escapeHtml(this.actionLabel(index))}</span><code>${escapeHtml(JSON.stringify(command.action))}</code><span>${kinematic ? 'kinematic pose' : 'physical target'}</span></div>`).join('') : physical ? '<div class="empty-state">Run the live async Python program to populate physical API boundaries.</div>' : this.isPolicyWorkspace() ? '<div class="empty-state">Run, Step, or Run to Cursor to execute live catalog-backed MicroDuck Python.</div>' : `<div class="empty-state">Run or Step Action to prepare the ${kinematic ? 'kinematic pose' : 'physical command'} queue.</div>`;
    const contacts = this.sim.getContacts();
    const contactsNote = physical && this.profileId === 'asimov'
      ? 'MUJOCO CONTACT EVIDENCE. Foot-floor and self contacts come from the source rigid-body model. Mounted support is an explicit scene condition; no grasp, walking or task-success claim is inferred.'
      : physical
      ? 'MUJOCO CONTACT / TASK EVIDENCE. Gripper contact, lift, carry, release, settling, and final target state are observation-derived; no synthetic contact/success events or hidden attachments.'
      : this.isPolicyWorkspace()
      ? 'APPROXIMATE MUJOCO CONTACT STATE. Ball motion is contact-derived; no grasp attachment, physical parity, or hardware validation is claimed.'
      : kinematic
      ? 'G1 CONTACT / SUPPORT IS NOT SIMULATED. This panel reports the explicit kinematic boundary, not physical contact data.'
      : 'MODELED CONTACT / SUPPORT STATE FROM THE PINNED SOURCE PLANT — no force, torque, current, or tactile sensor data.';
    $('contactsPanel').innerHTML = `<div class="panel-note">${contactsNote}</div><div class="metric-grid">${Object.entries(contacts).map(([key, value]) => `<span>${escapeHtml(key)}</span><strong>${typeof value === 'number' ? value.toFixed(3) : escapeHtml(String(value))}</strong>`).join('')}</div>`;
  }

  openBottom(name) {
    $('app').classList.add('bottom-open');
    $('bottomPanel').classList.remove('collapsed');
    $('panelToggle').setAttribute('aria-expanded', 'true');
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
    $('panelToggle').setAttribute('aria-expanded', 'false');
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
  setStatus(text) { this.statusText = String(text); $('statusMessage').textContent = this.statusText; }

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
    if (!this.workspaceMutationEnabled || this.workspaceStatus !== 'ready') return;
    const generation = this.workspaceGeneration;
    const reader = new FileReader();
    reader.onload = () => {
      if (!this.workspaceMutationEnabled || this.workspaceStatus !== 'ready' || generation !== this.workspaceGeneration) return;
      const name = file.name.endsWith('.py') ? file.name : 'main.py';
      this.workspaceGeneration += 1;
      this.files[name] = String(reader.result);
      this.prepared = null;
      this.dirty.add(name);
      this.openFile(name);
      this.renderFiles();
      this.emitAgentContextChange();
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

  queueSimulatorTransition(generation, transition) {
    const pending = this.workspaceLoadQueue.catch(() => {}).then(async () => {
      if (generation !== this.workspaceGeneration) return false;
      await transition();
      return generation === this.workspaceGeneration;
    });
    this.workspaceLoadQueue = pending;
    return pending;
  }

  setWorkspaceMutationEnabled(enabled) {
    this.workspaceMutationEnabled = Boolean(enabled);
    const editor = $('editor');
    editor.inert = !this.workspaceMutationEnabled;
    editor.setAttribute('aria-disabled', String(!this.workspaceMutationEnabled));
    for (const button of document.querySelectorAll('[data-action="import"], [data-action="resetWorkspace"]')) button.disabled = !this.workspaceMutationEnabled;
  }

  getAgentAccess() { return this.agentAccess; }
  getExecutionState() { return this.executionState; }

  onAgentAccessChange(listener) {
    this.agentAccessListeners.add(listener);
    return () => this.agentAccessListeners.delete(listener);
  }

  onAgentContextChange(listener) {
    this.agentContextListeners.add(listener);
    return () => this.agentContextListeners.delete(listener);
  }

  emitAgentContextChange() {
    for (const listener of this.agentContextListeners) listener(this.getAgentRegistrationContext());
  }

  setAgentAccessFromTrustedEvent(access, event) {
    if (!event?.isTrusted || !['off', 'assist'].includes(access)) return false;
    if (access === 'assist' && !this.agentRegistrationState.available) return false;
    if (this.agentAccess === access) return true;
    this.agentAccess = access;
    this.agentAccessEpoch += 1;
    this.renderAgentAccessControl();
    for (const listener of this.agentAccessListeners) listener(access);
    return true;
  }

  setAgentRegistrationState(state) {
    this.agentRegistrationState = {
      available: Boolean(state?.available),
      registered: Boolean(state?.registered),
      error: Boolean(state?.error),
      pending: Boolean(state?.pending),
    };
    this.renderAgentAccessControl();
  }

  renderAgentAccessControl() {
    const control = $('agentAccessControl');
    if (!control) return;
    const { available, registered, error, pending } = this.agentRegistrationState;
    control.dataset.access = this.agentAccess;
    control.dataset.available = String(available);
    control.dataset.tools = pending ? 'enabling' : (registered ? 'enabled' : 'disabled');
    control.dataset.error = String(error);
    const enabled = this.agentAccess === 'assist';
    const toggle = $('agentAccessToggle');
    toggle.setAttribute('aria-checked', String(enabled));
    // Access can always be revoked, even if the host becomes unavailable.
    toggle.disabled = !available && !enabled;
    control.querySelector('.agent-access-state').textContent = enabled ? 'ON' : 'OFF';
    const status = !enabled
      ? (available ? 'Agent access is off. Turn on to enable bounded WebMCP tools for this session.' : 'WebMCP is not available in this browser. Agent access is off.')
      : error ? 'Tool registration failed. Turn Agent off and on to retry.'
      : pending ? 'Agent access is on. Connecting tools…'
      : registered ? 'Agent access is on. Bounded WebMCP tools are connected for this session.'
      : 'Agent access is on, but no tools are connected.';
    $('agentAccessStatus').textContent = status;
    toggle.title = `${status} ${enabled ? 'Click to turn off and revoke access.' : 'Access is session-only and never saves or publishes source.'}`;
  }

  getAgentSnapshot() {
    const base = {
      workspaceStatus: this.workspaceStatus,
      workspaceGeneration: this.workspaceGeneration,
      accessEpoch: this.agentAccessEpoch,
      profileId: this.profileId,
      taskId: this.taskId,
      simulatorEpoch: this.sim.getEpoch(),
    };
    if (this.workspaceStatus !== 'ready' || !this.scenario) return deepFreeze(base);

    const profile = PROFILES[this.profileId];
    const policySimulation = profile?.simulationMode === 'policy_sim';
    const physicalSimulation = this.isPhysicalWorkspace();
    const sourcePlantAvailable = !physicalSimulation && !policySimulation && !this.isKinematicPoseWorkspace();
    return deepFreeze({
      ...base,
      title: String(this.scenario.title || profile?.task?.title || ''),
      brief: String(this.scenario.brief || profile?.source || ''),
      robot: String(profile?.label || this.profileId),
      simulationMode: physicalSimulation ? 'physical_mujoco' : policySimulation ? 'policy_sim' : sourcePlantAvailable ? 'source_plant' : 'kinematic_pose',
      stateKind: physicalSimulation ? 'mujoco_physical_state' : policySimulation ? 'browser_policy_sim' : sourcePlantAvailable ? 'modeled_source_plant' : 'browser_kinematic_pose',
      currentFile: this.currentFile,
      files: Object.fromEntries(Object.entries(this.files).map(([name, content]) => [name, String(content)])),
      simulation: {
        executionState: this.executionState,
        status: this.statusText,
        preparedActionCount: this.commands.length,
        telemetry: { ...this.sim.getTelemetry() },
        contacts: { ...this.sim.getContacts() },
        taskEvaluation: this.sim.getTaskEvaluation(),
        physicalAuthority: this.sim.getPhysicalAuthorityToken(),
        problems: this.problems.map(({ level, code, message }) => ({ level, code, message })),
      },
    });
  }

  getAgentRegistrationContext() {
    const profile = PROFILES[this.profileId];
    return deepFreeze({
      access: this.agentAccess,
      accessEpoch: this.agentAccessEpoch,
      workspaceStatus: this.workspaceStatus,
      workspaceGeneration: this.workspaceGeneration,
      profileId: this.profileId,
      simulationMode: this.scenario?.simulationMode || profile?.simulationMode || null,
      simulationReady: this.workspaceStatus === 'ready' && this.sim.isReady(),
      simulatorEpoch: this.sim.getEpoch(),
    });
  }

  isAgentMicroduckSimulationReady() {
    return this.workspaceStatus === 'ready' && this.profileId === 'microduck' && this.isPolicyWorkspace() && this.sim.isReady();
  }

  executeAgentMicroduckCommand(command, args, context) { return this.sim.executeCommand(command, args, context); }
  abortAgentMicroduckCommand(command, controllerId) { return this.sim.abortCommand(command, { source: 'webmcp', controllerId }); }
  isAgentMicroduckCommandComplete(command, controllerId) { return this.sim.isCommandComplete(command, { source: 'webmcp', controllerId }); }
  isAgentMicroduckControllerActive(controllerId) { return this.sim.isControllerActive('webmcp', controllerId); }
  getAgentMicroduckState() { return this.isAgentMicroduckSimulationReady() ? this.sim.getState() : null; }

  manageAgentMicroduckVisualCues(request) {
    if (!this.isAgentMicroduckSimulationReady()) return null;
    return this.sim.backend?.manageVisualCues?.(request) || null;
  }

  focusAgentWorkspaceLine(file, line) {
    if (this.workspaceStatus !== 'ready' || !Object.hasOwn(this.files, file)) return false;
    this.openFile(file);
    this.editor.highlightLine(line);
    this.setStatus(`Agent focused ${file}:${line} for shared review`);
    return true;
  }

  applyTemporaryAgentWorkspaceEdit({ file, startLine, endLine, replacement, workingStartLine }) {
    if (this.workspaceStatus !== 'ready' || !Object.hasOwn(this.files, file)) return null;
    this.openFile(file);
    this.editor.replaceLineRange(startLine, endLine, replacement);
    this.editor.highlightLine(workingStartLine);
    this.setStatus(`Agent drafted a temporary cooperative edit in ${file}; refresh reloads the saved workspace.`);
    return { file, workspaceGeneration: this.workspaceGeneration, workingStartLine };
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
    const stepLabel = this.isPhysicalWorkspace() ? 'Run: Step unavailable for live physical Python' : this.isPolicyWorkspace() ? 'Run: Step MicroDuck Python boundary' : 'Run: Step physical action';
    return [{ label: 'Run: Run simulation', run: () => this.run() }, { label: stepLabel, run: () => this.step() }, { label: 'Run: Run to cursor', run: () => this.runToCursor() }, { label: 'View: Toggle Explorer', run: () => this.toggleSidebar() }, { label: 'View: Toggle diagnostics panel', run: () => this.togglePanel() }, { label: 'View: Toggle high-contrast scene', run: () => this.toggleHighContrastScene() }, { label: 'View: Fit simulator', run: () => this.sim.fit() }, { label: 'Robot: Contact diagnostics', run: () => this.openBottom('contacts') }, { label: 'Robot: Simulated telemetry', run: () => this.openBottom('telemetry') }, { label: 'Help: About RoboBuddy IDE', run: () => this.openAbout() }, { label: 'File: Save draft', run: () => this.save() }, { label: 'File: Export workspace', run: () => this.exportWorkspace() }, ...themeCommands];
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
    $('agentAccessToggle').addEventListener('click', (event) => {
      this.setAgentAccessFromTrustedEvent(this.agentAccess === 'assist' ? 'off' : 'assist', event);
    });
    this.renderAgentAccessControl();
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

function deepFreeze(value) {
  if (!value || typeof value !== 'object' || Object.isFrozen(value)) return value;
  for (const child of Object.values(value)) deepFreeze(child);
  return Object.freeze(value);
}

const app = new App();
const agentFacade = installPhysicalAgentFacade(new AgentFacade(app));
const webMcpRegistration = createWebMcpRegistration(agentFacade, {
  onRegistrationChange: (state) => app.setAgentRegistrationState(state),
});
app.onAgentAccessChange((access) => { void webMcpRegistration.setAccess(access); });
app.onAgentContextChange(() => { void webMcpRegistration.reconcile(); });
void webMcpRegistration.setAccess(app.getAgentAccess());

if (new URLSearchParams(window.location.search).has('ci')) {
  window.__robobuddyCi = Object.freeze({ app, agentFacade, webMcpRegistration });
}
