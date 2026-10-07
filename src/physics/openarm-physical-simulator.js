import { worldPoint, graspState, funnelSeatingState } from './openarm-observation.js';
import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js';
import { OrbitControls } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/controls/OrbitControls.js';
import { BrowserMuJoCoBackend } from './browser-mujoco-backend.js';
import { PhysicsSession } from './session.js';
import { registerModelPackage, unregisterTransientModelPackage } from './model-registry.js';
import { OPENARM_V2_PHASE5A_MODEL_PACKAGE as BASE_PACKAGE } from './openarm-model-package.js';
import { OPENARM_V2_PHASE5A_SCENE as BASE_SCENE } from './openarm-scene.js';
import { OPENARM_GEOMETRY_SHA256, OPENARM_MODEL_SHA256 } from './openarm-generated.js';
import { OpenArmBimanualStackEvaluator } from './openarm-task-evaluator.js';
import { OpenArmPresentation, toThreePosition } from './openarm-presentation.js';
import { validateOpenArmEquipment, compileEquipmentDefinitions, appendEquipmentXml, OPENARM_EQUIPMENT_VERSION, EQUIPMENT_LIMITS, EQUIPMENT_KINDS } from './openarm-equipment.js';
import { OPENARM_CONTROL_PROFILE } from './openarm-servo.js';
import { GENERAL_SCENE_VERSION, GENERAL_LIMITS, compileGeneralScene, appendGeneralSceneXml } from './openarm-general-scene.js';
import { GeneralTaskEvaluator, validateGeneralTask, worldPort } from './openarm-general-task.js';
import { OPENARM_LAB_ASSETS } from './openarm-lab-assets.js';
import { OPENARM_SCENE_MODES, validateOpenArmSceneMode, workcellBaseXml, workcellBodies, workcellConstraints, workcellGeometry } from './openarm-workcell-scene.js';

const OBSERVATION_BATCH_STEPS = 2;
const hashText = async text => [...new Uint8Array(await crypto.subtle.digest('SHA-256', new TextEncoder().encode(text)))].map(x => x.toString(16).padStart(2, '0')).join('');
async function fetchVerified(url, hash) {
  const response = await fetch(url, { cache: 'no-store' });
  if (!response.ok) throw new Error(`OpenArm asset HTTP ${response.status}`);
  const text = await response.text();
  if (await hashText(text) !== hash) throw new Error('OpenArm asset checksum mismatch');
  return text;
}
const number = (x, min, max, label) => { if (typeof x !== 'number' || !Number.isFinite(x) || x < min || x > max) throw new RangeError(`${label} must be ${min}..${max}`); return x; };
const clone = x => structuredClone(x);
let sessionSequence = 0;

export class OpenArmPhysicalSimulator {
  constructor(canvas) {
    this.canvas = canvas; this.disposed = false; this.ready = false; this.generation = 0;
    this.session = null; this.unsubscribeSession = null; this.lastObservation = null; this.presentationDirty = false;
    this.selectedScene = clone(BASE_SCENE); this.selectedPackage = BASE_PACKAGE; this.equipment = []; this.sceneMode = 'baseline';
    this.generalSpec = null; this.generalCompiled = null; this.generalTask = null; this.generalReport = null;
    this.staged = null; this.workcellMutation = false; this.candidate = null; this.baseGeometry = null;
    this.presentation = null; this.preview = null; this.objectMeshes = new Map(); this.highContrast = true;
    this.evaluator = new OpenArmBimanualStackEvaluator();
    this.scene = new THREE.Scene(); this.scene.background = new THREE.Color(0xd7dfe2);
    this.camera = new THREE.PerspectiveCamera(42, 1, 1, 8000);
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.shadowMap.enabled = true;
    this.controls = new OrbitControls(this.camera, canvas); this.controls.enableDamping = true;
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x475569, 2));
    const key = new THREE.DirectionalLight(0xffffff, 2.5); key.position.set(700, 2000, 900); key.castShadow = true;
    key.shadow.camera.left = -1000; key.shadow.camera.right = 1000; key.shadow.camera.top = 1800; key.shadow.camera.bottom = -1000; key.shadow.mapSize.set(2048, 2048); this.scene.add(key);
    const grid = new THREE.GridHelper(1800, 36, 0x6a7982, 0xa7b1b7); grid.position.set(400, .5, 0); grid.userData.presentationOnly = true; this.scene.add(grid); this.grid = grid;
    this.targetMarkers = [];
    for (const [x, y, z, w, d] of [[.67, .1535, 1.0355, .034, .026], [.67, -.1535, 1.0755, .042, .042]]) {
      const marker = new THREE.Mesh(new THREE.PlaneGeometry(w * 1000, d * 1000), new THREE.MeshBasicMaterial({ color: 0x28a86b, transparent: true, opacity: .5, depthWrite: false, side: THREE.DoubleSide }));
      marker.rotation.x = -Math.PI / 2; marker.position.copy(toThreePosition([x, y, z])); marker.userData.presentationOnly = true; this.scene.add(marker); this.targetMarkers.push(marker);
    }
    this.setHighContrastScene(this.highContrast);
    this.statusPanel = document.createElement('div'); this.statusPanel.className = 'openarm-workcell-status';
    Object.assign(this.statusPanel.style, { position: 'absolute', right: '8px', top: '48px', maxWidth: '320px', padding: '6px 9px', background: '#ffffffeb', color: '#23313a', font: '12px/1.4 sans-serif', borderRadius: '5px', pointerEvents: 'none' });
    canvas.parentElement?.append(this.statusPanel); this.#updateStatus(); this.resize(); this.fit();
  }

  async setScenario(profileId, scenario) {
    if (profileId !== 'openarm' || scenario?.simulationMode !== 'physical_mujoco' || scenario?.physicalSceneId !== BASE_SCENE.id) throw new Error('OpenArm requires its physical MuJoCo workspace');
    const token = ++this.generation; this.ready = false;
    const geometry = JSON.parse(await fetchVerified(new URL('../../models/openarm_v2/geometry.json', import.meta.url), OPENARM_GEOMETRY_SHA256));
    this.#assertGeneration(token);
    if (geometry.modelSha256 !== OPENARM_MODEL_SHA256) throw new Error('OpenArm visual/physical model identity mismatch');
    this.baseGeometry = geometry; this.#disposeSession(); this.selectedPackage = BASE_PACKAGE; this.selectedScene = clone(BASE_SCENE); this.equipment = []; this.sceneMode = 'baseline';
    this.generalSpec = null; this.generalCompiled = null; this.generalReport = null; this.generalTask = null;
    await this.#createSession(token); this.#rebuildPresentation(); this.ready = true;
    Object.assign(this.canvas.dataset, { simulatorBackend: 'browser-mujoco', simulationAuthority: 'physics-session', physicalSceneId: this.selectedScene.id, modelPackageId: this.selectedPackage.id, openarmVisualSource: 'shared-source-collision-geometry-v3', openarmLegacyBaseYawRendered: 'false', presentationGroundColor: '#687378' });
    this.fit(); this.#updateStatus(); return true;
  }
  async reset() {
    this.#assertLive(); const token = ++this.generation; this.ready = false; this.workcellMutation = false;
    this.candidate?.dispose(); this.candidate = null;
    this.generalTask?.invalidate('Explicit reset invalidates the trial; define a new task before manipulation');
    this.evaluator = new OpenArmBimanualStackEvaluator(); this.programProgress = null;
    const diagnostics = this.session ? await this.session.getDiagnostics().catch(() => null) : null;
    this.#assertGeneration(token);
    if (diagnostics?.loaded) await this.session.reset({ reason: 'explicit-user-reset' });
    else { this.session?.dispose(); this.session = null; await this.#createSession(token); }
    this.#assertGeneration(token); this.ready = true; this.#updateStatus(); return true;
  }
  renderFrame() {
    if (this.disposed) return;
    if (this.presentationDirty && this.lastObservation && this.presentation) { this.presentation.applyObservation(this.lastObservation); this.presentationDirty = false; }
    this.controls.update(); this.renderer.render(this.scene, this.camera);
  }
  resize() {
    if (this.disposed) return false;
    const w = Math.max(1, this.canvas.clientWidth || 640), h = Math.max(1, this.canvas.clientHeight || 480);
    this.renderer.setSize(w, h, false); this.camera.aspect = w/h; this.camera.updateProjectionMatrix(); return true;
  }
  fit() {
    this.controls.target.set(415, 1050, 0);
    const scale = Math.max(1, 1.18 / this.camera.aspect);
    this.camera.position.copy(this.controls.target).add(new THREE.Vector3(1175, 570, 1130).multiplyScalar(scale));
    this.camera.updateProjectionMatrix(); this.controls.update(); return true;
  }
  setHighContrastScene(value) {
    // Presentation only: report the real visible markers, never advance/reset the plant.
    this.highContrast = Boolean(value);
    for (const marker of this.targetMarkers) marker.visible = this.highContrast && this.sceneMode === 'baseline';
    Object.assign(this.canvas.dataset, {
      highContrastScene: String(this.highContrast),
      highContrastPerimeterCount: String(this.targetMarkers.filter(marker => marker.visible).length),
      presentationGroundColor: '#687378',
    });
    return this.highContrast;
  }
  isHighContrastSceneEnabled() { return this.highContrast; }
  isReady() { return Boolean(this.ready && !this.disposed && this.session?.robotId && this.lastObservation); }
  getPhysicalSession() { return this.session; }
  getPhysicalAuthorityToken() { return this.isReady() ? { sessionId: this.session.sessionId, epoch: this.session.epoch, sceneRevision: this.session.sceneRevision, robotId: this.session.robotId, simulationTimeSeconds: this.lastObservation.simulationTimeSeconds } : null; }
  getTaskEvaluation() {
    if (this.sceneMode === 'authored') return { ...(this.generalTask?.snapshot() || { success: false, status: 'no-task', scope: 'Authored scene; define a task before manipulation.' }), hardwareValidated: false };
    if (this.sceneMode === 'blank') return { ...this.evaluator.snapshot(), success: false, status: 'custom-scene', scope: 'No baseline task in a blank workcell. Use explicit program outcome predicates; a seated state alone does not prove a robot transfer.', hardwareValidated: false };
    return { ...this.evaluator.snapshot(), scope: this.equipment.length ? 'baseline flask/beaker task in an extended workcell; does not evaluate custom goals' : 'default flask/beaker transfer task', hardwareValidated: false }; }
  getPresentationAudit() {
    return { source: this.baseGeometry?.source, geometrySha256: OPENARM_GEOMETRY_SHA256, physicalAuthority: 'MuJoCo PhysicsSession only', jointPresentationSource: 'observed MuJoCo body transforms, including each passive finger', sharedCollisionGeometry: !this.generalSpec?.objects.some(e=>e.visual_mesh), collisionGeometryAuthoritative:true, geometryCount: this.presentation?.geomMeshes.size ?? 0, legacyBaseYawControlled: false, legacyBaseYawRendered: false, observationBatchSteps: OBSERVATION_BATCH_STEPS, observationPeriodSeconds: .002, observationPhase: this.lastObservation?.openarm?.observationPhase, workcellRevision: this.session?.sceneRevision };
  }
  getTelemetry() {
    const o = this.lastObservation; if (!o) return {};
    const result = { simulation_time_s: o.simulationTimeSeconds };
    if (this.sceneMode === 'authored') {
      const task = this.getTaskEvaluation();
      result.authored_task_success = task.success === true;
      result.authored_settled_dwell_s = task.settledDwellSeconds ?? 0;
    } else result.maximum_task_penetration_mm = this.evaluator.snapshot().maximumPenetrationM * 1000;
    for (const [id, j] of Object.entries(o.joints)) result[`${id}_rad`] = j.positionRad;
    for (const id of ['flask', 'beaker']) o.bodies[id]?.positionM.forEach((v, i) => { result[`${id}_${'xyz'[i]}_m`] = v; });
    return result;
  }
  getContacts() {
    if (this.sceneMode === 'authored') {
      const task = this.getTaskEvaluation(), flags = task.status === 'invalidated' ? {} : task.observedSequence || {};
      return { contact_count: this.lastObservation?.contactCount || 0, task_success: task.success === true,
        authored_grasp_seen: Boolean(flags.grasp), authored_lift_seen: Boolean(flags.lift),
        authored_carry_seen: Boolean(flags.carry), authored_release_seen: Boolean(flags.release),
        task_invalidated: task.status === 'invalidated' };
    }
    const e = this.evaluator.snapshot(); return { contact_count: this.lastObservation?.contactCount || 0, flask_grasp_seen: e.flask.graspSeen, beaker_grasp_seen: e.beaker.graspSeen, flask_support_contact: e.flask.currentSupportContact, beaker_support_contact: e.beaker.currentSupportContact, maximum_penetration_m: e.maximumPenetrationM, excessive_penetration: e.excessivePenetrationSeen, task_success: e.success }; }
  getState() { return this.lastObservation ? { observation: clone(this.lastObservation), evaluation: this.getTaskEvaluation(), authority: this.getPhysicalAuthorityToken() } : null; }
  setProgramProgress(progress) { this.programProgress = clone(progress); this.#updateStatus(); }
  getWorkcellState() {
    return { schemaVersion: OPENARM_EQUIPMENT_VERSION, sceneMode: this.sceneMode, sceneModes: OPENARM_SCENE_MODES, assetCatalog: clone(OPENARM_LAB_ASSETS), funnelSeating: this.#funnelSeating(), program: clone(this.programProgress || null), authority: this.getPhysicalAuthorityToken(), frame: 'mujoco_world', units: { length: 'm', angle: 'rad', time: 's', force: 'N' }, controlProfile: OPENARM_CONTROL_PROFILE, limits: EQUIPMENT_LIMITS, kinds: EQUIPMENT_KINDS, pinchReferences: clone(this.lastObservation?.openarm?.pinchReferences || {}), grasp: this.lastObservation ? { flask: graspState(this.lastObservation, 'left', 'flask'), beaker: graspState(this.lastObservation, 'right', 'beaker') } : {}, equipment: (this.lastObservation?.openarm?.equipment || []).map(e => ({ ...clone(e), referencePointsWorldM: Object.fromEntries(Object.entries(e.affordances).filter(([key, v]) => key !== 'axisM' && Array.isArray(v) && v.length === 3).map(([key, v]) => [key, worldPoint(this.lastObservation.bodies[e.bodyId], v)])) })), equipmentJoints: clone(this.lastObservation?.openarm?.equipmentJoints || []), staged: this.staged ? { id: this.staged.id, sceneMode: this.staged.sceneMode, equipment: clone(this.staged.equipment), ...(this.staged.generalScene ? { generalScene: true, report: clone(this.staged.report) } : {}), status: 'preview-only; apply explicitly resets the physical scene' } : null, bodies: clone(this.lastObservation?.bodies || {}), joints: clone(this.lastObservation?.joints || {}), contacts: clone(this.lastObservation?.contacts || []), taskEvaluation: this.getTaskEvaluation(), geometryIds: [...(this.presentation?.geomMeshes.keys() || [])], hardwareValidated: false, collisionFreePlanning: false };
  }
  async applyAction() { throw new Error('Use the live OpenArm SI/radian simulation API, not legacy .pos replay'); }
  #validateAdvance(seconds, maxSteps) {
    number(seconds, 0, 2, 'advanceSeconds');
    if (!Number.isInteger(maxSteps) || maxSteps < 1 || maxSteps > 20000) throw new RangeError('maxSteps must be 1..20000');
    const steps = Math.round(seconds / .001);
    if (Math.abs(steps * .001 - seconds) > 1e-9 || steps > maxSteps) throw new RangeError('Advance must align to 1 ms and fit the command step budget');
  }
  async applyPhysicalTargets(targetsRad, { maxSteps = 2000, advanceSeconds = 0, durationSeconds = null } = {}) {
    this.#assertReady(); this.#validateAdvance(advanceSeconds, maxSteps);
    if (!targetsRad || typeof targetsRad !== 'object' || Array.isArray(targetsRad) || !Object.keys(targetsRad).length) throw new TypeError('targetsRad must be nonempty');
    for (const [id, value] of Object.entries(targetsRad)) {
      const j = BASE_PACKAGE.joints.find(j => j.id === id), a = BASE_PACKAGE.actuators.find(a => a.jointId === id);
      if (!j || !a) throw new Error(`Unknown controllable OpenArm joint ${id}`);
      number(value, Math.max(j.rangeRad[0], a.controlRangeRad[0]), Math.min(j.rangeRad[1], a.controlRangeRad[1]), id);
    }
    if (durationSeconds != null) number(durationSeconds, .04, 12, 'durationSeconds');
    const accepted = await this.session.sendCommand({ type: durationSeconds == null ? 'set_joint_targets' : 'move_joint_targets', targetsRad: clone(targetsRad), ...(durationSeconds == null ? {} : { durationSeconds }) }, { maxSteps });
    const observation = advanceSeconds ? await this.advanceTime(advanceSeconds) : accepted.observation;
    return { ...accepted, acceptedTargetsRad: clone(targetsRad), observation, taskEvaluation: this.getTaskEvaluation() };
  }
  async applyToolTarget(request, { maxSteps = 15000, advanceSeconds = 0 } = {}) {
    this.#assertReady(); this.#validateAdvance(advanceSeconds, maxSteps);
    const accepted = await this.session.sendCommand({ type: 'set_tool_target', ...clone(request) }, { maxSteps });
    const observation = advanceSeconds ? await this.advanceTime(advanceSeconds) : accepted.observation;
    return { ...accepted, observation, motionPlan: observation.openarm?.motionPlan, taskEvaluation: this.getTaskEvaluation() };
  }
  async advanceTime(seconds) {
    this.#assertReady(); number(seconds, 0, 15, 'advance seconds');
    const steps = Math.round(seconds/.001);
    if (Math.abs(steps * .001 - seconds) > 1e-9) throw new RangeError('Advance must align to the 1 ms physics timestep');
    return steps ? this.session.advanceSteps(steps) : this.session.getObservation();
  }
  pause() { return this.session?.pause() ?? false; }
  resume() { return this.session?.resume() ?? false; }
  async stop() {
    this.generalTask?.invalidate('Human Stop invalidated the trial');
    ++this.generation; this.ready = false; this.workcellMutation = false; this.candidate?.dispose(); this.candidate = null;
    if (!this.session) return false;
    try { await this.session.cancelRun('human-stop'); return true; } catch { return false; }
  }
  stageEquipment(items, sceneMode = this.sceneMode) {
    this.#assertReady(); validateOpenArmSceneMode(sceneMode); if (sceneMode === 'authored') throw new Error('Use manage_openarm_scene for authored scenes, or explicitly restore baseline/blank'); const equipment = validateOpenArmEquipment(items);
    const compiled = compileEquipmentDefinitions(equipment);
    this.preview?.dispose(); this.preview = new OpenArmPresentation({ meshes: compiled.meshes, geoms: compiled.geoms }, { preview: true });
    const bodies = {};
    for (const e of equipment) {
      const q = e.quaternion_wxyz || [Math.cos(e.yaw_rad/2), 0, 0, Math.sin(e.yaw_rad/2)];
      bodies[`lab_${e.id}`] = { positionM: e.position_m, quaternionWxyz: q };
      if (e.kind === 'button') bodies[`lab_${e.id}_cap`] = { positionM: worldPoint(bodies[`lab_${e.id}`], [0,0,.020]), quaternionWxyz: q };
    }
    this.preview.applyObservation({ bodies }); this.scene.add(this.preview.root);
    this.staged = { id: crypto.randomUUID(), equipment, sceneMode, expectedRevision: this.session.sceneRevision, expectedEpoch: this.session.epoch };
    this.#updateStatus(); return this.getWorkcellState().staged;
  }
  discardStagedEquipment() { this.preview?.dispose(); this.preview = null; this.staged = null; this.#updateStatus(); return true; }
  async applyStagedEquipment(stageId, acknowledgeReset, guard = () => {}) {
    this.#assertReady();
    if (acknowledgeReset !== true || !this.staged || stageId !== this.staged.id) throw new Error('A matching stage_id and acknowledge_reset:true are required');
    if (this.staged.expectedRevision !== this.session.sceneRevision || this.staged.expectedEpoch !== this.session.epoch) throw new Error('Staged scene is stale; stage it again after reset or scene changes');
    const token = ++this.generation, staged = clone(this.staged); this.workcellMutation = true; this.#updateStatus();
    let candidate = null, candidatePackage = null;
    try {
      guard(); const baseXml = await fetchVerified(new URL('../../models/openarm_v2/manipulation.xml', import.meta.url), OPENARM_MODEL_SHA256);
      this.#assertGeneration(token); guard();
      const compiled = staged.generalScene ? appendGeneralSceneXml(baseXml, staged.generalScene) : appendEquipmentXml(workcellBaseXml(baseXml, staged.sceneMode), staged.equipment);
      const specHash = staged.generalScene ? await hashText(JSON.stringify(compiled.spec)) : null;
      this.#assertGeneration(token); guard();
      const constraints = workcellConstraints(clone(BASE_PACKAGE.sceneConstraints), staged.sceneMode);
      const sha256 = await hashText(compiled.xml);
      const sequence = ++sessionSequence;
      const packageId = `openarm-custom-${sha256.slice(0, 12)}-${sequence}`;
      candidatePackage = registerModelPackage({ ...clone(BASE_PACKAGE), id: packageId, modelId: packageId, transient: true, baseSha256: OPENARM_MODEL_SHA256, sha256, ...(staged.generalScene ? { generalScene: compiled.spec, sceneSpecSha256: specHash } : {}), sceneMode: staged.sceneMode, sceneConstraints: constraints, equipment: staged.equipment, bodies: [...workcellBodies(clone(BASE_PACKAGE.bodies), staged.sceneMode), ...compiled.bodies] });
      const scene = { ...clone(BASE_SCENE), modelPackage: packageId, fixtures: constraints.fixtures.map(id=>({id})), objects: constraints.objects.map(id=>({id})), revision: `${BASE_SCENE.revision}-${staged.sceneMode}-equipment-${sha256.slice(0, 16)}${specHash ? '-'+specHash.slice(0,16) : ''}`, ...(staged.sceneMode !== 'baseline' ? { taskGoal: { type: 'custom-workcell-outcome-predicates' } } : {}) };
      candidate = this.#newSession(); this.candidate = candidate;
      const loaded = await candidate.loadScene(scene); this.#assertGeneration(token); guard();
      // Commit only after model compilation AND all initial penetration checks succeed.
      this.#disposeSession(); this.session = candidate; this.candidate = null; candidate = null;
      this.selectedPackage = candidatePackage; candidatePackage = null; this.selectedScene = scene; this.equipment = staged.equipment; this.sceneMode = staged.sceneMode;
      this.generalSpec = staged.generalScene ? compiled.spec : null; this.generalCompiled = staged.generalScene ? compiled : null; this.generalTask = null;
      this.generalReport = staged.generalScene ? { ...compiled.report, sceneSpecSha256: specHash, physicalModelSha256: sha256, physicalValidation: 'MuJoCo compile and initial overlap checks passed; settling and manipulation are separate', candidateCheck: staged.candidateCheck || null } : null;
      this.evaluator = new OpenArmBimanualStackEvaluator(); this.programProgress = null; this.#subscribe(); this.#consumeObservation(loaded.observation);
      this.#rebuildPresentation(); this.discardStagedEquipment(); this.ready = true;
      this.canvas.dataset.modelPackageId = this.selectedPackage.id;
      return { status: 'applied', reset: true, sceneMode: this.sceneMode, authority: this.getPhysicalAuthorityToken(), equipment: clone(compiled.records) };
    } finally {
      candidate?.dispose(); if (candidatePackage) unregisterTransientModelPackage(candidatePackage.id);
      if (token === this.generation) { this.workcellMutation = false; this.candidate = null; this.#updateStatus(); }
    }
  }
  dispose() {
    if (this.disposed) return; this.disposed = true; ++this.generation; this.ready = false;
    this.candidate?.dispose(); this.candidate = null; this.#disposeSession(); this.preview?.dispose(); this.presentation?.dispose();
    this.targetMarkers.forEach(m => { m.geometry.dispose(); m.material.dispose(); }); this.grid.geometry.dispose();
    for (const m of Array.isArray(this.grid.material) ? this.grid.material : [this.grid.material]) m.dispose();
    this.statusPanel.remove(); this.controls.dispose(); this.renderer.dispose();
  }
  #newSession() { return new PhysicsSession(new BrowserMuJoCoBackend({ workerUrl: new URL('./openarm-mujoco-worker.js', import.meta.url) }), { sessionId: `ide-openarm-v3-${++sessionSequence}`, observationBatchSteps: OBSERVATION_BATCH_STEPS }); }
  async #createSession(token) { this.session = this.#newSession(); this.#subscribe(); await this.session.loadScene(this.selectedScene); this.#assertGeneration(token); }
  #subscribe() { this.unsubscribeSession?.(); this.unsubscribeSession = this.session.subscribe(({ observation, epoch }) => { if (this.generalTask && epoch !== this.generalTaskEpoch) this.generalTask.invalidate('Physical session epoch changed'); this.#consumeObservation(observation); }); }
  #disposeSession() {
    this.unsubscribeSession?.(); this.unsubscribeSession = null; this.session?.dispose(); this.session = null;
    if (this.selectedPackage?.transient) unregisterTransientModelPackage(this.selectedPackage.id);
    this.lastObservation = null;
  }
  #consumeObservation(observation) {
    if (this.disposed || !observation) return;
    this.lastObservation = observation; this.generalTask?.observe(observation); if (this.sceneMode === 'baseline') this.evaluator.observe(observation); this.presentationDirty = true;
    const e = this.getTaskEvaluation(), authored = this.sceneMode === 'authored';
    const flags = e.status === 'invalidated' ? {} : (e.observedSequence || {});
    Object.assign(this.canvas.dataset, {
      simulationClockS: String(observation.simulationTimeSeconds), physicalTaskSuccess: String(e.success), physicalTaskScope: this.sceneMode,
      physicalTaskContact: String(authored ? Boolean(flags.grasp) : e.flask.graspSeen || e.beaker.graspSeen),
      physicalTaskLift: String(authored ? Boolean(flags.lift) : e.flask.liftSeen || e.beaker.liftSeen),
      physicalTaskCarry: String(authored ? Boolean(flags.carry) : e.flask.carrySeen || e.beaker.carrySeen),
      physicalTaskRelease: String(authored ? Boolean(flags.release) : e.flask.releaseSeen || e.beaker.releaseSeen),
      physicalTaskSettled: String(authored ? e.success === true : e.flask.settled && e.beaker.settled)
    });
  }
  #rebuildPresentation() {
    this.presentation?.dispose();
    const compiled = this.generalCompiled || compileEquipmentDefinitions(this.equipment);
    this.presentation = new OpenArmPresentation({ visualGeoms: compiled.visualGeoms || [], meshes: { ...this.baseGeometry.meshes, ...compiled.meshes }, geoms: [...workcellGeometry(this.baseGeometry.geoms, this.sceneMode), ...compiled.geoms] });
    this.presentation.setCollisionView?.(this.collisionView || false);
    this.setHighContrastScene(this.highContrast);
    this.objectMeshes = this.presentation.bodyGroups; this.scene.add(this.presentation.root);
    if (this.lastObservation) this.presentation.applyObservation(this.lastObservation);
  }
  stageGeneralScene(raw) {
    this.#assertReady(); const c = compileGeneralScene(raw);
    const preview = new OpenArmPresentation({ meshes: c.meshes, geoms: c.geoms }, { preview: true });
    try { preview.applyObservation({ bodies: Object.fromEntries(c.spec.objects.map(e => [`lab_${e.id}`, { positionM:e.position_m, quaternionWxyz:e.quaternion_wxyz }])) }); }
    catch (error) { preview.dispose(); throw error; }
    this.preview?.dispose(); this.preview = preview; this.scene.add(preview.root);
    this.staged = { id:crypto.randomUUID(), equipment:[], sceneMode:'authored', generalScene:c.spec, report:c.report, expectedRevision:this.session.sceneRevision, expectedEpoch:this.session.epoch };
    this.#updateStatus(); return { id:this.staged.id, report:clone(c.report), status:'preview-only; explicit apply resets the physical scene' };
  }
  async checkStagedGeneralScene(stageId, settleSeconds = .3, guard = () => {}) {
    this.#assertReady(); number(settleSeconds, .02, .5, 'candidate settle seconds');
    if(Math.abs(settleSeconds/.001-Math.round(settleSeconds/.001))>1e-7)throw new RangeError('Settle check time must align to 1 ms');
    if (!this.staged?.generalScene || this.staged.id !== stageId || this.staged.expectedRevision !== this.session.sceneRevision || this.staged.expectedEpoch !== this.session.epoch) throw new Error('Staged scene is missing or stale');
    const token = this.generation, staged = this.staged; this.workcellMutation = true;
    let candidate = null, pkg = null;
    try {
      guard(); const baseXml = await fetchVerified(new URL('../../models/openarm_v2/manipulation.xml', import.meta.url), OPENARM_MODEL_SHA256);
      const c = appendGeneralSceneXml(baseXml, staged.generalScene), sha256 = await hashText(c.xml), id = `openarm-check-${++sessionSequence}`;
      this.#assertGeneration(token); guard();
      const constraints = workcellConstraints(clone(BASE_PACKAGE.sceneConstraints),'authored');
      pkg = registerModelPackage({ ...clone(BASE_PACKAGE), id, modelId:id, transient:true, sha256, baseSha256:OPENARM_MODEL_SHA256, sceneMode:'authored', generalScene:c.spec, sceneSpecSha256:await hashText(JSON.stringify(c.spec)), sceneConstraints:constraints, bodies:[...workcellBodies(clone(BASE_PACKAGE.bodies),'authored'),...c.bodies] });
      candidate = this.#newSession(); this.candidate = candidate;
      const scene = { ...clone(BASE_SCENE), modelPackage:id, fixtures:constraints.fixtures.map(id=>({id})), objects:[], revision:`check-${sha256}`, taskGoal:{type:'validation-only'} };
      const started = performance.now(), initial = (await candidate.loadScene(scene)).observation;
      this.#assertGeneration(token); guard();
      const final = await candidate.advanceSteps(Math.round(settleSeconds/.001));
      this.#assertGeneration(token); guard();
      if (this.staged !== staged) throw new Error('Candidate scene changed during validation');
      const objects = c.records.map(e=>{
        const a=initial.bodies[e.bodyId], b=final.bodies[e.bodyId];
        return { id:e.id, motion:e.dynamic?'free':'fixed', displacementM:Math.hypot(...b.positionM.map((v,i)=>v-a.positionM[i])), linearSpeedMS:e.dynamic?Math.hypot(...b.linearVelocityMS):0, angularSpeedRadS:e.dynamic?Math.hypot(...b.angularVelocityRadS):0 };
      });
      const relationships = c.spec.relationships.map(r=>{ const a=c.records.find(e=>e.id===r.subject_id),b=c.records.find(e=>e.id===r.object_id); return {...r,contactObserved:final.contacts.some(x=>a.geometryIds.includes(x.geom1Name)&&b.geometryIds.includes(x.geom2Name)||a.geometryIds.includes(x.geom2Name)&&b.geometryIds.includes(x.geom1Name)),scope:'Contact only; a declared relationship does not impose a joint, weld, support or insert constraint'}; });
      const result = { relationships, physicalCompilePassed:true, initialOverlapCheckPassed:true, candidateSimulationSeconds:settleSeconds, wallTimeMs:performance.now()-started, activeSceneUnchanged:true, objects, allFreeObjectsLowMotion:objects.every(e=>e.linearSpeedMS<.01&&e.angularSpeedRadS<.15), scope:'Separate disposable candidate; low motion is not proof of correct support, reconstruction or manipulation', hardwareValidated:false };
      staged.candidateCheck = result; return clone(result);
    } finally {
      candidate?.dispose(); if(pkg) unregisterTransientModelPackage(pkg.id);
      if(token===this.generation){this.candidate=null;this.workcellMutation=false;this.#updateStatus();}
    }
  }
  getGeneralSceneState(view = 'summary', objectId = null) {
    const c=this.generalCompiled;
    if(view==='spec')return { scene:clone(this.generalSpec), stagedScene:clone(this.staged?.generalScene || null), autoStart:false };
    if(view==='object'){
      const e=c?.records.find(e=>e.id===objectId); if(!e)throw new Error('Unknown authored object');
      return { ...clone(e), portsWorld:e.ports.map(p=>worldPort(this.lastObservation.bodies[e.bodyId],p)), observedBody:clone(this.lastObservation?.bodies[e.bodyId] || null) };
    }
    return { schemaVersion:GENERAL_SCENE_VERSION, authority:this.getPhysicalAuthorityToken(), sceneMode:this.sceneMode, reference:clone(this.generalSpec?.reference || null), report:clone(this.generalReport), staged:this.staged?.generalScene?{ id:this.staged.id,report:clone(this.staged.report),candidateCheck:clone(this.staged.candidateCheck || null) }:null, objects:(c?.records||[]).map(e=>({id:e.id,label:e.label,kind:e.kind,motion:e.dynamic?'free':'fixed',ports:clone(e.ports),portsWorld:e.ports.map(p=>worldPort(this.lastObservation.bodies[e.bodyId],p))})), task:this.generalTask?.snapshot() || null, limits:GENERAL_LIMITS, imageInterpretation:'external agent; pixels and author claims are not independently verified', hardwareValidated:false };
  }
  defineGeneralTask(raw, assessOnly = false) {
    this.#assertReady(); if(!this.generalSpec)throw new Error('Apply an authored scene before defining its task');
    const v=validateGeneralTask(raw,this.lastObservation);
    if(assessOnly)return clone(v);
    this.generalTask=new GeneralTaskEvaluator(v,this.lastObservation,this.session.sceneRevision); this.generalTaskEpoch=this.session.epoch;
    this.#consumeObservation(this.lastObservation);
    return this.generalTask.snapshot();
  }
  clearGeneralTask() { this.#assertReady(); this.generalTask=null; this.#consumeObservation(this.lastObservation); return true; }
  setCollisionView(value) { this.collisionView=Boolean(value); this.presentation?.setCollisionView?.(this.collisionView); return this.collisionView; }
  #funnelSeating() {
    const o = this.lastObservation; if (!o) return [];
    const assets = o.openarm?.equipment || [];
    return assets.filter(e=>e.kind==='funnel').flatMap(f=>assets.filter(e=>e.kind==='burette').map(b=>({ objectId: f.bodyId, receiverId: b.bodyId, ...funnelSeatingState(o, f.bodyId, b.bodyId) })));
  }
  #updateStatus() {
    if (!this.statusPanel) return;
    this.statusPanel.textContent = this.programProgress?.status === 'running' ? `OpenArm program ${this.programProgress.index}/${this.programProgress.segments}: ${this.programProgress.label}` : this.workcellMutation ? 'Compiling physical workcell; existing scene retained until checks pass.' : this.staged ? `Staged ${this.staged.sceneMode} scene with ${this.staged.generalScene?.objects.length ?? this.staged.equipment.length} items. Apply resets; blank removes the reference fixtures and vessels.` : `OpenArm · ${this.sceneMode} workcell · ${this.generalSpec?.objects.length ?? this.equipment.length} equipment items · simulation only`;
  }
  #assertLive() { if (this.disposed) throw new Error('OpenArm simulator is disposed'); }
  #assertReady() { this.#assertLive(); if (!this.isReady() || this.workcellMutation) throw new Error('OpenArm physical session is not ready or is compiling a workcell'); }
  #assertGeneration(token) { this.#assertLive(); if (token !== this.generation) throw new Error('Stale OpenArm workcell operation'); }
}
