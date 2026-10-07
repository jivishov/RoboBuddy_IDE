import fs from 'node:fs';
import {
  LEKIWI_PHYSICAL_TASKS,
  OPENARM_PHYSICAL_TASKS,
  PATCH_TASKS,
  SO101_LEGACY_TASK_CLASSIFICATION,
  TASK_PATCH_REVISION,
  loadPatchedScenario,
  tasksForProfile,
} from '../src/task-catalog.js';
import { validateAction } from '../src/profiles.js';
import { buildPatchedWorkspace } from '../src/task-workspace.js';

if (TASK_PATCH_REVISION !== '75fe2669c0ab0b029986de424c69162071174df8') throw new Error('unexpected task patch revision');
const expected = {
  openarm: ['openarm-04-filtration-workcell'],
  so101: ['so101-v2-06-quantitative-transfer','so101-v2-08-burette-initial-reading','so101-v2-09-vacuum-filtration'],
};
for (const [profileId, ids] of Object.entries(expected)) {
  const configured = (PATCH_TASKS[profileId] || []).map((item) => item.id);
  if (JSON.stringify(configured) !== JSON.stringify(ids)) throw new Error(`${profileId} task catalog drift: ${configured}`);
  // OpenArm's legacy descriptor remains pinned for provenance; its normal catalog is the migrated
  // physical workspace, asserted separately below.
  if (profileId === 'openarm') continue;
  for (const id of ids) {
    const scenario = await loadPatchedScenario(profileId, id);
    const actions = scenario.portablePython.referenceActions;
    if (!actions.length) throw new Error(`${id}: no reference actions`);
    for (const [index, record] of actions.entries()) {
      try { validateAction(profileId, record.action); }
      catch (error) { throw new Error(`${id} action ${index + 1} violates physical public API envelope: ${error.message}`); }
    }
    const workspace = buildPatchedWorkspace(profileId, scenario);
    for (const file of ['main.py','trajectories.py','robot_config.py','workcell.py']) if (!workspace[file]) throw new Error(`${id}: missing ${file}`);
    if (/\.(grasp|attach|teleport|move_to)\s*\(/.test(workspace['main.py'])) throw new Error(`${id}: fake physical robot method exposed`);
  }
}

const visibleOpenarm = tasksForProfile('openarm');
if (visibleOpenarm.length !== 1 || visibleOpenarm[0].id !== 'openarm-04-filtration-workcell' || visibleOpenarm[0].simulationMode !== 'physical_mujoco') throw new Error(`OpenArm physical catalog drift: ${JSON.stringify(visibleOpenarm)}`);
if (OPENARM_PHYSICAL_TASKS[0].physicalSceneId !== 'phase5a-openarm-v2-bimanual-stack') throw new Error('OpenArm physical scene id drifted');
const openarmScenario = await loadPatchedScenario('openarm', 'openarm-04-filtration-workcell');
if (openarmScenario.modelPackage !== 'openarm-v2-phase5a-a8c9796-v3') throw new Error('OpenArm refined physical model package drifted');
if (openarmScenario.workspaceRevision !== 'openarm-v2-physical-bimanual-stack-v3') throw new Error('OpenArm refined workspace revision drifted');
if (openarmScenario.physicalSceneRevision !== 'phase5a-openarm-v2-bimanual-stack-v3') throw new Error('OpenArm refined physical scene revision drifted');
if (openarmScenario.robotId !== 'openarm_v2_bimanual') throw new Error('OpenArm physical robot identity drifted');
const openarmWorkspace = buildPatchedWorkspace('openarm', openarmScenario);
for (const token of ['from robobuddy.sim import connect', 'await connect(', 'await robot.send_action(', 'await robot.advance(', 'await robot.get_observation()']) if (!openarmWorkspace['main.py'].includes(token)) throw new Error(`OpenArm live physical starter missing ${token}`);
for (const forbidden of ['time.sleep(', 'lerobot', '.grasp(', '.attach(', '.teleport(', '.move_to(']) if (openarmWorkspace['main.py'].includes(forbidden)) throw new Error(`OpenArm physical starter exposes forbidden/legacy behavior ${forbidden}`);
if (!openarmWorkspace['trajectories.py'].includes('openarm_left_joint1') || !openarmWorkspace['trajectories.py'].includes('openarm_right_joint1')) throw new Error('OpenArm trajectories must expose both V2 arms as radian targets');
if (!openarmWorkspace['workcell.py'].includes('calibration')) throw new Error('OpenArm workcell must preserve calibration boundary');

const visibleLekiwi = tasksForProfile('lekiwi');
if (visibleLekiwi.length !== 1) throw new Error(`LeKiwi catalog must expose only the finalized physical workspace: ${JSON.stringify(visibleLekiwi)}`);
if (visibleLekiwi[0].id !== 'lekiwi-physical-beaker-courier' || visibleLekiwi[0].simulationMode !== 'physical_mujoco') throw new Error(`LeKiwi physical catalog drift: ${JSON.stringify(visibleLekiwi)}`);
if (visibleLekiwi.some((item) => item.id === 'lekiwi-01-beaker-courier')) throw new Error('legacy LeKiwi task leaked into the selectable catalog');
if (LEKIWI_PHYSICAL_TASKS[0].physicalSceneId !== 'p5b-lekiwi-beaker-courier') throw new Error('LeKiwi physical scene id drifted');
const lekiwiScenario = await loadPatchedScenario('lekiwi', 'lekiwi-physical-beaker-courier');
if (lekiwiScenario.modelPackage !== 'lekiwi-courier-efa608d-v1') throw new Error('LeKiwi physical model package drifted');
if (lekiwiScenario.physicalSceneRevision !== 'p5b-lekiwi-beaker-courier-v1') throw new Error('LeKiwi physical scene revision drifted');
if (lekiwiScenario.robotId !== 'lekiwi_v1_mobile_manipulator') throw new Error('LeKiwi physical robot identity drifted');
if (lekiwiScenario.canonicalModel.revision !== 'efa608d7ee5a495a4803b1d28cd0c955b4f1e033') throw new Error('LeKiwi official source pin drifted');
if (lekiwiScenario.canonicalModel.apiCompatibilityRevision !== '7e241bd630a3719a56157a497ce5d08f244784f1') throw new Error('LeRobot compatibility pin drifted');
if (lekiwiScenario.canonicalModel.legacyTaskRevision !== TASK_PATCH_REVISION) throw new Error('LeKiwi legacy task provenance drifted');
const lekiwiWorkspace = buildPatchedWorkspace('lekiwi', lekiwiScenario);
for (const file of ['main.py', 'trajectories.py', 'robot_config.py', 'workcell.py']) if (!lekiwiWorkspace[file]) throw new Error(`LeKiwi physical starter missing ${file}`);
for (const token of ['from robobuddy.sim import connect', 'await connect(', 'await robot.send_action(', 'await robot.advance(', 'await robot.get_observation()', 'x.vel', 'waypoint_reached', 'base_pose', 'advance_visible', 'PRESENTATION_PERIOD_S']) {
  if (!lekiwiWorkspace['main.py'].includes(token) && !lekiwiWorkspace['trajectories.py'].includes(token)) throw new Error(`LeKiwi live physical starter missing ${token}`);
}
for (const forbidden of ['time.sleep(', 'lerobot', '.grasp(', '.attach(', '.teleport(', '.move_to(', 'LeKiwiClient']) if (lekiwiWorkspace['main.py'].includes(forbidden)) throw new Error(`LeKiwi physical starter exposes forbidden/legacy behavior ${forbidden}`);
if (!lekiwiWorkspace['trajectories.py'].includes('lekiwi_base')) throw new Error('LeKiwi starter must read the authoritative base body');
if (!lekiwiWorkspace['workcell.py'].includes('calibration')) throw new Error('LeKiwi workcell must preserve the calibration boundary');
const lekiwiLegacyScenario = await loadPatchedScenario('lekiwi', 'lekiwi-01-beaker-courier');
if (lekiwiLegacyScenario !== null) throw new Error('the retired legacy LeKiwi task id must no longer resolve as a workspace');

const visibleSo101 = tasksForProfile('so101');
if (visibleSo101.length !== 1 || visibleSo101[0].id !== 'so101-physical-block-transfer' || visibleSo101[0].simulationMode !== 'physical_mujoco') throw new Error(`SO-101 physical catalog must expose only the validated block transfer: ${JSON.stringify(visibleSo101)}`);
for (const legacyId of expected.so101) {
  if (SO101_LEGACY_TASK_CLASSIFICATION[legacyId]?.classification !== 'B_DEFERRED') throw new Error(`${legacyId}: legacy physical classification must remain deliberately deferred`);
  if (visibleSo101.some((item) => item.id === legacyId)) throw new Error(`${legacyId}: deferred legacy task leaked into the physical SO-101 catalog`);
}
const physicalScenario = await loadPatchedScenario('so101', 'so101-physical-block-transfer');
if (physicalScenario.modelPackage !== 'so101-manipulation-menagerie-8161bba-v1') throw new Error('SO-101 physical task model package drifted');
const physicalWorkspace = buildPatchedWorkspace('so101', physicalScenario);
for (const forbidden of ['time.sleep(', 'lerobot', '.grasp(', '.attach(', '.teleport(', '.move_to(']) if (physicalWorkspace['main.py'].includes(forbidden)) throw new Error(`SO-101 physical starter exposes forbidden/legacy behavior ${forbidden}`);

const sourceSimulator = fs.readFileSync(new URL('../src/source-simulator.js', import.meta.url), 'utf8');
for (const token of ['ScenarioV2Engine.create', 'engine.plant.sendAction', 'engine.plant.tick()', 'engine.plant.fault']) if (!sourceSimulator.includes(token)) throw new Error(`source simulator missing ${token}`);
for (const token of ["kind: 'bimanual'", "side: 'bimanual'", 'connectionConfig']) if (!sourceSimulator.includes(token)) throw new Error(`OpenArm legacy source-plant provenance path missing ${token}`);
if (sourceSimulator.includes('left tool target would enter the modeled work surface')) throw new Error('stale standalone tool-point collision gate remains');

const host = fs.readFileSync(new URL('../src/simulator-host.js', import.meta.url), 'utf8');
for (const token of ['OpenArmPhysicalSimulator', "physical && profileId === 'openarm'", 'So101PhysicalSimulator', 'LeKiwiPhysicalSimulator', "physical && profileId === 'lekiwi'"]) if (!host.includes(token)) throw new Error(`simulator host missing physical routing token ${token}`);

const app = fs.readFileSync(new URL('../src/app-v2.js', import.meta.url), 'utf8');
for (const token of ['taskSelect', 'loadPatchedScenario', 'buildPatchedWorkspace', 'physicalRuntime.start', 'getPhysicalSession', 'isPhysicalWorkspace']) if (!app.includes(token)) throw new Error(`task-aware app missing ${token}`);

console.log('pinned legacy fixtures + SO-101/OpenArm physical catalogs/starters: OK');
