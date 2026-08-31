export const TASK_PATCH_REVISION = '75fe2669c0ab0b029986de424c69162071174df8';
export const TASK_PATCH_SOURCE = 'jivishov/RoboBuddy_AI';

const ROOT = `https://cdn.jsdelivr.net/gh/${TASK_PATCH_SOURCE}@${TASK_PATCH_REVISION}/missions/lab-assistant/v2/definitions`;
export const UNITREE_G1_VISUAL_REVISION = '66d18a029a0caeb6a6075e681dbd9ecd6b22affa';

const task = (profileId, family, file, id, title, robotId) => Object.freeze({
  profileId, family, file, id, title, robotId,
  url: `${ROOT}/${family}/${file}`,
});

export const PATCH_TASKS = Object.freeze({
  openarm: Object.freeze([
    task('openarm', 'openarm', 'openarm-04-filtration-workcell.json', 'openarm-04-filtration-workcell', 'Bimanual Heater and Ring-Stand Stack', 'openarm_v2_bimanual'),
  ]),
  so101: Object.freeze([
    task('so101', 'so101', 'so101-v2-06-quantitative-transfer.json', 'so101-v2-06-quantitative-transfer', 'Measured Two-Bottle Transfer Workcell', 'so101_follower'),
    task('so101', 'so101', 'so101-v2-08-burette-initial-reading.json', 'so101-v2-08-burette-initial-reading', 'Burette Receiver Clearance Calibration', 'so101_follower'),
    task('so101', 'so101', 'so101-v2-09-vacuum-filtration.json', 'so101-v2-09-vacuum-filtration', 'Vacuum Workcell Keep-Clear Preflight', 'so101_follower'),
  ]),
  lekiwi: Object.freeze([
    task('lekiwi', 'lekiwi', 'lekiwi-01-beaker-courier.json', 'lekiwi-01-beaker-courier', 'Beaker Courier', 'lekiwi_sim'),
  ]),
});

const UNITREE_G1_RIG_SCENARIO = Object.freeze({
  schema: 'robobuddy.ide-rig-inspection.v1',
  simulationMode: 'kinematic_pose',
  workspaceRevision: 'unitree-g1-rig-v1',
  id: 'unitree-g1-kinematic-pose-inspection',
  title: 'Unitree G1 29-DoF Kinematic Pose Inspection',
  brief: 'Inspect the canonical Unitree G1 mesh through bounded named joint poses. This workspace deliberately has no collision/contact plant, gait, balance, or hardware-control claim.',
  robotId: 'unitree_g1_29dof',
  canonicalModel: Object.freeze({
    repository: 'jivishov/RoboBuddy_AI',
    revision: UNITREE_G1_VISUAL_REVISION,
    module: 'simulator/js/robot-mesh-data-unitree-g1.js',
    sourceRepository: 'unitreerobotics/unitree_ros',
    sourceRevision: 'dd4fa6866e523ad61324f658d63736e4eda3a6e4',
    sourcePath: 'robots/g1_description/g1_29dof.urdf',
    license: 'BSD-3-Clause',
  }),
  frames: Object.freeze({
    geometry: 'Three.js Y-up metres',
    transforms: 'Three.js Y-up millimetres',
    root: 'fixed visual root; no locomotion model',
  }),
  portablePython: Object.freeze({
    referenceActions: Object.freeze([
      Object.freeze({
        label: 'Upper-body joint-pose inspection',
        hold_seconds: 0.35,
        action: Object.freeze({
          waist_pitch_joint: 8,
          left_shoulder_pitch_joint: -35, left_shoulder_roll_joint: 28, left_elbow_joint: 45, left_wrist_pitch_joint: -10,
          right_shoulder_pitch_joint: -35, right_shoulder_roll_joint: -28, right_elbow_joint: 45, right_wrist_pitch_joint: -10,
        }),
      }),
      Object.freeze({
        label: 'Lower-body joint-pose inspection (root fixed)',
        hold_seconds: 0.35,
        action: Object.freeze({
          left_hip_roll_joint: 8, left_knee_joint: 22, left_ankle_pitch_joint: -10,
          right_hip_roll_joint: -8, right_knee_joint: 22, right_ankle_pitch_joint: -10,
        }),
      }),
      Object.freeze({
        label: 'Return inspected joints to neutral',
        hold_seconds: 0.35,
        action: Object.freeze({
          waist_pitch_joint: 0,
          left_shoulder_pitch_joint: 0, left_shoulder_roll_joint: 0, left_elbow_joint: 0, left_wrist_pitch_joint: 0,
          right_shoulder_pitch_joint: 0, right_shoulder_roll_joint: 0, right_elbow_joint: 0, right_wrist_pitch_joint: 0,
          left_hip_roll_joint: 0, left_knee_joint: 0, left_ankle_pitch_joint: 0,
          right_hip_roll_joint: 0, right_knee_joint: 0, right_ankle_pitch_joint: 0,
        }),
      }),
    ]),
  }),
});

export const UNITREE_G1_RIG_TASKS = Object.freeze([
  Object.freeze({
    profileId: 'unitree',
    id: UNITREE_G1_RIG_SCENARIO.id,
    title: UNITREE_G1_RIG_SCENARIO.title,
    robotId: UNITREE_G1_RIG_SCENARIO.robotId,
    simulationMode: 'kinematic_pose',
    source: `RoboBuddy_AI@${UNITREE_G1_VISUAL_REVISION}/simulator/js/robot-mesh-data-unitree-g1.js`,
  }),
]);

const MICRODUCK_SCENARIO = Object.freeze({
  schema: 'robobuddy.microduck-workspace.v1', simulationMode: 'policy_sim', workspaceRevision: 'microduck-cycle04-live-python-v1',
  id: 'microduck-policy-demonstrator', title: 'MicroDuck Articulated Policy Demonstrator', robotId: 'microduck_runtime_visual', variant: 'walking',
  brief: 'Run live async Python against the exact pinned policies and approximate browser dynamics while inspecting the official compact runtime visual and modeled state.',
  canonicalModel: Object.freeze({ repository: 'pollen-robotics/microduck', revision: '590b986bd8c0d50ae02cb3ea2f59c463b6828168', sourcePath: 'robotctl/assets/duck.bin', hierarchySourcePath: 'kinematics/assets/alpha/robot_walk.xml', geometry: 'official compact robotctl monitor mesh' }),
  frames: Object.freeze({ geometry: 'Source Z-up metres converted to Three.js Y-up and displayed at millimetre scale', hierarchy: 'pinned runtime XML and DUCK v1 body records', mouthRollersContacts: 'original configured approximations' }),
  portablePython: Object.freeze({ referenceActions: Object.freeze([]) }),
});
const MICRODUCK_TASKS = Object.freeze([Object.freeze({ profileId:'microduck', id:MICRODUCK_SCENARIO.id, title:MICRODUCK_SCENARIO.title, robotId:MICRODUCK_SCENARIO.robotId, simulationMode:'policy_sim' })]);

const cache = new Map();

export function tasksForProfile(profileId) {
  if (profileId === 'unitree') return UNITREE_G1_RIG_TASKS;
  if (profileId === 'microduck') return MICRODUCK_TASKS;
  return PATCH_TASKS[profileId] || [];
}

export function defaultTaskId(profileId) {
  return tasksForProfile(profileId)[0]?.id || '';
}

export function taskDescriptor(profileId, taskId) {
  return tasksForProfile(profileId).find((item) => item.id === taskId) || tasksForProfile(profileId)[0] || null;
}

export async function loadPatchedScenario(profileId, taskId) {
  const descriptor = taskDescriptor(profileId, taskId);
  if (!descriptor) return null;
  if (descriptor.simulationMode === 'kinematic_pose') return structuredClone(UNITREE_G1_RIG_SCENARIO);
  if (descriptor.simulationMode === 'policy_sim') return structuredClone(MICRODUCK_SCENARIO);
  if (cache.has(descriptor.id)) return structuredClone(cache.get(descriptor.id));
  const response = await fetch(descriptor.url, { cache: 'force-cache' });
  if (!response.ok) throw new Error(`Pinned task ${descriptor.id} returned HTTP ${response.status}.`);
  const scenario = await response.json();
  if (scenario?.schema !== 'robobuddy.lab-scenario.v2') throw new Error(`${descriptor.id}: unexpected scenario schema.`);
  if (scenario.id !== descriptor.id) throw new Error(`${descriptor.id}: pinned task id mismatch (${scenario.id || 'missing'}).`);
  if (scenario.robotId !== descriptor.robotId) throw new Error(`${descriptor.id}: robot id mismatch (${scenario.robotId || 'missing'}).`);
  if (scenario.title !== descriptor.title) throw new Error(`${descriptor.id}: reviewed title mismatch; refusing silent task drift.`);
  const actions = scenario?.portablePython?.referenceActions;
  if (!Array.isArray(actions) || actions.length < 2) throw new Error(`${descriptor.id}: reviewed portablePython.referenceActions are missing.`);
  for (const [index, item] of actions.entries()) {
    if (!item || typeof item.action !== 'object' || Array.isArray(item.action)) throw new Error(`${descriptor.id}: reference action ${index + 1} is invalid.`);
    if (!Number.isFinite(Number(item.hold_seconds))) throw new Error(`${descriptor.id}: reference action ${index + 1} has no finite hold_seconds.`);
  }
  cache.set(descriptor.id, scenario);
  return structuredClone(scenario);
}

export function taskPatchProvenance(descriptor) {
  if (descriptor?.simulationMode === 'kinematic_pose') {
    return {
      repository: 'jivishov/RoboBuddy_AI',
      revision: UNITREE_G1_VISUAL_REVISION,
      scenarioId: descriptor.id,
      source: descriptor.source,
      simulationMode: 'kinematic_pose',
    };
  }
  return descriptor ? {
    repository: TASK_PATCH_SOURCE,
    revision: TASK_PATCH_REVISION,
    scenarioId: descriptor.id,
    source: descriptor.url,
  } : null;
}

export function isKinematicRigScenario(scenario) {
  return scenario?.simulationMode === 'kinematic_pose';
}
