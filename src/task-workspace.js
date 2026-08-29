import { TASK_PATCH_REVISION, TASK_PATCH_SOURCE } from './task-catalog.js';

const isKinematicPoseScenario = (scenario) => scenario?.simulationMode === 'kinematic_pose';

function py(value, depth = 0) {
  if (value === null || value === undefined) return 'None';
  if (typeof value === 'boolean') return value ? 'True' : 'False';
  if (typeof value === 'number') return Number.isFinite(value) ? String(value) : 'None';
  if (typeof value === 'string') return JSON.stringify(value);
  const indent = '    '.repeat(depth);
  const child = '    '.repeat(depth + 1);
  if (Array.isArray(value)) {
    if (!value.length) return '[]';
    return `[\n${value.map((item) => `${child}${py(item, depth + 1)},`).join('\n')}\n${indent}]`;
  }
  const entries = Object.entries(value);
  if (!entries.length) return '{}';
  return `{\n${entries.map(([key, item]) => `${child}${JSON.stringify(key)}: ${py(item, depth + 1)},`).join('\n')}\n${indent}}`;
}

function trajectoryModule(scenario) {
  const header = isKinematicPoseScenario(scenario)
    ? [
      '# Unitree G1 kinematic-pose inspection trace.',
      `# Canonical mesh source: ${scenario.canonicalModel.repository}@${scenario.canonicalModel.revision}`,
      `# Scenario: ${scenario.id} — ${scenario.title}`,
      '# Every action is a bounded named joint-angle dictionary for the browser pose rig.',
      '# This is not a Unitree SDK, collision/contact plant, gait, balance, or hardware-control program.',
      '',
    ].join('\n')
    : [
      '# Reviewed atomic physical-target action trace.',
      `# Source: ${TASK_PATCH_SOURCE}@${TASK_PATCH_REVISION}`,
      `# Scenario: ${scenario.id} — ${scenario.title}`,
      '# Every action below is an ordinary public robot.send_action() dictionary.',
      '# Edit the numerical joint/gripper/base targets here and immediately rerun the simulation.',
      '# RoboBuddy source generated this trace through its reviewed reference plant;',
      '# edits made here are revalidated by that same pinned source plant during simulation.',
      '',
    ].join('\n');
  const rows = scenario.portablePython.referenceActions.map((record, index) => ({
    index: index + 1,
    label: String(record.label || `action ${index + 1}`),
    hold_seconds: Number(record.hold_seconds),
    action: record.action,
  }));
  return `${header}REFERENCE_ACTIONS = ${py(rows)}\n`;
}

function openArmConfig() {
  return `from lerobot.robots.openarm_follower import OpenArmFollowerConfigBase\nfrom lerobot.robots.bi_openarm_follower import BiOpenArmFollower, BiOpenArmFollowerConfig\n\nLEFT_CAN = "can0"\nRIGHT_CAN = "can1"\n\ndef create_robot():\n    config = BiOpenArmFollowerConfig(\n        left_arm_config=OpenArmFollowerConfigBase(\n            port=LEFT_CAN,\n            side="left",\n            cameras={},\n        ),\n        right_arm_config=OpenArmFollowerConfigBase(\n            port=RIGHT_CAN,\n            side="right",\n            cameras={},\n        ),\n        cameras={},\n    )\n    return BiOpenArmFollower(config)\n`;
}

function so101Config() {
  return `from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig\n\nSERIAL_PORT = "/dev/ttyACM0"\n\ndef create_robot():\n    return SO101Follower(SO101FollowerConfig(\n        port=SERIAL_PORT,\n        cameras={},\n    ))\n`;
}

function lekiwiConfig() {
  return `from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig\n\nROBOT_IP = "192.168.4.1"\n\ndef create_robot():\n    return LeKiwiClient(LeKiwiClientConfig(\n        remote_ip=ROBOT_IP,\n        cameras={},\n    ))\n`;
}

function unitreeConfig() {
  return `# Browser-only adapter used by this kinematic visual workspace.
# It is deliberately not a Unitree SDK or hardware controller.
from robobuddy.simulation import UnitreeG1KinematicPoseAdapter

def create_robot():
    return UnitreeG1KinematicPoseAdapter()
`;
}

function mainFile(scenario) {
  const boundary = isKinematicPoseScenario(scenario)
    ? '# This is a browser-only Unitree G1 kinematic-pose workspace. The adapter\n# only displays bounded source-manifest joint angles; it is not a Unitree SDK,\n# contact simulation, gait/balance system, or hardware-control path.'
    : '# This is physical-target Python. The browser simulates the same public\n# send_action/get_observation sequence; it does not add grasp(), attach(),\n# teleport(), or Cartesian convenience methods to the learner program.';
  return `import time\n\nfrom robot_config import create_robot\nfrom trajectories import REFERENCE_ACTIONS\n\n# ${scenario.title}\n${boundary}\n\nrobot = create_robot()\nrobot.connect()\n\ntry:\n    for step in REFERENCE_ACTIONS:\n        sent = robot.send_action(step["action"])\n        time.sleep(step["hold_seconds"])\n        observation = robot.get_observation()\n        print(f'{step["index"]:02d} {step["label"]}', observation)\nfinally:\n    robot.disconnect()\n`;
}

function workcellFile(scenario) {
  const source = isKinematicPoseScenario(scenario)
    ? { repository: scenario.canonicalModel.repository, revision: scenario.canonicalModel.revision }
    : { repository: TASK_PATCH_SOURCE, revision: TASK_PATCH_REVISION };
  const summary = {
    scenario_id: scenario.id,
    title: scenario.title,
    robot_id: scenario.robotId,
    canonical_model: scenario.canonicalModel,
    frames: scenario.frames,
    source_repository: source.repository,
    source_revision: source.revision,
    simulation_mode: scenario.simulationMode || 'source_plant',
  };
  return `# Read-only reference geometry copied from the pinned reviewed mission.\n# Robot motion is NOT loaded from this file; motion remains visible in trajectories.py.\nWORKCELL = ${py(summary)}\n`;
}

export function buildPatchedWorkspace(profileId, scenario) {
  if (!scenario) throw new Error('A scenario is required to build the workspace.');
  const configs = { openarm: openArmConfig, so101: so101Config, lekiwi: lekiwiConfig, unitree: unitreeConfig };
  const configFactory = configs[profileId];
  if (!configFactory) throw new Error(`No workspace generator for ${profileId}.`);
  return {
    'main.py': mainFile(scenario),
    'trajectories.py': trajectoryModule(scenario),
    'robot_config.py': configFactory(),
    'workcell.py': workcellFile(scenario),
  };
}
