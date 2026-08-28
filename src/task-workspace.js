import { TASK_PATCH_REVISION, TASK_PATCH_SOURCE } from './task-catalog.js';

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
  const header = [
    `# Reviewed atomic physical-target action trace.`,
    `# Source: ${TASK_PATCH_SOURCE}@${TASK_PATCH_REVISION}`,
    `# Scenario: ${scenario.id} — ${scenario.title}`,
    `# Every action below is an ordinary public robot.send_action() dictionary.`,
    `# Edit the numerical joint/gripper targets here and immediately rerun the simulation.`,
    `# RoboBuddy's source task generated this trace through its collision-checked reference plant;`,
    `# edits made here are revalidated by the pinned source plant during simulation.`,
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

function mainFile(scenario) {
  return `import time\n\nfrom robot_config import create_robot\nfrom trajectories import REFERENCE_ACTIONS\n\n# ${scenario.title}\n# This is physical-target Python. The browser simulates the same public\n# send_action/get_observation sequence; it does not add grasp(), attach(),\n# teleport(), or Cartesian convenience methods to the learner program.\n\nrobot = create_robot()\nrobot.connect()\n\ntry:\n    for step in REFERENCE_ACTIONS:\n        sent = robot.send_action(step["action"])\n        time.sleep(step["hold_seconds"])\n        observation = robot.get_observation()\n        print(f'{step["index"]:02d} {step["label"]}', observation)\nfinally:\n    robot.disconnect()\n`;
}

function workcellFile(scenario) {
  const summary = {
    scenario_id: scenario.id,
    title: scenario.title,
    robot_id: scenario.robotId,
    canonical_model: scenario.canonicalModel,
    frames: scenario.frames,
    task_patch_repository: TASK_PATCH_SOURCE,
    task_patch_revision: TASK_PATCH_REVISION,
  };
  return `# Read-only reference geometry copied from the pinned reviewed mission.\n# Robot motion is NOT loaded from this file; motion remains visible in trajectories.py.\nWORKCELL = ${py(summary)}\n`;
}

export function buildPatchedWorkspace(profileId, scenario) {
  if (!scenario) throw new Error('A pinned scenario is required to build the patched workspace.');
  if (profileId !== 'openarm' && profileId !== 'so101') throw new Error(`No patched physical workspace generator for ${profileId}.`);
  return {
    'main.py': mainFile(scenario),
    'trajectories.py': trajectoryModule(scenario),
    'robot_config.py': profileId === 'openarm' ? openArmConfig() : so101Config(),
    'workcell.py': workcellFile(scenario),
  };
}
