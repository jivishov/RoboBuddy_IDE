import { asimovWorkspaceFiles } from './physics/asimov-workspaces.js';
import { TASK_PATCH_REVISION, TASK_PATCH_SOURCE } from './task-catalog.js';

const isKinematicPoseScenario = (scenario) => scenario?.simulationMode === 'kinematic_pose';
const isPhysicalMujocoScenario = (scenario) => scenario?.simulationMode === 'physical_mujoco';

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
  const rows = scenario.portablePython.referenceActions.map((record, index) => ({ index: index + 1, label: String(record.label || `action ${index + 1}`), hold_seconds: Number(record.hold_seconds), action: record.action }));
  return `${header}REFERENCE_ACTIONS = ${py(rows)}\n`;
}

function lekiwiPhysicalWorkspace(scenario) {
  const stages = scenario.portablePython.referenceActions.map((record, index) => ({
    index: index + 1,
    label: String(record.label),
    kind: String(record.kind),
    duration_seconds: record.hold_seconds,
    targets_rad: record.targetsRad,
    waypoints: record.waypoints,
    timeout_seconds: record.timeout_seconds,
  }));
  const route = scenario.route;
  const limits = {
    max_linear_ms: 0.22, max_yaw_deg_s: 51.566, linear_gain: 1.6, yaw_gain: 2.2,
    position_tolerance_m: 0.015, yaw_tolerance_rad: 0.06, arrival_speed_ms: 0.03, arrival_yaw_rate_rad_s: 0.15,
  };
  const q = '"'.repeat(3);
  const mainPy = [
    '# LeKiwi V1 physical beaker courier.',
    '# robobuddy.sim.v1 uses SI units, radians and simulation time.',
    '# A chassis request is NOT motion: send_action({"x.vel": ...}) is converted through the pinned',
    '# LeRobot Kiwi mapping into bounded wheel velocity targets, and MuJoCo decides what the base',
    '# actually does. Read achieved motion from get_observation(), never from what you sent.',
    '# The beaker is a free body: there is no grasp(), attach(), snap() or teleport() call.',
    'from robobuddy.sim import connect',
    'from robot_config import ROBOT_ID, CONTROLLER_PERIOD_S, PHYSICS_TIMESTEP_S, PRESENTATION_PERIOD_S',
    'from trajectories import STAGES, DRIVE_LIMITS, base_pose, chassis_command, waypoint_reached',
    '',
    'async def advance_visible(seconds):',
    '    # Advance only through the authoritative simulation, but in bounded chunks so the browser',
    '    # can render real intermediate MuJoCo observations instead of showing only stage endpoints.',
    '    remaining_steps = max(0, int(round(seconds / PHYSICS_TIMESTEP_S)))',
    '    chunk_steps = max(1, int(round(PRESENTATION_PERIOD_S / PHYSICS_TIMESTEP_S)))',
    '    observation = await robot.get_observation()',
    '    while remaining_steps > 0:',
    '        steps = min(chunk_steps, remaining_steps)',
    '        observation = await robot.advance(steps * PHYSICS_TIMESTEP_S)',
    '        remaining_steps -= steps',
    '    return observation',
    '',
    'robot = await connect(ROBOT_ID)',
    '',
    'try:',
    '    for stage in STAGES:',
    '        if stage["kind"] == "drive":',
    '            for waypoint in stage["waypoints"]:',
    '                elapsed = 0.0',
    '                while elapsed < stage["timeout_seconds"]:',
    '                    pose = base_pose(await robot.get_observation())',
    '                    if waypoint_reached(pose, waypoint, DRIVE_LIMITS):',
    '                        break',
    '                    command = chassis_command(pose, waypoint, DRIVE_LIMITS)',
    '                    await robot.send_action({**command, **stage["targets_rad"]}, max_steps=4000)',
    '                    await robot.advance(CONTROLLER_PERIOD_S)',
    '                    elapsed += CONTROLLER_PERIOD_S',
    '                await robot.send_action({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0, **stage["targets_rad"]}, max_steps=2000)',
    '                pose = base_pose(await robot.get_observation())',
    '                print(f\'{stage["index"]:02d} {stage["label"]} -> actual base x={pose["x"]:.4f} y={pose["y"]:.4f} yaw={pose["yaw"]:.4f}\')',
    '        else:',
    '            await robot.send_action({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0, **stage["targets_rad"]}, max_steps=4000)',
    '            observation = await advance_visible(stage["duration_seconds"])',
    '            beaker = observation["bodies"]["empty_beaker"]["positionM"]',
    '            print(f\'{stage["index"]:02d} {stage["label"]} t={observation["simulationTimeSeconds"]:.2f}s beaker={[round(v, 4) for v in beaker]}\')',
    '',
    '    final_observation = await robot.get_observation()',
    '    pose = base_pose(final_observation)',
    '    print("final actual base", round(pose["x"], 4), round(pose["y"], 4), round(pose["yaw"], 4))',
    '    print("final actual beaker", [round(v, 4) for v in final_observation["bodies"]["empty_beaker"]["positionM"]])',
    'finally:',
    '    await robot.disconnect()',
    '',
  ].join('\n');
  const trajectoriesPy = [
    '# Versioned physical controller stages plus the bounded chassis controller.',
    '# Arm targets are radians. Chassis fields are requests in LeRobot public units',
    '# (x.vel/y.vel m/s, theta.vel deg/s). Waypoint completion is decided from the observed',
    '# MuJoCo base pose, never from a command, a timer, or a planner cell.',
    'import math',
    '',
    `DRIVE_LIMITS = ${py(limits)}`,
    '',
    `ROUTE = ${py({ planner: route.planner, outbound: route.outbound, inbound: route.inbound, home_xy_m: route.homeXYM, service_stop_xy_m: route.serviceStopXYM, restricted_stop_xy_m: route.restrictedStopXYM })}`,
    '',
    `STAGES = ${py(stages)}`,
    '',
    '',
    'def base_pose(observation):',
    `    ${q}Achieved base pose read from the authoritative MuJoCo free body.${q}`,
    '    body = observation["bodies"]["lekiwi_base"]',
    '    w, x, y, z = body["quaternionWxyz"]',
    '    linear = body.get("linearVelocityMS", [0.0, 0.0, 0.0])',
    '    angular = body.get("angularVelocityRadS", [0.0, 0.0, 0.0])',
    '    return {',
    '        "x": body["positionM"][0],',
    '        "y": body["positionM"][1],',
    '        "yaw": math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),',
    '        "speed": math.hypot(linear[0], linear[1]),',
    '        "yaw_rate": angular[2],',
    '    }',
    '',
    '',
    'def wrap_angle(value):',
    '    while value > math.pi:',
    '        value -= 2.0 * math.pi',
    '    while value < -math.pi:',
    '        value += 2.0 * math.pi',
    '    return value',
    '',
    '',
    'def chassis_command(pose, waypoint, limits):',
    `    ${q}Bounded proportional chassis-velocity request in the base body frame.${q}`,
    '    dx = waypoint["xM"] - pose["x"]',
    '    dy = waypoint["yM"] - pose["y"]',
    '    cos, sin = math.cos(pose["yaw"]), math.sin(pose["yaw"])',
    '    vx = limits["linear_gain"] * (dx * cos + dy * sin)',
    '    vy = limits["linear_gain"] * (-dx * sin + dy * cos)',
    '    speed = math.hypot(vx, vy)',
    '    if speed > limits["max_linear_ms"]:',
    '        vx *= limits["max_linear_ms"] / speed',
    '        vy *= limits["max_linear_ms"] / speed',
    '    yaw_error = wrap_angle(waypoint["yawRad"] - pose["yaw"]) if waypoint.get("yawRad") is not None else 0.0',
    '    omega_deg = max(-limits["max_yaw_deg_s"], min(limits["max_yaw_deg_s"], limits["yaw_gain"] * yaw_error * 180.0 / math.pi))',
    '    return {"x.vel": vx, "y.vel": vy, "theta.vel": omega_deg}',
    '',
    '',
    'def waypoint_reached(pose, waypoint, limits):',
    `    ${q}Arrival is an observation about the plant, not about the command that was sent.${q}`,
    '    position_error = math.hypot(waypoint["xM"] - pose["x"], waypoint["yM"] - pose["y"])',
    '    yaw_error = abs(wrap_angle(waypoint["yawRad"] - pose["yaw"])) if waypoint.get("yawRad") is not None else 0.0',
    '    return (position_error <= limits["position_tolerance_m"]',
    '            and yaw_error <= limits["yaw_tolerance_rad"]',
    '            and pose["speed"] <= limits["arrival_speed_ms"]',
    '            and abs(pose["yaw_rate"]) <= limits["arrival_yaw_rate_rad_s"])',
    '',
  ].join('\n');
  const configPy = [
    '# Browser-only physical simulation identity. No ZMQ socket or hardware transport is opened.',
    `ROBOT_ID = ${JSON.stringify(scenario.robotId)}`,
    'PHYSICAL_API_VERSION = "robobuddy.sim.v1"',
    'ANGLE_UNIT = "rad"',
    'TIME_UNIT = "s"',
    `CHASSIS_UNITS = ${py(scenario.physicalApi.chassisUnits)}`,
    'CONTROLLER_PERIOD_S = 0.02',
    'PHYSICS_TIMESTEP_S = 0.002',
    'PRESENTATION_PERIOD_S = 0.05',
    '',
  ].join('\n');
  const workcellPy = [
    '# Physical-workspace metadata. Declared workcell geometry is configured educational geometry',
    '# and simulator evidence, not measured laboratory hardware or installed-robot calibration.',
    `WORKCELL = ${py({ scenario_id: scenario.id, title: scenario.title, simulation_mode: scenario.simulationMode, physical_scene_id: scenario.physicalSceneId, model_package: scenario.modelPackage, model_id: scenario.modelId, route: scenario.route, evaluator: scenario.taskEvaluation, limitations: scenario.limitations })}`,
    '',
  ].join('\n');
  return { 'main.py': mainPy, 'trajectories.py': trajectoriesPy, 'robot_config.py': configPy, 'workcell.py': workcellPy };
}

function microduckPhysicalWorkspace(scenario) {
  const commands = scenario.id === 'microduck-physical-kick'
    ? [{ label: 'Request a right-foot kick; observe contact and ball motion', action: { skill: 'kick_right' }, duration_seconds: 3 }]
    : scenario.id === 'microduck-physical-groundcontact'
    ? [{ label: 'Stand through the deployed policy', action: { vx: 0 }, duration_seconds: 1 },
       { label: 'Request sitting', action: { skill: 'sit' }, duration_seconds: 1 },
       { label: 'Request rising (not a fall-recovery reset)', action: { skill: 'stand_up' }, duration_seconds: 1 }]
    : [{ label: 'Request forward walking through foot-floor contact', action: { vx: 0.35 }, duration_seconds: 2 }];
  const stages = commands.map((stage, index) => ({ index: index + 1, ...stage }));
  const main = [
    '# MicroDuck physical MuJoCo workspace. No hardware transport is opened.',
    '# Commands are requests to the deployed policy; only actual is measured simulation state.',
    '# A kick can miss. A posture request can fail. Neither is manufactured into success.',
    'from robobuddy.sim import connect',
    'from robot_config import ROBOT_ID, CONTROL_INTERVAL_S',
    'from trajectories import STAGES',
    '',
    'robot = await connect(ROBOT_ID)',
    'try:',
    '    for stage in STAGES:',
    '        accepted = await robot.send_action(stage["action"])',
    '        remaining_ticks = int(round(stage["duration_seconds"] / CONTROL_INTERVAL_S))',
    '        while remaining_ticks > 0:',
    '            ticks = min(10, remaining_ticks)',
    '            observation = await robot.advance(ticks * CONTROL_INTERVAL_S)',
    '            remaining_ticks -= ticks',
    '        actual = (await robot.get_observation())["actual"]',
    '        print(stage["label"], "actual trunk (m)", actual["trunkPositionM"], "tilt (deg)", actual["trunkTiltDeg"])',
    '    final = await robot.get_observation()',
    '    print("MicroDuck physical observation", final["actual"])',
    '    print("Measured task report", final["taskEvaluation"])',
    'finally:',
    '    await robot.disconnect()',
    '',
  ].join('\n');
  return {
    'main.py': main,
    'trajectories.py': `# Explicit policy commands, not direct joint or free-body state writes.\n# Durations use simulation time. Edit these commands for the selected collision plant.\nSTAGES = ${py(stages)}\n`,
    'robot_config.py': `# Browser simulation only. No serial, CAN or network robot connection.\nROBOT_ID = ${JSON.stringify(scenario.robotId)}\nPHYSICAL_API_VERSION = "robobuddy.sim.v1"\nCONTROL_INTERVAL_S = ${scenario.controller.controlIntervalSeconds}\n`,
    'workcell.py': `# Read-only physical workspace metadata, not installed-hardware calibration.\nWORKCELL = ${py({ scenario_id: scenario.id, physical_scene_id: scenario.physicalSceneId, model_package: scenario.modelPackage, capabilities: scenario.capabilities, limitations: scenario.limitations })}\n`,
  };
}

function unitreeG1PhysicalWorkspace(scenario) {
  const main = [
    '# Unitree G1 29-DoF physical MuJoCo workspace. No Unitree SDK or hardware transport is opened.',
    '# set_joint_targets() latches a bounded low-level command; it never means the joint got there.',
    '# stand() engages the verified posture controller. It is a posture hold, not balance recovery,',
    '# and it can fail. There is no walk(): walking is unsupported in this workspace.',
    'from robobuddy.sim import connect',
    'from robot_config import ROBOT_ID',
    'from trajectories import STAGES',
    '',
    'robot = await connect(ROBOT_ID)',
    'try:',
    '    # Let the free-base robot settle on its feet under gravity before commanding anything.',
    '    await robot.wait_sim(0.2)',
    '',
    '    # Earn the upright posture through the physical plant, then watch it hold.',
    '    await robot.stand()',
    '    await robot.wait_sim(2.0)',
    '    await robot.wait_sim(2.0)',
    '    state = await robot.get_state()',
    "    print('pelvis height (m)', state['root']['position_m'][2], 'tilt (rad)', state['root']['tilt_rad'])",
    "    print('left foot contacts', state['foot_contacts']['left'])",
    "    print('right foot contacts', state['foot_contacts']['right'])",
    '',
    '    for stage in STAGES:',
    '        await robot.set_joint_targets(stage["targets_rad"])',
    '        await robot.wait_sim(stage["duration_seconds"])',
    '        state = await robot.get_state()',
    '        for joint in stage["targets_rad"]:',
    '            record = state["joints"][joint]',
    '            # Requested, accepted and measured are three different numbers. Read all three.',
    "            print(stage['label'], joint,",
    "                  'requested', round(record['requested_target_rad'], 4),",
    "                  'accepted', round(record['accepted_target_rad'], 4),",
    "                  'measured', round(record['position_rad'], 4),",
    "                  'effort Nm', round(record['effort_nm'], 3))",
    'finally:',
    '    await robot.disconnect()',
    '',
  ].join('\n');
  const stages = [
    { index: 1, label: 'Turn the waist while the posture controller holds the legs', duration_seconds: 1.5, targets_rad: { waist_yaw_joint: 0.4 } },
    { index: 2, label: 'Raise the left arm', duration_seconds: 1.5, targets_rad: { left_shoulder_pitch_joint: -0.6, left_elbow_joint: 1.2 } },
    { index: 3, label: 'Return the commanded joints to the standing posture', duration_seconds: 1.5, targets_rad: { waist_yaw_joint: 0, left_shoulder_pitch_joint: 0.35, left_elbow_joint: 0.87 } },
  ];
  return {
    'main.py': main,
    'trajectories.py': `# Bounded joint targets in radians, in the source Unitree G1 joint names.\n# Every target is clamped to the source joint range before it becomes a torque, and no target\n# is ever written into the simulator as state.\n# Note: set_joint_targets() releases the standing controller, so commanding a leg joint here\n# will make the robot fall. That is the plant behaving correctly, not a defect.\nSTAGES = ${py(stages)}\n`,
    'robot_config.py': `# Browser simulation only. No serial, CAN, DDS or network robot connection.\nROBOT_ID = ${JSON.stringify(scenario.robotId)}\nPHYSICAL_API_VERSION = "robobuddy.sim.v1"\nANGLE_UNIT = "rad"\nTIME_UNIT = "s"\nPHYSICS_TIMESTEP_S = ${scenario.controller.physicsTimestepSeconds}\nLOWLEVEL_CONTROL_INTERVAL_S = ${scenario.controller.controlIntervalSeconds}\nSTAND_CONTROLLER = ${JSON.stringify(scenario.controller.standControllerId)}\nWALKING = "unsupported"\n`,
    'workcell.py': `# Read-only physical workspace metadata, not installed-hardware calibration.\nWORKCELL = ${py({ scenario_id: scenario.id, physical_scene_id: scenario.physicalSceneId, model_package: scenario.modelPackage, capabilities: scenario.capabilities, evaluator: scenario.taskEvaluation, limitations: scenario.limitations })}\n`,
  };
}

function physicalWorkspace(profileId, scenario) {
  if (profileId === 'asimov') return asimovWorkspaceFiles(scenario);
  if (profileId === 'microduck') return microduckPhysicalWorkspace(scenario);
  if (profileId === 'lekiwi') return lekiwiPhysicalWorkspace(scenario);
  if (profileId === 'unitree') return unitreeG1PhysicalWorkspace(scenario);
  const timestep = profileId === 'openarm' ? 0.001 : 0.005;
  const stages = scenario.portablePython.referenceActions.map((record, index) => ({
    index: index + 1,
    label: String(record.label),
    duration_seconds: Number(record.hold_seconds),
    targets_rad: record.targetsRad,
    max_steps: Math.min(5000, Math.max(1, Math.ceil(Number(record.hold_seconds) / timestep) + (profileId === 'openarm' ? 1000 : 20))),
  }));
  if (profileId === 'so101') {
    const release = stages.find((stage) => stage.label.toLowerCase().includes('release'));
    if (release) release.max_steps = Math.max(release.max_steps, 260);
  }
  const objectLines = profileId === 'openarm'
    ? `        flask = observation["bodies"]["flask"]["positionM"]\n        beaker = observation["bodies"]["beaker"]["positionM"]\n        print(f'{stage["index"]:02d} actual t={observation["simulationTimeSeconds"]:.3f}s flask={flask} beaker={beaker}')`
    : `        block = observation["bodies"]["benchmark_block"]["positionM"]\n        print(f'{stage["index"]:02d} actual t={observation["simulationTimeSeconds"]:.3f}s block={block}')`;
  const finalLines = profileId === 'openarm'
    ? `    print("final actual flask", final_observation["bodies"]["flask"]["positionM"])\n    print("final actual beaker", final_observation["bodies"]["beaker"]["positionM"])`
    : `    print("final actual block", final_observation["bodies"]["benchmark_block"]["positionM"])`;
  const taskLabel = profileId === 'openarm' ? 'OpenArm V2 bimanual dry stacking task' : 'SO-101 physical MuJoCo benchmark';
  return {
    'main.py': `# ${taskLabel}.\n# robobuddy.sim.v1 uses SI units, radians, and simulation time.\n# send_action() acknowledges a latched target; it does NOT mean the joint reached it.\n# Free objects move only through MuJoCo gravity/contact; no grasp/attach/teleport API exists.\nfrom robobuddy.sim import connect\nfrom robot_config import ROBOT_ID\nfrom trajectories import STAGES\n\nrobot = await connect(ROBOT_ID)\n\ntry:\n    for stage in STAGES:\n        targets = stage["targets_rad"]\n        duration = stage["duration_seconds"]\n        if targets:\n            accepted = await robot.send_action(targets, max_steps=stage["max_steps"]${profileId === 'openarm' ? ', duration_seconds=duration' : ''})\n            print(f'{stage["index"]:02d} {stage["label"]} accepted', accepted["status"])\n        observation = await robot.advance(duration)\n${objectLines}\n\n    final_observation = await robot.get_observation()\n${finalLines}\nfinally:\n    await robot.disconnect()\n`,
    'trajectories.py': `# Versioned physical controller stages.\n# Joint targets are radians. Durations are simulation seconds, not wall-clock sleeps.\n# Objects are never attached, snapped, parented, welded, or teleported.\nSTAGES = ${py(stages)}\n`,
    'robot_config.py': `# Browser-only physical simulation identity. No serial/CAN transport is opened.\nROBOT_ID = ${JSON.stringify(scenario.robotId)}\nPHYSICAL_API_VERSION = "robobuddy.sim.v1"\nANGLE_UNIT = "rad"\nTIME_UNIT = "s"\n`,
    'workcell.py': `# Physical-workspace metadata. Declared benchmark geometry is simulator evidence,\n# not measured laboratory hardware dimensions or installed-hardware calibration.\nWORKCELL = ${py({ scenario_id: scenario.id, title: scenario.title, simulation_mode: scenario.simulationMode, physical_scene_id: scenario.physicalSceneId, model_package: scenario.modelPackage, model_id: scenario.modelId, evaluator: scenario.taskEvaluation, limitations: scenario.limitations })}\n`,
  };
}

function openArmConfig() {
  return `from lerobot.robots.openarm_follower import OpenArmFollowerConfigBase\nfrom lerobot.robots.bi_openarm_follower import BiOpenArmFollower, BiOpenArmFollowerConfig\n\nLEFT_CAN = "can0"\nRIGHT_CAN = "can1"\n\ndef create_robot():\n    config = BiOpenArmFollowerConfig(\n        left_arm_config=OpenArmFollowerConfigBase(port=LEFT_CAN, side="left", cameras={}),\n        right_arm_config=OpenArmFollowerConfigBase(port=RIGHT_CAN, side="right", cameras={}),\n        cameras={},\n    )\n    return BiOpenArmFollower(config)\n`;
}
function so101Config() { return `from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig\n\nSERIAL_PORT = "/dev/ttyACM0"\n\ndef create_robot():\n    return SO101Follower(SO101FollowerConfig(port=SERIAL_PORT, cameras={}))\n`; }
function lekiwiConfig() { return `from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig\n\nROBOT_IP = "192.168.4.1"\n\ndef create_robot():\n    return LeKiwiClient(LeKiwiClientConfig(remote_ip=ROBOT_IP, cameras={}))\n`; }
function unitreeConfig() { return `# Browser-only adapter used by this kinematic visual workspace.\n# It is deliberately not a Unitree SDK or hardware controller.\nfrom robobuddy.simulation import UnitreeG1KinematicPoseAdapter\n\ndef create_robot():\n    return UnitreeG1KinematicPoseAdapter()\n`; }
function microduckConfig() { return `# Browser-only MicroDuck policy-simulation client.\n# connect() acquires a simulator lease; it never opens a socket or discovers hardware.\nfrom microduck import MicroDuck\n\ndef create_robot():\n    return MicroDuck()\n`; }

function microduckWorkspace(scenario) {
  return {
    'main.py': `# Live browser-only MicroDuck policy simulation.\n# Exact pinned ONNX policies drive original approximate browser dynamics; this is not hardware or RL-environment parity.\nfrom robot_config import create_robot\n\nrobot = create_robot()\nawait robot.connect()\ntry:\n    await robot.enable(True)\n    # Request four modeled seconds at 0.30 m/s (about 1.2 m in this approximation).\n    # Sleep follows modeled time; the safety lease uses real time. Reissue\n    # movement between short sleeps so slow rendering does not expire it.\n    for _ in range(16):\n        await robot.move(0.30, 0.0, 0.0)\n        await robot.sleep(0.25)\n    state = await robot.get_state()\n    print("live modeled time", state["time"], "mode", state["mode"], "position", state["simulatedPose"]["position"])\n    await robot.stop()\nfinally:\n    await robot.disconnect()\n`,
    'trajectories.py': `# Callable examples for the catalog-backed robot methods. They are not run by main.py.\nasync def demonstrate_motion(robot):\n    await robot.head(neck_pitch=0.05, head_pitch=-0.1, head_yaw=0.2, head_roll=0.0)\n    await robot.look(0.35, 0.05, 0.18, neck_pitch=0.0)\n    await robot.pose(z=-0.01, roll=0.04, pitch=-0.04, active=True)\n    await robot.mouth(0.35)\n    await robot.do("kick_left")\n    await robot.init()\n    await robot.relax()\n\nasync def demonstrate_modes_and_audio(robot):\n    current = await robot.mode()\n    await robot.set_mode("roller" if current == "walking" else "walking")\n    # Audio calls require a trusted human unlock in the visible control deck.\n    await robot.sound("chirp")\n    await robot.sound("wheee", hold=True)\n    await robot.sound("wheee", hold=False)\n    await robot.theremin(True)\n    await robot.theremin(False)\n    await robot.chorale(True, piece="wistful", voices=2)\n    await robot.chorale(False)\n`,
    'robot_config.py': microduckConfig(),
    'workcell.py': `# Callable examples for browser-model presentation and workcell extensions.\n# Camera, ToF, contacts, geometry, rollers, mouth pivot, and dynamics are modeled approximations.\nWORKCELL = ${py({ scenario_id: scenario.id, simulation_mode: 'policy_sim', fidelity: 'exact pinned ONNX over original approximate browser dynamics; no RL-environment, locomotion, contact, or hardware parity' })}\n\n# API values remain stable; get_state() returns each view's visible identity and truthful frame.\nCAMERA_VIEWS = {\n    "orbit": "Overview: world-frame robot + ball context",\n    "chase": "Follow: stable robot-root third-person view",\n    "head": "Head POV: modeled head_camera render, not hardware video",\n}\n\nasync def configure_modeled_workcell(robot):\n    await robot.set_color("lavender")\n    await robot.spawn_ball([0.28, 0.0, 0.035])\n    await robot.set_tof_stimulus(0.32)\n    await robot.set_camera("chase")\n    camera = (await robot.get_state())["virtualCamera"]\n    print("camera", camera["name"], camera["frame"], camera["purpose"])\n    # Reset preserves the selected camera mode and restores its deterministic fit.\n    await robot.reset()\n`,
  };
}

function mainFile(scenario) {
  const boundary = isKinematicPoseScenario(scenario)
    ? '# This is a browser-only Unitree G1 kinematic-pose workspace. The adapter only displays bounded source-manifest joint angles.'
    : '# This is physical-target Python. The browser simulates the same public send_action/get_observation sequence; it does not add grasp(), attach(), teleport(), or Cartesian convenience methods.';
  return `import time\n\nfrom robot_config import create_robot\nfrom trajectories import REFERENCE_ACTIONS\n\n# ${scenario.title}\n${boundary}\n\nrobot = create_robot()\nrobot.connect()\n\ntry:\n    for step in REFERENCE_ACTIONS:\n        sent = robot.send_action(step["action"])\n        time.sleep(step["hold_seconds"])\n        observation = robot.get_observation()\n        print(f'{step["index"]:02d} {step["label"]}', observation)\nfinally:\n    robot.disconnect()\n`;
}
function workcellFile(scenario) {
  const source = isKinematicPoseScenario(scenario) ? { repository: scenario.canonicalModel.repository, revision: scenario.canonicalModel.revision } : { repository: TASK_PATCH_SOURCE, revision: TASK_PATCH_REVISION };
  const summary = { scenario_id: scenario.id, title: scenario.title, robot_id: scenario.robotId, canonical_model: scenario.canonicalModel, frames: scenario.frames, source_repository: source.repository, source_revision: source.revision, simulation_mode: scenario.simulationMode || 'source_plant' };
  return `# Read-only reference geometry copied from the pinned reviewed mission.\n# Robot motion is NOT loaded from this file; motion remains visible in trajectories.py.\nWORKCELL = ${py(summary)}\n`;
}

export function buildPatchedWorkspace(profileId, scenario) {
  if (!scenario) throw new Error('A scenario is required to build the workspace.');
  if (isPhysicalMujocoScenario(scenario)) return physicalWorkspace(profileId, scenario);
  if (scenario.simulationMode === 'policy_sim') return microduckWorkspace(scenario);
  const configs = { openarm: openArmConfig, so101: so101Config, lekiwi: lekiwiConfig, unitree: unitreeConfig };
  const configFactory = configs[profileId];
  if (!configFactory) throw new Error(`No workspace generator for ${profileId}.`);
  return { 'main.py': mainFile(scenario), 'trajectories.py': trajectoryModule(scenario), 'robot_config.py': configFactory(), 'workcell.py': workcellFile(scenario) };
}
