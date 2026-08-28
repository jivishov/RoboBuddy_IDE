export const FIDELITY_NOTICE = 'Physical-target Python API with a lightweight articulated browser model. Hardware validation pending.';
export const LEROBOT_REVISION = '7e241bd630a3719a56157a497ce5d08f244784f1';
const d = (values) => Object.freeze(values.map(Number));

export const PROFILES = Object.freeze({
  so101: Object.freeze({
    id:'so101', label:'SO-101 Follower', shortLabel:'SO-101', driver:'LeRobot SO101Follower', transport:'serial',
    limits:Object.freeze({
      'shoulder_pan.pos':d([-110,110]), 'shoulder_lift.pos':d([-100,100]), 'elbow_flex.pos':d([-96.83,96.83]),
      'wrist_flex.pos':d([-95,95]), 'wrist_roll.pos':d([-157.21,162.79]), 'gripper.pos':d([0,100]),
    }),
    rest:Object.freeze({'shoulder_pan.pos':0,'shoulder_lift.pos':-70,'elbow_flex.pos':70,'wrist_flex.pos':55,'wrist_roll.pos':88,'gripper.pos':20}),
    units:Object.freeze({'gripper.pos':'normalized 0–100'}),
    source:'Pinned LeRobot SOFollower public API. Mechanical envelopes follow the SO-101 URDF values used by RoboBuddy; the browser geometry remains a lightweight teaching representation.',
    task:Object.freeze({title:'Visible joint and gripper programming', steps:['Send a complete arm pose','Change shoulder pan and inspect the articulated result','Use a partial gripper-only action','Return to the starter pose'], limitations:'No servo dynamics, calibration transfer, force sensing, payload certification, or hardware validation.'}),
  }),
  openarm: Object.freeze({
    id:'openarm', label:'OpenArm Bimanual', shortLabel:'OpenArm', driver:'LeRobot BiOpenArmFollower', transport:'CAN/CAN-FD',
    limits:Object.freeze(Object.fromEntries([
      ...Object.entries({joint_1:[-75,75],joint_2:[-90,9],joint_3:[-85,85],joint_4:[0,135],joint_5:[-85,85],joint_6:[-40,40],joint_7:[-80,80],gripper:[-65,0]}).map(([k,v])=>[`left_${k}.pos`,d(v)]),
      ...Object.entries({joint_1:[-75,75],joint_2:[-9,90],joint_3:[-85,85],joint_4:[0,135],joint_5:[-85,85],joint_6:[-40,40],joint_7:[-80,80],gripper:[-65,0]}).map(([k,v])=>[`right_${k}.pos`,d(v)]),
    ])),
    rest:Object.freeze({
      'left_joint_1.pos':35,'left_joint_2.pos':-20,'left_joint_3.pos':0,'left_joint_4.pos':90,'left_joint_5.pos':0,'left_joint_6.pos':0,'left_joint_7.pos':0,'left_gripper.pos':-65,
      'right_joint_1.pos':0,'right_joint_2.pos':35,'right_joint_3.pos':0,'right_joint_4.pos':70,'right_joint_5.pos':0,'right_joint_6.pos':0,'right_joint_7.pos':0,'right_gripper.pos':-65,
    }),
    source:'Pinned LeRobot OpenArm follower/bimanual public API and configured left/right joint limits. The standalone 3D model is intentionally simplified and is not a production OpenArm mesh or hardware-calibrated digital twin.',
    task:Object.freeze({title:'Bilateral flask pinch and powered-off hotplate placement', steps:['Approach the flask with the gripper open','Close to first bilateral surface contact','Close slightly farther to satisfy the modeled pinch gate','Lift vertically while preserving the simplified wrist attitude','Transport to the powered-off hotplate','Release only when a support surface is available'], limitations:'Contact is geometry-based. No force/torque sensing, friction coefficient identification, glass compliance, motor dynamics, or hardware safety certification is claimed.'}),
  }),
  lekiwi: Object.freeze({
    id:'lekiwi', label:'LeKiwi Mobile Manipulator', shortLabel:'LeKiwi', driver:'LeRobot LeKiwiClient', transport:'ZMQ on physical deployment',
    limits:Object.freeze({
      'arm_shoulder_pan.pos':d([-110,110]),'arm_shoulder_lift.pos':d([-100,100]),'arm_elbow_flex.pos':d([-96.83,96.83]),
      'arm_wrist_flex.pos':d([-95,95]),'arm_wrist_roll.pos':d([-157.21,162.79]),'arm_gripper.pos':d([0,100]),
      'x.vel':d([-0.6,0.6]),'y.vel':d([-0.6,0.6]),'theta.vel':d([-180,180]),
    }),
    rest:Object.freeze({'arm_shoulder_pan.pos':0,'arm_shoulder_lift.pos':-75,'arm_elbow_flex.pos':70,'arm_wrist_flex.pos':65,'arm_wrist_roll.pos':88,'arm_gripper.pos':20,'x.vel':0,'y.vel':0,'theta.vel':0}),
    units:Object.freeze({'x.vel':'m/s','y.vel':'m/s','theta.vel':'deg/s','arm_gripper.pos':'normalized 0–100'}),
    source:'LeRobot LeKiwiClient public action fields. Browser base motion is kinematic visualization only; ZMQ transport, wheel slip, odometry error, and motor dynamics are not simulated.',
    task:Object.freeze({title:'Arm positioning and bounded base velocity', steps:['Send a stowed arm pose','Pan the arm left and right','Return the arm to stow','Command a bounded forward base velocity','Stop base velocity explicitly'], limitations:'No wheel-contact dynamics, odometry, SLAM, network timing, or hardware validation.'}),
  }),
});

export function validateAction(profileId, action) {
  const profile=PROFILES[profileId];
  if(!profile) throw new Error(`Unknown robot profile: ${profileId}`);
  if(!action || typeof action!=='object' || Array.isArray(action)) throw new Error('send_action() requires a Python dict/object.');
  const entries=Object.entries(action);
  if(!entries.length) throw new Error('send_action() requires at least one action field.');
  for(const [key,raw] of entries){
    const limit=profile.limits[key];
    if(!limit) throw new Error(`Unknown ${profile.shortLabel} action field: ${key}`);
    const value=Number(raw);
    if(!Number.isFinite(value)) throw new Error(`${key} must be a finite number.`);
    if(value<limit[0] || value>limit[1]) throw new Error(`${key}=${value} is outside the configured ${limit[0]}..${limit[1]} ${profile.units?.[key]||'deg'} envelope.`);
  }
  return Object.fromEntries(entries.map(([k,v])=>[k,Number(v)]));
}

const openArmMain=`from robot_config import create_robot\nfrom trajectories import (\n    FLASK_APPROACH, FLASK_CONTACT, FLASK_PINCH,\n    FLASK_LIFT, FLASK_TO_HOTPLATE, FLASK_PLACE, FLASK_RELEASE,\n)\nimport time\n\nrobot = create_robot()\nrobot.connect()\n\ntry:\n    robot.send_action(FLASK_APPROACH)\n    time.sleep(0.30)\n\n    # First modeled bilateral surface contact. The flask is not attached yet.\n    robot.send_action(FLASK_CONTACT)\n    time.sleep(0.15)\n\n    # Slight additional closure is required for the simulator's bounded pinch gate.\n    # No grasp()/attach() helper exists in this physical-target Python.\n    robot.send_action(FLASK_PINCH)\n    time.sleep(0.30)\n\n    for action in FLASK_LIFT:\n        robot.send_action(action)\n        time.sleep(0.05)\n\n    for action in FLASK_TO_HOTPLATE:\n        robot.send_action(action)\n        time.sleep(0.05)\n\n    robot.send_action(FLASK_PLACE)\n    time.sleep(0.20)\n    robot.send_action(FLASK_RELEASE)\n    time.sleep(0.20)\n\n    print(robot.get_observation())\nfinally:\n    robot.disconnect()\n`;

const openArmTrajectories=`# Physical-target action dictionaries for the pinned LeRobot OpenArm API.\n# Values are starter/reference commands for the browser teaching model; they are NOT hardware-validated poses.\n\nRIGHT_HOLD = {\n    "right_joint_1.pos": 0.0, "right_joint_2.pos": 35.0,\n    "right_joint_3.pos": 0.0, "right_joint_4.pos": 70.0,\n    "right_joint_5.pos": 0.0, "right_joint_6.pos": 0.0,\n    "right_joint_7.pos": 0.0, "right_gripper.pos": -65.0,\n}\n\ndef both(left):\n    action = dict(RIGHT_HOLD)\n    action.update(left)\n    return action\n\nFLASK_APPROACH = both({\n    "left_joint_1.pos": 10.0, "left_joint_2.pos": -41.0,\n    "left_joint_3.pos": 0.0, "left_joint_4.pos": 51.75,\n    "left_joint_5.pos": 0.0, "left_joint_6.pos": 0.0,\n    "left_joint_7.pos": 0.0, "left_gripper.pos": -65.0,\n})\n\nFLASK_CONTACT = dict(FLASK_APPROACH)\nFLASK_CONTACT["left_gripper.pos"] = -24.03\n\nFLASK_PINCH = dict(FLASK_CONTACT)\nFLASK_PINCH["left_gripper.pos"] = -23.70\n\n# In the simplified articulated model, j2/j4 are paired to preserve the same\n# approximate terminal-link attitude during vertical lift.\nFLASK_LIFT = [\n    both({"left_joint_1.pos": 10.0, "left_joint_2.pos": -35.0, "left_joint_3.pos": 0.0, "left_joint_4.pos": 63.75, "left_joint_5.pos": 0.0, "left_joint_6.pos": 0.0, "left_joint_7.pos": 0.0, "left_gripper.pos": -23.70}),\n    both({"left_joint_1.pos": 10.0, "left_joint_2.pos": -28.0, "left_joint_3.pos": 0.0, "left_joint_4.pos": 77.75, "left_joint_5.pos": 0.0, "left_joint_6.pos": 0.0, "left_joint_7.pos": 0.0, "left_gripper.pos": -23.70}),\n    both({"left_joint_1.pos": 10.0, "left_joint_2.pos": -20.0, "left_joint_3.pos": 0.0, "left_joint_4.pos": 93.75, "left_joint_5.pos": 0.0, "left_joint_6.pos": 0.0, "left_joint_7.pos": 0.0, "left_gripper.pos": -23.70}),\n]\n\nFLASK_TO_HOTPLATE = [\n    both({"left_joint_1.pos": 2.0, "left_joint_2.pos": -20.0, "left_joint_3.pos": 0.0, "left_joint_4.pos": 93.75, "left_joint_5.pos": 0.0, "left_joint_6.pos": 0.0, "left_joint_7.pos": 0.0, "left_gripper.pos": -23.70}),\n    both({"left_joint_1.pos": -8.0, "left_joint_2.pos": -20.0, "left_joint_3.pos": 0.0, "left_joint_4.pos": 93.75, "left_joint_5.pos": 0.0, "left_joint_6.pos": 0.0, "left_joint_7.pos": 0.0, "left_gripper.pos": -23.70}),\n    both({"left_joint_1.pos": -18.0, "left_joint_2.pos": -20.0, "left_joint_3.pos": 0.0, "left_joint_4.pos": 93.75, "left_joint_5.pos": 0.0, "left_joint_6.pos": 0.0, "left_joint_7.pos": 0.0, "left_gripper.pos": -23.70}),\n]\n\nFLASK_PLACE = both({\n    "left_joint_1.pos": -18.0, "left_joint_2.pos": -24.0,\n    "left_joint_3.pos": 0.0, "left_joint_4.pos": 85.75,\n    "left_joint_5.pos": 0.0, "left_joint_6.pos": 0.0,\n    "left_joint_7.pos": 0.0, "left_gripper.pos": -23.70,\n})\n\nFLASK_RELEASE = dict(FLASK_PLACE)\nFLASK_RELEASE["left_gripper.pos"] = -65.0\n`;

const openArmConfig=`from lerobot.robots.openarm_follower import OpenArmFollowerConfigBase\nfrom lerobot.robots.bi_openarm_follower import BiOpenArmFollower, BiOpenArmFollowerConfig\n\ndef create_robot():\n    config = BiOpenArmFollowerConfig(\n        left_arm_config=OpenArmFollowerConfigBase(port="can0", side="left", cameras={}),\n        right_arm_config=OpenArmFollowerConfigBase(port="can1", side="right", cameras={}),\n    )\n    return BiOpenArmFollower(config)\n`;

const so101Main=`from robot_config import create_robot\nfrom trajectories import HOME, INSPECT_LEFT, INSPECT_RIGHT, GRIPPER_CHECK\nimport time\n\nrobot = create_robot()\nrobot.connect()\ntry:\n    for action in (HOME, INSPECT_LEFT, INSPECT_RIGHT, HOME):\n        robot.send_action(action)\n        time.sleep(0.25)\n\n    # Partial actions are accepted by the pinned physical SOFollower API.\n    for gripper in GRIPPER_CHECK:\n        robot.send_action({"gripper.pos": gripper})\n        time.sleep(0.20)\n\n    print(robot.get_observation())\nfinally:\n    robot.disconnect()\n`;
const so101Traj=`HOME = {\n    "shoulder_pan.pos": 0.0, "shoulder_lift.pos": -70.0,\n    "elbow_flex.pos": 70.0, "wrist_flex.pos": 55.0,\n    "wrist_roll.pos": 88.0, "gripper.pos": 20.0,\n}\nINSPECT_LEFT = dict(HOME)\nINSPECT_LEFT["shoulder_pan.pos"] = -35.0\nINSPECT_RIGHT = dict(HOME)\nINSPECT_RIGHT["shoulder_pan.pos"] = 35.0\nGRIPPER_CHECK = [20.0, 55.0, 85.0, 20.0]\n`;
const so101Config=`from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig\n\ndef create_robot():\n    return SO101Follower(SO101FollowerConfig(port="/dev/ttyACM0", id="robobuddy_so101"))\n`;

const lekiwiMain=`from robot_config import create_robot\nfrom trajectories import STOW, LOOK_LEFT, LOOK_RIGHT, FORWARD, STOP\nimport time\n\nrobot = create_robot()\nrobot.connect()\ntry:\n    robot.send_action(STOW)\n    time.sleep(0.25)\n    robot.send_action(LOOK_LEFT)\n    time.sleep(0.25)\n    robot.send_action(LOOK_RIGHT)\n    time.sleep(0.25)\n    robot.send_action(STOW)\n    time.sleep(0.20)\n\n    robot.send_action(FORWARD)\n    time.sleep(1.20)\n    robot.send_action(STOP)\n    print(robot.get_observation())\nfinally:\n    robot.disconnect()\n`;
const lekiwiTraj=`STOW = {\n    "arm_shoulder_pan.pos": 0.0, "arm_shoulder_lift.pos": -75.0,\n    "arm_elbow_flex.pos": 70.0, "arm_wrist_flex.pos": 65.0,\n    "arm_wrist_roll.pos": 88.0, "arm_gripper.pos": 20.0,\n}\nLOOK_LEFT = dict(STOW)\nLOOK_LEFT["arm_shoulder_pan.pos"] = -30.0\nLOOK_RIGHT = dict(STOW)\nLOOK_RIGHT["arm_shoulder_pan.pos"] = 30.0\nFORWARD = dict(STOW)\nFORWARD.update({"x.vel": 0.18, "y.vel": 0.0, "theta.vel": 0.0})\nSTOP = dict(STOW)\nSTOP.update({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})\n`;
const lekiwiConfig=`from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig\n\ndef create_robot():\n    return LeKiwiClient(LeKiwiClientConfig(remote_ip="192.168.0.10", cameras={}))\n`;

export const STARTER_WORKSPACES=Object.freeze({
  openarm:Object.freeze({'main.py':openArmMain,'trajectories.py':openArmTrajectories,'robot_config.py':openArmConfig,'workcell.py':'# Browser teaching-model dimensions; not hardware measurements.\nWORKCELL = {"task": "flask pinch and powered-off hotplate placement", "table_top_mm": 320.0, "flask_contact_height_mm": 103.731, "hotplate_top_mm": 363.699, "hardware_validation": "pending"}\n'}),
  so101:Object.freeze({'main.py':so101Main,'trajectories.py':so101Traj,'robot_config.py':so101Config,'workcell.py':'WORKCELL = {"task": "joint and gripper positioning inspection", "hardware_validation": "pending"}\n'}),
  lekiwi:Object.freeze({'main.py':lekiwiMain,'trajectories.py':lekiwiTraj,'robot_config.py':lekiwiConfig,'workcell.py':'WORKCELL = {"task": "arm positioning plus bounded base velocity", "hardware_validation": "pending"}\n'}),
});
export function cloneStarter(profileId){return Object.fromEntries(Object.entries(STARTER_WORKSPACES[profileId]).map(([k,v])=>[k,String(v)]));}
