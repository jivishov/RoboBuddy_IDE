export const FIDELITY_NOTICE = 'Physical-target Python API with RoboBuddy_AI canonical robot visual meshes and joint hierarchy. Contact/support remain modeled; hardware validation pending.';
export const LEROBOT_REVISION = '7e241bd630a3719a56157a497ce5d08f244784f1';
export const ROBOBUDDY_AI_VISUAL_REVISION = '66d18a029a0caeb6a6075e681dbd9ecd6b22affa';
const d = (values) => Object.freeze(values.map(Number));
const unitreeG1JointLimits = Object.freeze({
  left_hip_pitch_joint:d([-144.9984,165.0004]), left_hip_roll_joint:d([-30.0001,170.0023]), left_hip_yaw_joint:d([-157.9988,157.9988]),
  left_knee_joint:d([-5,165.0004]), left_ankle_pitch_joint:d([-50.0003,30.0001]), left_ankle_roll_joint:d([-15,15]),
  right_hip_pitch_joint:d([-144.9984,165.0004]), right_hip_roll_joint:d([-170.0023,30.0001]), right_hip_yaw_joint:d([-157.9988,157.9988]),
  right_knee_joint:d([-5,165.0004]), right_ankle_pitch_joint:d([-50.0003,30.0001]), right_ankle_roll_joint:d([-15,15]),
  waist_yaw_joint:d([-150.0004,150.0004]), waist_roll_joint:d([-29.7938,29.7938]), waist_pitch_joint:d([-29.7938,29.7938]),
  left_shoulder_pitch_joint:d([-176.9981,153.0026]), left_shoulder_roll_joint:d([-90.9972,129.0014]), left_shoulder_yaw_joint:d([-150.0004,150.0004]),
  left_elbow_joint:d([-60.0001,120.0003]), left_wrist_roll_joint:d([-112.9999,112.9999]), left_wrist_pitch_joint:d([-92.5,92.5]), left_wrist_yaw_joint:d([-92.5,92.5]),
  right_shoulder_pitch_joint:d([-176.9981,153.0026]), right_shoulder_roll_joint:d([-129.0014,90.9972]), right_shoulder_yaw_joint:d([-150.0004,150.0004]),
  right_elbow_joint:d([-60.0001,120.0003]), right_wrist_roll_joint:d([-112.9999,112.9999]), right_wrist_pitch_joint:d([-92.5,92.5]), right_wrist_yaw_joint:d([-92.5,92.5]),
});
const unitreeG1Rest = Object.freeze(Object.fromEntries(Object.keys(unitreeG1JointLimits).map((key) => [key, 0])));

export const PROFILES = Object.freeze({
  so101: Object.freeze({
    id:'so101', label:'SO-101 Follower', shortLabel:'SO-101', driver:'LeRobot SO101Follower', transport:'serial',
    visual:Object.freeze({robotId:'so101_follower', repository:'jivishov/RoboBuddy_AI', revision:ROBOBUDDY_AI_VISUAL_REVISION, model:'official SO-101 URDF baked by RoboBuddy_AI'}),
    limits:Object.freeze({
      'shoulder_pan.pos':d([-110,110]), 'shoulder_lift.pos':d([-100,100]), 'elbow_flex.pos':d([-96.83,96.83]),
      'wrist_flex.pos':d([-95,95]), 'wrist_roll.pos':d([-157.21,162.79]), 'gripper.pos':d([0,100]),
    }),
    rest:Object.freeze({'shoulder_pan.pos':0,'shoulder_lift.pos':-70,'elbow_flex.pos':70,'wrist_flex.pos':55,'wrist_roll.pos':88,'gripper.pos':20}),
    units:Object.freeze({'gripper.pos':'normalized 0–100'}),
    source:'Pinned LeRobot SOFollower public API plus the same generated SO-101 visual mesh and joint hierarchy used by RoboBuddy_AI. Servo dynamics, device calibration transfer, contact force and hardware behavior are not inferred from the mesh.',
    task:Object.freeze({title:'Visible joint and gripper programming', steps:['Send a complete arm pose','Change shoulder pan and inspect the canonical articulated model','Use a partial gripper-only action','Return to the starter pose'], limitations:'Canonical RoboBuddy visual geometry is used. No servo dynamics, calibration transfer, force sensing, payload certification, or hardware validation.'}),
  }),
  openarm: Object.freeze({
    id:'openarm', label:'OpenArm V2 Bimanual', shortLabel:'OpenArm', driver:'LeRobot BiOpenArmFollower', transport:'CAN/CAN-FD',
    visual:Object.freeze({robotId:'openarm_v2_bimanual', repository:'jivishov/RoboBuddy_AI', revision:ROBOBUDDY_AI_VISUAL_REVISION, modelRevision:'6c7b720f1ba48e8bafa3a3dc752c45f397b42221'}),
    limits:Object.freeze(Object.fromEntries([
      ...Object.entries({joint_1:[-75,75],joint_2:[-90,9],joint_3:[-85,85],joint_4:[0,135],joint_5:[-85,85],joint_6:[-40,40],joint_7:[-80,80],gripper:[-65,0]}).map(([k,v])=>[`left_${k}.pos`,d(v)]),
      ...Object.entries({joint_1:[-75,75],joint_2:[-9,90],joint_3:[-85,85],joint_4:[0,135],joint_5:[-85,85],joint_6:[-40,40],joint_7:[-80,80],gripper:[-65,0]}).map(([k,v])=>[`right_${k}.pos`,d(v)]),
    ])),
    rest:Object.freeze({
      'left_joint_1.pos':35,'left_joint_2.pos':-20,'left_joint_3.pos':0,'left_joint_4.pos':90,'left_joint_5.pos':0,'left_joint_6.pos':0,'left_joint_7.pos':0,'left_gripper.pos':-65,
      'right_joint_1.pos':5,'right_joint_2.pos':40,'right_joint_3.pos':0,'right_joint_4.pos':120,'right_joint_5.pos':0,'right_joint_6.pos':5,'right_joint_7.pos':0,'right_gripper.pos':-65,
    }),
    source:'Pinned LeRobot OpenArm follower/bimanual public API plus RoboBuddy_AI openarm_v2_bimanual generated visual meshes, source revision 6c7b720f1ba48e8bafa3a3dc752c45f397b42221. Browser contact/support tests are modeled geometry, not hardware validation.',
    task:Object.freeze({title:'Bilateral flask pinch and powered-off hotplate placement', steps:['Reach the source-authored flask contact pose with the gripper open','Close the public left_gripper.pos command only after modeled bilateral finger contact','Lift using source-authored OpenArm joint targets','Translate to the hotplate region','Descend to a source-authored placement waypoint','Open the gripper only after modeled support is available'], limitations:'Canonical OpenArm visual geometry is used. Contact is a modeled rendered-finger envelope; no force/torque sensing, friction identification, glass compliance, motor dynamics, or hardware safety certification is claimed.'}),
  }),
  lekiwi: Object.freeze({
    id:'lekiwi', label:'LeKiwi Mobile Manipulator', shortLabel:'LeKiwi', driver:'LeRobot LeKiwiClient', transport:'ZMQ on physical deployment',
    visual:Object.freeze({robotId:'lekiwi_sim', repository:'jivishov/RoboBuddy_AI', revision:ROBOBUDDY_AI_VISUAL_REVISION, modelRevision:'efa608d7ee5a495a4803b1d28cd0c955b4f1e033'}),
    limits:Object.freeze({
      'arm_shoulder_pan.pos':d([-110,110]),'arm_shoulder_lift.pos':d([-100,100]),'arm_elbow_flex.pos':d([-96.83,96.83]),
      'arm_wrist_flex.pos':d([-95,95]),'arm_wrist_roll.pos':d([-157.21,162.79]),'arm_gripper.pos':d([0,100]),
      'x.vel':d([-0.6,0.6]),'y.vel':d([-0.6,0.6]),'theta.vel':d([-180,180]),
    }),
    rest:Object.freeze({'arm_shoulder_pan.pos':0,'arm_shoulder_lift.pos':-75,'arm_elbow_flex.pos':70,'arm_wrist_flex.pos':65,'arm_wrist_roll.pos':88,'arm_gripper.pos':20,'x.vel':0,'y.vel':0,'theta.vel':0}),
    units:Object.freeze({'x.vel':'m/s','y.vel':'m/s','theta.vel':'deg/s','arm_gripper.pos':'normalized 0–100'}),
    source:'LeRobot LeKiwiClient public action fields plus the same generated LeKiwi visual mesh and arm hierarchy used by RoboBuddy_AI, source revision efa608d7ee5a495a4803b1d28cd0c955b4f1e033. Browser base motion remains kinematic.',
    task:Object.freeze({title:'Arm positioning and bounded base velocity', steps:['Send a stowed arm pose','Pan the arm left and right on the canonical LeKiwi model','Return the arm to stow','Command a bounded forward base velocity','Stop base velocity explicitly'], limitations:'Canonical RoboBuddy visual geometry is used. No wheel-contact dynamics, odometry, SLAM, network timing, or hardware validation.'}),
  }),
  unitree: Object.freeze({
    id:'unitree', label:'Unitree G1 29-DoF', shortLabel:'Unitree G1', driver:'RoboBuddy G1 pose rig (kinematic only)', transport:'none — browser-only pose workspace', simulationMode:'kinematic_pose',
    visual:Object.freeze({robotId:'unitree_g1_29dof', repository:'jivishov/RoboBuddy_AI', revision:ROBOBUDDY_AI_VISUAL_REVISION, modelRevision:'dd4fa6866e523ad61324f658d63736e4eda3a6e4', modelRepository:'unitreerobotics/unitree_ros', modelPath:'robots/g1_description/g1_29dof.urdf', license:'BSD-3-Clause'}),
    limits:unitreeG1JointLimits,
    rest:unitreeG1Rest,
    source:'RoboBuddy_AI canonical Unitree G1 mesh with 29 source-manifest joint envelopes, generated from unitreerobotics/unitree_ros at dd4fa6866e523ad61324f658d63736e4eda3a6e4. This is a browser-only visual pose workspace, not a Unitree SDK or hardware-control API.',
    task:Object.freeze({title:'29-axis kinematic pose inspection', steps:['Inspect the neutral source-mesh pose','Send a bounded upper-body joint pose','Inspect a lower-body joint pose without moving the root','Return joints to neutral'], limitations:'The canonical G1 visual mesh and source joint ranges are used. Dynamic balance, walking, root translation, foot contact, collision, hand actuation, grasping, force/torque control, Unitree SDK control, and hardware validation are not simulated.'}),
  }),
});

export function fidelityNoticeFor(profileId) {
  const profile = PROFILES[profileId];
  if (profile?.simulationMode === 'kinematic_pose') {
    return 'Reference-sourced Unitree G1 mesh and bounded joint-pose visualization. No fixed-step contact plant, balance, locomotion, collision, or hardware validation is active.';
  }
  return FIDELITY_NOTICE;
}

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

const openArmMain=`from robot_config import create_robot\nfrom trajectories import (\n    FLASK_CONTACT, FLASK_PINCH, FLASK_LIFT,\n    FLASK_TRANSFER, FLASK_PLACE, FLASK_RELEASE, LEFT_RETREAT,\n)\nimport time\n\nrobot = create_robot()\nrobot.connect()\n\ntry:\n    # Every movement below is an ordinary physical OpenArm send_action() command.\n    # The browser creates no grasp/attach/move_to command.\n    robot.send_action(FLASK_CONTACT)\n    time.sleep(0.30)\n\n    # Close only after the canonical rendered finger geometry reaches the flask.\n    robot.send_action(FLASK_PINCH)\n    time.sleep(0.30)\n\n    robot.send_action(FLASK_LIFT)\n    time.sleep(0.30)\n    robot.send_action(FLASK_TRANSFER)\n    time.sleep(0.30)\n    robot.send_action(FLASK_PLACE)\n    time.sleep(0.25)\n    robot.send_action(FLASK_RELEASE)\n    time.sleep(0.20)\n    robot.send_action(LEFT_RETREAT)\n\n    print(robot.get_observation())\nfinally:\n    robot.disconnect()\n`;

const openArmTrajectories=`# Selected physical-target OpenArm commands copied from the RoboBuddy_AI\n# OpenArm V2 reference mission. They use the public LeRobot BiOpenArmFollower fields.\n# This is source/reference calibration, NOT validation on a physical OpenArm.\n\nRIGHT_HOLD = {\n    "right_joint_1.pos": 5.0, "right_joint_2.pos": 40.0,\n    "right_joint_3.pos": 0.0, "right_joint_4.pos": 120.0,\n    "right_joint_5.pos": 0.0, "right_joint_6.pos": 5.0,\n    "right_joint_7.pos": 0.0, "right_gripper.pos": -65.0,\n}\n\ndef both(left):\n    action = dict(RIGHT_HOLD)\n    action.update(left)\n    return action\n\nFLASK_CONTACT = both({\n    "left_joint_1.pos": 20.880802, "left_joint_2.pos": -37.132083,\n    "left_joint_3.pos": -22.542829, "left_joint_4.pos": 95.302776,\n    "left_joint_5.pos": -44.624521, "left_joint_6.pos": 15.591245,\n    "left_joint_7.pos": -14.957229, "left_gripper.pos": -65.0,\n})\n\nFLASK_PINCH = dict(FLASK_CONTACT)\nFLASK_PINCH["left_gripper.pos"] = 0.0\n\nFLASK_LIFT = both({\n    "left_joint_1.pos": 21.755513, "left_joint_2.pos": -50.032074,\n    "left_joint_3.pos": -39.896139, "left_joint_4.pos": 109.289863,\n    "left_joint_5.pos": -60.031388, "left_joint_6.pos": -7.790411,\n    "left_joint_7.pos": -11.718561, "left_gripper.pos": 0.0,\n})\n\nFLASK_TRANSFER = both({\n    "left_joint_1.pos": -30.984082, "left_joint_2.pos": -51.356900,\n    "left_joint_3.pos": 8.588629, "left_joint_4.pos": 79.098555,\n    "left_joint_5.pos": -51.369924, "left_joint_6.pos": -3.994894,\n    "left_joint_7.pos": -2.596416, "left_gripper.pos": 0.0,\n})\n\n# Source-authored descent waypoint reused as the powered-off-hotplate contact height.\nFLASK_PLACE = both({\n    "left_joint_1.pos": -23.619920, "left_joint_2.pos": -45.380579,\n    "left_joint_3.pos": 10.665934, "left_joint_4.pos": 72.620538,\n    "left_joint_5.pos": -46.151799, "left_joint_6.pos": 9.550320,\n    "left_joint_7.pos": -1.593155, "left_gripper.pos": 0.0,\n})\n\nFLASK_RELEASE = dict(FLASK_PLACE)\nFLASK_RELEASE["left_gripper.pos"] = -65.0\n\nLEFT_RETREAT = both({\n    "left_joint_1.pos": -31.598360, "left_joint_2.pos": -44.635073,\n    "left_joint_3.pos": 9.199085, "left_joint_4.pos": 69.103824,\n    "left_joint_5.pos": -47.628854, "left_joint_6.pos": 18.416264,\n    "left_joint_7.pos": -8.832295, "left_gripper.pos": -65.0,\n})\n`;

const openArmConfig=`from lerobot.robots.openarm_follower import OpenArmFollowerConfigBase\nfrom lerobot.robots.bi_openarm_follower import BiOpenArmFollower, BiOpenArmFollowerConfig\n\ndef create_robot():\n    config = BiOpenArmFollowerConfig(\n        left_arm_config=OpenArmFollowerConfigBase(port="can0", side="left", cameras={}),\n        right_arm_config=OpenArmFollowerConfigBase(port="can1", side="right", cameras={}),\n    )\n    return BiOpenArmFollower(config)\n`;

const so101Main=`from robot_config import create_robot\nfrom trajectories import HOME, INSPECT_LEFT, INSPECT_RIGHT, GRIPPER_CHECK\nimport time\n\nrobot = create_robot()\nrobot.connect()\ntry:\n    for action in (HOME, INSPECT_LEFT, INSPECT_RIGHT, HOME):\n        robot.send_action(action)\n        time.sleep(0.25)\n\n    # Partial actions are accepted by the pinned physical SOFollower API.\n    for gripper in GRIPPER_CHECK:\n        robot.send_action({"gripper.pos": gripper})\n        time.sleep(0.20)\n\n    print(robot.get_observation())\nfinally:\n    robot.disconnect()\n`;
const so101Traj=`HOME = {\n    "shoulder_pan.pos": 0.0, "shoulder_lift.pos": -70.0,\n    "elbow_flex.pos": 70.0, "wrist_flex.pos": 55.0,\n    "wrist_roll.pos": 88.0, "gripper.pos": 20.0,\n}\nINSPECT_LEFT = dict(HOME)\nINSPECT_LEFT["shoulder_pan.pos"] = -35.0\nINSPECT_RIGHT = dict(HOME)\nINSPECT_RIGHT["shoulder_pan.pos"] = 35.0\nGRIPPER_CHECK = [20.0, 55.0, 85.0, 20.0]\n`;
const so101Config=`from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig\n\ndef create_robot():\n    return SO101Follower(SO101FollowerConfig(port="/dev/ttyACM0", id="robobuddy_so101"))\n`;

const lekiwiMain=`from robot_config import create_robot\nfrom trajectories import STOW, LOOK_LEFT, LOOK_RIGHT, FORWARD, STOP\nimport time\n\nrobot = create_robot()\nrobot.connect()\ntry:\n    robot.send_action(STOW)\n    time.sleep(0.25)\n    robot.send_action(LOOK_LEFT)\n    time.sleep(0.25)\n    robot.send_action(LOOK_RIGHT)\n    time.sleep(0.25)\n    robot.send_action(STOW)\n    time.sleep(0.20)\n\n    robot.send_action(FORWARD)\n    time.sleep(1.20)\n    robot.send_action(STOP)\n    print(robot.get_observation())\nfinally:\n    robot.disconnect()\n`;
const lekiwiTraj=`STOW = {\n    "arm_shoulder_pan.pos": 0.0, "arm_shoulder_lift.pos": -75.0,\n    "arm_elbow_flex.pos": 70.0, "arm_wrist_flex.pos": 65.0,\n    "arm_wrist_roll.pos": 88.0, "arm_gripper.pos": 20.0,\n}\nLOOK_LEFT = dict(STOW)\nLOOK_LEFT["arm_shoulder_pan.pos"] = -30.0\nLOOK_RIGHT = dict(STOW)\nLOOK_RIGHT["arm_shoulder_pan.pos"] = 30.0\nFORWARD = dict(STOW)\nFORWARD.update({"x.vel": 0.18, "y.vel": 0.0, "theta.vel": 0.0})\nSTOP = dict(STOW)\nSTOP.update({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})\n`;
const lekiwiConfig=`from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig\n\ndef create_robot():\n    return LeKiwiClient(LeKiwiClientConfig(remote_ip="192.168.0.10", cameras={}))\n`;

export const STARTER_WORKSPACES=Object.freeze({
  openarm:Object.freeze({'main.py':openArmMain,'trajectories.py':openArmTrajectories,'robot_config.py':openArmConfig,'workcell.py':'# Reference-calibrated browser workcell; dimensions are simulation configuration, not hardware measurements.\nWORKCELL = {"task": "flask pinch and powered-off hotplate placement", "worktop_top_mm": 320.0, "hotplate_height_mm": 46.329, "hotplate_center_mm": [439.626, 343.1645, -339.239], "visual_source": "RoboBuddy_AI@66d18a029a0caeb6a6075e681dbd9ecd6b22affa", "hardware_validation": "pending"}\n'}),
  so101:Object.freeze({'main.py':so101Main,'trajectories.py':so101Traj,'robot_config.py':so101Config,'workcell.py':'WORKCELL = {"task": "joint and gripper positioning inspection", "visual_source": "RoboBuddy_AI@66d18a029a0caeb6a6075e681dbd9ecd6b22affa", "hardware_validation": "pending"}\n'}),
  lekiwi:Object.freeze({'main.py':lekiwiMain,'trajectories.py':lekiwiTraj,'robot_config.py':lekiwiConfig,'workcell.py':'WORKCELL = {"task": "arm positioning plus bounded base velocity", "visual_source": "RoboBuddy_AI@66d18a029a0caeb6a6075e681dbd9ecd6b22affa", "hardware_validation": "pending"}\n'}),
});
export function cloneStarter(profileId){return Object.fromEntries(Object.entries(STARTER_WORKSPACES[profileId]).map(([k,v])=>[k,String(v)]));}
