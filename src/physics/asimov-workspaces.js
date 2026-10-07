import { ASIMOV_SCENES } from './asimov-scene.js';
import { ASIMOV_ALL_PACKAGES } from './asimov-model-package.js';
import { ASIMOV_SOURCE } from './asimov-generated.js';
const isStanding=v=>v==='standing'||v.startsWith('sensor-standing');
const hasWholeBody=v=>v!=='drop'&&ASIMOV_ALL_PACKAGES[v]?.rootMode==='free-base';
const REFERENCE_VARIANTS=Object.freeze(['mounted','freebase','drop']);
const USER_VARIANTS=Object.freeze(['actuator-mounted','actuator-freebase','standing','sensor-standing','sensor-standing-delay','sensor-standing-ankle-stress']);
const labels={'sensor-standing':'Asimov 1 — Sensor Standing (experimental)','sensor-standing-delay':'Asimov 1 — Delayed Sensor Standing (sensitivity)','sensor-standing-ankle-stress':'Asimov 1 — Ankle Loss Standing (sensitivity)',mounted:'Asimov 1 — Mounted Joint Lab',freebase:'Asimov 1 — Free-base Dynamics',drop:'Asimov 1 — Passive Gravity Drop','actuator-mounted':'Asimov 1 — Actuator Lab (experimental)','actuator-freebase':'Asimov 1 — Whole-Body Dynamics (experimental)',standing:'Asimov 1 — Standing Trial (experimental)'};
export const ASIMOV_WORKSPACES=Object.freeze(Object.fromEntries(Object.entries(ASIMOV_SCENES).map(([variant,s])=>[s.id,Object.freeze({
  schema:'robobuddy.physical-workspace.v1',schemaVersion:1,simulationMode:'physical_mujoco',profileId:'asimov',id:s.id,title:labels[variant],
  catalogVisibility:REFERENCE_VARIANTS.includes(variant)?'internal-reference':'user',
  brief:variant.startsWith('sensor-standing')?'Stand using only synthetic body-frame balance sensors; local joint PD stays ideal. Separately declared feedback delay or ankle loss can cause failure. No fitted hardware claim.':variant==='standing'?'Test a bounded torso-feedback controller for flat-floor stance. The starter requests 12 simulated seconds with a 120-second wall deadline. Read the continuous support assessment; no validated walking or hardware claim.':variant==='actuator-freebase'?'Primary free-base workspace for agent-generated whole-body trajectories. Continuous torque limits, estimated speed/friction/delay behavior and MuJoCo contacts determine the measured result; stepping or walking intent is not treated as gait success.':variant==='actuator-mounted'?'Fixed-pelvis actuator laboratory with continuous torque limits, estimated speed derating, smooth friction, command delay and a separate synthetic sensor view. Use it for isolated joint/control experiments, not standing evidence.':variant==='mounted'?'Internal reference fixture: the pelvis is explicitly fixed to a visible mount for source/reference joint-dynamics validation. It is retained for provenance and regression testing, not listed in the normal task menu.':
    variant==='drop'?'Internal negative-control fixture: an elevated passive free-base robot falls under gravity and collides with the floor. It is retained for gravity/contact validation, not listed in the normal task menu.':
    'Internal reference fixture: a free-base Asimov under real gravity, source contacts and estimated joint-space PD. It is retained as the source-derived baseline for experimental free-base models and regression testing, not listed in the normal task menu.',
  robotId:s.robotId,workspaceRevision:s.revision,physicalSceneId:s.id,physicalSceneRevision:s.revision,modelPackage:s.modelPackage,modelId:ASIMOV_ALL_PACKAGES[variant].modelId,
  executionBudget:Object.freeze({pythonWallTimeMs:isStanding(variant)?120000:30000}),
  physicalApi:{version:'robobuddy.sim.v1',angleUnit:'rad',timeUnit:'s',lengthUnit:'m',torqueUnit:'N*m'},
  canonicalModel:{repository:'menloresearch/asimov-1',revision:ASIMOV_SOURCE.revision,sourceMjcfSha256:ASIMOV_SOURCE.sourceXmlSha256,authority:'Observed MuJoCo world-body transforms only'},
  controller:{physicsTimestepSeconds:s.physics.timestepSeconds,controlIntervalSeconds:.005,claim:'Repository-estimated controller, not hardware calibration'},
  capabilities:{standing:isStanding(variant)?'experimental-flat-floor-trial':'unsupported',actuatorProfile:ASIMOV_ALL_PACKAGES[variant].actuatorProfileId??'reference-ideal',sensorView:ASIMOV_ALL_PACKAGES[variant].sensorProfileId??'ground_truth',jointTargets:'supported',gravity:'supported',contacts:'source-primitives',wholeBodyMotion:hasWholeBody(variant)?'experimental-agent-generated-keyframes':'unsupported',walking:hasWholeBody(variant)?'experimental-agent-generated-trajectory-not-validated-gait':'unsupported',grasping:'unsupported',balanceRecovery:'unsupported',neck:'fixed in source'},
  taskEvaluation:{type:isStanding(variant)?'bounded-standing-trial':'observation-only',syntheticSuccessEvents:false},limitations:ASIMOV_ALL_PACKAGES[variant].limitations,
})])));
// The normal catalog exposes only the six user-facing experiment/sensitivity workspaces.
// Mounted/free-base/drop remain fully registered internal reference fixtures because the
// experimental packages and regression evidence are derived from or checked against them.
export const ASIMOV_TASKS=Object.freeze(USER_VARIANTS.map(v=>ASIMOV_WORKSPACES[`asimov-${v}`]));
export function asimovWorkspaceFiles(scenario) {
  const passive=scenario.id==='asimov-drop';
  const standing=scenario.id==='asimov-standing'||scenario.id.startsWith('asimov-sensor-standing');
  const controllerId=scenario.id.startsWith('asimov-sensor-standing')?'asimov-sensor-stance-v2':'asimov-stance-feedback-v1';
  const experiment=standing||scenario.id.startsWith('asimov-actuator-');
  const main=[
    '# Simulation only: no serial, CAN, DDS or physical robot is connected.',
    '# All joint targets use radians. Requested targets are not measured positions.',
    'from robobuddy.sim import connect','from robot_config import ROBOT_ID','',
    'robot = await connect(ROBOT_ID)','try:',
    ...(standing?[`    await robot.stand(controller_id="${controllerId}", max_steps=8000)`]:passive?[]:['    await robot.set_joint_targets({"left_elbow_joint": 1.0, "right_elbow_joint": -1.0})']),
    // High-rate feedback/evaluation stays in the physics worker. Python polls coarsely
    // to avoid turning 120 UI round trips into a wall-clock timeout on software rendering.
    `    for _ in range(${standing?12:10}):`,`        await robot.wait_sim(${standing?'1.0':'0.1'})`,'        state = await robot.get_state()',
    '        print("t", state["simulation_time_s"], "root", state["root"]["position_m"],',
    '              "elbow", state["joints"]["left_elbow_joint"]["position_rad"])',
    ...(experiment?['    sensors = await robot.get_observation(view="hardware_like")','    print("Sensor profile", sensors["profileId"], "sample", sensors["joints"]["left_elbow_joint"])','    print("Actuator model", state["actuator_model"]["id"])']:[]),
    ...(standing?['    print("Standing assessment", state["standing_assessment"])']:[]),
    'finally:','    await robot.disconnect()','',
  ].join('\n');
  return {'main.py':main,'robot_config.py':`ROBOT_ID = ${JSON.stringify(scenario.robotId)}\n# 23 source hinges; neck fixed. Hardware model parameters are not calibrated.\n`,
    'trajectories.py':'# Edit main.py to command source-named joints in radians. Whole-body agent trajectories are exposed through bounded WebMCP; no trained gait or grasp operation is supplied.\n',
    'workcell.py':`# Declared physical scene: ${scenario.physicalSceneId}\n# ${scenario.brief}\n`};
}
