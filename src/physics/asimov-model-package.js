import { ASIMOV_SENSITIVITY_PROFILES, ASIMOV_BODY_SENSOR_PROFILE, ASIMOV_SENSOR_STANDING_CONTROLLER } from './asimov-sensitivity.js';
import { ASIMOV_ACTUATOR_MODELS } from './asimov-actuator-generated.js';
import { ASIMOV_ACTUATOR_PROFILE, ASIMOV_SENSOR_PROFILE } from './asimov-actuator-profile.js';
import { ASIMOV_STANDING_CONTROLLER } from './asimov-standing.js';
import { registerModelPackage } from './model-registry.js';
import { ASIMOV_SOURCE } from './asimov-generated.js';
import { ASIMOV_CONTROLLERS, ASIMOV_FOOT_CONTACT_GEOMS } from './asimov-controller.js';
export const ASIMOV_ROBOT_ID='asimov_1_23dof_physical';
export const ASIMOV_LIMITATIONS=Object.freeze([
  'Pinned Menlo MJCF has 23 hinges and two fixed neck bodies, not the advertised 25 actuated hardware joints.',
  'Source link inertias, 32.224913 kg model mass, joint axes, reference offsets, collision primitives, foot contacts, exclusions and solver settings are retained.',
  'Ideal torque motors use URDF effort limits. Joint-space PD gains are repository estimates, not a Menlo trained policy or measured actuator dynamics.',
  'Ankles are source ideal pitch/roll joints, not a simulation of the hardware parallel transmission. No finger actuators, grasping, trained/validated walking controller, general balance recovery or hardware fidelity is claimed.',
  'Free-base posture hold or agent-generated whole-body trajectories may fall. Mounted tests explicitly fix the pelvis and cannot establish standing or walking capability.',
]);
const pkg = (variant,rootMode,initialCommand) => registerModelPackage({
  id:`asimov-1-${variant}-732cc60-v1`,modelId:`robobuddy-asimov-${variant}-v1`,robotId:ASIMOV_ROBOT_ID,
  asset:`models/asimov/${variant}.xml`,sha256:ASIMOV_SOURCE.modelHashes[variant],
  source:{url:`https://github.com/menloresearch/asimov-1/blob/${ASIMOV_SOURCE.revision}/sim-model/xmls/asimov_1.xml`,revision:ASIMOV_SOURCE.revision},
  license:'Menlo source: CERN-OHL-S-2.0 hardware and GPL-2.0 software; see models/asimov/source licenses and THIRD_PARTY_NOTICES.md',
  physics:{timestepSeconds:.005,integrator:'implicitfast',iterations:10,lsIterations:20},
  controllers:[ASIMOV_CONTROLLERS.LOWLEVEL,ASIMOV_CONTROLLERS.JOINT_HOLD],
  joints:ASIMOV_SOURCE.joints.map(({referenceRad,armature,...j})=>({...j,evidence:'source-derived'})),
  actuators:ASIMOV_SOURCE.joints.map(j=>({id:j.id,jointId:j.id,controllerId:ASIMOV_CONTROLLERS.LOWLEVEL,command:'torque-nm',
    controlRangeNm:[-j.effortLimitNm,j.effortLimitNm],forceRangeNm:[-j.effortLimitNm,j.effortLimitNm],evidence:'estimated'})),
  bodies:ASIMOV_SOURCE.bodies.map(id=>id==='pelvis_link' && rootMode==='free-base'?{id,freeJointId:'floating_base'}:{id}),
  rootMode,initialCommand,footContactGeoms:ASIMOV_FOOT_CONTACT_GEOMS,
  sceneConstraints:{fixtures:variant==='mounted'?['floor','asimov_declared_mount']:['floor'],objects:[]},
  evidence:{geometryAndInertia:'source-derived',effortAndVelocityLimits:'source-derived',idealActuation:'estimated',pdGains:'estimated',hardwareAlignment:'calibration-required'},
  limitations:ASIMOV_LIMITATIONS,
});
export const ASIMOV_FREEBASE_PACKAGE=pkg('freebase','free-base','joint-hold');
export const ASIMOV_MOUNTED_PACKAGE=pkg('mounted','fixed-mounted','joint-hold');
export const ASIMOV_DROP_PACKAGE=pkg('drop','free-base','passive');
export const ASIMOV_PACKAGES=Object.freeze({freebase:ASIMOV_FREEBASE_PACKAGE,mounted:ASIMOV_MOUNTED_PACKAGE,drop:ASIMOV_DROP_PACKAGE});

export const ASIMOV_ACTUATOR_LIMITATIONS=Object.freeze([
  'Spec-informed experimental profile; not a calibrated digital twin. Original source geometry, mass, joint limits and reflected inertia are unchanged.',
  'Nineteen single-axis motors use the lower of source effort limit and published continuous rating. Peak output is disabled without duty-cycle evidence.',
  'Linear motoring-speed derating, smoothed friction, 5 ms command delay and sensor stress are explicit estimates, not identified full-body hardware responses.',
  'The four ankle axes retain the source equivalent joint-space plant. A/B motor ratings and linkage ratios are unresolved; no guessed transmission or extra rotor inertia is enabled.',
  'Experimental standing uses bounded ankle torques with ground-truth torso feedback on a flat floor. It may fail; it is neither a trained walking controller nor general balance recovery.',
  'Agent-generated full-body WebMCP trajectories can be tested in actuated free-base scenes; successful execution does not establish a stable gait or hardware-feasible locomotion.',
  'Neck remains fixed; no actuated fingers, hardware connection or hardware validation.',
]);
const experiment=(key,sourceVariant,standing=false)=>{
  const base=ASIMOV_PACKAGES[sourceVariant], generated=ASIMOV_ACTUATOR_MODELS[sourceVariant];
  return registerModelPackage({...structuredClone(base),id:`asimov-1-${key}-v1`,modelId:`robobuddy-asimov-actuator-${sourceVariant}-v1`,
    asset:generated.asset,sha256:generated.sha256,physics:{...base.physics,timestepSeconds:generated.timestepSeconds},
    actuatorProfileId:ASIMOV_ACTUATOR_PROFILE.id,sensorProfileId:ASIMOV_SENSOR_PROFILE.id,
    ...(standing?{standingControllerId:ASIMOV_STANDING_CONTROLLER.id}:{}),
    controllers:[...base.controllers,...(standing?[ASIMOV_STANDING_CONTROLLER.id]:[])],
    evidence:{...base.evidence,actuatorEnvelope:'estimated',sensorModel:'estimated',standing:'estimated'},
    limitations:ASIMOV_ACTUATOR_LIMITATIONS});
};
export const ASIMOV_ACTUATOR_PACKAGES=Object.freeze({
  'actuator-mounted':experiment('actuator-mounted','mounted'),
  'actuator-freebase':experiment('actuator-freebase','freebase'),
  standing:experiment('standing','freebase',true),
});
export const ASIMOV_SENSOR_PACKAGES=Object.freeze(Object.fromEntries(Object.keys(ASIMOV_SENSITIVITY_PROFILES).map(key=>[key,registerModelPackage({
  ...structuredClone(ASIMOV_ACTUATOR_PACKAGES.standing),id:`asimov-1-${key}-v2`,
  sensorProfileId:ASIMOV_BODY_SENSOR_PROFILE.id,standingControllerId:ASIMOV_SENSOR_STANDING_CONTROLLER.id,sensitivityProfileId:key,
  controllers:[...ASIMOV_PACKAGES.freebase.controllers,ASIMOV_SENSOR_STANDING_CONTROLLER.id],
  limitations:[...ASIMOV_ACTUATOR_LIMITATIONS.filter(s=>!s.startsWith('Experimental standing uses')),
    'Balance uses synthetic body-frame gyro/projected gravity only, with declared delay and filtering. Inner joint PD remains ideal local feedback.',
    'Ankle stress is hypothetical independent-axis loss, not physical A/B transmission calibration. Sensor and mass/contact sweeps are not confidence intervals.',
    'Pinned model mass is 32.224913 kg versus published nominal 35 kg; assembly configuration remains unreconciled.'],
})])));
export const ASIMOV_ALL_PACKAGES=Object.freeze({...ASIMOV_PACKAGES,...ASIMOV_ACTUATOR_PACKAGES,...ASIMOV_SENSOR_PACKAGES});
