import { ASIMOV_SOURCE } from './asimov-generated.js';
// Repository-authored ideal joint-space PD, not a Menlo policy or motor calibration.
// In particular, the source ankle coordinates are NOT the physical parallel-drive motors.
export const ASIMOV_JOINT_ORDER = Object.freeze(ASIMOV_SOURCE.joints.map(j => j.id));
export const ASIMOV_PHYSICS_TIMESTEP_SECONDS = .005;
export const ASIMOV_LOWLEVEL_CONTROL_INTERVAL_SECONDS = .005;
export const ASIMOV_CONTROLLERS = Object.freeze({LOWLEVEL:'asimov_ideal_torque_v1',JOINT_HOLD:'asimov_estimated_pd_v1'});
export const ASIMOV_MAX_KP = 300;
export const ASIMOV_MAX_KD = 20;
export const ASIMOV_JOINT_HOLD_PROFILE = Object.freeze({
  id: ASIMOV_CONTROLLERS.JOINT_HOLD, label:'Estimated joint-space PD hold',
  claim:'Bounded ideal joint torques; not a verified balance, walking or hardware controller.',
  kp: ASIMOV_SOURCE.joints.map(j => j.id.includes('ankle') ? 140 : j.id.includes('hip') || j.id.includes('knee') ? 180 : 60),
  kd: ASIMOV_SOURCE.joints.map(j => j.id.includes('ankle') ? 4 : j.id.includes('hip') || j.id.includes('knee') ? 6 : 3),
});
export const ASIMOV_LOWLEVEL_COMMAND_PROFILE = Object.freeze({id:ASIMOV_CONTROLLERS.LOWLEVEL,label:'Bounded ideal joint torque control',claim:ASIMOV_JOINT_HOLD_PROFILE.claim});
export const ASIMOV_FOOT_CONTACT_GEOMS = Object.freeze(Object.fromEntries(['left','right'].map(side => [side,[1,2,3,4].map(i => `${side}_foot${i}_collision`)])));
const clamp = (v,l,h) => Math.min(h,Math.max(l,v));
export function boundLowLevelCommand(jointId, request = {}) {
  const index=ASIMOV_JOINT_ORDER.indexOf(jointId);
  if(index<0) throw new RangeError(`Unknown Asimov joint ${jointId}`);
  const j=ASIMOV_SOURCE.joints[index];
  const fields=['positionRad','velocityRadS','feedforwardTorqueNm','kp','kd'];
  if(!request || typeof request!=='object' || Array.isArray(request)) throw new TypeError('Command must be an object');
  for(const key of Object.keys(request)) if(!fields.includes(key)) throw new RangeError(`Unknown low-level field ${key}`);
  const requested=Object.fromEntries(fields.map(key=>[key,request[key] ?? 0]));
  if(!Object.values(requested).every(v=>typeof v==='number' && Number.isFinite(v))) throw new TypeError('Asimov commands require finite numeric values');
  const ranges=[j.rangeRad,[-j.velocityLimitRadS,j.velocityLimitRadS],[-j.effortLimitNm,j.effortLimitNm],[0,ASIMOV_MAX_KP],[0,ASIMOV_MAX_KD]];
  const accepted=Object.fromEntries(fields.map((key,i)=>[key,clamp(requested[key],...ranges[i])]));
  return Object.freeze({...accepted,jointId,index,requested:Object.freeze(requested),bounded:fields.some(key=>accepted[key]!==requested[key]),
    limits:Object.freeze({jointRangeRad:j.rangeRad,velocityLimitRadS:j.velocityLimitRadS,effortLimitNm:j.effortLimitNm})});
}
export function lowLevelTorqueNm(command, measured) {
  const {positionRad:q,velocityRadS:dq}=measured;
  if(!Number.isFinite(q)||!Number.isFinite(dq)) throw new TypeError('Non-finite physical state');
  const tau=command.feedforwardTorqueNm+command.kp*(command.positionRad-q)+command.kd*(command.velocityRadS-dq);
  return clamp(tau,-command.limits.effortLimitNm,command.limits.effortLimitNm);
}
