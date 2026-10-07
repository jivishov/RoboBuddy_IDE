import { ASIMOV_SOURCE } from './asimov-generated.js';
export const ASIMOV_STANDING_CONTROLLER = Object.freeze({
  id:'asimov-stance-feedback-v1',label:'Experimental torso-feedback standing',
  claim:'Repository-designed flat-floor stance controller. Ground-truth base IMU feedback; no walking, hardware calibration, or general recovery claim.',
  orientationKp:200, angularKd:20, controlIntervalSeconds:.005,
});
export function stanceFeedback(root) {
  const [w,x,y,z]=root.quaternionWxyz;
  const pitch=Math.asin(Math.max(-1,Math.min(1,2*(w*y-z*x))));
  const roll=Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y));
  const yaw=Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));
  const [wx,wy]=root.angularVelocityRadS, c=Math.cos(yaw), s=Math.sin(yaw);
  const {orientationKp:kp,angularKd:kd}=ASIMOV_STANDING_CONTROLLER;
  const pitchMoment=(kp*pitch+kd*(-s*wx+c*wy))/2;
  const rollMoment=-(kp*roll+kd*(c*wx+s*wy))/2;
  return ASIMOV_SOURCE.joints.map(j=>j.id==='left_ankle_pitch_joint'?pitchMoment:j.id==='right_ankle_pitch_joint'?-pitchMoment:j.id.includes('ankle_roll')?rollMoment:0);
}
// Criteria chosen BEFORE the acceptance runs; a historical pass does not survive
// a later fall. Contacts are checked each physics step, not just each display frame.
export const ASIMOV_STANDING_CRITERIA=Object.freeze({settlingSeconds:1,requiredDwellSeconds:10,minPelvisHeightM:.50,maxTiltRad:.12,maxDriftM:.08,minFootForceN:20});
export class AsimovStandingEvaluator {
  constructor(){this.reset();}
  reset(){this.startedAt=null;this.origin=null;this.validDwellSeconds=0;this.failure=null;this.lastTime=null;this.maxTiltRad=0;this.maxDriftM=0;this.minHeightM=Infinity;this.samples=0;}
  start(time,root){this.reset();this.startedAt=time;this.origin=[...root.positionM];this.lastTime=time;}
  update(time,root,contacts,enabled,engaged) {
    if(this.startedAt==null)return;
    const dt=time-this.lastTime;this.lastTime=time;this.samples++;
    const drift=Math.hypot(root.positionM[0]-this.origin[0],root.positionM[1]-this.origin[1]);
    this.maxTiltRad=Math.max(this.maxTiltRad,root.tiltRad);this.maxDriftM=Math.max(this.maxDriftM,drift);this.minHeightM=Math.min(this.minHeightM,root.positionM[2]);
    const c=ASIMOV_STANDING_CRITERIA;
    const force=side=>(contacts.classified[side+'FootFloor']??[]).reduce((s,c)=>s+Math.max(0,c.normalForceN),0);
    const hardFailure=!enabled?'actuation-disabled':!engaged?'controller-released':!root.free?'mounted-root':root.positionM[2]<c.minPelvisHeightM?'low-pelvis':root.tiltRad>c.maxTiltRad?'excess-tilt':drift>c.maxDriftM?'excess-drift':contacts.classified.otherBodyFloor.length?'non-foot-ground-contact':null;
    const settling=time-this.startedAt<=c.settlingSeconds+1e-10;
    const fail=hardFailure??(!settling && (!contacts.readable || force('left')<c.minFootForceN || force('right')<c.minFootForceN)?'insufficient-two-foot-support':null);
    if(fail && !this.failure)this.failure={reason:fail,simulationTimeSeconds:time};
    if(!settling && !this.failure)this.validDwellSeconds+=dt;
  }
  snapshot(){return {success:this.startedAt!=null&&!this.failure&&this.validDwellSeconds+1e-8>=ASIMOV_STANDING_CRITERIA.requiredDwellSeconds,status:this.startedAt==null?'not-started':this.failure?'failed':this.validDwellSeconds+1e-8>=ASIMOV_STANDING_CRITERIA.requiredDwellSeconds?'passed':'running',
    controllerId:ASIMOV_STANDING_CONTROLLER.id,criteria:{...ASIMOV_STANDING_CRITERIA},validDwellSeconds:this.validDwellSeconds,firstFailure:this.failure,
    maxTiltRad:this.maxTiltRad,maxDriftM:this.maxDriftM,minHeightM:Number.isFinite(this.minHeightM)?this.minHeightM:null,samples:this.samples,
    scope:'Declared flat-floor simulation trial only; not calibrated hardware or walking.'};}
}
