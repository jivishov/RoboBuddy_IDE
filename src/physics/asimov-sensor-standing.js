import { ASIMOV_SOURCE } from './asimov-generated.js';
import { ASIMOV_SENSOR_STANDING_CONTROLLER as P, ASIMOV_BODY_SENSOR_PROFILE } from './asimov-sensitivity.js';
const zeros=()=>ASIMOV_SOURCE.joints.map(()=>0);
const vec=v=>Array.isArray(v)&&v.length===3&&v.every(Number.isFinite);
/** No simulator state dependency: only the documented sensor observation enters feedback. */
export class AsimovSensorStanding {
  constructor() { this.reset(); }
  reset(time=0) { this.start=time;this.lastTick=null;this.lastSample=null;this.filtered=null;this.status='warming-up';this.fault=null;this.torques=zeros();this.updates=0;this.ageSeconds=null; }
  update(time, observation) {
    if (!Number.isFinite(time) || time<this.start-1e-9) throw new RangeError('Invalid balance time');
    const tick=Math.round(time/P.controlIntervalSeconds);
    if (this.lastTick===tick) return [...this.torques];
    if (Math.abs(time-tick*P.controlIntervalSeconds)>1e-8) throw new RangeError('Balance must run on the 5 ms motor clock');
    if (this.lastTick!=null && tick!==this.lastTick+1) throw new Error('Missed balance tick');
    this.lastTick=tick;this.updates++;
    if (this.fault) return zeros();
    const imu=observation?.imu;
    const age=time-Number(imu?.sampleTimeSeconds);
    const valid=observation?.profileId===ASIMOV_BODY_SENSOR_PROFILE.id && imu?.valid===true && imu.frame==='pelvis_link' && vec(imu.angularVelocityBodyRadS) && vec(imu.projectedGravityBase) &&
      Number.isFinite(imu.sampleTimeSeconds) && age>=-1e-9 && age<=P.maximumSensorAgeSeconds+1e-9 && Math.abs(Math.hypot(...imu.projectedGravityBase)-1)<1e-6 &&
      (this.lastSample==null || imu.sampleTimeSeconds>=this.lastSample-1e-9);
    if (!valid) {
      this.torques=zeros();this.ageSeconds=null;
      // Only unavailable cold-start history receives a bounded grace period.
      if (this.lastSample==null && imu?.valid===false && time-this.start<=P.warmupSeconds+1e-9) this.status='warming-up';
      else {this.status='fault';this.fault={reason:'invalid-or-stale-sensor',simulationTimeSeconds:time};}
      return [...this.torques];
    }
    this.ageSeconds=Math.max(0,age);this.lastSample=imu.sampleTimeSeconds;
    const [gx,gy,gz]=imu.projectedGravityBase;
    const pitch=Math.atan2(gx,Math.hypot(gy,gz)),roll=Math.atan2(-gy,-gz);
    const values=[roll,pitch,imu.angularVelocityBodyRadS[0],imu.angularVelocityBodyRadS[1]];
    const alpha=1-Math.exp(-P.controlIntervalSeconds/P.filterTimeConstantSeconds);
    this.filtered=this.filtered?values.map((v,i)=>this.filtered[i]+alpha*(v-this.filtered[i])):values;
    const [r,p,wx,wy]=this.filtered;
    const pm=(P.orientationKp*p+P.angularKd*wy)/2,rm=-(P.orientationKp*r+P.angularKd*wx)/2;
    this.torques=ASIMOV_SOURCE.joints.map(j=>j.id==='left_ankle_pitch_joint'?pm:j.id==='right_ankle_pitch_joint'?-pm:j.id.includes('ankle_roll')?rm:0);
    this.status='active';return [...this.torques];
  }
  snapshot() { return {id:P.id,status:this.status,fault:this.fault?{...this.fault}:null,updates:this.updates,sensorAgeSeconds:this.ageSeconds,
    filterTimeConstantSeconds:P.filterTimeConstantSeconds,input:'hardware_like body sensors only',
    localJointPD:'ideal current local encoder feedback; not the delayed high-level sensor view',groundTruthFallback:false}; }
}
