import { ASIMOV_SENSOR_PROFILE as PROFILE } from './asimov-actuator-profile.js';
/** Sampling is tied to simulation time, never to reads or rendering. */
export class AsimovSensorModel {
  constructor() {this.reset();}
  reset() {this.rng=PROFILE.seed;this.samples=[];this.nextSample=0;this.current=null;}
  noise(amplitude) {if(!Number.isFinite(amplitude)||amplitude<0)throw new TypeError('Invalid sensor noise amplitude');this.rng=(Math.imul(1664525,this.rng)+1013904223)>>>0;return (2*this.rng/4294967296-1)*amplitude;}
  sample(time, joints, root) {
    if(time+1e-10<this.nextSample) return;
    const tick=Math.round(time/PROFILE.sampleIntervalSeconds);
    const sampleTime=tick*PROFILE.sampleIntervalSeconds;
    if(Math.abs(time-sampleTime)>1e-8) throw new Error('Sensor sample missed its declared fixed tick');
    const [w,x,y,z]=root.quaternionWxyz;
    const measured=Object.fromEntries(Object.entries(joints).map(([id,j])=>[id,{positionRad:j.positionRad+this.noise(PROFILE.positionNoiseRad),velocityRadS:j.velocityRadS+this.noise(PROFILE.velocityNoiseRadS)}]));
    // Project world gravity into the base frame. Angular velocity is explicitly WORLD frame.
    const g=[2*(w*y-x*z),-2*(w*x+y*z),-(1-2*(x*x+y*y))];
    const imu={sampleTimeSeconds:sampleTime,angularVelocityWorldRadS:root.angularVelocityRadS.map(v=>v+this.noise(PROFILE.angularVelocityNoiseRadS)),
      projectedGravityBase:g.map(v=>v+this.noise(PROFILE.gravityNoise))};
    this.samples.push({time:sampleTime,joints:measured,imu});if(this.samples.length>4)this.samples.shift();
    const delayed=Object.fromEntries(Object.keys(joints).map((id,i)=>{
      const cutoff=sampleTime-PROFILE.jointDelaySeconds[i];
      const s=[...this.samples].reverse().find(s=>s.time<=cutoff+1e-10)??null;
      return [id,s?{...s.joints[id],sampleTimeSeconds:s.time,ageSeconds:Math.max(0,time-s.time),valid:true}:{positionRad:null,velocityRadS:null,sampleTimeSeconds:null,ageSeconds:null,valid:false}];
    }));
    this.current={view:'hardware_like',profileId:PROFILE.id,evidence:PROFILE.label,seed:PROFILE.seed,simulationTimeSeconds:time,deliverySampleTimeSeconds:sampleTime,joints:delayed,imu,
      note:'Synthetic timing/noise sensitivity view. No exact root position, contact force, body pose, or effort measurement is exposed.'};
    this.nextSample=(tick+1)*PROFILE.sampleIntervalSeconds;
  }
  observation(time) {
    if(!this.current)return null;
    const result=structuredClone(this.current);result.simulationTimeSeconds=time;
    for(const j of Object.values(result.joints))if(j.valid)j.ageSeconds=Math.max(0,time-j.sampleTimeSeconds);
    result.imu.ageSeconds=Math.max(0,time-result.imu.sampleTimeSeconds);return result;
  }
}
