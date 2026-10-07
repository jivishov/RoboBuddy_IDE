import { ASIMOV_SOURCE } from './asimov-generated.js';
import { ASIMOV_BODY_SENSOR_PROFILE } from './asimov-sensitivity.js';
const JOINT_DELAYS = Object.freeze(Object.fromEntries(ASIMOV_SOURCE.joints.map((j, i) => [j.id, i < 8 ? 0.01 : i < 16 ? 0.005 : 0])));
const finiteVector = (v, n) => Array.isArray(v) && v.length === n && v.every(Number.isFinite);
export function worldToBody(q, v) {
  if (!finiteVector(q, 4) || !finiteVector(v, 3)) throw new TypeError('Finite quaternion and three-vector required');
  const norm = Math.hypot(...q); if (Math.abs(norm - 1) > 1e-6) throw new RangeError('Unit quaternion required');
  const [w,x,y,z] = q, [a,b,c] = v;
  return [(1-2*(y*y+z*z))*a+2*(x*y+w*z)*b+2*(x*z-w*y)*c,
    2*(x*y-w*z)*a+(1-2*(x*x+z*z))*b+2*(y*z+w*x)*c,
    2*(x*z+w*y)*a+2*(y*z-w*x)*b+(1-2*(x*x+y*y))*c];
}
/** This sensor is a synthetic attitude-derived observation, not a raw IMU emulator. */
export class AsimovBodySensors {
  constructor(profile = ASIMOV_BODY_SENSOR_PROFILE) {
    this.profile = structuredClone(profile);
    const p = this.profile;
    if (p.id !== ASIMOV_BODY_SENSOR_PROFILE.id || p.sampleIntervalSeconds !== 0.005 || p.frame !== 'pelvis_link') throw new RangeError('Unsupported sensor contract');
    if (!Number.isInteger(p.seed) || p.seed < 0 || p.seed > 0xffffffff) throw new RangeError('Sensor seed must be uint32');
    if (!Number.isFinite(p.imuDelaySeconds) || p.imuDelaySeconds < 0 || p.imuDelaySeconds > 0.1 || Math.abs(p.imuDelaySeconds/0.005-Math.round(p.imuDelaySeconds/0.005))>1e-9) throw new RangeError('IMU delay must align to 5 ms, in 0..100 ms');
    for (const k of ['positionNoiseRad','velocityNoiseRadS','angularVelocityNoiseRadS','gravityNoise']) if (!Number.isFinite(p[k]) || p[k]<0 || p[k]>1) throw new RangeError('Invalid synthetic noise amplitude');
    Object.freeze(this.profile); this.reset();
  }
  reset() { this.rng=this.profile.seed; this.nextTick=0; this.samples=[]; this.current=null; }
  noise(a) { this.rng=(Math.imul(1664525,this.rng)+1013904223)>>>0; return (2*this.rng/4294967296-1)*a; }
  sample(time, joints, root) {
    const p=this.profile, dt=p.sampleIntervalSeconds;
    if (!Number.isFinite(time) || time<0) throw new RangeError('Invalid sensor time');
    if (time+1e-10<this.nextTick*dt) return;
    if (Math.abs(time-this.nextTick*dt)>1e-8) throw new Error('Sensor sample missed a fixed tick');
    const measured={};
    for (const {id} of ASIMOV_SOURCE.joints) {
      const j=joints[id]; if (!j || !Number.isFinite(j.positionRad) || !Number.isFinite(j.velocityRadS)) throw new TypeError('Invalid joint sensor source');
      measured[id]={positionRad:j.positionRad+this.noise(p.positionNoiseRad),velocityRadS:j.velocityRadS+this.noise(p.velocityNoiseRadS)};
    }
    const gyro=worldToBody(root.quaternionWxyz,root.angularVelocityRadS).map(v=>v+this.noise(p.angularVelocityNoiseRadS));
    const gravity=worldToBody(root.quaternionWxyz,[0,0,-1]).map(v=>v+this.noise(p.gravityNoise));
    const norm=Math.hypot(...gravity);
    const imu={frame:p.frame,angularVelocityBodyRadS:gyro,projectedGravityBase:gravity.map(v=>v/norm)};
    const t=this.nextTick*dt;
    this.samples.push({time:t,joints:measured,imu}); if (this.samples.length>24) this.samples.shift();
    const past=delay=>this.samples.findLast(s=>s.time<=t-delay+1e-10);
    const out={};
    for (const {id} of ASIMOV_SOURCE.joints) {
      const s=past(JOINT_DELAYS[id]);
      out[id]=s?{...s.joints[id],valid:true,sampleTimeSeconds:s.time}:{positionRad:null,velocityRadS:null,valid:false,sampleTimeSeconds:null};
    }
    const s=past(p.imuDelaySeconds);
    this.current={view:'hardware_like',profileId:p.id,frame:p.frame,seed:p.seed,evidence:p.evidence,
      deliverySampleTimeSeconds:t,joints:out,
      imu:s?{...s.imu,valid:true,sampleTimeSeconds:s.time}:{frame:p.frame,valid:false,sampleTimeSeconds:null,angularVelocityBodyRadS:null,projectedGravityBase:null},
      timing:{sampleIntervalSeconds:dt,imuDelaySeconds:p.imuDelaySeconds,jointDelaySeconds:JOINT_DELAYS},
      note:'Only synthetic measurements. No exact root pose, world-frame gyro, contact force, or body transforms.'};
    this.nextTick++;
  }
  observation(time) {
    if (!this.current) return null;
    if (!Number.isFinite(time) || time+1e-9<this.current.deliverySampleTimeSeconds) throw new RangeError('Cannot read sensors before delivery');
    const o=structuredClone(this.current); o.simulationTimeSeconds=time;
    for (const s of [...Object.values(o.joints),o.imu]) s.ageSeconds=s.valid?Math.max(0,time-s.sampleTimeSeconds):null;
    return o;
  }
}
