import { ASIMOV_ACTUATOR_PROFILE as PROFILE } from './asimov-actuator-profile.js';
const clamp = (v, limit) => Math.max(-limit, Math.min(limit, v));
export function actuatorOutput(spec, requestedNm, velocityRadS, { enabled = true, frictionScale = 1, speedScale = 1 } = {}) {
  if (![requestedNm, velocityRadS, frictionScale, speedScale].every(Number.isFinite) || frictionScale < 0 || speedScale <= 0) throw new TypeError('Invalid actuator state or sensitivity multiplier');
  const motoring = requestedNm * velocityRadS > 0;
  const taper = spec.family && motoring ? Math.max(0,1-Math.abs(velocityRadS)/(spec.speedRadS*speedScale)) : 1;
  const effectiveLimitNm = spec.continuousLimitNm * taper;
  const motorNm = enabled ? clamp(requestedNm,effectiveLimitNm) : 0;
  const magnitude = spec.dynamicNm+(spec.staticNm-spec.dynamicNm)*Math.exp(-((velocityRadS/PROFILE.frictionTransitionRadS)**2));
  const frictionNm = -frictionScale*magnitude*Math.tanh(velocityRadS/PROFILE.frictionRegularizationRadS);
  return { requestedNm, motorNm, frictionNm, effectiveLimitNm, saturated:enabled && Math.abs(motorNm-requestedNm)>1e-10 };
}
/** Command history advances only at fixed simulation-time motor ticks. */
export class AsimovActuatorPlant {
  constructor(timestepSeconds, options = {}) {
    if(typeof timestepSeconds!=='number'||!Number.isFinite(timestepSeconds)||timestepSeconds<=0) throw new RangeError('Invalid actuator timestep');
    this.dt=timestepSeconds; this.options=Object.freeze({...options});
    this.every=Math.round(PROFILE.controlIntervalSeconds/timestepSeconds);
    this.delayTicks=Math.round((options.commandDelaySeconds ?? PROFILE.commandDelaySeconds)/PROFILE.controlIntervalSeconds);
    if(this.every<1 || Math.abs(this.every*timestepSeconds-PROFILE.controlIntervalSeconds)>1e-12 || !Number.isInteger(this.delayTicks) || this.delayTicks<0 || this.delayTicks>4 || Math.abs(this.delayTicks*PROFILE.controlIntervalSeconds-(options.commandDelaySeconds??PROFILE.commandDelaySeconds))>1e-12) throw new RangeError('Actuator timing must align with fixed 5 ms ticks');
    const ankle=options.ankleStress;
    if (ankle && (![ankle.torqueScale,ankle.speedRadS,ankle.staticNm,ankle.dynamicNm].every(Number.isFinite) || ankle.torqueScale<=0 || ankle.torqueScale>1 || ankle.speedRadS<=0 || ankle.staticNm<0 || ankle.dynamicNm<0)) throw new RangeError('Invalid hypothetical ankle loss profile');
    this.specs=PROFILE.joints.map(spec=>!spec.family && ankle ? {...spec,family:'hypothetical-independent-ankle',continuousLimitNm:spec.continuousLimitNm*ankle.torqueScale,speedRadS:ankle.speedRadS,staticNm:ankle.staticNm,dynamicNm:ankle.dynamicNm} : spec);
    this.reset();
  }
  reset() { this.stepIndex=0;this.queue=[];this.active=null;this.held=new Array(23).fill(0);this.last=[];this.updates=0; }
  disable() { this.queue=[];this.active=null;this.held.fill(0); }
  step(commands, measurements, enabled, feedbackNm = null) {
    if(commands.length!==23 || measurements.length!==23) throw new RangeError('Asimov requires 23 joint records');
    if(this.stepIndex%this.every===0) {
      this.updates++;
      this.queue.push(structuredClone(commands));
      if(this.queue.length>this.delayTicks) this.active=this.queue.shift();
      this.held=commands.map((_,i)=>{
        const c=this.active?.[i], m=measurements[i];
        return c ? c.feedforwardTorqueNm+c.kp*(c.positionRad-m.positionRad)+c.kd*(c.velocityRadS-m.velocityRadS)+(feedbackNm?.[i]??0) : 0;
      });
    }
    this.last=this.specs.map((spec,i)=>({...actuatorOutput(spec,this.held[i],measurements[i].velocityRadS,{...this.options,enabled}),
      appliedTargetRad:this.active?.[i]?.positionRad??null,profileId:PROFILE.id}));
    this.stepIndex++;
    return this.last;
  }
  snapshot() { return {id:PROFILE.id,controlTicks:this.updates,stepIndex:this.stepIndex,queuedFrames:this.queue.length,commandDelaySeconds:this.delayTicks*PROFILE.controlIntervalSeconds,
    continuousOnly:true,peakOperation:PROFILE.peakOperation,...(this.options.ankleStress?{ankleStress:structuredClone(this.options.ankleStress)}:{}),ankleTransmission:'source-equivalent; ratio/motor-rating evidence unresolved',joints:structuredClone(this.last)}; }
}
