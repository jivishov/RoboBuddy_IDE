import assert from 'node:assert/strict';
import {ASIMOV_SOURCE} from '../../src/physics/asimov-generated.js';
import {ASIMOV_ACTUATOR_PROFILE as P,ASIMOV_SENSOR_PROFILE as S} from '../../src/physics/asimov-actuator-profile.js';
import {actuatorOutput,AsimovActuatorPlant} from '../../src/physics/asimov-actuator-plant.js';
import {parallelAnkle} from '../../src/physics/asimov-transmission.js';
import {AsimovSensorModel} from '../../src/physics/asimov-sensor-model.js';
import {AsimovStandingEvaluator,stanceFeedback} from '../../src/physics/asimov-standing.js';
assert.equal(P.joints.filter(j=>j.family).length,19);assert.equal(P.joints.filter(j=>!j.family).length,4);
for(const [i,j] of P.joints.entries()) {
 assert.equal(j.armatureKgM2,ASIMOV_SOURCE.joints[i].armature);
 assert.ok(j.continuousLimitNm<=j.sourceEffortNm);
 for(const v of [-40,-1,-.01,0,.01,1,40])for(const tau of [-1000,-1,0,1,1000]) {
  const o=actuatorOutput(j,tau,v);assert.ok(Math.abs(o.motorNm)<=j.continuousLimitNm+1e-12);assert.ok(o.frictionNm*v<=0);assert.equal(actuatorOutput(j,tau,v,{enabled:false}).motorNm,0);
  if(j.family&&v*tau>0&&Math.abs(v)>=j.speedRadS)assert.equal(Math.abs(o.motorNm),0);
  if(j.family&&v*tau<0)assert.ok(Math.abs(o.motorNm)>0);
 }
}
assert.equal(actuatorOutput(P.joints[0],100,0).motorNm,40);assert.equal(actuatorOutput(P.joints[3],100,0).motorNm,25);
assert.throws(()=>parallelAnkle(),/explicit/);
for(const [kp,kr,ia,ib] of [[1,1,.02,.02],[2,.7,.02,.03]]){
 const t=parallelAnkle({pitchRatio:kp,rollRatio:kr,motorInertiasKgM2:[ia,ib]});
 for(const q of [[.03,-.01],[-.2,.04]]) {
  const m=t.toMotor(q),r=t.toJoint(m);r.forEach((v,i)=>assert.ok(Math.abs(v-q[i])<1e-12));
  const tq=[7,-3],tm=t.jointToMotorTorque(tq);assert.ok(Math.abs(tq[0]*q[0]+tq[1]*q[1]-tm[0]*m[0]-tm[1]*m[1])<1e-12);
  const I=t.reflectedInertiaKgM2;assert.ok(Math.abs(I[0][0]*q[0]**2+2*I[0][1]*q[0]*q[1]+I[1][1]*q[1]**2-ia*m[0]**2-ib*m[1]**2)<1e-12);
 }
 const limited=t.limitJointTorque([100,100],[12,12]);assert.ok(limited.acceptedMotorNm.every(v=>Math.abs(v)<=12));assert.ok(limited.saturated);
}
const measurements=ASIMOV_SOURCE.joints.map(j=>({positionRad:j.referenceRad,velocityRadS:0}));
const commands=ASIMOV_SOURCE.joints.map(j=>({positionRad:j.referenceRad+.1,velocityRadS:0,feedforwardTorqueNm:0,kp:60,kd:3}));
for(const dt of [.005,.0025,.00125]){
 const plant=new AsimovActuatorPlant(dt);let first=null;
 for(let i=0;i<Math.round(.02/dt);i++){const out=plant.step(commands,measurements,true);if(out[0].motorNm!==0&&first==null)first=i*dt;}
 assert.equal(first,.005);assert.equal(plant.snapshot().controlTicks,4);
 plant.disable();assert.ok(plant.step(commands,measurements,false).every(o=>o.motorNm===0));plant.reset();assert.equal(plant.snapshot().stepIndex,0);
}
assert.throws(()=>new AsimovActuatorPlant(.003),/align/);
const root={quaternionWxyz:[1,0,0,0],angularVelocityRadS:[0,0,0],positionM:[0,0,.63],free:true,tiltRad:0};
const joints=Object.fromEntries(ASIMOV_SOURCE.joints.map(j=>[j.id,{positionRad:0,velocityRadS:0}]));
const sensor=new AsimovSensorModel();sensor.sample(0,joints,root);const a=sensor.observation(0);assert.equal(a.joints.left_hip_pitch_joint.valid,false);assert.equal(a.joints.left_elbow_joint.valid,true);
const rngBefore=sensor.rng;assert.deepEqual(sensor.observation(0),a);assert.equal(sensor.rng,rngBefore);
for(let t=.005;t<=.02+1e-10;t+=.005)sensor.sample(t,joints,root);
const b=sensor.observation(.0225);assert.ok(Math.abs(b.joints.left_hip_pitch_joint.ageSeconds-.0125)<1e-9);assert.equal('bodies' in b,false);assert.equal('root' in b,false);
for(const j of Object.values(b.joints)){assert.ok(Math.abs(j.positionRad)<=S.positionNoiseRad);assert.ok(Math.abs(j.velocityRadS)<=S.velocityNoiseRadS);}
sensor.reset();sensor.sample(0,joints,root);assert.deepEqual(sensor.observation(0),a);
const feedback=stanceFeedback({...root,quaternionWxyz:[Math.cos(.01),0,Math.sin(.01),0]});assert.ok(feedback[4]>0&&feedback[10]<0);
const ev=new AsimovStandingEvaluator();ev.start(0,root);const contact={readable:true,classified:{leftFootFloor:[{normalForceN:100}],rightFootFloor:[{normalForceN:100}],otherBodyFloor:[]}};
for(let i=1;i<=4401;i++)ev.update(i*.0025,root,contact,true,true);assert.equal(ev.snapshot().status,'passed');
ev.update(11.005,root,contact,false,true);assert.equal(ev.snapshot().status,'failed');assert.equal(ev.snapshot().firstFailure.reason,'actuation-disabled');
console.log('Asimov actuator: 19 role-matched motors, power/inertia-consistent ankle math, limits, friction passivity, fixed delay, deterministic sensors, standing evidence: OK');
