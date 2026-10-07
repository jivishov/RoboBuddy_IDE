import assert from 'node:assert/strict';
import {ASIMOV_SOURCE} from '../../src/physics/asimov-generated.js';
import {ASIMOV_BODY_SENSOR_PROFILE as P, ASIMOV_SENSITIVITY_PROFILES as profiles} from '../../src/physics/asimov-sensitivity.js';
import {AsimovBodySensors,worldToBody} from '../../src/physics/asimov-body-sensors.js';
import {AsimovSensorStanding} from '../../src/physics/asimov-sensor-standing.js';
import {AsimovActuatorPlant,actuatorOutput} from '../../src/physics/asimov-actuator-plant.js';
import {ASIMOV_ACTUATOR_PROFILE as motors} from '../../src/physics/asimov-actuator-profile.js';
const close=(a,b)=>a.forEach((v,i)=>assert.ok(Math.abs(v-b[i])<1e-10));
const h=Math.sqrt(.5);
close(worldToBody([h,0,0,h],[0,1,0]),[1,0,0]);
close(worldToBody([h,h,0,0],[0,0,-1]),[0,-1,0]);
close(worldToBody([h,0,h,0],[0,0,-1]),[1,0,0]);
assert.throws(()=>worldToBody([2,0,0,0],[0,0,0]));
const root={quaternionWxyz:[1,0,0,0],angularVelocityRadS:[0,0,0]};
const joints=Object.fromEntries(ASIMOV_SOURCE.joints.map(j=>[j.id,{positionRad:0,velocityRadS:0}]));
for(const delay of [0,.01,.025]) {
 const sensor=new AsimovBodySensors({...P,imuDelaySeconds:delay});sensor.sample(0,joints,root);
 assert.equal(sensor.observation(0).imu.valid,delay===0);
 const first=sensor.observation(0);for(let i=0;i<6;i++)assert.deepEqual(sensor.observation(0),first);
 for(let i=1;i<=8;i++)sensor.sample(i*.005,joints,root);
 const sample=sensor.observation(.04);assert.ok(Math.abs(sample.imu.ageSeconds-delay)<1e-9);
 assert.equal('root' in sample,false);assert.equal('bodies' in sample,false);assert.equal('angularVelocityWorldRadS' in sample.imu,false);
 assert.ok(Math.abs(Math.hypot(...sample.imu.projectedGravityBase)-1)<1e-12);
 assert.throws(()=>sensor.observation(.02));
 sensor.reset();sensor.sample(0,joints,root);assert.deepEqual(sensor.observation(0),first);
}
for(const bad of [-1,.003,Infinity,'0.01'])assert.throws(()=>new AsimovBodySensors({...P,imuDelaySeconds:bad}));
const s=new AsimovBodySensors();s.sample(0,joints,root);assert.throws(()=>s.sample(.01,joints,root));
const clean=new AsimovBodySensors({...P,imuDelaySeconds:0,gravityNoise:0,angularVelocityNoiseRadS:0});
const pitch=.03;clean.sample(0,joints,{quaternionWxyz:[Math.cos(pitch/2),0,Math.sin(pitch/2),0],angularVelocityRadS:[0,0,0]});
const obs=clean.observation(0);
Object.defineProperty(obs,'root',{get(){throw new Error('Ground-truth access is forbidden');}});
const balance=new AsimovSensorStanding();const torque=balance.update(0,obs);assert.ok(torque[4]>0&&torque[10]<0);assert.equal(balance.snapshot().groundTruthFallback,false);
const snap=balance.snapshot();assert.deepEqual(balance.update(0,obs),torque);assert.deepEqual(balance.snapshot(),snap);
for(let i=1;i<=8;i++)balance.update(i*.005,obs);
balance.update(.045,obs);assert.equal(balance.snapshot().status,'fault');
assert.ok(balance.torques.every(v=>v===0));balance.update(.05,{...obs,imu:{...obs.imu,sampleTimeSeconds:.05}});assert.equal(balance.snapshot().status,'fault');
const waiting=new AsimovSensorStanding();for(let i=0;i<12;i++)waiting.update(i*.005,{imu:{valid:false}});assert.equal(waiting.snapshot().status,'fault');
const ankle=profiles['sensor-standing-ankle-stress'].actuator;
const plant=new AsimovActuatorPlant(.0025,ankle);const spec=plant.specs[4];
assert.equal(spec.continuousLimitNm,28);assert.equal(motors.joints[4].continuousLimitNm,40);
for(const speed of [-20,-1,0,1,20])for(const request of [-100,100]){
 const o=actuatorOutput(spec,request,speed);assert.ok(o.frictionNm*speed<=0);assert.ok(Math.abs(o.motorNm)<=28);
 if(request*speed>0&&Math.abs(speed)>=9.32)assert.equal(Math.abs(o.motorNm),0);
}
assert.equal(plant.specs[0],motors.joints[0]);
console.log('Asimov sensor core: frames, normalized gravity, read independence, explicit delays, sensor-only feedback, latched faults and real ankle losses: OK');
