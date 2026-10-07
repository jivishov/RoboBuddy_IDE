#!/usr/bin/env python3
"""Native reimplementation plus named synthetic sensitivity tests, not hardware validation.

Uses the unchanged MuJoCo body model and an independent Python sensor/controller law.
Experimental mass/COM/contact changes live in a native test instance only. No public
WebMCP operation changes these quantities. All trial outcomes are reported.
"""
import argparse
import copy
import hashlib
import json
import subprocess
from pathlib import Path
import mujoco
import numpy as np
from validate_asimov_actuators import Native, SOURCE, ROOT, CFG

PROFILES = json.loads(subprocess.check_output(['node','--input-type=module','-e',
    "import {ASIMOV_SENSITIVITY_PROFILES as p} from './src/physics/asimov-sensitivity.js';console.log(JSON.stringify(p));"],cwd=ROOT,text=True))

class SensorNative(Native):
    def __init__(self,profile='sensor-standing',dt=.0025,seed=None,delay=None,ankle_scale=None,mass_scale=1,com_shift=0,floor_friction=None,contact_time=None):
        super().__init__(dt=dt,feedback=False)
        # Published caps happen to be integers; sensitivity multipliers are not.
        self.cont = self.cont.astype(float)
        self.profile=copy.deepcopy(PROFILES[profile]);self.sensor=self.profile['sensor'];self.balance=self.profile['balanceController']
        if seed is not None:self.sensor['seed']=seed
        if delay is not None:self.sensor['imuDelaySeconds']=delay
        self.rng=self.sensor['seed'];self.sensor_tick=0;self.history=[];self.filtered=None;self.last_sample=None;self.sensor_fault=False;self.feedback_torque=np.zeros(23)
        ankle=self.profile['actuator'].get('ankleStress')
        if ankle:
            self.cont[[4,5,10,11]]*=ankle['torqueScale'];self.speed_limits[[4,5,10,11]]=ankle['speedRadS'];self.family[[4,5,10,11]]=True
            self.fs[[4,5,10,11]]=ankle['staticNm'];self.fd[[4,5,10,11]]=ankle['dynamicNm']
        if ankle_scale is not None:self.cont[[4,5,10,11]]*=ankle_scale
        self.m.body_mass[:]*=mass_scale;self.m.body_inertia[:]*=mass_scale
        self.m.body_ipos[self.pelvis,0]+=com_shift
        if floor_friction is not None:
            # MuJoCo contact combination takes both materials into account.
            # Change all floor/foot sliding coefficients, not only the floor.
            for gid in range(self.m.ngeom):
                name=self.m.geom(gid).name or ''
                if name=='floor' or name.startswith(('left_foot','right_foot')):self.m.geom_friction[gid,0]=floor_friction
        if contact_time is not None:self.m.geom_solref[:,0]=contact_time
        # mj_setConst uses mjData as scratch and overwrites qpos with qpos0.
        # Never let a test-only parameter change erase the declared elbow offsets.
        if mass_scale != 1 or com_shift != 0:
            scratch = mujoco.MjData(self.m)
            mujoco.mj_setConst(self.m, scratch)
        mujoco.mj_resetDataKeyframe(self.m, self.d, 0)
        mujoco.mj_forward(self.m, self.d)
        assert np.array_equal(self.d.qpos, self.m.key_qpos[0]), 'Declared initial keyframe lost'
        assert np.array_equal(self.nom, self.d.qpos[self.q]), 'Native hold target differs from initial keyframe'
        self.sample()
    def noise(self,a):
        self.rng=(1664525*self.rng+1013904223)&0xffffffff
        return (2*self.rng/4294967296-1)*a
    def sample(self):
        t=float(self.d.time)
        if t+1e-10<self.sensor_tick*.005:return
        assert abs(t-self.sensor_tick*.005)<1e-8
        # Match the specified deterministic RNG order, including joint sensors
        # not consumed by the high-level balance regulator.
        for _ in range(23):self.noise(self.sensor['positionNoiseRad']);self.noise(self.sensor['velocityNoiseRadS'])
        w,x,y,z=self.d.xquat[self.pelvis]
        R=np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                    [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                    [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
        world=R@self.d.qvel[3:6];gyro=R.T@world+np.array([self.noise(self.sensor['angularVelocityNoiseRadS']) for _ in range(3)])
        gravity=R.T@np.array([0,0,-1])+np.array([self.noise(self.sensor['gravityNoise']) for _ in range(3)])
        gravity/=np.linalg.norm(gravity)
        self.history.append((self.sensor_tick*.005,gyro,gravity));self.history=self.history[-24:];self.sensor_tick+=1
    def balance_feedback(self):
        t=float(self.d.time);record=next((s for s in reversed(self.history) if s[0]<=t-self.sensor['imuDelaySeconds']+1e-10),None)
        if self.sensor_fault:return np.zeros(23)
        if record is None:
            if self.last_sample is not None or t>self.balance['warmupSeconds']+1e-9:self.sensor_fault=True
            return np.zeros(23)
        st,gyro,g=record
        if t-st>self.balance['maximumSensorAgeSeconds']+1e-9:self.sensor_fault=True;return np.zeros(23)
        self.last_sample=st
        values=np.array([np.arctan2(-g[1],-g[2]),np.arctan2(g[0],np.hypot(g[1],g[2])),gyro[0],gyro[1]])
        alpha=1-np.exp(-.005/self.balance['filterTimeConstantSeconds'])
        self.filtered=values if self.filtered is None else self.filtered+alpha*(values-self.filtered)
        r,p,wx,wy=self.filtered;kp=self.balance['orientationKp'];kd=self.balance['angularKd']
        pm=(kp*p+kd*wy)/2;rm=-(kp*r+kd*wx)/2
        out=np.zeros(23);out[[4,10]]=[pm,-pm];out[[5,11]]=rm
        return out
    def torque(self,enabled):
        if self.count%self.tick==0:
            self.feedback_torque=self.balance_feedback()
            self.frames.append(self.nom.copy())
            if len(self.frames)>self.delay:
                target=self.frames.pop(0)
                self.held=self.kp*(target-self.d.qpos[self.q])-self.kd*self.d.qvel[self.v]+self.feedback_torque
            else:self.held[:]=0
        vel=self.d.qvel[self.v]
        taper=np.where(self.family&(self.held*vel>0),np.maximum(0,1-np.abs(vel)/self.speed_limits),1)
        self.motor=np.clip(self.held,-self.cont*taper,self.cont*taper) if enabled else np.zeros(23)
        mag=self.fd+(self.fs-self.fd)*np.exp(-(vel/.1)**2)
        self.passive=-mag*np.tanh(vel/.02)
        self.d.ctrl[:]=self.motor;self.d.qfrc_applied[self.v]=self.passive
    def step(self,enabled=True,force=(0,0,0)):
        super().step(enabled,force);self.sample()


def run(name,**options):
    n=SensorNative(**options);dt=float(n.m.opt.timestep);c=CFG['criteria'];origin=n.d.xpos[n.pelvis].copy()
    fail=None;max_tilt=0;max_drift=0;dwell=0;impulse=np.zeros(2);switches=0;previous=None;trace=[];max_torque=0;saturated=0
    for k in range(round(12/dt)):
        n.step();tilt,feet,nonfoot=n.metrics();pos=n.d.xpos[n.pelvis];drift=float(np.linalg.norm(pos[:2]-origin[:2]));max_tilt=max(max_tilt,tilt);max_drift=max(max_drift,drift)
        impulse+=dt*np.array(feet);contact=tuple(f>0 for f in feet);switches+=int(previous is not None and contact!=previous);previous=contact
        max_torque=max(max_torque,float(np.max(np.abs(n.motor))));saturated+=int(np.any(np.abs(n.motor-n.held)>1e-8))
        error='sensor-fault' if n.sensor_fault else 'low-pelvis' if pos[2]<c['minPelvisHeightM'] else 'excess-tilt' if tilt>c['maxTiltRad'] else 'excess-drift' if drift>c['maxDriftM'] else 'non-foot-contact' if nonfoot else None
        if n.d.time>1+1e-10 and min(feet)<20:error=error or 'insufficient-support'
        if error and fail is None:fail={'reason':error,'time':float(n.d.time)}
        if n.d.time>1+1e-10 and fail is None:dwell+=dt
        if (k+1)%round(.1/dt)==0:trace.append({'time':float(n.d.time),'q':n.d.qpos[n.q].tolist(),'v':n.d.qvel[n.v].tolist(),'b':n.d.xpos[n.body_ids].tolist(),'motor':n.motor.tolist()})
    result={'name':name,'options':options,'pass':fail is None and dwell+1e-8>=10,'failure':fail,'maxTiltRad':max_tilt,'maxDriftM':max_drift,
        'footNormalImpulseNs':impulse.tolist(),'footContactStateChanges':switches,'maxMotorTorqueNm':max_torque,'saturationStepFraction':saturated/round(12/dt),'trace':trace}
    print(json.dumps({k:v for k,v in result.items() if k!='trace'}),flush=True);return result


def validate(wasm_path,report):
    wasm=json.loads(Path(wasm_path).read_text());parity={}
    for key,w in wasm.items():
        n=SensorNative(key);errors={'jointRad':0.,'bodyM':0.,'motorNm':0.}
        initial=w['initial']
        expected_q=np.array([initial['joints'][j['id']]['positionRad'] for j in SOURCE['joints']])
        expected_b=np.array([initial['bodies'][b]['positionM'] for b in SOURCE['bodies']])
        assert np.max(np.abs(n.d.qpos[n.q]-expected_q))<1e-12, (key,'initial joint mismatch')
        assert np.max(np.abs(n.d.xpos[n.body_ids]-expected_b))<1e-12, (key,'initial body mismatch')
        for o in w['trace']:
            while n.d.time<o['simulationTime']-1e-10:n.step()
            for label,a,b in [('jointRad',n.d.qpos[n.q],[o['joints'][j['id']]['positionRad'] for j in SOURCE['joints']]),('bodyM',n.d.xpos[n.body_ids],[o['bodies'][b]['positionM'] for b in SOURCE['bodies']]),('motorNm',n.motor,[j['motorNm'] for j in o['actuatorModel']['joints']])]:errors[label]=max(errors[label],float(np.max(np.abs(a-np.array(b)))))
        assert errors['jointRad']<1e-6 and errors['bodyM']<1e-6 and errors['motorNm']<1e-4,(key,errors)
        parity[key]=errors
        print(json.dumps({'nativeWasmProfile':key,'errors':errors}),flush=True)
    baseline=run('nominal');tight=run('tight-dt',dt=.00125)
    convergence={key:max(float(np.max(np.abs(np.array(a[field])-b[field]))) for a,b in zip(baseline['trace'],tight['trace'])) for key,field in [('jointRad','q'),('bodyM','b'),('velocityRadS','v'),('motorNm','motor')]}
    assert baseline['pass'] and tight['pass'] and convergence['jointRad']<=.025 and convergence['bodyM']<=.01,convergence
    tests=[baseline,tight]
    cases=[('seed-7',{'seed':7}),('seed-2026',{'seed':2026}),('delay-25ms',{'delay':.025}),('ankle-loss',{'profile':'sensor-standing-ankle-stress'}),
      ('nominal-35kg-hypothesis',{'mass_scale':35/32.22491296549718}),('pelvis-com-plus-2cm',{'com_shift':.02}),('low-floor-friction',{'floor_friction':.2}),('softer-contact',{'contact_time':.04}),
      ('combined',{'profile':'sensor-standing-ankle-stress','seed':7,'delay':.025,'mass_scale':35/32.22491296549718,'com_shift':.02,'floor_friction':.2}),
      ('sensor-stale-negative',{'delay':.06}),('ankle-disabled-negative',{'ankle_scale':0})]
    for name,kwargs in cases:tests.append(run(name,**kwargs))
    assert not tests[-1]['pass'] and not tests[-2]['pass'],'Negative controls must fail'
    output={'scope':'Synthetic model checks only; no hardware validation or confidence interval','engine':mujoco.__version__,'modelSha256':hashlib.sha256((ROOT/'models/asimov/actuator/freebase.xml').read_bytes()).hexdigest(),
      'nativeWasm':parity,'convergence':convergence,'trials':[{k:v for k,v in t.items() if k!='trace'} for t in tests],
      'massStudy':'Uniform mass/inertia scaling is a labeled hypothetical stress case only; production model remains unchanged','pass':True}
    Path(report).write_text(json.dumps(output,indent=2)+'\n');print(json.dumps({'parity':parity,'convergence':convergence}),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--wasm-report',required=True);p.add_argument('--report',required=True);args=p.parse_args();validate(args.wasm_report,args.report)
