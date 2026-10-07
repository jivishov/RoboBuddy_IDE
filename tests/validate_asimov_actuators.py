#!/usr/bin/env python3
"""Independent native reference, numerical convergence and bounded sensitivity.

The friction/speed/delay sweeps are experiments, not measured parameter uncertainty.
External forces are applied only in this explicitly declared native test runner.
They never change root position/velocity or sneak into the browser controller.
"""
import argparse, hashlib, json, subprocess
from pathlib import Path
import mujoco
import numpy as np
ROOT=Path(__file__).resolve().parents[1]

def configuration():
    return json.loads(subprocess.check_output(['node','--input-type=module','-e',
        "import {ASIMOV_ACTUATOR_AUDIT as audit,ASIMOV_ACTUATOR_PROFILE as actuator,ASIMOV_SENSOR_PROFILE as sensor} from './src/physics/asimov-actuator-profile.js';import {ASIMOV_STANDING_CONTROLLER as standing,ASIMOV_STANDING_CRITERIA as criteria} from './src/physics/asimov-standing.js';console.log(JSON.stringify({audit,actuator,sensor,standing,criteria}));"],cwd=ROOT,text=True))
CFG=configuration()
SOURCE=json.loads((ROOT/'models/asimov/source/audit.json').read_text())

class Native:
    def __init__(self,variant='freebase',dt=.0025,delay=.005,friction=1,speed=1,feedback=True):
        self.m=mujoco.MjModel.from_xml_path(str(ROOT/f'models/asimov/actuator/{variant}.xml'))
        self.m.opt.timestep=dt;self.d=mujoco.MjData(self.m);mujoco.mj_resetDataKeyframe(self.m,self.d,0);mujoco.mj_forward(self.m,self.d)
        self.q=np.array([self.m.joint(j['id']).qposadr[0] for j in SOURCE['joints']]);self.v=np.array([self.m.joint(j['id']).dofadr[0] for j in SOURCE['joints']])
        self.nom=self.d.qpos[self.q].copy();self.pelvis=self.m.body('pelvis_link').id
        self.kp=np.array([140 if 'ankle' in j['id'] else 180 if 'hip' in j['id'] or 'knee' in j['id'] else 60 for j in SOURCE['joints']]);self.kd=np.array([4 if 'ankle' in j['id'] else 6 if 'hip' in j['id'] or 'knee' in j['id'] else 3 for j in SOURCE['joints']])
        self.tick=round(.005/dt);self.delay=round(delay/.005);assert abs(self.tick*dt-.005)<1e-12 and abs(self.delay*.005-delay)<1e-12
        self.friction=friction;self.speed=speed;self.feedback=feedback;self.held=np.zeros(23);self.frames=[];self.count=0;self.motor=np.zeros(23);self.passive=np.zeros(23)
        self.specs=CFG['actuator']['joints'];self.cont=np.array([s['continuousLimitNm'] for s in self.specs]);self.speed_limits=np.array([s['speedRadS'] for s in self.specs]);self.family=np.array([bool(s['family']) for s in self.specs]);self.fs=np.array([s['staticNm'] for s in self.specs]);self.fd=np.array([s['dynamicNm'] for s in self.specs]);self.body_ids=[self.m.body(n).id for n in SOURCE['bodies']]
    def torque(self,enabled):
        if self.count%self.tick==0:
            self.frames.append(self.nom.copy())
            if len(self.frames)>self.delay:
                target=self.frames.pop(0)
                self.held=self.kp*(target-self.d.qpos[self.q])-self.kd*self.d.qvel[self.v]
                if self.feedback:
                    w,x,y,z=self.d.xquat[self.pelvis];pitch=np.arcsin(np.clip(2*(w*y-z*x),-1,1));roll=np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y));yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
                    angular=self.d.xmat[self.pelvis].reshape(3,3)@self.d.qvel[3:6]
                    wx,wy=angular[:2];c=np.cos(yaw);s=np.sin(yaw);kp=CFG['standing']['orientationKp'];kd=CFG['standing']['angularKd']
                    pm=(kp*pitch+kd*(-s*wx+c*wy))/2;rm=-(kp*roll+kd*(c*wx+s*wy))/2
                    self.held[4]+=pm;self.held[10]-=pm;self.held[5]+=rm;self.held[11]+=rm
            else:self.held[:]=0
        vel=self.d.qvel[self.v]
        taper=np.where(self.family & (self.held*vel>0),np.maximum(0,1-np.abs(vel)/(self.speed_limits*self.speed)),1)
        cap=self.cont*taper
        self.motor=np.clip(self.held,-cap,cap) if enabled else np.zeros(23)
        mag=self.fd+(self.fs-self.fd)*np.exp(-(vel/CFG['actuator']['frictionTransitionRadS'])**2)
        self.passive=-self.friction*mag*np.tanh(vel/CFG['actuator']['frictionRegularizationRadS'])
        self.d.ctrl[:]=self.motor;self.d.qfrc_applied[self.v]=self.passive
    def step(self,enabled=True,force=(0,0,0)):
        self.torque(enabled);self.d.xfrc_applied[self.pelvis,:3]=force
        mujoco.mj_step(self.m,self.d);mujoco.mj_forward(self.m,self.d);self.count+=1
        assert np.isfinite(self.d.qpos).all() and np.isfinite(self.d.qvel).all()
    def state(self):
        return {'time':float(self.d.time),'q':self.d.qpos[self.q].tolist(),'body':self.d.xpos[self.body_ids].tolist(),'motor':self.motor.tolist()}
    def metrics(self):
        w,x,y,z=self.d.xquat[self.pelvis];tilt=np.arccos(np.clip(1-2*(x*x+y*y),-1,1))
        foot=[0.,0.];nonfoot=False
        for i,c in enumerate(self.d.contact):
            names=[self.m.geom(int(g)).name for g in c.geom];f=np.zeros(6);mujoco.mj_contactForce(self.m,self.d,i,f)
            if 'floor' not in names:continue
            other=next(n for n in names if n!='floor')
            if other.startswith('left_foot'):foot[0]+=max(0,f[0])
            elif other.startswith('right_foot'):foot[1]+=max(0,f[0])
            else:nonfoot=True
        return float(tilt),foot,nonfoot

def trial(name,dt=.0025,feedback=True,force=(0,0,0),off=False,friction=1,speed=1,delay=.005):
    n=Native(dt=dt,feedback=feedback,friction=friction,speed=speed,delay=delay);origin=n.d.xpos[n.pelvis].copy();failure=None;dwell=0;max_tilt=0;max_drift=0;min_height=1e9;trace=[];impulse=[0.,0.]
    c=CFG['criteria'];samples=round(12/dt)
    for i in range(samples):
        t=i*dt;n.step(enabled=not off,force=force if 2<=t<2.2 else (0,0,0));tilt,feet,nonfoot=n.metrics();pos=n.d.xpos[n.pelvis];drift=float(np.linalg.norm(pos[:2]-origin[:2]));max_tilt=max(max_tilt,tilt);max_drift=max(max_drift,drift);min_height=min(min_height,float(pos[2]));impulse=[a+dt*b for a,b in zip(impulse,feet)]
        error='actuation-disabled' if off else 'low-pelvis' if pos[2]<c['minPelvisHeightM'] else 'excess-tilt' if tilt>c['maxTiltRad'] else 'excess-drift' if drift>c['maxDriftM'] else 'non-foot-ground-contact' if nonfoot else None
        settling=n.d.time<=c['settlingSeconds']+1e-10
        if not settling and min(feet)<c['minFootForceN']:error=error or 'insufficient-two-foot-support'
        if error and failure is None:failure={'reason':error,'time':float(n.d.time)}
        if not settling and failure is None:dwell+=dt
        if (i+1)%round(.1/dt)==0:trace.append(n.state())
    print(f'Completed {name}: tilt={max_tilt:.6f}, drift={max_drift:.6f}, firstFailure={failure}',flush=True)
    return {'name':name,'dt':dt,'controlIntervalSeconds':.005,'delaySeconds':delay,'frictionScale':friction,'speedScale':speed,
        'declaredForcePulse':{'newtons':list(force),'startSeconds':2,'durationSeconds':.2,'application':'pelvis external force; no state overwrite'},
        'pass':failure is None and dwell+1e-8>=c['requiredDwellSeconds'],'firstFailure':failure,'maxTiltRad':max_tilt,'maxDriftM':max_drift,'minHeightM':min_height,'validDwellSeconds':dwell,'footNormalImpulseNs':impulse,'trace':trace}

def parity(path):
    wasm=json.loads(Path(path).read_text());result={}
    for variant,w in wasm.items():
        native=Native('mounted' if variant=='actuator-mounted' else 'freebase',feedback=variant=='standing')
        if variant!='standing':native.nom[[j['id'] for j in SOURCE['joints']].index('left_elbow_joint')]=1;native.nom[[j['id'] for j in SOURCE['joints']].index('right_elbow_joint')]=-1
        errors={'jointRad':0.,'bodyM':0.,'motorNm':0.}
        for sample in w['trajectory']:
            target=sample['simulationTime']
            while native.d.time<target-1e-10:native.step()
            for key,actual,expect in [('jointRad',native.d.qpos[native.q],[sample['joints'][j['id']]['positionRad'] for j in SOURCE['joints']]),('bodyM',native.d.xpos[native.body_ids],[sample['bodies'][b]['positionM'] for b in SOURCE['bodies']]),('motorNm',native.motor,[j['motorNm'] for j in sample['actuatorModel']['joints']])]:
                errors[key]=max(errors[key],float(np.max(np.abs(np.array(actual)-np.array(expect)))))
        assert errors['jointRad']<1e-6 and errors['bodyM']<1e-6 and errors['motorNm']<1e-4,errors
        result[variant]={**errors,'samples':len(w['trajectory'])}
    return result

def mounted_excitation():
    """Changing commands expose delay/speed effects hidden by a quiet stance."""
    traces={};summaries=[]
    for name,delay,speed in [('default',.005,1),('delay-zero',0,1),('delay-10ms',.01,1),('speed-80pct',.005,.8),('speed-120pct',.005,1.2)]:
        n=Native('mounted',feedback=False,delay=delay,speed=speed)
        index=next(i for i,j in enumerate(SOURCE['joints']) if j['id']=='left_elbow_joint')
        rows=[]
        for k in range(round(2/n.m.opt.timestep)):
            t=k*n.m.opt.timestep
            target=.9 if t<.2 else 1.9 if t<.7 else .4 if t<1.2 else 1.6
            n.nom[index]=target;n.step()
            rows.append([n.d.time,n.d.qpos[n.q[index]],n.d.qvel[n.v[index]],n.motor[index],target])
        a=np.array(rows);traces[name]=a
        summaries.append({'name':name,'delaySeconds':delay,'speedScale':speed,
            'trackingRmseRad':float(np.sqrt(np.mean((a[:,1]-a[:,4])**2))),
            'maxSpeedRadS':float(np.max(np.abs(a[:,2]))),'maxMotorTorqueNm':float(np.max(np.abs(a[:,3])))})
    effects={name:float(np.max(np.abs(a[:,1]-traces['default'][:,1]))) for name,a in traces.items() if name!='default'}
    assert all(delta>1e-4 for delta in effects.values()),effects
    return {'durationSeconds':2,'joint':'left_elbow_joint','targetScheduleRad':[[0,.9],[.2,1.9],[.7,.4],[1.2,1.6]],
        'scope':'Sensitivity experiment only. Changing speed/delay must change the observed trajectory; not a hardware calibration.',
        'trials':summaries,'maxPositionDeltaFromDefaultRad':effects}

def validate(report,wasm=None):
    output={'engine':mujoco.__version__,'configuration':CFG,'scope':'Numerical and model checks only; no hardware data','nativeWasm':parity(wasm) if wasm else None}
    for v in ['mounted','freebase']:
        original=mujoco.MjModel.from_xml_path(str(ROOT/f'models/asimov/{v}.xml'));new=mujoco.MjModel.from_xml_path(str(ROOT/f'models/asimov/actuator/{v}.xml'))
        for field in ['body_mass','body_inertia','body_pos','body_quat','jnt_axis','jnt_range','dof_armature','dof_damping','dof_frictionloss','geom_pos','geom_size','geom_contype','geom_conaffinity','exclude_signature']:
            assert np.array_equal(getattr(original,field),getattr(new,field)),(v,field)
        assert np.all(new.dof_damping==0) and np.all(new.dof_frictionloss==0)
    output['preservedCompiledArrays']='exact (body, joint, inertia, collision, exclusions); only model name and integration timestep changed'
    runs=[trial('baseline-5ms',dt=.005),trial('release-2.5ms'),trial('tight-1.25ms',dt=.00125)]
    # Predeclared tolerances: same pass/fail and <= 0.025 rad joint / 0.01 m body
    # differences at common sample times. Do not compare peak impact forces pointwise.
    convergence=[]
    for x in runs[:2]:
        ref=runs[2];jq=max(float(np.max(np.abs(np.array(a['q'])-b['q']))) for a,b in zip(x['trace'],ref['trace']));bp=max(float(np.max(np.abs(np.array(a['body'])-b['body']))) for a,b in zip(x['trace'],ref['trace']))
        convergence.append({'name':x['name'],'maxJointDeltaRad':jq,'maxBodyDeltaM':bp,'pass':x['pass']==ref['pass'] and jq<=.025 and bp<=.01})
    assert all(r['pass'] for r in runs) and all(c['pass'] for c in convergence),convergence
    output['convergence']=convergence
    for name,args in [('push-x-plus',{'force':(5,0,0)}),('push-x-minus',{'force':(-5,0,0)}),('push-y-plus',{'force':(0,5,0)}),('push-y-minus',{'force':(0,-5,0)}),('friction-half',{'friction':.5}),('friction-double',{'friction':2}),('speed-80pct',{'speed':.8}),('speed-120pct',{'speed':1.2}),('delay-zero',{'delay':0}),('delay-10ms',{'delay':.01}),('hold-only-negative',{'feedback':False}),('actuation-off-negative',{'off':True}),('large-push-negative',{'force':(-150,0,0)})]:
        runs.append(trial(name,**args))
    assert all(r['pass'] for r in runs[:-3]),[(r['name'],r['firstFailure']) for r in runs[:-3] if not r['pass']]
    assert not runs[-1]['pass'] and not runs[-2]['pass'] and not runs[-3]['pass']
    output['trials']=[{k:v for k,v in run.items() if k!='trace'} for run in runs]
    output['mountedExcitation']=mounted_excitation()
    output['parameterSensitivityIsNotConfidenceInterval']=True
    output['pass']=True
    Path(report).write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps({'parity':output['nativeWasm'],'convergence':convergence,'trials':[{k:r[k] for k in ['name','pass','maxTiltRad','maxDriftM']} for r in output['trials']]},indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--report',default='/tmp/asimov-actuator-validation.json');p.add_argument('--wasm-report');args=p.parse_args();validate(args.report,args.wasm_report)
