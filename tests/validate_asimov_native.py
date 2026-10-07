#!/usr/bin/env python3
"""Executable source preservation and native/shipped-WASM parity gates, not hardware validation."""
import argparse, json, hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--wasm-report',type=Path,required=True);p.add_argument('--upstream',type=Path);p.add_argument('--report',type=Path);args=p.parse_args()
assert mujoco.__version__=='3.11.0'
audit=json.loads((ROOT/'models/asimov/source/audit.json').read_text()); wasm=json.loads(args.wasm_report.read_text()); metrics={}
source=ET.parse(ROOT/'models/asimov/source/asimov_1.xml').getroot()
for variant in ['mounted','freebase','drop']:
 path=ROOT/f'models/asimov/{variant}.xml'; assert hashlib.sha256(path.read_bytes()).hexdigest()==audit['modelHashes'][variant]
 xml=ET.parse(path).getroot()
 # Exact retained source inertials, body transforms (except declared root fixture), joints
 # (except declared torque limits), options, sensors and each collision geom's attributes.
 for tag in ['inertial','sensor']:
  a=source.findall('.//'+tag); b=xml.findall('.//'+tag)
  assert len(a)==len(b) and all(x.attrib==y.attrib for x,y in zip(a,b)),tag
 assert xml.find('option').attrib==source.find('option').attrib
 src_bodies=source.findall('.//worldbody//body'); out_bodies=xml.findall('.//worldbody//body')
 for a,b in zip(src_bodies,out_bodies):
  expected=dict(a.attrib);actual=dict(b.attrib)
  if a.get('name')=='pelvis_link': expected.pop('pos');actual.pop('pos')
  assert expected==actual
 for a,b in zip(source.findall('.//worldbody//joint'),xml.findall('.//worldbody//joint')):
  actual=dict(b.attrib);actual.pop('actuatorfrcrange');assert a.attrib==actual
 for a in source.findall('.//worldbody//geom'):
  if a.get('class')=='visual':continue
  b=xml.find(f'.//geom[@name="{a.get("name")}"]'); expected=dict(a.attrib);expected.pop('material',None);assert b is not None and b.attrib==expected
 assert len(xml.findall('.//worldbody//joint'))==23 and not xml.findall('.//joint[@name="neck_yaw_joint"]')
 m=mujoco.MjModel.from_xml_path(str(path));d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,0);mujoco.mj_forward(m,d)
 assert m.nu==23 and abs(m.body_mass.sum()-audit['totalMassKg'])<1e-10 and m.neq==0
 idx=np.array([m.joint(j['id']).qposadr[0] for j in audit['joints']]); dof=np.array([m.joint(j['id']).dofadr[0] for j in audit['joints']]); effort=np.array([j['effortLimitNm'] for j in audit['joints']])
 target=d.qpos[idx].copy();kp=np.array([140 if 'ankle' in j['id'] else 180 if 'hip' in j['id'] or 'knee' in j['id'] else 60 for j in audit['joints']]);kd=np.array([4 if 'ankle' in j['id'] else 6 if 'hip' in j['id'] or 'knee' in j['id'] else 3 for j in audit['joints']])
 if variant=='drop':kp*=0;kd*=0
 if variant=='mounted':target[[i for i,j in enumerate(audit['joints']) if j['id']=='left_elbow_joint']]=1;target[[i for i,j in enumerate(audit['joints']) if j['id']=='right_elbow_joint']]=-1
 samples=wasm['variants'][variant]['trajectory']; cursor=0; worst_q=worst_body=worst_force=0
 count=30 if variant=='drop' else 400
 for step in range(1,count+1):
  d.ctrl[:]=np.clip(kp*(target-d.qpos[idx])-kd*d.qvel[dof],-effort,effort);mujoco.mj_step(m,d)
  if step%4==0 or step==count:
   mujoco.mj_forward(m,d);w=samples[cursor];cursor+=1
   assert abs(d.time-w['simulationTime'])<1e-10
   worst_q=max(worst_q,max(abs(d.qpos[idx[i]]-w['joints'][j['id']]['positionRad']) for i,j in enumerate(audit['joints'])))
   for name in audit['bodies']:worst_body=max(worst_body,float(np.max(np.abs(d.xpos[m.body(name).id]-w['bodies'][name]['positionM']))))
   for i,c in enumerate(d.contact):
    force=np.zeros(6);mujoco.mj_contactForce(m,d,i,force)
    names=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,g) for g in c.geom]
    matches=[x for x in w['contacts'] if x['geoms']==names and abs(x['distanceM']-c.dist)<1e-7]
    assert matches,'native/browser contact pair mismatch';worst_force=max(worst_force,min(abs(force[0]-x['normalForceN']) for x in matches))
 assert worst_q<1e-6 and worst_body<1e-6 and worst_force<1e-3,(variant,worst_q,worst_body,worst_force)
 metrics[variant]={'maxJointErrorRad':worst_q,'maxBodyErrorM':worst_body,'maxContactNormalForceErrorN':worst_force,'samples':cursor}
 if variant=='drop':
  for _ in range(400):mujoco.mj_step(m,d)
  mujoco.mj_forward(m,d);assert np.isfinite(d.qpos).all() and d.ncon>0 and d.xpos[m.body('pelvis_link').id][2]<.5
  metrics[variant]['passiveFallContacts']=d.ncon
# Original source full visual model and stripped package preserve compiled physical body properties.
if args.upstream:
 full=mujoco.MjModel.from_xml_path(str(args.upstream/'sim-model/xmls/asimov_1.xml')); stripped=mujoco.MjModel.from_xml_path(str(ROOT/'models/asimov/freebase.xml'))
 for attr in ['body_mass','body_inertia','body_ipos','body_iquat','body_pos','body_quat','jnt_axis','jnt_pos','jnt_range','qpos0','dof_armature','dof_damping']:
  assert np.allclose(getattr(full,attr),getattr(stripped,attr),rtol=0,atol=1e-12),attr
 metrics['fullSourceCompiledPhysics']='identical body inertias/transforms/joints/armature/damping to 1e-12'
print(json.dumps(metrics,indent=2))
if args.report:args.report.write_text(json.dumps({'pass':True,'runtime':mujoco.__version__,'metrics':metrics},indent=2)+'\n')
