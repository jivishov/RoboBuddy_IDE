import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js';

const DEG=Math.PI/180;
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const vec=(x=0,y=0,z=0)=>new THREE.Vector3(x,y,z);
function makeMat(color,metalness=.08,roughness=.65){return new THREE.MeshStandardMaterial({color,metalness,roughness});}
function segmentMesh(color,r=14){const m=new THREE.Mesh(new THREE.CylinderGeometry(r,r,1,16),makeMat(color));m.castShadow=true;m.receiveShadow=true;return m;}
function setSegment(mesh,a,b){const mid=a.clone().add(b).multiplyScalar(.5);const dir=b.clone().sub(a);mesh.position.copy(mid);mesh.scale.set(1,Math.max(.001,dir.length()),1);mesh.quaternion.setFromUnitVectors(vec(0,1,0),dir.clone().normalize());}
function box(w,h,d,color){const m=new THREE.Mesh(new THREE.BoxGeometry(w,h,d),makeMat(color));m.castShadow=true;m.receiveShadow=true;return m;}

export class RobotSimulator{
  constructor(canvas){
    this.canvas=canvas;this.scene=new THREE.Scene();this.scene.background=new THREE.Color(0x12171d);
    this.camera=new THREE.PerspectiveCamera(42,1,1,5000);this.camera.position.set(720,600,880);this.cameraTarget=vec(0,350,0);
    this.renderer=new THREE.WebGLRenderer({canvas,antialias:true,alpha:false});this.renderer.setPixelRatio(Math.min(devicePixelRatio||1,2));this.renderer.shadowMap.enabled=true;
    this.scene.add(new THREE.HemisphereLight(0xddeeff,0x223344,1.7));const key=new THREE.DirectionalLight(0xffffff,2.2);key.position.set(400,900,500);key.castShadow=true;this.scene.add(key);
    this.root=new THREE.Group();this.scene.add(this.root);this.profileId='openarm';this.state={};this.actionLog=[];this.contact={};this.attached=false;this.flaskVy=0;this.lastTime=performance.now();
    this.yaw=-0.55;this.pitch=0.42;this.distance=1120;this.drag=null;
    this._bindCamera();this.reset('openarm');this._loop();
  }
  _bindCamera(){
    this.canvas.addEventListener('pointerdown',e=>{this.drag={x:e.clientX,y:e.clientY,yaw:this.yaw,pitch:this.pitch};this.canvas.setPointerCapture(e.pointerId);});
    this.canvas.addEventListener('pointermove',e=>{if(!this.drag)return;this.yaw=this.drag.yaw-(e.clientX-this.drag.x)*.006;this.pitch=clamp(this.drag.pitch+(e.clientY-this.drag.y)*.005,-.1,1.25);});
    this.canvas.addEventListener('pointerup',()=>{this.drag=null;});
    this.canvas.addEventListener('wheel',e=>{e.preventDefault();this.distance=clamp(this.distance*Math.exp(e.deltaY*.001),420,2200);},{passive:false});
  }
  fit(){this.distance=this.profileId==='lekiwi'?1200:1050;this.yaw=-.55;this.pitch=.42;}
  reset(profileId,rest={}){this.profileId=profileId;this.state={...rest};this.attached=false;this.flaskVy=0;this.actionLog=[];this.contact={};this._buildScene();}
  _clear(){while(this.root.children.length)this.root.remove(this.root.children[0]);}
  _buildScene(){
    this._clear();const floor=box(1200,16,900,0x303740);floor.position.y=-8;this.root.add(floor);
    if(this.profileId==='openarm')this._buildOpenArm(); else if(this.profileId==='so101')this._buildSO101(); else this._buildLeKiwi();
  }
  _buildOpenArm(){
    const table=box(1000,30,700,0xd9dde0);table.position.y=305;this.root.add(table);this.tableTop=320;
    this.open={};this.open.left=this._armVisual(0x48a7a8);this.open.right=this._armVisual(0x737f8e);this.root.add(this.open.left.group,this.open.right.group);
    this.flask=new THREE.Group();const body=new THREE.Mesh(new THREE.ConeGeometry(44,76,28),new THREE.MeshPhysicalMaterial({color:0xa8e9ea,transparent:true,opacity:.42,roughness:.12,transmission:.18}));body.position.y=38;const neck=new THREE.Mesh(new THREE.CylinderGeometry(15.2,15.2,40,24),body.material);neck.position.y=95;this.flask.add(body,neck);this.root.add(this.flask);
    const save=this.state;this.state={...this.state,'left_joint_1.pos':10,'left_joint_2.pos':-41,'left_joint_4.pos':51.75,'left_gripper.pos':-65};const approach=this._openFK('left');this.flaskBottomOffset=approach.tool.y-this.tableTop;
    this.state={...save,'left_joint_1.pos':-18,'left_joint_2.pos':-24,'left_joint_4.pos':85.75,'left_gripper.pos':-23.70};const place=this._openFK('left');this.state=save;this.hotplateTop=place.tool.y-this.flaskBottomOffset;const hotplateHeight=Math.max(12,this.hotplateTop-this.tableTop);this.hotplate=box(190,hotplateHeight,180,0x31363b);this.hotplate.position.set(place.tool.x,this.tableTop+hotplateHeight/2,place.tool.z);this.root.add(this.hotplate);
    this._syncOpenArmPlacement(true);
  }
  _armVisual(color){const g=new THREE.Group();const base=box(70,70,70,0x22272d);g.add(base);const segs=[segmentMesh(color,16),segmentMesh(color,14),segmentMesh(color,11)];segs.forEach(s=>g.add(s));const wrist=box(42,42,46,0x20262b);g.add(wrist);const pads=[box(7,42,18,0x111417),box(7,42,18,0x111417)];pads.forEach(p=>g.add(p));return{group:g,base,segs,wrist,pads,tool:vec()};}
  _openFK(side){
    const s=side==='left'?1:-1;const prefix=side+'_';const shoulder=vec(s*-320,570,0);const j1=(this.state[prefix+'joint_1.pos']??0)*DEG;const j2=this.state[prefix+'joint_2.pos']??(side==='left'?-41:35);const j4=this.state[prefix+'joint_4.pos']??70;
    const effectiveJ2=side==='left'?j2:-j2;const t1=(effectiveJ2+10)*DEG;const t2=(effectiveJ2+50-.5*j4)*DEG;const L1=160,L2=160,L3=60;
    const radial1=vec(Math.cos(t1)*L1,Math.sin(t1)*L1,0);const radial2=vec(Math.cos(t2)*L2,Math.sin(t2)*L2,0);const radial3=vec(Math.cos(t2)*L3,Math.sin(t2)*L3,0);
    const rotate=v=>{const x=v.x*s;return vec(x*Math.cos(j1),v.y,-x*Math.sin(j1));};const elbow=shoulder.clone().add(rotate(radial1));const wrist=elbow.clone().add(rotate(radial2));const tool=wrist.clone().add(rotate(radial3));const axis=vec(-Math.sin(j1),0,-Math.cos(j1)*s).normalize();
    return{shoulder,elbow,wrist,tool,axis,terminalAngle:t2/DEG};
  }
  _gripperGap(command){const c=clamp(Number(command??-65),-65,0);return 6+(-c/65)*66;}
  _updateOpenArmVisual(side){const v=this.open[side],fk=this._openFK(side);v.base.position.copy(fk.shoulder);setSegment(v.segs[0],fk.shoulder,fk.elbow);setSegment(v.segs[1],fk.elbow,fk.wrist);setSegment(v.segs[2],fk.wrist,fk.tool);v.wrist.position.copy(fk.tool);const gap=this._gripperGap(this.state[side+'_gripper.pos']);const centerOffset=gap/2+3.5;v.pads[0].position.copy(fk.tool).add(fk.axis.clone().multiplyScalar(centerOffset));v.pads[1].position.copy(fk.tool).add(fk.axis.clone().multiplyScalar(-centerOffset));v.pads.forEach(p=>p.quaternion.setFromAxisAngle(vec(0,1,0),Math.atan2(fk.axis.x,fk.axis.z)));v.tool.copy(fk.tool);return fk;}
  _syncOpenArmPlacement(initial=false){
    const left=this._updateOpenArmVisual('left');this._updateOpenArmVisual('right');
    if(initial){const save=this.state;this.state={...save,'left_joint_1.pos':10,'left_joint_2.pos':-41,'left_joint_4.pos':51.75,'left_gripper.pos':-65};const target=this._openFK('left');this.state=save;this.flask.position.set(target.tool.x,this.tableTop,target.tool.z);}
    if(this.attached){this.flask.position.set(left.tool.x,left.tool.y-this.flaskBottomOffset,left.tool.z);this.flaskVy=0;}this._evaluateOpenArmContact(left);
  }
  _evaluateOpenArmContact(left){const neck=vec(this.flask.position.x,this.flask.position.y+this.flaskBottomOffset,this.flask.position.z);const dxz=Math.hypot(left.tool.x-neck.x,left.tool.z-neck.z);const dy=Math.abs(left.tool.y-neck.y);const gap=this._gripperGap(this.state['left_gripper.pos']);const diameter=30.4;const surfaceError=gap-diameter;const bilateral=dxz<=4&&dy<=4&&surfaceError<=.75&&surfaceError>=-0.35;const overlap=Math.max(0,-surfaceError);this.contact={bilateral,dxz,dy,gap,diameter,surfaceError,overlap,pinchValid:bilateral&&overlap>=.15&&overlap<=.35,attached:this.attached};}
  _buildSO101(){this.so={};const table=box(760,25,580,0xd7dadd);table.position.y=12.5;this.root.add(table);const base=box(90,42,90,0x333941);base.position.set(0,46,0);this.root.add(base);this.so.segs=[segmentMesh(0xf6c85f,14),segmentMesh(0xf6c85f,13),segmentMesh(0xf6c85f,11),segmentMesh(0xf6c85f,9)];this.so.segs.forEach(s=>this.root.add(s));this.so.gripper=[box(8,42,12,0x202428),box(8,42,12,0x202428)];this.so.gripper.forEach(p=>this.root.add(p));this._updateSO101();}
  _updateSO101(){const st=this.state;const base=vec(0,70,0);const pan=(st['shoulder_pan.pos']??0)*DEG;const a1=(-30-(st['shoulder_lift.pos']??-70)*.55)*DEG;const a2=a1+(55-(st['elbow_flex.pos']??70)*.55)*DEG;const a3=a2+(20-(st['wrist_flex.pos']??55)*.25)*DEG;const pts=[base];const add=(p,L,a)=>p.clone().add(vec(Math.cos(a)*L*Math.cos(pan),Math.sin(a)*L,-Math.cos(a)*L*Math.sin(pan)));pts.push(add(pts[0],145,a1));pts.push(add(pts[1],145,a2));pts.push(add(pts[2],90,a3));pts.push(add(pts[3],45,a3));this.so.segs.forEach((m,i)=>setSegment(m,pts[i],pts[i+1]));const gap=12+(1-(st['gripper.pos']??20)/100)*46;const axis=vec(-Math.sin(pan),0,-Math.cos(pan));this.so.gripper[0].position.copy(pts[4]).add(axis.clone().multiplyScalar(gap/2));this.so.gripper[1].position.copy(pts[4]).add(axis.clone().multiplyScalar(-gap/2));this.contact={toolHeight:pts[4].y,gripperGap:gap};}
  _buildLeKiwi(){this.lekiwi={base:box(280,70,240,0x34404b)};this.lekiwi.base.position.y=45;this.root.add(this.lekiwi.base);this.lekiwi.segs=[segmentMesh(0xe3b957,13),segmentMesh(0xe3b957,12),segmentMesh(0xe3b957,9)];this.lekiwi.segs.forEach(s=>this.root.add(s));this.basePose={x:0,z:0,yaw:0};this._updateLeKiwi();}
  _updateLeKiwi(){const st=this.state;const base=vec(this.basePose.x,105,this.basePose.z);const pan=(this.basePose.yaw+(st['arm_shoulder_pan.pos']??0)*DEG);const a1=(-25-(st['arm_shoulder_lift.pos']??-75)*.45)*DEG;const a2=a1+(45-(st['arm_elbow_flex.pos']??70)*.45)*DEG;const p1=base.clone().add(vec(Math.cos(a1)*130*Math.cos(pan),Math.sin(a1)*130,-Math.cos(a1)*130*Math.sin(pan)));const p2=p1.clone().add(vec(Math.cos(a2)*130*Math.cos(pan),Math.sin(a2)*130,-Math.cos(a2)*130*Math.sin(pan)));const p3=p2.clone().add(vec(Math.cos(a2)*55*Math.cos(pan),Math.sin(a2)*55,-Math.cos(a2)*55*Math.sin(pan)));setSegment(this.lekiwi.segs[0],base,p1);setSegment(this.lekiwi.segs[1],p1,p2);setSegment(this.lekiwi.segs[2],p2,p3);this.lekiwi.base.position.set(this.basePose.x,45,this.basePose.z);this.lekiwi.base.rotation.y=this.basePose.yaw;this.contact={baseXmm:this.basePose.x,baseZmm:this.basePose.z,baseYawDeg:this.basePose.yaw/DEG};}
  _updateProfile(){if(this.profileId==='openarm')this._syncOpenArmPlacement(false);else if(this.profileId==='so101')this._updateSO101();else this._updateLeKiwi();}
  _checkTarget(target){if(this.profileId!=='openarm')return;const save=this.state;this.state=target;const left=this._openFK('left'),right=this._openFK('right');this.state=save;for(const [side,fk] of [['left',left],['right',right]])for(const [name,p] of [['elbow',fk.elbow],['wrist',fk.wrist],['tool',fk.tool]])if(p.y<this.tableTop+4)throw new Error(`${side} ${name} target would enter the modeled work surface.`);if(this.attached){const bottom=left.tool.y-this.flaskBottomOffset;const surface=this._supportSurface(left.tool.x,left.tool.z);if(bottom<surface-.6)throw new Error(`Attached flask target penetrates the modeled support surface by ${(surface-bottom).toFixed(2)} mm.`);}}
  applyAction(action,{duration=0.22}={}){
    const before={...this.state},target={...this.state,...action};this._checkTarget(target);this.actionLog.push({action:{...action},time:performance.now()});const keys=Object.keys(action);const start=performance.now();const ms=Math.max(1,duration*1000);
    return new Promise((resolve,reject)=>{const tick=now=>{try{const t=clamp((now-start)/ms,0,1),ease=t*t*(3-2*t);for(const key of keys)this.state[key]=Number(before[key]??target[key])+(Number(target[key])-Number(before[key]??target[key]))*ease;if(this.profileId==='openarm'){this._syncOpenArmPlacement(false);const g0=before['left_gripper.pos']??-65,g=this.state['left_gripper.pos']??g0;const closing=(target['left_gripper.pos']??g0)>g0;if(!this.attached&&closing&&this.contact.pinchValid){this.attached=true;this.contact.attached=true;this.flaskVy=0;}if(this.attached&&g<-30){this.attached=false;this.contact.attached=false;this.flaskVy=0;}}else this._updateProfile();if(t<1)requestAnimationFrame(tick);else{Object.assign(this.state,target);this._updateProfile();resolve();}}catch(e){reject(e);}};requestAnimationFrame(tick);});
  }
  advanceBase(seconds){if(this.profileId!=='lekiwi')return;const x=(this.state['x.vel']??0)*1000,y=(this.state['y.vel']??0)*1000,w=(this.state['theta.vel']??0)*DEG;const c=Math.cos(this.basePose.yaw),s=Math.sin(this.basePose.yaw);this.basePose.x+=(x*c-y*s)*seconds;this.basePose.z+=(-x*s-y*c)*seconds;this.basePose.yaw+=w*seconds;this._updateLeKiwi();}
  _supportSurface(x,z){if(this.profileId!=='openarm')return 0;const hp=this.hotplate.position;if(Math.abs(x-hp.x)<=95-28&&Math.abs(z-hp.z)<=90-28)return this.hotplateTop;return this.tableTop;}
  _physics(dt){if(this.profileId!=='openarm'||!this.flask||this.attached)return;const surface=this._supportSurface(this.flask.position.x,this.flask.position.z);if(this.flask.position.y>surface+.05||this.flaskVy<0){this.flaskVy-=9810*dt;this.flask.position.y+=this.flaskVy*dt;if(this.flask.position.y<=surface){this.flask.position.y=surface;this.flaskVy=0;}}}
  getTelemetry(){return{...this.state};}
  getContacts(){return{...this.contact,hardwareValidation:'pending',model:'browser geometry only'};}
  _loop(){requestAnimationFrame(()=>this._loop());const now=performance.now(),dt=Math.min(.05,(now-this.lastTime)/1000);this.lastTime=now;this._physics(dt);if(this.profileId==='openarm')this._syncOpenArmPlacement(false);this._resize();const cp=this.cameraTarget.clone().add(vec(Math.cos(this.yaw)*Math.cos(this.pitch)*this.distance,Math.sin(this.pitch)*this.distance,Math.sin(this.yaw)*Math.cos(this.pitch)*this.distance));this.camera.position.lerp(cp,.18);this.camera.lookAt(this.cameraTarget);this.renderer.render(this.scene,this.camera);}
  _resize(){const w=this.canvas.clientWidth,h=this.canvas.clientHeight;if(!w||!h)return;if(this.canvas.width!==Math.round(w*this.renderer.getPixelRatio())||this.canvas.height!==Math.round(h*this.renderer.getPixelRatio())){this.renderer.setSize(w,h,false);this.camera.aspect=w/h;this.camera.updateProjectionMatrix();}}
}
