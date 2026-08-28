import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js';
import { CanonicalRobotRig, canonicalVisualProvenance } from './canonical-rig.js';

const DEG=Math.PI/180;
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const vec=(x=0,y=0,z=0)=>new THREE.Vector3(x,y,z);
function makeMat(color,metalness=.08,roughness=.65,extra={}){return new THREE.MeshStandardMaterial({color,metalness,roughness,...extra});}
function box(w,h,d,color,materialOptions={}){const m=new THREE.Mesh(new THREE.BoxGeometry(w,h,d),makeMat(color,materialOptions.metalness??.08,materialOptions.roughness??.65,materialOptions));m.castShadow=true;m.receiveShadow=true;return m;}

const OPENARM_WORKTOP_TOP=320;
const OPENARM_FLASK_HEIGHT=114;
const OPENARM_FLASK_GRIP_Y=96;
const OPENARM_HOTPLATE={x:439.626,z:-339.239,width:170,depth:170,height:46.329};

export class RobotSimulator{
  constructor(canvas){
    this.canvas=canvas;this.scene=new THREE.Scene();this.scene.background=new THREE.Color(0x12171d);
    this.camera=new THREE.PerspectiveCamera(42,1,1,5000);this.cameraTarget=vec(0,350,0);
    this.renderer=new THREE.WebGLRenderer({canvas,antialias:true,alpha:false});this.renderer.setPixelRatio(Math.min(devicePixelRatio||1,2));this.renderer.shadowMap.enabled=true;
    this.scene.add(new THREE.HemisphereLight(0xddeeff,0x223344,1.7));const key=new THREE.DirectionalLight(0xffffff,2.2);key.position.set(400,900,500);key.castShadow=true;this.scene.add(key);
    this.root=new THREE.Group();this.scene.add(this.root);this.environment=new THREE.Group();this.root.add(this.environment);
    this.profileId='openarm';this.state={};this.actionLog=[];this.contact={};this.attached=false;this.attachment=null;this.flaskVy=0;this.basePose={x:0,z:0,yaw:0};this.rig=null;this.rigLoadToken=0;this.rigError=null;this.rigReady=Promise.resolve(null);
    this.yaw=-0.55;this.pitch=0.42;this.distance=1120;this.drag=null;
    this._bindCamera();this.reset('openarm');this._loop();
  }
  _bindCamera(){
    this.canvas.addEventListener('pointerdown',e=>{this.drag={x:e.clientX,y:e.clientY,yaw:this.yaw,pitch:this.pitch};this.canvas.setPointerCapture(e.pointerId);});
    this.canvas.addEventListener('pointermove',e=>{if(!this.drag)return;this.yaw=this.drag.yaw-(e.clientX-this.drag.x)*.006;this.pitch=clamp(this.drag.pitch+(e.clientY-this.drag.y)*.005,-.1,1.25);});
    this.canvas.addEventListener('pointerup',()=>{this.drag=null;});
    this.canvas.addEventListener('wheel',e=>{e.preventDefault();this.distance=clamp(this.distance*Math.exp(e.deltaY*.001),260,3600);},{passive:false});
  }
  fit(){
    if(this.profileId==='openarm'){this.distance=1800;this.cameraTarget.set(230,500,0);this.yaw=-1.52;this.pitch=.32;}
    else if(this.profileId==='so101'){this.distance=650;this.cameraTarget.set(145,145,0);this.yaw=-.7;this.pitch=.35;}
    else{this.distance=720;this.cameraTarget.set(0,120,-20);this.yaw=-.65;this.pitch=.34;}
  }
  reset(profileId,rest={}){
    this.profileId=profileId;this.state={...rest};this.attached=false;this.attachment=null;this.flaskVy=0;this.actionLog=[];this.contact={canonicalVisual:'loading'};this.basePose={x:0,z:0,yaw:0};this.rigError=null;
    this._clearEnvironment();this._buildEnvironment();this._loadCanonicalRig(profileId);this.fit();
  }
  _clearEnvironment(){while(this.environment.children.length)this.environment.remove(this.environment.children[0]);}
  _disposeRig(){if(!this.rig)return;this.root.remove(this.rig.root);this.rig.dispose();this.rig=null;}
  _loadCanonicalRig(profileId){
    const token=++this.rigLoadToken;this._disposeRig();
    this.rigReady=CanonicalRobotRig.load(profileId).then(rig=>{if(token!==this.rigLoadToken){rig.dispose();return null;}this.rig=rig;this.root.add(rig.root);rig.applyPhysicalState(this.state,this.basePose);this.rigError=null;this.contact={...this.contact,canonicalVisual:'ready',robotId:rig.meshData.robotId};if(profileId==='openarm')this._evaluateOpenArmContact();return rig;}).catch(error=>{if(token!==this.rigLoadToken)return null;this.rigError=error;this.contact={canonicalVisual:'unavailable',visualError:error.message};console.error('Canonical RoboBuddy robot mesh failed to load.',error);return null;});
  }
  _buildEnvironment(){
    const floor=box(this.profileId==='openarm'?1800:1000,16,this.profileId==='openarm'?1500:900,0x303740);floor.position.y=-8;this.environment.add(floor);
    if(this.profileId==='openarm')this._buildOpenArmEnvironment();
  }
  _worktop(centerX,centerZ){
    const top=box(380,24,356,0xd9dde0,{roughness:.78,metalness:.02});top.position.set(centerX,308,centerZ);this.environment.add(top);
    for(const x of [centerX-140,centerX+140])for(const z of [centerZ-160,centerZ+160]){const leg=box(36,296,36,0x4a5158,{roughness:.6,metalness:.3});leg.position.set(x,148,z);this.environment.add(leg);}
    return top;
  }
  _buildOpenArmEnvironment(){
    this._worktop(375,-390);this._worktop(375,390);
    this.flask=new THREE.Group();this.flask.name='empty-erlenmeyer-flask';
    const glass=new THREE.MeshPhysicalMaterial({color:0xa8e9ea,transparent:true,opacity:.42,roughness:.12,transmission:.18,side:THREE.DoubleSide});
    const body=new THREE.Mesh(new THREE.CylinderGeometry(34,40,56,32),glass);body.position.y=28;
    const shoulder=new THREE.Mesh(new THREE.ConeGeometry(15.2,38,34,32,true),glass);shoulder.position.y=66;
    const neck=new THREE.Mesh(new THREE.CylinderGeometry(15.2,15.2,40,28),glass);neck.position.y=94;
    this.flask.add(body,shoulder,neck);this.flask.position.set(310,OPENARM_WORKTOP_TOP,-340);this.environment.add(this.flask);
    this.flaskNeck=neck;this.flaskHeight=OPENARM_FLASK_HEIGHT;
    const h=OPENARM_HOTPLATE.height;this.hotplate=box(OPENARM_HOTPLATE.width,h,OPENARM_HOTPLATE.depth,0x31363b,{roughness:.5,metalness:.25});this.hotplate.position.set(OPENARM_HOTPLATE.x,OPENARM_WORKTOP_TOP+h/2,OPENARM_HOTPLATE.z);this.environment.add(this.hotplate);
    const topPlate=box(150,3,150,0x202428,{roughness:.25,metalness:.45});topPlate.position.set(OPENARM_HOTPLATE.x,OPENARM_WORKTOP_TOP+h+1.5,OPENARM_HOTPLATE.z);this.environment.add(topPlate);
  }
  async _ensureRig(){const rig=await this.rigReady;if(!rig)throw new Error(`Canonical ${this.profileId} RoboBuddy visual model is unavailable${this.rigError?`: ${this.rigError.message}`:''}.`);return rig;}
  _updateProfile(){if(!this.rig)return;this.rig.applyPhysicalState(this.state,this.basePose);if(this.profileId==='openarm'){this._syncAttachedFlask();this._evaluateOpenArmContact();}}
  _flaskGripSocketWorld(){if(!this.flask)return null;const p=new THREE.Vector3(0,OPENARM_FLASK_GRIP_Y,0);return this.flask.localToWorld(p);}
  _fingerContact(name,neckCenter,radius=15.2){
    if(!this.rig)return{contact:false,distance:Infinity};const mesh=this.rig.meshes.find(m=>m.name===name);if(!mesh)return{contact:false,distance:Infinity};
    const bounds=new THREE.Box3().setFromObject(mesh);const closest=bounds.clampPoint(neckCenter.clone(),new THREE.Vector3());const distance=closest.distanceTo(neckCenter);return{contact:distance<=radius+0.75,distance,bounds};
  }
  _evaluateOpenArmContact(){
    if(!this.rig||!this.flask){return;}const tool=this.rig.getOpenArmTool('left');const socket=this._flaskGripSocketWorld();if(!tool||!socket)return;
    const toolError=tool.distanceTo(socket);const inner=this._fingerContact('left_finger_inner_mesh',socket);const outer=this._fingerContact('left_finger_outer_mesh',socket);const grip=Number(this.state['left_gripper.pos']??-65);
    const bilateral=inner.contact&&outer.contact&&toolError<=3.0;const pinchValid=bilateral&&grip>=-1.0;
    this.contact={canonicalVisual:'ready',robotId:this.rig.meshData.robotId,toolSocketErrorMm:toolError,innerFingerSurfaceDistanceMm:inner.distance,outerFingerSurfaceDistanceMm:outer.distance,bilateral,pinchValid,attached:this.attached,support:this._supportName()};
  }
  _captureAttachment(){
    if(!this.rig||!this.flask)return;const tool=this.rig.toolFrames.left;if(!tool)return;this.rig.root.updateMatrixWorld(true);this.flask.updateMatrixWorld(true);
    const toolWorld=new THREE.Matrix4().copy(tool.matrixWorld);const inv=toolWorld.clone().invert();const flaskWorld=new THREE.Matrix4().copy(this.flask.matrixWorld);this.attachment={relative:new THREE.Matrix4().multiplyMatrices(inv,flaskWorld)};this.attached=true;this.flaskVy=0;
  }
  _syncAttachedFlask(){
    if(!this.attached||!this.attachment||!this.rig||!this.flask)return;const tool=this.rig.toolFrames.left;if(!tool)return;this.rig.root.updateMatrixWorld(true);const world=new THREE.Matrix4().multiplyMatrices(tool.matrixWorld,this.attachment.relative);world.decompose(this.flask.position,this.flask.quaternion,this.flask.scale);this.flaskVy=0;this.flask.updateMatrixWorld(true);
  }
  _flaskWorldBox(){return this.flask?new THREE.Box3().setFromObject(this.flask):null;}
  _supportName(){
    if(!this.flask)return'none';const b=this._flaskWorldBox();if(!b)return'none';const bottom=b.min.y;const c=b.getCenter(new THREE.Vector3());const hotTop=OPENARM_WORKTOP_TOP+OPENARM_HOTPLATE.height;
    if(Math.abs(bottom-hotTop)<=2.0&&Math.abs(c.x-OPENARM_HOTPLATE.x)<=OPENARM_HOTPLATE.width/2-12&&Math.abs(c.z-OPENARM_HOTPLATE.z)<=OPENARM_HOTPLATE.depth/2-12)return'powered-off hotplate';
    const onLeft=Math.abs(c.x-375)<=178&&Math.abs(c.z+390)<=166;const onRight=Math.abs(c.x-375)<=178&&Math.abs(c.z-390)<=166;if(Math.abs(bottom-OPENARM_WORKTOP_TOP)<=2.0&&(onLeft||onRight))return'worktop';return'none';
  }
  _settleReleasedFlask(dt){
    if(this.profileId!=='openarm'||this.attached||!this.flask)return;const support=this._supportName();if(support!=='none'){this.flaskVy=0;return;}this.flaskVy-=9810*dt;this.flask.position.y+=this.flaskVy*dt;const b=this._flaskWorldBox();if(!b)return;
    const c=b.getCenter(new THREE.Vector3());let top=OPENARM_WORKTOP_TOP;const hotInside=Math.abs(c.x-OPENARM_HOTPLATE.x)<=OPENARM_HOTPLATE.width/2-8&&Math.abs(c.z-OPENARM_HOTPLATE.z)<=OPENARM_HOTPLATE.depth/2-8;if(hotInside)top=OPENARM_WORKTOP_TOP+OPENARM_HOTPLATE.height;
    if(b.min.y<=top){this.flask.position.y+=top-b.min.y;this.flaskVy=0;}
  }
  async _checkTarget(target){
    if(this.profileId!=='openarm')return;const rig=await this._ensureRig();const previous={...this.state};rig.applyPhysicalState(target,this.basePose);const left=rig.getOpenArmTool('left'),right=rig.getOpenArmTool('right');rig.applyPhysicalState(previous,this.basePose);
    for(const [side,p] of [['left',left],['right',right]]){if(!p)continue;const inLeft=Math.abs(p.x-375)<=190&&Math.abs(p.z+390)<=178;const inRight=Math.abs(p.x-375)<=190&&Math.abs(p.z-390)<=178;if((inLeft||inRight)&&p.y<OPENARM_WORKTOP_TOP+4)throw new Error(`${side} tool target would enter the modeled work surface.`);}
    if(this.attached&&left){const flaskBottom=left.y-OPENARM_FLASK_GRIP_Y;const hotTop=OPENARM_WORKTOP_TOP+OPENARM_HOTPLATE.height;const hotInside=Math.abs(left.x-OPENARM_HOTPLATE.x)<=OPENARM_HOTPLATE.width/2&&Math.abs(left.z-OPENARM_HOTPLATE.z)<=OPENARM_HOTPLATE.depth/2;if(hotInside&&flaskBottom<hotTop-.8)throw new Error(`Attached flask target penetrates the powered-off hotplate support by ${(hotTop-flaskBottom).toFixed(2)} mm.`);}
  }
  async applyAction(action,{duration=0.22}={}){
    await this._ensureRig();const before={...this.state},target={...this.state,...action};await this._checkTarget(target);this.actionLog.push({action:{...action},time:performance.now()});const keys=Object.keys(action);const start=performance.now();const ms=Math.max(1,duration*1000);
    return new Promise((resolve,reject)=>{const tick=now=>{try{const t=clamp((now-start)/ms,0,1),ease=t*t*(3-2*t);for(const key of keys)this.state[key]=Number(before[key]??target[key])+(Number(target[key])-Number(before[key]??target[key]))*ease;this._updateProfile();if(this.profileId==='openarm'){
          const g0=Number(before['left_gripper.pos']??-65),g=Number(this.state['left_gripper.pos']??g0),gTarget=Number(target['left_gripper.pos']??g0);const closing=gTarget>g0;
          if(!this.attached&&closing&&this.contact.pinchValid)this._captureAttachment();
          if(this.attached&&g<-30){this.attached=false;this.attachment=null;this.flaskVy=0;this._evaluateOpenArmContact();}
        }
        if(t<1)requestAnimationFrame(tick);else{Object.assign(this.state,target);this._updateProfile();resolve();}}catch(e){reject(e);}};requestAnimationFrame(tick);});
  }
  advanceBase(seconds){
    if(this.profileId!=='lekiwi')return;const forward=(this.state['x.vel']??0)*1000,lateral=(this.state['y.vel']??0)*1000,omega=(this.state['theta.vel']??0)*DEG;const c=Math.cos(this.basePose.yaw),s=Math.sin(this.basePose.yaw);this.basePose.x+=(forward*c-lateral*s)*seconds;this.basePose.z+=(-forward*s-lateral*c)*seconds;this.basePose.yaw+=omega*seconds;this._updateProfile();
  }
  getTelemetry(){const out={...this.state};if(this.profileId==='lekiwi')Object.assign(out,{base_x_mm:this.basePose.x,base_z_mm:this.basePose.z,base_yaw_deg:this.basePose.yaw/DEG});return out;}
  getContacts(){return{...this.contact,visualSource:canonicalVisualProvenance(this.profileId)?.robotId||this.profileId,hardwareValidation:'pending'};}
  _loop(){
    requestAnimationFrame(()=>this._loop());const now=performance.now(),dt=Math.min(.033,Math.max(0,(now-(this.lastTime||now))/1000));this.lastTime=now;this._settleReleasedFlask(dt);const w=Math.max(1,this.canvas.clientWidth),h=Math.max(1,this.canvas.clientHeight);if(this.canvas.width!==Math.round(w*devicePixelRatio)||this.canvas.height!==Math.round(h*devicePixelRatio)){this.renderer.setSize(w,h,false);this.camera.aspect=w/h;this.camera.updateProjectionMatrix();}
    const cp=Math.cos(this.pitch),sp=Math.sin(this.pitch),cy=Math.cos(this.yaw),sy=Math.sin(this.yaw);this.camera.position.set(this.cameraTarget.x+this.distance*cp*cy,this.cameraTarget.y+this.distance*sp,this.cameraTarget.z+this.distance*cp*sy);this.camera.lookAt(this.cameraTarget);this.renderer.render(this.scene,this.camera);
  }
}
