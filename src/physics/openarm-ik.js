// Bounded kinematic planning on SEPARATE MjData. The physical plant is never
// assigned the IK solution: only its finite, bounded actuator targets are used.
const slice = (v, i, n) => Array.from(v.slice(i, i+n));
const norm = v => Math.hypot(...v);
function solve(matrix, rhs) {
  const a = matrix.map((r,i) => [...r,rhs[i]]), n = rhs.length;
  for (let i=0;i<n;i++) {
    let p=i;for(let j=i+1;j<n;j++)if(Math.abs(a[j][i])>Math.abs(a[p][i]))p=j;
    if(Math.abs(a[p][i])<1e-14)throw new Error('Singular IK system');
    [a[i],a[p]]=[a[p],a[i]]; const v=a[i][i];for(let k=i;k<=n;k++)a[i][k]/=v;
    for(let j=0;j<n;j++)if(j!==i){const f=a[j][i];for(let k=i;k<=n;k++)a[j][k]-=f*a[i][k];}
  }
  return a.map(r=>r[n]);
}
function rotationError(target, current) {
  const [w,x,y,z]=target,[a,b,c,d]=current;
  let q=[w*a+x*b+y*c+z*d,-w*b+x*a-y*d+z*c,-w*c+x*d+y*a-z*b,-w*d-x*c+y*b+z*a];
  if(q[0]<0)q=q.map(v=>-v);
  const len=norm(q.slice(1));if(len<1e-12)return [0,0,0];
  const scale=2*Math.atan2(len,q[0])/len;return q.slice(1).map(v=>v*scale);
}
export function solveOpenArmIK(mj, model, plant, jointState, request) {
  if(!['left','right'].includes(request.side))throw new TypeError('IK side must be left or right');
  const target=request.positionM;
  if(!Array.isArray(target)||target.length!==3||target.some(x=>typeof x!=='number'||!Number.isFinite(x)))throw new TypeError('IK positionM requires finite metres');
  if(target[0]<-.3||target[0]>.95||Math.abs(target[1])>.75||target[2]<.8||target[2]>1.9)throw new RangeError('IK target is outside the bounded workcell');
  let orientation=request.quaternionWxyz??null;
  if(orientation){if(!Array.isArray(orientation)||orientation.length!==4||orientation.some(x=>typeof x!=='number'||!Number.isFinite(x))||Math.abs(norm(orientation)-1)>.001)throw new TypeError('IK quaternion must be normalized WXYZ');orientation=orientation.map(x=>x/norm(orientation));}
  const enumCode=n=>Number(mj.mjtObj[n].value??mj.mjtObj[n]);
  const site=Number(mj.mj_name2id(model,enumCode('mjOBJ_SITE'),`${request.side}_pinch_reference`));
  const body=Number(mj.mj_name2id(model,enumCode('mjOBJ_BODY'),`openarm_${request.side}_ee_base_link`));
  if(site<0||body<0)throw new Error('Physical pinch reference is unavailable');
  const joints=Array.from({length:7},(_,i)=>({name:`openarm_${request.side}_joint${i+1}`, ...jointState.get(`openarm_${request.side}_joint${i+1}`)}));
  const scratch=new mj.MjData(model);
  try {
    scratch.qpos.set(plant.qpos);let iteration=0, ep,er;
    for(;iteration<120;iteration++){
      mj.mj_kinematics(model,scratch);
      const pos=slice(scratch.site_xpos,site*3,3);
      ep=target.map((v,i)=>v-pos[i]);er=orientation?rotationError(orientation,slice(scratch.xquat,body*4,4)):[];
      if(norm(ep)<.0015&&(!orientation||norm(er)<.025))break;
      const error=[...ep,...er.map(v=>v*.08)];
      const columns=joints.map(j=>{
        const ax=slice(scratch.xaxis,j.id*3,3), anchor=slice(scratch.xanchor,j.id*3,3),r=pos.map((v,i)=>v-anchor[i]);
        return [ax[1]*r[2]-ax[2]*r[1],ax[2]*r[0]-ax[0]*r[2],ax[0]*r[1]-ax[1]*r[0],...(orientation?ax.map(v=>v*.08):[])];
      });
      const matrix=error.map((_,i)=>error.map((_,k)=>columns.reduce((s,c)=>s+c[i]*c[k],0)+(i===k?.000025:0)));
      const solution=solve(matrix,error);
      joints.forEach((j,k)=>{const step=columns[k].reduce((s,v,i)=>s+v*solution[i],0);scratch.qpos[j.qpos]=Math.max(j.range[0]+1e-5,Math.min(j.range[1]-1e-5,scratch.qpos[j.qpos]+Math.max(-.10,Math.min(.10,step))));});
    }
    // Re-evaluate the FINAL iterate, including a limit-clamped final update.
    mj.mj_kinematics(model,scratch);
    ep=target.map((v,i)=>v-Number(scratch.site_xpos[site*3+i]));
    er=orientation?rotationError(orientation,slice(scratch.xquat,body*4,4)):[];
    if(norm(ep)>.0015||(orientation&&norm(er)>.025))throw new RangeError(`IK did not converge: position residual ${norm(ep).toFixed(5)} m${orientation?`, orientation ${norm(er).toFixed(4)} rad`:''}. No physical target was changed.`);
    return { targetsRad:Object.fromEntries(joints.map(j=>[j.name,Number(scratch.qpos[j.qpos])])), positionResidualM:norm(ep), orientationResidualRad:orientation?norm(er):null, iterations:iteration, frame:'mujoco_world', point:`${request.side}_pinch_reference`, collisionFreePath:false };
  } finally {scratch.delete();}
}
