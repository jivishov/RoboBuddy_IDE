/** Pure, deterministic construction operations; no eval, URLs, plugins or state writes. */
export const IDENTITY = Object.freeze([1, 0, 0, 0]);
export const GEOMETRY_LIMITS = Object.freeze({ parts: 32, componentsPerObject: 160, components: 512, vertices: 128, triangles: 256, visualVertices: 2048, visualTriangles: 4096 });
export const SHAPES = Object.freeze(['box', 'sphere', 'cylinder', 'frustum', 'hollow_profile', 'extrusion', 'convex_mesh']);
export function object(value, allowed, label) {
  if (!value || typeof value !== 'object' || Array.isArray(value) || ![Object.prototype, null].includes(Object.getPrototypeOf(value))) throw new TypeError(`${label} must be a plain object`);
  for (const key of Object.keys(value)) { if (!allowed.includes(key)) throw new TypeError(`${label}: unsupported field ${key}`); if(value[key]===null)throw new TypeError(`${label}.${key}: omit optional fields instead of null`); }
  return value;
}
export function number(v, lo, hi, label) {
  if (typeof v !== 'number' || !Number.isFinite(v) || v < lo || v > hi) throw new RangeError(`${label} must be ${lo}..${hi}`);
  return v;
}
export function integer(v, lo, hi, label) { number(v, lo, hi, label); if (!Number.isInteger(v)) throw new TypeError(`${label} must be an integer`); return v; }
export function vector(v, n, label, lo = -3, hi = 3) {
  if (!Array.isArray(v) || v.length !== n) throw new TypeError(`${label} must contain ${n} numbers`);
  return v.map(x => number(x, lo, hi, label));
}
export function quaternion(v = IDENTITY) {
  const q = vector(v, 4, 'quaternion_wxyz', -1, 1);
  if (Math.abs(Math.hypot(...q) - 1) > 1e-6) throw new RangeError('quaternion_wxyz must be normalized (WXYZ)');
  return q;
}
export function identifier(v, label = 'id') {
  if (typeof v !== 'string' || !/^[a-z][a-z0-9_]{0,23}$/.test(v) || ['constructor', 'prototype', '__proto__'].includes(v)) throw new TypeError(`${label} must be a safe lowercase identifier, 1..24 characters`);
  return v;
}
export const add = (a, b) => a.map((v, i) => v + b[i]);
export const sub = (a, b) => a.map((v, i) => v - b[i]);
export const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
export const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
export function rotate(q, p) {
  const [w,x,y,z] = q, [a,b,c] = p;
  return [(1-2*(y*y+z*z))*a+2*(x*y-w*z)*b+2*(x*z+w*y)*c,
    2*(x*y+w*z)*a+(1-2*(x*x+z*z))*b+2*(y*z-w*x)*c,
    2*(x*z-w*y)*a+2*(y*z+w*x)*b+(1-2*(x*x+y*y))*c];
}
export const transform = (p, position = [0,0,0], q = IDENTITY) => add(position, rotate(q, p));
export const inverseRotate = (q, p) => rotate([q[0],-q[1],-q[2],-q[3]],p);

/** Reject concavity instead of silently filling cavities with MuJoCo's convex hull.
 * Check finite/index bounds, outward halfspaces, closed oriented edges, connectivity,
 * non-degenerate volume, and use of every vertex. Visual-only meshes skip convexity.
 */
export function validateMesh(raw, { visual = false } = {}) {
  object(raw, ['vertices_m', 'triangles'], 'mesh');
  const maxV = visual ? GEOMETRY_LIMITS.visualVertices : GEOMETRY_LIMITS.vertices;
  const maxT = visual ? GEOMETRY_LIMITS.visualTriangles : GEOMETRY_LIMITS.triangles;
  if (!Array.isArray(raw.vertices_m) || raw.vertices_m.length < 4 || raw.vertices_m.length > maxV) throw new RangeError(`mesh vertices: 4..${maxV}`);
  if (!Array.isArray(raw.triangles) || raw.triangles.length < 4 || raw.triangles.length > maxT) throw new RangeError(`mesh triangles: 4..${maxT}`);
  const vertices = raw.vertices_m.map(p => vector(p, 3, 'mesh vertex'));
  const triangles = raw.triangles.map(t => {
    if (!Array.isArray(t) || t.length !== 3) throw new TypeError('mesh triangle must have three indices');
    return t.map(i => integer(i, 0, vertices.length-1, 'mesh index'));
  });
  const edges = new Map(), used = new Set(), seenFaces = new Set(); let volume6 = 0;
  const center = vertices.reduce((s,v)=>add(s,v),[0,0,0]).map(x=>x/vertices.length);
  for (let i=0; i<triangles.length; i++) {
    const t = triangles[i], faceKey = [...t].sort((a,b)=>a-b).join(',');
    if (seenFaces.has(faceKey)) throw new TypeError('mesh contains duplicate triangles'); seenFaces.add(faceKey);
    const [a,b,c] = t.map(j=>vertices[j]), n = cross(sub(b,a),sub(c,a)), length = Math.hypot(...n);
    if (length < 1e-12) throw new RangeError('mesh contains degenerate triangles');
    volume6 += dot(sub(a,center), cross(sub(b,center),sub(c,center)));
    for (const j of t) used.add(j);
    if (!visual && vertices.some(v=>dot(n,sub(v,a))/length>1e-8)) throw new RangeError('Collision mesh must be convex with outward winding; decompose concave objects into closed convex components. No automatic hull filling.');
    for (let j=0;j<3;j++) {
      const a=t[j], b=t[(j+1)%3], key=[Math.min(a,b),Math.max(a,b)].join(',');
      const list=edges.get(key)||[];list.push({a,b,face:i});edges.set(key,list);
    }
  }
  if (used.size !== vertices.length) throw new TypeError('mesh has unused vertices');
  if (!visual) {
    const neighbors=triangles.map(()=>[]);
    for (const list of edges.values()) {
      if (list.length!==2 || list[0].a!==list[1].b) throw new TypeError('Collision mesh must be closed and consistently oriented');
      neighbors[list[0].face].push(list[1].face);neighbors[list[1].face].push(list[0].face);
    }
    const visited=new Set([0]),queue=[0];
    while(queue.length) for(const n of neighbors[queue.pop()]) if(!visited.has(n)){visited.add(n);queue.push(n);}
    if(visited.size!==triangles.length) throw new TypeError('Collision component must be connected');
    if(volume6/6<1e-11) throw new RangeError('Collision mesh has non-positive or insufficient volume');
  }
  return { vertices_m: vertices, triangles, volume: volume6/6 };
}
function triangulateFaces(vertices, faces) {
  // Deduplicate the axial vertices of a solid frustum/cone, removing collapsed faces.
  const unique=[],map=[],keys=new Map();
  for(const v of vertices){const k=v.map(x=>Math.abs(x)<1e-15?0:x).join(',');if(!keys.has(k)){keys.set(k,unique.length);unique.push(v);}map.push(keys.get(k));}
  const triangles=[];
  for(const f of faces) for(let i=1;i<f.length-1;i++){
    const t=[map[f[0]],map[f[i]],map[f[i+1]]];
    if(new Set(t).size===3 && Math.hypot(...cross(sub(unique[t[1]],unique[t[0]]),sub(unique[t[2]],unique[t[0]])))>1e-12) triangles.push(t);
  }
  return validateMesh({vertices_m:unique,triangles});
}
function sector(z0,z1,ri0,ro0,ri1,ro1,a0,a1){
  const v=[];
  for(const [z,ri,ro] of [[z0,ri0,ro0],[z1,ri1,ro1]]) for(const [r,a] of [[ri,a0],[ro,a0],[ro,a1],[ri,a1]]) v.push([r*Math.cos(a),r*Math.sin(a),z]);
  return triangulateFaces(v,[[0,3,2,1],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]);
}
export function normalizePart(raw) {
  object(raw,['id','shape','position_m','quaternion_wxyz','dimensions_m','radius_m','height_m','bottom_radius_m','top_radius_m','profile','segments','polygon_m','vertices_m','triangles','grid','rgba','density_kg_m3'],'part');
  const id=identifier(raw.id,'part.id'),shape=raw.shape;
  if(!SHAPES.includes(shape)) throw new TypeError(`Unsupported construction shape ${String(shape)}`);
  const fields={box:['dimensions_m'],sphere:['radius_m'],cylinder:['radius_m','height_m'],frustum:['bottom_radius_m','top_radius_m','height_m','segments'],hollow_profile:['profile','segments'],extrusion:['polygon_m','height_m'],convex_mesh:['vertices_m','triangles']}[shape];
  const common=['id','shape','position_m','quaternion_wxyz','grid','rgba','density_kg_m3'];
  for(const key of Object.keys(raw)) if(!fields.includes(key)&&!common.includes(key)) throw new TypeError(`${shape} does not accept ${key}`);
  const p={id,shape,position_m:vector(raw.position_m??[0,0,0],3,'part.position_m'),quaternion_wxyz:quaternion(raw.quaternion_wxyz),rgba:vector(raw.rgba??[.48,.64,.70,1],4,'rgba',0,1),density_kg_m3:number(raw.density_kg_m3??1000,1,20000,'density_kg_m3')};
  if(p.rgba[3]<.1)throw new RangeError('Physical components must remain visible (alpha >= 0.1)');
  if(shape==='box')p.dimensions_m=vector(raw.dimensions_m,3,'part dimensions',.002,2);
  if(['sphere','cylinder'].includes(shape))p.radius_m=number(raw.radius_m,.001,1,'radius_m');
  if(['cylinder','frustum','extrusion'].includes(shape))p.height_m=number(raw.height_m,.002,2,'height_m');
  if(shape==='frustum'){
    p.bottom_radius_m=number(raw.bottom_radius_m,.001,1,'bottom_radius_m');p.top_radius_m=number(raw.top_radius_m,0,1,'top_radius_m');
  }
  if(['hollow_profile','frustum'].includes(shape))p.segments=integer(raw.segments??16,8,32,'segments');
  if(shape==='hollow_profile'){
    if(!Array.isArray(raw.profile)||raw.profile.length<2||raw.profile.length>8)throw new RangeError('profile needs 2..8 [height, inner radius, outer radius] stations');
    p.profile=raw.profile.map(v=>vector(v,3,'profile station',0,2));
    p.profile.forEach(([z,ri,ro],i)=>{
      if(ri<.001||ro-ri<.001||ro>1||i&&z-p.profile[i-1][0]<.002)throw new RangeError('Profile stations must increase in height by >=2 mm; radii and walls >=1 mm');
    });
    if(p.profile[0][0]!==0)throw new RangeError('Hollow profile starts at local z=0');
  }
  if(shape==='extrusion'){
    if(!Array.isArray(raw.polygon_m)||raw.polygon_m.length<3||raw.polygon_m.length>16)throw new RangeError('extrusion polygon: 3..16 counterclockwise vertices');
    p.polygon_m=raw.polygon_m.map(v=>vector(v,2,'polygon vertex',-1,1));
    // Require a strictly convex CCW polygon. Concave profiles must be decomposed.
    for(let i=0;i<p.polygon_m.length;i++){
      const a=p.polygon_m[i],b=p.polygon_m[(i+1)%p.polygon_m.length];
      for(let j=0;j<p.polygon_m.length;j++)if(j!==i&&j!==(i+1)%p.polygon_m.length){const c=p.polygon_m[j];if((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])<=1e-10)throw new RangeError('Extrusion polygon must be strictly convex and counterclockwise');}
    }
  }
  if(shape==='convex_mesh'){const m=validateMesh({vertices_m:raw.vertices_m,triangles:raw.triangles});p.vertices_m=m.vertices_m;p.triangles=m.triangles;}
  if(raw.grid!==undefined){
    object(raw.grid,['counts','spacing_m'],'grid');
    const counts=vector(raw.grid.counts,3,'grid counts',1,12).map(v=>integer(v,1,12,'grid count'));
    const spacing=vector(raw.grid.spacing_m,3,'grid spacing',-2,2);
    if(counts.reduce((a,b)=>a*b,1)>64||counts.some((n,i)=>n>1&&Math.abs(spacing[i])<.002))throw new RangeError('grid: at most 64 instances, nonzero spacing on repeated axes');
    p.grid={counts,spacing_m:spacing};
  }
  return p;
}
export function partComponents(p){
  const items=[];
  const mesh=m=>items.push({type:'mesh',positionM:[0,0,0],quaternionWxyz:IDENTITY,sizeM:[],meshData:m,volume:m.volume});
  if(p.shape==='box')items.push({type:'box',positionM:[0,0,0],sizeM:p.dimensions_m.map(x=>x/2),volume:p.dimensions_m.reduce((a,b)=>a*b,1)});
  if(p.shape==='sphere')items.push({type:'sphere',positionM:[0,0,0],sizeM:[p.radius_m],volume:4*Math.PI*p.radius_m**3/3});
  if(p.shape==='cylinder')items.push({type:'cylinder',positionM:[0,0,p.height_m/2],sizeM:[p.radius_m,p.height_m/2],volume:Math.PI*p.radius_m**2*p.height_m});
  if(p.shape==='frustum'){
    const n=p.segments,v=[];
    for(const [z,r] of [[0,p.bottom_radius_m],[p.height_m,p.top_radius_m]])for(let k=0;k<n;k++)v.push([r*Math.cos(2*Math.PI*k/n),r*Math.sin(2*Math.PI*k/n),z]);
    const faces=[Array.from({length:n},(_,i)=>n-1-i),Array.from({length:n},(_,i)=>n+i)];
    for(let i=0;i<n;i++)faces.push([i,(i+1)%n,(i+1)%n+n,i+n]);
    mesh(triangulateFaces(v,faces));
  }
  if(p.shape==='hollow_profile'){
    const stations=p.profile;
    for(let j=1;j<stations.length;j++)for(let k=0;k<p.segments;k++){
      const a=stations[j-1],b=stations[j];mesh(sector(a[0],b[0],a[1],a[2],b[1],b[2],2*Math.PI*k/p.segments,2*Math.PI*(k+1)/p.segments));
    }
  }
  if(p.shape==='convex_mesh')mesh(validateMesh({vertices_m:p.vertices_m,triangles:p.triangles}));
  if(p.shape==='extrusion'){
    const n=p.polygon_m.length,v=[...p.polygon_m.map(([x,y])=>[x,y,0]),...p.polygon_m.map(([x,y])=>[x,y,p.height_m])];
    const faces=[Array.from({length:n},(_,i)=>n-1-i),Array.from({length:n},(_,i)=>n+i)];
    for(let i=0;i<n;i++)faces.push([i,(i+1)%n,(i+1)%n+n,i+n]);mesh(triangulateFaces(v,faces));
  }
  const counts=p.grid?.counts??[1,1,1],spacing=p.grid?.spacing_m??[0,0,0];
  if(items.length*counts.reduce((a,b)=>a*b,1)>GEOMETRY_LIMITS.componentsPerObject)throw new RangeError('Construction operation exceeds per-object collision budget');
  const all=[];
  for(let x=0;x<counts[0];x++)for(let y=0;y<counts[1];y++)for(let z=0;z<counts[2];z++)for(const item of items){
    const offset=add(p.position_m,[x*spacing[0],y*spacing[1],z*spacing[2]]);
    all.push({...item,positionM:transform(item.positionM,offset,p.quaternion_wxyz),quaternionWxyz:p.quaternion_wxyz,rgba:p.rgba,density:p.density_kg_m3,partId:p.id});
  }
  return all;
}
/** Conservative local bounds, including rotated primitives and actual mesh vertices. */
export function componentPoints(c){
  if(c.meshData)return c.meshData.vertices_m.map(p=>transform(p,c.positionM,c.quaternionWxyz));
  const size=c.type==='box'?c.sizeM:c.type==='sphere'?[c.sizeM[0],c.sizeM[0],c.sizeM[0]]:[c.sizeM[0],c.sizeM[0],c.sizeM[1]];
  const points=[];for(const x of [-size[0],size[0]])for(const y of [-size[1],size[1]])for(const z of [-size[2],size[2]])points.push(transform([x,y,z],c.positionM,c.quaternionWxyz));
  return points;
}
export function bounds(points){return {min:[0,1,2].map(i=>Math.min(...points.map(p=>p[i]))),max:[0,1,2].map(i=>Math.max(...points.map(p=>p[i])))};}
export const multiplyQuaternion = (a,b) => [a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]];
