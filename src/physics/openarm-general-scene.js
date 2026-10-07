import { OPENARM_LAB_ASSETS, compileLabAsset, validateLabAsset } from './openarm-lab-assets.js';
import { workcellBaseXml } from './openarm-workcell-scene.js';
import { IDENTITY, GEOMETRY_LIMITS, object, number, vector, quaternion, identifier, normalizePart, partComponents, componentPoints, bounds, validateMesh, transform, add, rotate } from './openarm-general-geometry.js';

export const GENERAL_SCENE_VERSION = 'robobuddy.lab.scene.v2';
export const GENERAL_LIMITS = Object.freeze({ objects:24, inputBytes:512000, inventory:64, ports:16, ...GEOMETRY_LIMITS, worldMin:[-.7,-1.2,0], worldMax:[1.7,1.2,2.2] });
export const QUANTITY_SOURCES = Object.freeze(['source_provided','user_measured','image_estimated','fitted','assumed']);
export function text(v,label,max=500){if(typeof v!=='string'||!v.trim()||v.length>max||/[\x00-\x1f]/.test(v))throw new TypeError(`${label} must be printable text, 1..${max} characters`);return v.trim();}
function list(v,max,label){if(!Array.isArray(v)||v.length>max)throw new RangeError(`${label}: 0..${max} entries`);return v;}
function enumValue(v,values,label){if(!values.includes(v))throw new TypeError(`${label} must be ${values.join(', ')}`);return v;}
function evidence(raw={source:'assumed',source_detail:'Not supplied; illustrative default.'},label='evidence'){
  object(raw,['source','source_detail','uncertainty_m','relative_uncertainty'],label);
  const out={source:enumValue(raw.source,QUANTITY_SOURCES,`${label}.source`),source_detail:text(raw.source_detail??'Not supplied; illustrative default.',`${label}.source_detail`)};
  if(raw.uncertainty_m!==undefined)out.uncertainty_m=number(raw.uncertainty_m,0,1,'uncertainty_m');
  if(raw.relative_uncertainty!==undefined)out.relative_uncertainty=number(raw.relative_uncertainty,0,10,'relative_uncertainty');
  return out;
}
function reference(raw={mode:'none'}){
  object(raw,['mode','label','dimensions','notes','agent_image_access'],'reference');
  const mode=enumValue(raw.mode,['none','external_reference','local_file','synthetic_fixture'],'reference.mode');
  const out={mode,label:text(raw.label??(mode==='none'?'No image reference':'Author-supplied reference'),'reference.label',160),agent_image_access:enumValue(raw.agent_image_access??'not_verified',['not_verified','declared_by_agent'],'agent_image_access'),notes:text(raw.notes??'The application does not independently interpret or verify reference-image pixels.','reference.notes',1000)};
  if(raw.dimensions!==undefined){
    object(raw.dimensions,['known_length_m','description','source'],'reference.dimensions');
    out.dimensions={known_length_m:number(raw.dimensions.known_length_m,.001,10,'known_length_m'),description:text(raw.dimensions.description,'dimension description'),source:enumValue(raw.dimensions.source,QUANTITY_SOURCES,'dimension source')};
  }
  return out;
}
function normalizeObject(raw){
  object(raw,['id','label','role','position_m','quaternion_wxyz','motion','fixed_reason','mass_kg','friction','quantity_evidence','parts','catalog','ports','appearance','visual_mesh','limitations'],'scene object');
  const e={id:identifier(raw.id),label:text(raw.label??raw.id,'object label',100),role:enumValue(raw.role??'equipment',['bench','equipment','fixture','obstacle'],'object role'),position_m:vector(raw.position_m,3,'object position'),quaternion_wxyz:quaternion(raw.quaternion_wxyz),motion:enumValue(raw.motion??'free',['free','fixed'],'motion')};
  if(e.motion==='fixed')e.fixed_reason=text(raw.fixed_reason,'fixed_reason (declared mounting, not contact-derived support)');
  else if(raw.fixed_reason!==undefined)throw new TypeError('fixed_reason is only valid for a fixed object');
  if(e.role==='bench'&&e.motion!=='fixed')throw new TypeError('A bench must explicitly declare its fixed support boundary');
  const qe=raw.quantity_evidence??{};object(qe,['geometry','pose','mass','friction'],'quantity_evidence');
  e.quantity_evidence=Object.fromEntries(['geometry','pose','mass','friction'].map(k=>[k,evidence(qe[k],`quantity_evidence.${k}`)]));
  // Do not label a compiler-supplied default as a sourced or measured value.
  for (const [quantity, field] of [['mass','mass_kg'], ['friction','friction']]) {
    if (raw[field] === undefined && qe[quantity] && qe[quantity].source !== 'assumed')
      throw new TypeError(`${quantity} provenance requires an explicit ${field}; omitted values are assumed defaults`);
  }
  if (raw.catalog && raw.catalog.dimensions_m === undefined && qe.geometry && qe.geometry.source !== 'assumed')
    throw new TypeError('Sourced catalog geometry requires explicit dimensions_m; catalog defaults are assumed');
  e.mass_kg=number(raw.mass_kg??(e.motion==='free'?.05:1),.005,e.motion==='free'?2:100,'mass_kg (not a robot payload rating)');
  e.friction=vector(raw.friction??[.8,.005,.0005],3,'friction',0,2);
  if(e.friction[1]>.05||e.friction[2]>.01)throw new RangeError('Torsional/rolling friction bounds exceeded');
  e.limitations=list(raw.limitations??[],16,'object limitations').map(s=>text(s,'limitation'));
  if((raw.parts!==undefined)===(raw.catalog!==undefined))throw new TypeError('Each object needs exactly one of parts or catalog');
  if(raw.catalog!==undefined){
    object(raw.catalog,['kind','dimensions_m'],'catalog');
    if(!Object.hasOwn(OPENARM_LAB_ASSETS,raw.catalog.kind))throw new TypeError('Unknown catalog kind; use parts to construct novel equipment');
    const dimensions=vector(raw.catalog.dimensions_m??OPENARM_LAB_ASSETS[raw.catalog.kind].dimensions_m,3,'catalog dimensions',.004,2);
    validateLabAsset({kind:raw.catalog.kind,dimensions_m:dimensions,dynamic:e.motion==='free'});
    e.catalog={kind:raw.catalog.kind,dimensions_m:dimensions};
  } else {
    e.parts=list(raw.parts,GENERAL_LIMITS.parts,'object parts').map(normalizePart);
    if(!e.parts.length)throw new TypeError('An object must have physical geometry');
    if(new Set(e.parts.map(p=>p.id)).size!==e.parts.length)throw new TypeError('Part IDs must be unique within an object');
  }
  e.ports=list(raw.ports??[],GENERAL_LIMITS.ports,'ports').map(p=>{
    object(p,['id','type','part_id','position_m'],'port');
    const port={id:identifier(p.id),type:enumValue(p.type,['grasp','support','opening','peg'],'port.type')};
    if(port.type==='grasp'){if(p.part_id!==undefined)throw new TypeError('A grasp candidate uses position_m, not part_id');port.position_m=vector(p.position_m,3,'candidate grasp position');}
    else { port.part_id=identifier(p.part_id,'port.part_id'); if(p.position_m!==undefined)throw new TypeError('Physical feature coordinates are derived from the referenced part, not supplied by the caller'); }
    if(e.catalog&&port.type!=='grasp')throw new TypeError('Authored task ports require construction parts; catalog interactions retain their legacy affordances');
    return port;
  });
  if(new Set(e.ports.map(p=>p.id)).size!==e.ports.length)throw new TypeError('Port IDs must be unique');
  if(raw.appearance!==undefined){
    object(raw.appearance,['rgba','roughness','metalness'],'appearance');
    e.appearance={rgba:vector(raw.appearance.rgba??[.5,.65,.7,1],4,'appearance rgba',0,1),roughness:number(raw.appearance.roughness??.55,0,1,'roughness'),metalness:number(raw.appearance.metalness??.05,0,1,'metalness')};
    if(e.appearance.rgba[3]<.1)throw new RangeError('Physical objects may not be invisible');
  }
  if(raw.visual_mesh!==undefined){const m=validateMesh(raw.visual_mesh,{visual:true});e.visual_mesh={vertices_m:m.vertices_m,triangles:m.triangles};}
  return e;
}
export function normalizeGeneralScene(raw){
  object(raw,['schema_version','id','reference','objects','inventory','assumptions','adjustments','relationships'],'SceneSpec');
  if(raw.schema_version!==GENERAL_SCENE_VERSION)throw new TypeError(`SceneSpec requires ${GENERAL_SCENE_VERSION}`);
  if(new TextEncoder().encode(JSON.stringify(raw)).length>GENERAL_LIMITS.inputBytes)throw new RangeError('SceneSpec exceeds 512 kB input limit');
  const scene={schema_version:GENERAL_SCENE_VERSION,id:identifier(raw.id,'scene id'),reference:reference(raw.reference),objects:list(raw.objects,GENERAL_LIMITS.objects,'objects').map(normalizeObject)};
  const ids=new Set(scene.objects.map(e=>e.id));if(ids.size!==scene.objects.length)throw new TypeError('Object IDs must be unique');
  scene.inventory=list(raw.inventory??[],GENERAL_LIMITS.inventory,'inventory').map(row=>{
    object(row,['id','label','status','object_ids','reason','image_bbox_uv'],'inventory item');
    const r={id:identifier(row.id),label:text(row.label??row.id,'inventory label',100),status:enumValue(row.status,['represented','approximated','unresolved','supplemental'],'inventory status'),object_ids:list(row.object_ids??[],24,'inventory object_ids').map(v=>identifier(v)),reason:text(row.reason,'inventory reason')};
    if(r.object_ids.some(id=>!ids.has(id)))throw new TypeError('Inventory references an unknown scene object');
    if((r.status==='unresolved')!==(r.object_ids.length===0))throw new TypeError('Unresolved items have no object_ids; other inventory entries must reference geometry');
    if(row.image_bbox_uv!==undefined){r.image_bbox_uv=vector(row.image_bbox_uv,4,'image_bbox_uv',0,1);if(r.image_bbox_uv[2]<=r.image_bbox_uv[0]||r.image_bbox_uv[3]<=r.image_bbox_uv[1])throw new RangeError('Image bbox must be [left,top,right,bottom]');}
    return r;
  });
  if(new Set(scene.inventory.map(r=>r.id)).size!==scene.inventory.length)throw new TypeError('Inventory IDs must be unique');
  if(scene.reference.mode!=='none'){
    if(!scene.inventory.length)throw new TypeError('Image-referenced scenes require an explicit source inventory');
    const covered=new Set(scene.inventory.flatMap(r=>r.object_ids));
    if(scene.objects.some(e=>!covered.has(e.id)))throw new TypeError('Every object must map to source inventory or an explicit supplemental item');
  }
  scene.assumptions=list(raw.assumptions??[],32,'assumptions').map(s=>text(s,'assumption'));
  scene.adjustments=list(raw.adjustments??[],32,'adjustments').map(r=>{
    object(r,['object_id','description','reason'],'adjustment');if(!ids.has(r.object_id))throw new TypeError('Adjustment object is unknown');
    return {object_id:r.object_id,description:text(r.description,'adjustment description'),reason:text(r.reason,'adjustment reason')};
  });
  scene.relationships=list(raw.relationships??[],48,'relationships').map(r=>{
    object(r,['subject_id','relation','object_id','source','notes'],'relationship');
    if(!ids.has(r.subject_id)||!ids.has(r.object_id)||r.subject_id===r.object_id)throw new TypeError('Relationship requires two different declared objects');
    return {subject_id:r.subject_id,relation:enumValue(r.relation,['supported_by','inserted_into','near','attached_to'],'relation'),object_id:r.object_id,source:enumValue(r.source,QUANTITY_SOURCES,'relationship source'),notes:text(r.notes??'Descriptive relationship only; does not weld or move objects.','relationship notes')};
  });
  if(new TextEncoder().encode(JSON.stringify(scene)).length>GENERAL_LIMITS.inputBytes)throw new RangeError('Normalized SceneSpec exceeds the 512 kB budget');
  return scene;
}
function derivedPorts(e,localBounds){
  return e.ports.map(port=>{
    if(port.type==='grasp'){
      if(port.position_m.some((x,i)=>x<localBounds.min[i]-.005||x>localBounds.max[i]+.005))throw new RangeError('Grasp candidate must lie within the object envelope (+5 mm)');
      return {...port,positionM:port.position_m,quaternionWxyz:IDENTITY,geometryEvidence:'author-defined candidate; graspability not verified'};
    }
    const p=e.parts.find(p=>p.id===port.part_id);if(!p)throw new TypeError(`Unknown feature part ${port.part_id}`);
    if(p.grid)throw new TypeError('A task port must reference a non-repeated part; author an explicit target instance');
    let pos,feature;
    if(port.type==='support'){
      if(p.shape==='box'){pos=[0,0,p.dimensions_m[2]/2];feature={halfExtentsM:p.dimensions_m.slice(0,2).map(x=>x/2)};}
      else if(p.shape==='cylinder'){pos=[0,0,p.height_m];feature={radiusM:p.radius_m};}
      else throw new TypeError('Support ports currently require a box or cylinder top surface');
    }else if(port.type==='opening'){
      if(p.shape!=='hollow_profile')throw new TypeError('Opening ports require a hollow_profile; imported cavities are not automatically certified');
      const height=p.profile.at(-1)[0];pos=[0,0,height];feature={radiusM:Math.min(...p.profile.map(s=>s[1]))*Math.cos(Math.PI/p.segments),depthM:height,profileStations:p.profile,segments:p.segments};
    }else{
      if(p.shape==='cylinder')feature={radiusM:p.radius_m,lengthM:p.height_m};
      else if(p.shape==='hollow_profile'&&p.profile.every(s=>Math.abs(s[2]-p.profile[0][2])<1e-9))feature={radiusM:p.profile[0][2],lengthM:p.profile.at(-1)[0]};
      else throw new TypeError('Peg ports require a cylinder or a hollow profile with constant outer radius');
      pos=[0,0,0];
    }
    return {...port,...feature,positionM:transform(pos,p.position_m,p.quaternion_wxyz),quaternionWxyz:p.quaternion_wxyz,axisM:rotate(p.quaternion_wxyz,[0,0,1]),geometryEvidence:'derived from construction geometry; not a measured real-world feature'};
  });
}
const join=v=>v.join(' ');
export function compileGeneralScene(raw){
  const spec=normalizeGeneralScene(raw),geoms=[],bodies=[],records=[],meshes={},visualGeoms=[],warnings=[];let xml='',assetXml='';
  for(const e of spec.objects){
    const bodyId=`lab_${e.id}`,free=e.motion==='free';let compiled,points,components;
    if(e.catalog){
      compiled=compileLabAsset({id:e.id,label:e.label,kind:e.catalog.kind,position_m:e.position_m,quaternion_wxyz:e.quaternion_wxyz,dimensions_m:e.catalog.dimensions_m,mass_kg:e.mass_kg,dynamic:free});
      compiled.xml=compiled.xml.replaceAll('friction="0.8 .005 .0005"',`friction="${join(e.friction)}"`);
      points=compiled.geoms.flatMap(g=>componentPoints({...g,meshData:g.mesh?{vertices_m:Array.from({length:compiled.meshes[g.mesh].vertices.length/3},(_,i)=>compiled.meshes[g.mesh].vertices.slice(i*3,i*3+3))}:null}));
    }else{
      components=e.parts.flatMap(partComponents);
      if(components.length>GENERAL_LIMITS.componentsPerObject)throw new RangeError(`${e.id} exceeds 160 collision components`);
      points=components.flatMap(componentPoints);
      const total=components.reduce((s,c)=>s+c.volume*c.density,0);
      if(!(total>0))throw new RangeError('Object has no positive physical volume');
      compiled={geoms:[],meshes:{},bodies:[{id:bodyId,...(free?{freeJointId:`${bodyId}_free`}:{})}],assetXml:'',records:[]};
      const geomXml=components.map((c,i)=>{
        const id=`${bodyId}_c${i}`,mass=e.mass_kg*c.volume*c.density/total;
        const g={id,bodyId,type:c.type,positionM:c.positionM,quaternionWxyz:c.quaternionWxyz,sizeM:c.sizeM,rgba:c.rgba,partId:c.partId};
        if(c.meshData){const mesh=`${id}_mesh`;g.mesh=mesh;compiled.meshes[mesh]={vertices:c.meshData.vertices_m.flat(),indices:c.meshData.triangles.flat()};compiled.assetXml+=`<mesh name="${mesh}" vertex="${join(c.meshData.vertices_m.flat())}" face="${join(c.meshData.triangles.flat())}"/>`;}
        compiled.geoms.push(g);
        return `<geom name="${id}" type="${c.type}" ${g.mesh?`mesh="${g.mesh}"`:`size="${join(g.sizeM)}"`} pos="${join(g.positionM)}" quat="${join(g.quaternionWxyz)}" mass="${mass}" contype="3" conaffinity="3" friction="${join(e.friction)}" solref=".005 1" rgba="${join(g.rgba)}"/>`;
      }).join('');
      compiled.xml=`<body name="${bodyId}" pos="${join(e.position_m)}" quat="${join(e.quaternion_wxyz)}">${free?`<freejoint name="${bodyId}_free"/>`:''}${geomXml}</body>`;
      if(e.parts.length>1 || e.parts.some(p=>p.grid && p.grid.counts.reduce((a,b)=>a*b,1)>1))warnings.push({objectId:e.id,code:'COMPOUND_MASS_ASSUMPTION',message:'Parts share one rigid body even when disconnected. Mass is weighted by component volume and density; overlapping volumes are counted separately. This is an authored rigid-coupling assumption.'});
    }
    const localBounds=bounds(points),worldBounds=bounds(points.map(p=>transform(p,e.position_m,e.quaternion_wxyz)));
    if(worldBounds.min.some((v,i)=>v<GENERAL_LIMITS.worldMin[i]-1e-8)||worldBounds.max.some((v,i)=>v>GENERAL_LIMITS.worldMax[i]+1e-8))throw new RangeError(`${e.id}: geometry exceeds the bounded 2.4 x 2.4 x 2.2 m construction volume`);
    const ports=derivedPorts(e,localBounds);
    // Port-specific geom lists prevent a different part of the receiver from falsely satisfying a contact test.
    for(const port of ports)port.geometryIds=compiled.geoms.filter(g=>g.partId===port.part_id).map(g=>g.id);
    const legacy=compiled.records?.[0]??{};
    // A single analytic primitive must not be tested against square AABB corners on a
    // circular receiving surface. Complex bodies retain an explicitly conservative envelope.
    const primitive = compiled.geoms.length === 1 && ['box','sphere','cylinder'].includes(compiled.geoms[0].type)
      ? (({type,positionM,quaternionWxyz,sizeM})=>({type,positionM,quaternionWxyz,sizeM}))(compiled.geoms[0]) : null;
    records.push({id:e.id,label:e.label,bodyId,kind:e.catalog?.kind??'constructed',dynamic:free,role:e.role,mass_kg:e.mass_kg,friction:e.friction,position_m:e.position_m,quaternion_wxyz:e.quaternion_wxyz,localBounds,worldBounds,...(primitive ? {placementPrimitive:primitive} : {}),ports,quantityEvidence:e.quantity_evidence,geometryIds:compiled.geoms.map(g=>g.id),affordances:legacy.affordances??{},mounting:free?'free body':e.fixed_reason,processModel:'dry rigid body; not instrument operation',limitations:e.limitations});
    if(e.motion==='fixed')warnings.push({objectId:e.id,code:'FIXED_BOUNDARY',message:e.fixed_reason});
    for(const [quantity,ev]of Object.entries(e.quantity_evidence))if(['image_estimated','assumed','fitted'].includes(ev.source)||quantity==='geometry'&&ev.uncertainty_m===undefined)warnings.push({objectId:e.id,code:'UNVERIFIED_QUANTITY',quantity,source:ev.source,message:ev.source_detail});
    if(e.appearance)for(const g of compiled.geoms){g.rgba=e.appearance.rgba;g.roughness=e.appearance.roughness;g.metalness=e.appearance.metalness;}
    if(e.visual_mesh){
      const b=bounds(e.visual_mesh.vertices_m);if(b.min.some((v,i)=>v<localBounds.min[i]-.01)||b.max.some((v,i)=>v>localBounds.max[i]+.01))throw new RangeError('Visual mesh exceeds collision envelope by more than 10 mm; provide an appropriate collision model');
      const name=`${bodyId}_visual_mesh`;compiled.meshes[name]={vertices:e.visual_mesh.vertices_m.flat(),indices:e.visual_mesh.triangles.flat()};
      visualGeoms.push({id:`${bodyId}_visual`,bodyId,type:'mesh',mesh:name,positionM:[0,0,0],quaternionWxyz:IDENTITY,sizeM:[],rgba:e.appearance?.rgba??[.48,.64,.7,1],roughness:e.appearance?.roughness??.55,metalness:e.appearance?.metalness??.05,visualOnly:true});
      warnings.push({objectId:e.id,code:'VISUAL_COLLISION_DIFFERENCE',message:'Detailed appearance is visual-only. Envelope checked, surface accuracy not certified. Use collision view for task-critical geometry.'});
    }
    xml+=compiled.xml;assetXml+=compiled.assetXml;Object.assign(meshes,compiled.meshes);geoms.push(...compiled.geoms);bodies.push(...compiled.bodies);
    if(geoms.length>GENERAL_LIMITS.components)throw new RangeError('Scene exceeds 512 physical collision components');
  }
  const unresolved=spec.inventory.filter(i=>i.status==='unresolved');
  return {spec,xml,assetXml,geoms,bodies,records,meshes,visualGeoms,passiveJoints:[],report:{schemaVersion:GENERAL_SCENE_VERSION,objectCount:records.length,collisionComponents:geoms.length,visualTriangles:spec.objects.reduce((s,e)=>s+(e.visual_mesh?.triangles.length??0),0),inventory:spec.inventory,unresolvedCount:unresolved.length,coverageComplete:spec.reference.mode==='none'?null:unresolved.length===0,coverageBasis:'author-declared source inventory; image omissions are not independently detected',imageReconstructionVerified:false,physicalValidation:'not yet compiled in MuJoCo',manipulationVerified:false,hardwareValidated:false,warnings,assumptions:spec.assumptions,adjustments:spec.adjustments,relationships:spec.relationships,limits:GENERAL_LIMITS}};
}
export function appendGeneralSceneXml(baseXml,raw){
  const c=compileGeneralScene(raw);let xml=workcellBaseXml(baseXml,'authored');
  if((xml.match(/<\/worldbody>/g)||[]).length!==1||(xml.match(/<\/asset>/g)||[]).length!==1)throw new Error('Unexpected pinned model structure');
  xml=xml.replace('</worldbody>',`${c.xml}</worldbody>`).replace('</asset>',`${c.assetXml}</asset>`);
  return {...c,xml};
}
