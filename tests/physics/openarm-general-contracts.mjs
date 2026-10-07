import assert from 'node:assert/strict';
import {normalizeGeneralScene,compileGeneralScene} from '../../src/physics/openarm-general-scene.js';
import {normalizePart,partComponents} from '../../src/physics/openarm-general-geometry.js';
import {generalSceneSchema,generalTaskSchema,manageGeneralSceneSchema,manageGeneralTaskSchema} from '../../src/physics/openarm-general-schema.js';
import {readFile} from 'node:fs/promises';
if(process.argv.includes('--schemas')){console.log(JSON.stringify({scene:generalSceneSchema(),task:generalTaskSchema(),manage:manageGeneralSceneSchema(),manageTask:manageGeneralTaskSchema()}));process.exit(0);}
const base={schema_version:'robobuddy.lab.scene.v2',id:'contracts',objects:[{id:'object',position_m:[.4,-.3,1.1],parts:[{id:'cube',shape:'box',dimensions_m:[.04,.04,.04]}]}]};
for(const change of [
 s=>s.objects[0].parts[0].quaternion_wxyz=[0,0,0,0],
 s=>s.objects[0].parts[0].rgba=[1,1,1,0],
 s=>s.objects[0].mass_kg=20,
 s=>s.objects[0].motion='fixed',
 s=>s.objects[0].ports=[{id:'fake_hole',type:'opening',part_id:'cube'}],
 s=>s.objects[0].parts.push({...s.objects[0].parts[0]}),
 s=>s.objects[0].parts[0].shape='script',
 s=>s.objects[0].friction=[.8,1,1],
 s=>s.objects[0].visual_mesh={vertices_m:[[0,0,0],[.1,0,0],[0,.1,0]],triangles:[[0,1,2]]},
 s=>s.relationships=[{subject_id:'object',relation:'attached_to',object_id:'missing',source:'assumed'}],
 s=>s.reference={mode:'external_reference',content_sha256:'0'.repeat(64)},
 s=>s.reference={mode:'external_reference',agent_image_access:'verified'},
]){const s=structuredClone(base);change(s);assert.throws(()=>compileGeneralScene(s));}
const allocated=compileGeneralScene({...base,objects:[{...base.objects[0],mass_kg:.1,parts:[{id:'a',shape:'box',dimensions_m:[.02,.02,.02],density_kg_m3:1000},{id:'b',shape:'box',position_m:[.02,0,0],dimensions_m:[.02,.02,.02],density_kg_m3:2000}]}]});
const masses=[...allocated.xml.matchAll(/ mass="([^"]+)"/g)].map(m=>Number(m[1]));assert.ok(Math.abs(masses.reduce((a,b)=>a+b,0)-.1)<1e-12);assert.ok(Math.abs(masses[1]/masses[0]-2)<1e-12);
assert.equal(partComponents(normalizePart({id:'cone',shape:'frustum',height_m:.1,bottom_radius_m:.03,top_radius_m:0})).length,1);
for(const name of ['novel-adapter','titration-construction','novel-equipment-bench']){
 const s=JSON.parse(await readFile(new URL(`../fixtures/general-scenes/${name}.json`,import.meta.url),'utf8'));
 const c=compileGeneralScene(s);assert.deepEqual(normalizeGeneralScene(c.spec),c.spec);assert.ok(c.geoms.length<=512);assert.equal(c.report.imageReconstructionVerified,false);
 console.log(`${name}: ${c.records.length} objects, ${c.geoms.length} collision components; reference evidence remains declared`);
}
console.log('General scene deterministic contracts and adversarial input rejection: OK');
