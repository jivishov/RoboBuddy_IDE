"""Validate the actual exported builder schemas, including raw and normalized examples."""
import json
import copy
import subprocess
from pathlib import Path
from jsonschema import Draft202012Validator
schemas = json.loads(subprocess.check_output(['node', 'tests/physics/openarm-general-contracts.mjs', '--schemas'], text=True))
for schema in schemas.values():
    Draft202012Validator.check_schema(schema)
scene_validator = Draft202012Validator(schemas['scene'])
for path in Path('tests/fixtures/general-scenes').glob('*.json'):
    scene = json.loads(path.read_text())
    assert scene_validator.is_valid(scene), (path, list(scene_validator.iter_errors(scene)))
    normalized = json.loads(subprocess.check_output(['node', '--input-type=module', '-e',
        "import fs from 'node:fs';import {normalizeGeneralScene} from './src/physics/openarm-general-scene.js';console.log(JSON.stringify(normalizeGeneralScene(JSON.parse(fs.readFileSync(process.argv[1],'utf8')))))", str(path)], text=True))
    assert scene_validator.is_valid(normalized), list(scene_validator.iter_errors(normalized))
    assert not scene_validator.is_valid({**scene, 'xml': '<mujoco/>'})
    stage = {'command':'stage', 'expected_scene_revision':'example', 'scene':scene}
    assert Draft202012Validator(schemas['manage']).is_valid(stage)
assert not Draft202012Validator(schemas['manage']).is_valid({'command':'apply','stage_id':'id','acknowledge_reset':False})
print('General laboratory scene schemas and example round trips: OK')

# Schema-visible structural errors should fail before reaching the physics compiler.
base = {'schema_version':'robobuddy.lab.scene.v2','id':'schema_review','objects':[
    {'id':'sample','position_m':[0.4,0,1.1],'parts':[{'id':'part','shape':'box','dimensions_m':[0.04,0.04,0.04]}]}]}
negative_objects = [
    {'motion':'fixed'}, {'mass_kg':20}, {'role':'bench'}, {'fixed_reason':'not fixed'},
    {'id':'constructor'}, {'friction':[0.8,0.1,0.0005]},
    {'quantity_evidence':{'mass':{'source':'user_measured'}}},
    {'quantity_evidence':{'friction':{'source':'source_provided'}}},
]
for invalid in negative_objects:
    scene = copy.deepcopy(base)
    scene['objects'][0].update(invalid)
    assert not scene_validator.is_valid(scene), invalid
positive = copy.deepcopy(base)
positive['objects'][0].update({'motion':'fixed','fixed_reason':'installed','mass_kg':20})
assert scene_validator.is_valid(positive)
task = {'schema_version':'robobuddy.lab.task.v2','id':'task','type':'dry_transfer','object_id':'sample',
        'receiver_id':'receiver','target_port':'top','side':'left','acknowledge_simulation_only':True}
validator = Draft202012Validator(schemas['task'])
assert validator.is_valid(task)
assert not validator.is_valid({**task,'object_port':'peg'})
assert not validator.is_valid({**task,'insertion_depth_m':0.02})
assert not validator.is_valid({**task,'type':'insert'})
assert validator.is_valid({**task,'type':'insert','object_port':'peg'})
print('General scene review: 8 invalid-object schema checks and task-family constraints passed')
