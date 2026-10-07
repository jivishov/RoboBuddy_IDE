"""Validate actual exported tool schemas, including oneOf additionalProperties scopes."""
import json, subprocess
from jsonschema import Draft202012Validator
schemas=json.loads(subprocess.check_output(['node','tests/physics/openarm-phase5a-core.mjs','--schemas'],text=True))
for schema in schemas.values(): Draft202012Validator.check_schema(schema)
v=Draft202012Validator(schemas['control'])
for request in [{'schema_version':'robobuddy.openarm.physical.v1','command':'reset'},{'schema_version':'robobuddy.openarm.physical.v1','command':'set_joint_targets','targets_rad':{'openarm_left_joint1':0.1}},{'schema_version':'robobuddy.openarm.physical.v1','command':'advance','seconds':0.1},{'schema_version':'robobuddy.openarm.physical.v1','command':'move_tool','side':'left','position_m':[.55,.15,1.2]}]:
 assert v.is_valid(request),list(v.iter_errors(request))
 assert not v.is_valid({**request,'unknown':0})
assert not v.is_valid({'schema_version':'robobuddy.openarm.physical.v1','command':'set_joint_targets','targets_rad':{'openarm_left_joint1':'0.1'}})
e=Draft202012Validator(schemas['equipment']);assert e.is_valid({'schema_version':'robobuddy.openarm.equipment.v1','command':'stage','expected_scene_revision':'test','equipment':[{'id':'tray','kind':'tray','position_m':[.3,-.3,1.005]}]})
assert not e.is_valid({'schema_version':'robobuddy.openarm.equipment.v1','command':'apply','stage_id':'a','acknowledge_reset':False})
p=Draft202012Validator(schemas['program']);assert p.is_valid({'schema_version':'robobuddy.openarm.program.v1','expected_scene_revision':'test','segments':[{'duration_seconds':1,'targets_rad':{'openarm_left_joint1':0},'wait_for':{'type':'supported','object_id':'flask','support_geom':'left_hotplate'}}]})
print('OpenArm exported JSON Schema positive/negative tests: OK')
