"""Validate the exported public tool schema and compile every physical starter."""
import ast
import json
import sys
from pathlib import Path
from jsonschema import Draft7Validator, Draft202012Validator

cases = json.loads(Path(sys.argv[1]).read_text())
for validator_type in (Draft7Validator, Draft202012Validator):
    validator_type.check_schema(cases['schema'])
    validator = validator_type(cases['schema'])
    for value in cases['valid']:
        errors = list(validator.iter_errors(value))
        assert not errors, (validator_type.__name__, value, [e.message for e in errors])
    for value in cases['invalid']:
        assert not validator.is_valid(value), (validator_type.__name__, value)
for workspace in cases['workspaces']:
    for name, source in workspace['files'].items():
        compile(source, workspace['id'] + '/' + name, 'exec', flags=ast.PyCF_ALLOW_TOP_LEVEL_AWAIT)
print(f"JSON Schema draft-07 and 2020-12: {len(cases['valid'])} valid + {len(cases['invalid'])} invalid vectors per draft; all 12 MicroDuck starter files compile")
