#!/usr/bin/env python3
"""Finalize task-specific MicroDuck collision-plant routing.

This is intentionally a deterministic migration script.  It refuses to continue if the
expected Phase-5C source text has drifted, so the one-shot materialisation workflow cannot
silently patch a different architecture.
"""
from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text()


def write(path: str, text: str) -> None:
    (ROOT / path).write_text(text)


def replace_once(path: str, old: str, new: str) -> None:
    text = read(path)
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{path}: expected exactly one replacement target, found {count}")
    write(path, text.replace(old, new, 1))


def regex_once(path: str, pattern: str, replacement: str) -> None:
    text = read(path)
    new, count = re.subn(pattern, replacement, text, count=1, flags=re.S)
    if count != 1:
        raise SystemExit(f"{path}: expected exactly one regex target, found {count}")
    write(path, new)


# ---------------------------------------------------------------- model package
path = "src/physics/microduck-model-package.js"
replace_once(
    path,
    "  'Task-specific collision geometry now uses the exact STL mesh bytes from the pinned microduck_rl source. Walking uses the source reduced collision set; kick/recovery use the source all-collision plant. These 3D model files remain under upstream Creative Commons BY-SA-NC terms (version not specified upstream) and are outside RoboBuddy original-code MIT scope.',",
    "  'Task-specific collision geometry uses the exact STL mesh bytes from the pinned microduck_rl source. Locomotion uses robot_walk.xml; explicit stand/recovery, sit/stand, ground-pick and the conservative roulade runtime mapping use robot_allcollisions.xml; kick uses that all-collision robot plus the source ball.xml prop. These 3D model files remain under upstream Creative Commons BY-SA-NC terms (version not specified upstream) and are outside RoboBuddy original-code MIT scope.',",
)
replace_once(
    path,
    "export const MICRODUCK_KICK_PACKAGE = registerModelPackage((() => {",
    "export const MICRODUCK_GROUNDCONTACT_PACKAGE = registerModelPackage(basePackage({\n"
    "  id: 'microduck-groundcontact-519142b-v1',\n"
    "  modelId: 'robobuddy-microduck-groundcontact-v1',\n"
    "  asset: 'models/microduck/groundcontact.xml',\n"
    "  sha256: '43c828db58973bc2d1a56ccba72a46f0ae89837f859f64641ea1c251d2cfd108',\n"
    "  variant: 'MicroDuck alpha free-base biped with the pinned robot_allcollisions.xml mesh-contact plant for body-on-ground skills',\n"
    "  floorFriction: MICRODUCK_NOMINAL_FLOOR_FRICTION,\n"
    "  limitations: [\n"
    "    'This package is the broad source collision plant used by pinned stand-up, sit/stand and ground-pick task configurations. It intentionally differs from the reduced walking collision plant.',\n"
    "    'The pinned microduck_rl revision does not contain the roulade task configuration even though the deployed runtime carries roulade.onnx. Roulade is therefore routed to this broad contact plant conservatively and remains experimental rather than being claimed as an exact pinned training-task match.',\n"
    "  ],\n"
    "}));\n\n"
    "export const MICRODUCK_KICK_PACKAGE = registerModelPackage((() => {",
)
replace_once(
    path,
    "export const MICRODUCK_MODEL_PACKAGES = Object.freeze([\n  MICRODUCK_WALK_PACKAGE,\n  MICRODUCK_LOW_TRACTION_PACKAGE,\n  MICRODUCK_KICK_PACKAGE,\n]);",
    "export const MICRODUCK_MODEL_PACKAGES = Object.freeze([\n  MICRODUCK_WALK_PACKAGE,\n  MICRODUCK_LOW_TRACTION_PACKAGE,\n  MICRODUCK_GROUNDCONTACT_PACKAGE,\n  MICRODUCK_KICK_PACKAGE,\n]);",
)

# ---------------------------------------------------------------------- scenes
path = "src/physics/microduck-scene.js"
replace_once(
    path,
    "  MICRODUCK_FLOOR_ID, MICRODUCK_JOINT_CONTROLLER, MICRODUCK_KICK_PACKAGE,\n  MICRODUCK_LOW_TRACTION_PACKAGE, MICRODUCK_ROBOT_ID, MICRODUCK_WALK_PACKAGE,",
    "  MICRODUCK_FLOOR_ID, MICRODUCK_GROUNDCONTACT_PACKAGE, MICRODUCK_JOINT_CONTROLLER, MICRODUCK_KICK_PACKAGE,\n  MICRODUCK_LOW_TRACTION_PACKAGE, MICRODUCK_ROBOT_ID, MICRODUCK_WALK_PACKAGE,",
)
replace_once(
    path,
    "export const MICRODUCK_KICK_SCENE = scene({",
    "export const MICRODUCK_GROUNDCONTACT_SCENE = scene({\n"
    "  id: 'microduck-physical-groundcontact',\n"
    "  modelPackage: MICRODUCK_GROUNDCONTACT_PACKAGE,\n"
    "  taskGoal: {\n"
    "    id: 'microduck-body-ground-contact-skills',\n"
    "    description: 'Run body-on-ground skills against the pinned all-collision robot plant.',\n"
    "    successCriteria: 'outcomes come from the requested policy plus actual MuJoCo body/floor contact; reset/setup never counts as progress',\n"
    "    evaluatedFrom: 'authoritative MuJoCo trunk pose and named robot/floor contacts',\n"
    "  },\n"
    "});\n\n"
    "export const MICRODUCK_KICK_SCENE = scene({",
)
replace_once(
    path,
    "export const MICRODUCK_SCENES = Object.freeze({\n  walk: MICRODUCK_WALK_SCENE,\n  lowTraction: MICRODUCK_LOW_TRACTION_SCENE,\n  kick: MICRODUCK_KICK_SCENE,\n});",
    "export const MICRODUCK_SCENES = Object.freeze({\n  walk: MICRODUCK_WALK_SCENE,\n  lowTraction: MICRODUCK_LOW_TRACTION_SCENE,\n  groundContact: MICRODUCK_GROUNDCONTACT_SCENE,\n  kick: MICRODUCK_KICK_SCENE,\n});",
)

# ---------------------------------------------------------- capabilities/routing
path = "src/physics/microduck-capabilities.js"
replace_once(
    path,
    "import { MICRODUCK_KICK_PACKAGE, MICRODUCK_LOW_TRACTION_PACKAGE, MICRODUCK_WALK_PACKAGE } from './microduck-model-package.js';",
    "import { MICRODUCK_GROUNDCONTACT_PACKAGE, MICRODUCK_KICK_PACKAGE, MICRODUCK_LOW_TRACTION_PACKAGE, MICRODUCK_WALK_PACKAGE } from './microduck-model-package.js';",
)
replace_once(
    path,
    "export function isPhysicallySupported(id) {\n  const record = microduckCapability(id);\n  return Boolean(record && record.physicalPolicy);\n}\n",
    "export function isPhysicallySupported(id) {\n  const record = microduckCapability(id);\n  return Boolean(record && record.physicalPolicy);\n}\n\n"
    "// Collision-plant routing is part of policy compatibility. A source policy is not allowed\n"
    "// to run merely because its observation/action widths match: its task must also use the\n"
    "// collision plant against which that behaviour is represented here.\n"
    "export const MICRODUCK_CAPABILITY_PACKAGE_KEYS = Object.freeze({\n"
    "  stand: Object.freeze(['groundContact']),\n"
    "  walk: Object.freeze(['walk', 'lowTraction']),\n"
    "  sit_stand: Object.freeze(['groundContact']),\n"
    "  ground_pick: Object.freeze(['groundContact']),\n"
    "  kick_left: Object.freeze(['kick']),\n"
    "  kick_right: Object.freeze(['kick']),\n"
    "  recovery: Object.freeze(['groundContact']),\n"
    "  roulade: Object.freeze(['groundContact']),\n"
    "});\n\n"
    "export function microduckRequiredPackageKeys(capabilityId) {\n"
    "  return MICRODUCK_CAPABILITY_PACKAGE_KEYS[capabilityId] || Object.freeze([]);\n"
    "}\n\n"
    "export function microduckPackageSupportsCapability(packageKey, capabilityId) {\n"
    "  return microduckRequiredPackageKeys(capabilityId).includes(packageKey);\n"
    "}\n",
)
regex_once(
    path,
    r"    id: 'recovery', label: 'Fall recovery'.*?\n  \}\),\n  Object\.freeze\(\{\n    id: 'roulade', label: 'Roulade \(forward roll\)'.*?\n  \}\),",
    "    id: 'recovery', label: 'Fall recovery', status: S.PHYSICAL_VERIFIED, physicalPolicy: 'stand',\n"
    "    evidence: 'Runs only on the source all-collision ground-contact plant. Positive and negative BAM/MuJoCo recovery controls are validated from declared setup orientations, actuator force, trunk pose and contact; reset is never counted as recovery.',\n"
    "  }),\n"
    "  Object.freeze({\n"
    "    id: 'roulade', label: 'Roulade (forward roll)', status: S.PHYSICAL_EXPERIMENTAL, physicalPolicy: 'roulade',\n"
    "    evidence: 'The deployed runtime includes roulade.onnx. It is routed to the broad source all-collision plant so trunk/head contact is represented by exact source meshes. The pinned microduck_rl revision does not contain the roulade task configuration, so this remains experimental rather than an exact task-training-plant claim.',\n"
    "  }),",
)
regex_once(
    path,
    r"const PHYSICAL_LIMITATIONS = Object\.freeze\(\[.*?\n\]\);",
    "const PHYSICAL_LIMITATIONS = Object.freeze([\n"
    "  'Task-specific collision geometry is source-derived, not a single universal approximation: locomotion uses the pinned robot_walk.xml reduced set; explicit body-on-ground skills use robot_allcollisions.xml; kick uses robot_allcollisions.xml plus the source ball.xml prop.',\n"
    "  'The committed STL collider bytes are copied exactly from the pinned microduck_rl revision and retain upstream Creative Commons BY-SA-NC terms; upstream does not state a CC version. RoboBuddy original software remains MIT-scoped separately.',\n"
    "  'The interactive physical workspace uses BAM 1.0.1 XL330/M6 dynamics with the deployed firmware-gain schedule and deployed median-of-three IMU preprocessing. Training-only domain randomisation/delay is retained as a separate source reference profile rather than silently mixed into deployment rehearsal.',\n"
    "  'Roller-mode locomotion and roller crouch are unsupported in physical mode: no matched roller plant exists at the pinned revision. They are not routed anywhere else.',\n"
    "  'Native/browser agreement and source-model fidelity are software/model evidence, not assembled-hardware calibration. Battery/internal resistance, bus latency, thermal effects, individual servo variation, wear and real contact materials remain hardware-validation items.',\n"
    "]);",
)
replace_once(
    path,
    "export const MICRODUCK_PHYSICAL_PACKAGES = Object.freeze({\n  walk: MICRODUCK_WALK_PACKAGE,\n  lowTraction: MICRODUCK_LOW_TRACTION_PACKAGE,\n  kick: MICRODUCK_KICK_PACKAGE,\n});",
    "export const MICRODUCK_PHYSICAL_PACKAGES = Object.freeze({\n  walk: MICRODUCK_WALK_PACKAGE,\n  lowTraction: MICRODUCK_LOW_TRACTION_PACKAGE,\n  groundContact: MICRODUCK_GROUNDCONTACT_PACKAGE,\n  kick: MICRODUCK_KICK_PACKAGE,\n});",
)

# --------------------------------------------------------------- simulator guard
path = "src/physics/microduck-physical-simulator.js"
replace_once(
    path,
    "  MICRODUCK_PHYSICAL_POLICIES, assertMicroDuckCompatibility, isPhysicallySupported,\n  microduckCapability, microduckCompatibilityIdentity,",
    "  MICRODUCK_PHYSICAL_POLICIES, assertMicroDuckCompatibility, isPhysicallySupported,\n  microduckCapability, microduckCompatibilityIdentity, microduckPackageSupportsCapability,\n  microduckRequiredPackageKeys,",
)
replace_once(
    path,
    "    const { command, limitedBy } = boundedMicroDuckCommand(requested);\n    this.requested = command;",
    "    const { command, limitedBy } = boundedMicroDuckCommand(requested);\n"
    "    if (!['walk', 'lowTraction'].includes(this.packageKey) && command.twist.some((value) => Math.abs(value) > 1e-12)) {\n"
    "      throw new Error(`MicroDuck collision-plant mismatch: non-zero velocity commands require the walk plant; active package is ${this.packageKey}`);\n"
    "    }\n"
    "    this.requested = command;",
)
replace_once(
    path,
    "    if (capability && !capability.physicalPolicy) {\n      return { accepted: false, status: 'unsupported', capability: capability.id, reason: capability.evidence };\n    }\n    const started = this.controller.requestSkill(skill);",
    "    if (capability && !capability.physicalPolicy) {\n      return { accepted: false, status: 'unsupported', capability: capability.id, reason: capability.evidence };\n    }\n"
    "    if (capability && !microduckPackageSupportsCapability(this.packageKey, capability.id)) {\n"
    "      const requiredPackageKeys = [...microduckRequiredPackageKeys(capability.id)];\n"
    "      return {\n"
    "        accepted: false, status: 'wrong-plant', capability: capability.id, requiredPackageKeys,\n"
    "        reason: `${capability.id} requires MicroDuck collision plant ${requiredPackageKeys.join(' or ')}; active package is ${this.packageKey}`,\n"
    "      };\n"
    "    }\n"
    "    const started = this.controller.requestSkill(skill);",
)
replace_once(
    path,
    "      capabilities: MICRODUCK_CAPABILITY_AUDIT.map((item) => ({ id: item.id, label: item.label, status: item.status, physical: Boolean(item.physicalPolicy) })),",
    "      capabilities: MICRODUCK_CAPABILITY_AUDIT.map((item) => ({ id: item.id, label: item.label, status: item.status, physical: Boolean(item.physicalPolicy), availableInActivePlant: microduckPackageSupportsCapability(this.packageKey, item.id), requiredPackageKeys: [...microduckRequiredPackageKeys(item.id)] })),",
)

# ------------------------------------------------------------- native reference
path = "native/microduck_reference.py"
replace_once(
    path,
    '    "lowtraction": ROOT / "models" / "microduck" / "walk_lowtraction.xml",\n    "kick": ROOT / "models" / "microduck" / "kick.xml",',
    '    "lowtraction": ROOT / "models" / "microduck" / "walk_lowtraction.xml",\n    "groundcontact": ROOT / "models" / "microduck" / "groundcontact.xml",\n    "kick": ROOT / "models" / "microduck" / "kick.xml",',
)
replace_once(
    path,
    '        "modelPackageAsset": str(plant.path.relative_to(ROOT)),\n        "modelSha256": plant.sha256,',
    '        "collisionPlant": model_key,\n        "modelPackageAsset": str(plant.path.relative_to(ROOT)),\n        "modelSha256": plant.sha256,',
)
replace_once(
    path,
    '                       "appliedKp": round(plant.applied_kp, 6)},',
    '                       "appliedKp": None if plant.applied_kp is None else round(plant.applied_kp, 6)},',
)
for old, new in [
    ('"stand": dict(model_key="walk",', '"stand": dict(model_key="groundcontact",'),
    ('"sit": dict(model_key="walk",', '"sit": dict(model_key="groundcontact",'),
    ('"ground-pick": dict(model_key="walk",', '"ground-pick": dict(model_key="groundcontact",'),
    ('"roulade": dict(model_key="walk",', '"roulade": dict(model_key="groundcontact",'),
    ('"recover-face-down": dict(model_key="walk",', '"recover-face-down": dict(model_key="groundcontact",'),
    ('"recover-face-up": dict(model_key="walk",', '"recover-face-up": dict(model_key="groundcontact",'),
    ('"recover-on-side": dict(model_key="walk",', '"recover-on-side": dict(model_key="groundcontact",'),
    ('"recover-no-actuation": dict(model_key="walk",', '"recover-no-actuation": dict(model_key="groundcontact",'),
    ('"recover-short-budget": dict(model_key="walk",', '"recover-short-budget": dict(model_key="groundcontact",'),
]:
    replace_once(path, old, new)
regex_once(path, r'\n    "recover-low-traction": dict\(model_key="lowtraction".*?\),', '')

# ----------------------------------------------------------- BAM evidence gate
path = "tests/validate_microduck_bam_evidence.py"
replace_once(
    path,
    "# Deployment-reference gain is fed into BAM, not converted into MuJoCo stiffness.\n",
    "# Collision routing is part of the evidence, not an incidental filename choice.\n"
    "for name in ('walk', 'walk-fast', 'walk-no-actuation'):\n"
    "    assert reports[name]['collisionPlant'] == 'walk', (name, reports[name]['collisionPlant'])\n"
    "assert reports['walk-low-traction']['collisionPlant'] == 'lowtraction'\n"
    "for name in ('stand', 'sit', 'ground-pick', 'roulade', 'recover-face-down', 'recover-face-up', 'recover-on-side', 'recover-no-actuation', 'recover-short-budget'):\n"
    "    assert reports[name]['collisionPlant'] == 'groundcontact', (name, reports[name]['collisionPlant'])\n"
    "for name in ('kick-right', 'kick-left', 'kick-miss'):\n"
    "    assert reports[name]['collisionPlant'] == 'kick', (name, reports[name]['collisionPlant'])\n\n"
    "# Deployment-reference gain is fed into BAM, not converted into MuJoCo stiffness.\n",
)

# ---------------------------------------------------------------- source audit
path = "src/physics/microduck-source-audit.js"
replace_once(
    path,
    "  license: 'Apache-2.0 for code and MJCF; the STL mesh assets are CC BY-SA-NC and are NOT redistributed by this repository',",
    "  license: 'Upstream currently declares project software Apache-2.0 and 3D model files Creative Commons BY-SA-NC (CC version not specified). Exact pinned STL collision bytes are redistributed here under that separate upstream 3D-model scope.',",
)

# ------------------------------------------------------------ WebMCP/live errors
path = "src/webmcp/microduck-physical-control.js"
replace_once(
    path,
    "      if (!result.accepted) {\n        throw new WebMcpDomainError('CAPABILITY_UNSUPPORTED', `${parsed.skill} is unsupported in MicroDuck physical mode: ${result.reason}`, {\n          retryable: false,\n          details: { capability: parsed.skill, status: MICRODUCK_CAPABILITY_STATUS.UNSUPPORTED, routedToLegacy: false },\n        });\n      }",
    "      if (!result.accepted) {\n"
    "        const plantMismatch = result.status === 'wrong-plant';\n"
    "        throw new WebMcpDomainError(plantMismatch ? 'PLANT_MISMATCH' : 'CAPABILITY_UNSUPPORTED', `${parsed.skill} cannot run in the active MicroDuck physical plant: ${result.reason}`, {\n"
    "          retryable: plantMismatch,\n"
    "          details: { capability: parsed.skill, status: plantMismatch ? result.status : MICRODUCK_CAPABILITY_STATUS.UNSUPPORTED, requiredPackageKeys: result.requiredPackageKeys || [], routedToLegacy: false },\n"
    "        });\n"
    "      }",
)
path = "src/runtime/microduck-physical-bridge.js"
replace_once(
    path,
    "      if (!skillResult.accepted) {\n        const capability = microduckCapability(skill);\n        throw liveError('CAPABILITY_UNSUPPORTED', `${skill} is unsupported in MicroDuck physical mode: ${skillResult.reason}`, {\n          capability: skill, status: capability?.status ?? 'unsupported in physical mode', routedToLegacy: false,\n        });\n      }",
    "      if (!skillResult.accepted) {\n"
    "        const capability = microduckCapability(skill);\n"
    "        const plantMismatch = skillResult.status === 'wrong-plant';\n"
    "        throw liveError(plantMismatch ? 'PLANT_MISMATCH' : 'CAPABILITY_UNSUPPORTED', `${skill} cannot run in the active MicroDuck physical plant: ${skillResult.reason}`, {\n"
    "          capability: skill, status: plantMismatch ? skillResult.status : (capability?.status ?? 'unsupported in physical mode'), requiredPackageKeys: skillResult.requiredPackageKeys || [], routedToLegacy: false,\n"
    "        });\n"
    "      }",
)

# --------------------------------------------------------------- task catalogue
path = "src/task-catalog.js"
replace_once(
    path,
    "import { MICRODUCK_WALK_PACKAGE } from './physics/microduck-model-package.js';\nimport { MICRODUCK_GAIT_ONSET_MS, MICRODUCK_WALK_SCENE } from './physics/microduck-scene.js';",
    "import { MICRODUCK_GROUNDCONTACT_PACKAGE, MICRODUCK_KICK_PACKAGE, MICRODUCK_WALK_PACKAGE } from './physics/microduck-model-package.js';\n"
    "import { MICRODUCK_GAIT_ONSET_MS, MICRODUCK_GROUNDCONTACT_SCENE, MICRODUCK_KICK_SCENE, MICRODUCK_WALK_SCENE } from './physics/microduck-scene.js';",
)
regex_once(
    path,
    r"const MICRODUCK_PHYSICAL_SCENARIO = Object\.freeze\(\{.*?export const MICRODUCK_PHYSICAL_TASKS = Object\.freeze\(\[Object\.freeze\(\{.*?\}\)\]\);",
    "function microduckPhysicalScenario({ id, title, brief, modelPackage, physicalScene, capabilityIds, taskEvaluation }) {\n"
    "  return Object.freeze({\n"
    "    schema: 'robobuddy.physical-workspace.v1', schemaVersion: 1, simulationMode: 'physical_mujoco',\n"
    "    workspaceRevision: physicalScene.revision, id, title, brief, robotId: modelPackage.robotId,\n"
    "    physicalSceneId: physicalScene.id, physicalSceneRevision: physicalScene.revision,\n"
    "    modelPackage: modelPackage.id, modelId: modelPackage.modelId,\n"
    "    physicalApi: Object.freeze({ version: 'robobuddy.sim.v1', angleUnit: 'rad', timeUnit: 's', lengthUnit: 'm', velocityUnit: 'm/s', yawRateUnit: 'rad/s' }),\n"
    "    canonicalModel: Object.freeze({ repository: MICRODUCK_RUNTIME_SOURCE.repository, revision: MICRODUCK_RUNTIME_SOURCE.revision, sourcePath: 'kinematics/assets/alpha/robot_walk.xml', visualSourcePath: 'robotctl/assets/duck.bin', variant: MICRODUCK_RUNTIME_SOURCE.variant, physicalEnvironmentRepository: MICRODUCK_RL_SOURCE.repository, physicalEnvironmentRevision: MICRODUCK_RL_SOURCE.revision, collisionPlant: modelPackage.asset, authority: 'MuJoCo PhysicsSession; the official MicroDuck visual consumes observed trunk and joint state only' }),\n"
    "    controller: Object.freeze({ observationWidth: 61, actionWidth: 14, mouthWireIndex: 9, physicsTimestepSeconds: MICRODUCK_PHYSICS_TIMESTEP_SECONDS, controlIntervalSeconds: MICRODUCK_CONTROL_INTERVAL_SECONDS, previousActionSemantics: 'raw policy output before action scaling', source: `${MICRODUCK_RUNTIME_SOURCE.repository} duck-control/src/obs.rs and robotd/src/control.rs` }),\n"
    "    frames: Object.freeze({ physics: 'MuJoCo right-handed Z-up world, metres/radians; the trunk is a free body and the gyro is read at the source-named imu site', rendering: 'Three.js Y-up millimetres derived from the observed MuJoCo trunk transform; the renderer never re-seats the robot on the floor and never advances physics' }),\n"
    "    capabilities: Object.freeze(MICRODUCK_CAPABILITY_AUDIT.filter((item) => capabilityIds.includes(item.id)).map((item) => Object.freeze({ id: item.id, label: item.label, status: item.status, physical: Boolean(item.physicalPolicy) }))),\n"
    "    portablePython: Object.freeze({ referenceActions: Object.freeze([]) }),\n"
    "    taskEvaluation: Object.freeze(taskEvaluation), limitations: Object.freeze([...modelPackage.limitations]),\n"
    "  });\n"
    "}\n\n"
    "const MICRODUCK_PHYSICAL_SCENARIO = microduckPhysicalScenario({\n"
    "  id: 'microduck-physical-locomotion', title: 'Physical MicroDuck Locomotion', modelPackage: MICRODUCK_WALK_PACKAGE, physicalScene: MICRODUCK_WALK_SCENE, capabilityIds: ['walk'],\n"
    "  brief: `Command MicroDuck through the pinned deployed controller on the source reduced walking collision plant. Below about ${MICRODUCK_GAIT_ONSET_MS} m/s the deployed scheduler may hold a stand; non-zero locomotion is never routed to the all-collision ground-contact plant.`,\n"
    "  taskEvaluation: { source: 'MuJoCo trunk free-body pose plus named foot-floor contacts', requires: Object.freeze(['commanded-axis displacement', 'alternating foot-contact transitions', 'no fall', 'propulsion lost when actuation or traction is removed']), syntheticSuccessEvents: false },\n"
    "});\n"
    "const MICRODUCK_GROUNDCONTACT_SCENARIO = microduckPhysicalScenario({\n"
    "  id: 'microduck-physical-groundcontact', title: 'Physical MicroDuck Ground Contact', modelPackage: MICRODUCK_GROUNDCONTACT_PACKAGE, physicalScene: MICRODUCK_GROUNDCONTACT_SCENE, capabilityIds: ['stand', 'sit_stand', 'ground_pick', 'recovery', 'roulade'],\n"
    "  brief: 'Run explicit standing/body-pose, sit/stand, ground-pick and fall-recovery behaviours on the pinned robot_allcollisions.xml plant so trunk, head and limb contacts with the floor are represented by the source meshes. Roulade is available here only as an experimental conservative mapping because its task config is absent from the pinned RL revision.',\n"
    "  taskEvaluation: { source: 'MuJoCo trunk pose, joint state and named robot/floor contacts', requires: Object.freeze(['source all-collision plant', 'contact-driven body motion', 'declared setup separated from progress', 'no reset-as-recovery']), syntheticSuccessEvents: false },\n"
    "});\n"
    "const MICRODUCK_KICK_PHYSICAL_SCENARIO = microduckPhysicalScenario({\n"
    "  id: 'microduck-physical-kick', title: 'Physical MicroDuck Ball Kick', modelPackage: MICRODUCK_KICK_PACKAGE, physicalScene: MICRODUCK_KICK_SCENE, capabilityIds: ['kick_left', 'kick_right'],\n"
    "  brief: 'Run the deployed left/right kick policies on the source all-collision robot plus the pinned 70 mm / 15 g ball. Ball motion must follow named sole/ball MuJoCo contact; no kick impulse or ball-state overwrite exists.',\n"
    "  taskEvaluation: { source: 'MuJoCo named sole/ball contacts and ball free-body pose', requires: Object.freeze(['named foot-ball contact', 'contact-following ball displacement', 'miss negative control']), syntheticSuccessEvents: false },\n"
    "});\n"
    "const MICRODUCK_PHYSICAL_SCENARIOS = Object.freeze({\n"
    "  [MICRODUCK_PHYSICAL_SCENARIO.id]: MICRODUCK_PHYSICAL_SCENARIO,\n"
    "  [MICRODUCK_GROUNDCONTACT_SCENARIO.id]: MICRODUCK_GROUNDCONTACT_SCENARIO,\n"
    "  [MICRODUCK_KICK_PHYSICAL_SCENARIO.id]: MICRODUCK_KICK_PHYSICAL_SCENARIO,\n"
    "});\n"
    "export const MICRODUCK_PHYSICAL_TASKS = Object.freeze(Object.values(MICRODUCK_PHYSICAL_SCENARIOS).map((scenario) => Object.freeze({ profileId: 'microduck', id: scenario.id, title: scenario.title, robotId: scenario.robotId, simulationMode: scenario.simulationMode, physicalSceneId: scenario.physicalSceneId })));",
)
replace_once(
    path,
    "    if (profileId === 'microduck') return structuredClone(MICRODUCK_PHYSICAL_SCENARIO);",
    "    if (profileId === 'microduck') return structuredClone(MICRODUCK_PHYSICAL_SCENARIOS[descriptor.id]);",
)
replace_once(
    path,
    "      modelPackage: MICRODUCK_PHYSICAL_SCENARIO.modelPackage,",
    "      modelPackage: MICRODUCK_PHYSICAL_SCENARIOS[descriptor.id]?.modelPackage ?? null,",
)

print('MicroDuck task-specific collision routing finalized')
