#!/usr/bin/env python3
"""Materialize the source-pinned MicroDuck physical plant.

This script consumes a checkout of pollen-robotics/microduck_rl at the pinned
revision and creates RoboBuddy's task-specific MuJoCo assets without replacing
source collision geometry with fitted primitives. It also updates the registered
model hashes and the permanent Phase 5C assertions that describe the BAM plant.

The upstream project currently declares its 3D model files as Creative Commons
BY-SA-NC without naming a license version. RoboBuddy therefore preserves that
exact declaration and does not invent a version-specific Creative Commons grant.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
PIN = "519142b1f5bf59fdfd44d06c205119e7fff8e3cb"
MODEL_DIR = REPO / "models" / "microduck"
SOURCE_DIR = MODEL_DIR / "source"
MESH_DIR = MODEL_DIR / "assets"
JOINTS = (
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
)
HOME = (
    0.0, -0.08726646259971647, -0.457924, -0.004940, 0.452984,
    0.3490658503988659, 0.3490658503988659, 0.0, 0.0,
    0.0, 0.08726646259971647, 0.457924, 0.004940, -0.452984,
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def indent_write(root: ET.Element, path: Path) -> None:
    ET.indent(root, space="  ")
    path.write_text(ET.tostring(root, encoding="unicode") + "\n", encoding="utf-8")


def source_robot_dir(checkout: Path) -> Path:
    result = checkout / "src" / "mjlab_microduck" / "robot" / "microduck"
    for name in ("robot_walk.xml", "robot_allcollisions.xml", "ball.xml"):
        if not (result / name).is_file():
            raise SystemExit(f"Pinned MicroDuck source is missing {name}: {result}")
    return result


def copy_sources(src: Path) -> None:
    SOURCE_DIR.mkdir(parents=True, exist_ok=True)
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    for name in ("robot_walk.xml", "robot_allcollisions.xml", "ball.xml", "config_mjcf_walk.json"):
        path = src / name
        if path.exists():
            shutil.copy2(path, SOURCE_DIR / name)
    shutil.rmtree(MESH_DIR, ignore_errors=True)
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    meshes = sorted((src / "assets").glob("*.stl"))
    if not meshes:
        raise SystemExit("Pinned MicroDuck source contains no STL assets")
    for path in meshes:
        shutil.copy2(path, MESH_DIR / path.name)

    lines = [
        f"# Source: pollen-robotics/microduck_rl@{PIN}",
        "# Format: git-blob-sha  path",
    ]
    for path in meshes:
        # Git blob SHA = sha1("blob <len>\\0" + bytes), independently reproducible.
        data = path.read_bytes()
        blob = hashlib.sha1(f"blob {len(data)}\0".encode() + data).hexdigest()
        lines.append(f"{blob}  assets/{path.name}")
    (SOURCE_DIR / "GIT_BLOB_SHA1SUMS").write_text("\n".join(lines) + "\n")


def add_runtime_wrapper(root: ET.Element, *, name: str, floor_friction: float, ball_source: Path | None) -> ET.Element:
    root = copy.deepcopy(root)
    root.set("model", name)
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("angle", "radian")
    compiler.set("autolimits", "true")
    compiler.set("meshdir", "assets")

    old_option = root.find("option")
    if old_option is not None:
        root.remove(old_option)
    option = ET.Element("option", {
        "integrator": "Euler", "timestep": "0.005", "iterations": "100",
        "ls_iterations": "50", "gravity": "0 0 -9.81",
    })
    root.insert(list(root).index(root.find("worldbody")), option)

    world = root.find("worldbody")
    if world is None:
        raise SystemExit("Pinned source model has no worldbody")
    floor = ET.Element("geom", {
        "name": "microduck_floor", "type": "plane", "size": "4 4 0.05",
        "pos": "0 0 0", "contype": "1", "conaffinity": "1", "condim": "3",
        "friction": f"{floor_friction:g} 0.005 0.0001", "solref": "0.02 1",
    })
    world.insert(0, floor)

    # mjlab's FULL_COLLISION configuration gives the two named soles friction
    # priority. Apply that source runtime configuration explicitly because this
    # standalone browser model is compiled outside mjlab.
    for geom in root.iter("geom"):
        if geom.get("name") in {"left_foot_collision", "right_foot_collision"}:
            geom.set("friction", f"{floor_friction:g} 0.005 0.0001")
            geom.set("priority", "1")
            geom.set("condim", "3")

    if ball_source is not None:
        ball_root = ET.parse(ball_source).getroot()
        ball_body = ball_root.find("./worldbody/body")
        if ball_body is None:
            raise SystemExit("Pinned ball.xml has no body")
        ball_body = copy.deepcopy(ball_body)
        ball_body.set("name", "microduck_ball")
        # Deterministic deployment-reference placement = source kick-task right-foot offset.
        ball_body.set("pos", "0.09 -0.042 0.035")
        free = ball_body.find("freejoint")
        geom = ball_body.find("geom")
        if free is None or geom is None:
            raise SystemExit("Pinned ball.xml is missing its free joint or geom")
        free.set("name", "microduck_ball_free")
        geom.set("name", "microduck_ball_geom")
        world.append(ball_body)

    old_actuator = root.find("actuator")
    if old_actuator is not None:
        root.remove(old_actuator)
    actuators = ET.SubElement(root, "actuator")
    for joint in JOINTS:
        ET.SubElement(actuators, "position", {
            "name": f"act_{joint}", "class": "chosen_actuator", "joint": joint,
        })

    old_keyframe = root.find("keyframe")
    if old_keyframe is not None:
        root.remove(old_keyframe)
    keyframe = ET.SubElement(root, "keyframe")
    qpos = ["0", "0", "0.12", "1", "0", "0", "0", *[f"{v:.15g}" for v in HOME]]
    if ball_source is not None:
        qpos += ["0.09", "-0.042", "0.035", "1", "0", "0", "0"]
    ET.SubElement(keyframe, "key", {
        "name": "HOME", "qpos": " ".join(qpos), "ctrl": " ".join(f"{v:.15g}" for v in HOME),
    })
    return root


def generate_models() -> dict[str, str]:
    walk_source = ET.parse(SOURCE_DIR / "robot_walk.xml").getroot()
    full_source = ET.parse(SOURCE_DIR / "robot_allcollisions.xml").getroot()
    builds = {
        "walk.xml": add_runtime_wrapper(walk_source, name="robobuddy_microduck_walk", floor_friction=1.0, ball_source=None),
        "walk_lowtraction.xml": add_runtime_wrapper(walk_source, name="robobuddy_microduck_walk_lowtraction", floor_friction=0.02, ball_source=None),
        "groundcontact.xml": add_runtime_wrapper(full_source, name="robobuddy_microduck_groundcontact", floor_friction=1.0, ball_source=None),
        "kick.xml": add_runtime_wrapper(full_source, name="robobuddy_microduck_kick", floor_friction=1.0, ball_source=SOURCE_DIR / "ball.xml"),
    }
    hashes = {}
    for filename, root in builds.items():
        path = MODEL_DIR / filename
        indent_write(root, path)
        hashes[filename] = sha256(path)
    (MODEL_DIR / "SHA256SUMS").write_text("".join(f"{digest}  {name}\n" for name, digest in sorted(hashes.items())))
    return hashes


def replace_between(text: str, start: str, next_start: str, replacement: str) -> str:
    a = text.find(start)
    if a < 0:
        raise SystemExit(f"patch marker missing: {start}")
    b = text.find(next_start, a + len(start))
    if b < 0:
        raise SystemExit(f"next patch marker missing: {next_start}")
    return text[:a] + replacement.rstrip() + "\n\n" + text[b:]


def patch_registry(hashes: dict[str, str]) -> None:
    path = REPO / "src" / "physics" / "microduck-model-package.js"
    text = path.read_text()
    for filename in ("walk.xml", "walk_lowtraction.xml", "kick.xml"):
        asset = f"models/microduck/{filename}"
        pattern = rf"(asset: '{re.escape(asset)}',\n\s*sha256: ')[0-9a-f]{{64}}(')"
        text, count = re.subn(pattern, rf"\g<1>{hashes[filename]}\2", text)
        if count != 1:
            raise SystemExit(f"Could not update registered hash for {asset}")
    text = text.replace(
        "  colliderPrimitiveShape: PARAMETER_EVIDENCE.ESTIMATED,",
        "  collisionMeshBytes: PARAMETER_EVIDENCE.SOURCE_DERIVED,\n  colliderPrimitiveShape: PARAMETER_EVIDENCE.SOURCE_DERIVED,",
    )
    text = text.replace(
        "  'The upstream collision meshes are CC BY-SA-NC assets and are not redistributed in this MIT repository. Repository-authored fitted collision boxes are therefore retained. The soles use the measured 45.6 x 34.0 mm contact face rather than an enlarged mesh bounding box. Exact source collision-mesh parity remains license-constrained.',",
        "  'Task-specific collision geometry now uses the exact STL mesh bytes from the pinned microduck_rl source. Walking uses the source reduced collision set; kick/recovery use the source all-collision plant. These 3D model files remain under upstream Creative Commons BY-SA-NC terms (version not specified upstream) and are outside RoboBuddy original-code MIT scope.',",
    )
    text = text.replace(
        "license: 'Apache-2.0 source-derived MicroDuck/BAM parameters; repository-authored collision proxies and floor are MIT. No CC BY-SA-NC collision mesh is redistributed.',",
        "license: 'Multi-license: RoboBuddy original software/floor wrapper MIT; source-derived MicroDuck/BAM software parameters Apache-2.0; pinned MicroDuck RL 3D model/collision files Creative Commons BY-SA-NC (upstream does not specify a version).',",
    )
    path.write_text(text)


def patch_worker() -> None:
    path = REPO / "src" / "physics" / "microduck-mujoco-worker.js"
    text = path.read_text()
    old = "    model.dof_frictionloss[actuator.joint.dof] = MICRODUCK_BAM_M6.frictionBaseNm;\n    model.dof_damping[actuator.joint.dof] = MICRODUCK_BAM_M6.frictionViscousNmPerRadS;"
    new = "    // BAM edit_spec replaces the XML joint friction/damping. The dynamic BAM values are\n    // written immediately before each mj_step; start from zero so the fallback servo plant\n    // can never be double-counted during model setup.\n    model.dof_frictionloss[actuator.joint.dof] = 0;\n    model.dof_damping[actuator.joint.dof] = 0;"
    if old not in text:
        raise SystemExit("BAM setup friction patch marker missing")
    path.write_text(text.replace(old, new))


def patch_phase5c_core() -> None:
    path = REPO / "tests" / "physics" / "microduck-phase5c-core.mjs"
    text = path.read_text()
    text = text.replace(
        "      assert(Math.abs(actuator.forceRangeNm[1] - MICRODUCK_SERVO_FORCE_NM) < 1e-12, `${actuator.id} force range is not the identified servo envelope`);",
        "      assert(Math.abs(actuator.sourceForceRangeNm[1] - MICRODUCK_SERVO_FORCE_NM) < 1e-12, `${actuator.id} lost the source XML fallback envelope`);\n      assert(actuator.forceRangeNm[1] > MICRODUCK_SERVO_FORCE_NM, `${actuator.id} was not converted to the BAM voltage-domain torque ceiling`);\n      assert(/BAM XL330 M6/.test(actuator.actuatorModel), `${actuator.id} does not declare the BAM plant`);",
    )
    text = text.replace(
        "  // Ordinary control writes actuator targets only.\n  const commandStart = worker.indexOf('function command(');\n  const commandEnd = worker.indexOf('function setup(');\n  const commandBody = worker.slice(commandStart, commandEnd);\n  assert(/data\\.ctrl\\[actuator\\.id\\]/.test(commandBody), 'the command path does not write actuator targets');\n  assert(!/qpos|qvel/.test(commandBody), 'the command path touches physical state directly');",
        "  // Ordinary control writes the dedicated position-target buffer only. BAM is the sole\n  // writer of MuJoCo ctrl, where ctrl is motor torque after runtime conversion.\n  const commandStart = worker.indexOf('function command(');\n  const commandEnd = worker.indexOf('function setup(');\n  const commandBody = worker.slice(commandStart, commandEnd);\n  assert(/requestedTargets\\[actuator\\.id\\] = target/.test(commandBody), 'the command path does not preserve actuator target intent');\n  assert(!/data\\.ctrl/.test(commandBody), 'the high-level command path bypasses BAM and writes MuJoCo torque directly');\n  assert(!/qpos|qvel/.test(commandBody), 'the command path touches physical state directly');\n  assert(/data\\.ctrl\\[index\\] = clamp\\(motorTorque/.test(worker), 'BAM is no longer the sole MuJoCo torque writer');",
    )

    model_check = """check('the committed model assets carry their registered hashes and the named contacts', () => {
  for (const modelPackage of MICRODUCK_MODEL_PACKAGES) {
    const actual = sha256(modelPackage.asset);
    assert(actual === modelPackage.sha256, `${modelPackage.asset} is ${actual}, registered ${modelPackage.sha256}`);
    const xml = readFileSync(resolve(ROOT, modelPackage.asset), 'utf8');
    assert(xml.includes('name=\"left_foot_collision\"'), `${modelPackage.asset} has no named left foot collider`);
    assert(xml.includes('name=\"right_foot_collision\"'), `${modelPackage.asset} has no named right foot collider`);
    assert(xml.includes('name=\"microduck_floor\"'), `${modelPackage.asset} has no named floor`);
    assert(xml.includes('name=\"trunk_base_freejoint\"'), `${modelPackage.asset} has no free trunk joint`);
    assert(xml.includes('name=\"imu_ang_vel\"'), `${modelPackage.asset} has no source-named imu gyro`);
    assert(!/<equality>/.test(xml), `${modelPackage.asset} declares an active equality block`);
    assert(!/\\bweld\\b/.test(xml), `${modelPackage.asset} declares a weld`);
    assert(/type=\"mesh\"/.test(xml), `${modelPackage.asset} no longer uses source mesh geometry`);
    assert(/mesh=\"sole_left\"/.test(xml) && /mesh=\"sole_right\"/.test(xml), `${modelPackage.asset} lost exact source sole meshes`);
    assert(/mass=\"0.264385\"/.test(xml), `${modelPackage.asset} lost the source trunk inertial`);
  }
  const walk = readFileSync(resolve(ROOT, MICRODUCK_WALK_PACKAGE.asset), 'utf8');
  // Pinned robot_walk: soles collide with the world; leg and power_support are self-collision-only.
  assert(/name=\"left_foot_collision\"[^>]*mesh=\"sole_left\"/.test(walk), 'walk plant lost the source left-sole collision');
  assert(/name=\"right_foot_collision\"[^>]*mesh=\"sole_right\"/.test(walk), 'walk plant lost the source right-sole collision');
  assert(/class=\"self_collision_only\"[^>]*mesh=\"leg\"/.test(walk), 'walk plant lost source leg self-collision');
  assert(/class=\"self_collision_only\"[^>]*mesh=\"power_support\"/.test(walk), 'walk plant lost source power-support self-collision');
  const lowTraction = readFileSync(resolve(ROOT, MICRODUCK_LOW_TRACTION_PACKAGE.asset), 'utf8');
  assert(lowTraction.includes('friction=\"0.02 0.005 0.0001\"'), 'the reduced-traction fixture does not declare its degraded friction');
  const kick = readFileSync(resolve(ROOT, MICRODUCK_KICK_PACKAGE.asset), 'utf8');
  assert(kick.includes('name=\"microduck_ball_geom\"') && kick.includes('mass=\"0.015\"'), 'the kick package lost the source ball');
  assert((kick.match(/class=\"collision\"/g) || []).length > (walk.match(/class=\"collision\"/g) || []).length,
    'kick/recovery plant is not using the broader source all-collision geometry');
  const notice = readFileSync(resolve(ROOT, 'THIRD_PARTY_NOTICES.md'), 'utf8');
  assert(notice.includes('Creative Commons BY-SA-NC') && notice.includes('version'), 'collision asset license/version scope is not disclosed');
});"""
    text = replace_between(text,
        "check('the committed model assets carry their registered hashes and the named contacts'",
        "check('the scenes validate against the shared physical-scene contract'",
        model_check)

    gain_check = """check('the deployed standing threshold is inclusive, and the gain schedule is real BAM physics', async () => {
  const { MICRODUCK_NOMINAL_FIRMWARE_GAIN } = await import('../../src/physics/microduck-model-package.js');
  const controller = new MicroDuckController({ availablePolicies: MICRODUCK_PHYSICAL_POLICIES });
  assert(controller.willStand(0.05), 'a twist of exactly the standing threshold no longer stands');
  assert(controller.willStand(0.049), 'a twist below the standing threshold no longer stands');
  assert(!controller.willStand(0.051), 'a twist above the standing threshold now stands');
  assert(MICRODUCK_NOMINAL_FIRMWARE_GAIN === 200, 'the nominal firmware gain moved off the source value');
  const simulator = readFileSync(resolve(ROOT, 'src/physics/microduck-physical-simulator.js'), 'utf8');
  assert(/firmwareGain: lastStep\\.gain/.test(simulator), 'the deployed firmware gain no longer travels with the controller targets');
  const worker = readFileSync(resolve(ROOT, 'src/physics/microduck-mujoco-worker.js'), 'utf8');
  assert(/firmwareGain: appliedFirmwareGain/.test(worker), 'BAM no longer receives the deployed firmware gain');
  assert(/microDuckBamMotorTorqueNm/.test(worker), 'the worker no longer computes BAM motor torque');
  assert(!/actuator_gainprm\\[actuator\\.id \* gainWidth\\] = kp/.test(worker), 'firmware gain has regressed into synthetic MuJoCo stiffness mutation');
});"""
    text = replace_between(text,
        "check('the deployed standing threshold is inclusive, and the gain schedule is real physics'",
        "check('a declared torque-off removes force, never the requested target'",
        gain_check)

    off_check = """check('a declared torque-off removes electromagnetic torque, never the requested target', () => {
  const worker = readFileSync(resolve(ROOT, 'src/physics/microduck-mujoco-worker.js'), 'utf8');
  assert(/enabled: actuationEnabled/.test(worker), 'BAM no longer receives the actuation-enabled state');
  assert(/data\\.ctrl\\[index\\] = clamp\\(motorTorque/.test(worker), 'MuJoCo ctrl is no longer BAM motor torque');
  assert(/requestedTargets\\[actuator\\.id\\] = target/.test(worker), 'torque-off can no longer preserve the requested joint target');
  assert(/actuatorForceTotalNm/.test(worker), 'the worker stopped publishing actuator effort, so torque-off is unfalsifiable');
  const bam = readFileSync(resolve(ROOT, 'src/physics/microduck-bam-plant.js'), 'utf8');
  assert(/if \(!enabled\) return 0/.test(bam), 'the BAM motor model no longer guarantees zero electromagnetic torque when disabled');
  // Passive BAM drivetrain friction remains physically present when motor torque is off.
  assert(/microDuckBamFrictionLossNm/.test(worker), 'motor-off incorrectly removes passive BAM drivetrain friction');
});"""
    text = replace_between(text,
        "check('a declared torque-off removes force, never the requested target'",
        "check('projected gravity is normalised, and there is only one implementation of it'",
        off_check)
    path.write_text(text)


def patch_native_bam() -> None:
    path = REPO / "native" / "microduck_bam_physical_reference.py"
    text = path.read_text()
    text = text.replace(
        "        m.dof_frictionloss[self.vadr:self.vadr + ref.ACTION_LEN] = FRICTION_BASE\n        m.dof_damping[self.vadr:self.vadr + ref.ACTION_LEN] = FRICTION_VISCOUS",
        "        # bam.mjlab.BamActuator.edit_spec zeros XML damping/friction. Dynamic BAM\n        # friction is written immediately before every mj_step below.\n        m.dof_frictionloss[self.vadr:self.vadr + ref.ACTION_LEN] = 0.0\n        m.dof_damping[self.vadr:self.vadr + ref.ACTION_LEN] = 0.0",
    )
    # Keep LegacyPlant's API only as a compatibility adapter, but make the semantics explicit.
    text = text.replace(
        "        self.applied_kp = kp\n        self.firmware_gain = max(0.0, kp / ref.IDENTIFIED_KP * ref.NOMINAL_FIRMWARE_GAIN)",
        "        # Compatibility adapter for the independent legacy controller harness. `kp` is\n        # converted back to the firmware register value; MuJoCo stiffness is never mutated.\n        self.applied_kp = None\n        self.firmware_gain = max(0.0, kp / ref.IDENTIFIED_KP * ref.NOMINAL_FIRMWARE_GAIN)",
    )
    path.write_text(text)


def patch_asset_manifest_script() -> None:
    path = REPO / "scripts" / "prepare_microduck_assets.mjs"
    text = path.read_text()
    text = text.replace(
        "excludedSources: ['pollen-robotics/microduck_rl mesh/MJCF bytes', 'Hugging Face Space implementation/assets', 'pollen-robotics/microduck sounds/scores/outer_wilds.mid'],",
        "excludedSources: ['Hugging Face Space implementation/assets', 'pollen-robotics/microduck sounds/scores/outer_wilds.mid'],\n  externalPhysicalPlant: { path: 'models/microduck/', repository: 'pollen-robotics/microduck_rl', revision: '519142b1f5bf59fdfd44d06c205119e7fff8e3cb', license: 'Creative Commons BY-SA-NC for upstream 3D model files; version not specified upstream', scope: 'task-specific physical collision meshes and pinned source MJCF' },",
    )
    path.write_text(text)


def write_licensing() -> None:
    license_path = REPO / "LICENSE"
    licenses = REPO / "LICENSES"
    licenses.mkdir(exist_ok=True)
    # Preserve the prior repository-wide MIT grant as the default license for original code.
    prior = license_path.read_text()
    if "MIT License" in prior:
        (licenses / "MIT.txt").write_text(prior)
    license_path.write_text("""RoboBuddy IDE — multi-license repository\n\nThere is no single license that applies to every file in this repository.\n\n1. Original RoboBuddy IDE source code is licensed under the MIT License unless a file or directory states otherwise. The MIT text is in LICENSES/MIT.txt.\n\n2. Third-party software, source-derived parameters, runtime distributions, model files, policies, and generated derivatives retain the licenses and notices stated in their source directories, manifests, or THIRD_PARTY_NOTICES.md.\n\n3. MicroDuck 3D model and collision files copied from pollen-robotics/microduck_rl are NOT covered by the RoboBuddy MIT grant. Upstream declares its 3D model files as “Creative Commons BY-SA-NC”. The upstream declaration does not specify a Creative Commons license version, so RoboBuddy does not assign or imply one. See THIRD_PARTY_NOTICES.md and models/microduck/source/.\n\n4. Some existing locally generated MicroDuck artifacts are marked PolyForm-Noncommercial-1.0.0 in assets/microduck/manifest.json. Those file-level declarations remain controlling for those artifacts.\n\nWhen redistributing a subset of this repository, comply with the license that applies to every included component.\n""")
    (REPO / "THIRD_PARTY_NOTICES.md").write_text(f"""# Third-party notices\n\n## MicroDuck runtime and controller sources\n\nSource: `pollen-robotics/microduck` pinned by the MicroDuck source audit. Upstream software license: Apache-2.0. RoboBuddy uses source-derived hierarchy/controller parameters and exact policy/runtime artifacts as documented in `assets/microduck/manifest.json` and `docs/physics/microduck-provenance.md`.\n\n## MicroDuck RL physical model and collision assets\n\nSource: `pollen-robotics/microduck_rl` at `{PIN}`.\n\nUpstream's current project license notice states: the project software is Apache 2.0 and **3D model files are licensed under Creative Commons BY-SA-NC**. That notice does not name a Creative Commons version. This repository therefore preserves the upstream wording and intentionally does not substitute a version-specific CC license.\n\nThe exact source MJCF files used for physical collision semantics are retained in `models/microduck/source/`. Exact source STL bytes are retained in `models/microduck/assets/`. `models/microduck/source/GIT_BLOB_SHA1SUMS` records their Git blob identities against the pinned source checkout. Generated RoboBuddy wrappers `walk.xml`, `walk_lowtraction.xml`, `groundcontact.xml`, and `kick.xml` preserve source mesh geometry while adding only the local floor, deterministic task prop placement, actuator names, timestep, and keyframe needed by the browser/native harness.\n\nWalking uses the pinned reduced `robot_walk.xml` collision set. Kick and recovery evidence use the pinned `robot_allcollisions.xml` plant, matching the task-specific upstream configuration rather than applying one collision approximation to all tasks.\n\n## BAM\n\nSource: `Rhoban/bam` v1.0.1, commit `ab81512c44f1f709b99ef332addb5e51568cd51c`. The MicroDuck plant reproduces the source XL330/M6 voltage-domain actuator equations and is independently checked against `better-actuator-models==1.0.1`. Refer to that upstream project for its license terms.\n\n## Important scope note\n\nThe component licenses above are not relicensed by RoboBuddy's MIT default. This file records provenance and scope; it is not a claim that hardware calibration has been performed.\n""")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-checkout", type=Path, required=True)
    args = parser.parse_args()
    src = source_robot_dir(args.source_checkout)
    copy_sources(src)
    hashes = generate_models()
    patch_registry(hashes)
    patch_worker()
    patch_native_bam()
    patch_phase5c_core()
    patch_asset_manifest_script()
    write_licensing()
    print("MicroDuck source plant finalized")
    for name, digest in sorted(hashes.items()):
        print(f"  {name}: {digest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
