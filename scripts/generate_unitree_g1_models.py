#!/usr/bin/env python3
"""Deterministically generate the Unitree G1 29-DoF physical MJCF variants.

Provenance of every element the generated models contain
--------------------------------------------------------
Tier 1 - `unitreerobotics/unitree_ros@dd4fa6866e523ad61324f658d63736e4eda3a6e4`
         `robots/g1_description/g1_29dof.xml`, vendored at models/unitree_g1/source/.
         Supplies the whole body tree: body positions/orientations, every inertial,
         all 29 joint names, axes, positions, ranges and actuatorfrcrange, the four
         per-foot contact spheres, the two shoulder cylinders, the imu site, the
         pelvis stand height and the 29 motor/joint transmissions.

Tier 2 - `unitreerobotics/unitree_mujoco@1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`
         `unitree_robots/g1/g1_29dof.xml`, vendored alongside it. Its body tree,
         inertials, joint ranges, axes and actuator force ranges are byte-equal to the
         Tier 1 model (verified: maxdiff 0.0 on body_pos/body_quat/body_mass/body_inertia/
         body_ipos/body_iquat/jnt_range/jnt_axis/jnt_pos/jnt_actfrcrange), and its meshes
         are byte-identical by SHA-256. It is therefore the same model with the dynamic
         augmentation an actual simulation needs, and this generator takes exactly that
         augmentation from it: joint armature 0.01, damping 0.05, frictionloss 0.2
         (0.1 for the four wrist pitch/yaw joints), and motor ctrlrange = the joint's
         own actuatorfrcrange.

Repository-authored, and declared as such in the model package
--------------------------------------------------------------
  * Geom names. The source leaves every geom unnamed; named contacts are required so a
    support evaluation can tell a foot from a torso.
  * Collision meshes replaced by their convex hulls (see scripts/prepare_unitree_g1_assets.py).
    MuJoCo collides mesh geoms through their convex hull, so this is collision-identical.
  * Source visual geoms dropped. They carry contype=0 conaffinity=0 density=0 and every
    body declares an explicit <inertial>, so they contribute nothing to the physics; the
    browser presentation is the canonical RoboBuddy Three.js rig, not MuJoCo meshes.
  * Two contact exclusions, listed and justified in SELF_CONTACT_EXCLUSIONS below.
  * The declared mount, the declared blocking stop bar, the floor, and the declared free
    external object.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
MODEL_DIR = REPO / "models" / "unitree_g1"
SOURCE_XML = MODEL_DIR / "source" / "g1_29dof.xml"
SOURCE_SHA256 = "599a3f3e55f92543a404944cd0501f8fbcba57a57210d1a1e42ef45839768c84"
UNITREE_MUJOCO_XML = MODEL_DIR / "source" / "g1_29dof_unitree_mujoco.xml"
UNITREE_MUJOCO_SHA256 = "423e28bd718b19f7a65cda539b6f794ddbb268b4b9bdbd85f4bd982b30729617"

# --- Tier 2 dynamic augmentation, read from the unitree_mujoco model -------------------
ARMATURE = 0.01
DAMPING = 0.05
FRICTIONLOSS_DEFAULT = 0.2
FRICTIONLOSS_WRIST = 0.1
WRIST_FRICTIONLOSS_JOINTS = (
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)

# --- numerical settings ----------------------------------------------------------------
# The pinned source model and the unitree_mujoco model both declare no <option>, so the
# nominal configuration is exactly MuJoCo's default that Unitree's own low-level simulator
# runs: 2 ms Euler, Newton solver, 100 iterations, 50 line-search iterations. The generator
# writes them explicitly so the compiled model can be asserted against the manifest.
TIMESTEP = 0.002
INTEGRATOR = "Euler"
ITERATIONS = 100
LS_ITERATIONS = 50

# --- source FixStand standing posture ---------------------------------------------------
# unitree_rl_mjlab@1425b15f73bd4095f0df53709d7c389c3eb9e790
#   deploy/robots/g1/config/config.yaml -> FSM.FixStand.qs[1]
# The same 29 values are the default_joint_pos of that repository's G1 velocity policy.
STAND_POSE = (
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
    0.0, 0.0, 0.0,
    0.35, 0.18, 0.0, 0.87, 0.0, 0.0, 0.0,
    0.35, -0.18, 0.0, 0.87, 0.0, 0.0, 0.0,
)

# Pelvis height at which the source foot contact spheres just touch z=0 in STAND_POSE,
# computed by forward kinematics from the source model (source pelvis height 0.793 minus
# the 0.0087978 m the lowest foot sphere sits above the floor at that pose).
STAND_PELVIS_Z = 0.7842
# Mounted fixture: high enough that every leg configuration in the source joint ranges
# stays clear of the floor.
MOUNT_PELVIS_Z = 1.20
# Deterministic gravity/fall drop: the neutral pose released above the floor.
DROP_PELVIS_Z = 0.95

JOINT_ORDER = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)

# Named foot contact primitives, in the source order of the four spheres per ankle_roll link.
FOOT_SPHERE_NAMES = ("heel_lateral", "heel_medial", "toe_lateral", "toe_medial")

# --- explicit, justified self-contact exclusions ----------------------------------------
# Everything else stays enabled, including every non-adjacent limb/body pair. MuJoCo's own
# parent-child filter already removes directly connected bodies.
SELF_CONTACT_EXCLUSIONS = (
    ("torso_link", "left_shoulder_roll_link",
     "The shoulder roll housing is mechanically seated inside the torso shell. The torso shell is "
     "concave at the shoulder, so its convex hull - which is what MuJoCo collides - fills the "
     "armpit and reports the housing as penetrating the torso from a source-legal 0.5 rad of "
     "adduction onward. The pair is excluded; torso vs shoulder_yaw, elbow and every distal arm "
     "body stays enabled, so arm-to-torso self-contact is still part of the plant."),
    ("torso_link", "right_shoulder_roll_link", "Mirror of the left exclusion, same justification."),
    ("left_wrist_roll_link", "left_wrist_yaw_link",
     "The three wrist bodies are one nested mechanical assembly; the roll and yaw hull envelopes "
     "overlap by at most 1.6 mm at the source joint limits. The intervening wrist_pitch_link is "
     "already parent-child filtered against both."),
    ("right_wrist_roll_link", "right_wrist_yaw_link", "Mirror of the left exclusion, same justification."),
)

# Mounted variants only. MuJoCo's parent-child contact filter is skipped when the parent body is
# welded to the world, and a mounted pelvis is exactly that. Without these three the mounted fixture
# would carry pelvis-to-hip and pelvis-to-waist hull grazes that the free-base plant does not have,
# so the fixture would not be testing the same plant. They restore the free-base behaviour and
# nothing more: the pairs are precisely the direct children of the pelvis.
MOUNTED_PARENT_RESTORE_EXCLUSIONS = (
    ("pelvis", "left_hip_pitch_link"),
    ("pelvis", "right_hip_pitch_link"),
    ("pelvis", "waist_yaw_link"),
)

# --- declared external object (free-base scenes) ----------------------------------------
EXTERNAL_OBJECT = {
    "name": "contact_probe_block",
    "halfExtentsM": (0.05, 0.05, 0.05),
    "massKg": 0.4,
    "posM": (0.30, 0.0, 0.05),
    "friction": 0.8,
}

# --- declared blocking wall (mounted_blocked) -------------------------------------------
# A visible rigid wall beside the mounted robot's left leg. It physically obstructs left hip
# abduction: with the pelvis mounted the whole left leg swings outward about the hip roll axis,
# so the leg presses flat against the wall face and the contact normal directly opposes the
# commanded motion. The wall is large in every other direction, so the leg cannot travel around
# or above it, and it is thick enough that the leg cannot end up inside it.
BLOCK_WALL = {
    "name": "hip_abduction_stop_wall",
    "joint": "left_hip_roll_joint",
    "posM": (0.0, 0.34, 0.55),
    "halfExtentsM": (0.45, 0.10, 0.45),
    # A near-rigid stop. The MuJoCo default contact is deliberately soft, and an 88 N m hip motor
    # would squeeze a soft wall out of the way, which would test the solver rather than the
    # obstruction. These are declared fixture contact parameters; no robot geom is changed.
    "solref": (0.004, 1),
    "solimp": (0.98, 0.9999, 0.001),
}

FLOAT_KEYS = {"pos", "quat", "size", "fromto", "range", "actuatorfrcrange", "axis", "ctrlrange",
              "diaginertia", "mass", "friction", "rgba", "solimp", "solref", "gravity"}


def fnum(value: float) -> str:
    text = f"{float(value):.9g}"
    return "0" if text in ("-0", "-0.0") else text


def fvec(values) -> str:
    return " ".join(fnum(v) for v in values)


def load_source() -> ET.Element:
    data = SOURCE_XML.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if digest != SOURCE_SHA256:
        raise SystemExit(f"{SOURCE_XML} SHA-256 {digest} does not match the pinned source {SOURCE_SHA256}")
    other = hashlib.sha256(UNITREE_MUJOCO_XML.read_bytes()).hexdigest()
    if other != UNITREE_MUJOCO_SHA256:
        raise SystemExit(f"{UNITREE_MUJOCO_XML} SHA-256 {other} does not match the pinned source {UNITREE_MUJOCO_SHA256}")
    return ET.fromstring(data.decode("utf-8"))


def source_pelvis(root: ET.Element) -> ET.Element:
    for worldbody in root.findall("worldbody"):
        for body in worldbody.findall("body"):
            if body.get("name") == "pelvis":
                return body
    raise SystemExit("pinned source model has no pelvis body")


def source_mesh_files(root: ET.Element) -> "dict[str, str]":
    files = {}
    for asset in root.findall("asset"):
        for mesh in asset.findall("mesh"):
            if mesh.get("name") and mesh.get("file"):
                files[mesh.get("name")] = mesh.get("file")
    return files


def is_visual(geom: ET.Element) -> bool:
    return geom.get("contype") == "0" and geom.get("conaffinity") == "0"


class Builder:
    """Rewrites the pinned source body tree into a named, collision-only physical tree."""

    def __init__(self, root: ET.Element):
        self.source_root = root
        self.mesh_files = source_mesh_files(root)
        self.collision_meshes: "list[str]" = []
        self.geom_names: "list[dict]" = []
        self.foot_geoms: "dict[str, list[str]]" = {"left": [], "right": []}

    def build_body(self, source: ET.Element, *, free_root: bool, root_pos) -> ET.Element:
        name = source.get("name")
        out = ET.Element("body", {"name": name})
        if root_pos is not None:
            out.set("pos", fvec(root_pos))
        elif source.get("pos"):
            out.set("pos", source.get("pos"))
        if source.get("quat"):
            out.set("quat", source.get("quat"))

        inertial = source.find("inertial")
        if inertial is not None:
            out.append(ET.Element("inertial", dict(inertial.attrib)))

        for joint in source.findall("joint"):
            if joint.get("type") == "free":
                if free_root:
                    out.append(ET.Element("joint", {
                        "name": "floating_base_joint", "type": "free",
                        "limited": "false", "actuatorfrclimited": "false",
                    }))
                continue
            attrib = dict(joint.attrib)
            jname = attrib["name"]
            attrib["class"] = "g1_wrist_motor" if jname in WRIST_FRICTIONLOSS_JOINTS else "g1_motor"
            out.append(ET.Element("joint", attrib))

        for site in source.findall("site"):
            out.append(ET.Element("site", dict(site.attrib)))

        sphere_index = 0
        cylinder_index = 0
        mesh_index = 0
        for geom in source.findall("geom"):
            if is_visual(geom):
                continue
            attrib = dict(geom.attrib)
            attrib.pop("group", None)
            attrib.pop("density", None)
            attrib.pop("rgba", None)
            gtype = attrib.get("type", "sphere")
            if gtype == "mesh":
                mesh = attrib["mesh"]
                if mesh not in self.collision_meshes:
                    self.collision_meshes.append(mesh)
                gname = f"{name}_hull" if mesh_index == 0 else f"{name}_hull{mesh_index}"
                mesh_index += 1
                attrib["class"] = "g1_collision"
            elif gtype == "cylinder":
                gname = f"{name}_cyl{cylinder_index}" if cylinder_index else f"{name}_cyl"
                cylinder_index += 1
                attrib["class"] = "g1_collision"
            else:
                side = "left" if name.startswith("left") else "right"
                gname = f"{side}_foot_{FOOT_SPHERE_NAMES[sphere_index]}"
                self.foot_geoms[side].append(gname)
                sphere_index += 1
                attrib["class"] = "g1_foot"
                attrib.pop("size", None)
            attrib["name"] = gname
            self.geom_names.append({"geom": gname, "body": name, "type": gtype})
            out.append(ET.Element("geom", attrib))

        for child in source.findall("body"):
            out.append(self.build_body(child, free_root=False, root_pos=None))
        return out


def indent(element: ET.Element, level: int = 0) -> None:
    pad = "\n" + "  " * level
    if len(element):
        if not (element.text or "").strip():
            element.text = pad + "  "
        for child in element:
            indent(child, level + 1)
            if not (child.tail or "").strip():
                child.tail = pad + "  "
        if not (element[-1].tail or "").strip():
            element[-1].tail = pad
    if level and not (element.tail or "").strip():
        element.tail = pad


def build_model(variant: str, source_root: ET.Element) -> "tuple[str, dict]":
    free_root = variant.startswith("freebase")
    if variant == "mounted" or variant == "mounted_blocked":
        pelvis_z = MOUNT_PELVIS_Z
        pose = [0.0] * 29
    elif variant == "freebase_drop":
        pelvis_z = DROP_PELVIS_Z
        pose = [0.0] * 29
    else:
        pelvis_z = STAND_PELVIS_Z
        pose = list(STAND_POSE)

    builder = Builder(source_root)
    pelvis = builder.build_body(source_pelvis(source_root), free_root=free_root, root_pos=(0.0, 0.0, pelvis_z))

    mujoco = ET.Element("mujoco", {"model": f"robobuddy_g1_29dof_{variant}"})
    ET.SubElement(mujoco, "compiler", {"angle": "radian", "meshdir": "assets", "autolimits": "true"})
    ET.SubElement(mujoco, "option", {
        "timestep": fnum(TIMESTEP), "integrator": INTEGRATOR,
        "iterations": str(ITERATIONS), "ls_iterations": str(LS_ITERATIONS),
    })

    default = ET.SubElement(mujoco, "default")
    motor = ET.SubElement(default, "default", {"class": "g1_motor"})
    ET.SubElement(motor, "joint", {"armature": fnum(ARMATURE), "damping": fnum(DAMPING), "frictionloss": fnum(FRICTIONLOSS_DEFAULT)})
    wrist = ET.SubElement(default, "default", {"class": "g1_wrist_motor"})
    ET.SubElement(wrist, "joint", {"armature": fnum(ARMATURE), "damping": fnum(DAMPING), "frictionloss": fnum(FRICTIONLOSS_WRIST)})
    collision = ET.SubElement(default, "default", {"class": "g1_collision"})
    ET.SubElement(collision, "geom", {"group": "3", "rgba": "0.35 0.55 0.75 0.35"})
    foot = ET.SubElement(default, "default", {"class": "g1_foot"})
    ET.SubElement(foot, "geom", {"type": "sphere", "size": "0.005", "group": "3", "rgba": "0.9 0.45 0.15 1"})
    fixture = ET.SubElement(default, "default", {"class": "robobuddy_fixture"})
    ET.SubElement(fixture, "geom", {"group": "4", "rgba": "0.85 0.25 0.25 0.55", "contype": "0", "conaffinity": "0", "density": "0"})

    asset = ET.SubElement(mujoco, "asset")
    ET.SubElement(asset, "texture", {"type": "2d", "name": "robobuddy_ground", "builtin": "checker", "mark": "edge",
                                     "rgb1": "0.2 0.3 0.4", "rgb2": "0.1 0.2 0.3", "markrgb": "0.8 0.8 0.8",
                                     "width": "300", "height": "300"})
    ET.SubElement(asset, "material", {"name": "robobuddy_ground", "texture": "robobuddy_ground",
                                      "texuniform": "true", "texrepeat": "5 5", "reflectance": "0.2"})
    for mesh in sorted(builder.collision_meshes):
        ET.SubElement(asset, "mesh", {"name": mesh, "file": builder.mesh_files[mesh]})

    worldbody = ET.SubElement(mujoco, "worldbody")
    ET.SubElement(worldbody, "light", {"pos": "0 0 3", "dir": "0 0 -1", "directional": "true"})
    ET.SubElement(worldbody, "geom", {"name": "floor", "type": "plane", "size": "0 0 0.05", "material": "robobuddy_ground"})

    fixtures = []
    if not free_root:
        # The support is the absence of a root joint: the pelvis is welded to the world by the
        # model itself. The post and clamp are declared, visible, non-colliding markers of that
        # weld, never a force that could be mistaken for balance.
        mount = ET.SubElement(worldbody, "body", {"name": "robobuddy_g1_mount", "pos": fvec((0, 0, 0))})
        ET.SubElement(mount, "geom", {"name": "mount_post", "class": "robobuddy_fixture", "type": "box",
                                      "size": fvec((0.05, 0.05, pelvis_z / 2)), "pos": fvec((-0.32, 0, pelvis_z / 2))})
        ET.SubElement(mount, "geom", {"name": "mount_arm", "class": "robobuddy_fixture", "type": "box",
                                      "size": fvec((0.16, 0.05, 0.04)), "pos": fvec((-0.16, 0, pelvis_z))})
        fixtures += ["robobuddy_g1_mount"]

    if variant == "mounted_blocked":
        wall = ET.SubElement(worldbody, "body", {"name": "robobuddy_g1_joint_stop", "pos": fvec(BLOCK_WALL["posM"])})
        ET.SubElement(wall, "geom", {"name": BLOCK_WALL["name"], "type": "box",
                                     "size": fvec(BLOCK_WALL["halfExtentsM"]), "rgba": "0.9 0.2 0.2 0.9",
                                     "group": "3", "density": "0",
                                     "solref": fvec(BLOCK_WALL["solref"]), "solimp": fvec(BLOCK_WALL["solimp"])})
        fixtures += ["robobuddy_g1_joint_stop"]

    worldbody.append(pelvis)

    objects = []
    if free_root and variant != "freebase_drop":
        block = ET.SubElement(worldbody, "body", {"name": EXTERNAL_OBJECT["name"], "pos": fvec(EXTERNAL_OBJECT["posM"])})
        ET.SubElement(block, "freejoint", {"name": f"{EXTERNAL_OBJECT['name']}_free"})
        ET.SubElement(block, "geom", {
            "name": f"{EXTERNAL_OBJECT['name']}_geom", "type": "box",
            "size": fvec(EXTERNAL_OBJECT["halfExtentsM"]), "mass": fnum(EXTERNAL_OBJECT["massKg"]),
            "friction": f"{fnum(EXTERNAL_OBJECT['friction'])} 0.005 0.0001", "rgba": "0.25 0.65 0.35 1",
        })
        objects.append(EXTERNAL_OBJECT["name"])

    contact = ET.SubElement(mujoco, "contact")
    for body1, body2, _ in SELF_CONTACT_EXCLUSIONS:
        ET.SubElement(contact, "exclude", {"name": f"exclude_{body1}__{body2}", "body1": body1, "body2": body2})
    if not free_root:
        for body1, body2 in MOUNTED_PARENT_RESTORE_EXCLUSIONS:
            ET.SubElement(contact, "exclude", {"name": f"exclude_{body1}__{body2}", "body1": body1, "body2": body2})

    actuator = ET.SubElement(mujoco, "actuator")
    force_range = {}
    for body in source_pelvis(source_root).iter("body"):
        for joint in body.findall("joint"):
            if joint.get("actuatorfrcrange"):
                force_range[joint.get("name")] = joint.get("actuatorfrcrange")
    for joint in JOINT_ORDER:
        if joint not in force_range:
            raise SystemExit(f"source model declares no actuatorfrcrange for {joint}")
        ET.SubElement(actuator, "motor", {"name": joint, "joint": joint, "ctrlrange": force_range[joint]})

    sensor = ET.SubElement(mujoco, "sensor")
    ET.SubElement(sensor, "framequat", {"name": "imu_quat", "objtype": "site", "objname": "imu"})
    ET.SubElement(sensor, "gyro", {"name": "imu_gyro", "site": "imu"})
    ET.SubElement(sensor, "accelerometer", {"name": "imu_acc", "site": "imu"})

    keyframe = ET.SubElement(mujoco, "keyframe")
    qpos = ([0.0, 0.0, pelvis_z, 1.0, 0.0, 0.0, 0.0] if free_root else []) + list(pose)
    if objects:
        qpos += list(EXTERNAL_OBJECT["posM"]) + [1.0, 0.0, 0.0, 0.0]
    ET.SubElement(keyframe, "key", {"name": "initial", "qpos": fvec(qpos)})

    indent(mujoco)
    xml = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n" + ET.tostring(mujoco, encoding="unicode") + "\n"
    meta = {
        "variant": variant,
        "rootMode": "free-base" if free_root else "fixed-mounted",
        "pelvisInitialZM": pelvis_z,
        "initialJointPose": {name: value for name, value in zip(JOINT_ORDER, pose)},
        "fixtures": fixtures,
        "objects": objects,
        "footGeoms": {"left": list(builder.foot_geoms["left"]), "right": list(builder.foot_geoms["right"])},
        "geoms": builder.geom_names,
        "collisionMeshes": sorted(builder.collision_meshes),
        "excludedPairs": [[a, b] for a, b, _ in SELF_CONTACT_EXCLUSIONS] + ([] if free_root else [list(pair) for pair in MOUNTED_PARENT_RESTORE_EXCLUSIONS]),
    }
    return xml, meta


VARIANTS = ("mounted", "mounted_blocked", "freebase", "freebase_drop")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="verify the committed models match this generator byte for byte")
    parser.add_argument("--meta", action="store_true", help="print the generated model metadata as JSON")
    args = parser.parse_args()

    source_root = load_source()
    failures = []
    meta_all = {}
    for variant in VARIANTS:
        xml, meta = build_model(variant, source_root)
        meta_all[variant] = meta
        target = MODEL_DIR / f"{variant}.xml"
        if args.check:
            current = target.read_text(encoding="utf-8") if target.exists() else None
            if current != xml:
                failures.append(str(target.relative_to(REPO)))
        elif not args.meta:
            target.write_text(xml, encoding="utf-8")
            print(f"wrote {target.relative_to(REPO)} ({len(xml)} bytes, sha256 {hashlib.sha256(xml.encode()).hexdigest()})")

    if args.meta:
        print(json.dumps(meta_all, indent=1))
        return 0
    if args.check:
        for failure in failures:
            print(f"{failure} differs from the deterministic generator output", file=sys.stderr)
        if failures:
            return 1
        print(f"unitree_g1 models verified: {len(VARIANTS)} variants match the generator")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
