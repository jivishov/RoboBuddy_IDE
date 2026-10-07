#!/usr/bin/env python3
"""Generate the pinned LeKiwi Phase 5B MuJoCo assets.

Every geometric constant below is derived from one of two pinned sources and is
classified in docs/physics/lekiwi-provenance.md:

  * SIGRobotics-UIUC/LeKiwi@efa608d7ee5a495a4803b1d28cd0c955b4f1e033
    (URDF/LeKiwi.urdf, URDF/JOINT_NAMES.md, URDF/meshes/*) - base geometry,
    wheel placement, wheel joint axes, wheel/chassis inertials, arm mount.
  * google-deepmind/mujoco_menagerie@8161bba264d7fa7c99ca301e91e7fb44737676ad
    robotstudio_so101 - the SO-ARM101 arm links/inertials already pinned by this
    repository as models/so101/manipulation.xml.

The generator exists so the repeated omni-wheel roller definitions stay
reproducible: CI regenerates the assets and requires byte-identical output.
Run:  python scripts/generate_lekiwi_models.py --check
"""
from __future__ import annotations

import argparse
import hashlib
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
OUT = REPO / "models" / "lekiwi"

# --- source-derived wheel geometry (LeKiwi@efa608d URDF + wheel mesh) -----------
WHEEL_RADIUS_M = 0.0508          # 4-Omni-Directional-Wheel mesh outer radius (4 inch)
WHEEL_HALF_WIDTH_M = 0.019414    # mesh half axial width
ROLLER_RADIUS_M = 0.009707       # one roller row is one roller diameter wide
ROLLER_CIRCLE_M = 0.041093       # WHEEL_RADIUS_M - ROLLER_RADIUS_M
ROLLER_ROW_OFFSET_M = 0.009707   # row centres at +/- half a row width
ROLLER_SEMI_AXIS_M = 0.0205      # ellipsoid semi-axis along the roller axis; the resulting
                                 # barrel profile tracks the source wheel envelope to <0.1 mm
                                 # over the +/-15 deg arc each roller actually carries
ROLLERS_PER_ROW = 6
ROLLER_ROW_STAGGER_RAD = math.pi / ROLLERS_PER_ROW
HUB_RADIUS_M = 0.030
WHEEL_MASS_KG = 1.354285         # URDF 4-Omni-Directional-Wheel_Single_Body mass
ROLLER_TOTAL_MASS_KG = 0.952624  # split chosen to match the URDF wheel spin inertia
HUB_MASS_KG = WHEEL_MASS_KG - ROLLER_TOTAL_MASS_KG
ROLLER_MASS_KG = ROLLER_TOTAL_MASS_KG / (2 * ROLLERS_PER_ROW)
URDF_WHEEL_SPIN_INERTIA = 0.00187981

# model frame: x forward, y left, z up; origin at the wheel centroid, axle height
WHEELS = (
    # name,            position,                          drive direction (unit)
    ("base_left_wheel", (0.059193, 0.092659, 0.0), (-0.866025, 0.5, 0.0)),
    ("base_back_wheel", (-0.119842, -0.000379, 0.0), (0.0, -1.0, 0.0)),
    ("base_right_wheel", (0.060649, -0.092280, 0.0), (0.866025, 0.5, 0.0)),
)

CHASSIS_MASS_KG = 6.658975
CHASSIS_COM_M = (0.001587, 0.011732, 0.006636)
CHASSIS_FULLINERTIA = (0.023017161, 0.023916263, 0.038405753,
                       0.001177072, 0.000923805, 0.000196823)
CHASSIS_RADIUS_M = 0.1075
CHASSIS_HALF_HEIGHT_M = 0.030
CHASSIS_GEOM_Z_M = 0.020

ARM_MOUNT_M = (0.034846, 0.000231, 0.039140)

# --- declared simulation parameters (estimated; see provenance) ----------------
TIMESTEP = 0.002
WHEEL_CTRL_RAD_S = 4.60061      # 3000 raw ST3215 ticks at 4096 ticks / 360 deg
WHEEL_FORCE_NM = 2.94           # ST3215/STS3215 30 kg cm class stall torque
WHEEL_KV = 2.2
WHEEL_ARMATURE = 0.010
WHEEL_DAMPING = 0.004
WHEEL_FRICTIONLOSS = 0.006
ROLLER_DAMPING = 2e-5
ROLLER_FRICTIONLOSS = 2e-5
ROLLER_ARMATURE = 2e-6
NOMINAL_FLOOR_FRICTION = 0.85
LOW_TRACTION_FLOOR_FRICTION = 0.02

FLOOR_Z_M = -WHEEL_RADIUS_M     # floor plane in the base body frame at rest
STAND_CLEARANCE_M = 0.020       # visible test-stand lift for the lifted-base gate


def f(value: float) -> str:
    text = f"{value:.9g}"
    return "0" if text in ("-0", "-0.0") else text


def vec(values) -> str:
    return " ".join(f(v) for v in values)


def roller_row(prefix: str, row: str, start_rad: float, z_offset: float) -> list[str]:
    a = start_rad
    pos = (ROLLER_CIRCLE_M * math.cos(a), ROLLER_CIRCLE_M * math.sin(a), z_offset)
    axis = (-math.sin(a), math.cos(a), 0.0)
    # The compiler runs in radian mode, so the replicate increment is radians.
    step = 2.0 * math.pi / ROLLERS_PER_ROW
    return [
        f'        <replicate count="{ROLLERS_PER_ROW}" euler="0 0 {f(step)}" sep="">',
        f'          <body name="{prefix}_roller_{row}" pos="{vec(pos)}">',
        f'            <joint name="{prefix}_roller_{row}" class="passive_roller" axis="{vec(axis)}"/>',
        f'            <geom name="{prefix}_roller_{row}" class="roller_contact" type="ellipsoid"'
        f' size="{f(ROLLER_RADIUS_M)} {f(ROLLER_RADIUS_M)} {f(ROLLER_SEMI_AXIS_M)}"'
        f' zaxis="{vec(axis)}" mass="{f(ROLLER_MASS_KG)}"/>',
        '          </body>',
        '        </replicate>',
    ]


def wheel_block(name: str, pos, drive) -> list[str]:
    prefix = name.replace("base_", "").replace("_wheel", "")
    prefix = f"lekiwi_{prefix}"
    lines = [
        f'      <body name="{prefix}_wheel" pos="{vec(pos)}" xyaxes="0 0 1 {vec(drive)}">',
        f'        <joint name="{name}" class="drive_wheel" axis="0 0 1"/>',
        f'        <geom name="{prefix}_wheel_hub" class="wheel_hub" type="cylinder"'
        f' size="{f(HUB_RADIUS_M)} {f(WHEEL_HALF_WIDTH_M)}" mass="{f(HUB_MASS_KG)}"/>',
    ]
    lines += roller_row(prefix, "a", 0.0, ROLLER_ROW_OFFSET_M)
    lines += roller_row(prefix, "b", ROLLER_ROW_STAGGER_RAD, -ROLLER_ROW_OFFSET_M)
    lines.append('      </body>')
    return lines


def defaults(floor_friction: float) -> list[str]:
    return [
        '  <default>',
        '    <default class="drive_wheel">',
        f'      <joint type="hinge" armature="{f(WHEEL_ARMATURE)}" damping="{f(WHEEL_DAMPING)}"'
        f' frictionloss="{f(WHEEL_FRICTIONLOSS)}"/>',
        f'      <velocity kv="{f(WHEEL_KV)}" ctrlrange="{f(-WHEEL_CTRL_RAD_S)} {f(WHEEL_CTRL_RAD_S)}"'
        f' forcerange="{f(-WHEEL_FORCE_NM)} {f(WHEEL_FORCE_NM)}"/>',
        '    </default>',
        '    <default class="passive_roller">',
        f'      <joint type="hinge" armature="{f(ROLLER_ARMATURE)}" damping="{f(ROLLER_DAMPING)}"'
        f' frictionloss="{f(ROLLER_FRICTIONLOSS)}"/>',
        '    </default>',
        '    <default class="roller_contact">',
        f'      <geom group="3" contype="2" conaffinity="1" condim="3"'
        f' friction="{f(floor_friction)} 0.005 0.0001" solref="0.008 1" rgba="0.20 0.22 0.26 1"/>',
        '    </default>',
        '    <default class="wheel_hub">',
        '      <geom group="3" contype="4" conaffinity="0" condim="3" rgba="0.55 0.57 0.60 1"/>',
        '    </default>',
        '    <default class="chassis">',
        '      <geom group="3" contype="4" conaffinity="0" condim="3" rgba="0.36 0.40 0.45 1"/>',
        '    </default>',
        '    <default class="lekiwi_floor">',
        f'      <geom type="plane" contype="1" conaffinity="7" condim="3"'
        f' friction="{f(floor_friction)} 0.005 0.0001" rgba="0.42 0.45 0.48 1"/>',
        '    </default>',
        '  </default>',
    ]


def header(model_name: str, floor_friction: float, timestep: float = TIMESTEP) -> list[str]:
    return [
        f'<mujoco model="{model_name}">',
        '  <compiler angle="radian" autolimits="true"/>',
        f'  <option integrator="implicitfast" timestep="{f(timestep)}" cone="elliptic"'
        ' iterations="20" ls_iterations="20" impratio="10" gravity="0 0 -9.81"/>',
    ] + defaults(floor_friction)


def base_body(pos_z: float, extra: list[str] | None = None) -> list[str]:
    lines = [
        f'    <body name="lekiwi_base" pos="0 0 {f(pos_z)}">',
        '      <freejoint name="lekiwi_base_free"/>',
        f'      <inertial pos="{vec(CHASSIS_COM_M)}" mass="{f(CHASSIS_MASS_KG)}"'
        f' fullinertia="{vec(CHASSIS_FULLINERTIA)}"/>',
        f'      <geom name="lekiwi_chassis" class="chassis" type="cylinder"'
        f' size="{f(CHASSIS_RADIUS_M)} {f(CHASSIS_HALF_HEIGHT_M)}" pos="0 0 {f(CHASSIS_GEOM_Z_M)}" mass="0"/>',
        '      <site name="lekiwi_base_origin" pos="0 0 0" size="0.005" group="3"/>',
    ]
    for name, pos, drive in WHEELS:
        lines += wheel_block(name, pos, drive)
    if extra:
        lines += extra
    lines.append('    </body>')
    return lines


def actuators() -> list[str]:
    lines = ['  <actuator>']
    for name, _pos, _drive in WHEELS:
        lines.append(f'    <velocity class="drive_wheel" name="{name}" joint="{name}"/>')
    lines.append('  </actuator>')
    return lines


def wheel_reference_xml() -> str:
    """One driven omni wheel on a declared low-friction reference carriage."""
    lines = header("robobuddy_lekiwi_wheel_reference", NOMINAL_FLOOR_FRICTION)
    name, _pos, drive = WHEELS[1]  # back wheel: drive direction along -y
    lines += [
        '  <worldbody>',
        '    <light pos="0 0 2" dir="0 0 -1"/>',
        f'    <geom name="reference_floor" class="lekiwi_floor" size="4 4 0.1" pos="0 0 {f(FLOOR_Z_M)}"/>',
        '    <!-- Declared single-wheel characterization rig: the carriage runs on frictionless',
        '         planar bearings (x, y, yaw) and a vertical bearing (z), so the wheel still has to',
        '         generate every propulsive force through roller/ground contact, but the rig cannot',
        '         tip. The bearings are reference-fixture geometry, not LeKiwi hardware. -->',
        '    <body name="wheel_carriage" pos="0 0 0">',
        '      <joint name="carriage_x" type="slide" axis="1 0 0"/>',
        '      <joint name="carriage_y" type="slide" axis="0 1 0"/>',
        '      <joint name="carriage_z" type="slide" axis="0 0 1"/>',
        '      <joint name="carriage_yaw" type="hinge" axis="0 0 1"/>',
        '      <inertial pos="0 0 0.02" mass="2.0" diaginertia="0.010 0.010 0.016"/>',
        '      <geom name="carriage_deck" class="chassis" type="box" size="0.09 0.06 0.008" pos="0 0 0.030" mass="0"/>',
    ]
    lines += wheel_block(name, (0.0, 0.0, 0.0), drive)
    lines += ['    </body>', '  </worldbody>']
    lines += ['  <actuator>',
              f'    <velocity class="drive_wheel" name="{name}" joint="{name}"/>',
              '  </actuator>', '</mujoco>', '']
    return "\n".join(lines)


def base_xml(model_name: str, floor_friction: float, stand: bool) -> str:
    lines = header(model_name, floor_friction)
    lines += ['  <worldbody>', '    <light pos="0 0 2" dir="0 0 -1"/>',
              f'    <geom name="lekiwi_floor" class="lekiwi_floor" size="6 6 0.1" pos="0 0 {f(FLOOR_Z_M)}"/>']
    if stand:
        # A visible, declared rigid test stand that holds every wheel clear of the floor.
        base_z = STAND_CLEARANCE_M
        stand_top = base_z + CHASSIS_GEOM_Z_M - CHASSIS_HALF_HEIGHT_M
        half = (stand_top - FLOOR_Z_M) / 2
        lines += [
            '    <!-- Declared visible test stand. The base rests on it, every wheel is'
            f' {f(STAND_CLEARANCE_M)} m clear of the floor, and wheel actuation therefore has no'
            ' ground to react against. -->',
            f'    <geom name="lekiwi_test_stand" type="cylinder" size="0.06 {f(half)}"'
            f' pos="0 0 {f(FLOOR_Z_M + half)}" contype="1" conaffinity="7" condim="3"'
            ' friction="1.0 0.02 0.001" group="3" rgba="0.85 0.55 0.15 1"/>',
        ]
        lines += base_body(base_z)
    else:
        lines += base_body(0.0)
    lines += ['  </worldbody>']
    lines += actuators()
    lines += ['</mujoco>', '']
    return "\n".join(lines)


# --- SO-ARM101 arm, pinned Menagerie robotstudio_so101 bodies/inertials --------
ARM_BLOCK = '''      <body name="arm_base" pos="{mount}" childclass="so101">
        <inertial pos="0.0137179 -5.19711e-05 0.0334843" mass="0.147"
          fullinertia="0.000114686 0.000136117 0.000130364 -4.59787e-07 4.97151e-06 9.75275e-08"/>
        <geom name="arm_base_proxy" type="box" size="0.045 0.045 0.03" pos="0 0 0.03" contype="0" conaffinity="0" group="2"/>
        <body name="arm_shoulder" pos="0.0388353 0 0.0624" quat="0 0 -1 0">
          <joint axis="0 0 1" name="arm_shoulder_pan" type="hinge" range="-1.91986 1.91986" class="sts3215"/>
          <inertial pos="-0.0307604 -1.7e-05 -0.0252713" mass="0.100006"
            fullinertia="8.3759e-05 8.10403e-05 2.39783e-05 7.55525e-08 -1.16342e-06 1.54663e-07"/>
          <geom name="arm_shoulder_motor_proxy" type="box" pos="-0.030399 0.000422 -0.0387" quat="1 1 1 -1" size="0.023 0.015 0.01"/>
          <geom name="arm_shoulder_holder_proxy" type="box" pos="-0.025 0 0" quat="1 1 -1 1" size="0.038 0.025 0.02"/>
          <body name="arm_upper" pos="-0.0303992 -0.0182778 -0.0542" quat="1 -1 -1 -1">
            <joint axis="0 0 1" name="arm_shoulder_lift" type="hinge" range="-1.7453293 1.7453293" class="sts3215"/>
            <inertial pos="-0.089847 -0.008382 0.018409" mass="0.103"
              fullinertia="4.08002e-05 0.000147318 0.000142487 -1.97819e-05 -4.03016e-08 8.97326e-09"/>
            <geom name="arm_upper_motor_proxy" type="box" pos="-0.06 0 0.02" quat="0 -1 1 0" size="0.01 0.07 0.03"/>
            <geom name="arm_upper_link_proxy" type="box" pos="-0.12 -0.014 0.0182" quat="0 1 0 0" size="0.01 0.02 0.015"/>
            <body name="arm_lower" pos="-0.11257 -0.028 0" quat="1 0 0 1">
              <joint axis="0 0 1" name="arm_elbow_flex" type="hinge" range="-1.69 1.69" class="sts3215"/>
              <inertial pos="-0.098070 0.0032438 0.018283" mass="0.104"
                fullinertia="2.87438e-05 0.000159844 0.00014529 7.41152e-06 1.26409e-06 -4.90188e-08"/>
              <geom name="arm_lower_link_proxy" type="box" pos="-0.05 0 0.0182" quat="0 1 0 0" size="0.07 0.01 0.03"/>
              <geom name="arm_lower_motor_proxy" type="box" pos="-0.125 0.005 0.018" quat="0 -1 0 0" size="0.023 0.013 0.018"/>
              <body name="arm_wrist" pos="-0.1349 0.0052 0" quat="1 0 0 -1">
                <joint axis="0 0 1" name="arm_wrist_flex" type="hinge" range="-1.658063 1.658063" class="sts3215"/>
                <inertial pos="-0.000103312 -0.0386143 0.0281156" mass="0.079"
                  fullinertia="3.68263e-05 2.5391e-05 2.1e-05 1.7893e-08 -5.28128e-08 3.6412e-06"/>
                <geom name="arm_wrist_motor_proxy" type="box" pos="0 -0.0424 0.0256" quat="1 1 1 -1" size="0.029 0.015 0.018"/>
                <geom name="arm_wrist_link_proxy" type="box" pos="0 -0.02 0.0191" quat="1 -1 -1 -1" size="0.027 0.01 0.03"/>
                <body name="arm_gripper_body" pos="5.55112e-17 -0.0611 0.0181" quat="0.0172091 -0.0172091 0.706897 0.706897">
                  <joint axis="0 0 1" name="arm_wrist_roll" type="hinge" range="-2.7438473 2.7438473" class="sts3215"/>
                  <inertial pos="0.000214 0.000245 -0.025187" mass="0.087"
                    fullinertia="2.75087e-05 4.33657e-05 3.45059e-05 -3.35241e-07 -5.7352e-06 -5.17847e-08"/>
                  <geom name="fixed_jaw_box1" class="gripper_collision" type="box" size="0.0325 0.015 0.015" pos="-0.0025 0 -0.022"/>
                  <geom name="fixed_jaw_box2" class="gripper_collision" type="box" size="0.01 0.015 0.005" pos="-0.024 0 -0.04"/>
                  <geom name="fixed_jaw_tip" class="gripper_collision" type="capsule" size="0.0011 0.002" pos="-0.009 0 -0.103" euler="1.57 0 0"/>
                  <geom name="fixed_jaw_pad1" class="gripper_collision" type="box" size="0.001 0.004 0.004" pos="-0.009 0 -0.0982"/>
                  <geom name="fixed_jaw_pad2" class="gripper_collision" type="box" size="0.001 0.005 0.006" pos="-0.0108 0 -0.0905"/>
                  <geom name="fixed_jaw_pad3" class="gripper_collision" type="box" size="0.001 0.009 0.008" pos="-0.0125 0 -0.0727"/>
                  <geom name="fixed_jaw_pad4" class="gripper_collision" type="box" size="0.001 0.01 0.008" pos="-0.0143 0 -0.053"/>
                  <site group="3" name="arm_tool_site" pos="0.012 -0.000218 -0.098127" quat="1 0 1 0"/>
                  <body name="arm_wrist_camera">
                    <geom name="arm_camera_box1" class="gripper_collision" type="box" size="0.015 0.015 0.003" pos="-0.0025 0.03 -0.03" mass="0.0040540541"/>
                    <geom name="arm_camera_box2" class="gripper_collision" type="box" size="0.021 0.021 0.003" pos="-0.001 0.06 -0.04" euler="-0.55 0 0" mass="0.0079459459"/>
                  </body>
                  <body name="arm_moving_jaw" pos="0.0202 0.0188 -0.0234" quat="1 1 0 0">
                    <joint axis="0 0 1" name="arm_gripper" type="hinge" range="-0.174533 1.7453292" class="sts3215"/>
                    <inertial pos="-0.001575 -0.0300244 0.0192755" mass="0.012"
                      fullinertia="6.61427e-06 1.89032e-06 5.28738e-06 -3.19807e-07 -5.90717e-09 -1.09945e-07"/>
                    <geom name="moving_jaw_box1" class="gripper_collision" type="box" size="0.01 0.01 0.015" pos="0 -0.013 0.019"/>
                    <geom name="moving_jaw_tip" class="gripper_collision" type="capsule" size="0.0011 0.002" pos="-0.0118 -0.0765 0.019" euler="0 1.57 0"/>
                    <geom name="moving_jaw_pad1" class="gripper_collision" type="box" size="0.001 0.004 0.004" pos="-0.0113 -0.076 0.01875"/>
                    <geom name="moving_jaw_pad2" class="gripper_collision" type="box" size="0.001 0.005 0.006" pos="-0.0093 -0.067 0.01875"/>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>'''


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="fail if committed assets differ")
    args = parser.parse_args()

    from lekiwi_courier_scene import courier_xml  # noqa: E402  (co-located helper)

    assets = {
        "wheel_reference.xml": wheel_reference_xml(),
        "base.xml": base_xml("robobuddy_lekiwi_base", NOMINAL_FLOOR_FRICTION, stand=False),
        "base_stand.xml": base_xml("robobuddy_lekiwi_base_stand", NOMINAL_FLOOR_FRICTION, stand=True),
        "base_lowtraction.xml": base_xml("robobuddy_lekiwi_base_lowtraction", LOW_TRACTION_FLOOR_FRICTION, stand=False),
        "courier.xml": courier_xml(),
    }
    OUT.mkdir(parents=True, exist_ok=True)
    failures = []
    for name, text in assets.items():
        path = OUT / name
        digest = hashlib.sha256(text.encode()).hexdigest()
        if args.check:
            current = path.read_text() if path.exists() else None
            if current != text:
                failures.append(name)
            print(f"{name}: sha256 {digest} {'OK' if current == text else 'DIFFERS'}")
        else:
            path.write_text(text)
            print(f"{name}: sha256 {digest}")
    if failures:
        print(f"Regenerated LeKiwi assets differ from the committed files: {', '.join(failures)}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    raise SystemExit(main())
