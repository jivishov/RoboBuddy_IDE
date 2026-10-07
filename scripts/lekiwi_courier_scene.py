"""LeKiwi Phase 5B courier workcell.

Task identity, educational objective, workcell envelope, beaker envelope, route
intent and success sequence come from the pinned legacy scenario
jivishov/RoboBuddy_AI@75fe2669c0ab0b029986de424c69162071174df8
missions/lab-assistant/v2/definitions/lekiwi/lekiwi-01-beaker-courier.json.

Legacy poses are given in a Three.js Y-up millimetre frame. The physical scene
uses the MuJoCo right-handed Z-up metre world with the floor at z = 0 and the
legacy home base at the world origin:

    physics (x, y, z) = (three_x/1000, -three_z/1000, three_y/1000)

The legacy simulation-state machine (attachment state, placement latch,
frame-visited events, discrete seating state) is NOT carried over.
"""
from __future__ import annotations

import math

import generate_lekiwi_models as G

# --- legacy configured workcell, converted (source: legacy scenario) ----------
WORKTOP_CENTER_M = (0.305, -0.390, 0.199)
WORKTOP_HALF_M = (0.145, 0.145, 0.012)
WORKTOP_TOP_Z_M = WORKTOP_CENTER_M[2] + WORKTOP_HALF_M[2]          # 0.211
BACK_PANEL_CENTER_M = (0.206, -0.390, 0.0995)
BACK_PANEL_HALF_M = (0.016, 0.125, 0.0995)
BEAKER_PICKUP_XY_M = (0.305, -0.292)
BEAKER_DELIVERY_XY_M = (0.242928, -0.370144)
DELIVERY_MARKER_HALF_M = (0.045, 0.045, 0.0005)
RESTRICTED_STOP_XY_M = (0.100, -0.650)
RESTRICTED_RADIUS_M = 0.150

# --- legacy configured beaker envelope, repaired to a hollow vessel -----------
BEAKER_OUTER_RADIUS_M = 0.037        # legacy 74 mm footprint
BEAKER_HEIGHT_M = 0.080              # legacy 80 mm height
BEAKER_WALL_HALF_THICK_M = 0.0015
BEAKER_WALL_RADIUS_M = 0.0355
BEAKER_WALL_HALF_TANGENT_M = 0.0138
BEAKER_WALL_HALF_HEIGHT_M = 0.0345
BEAKER_RIM_RADIUS_M = 0.0355
BEAKER_RIM_HALF_THICK_M = 0.0030
BEAKER_RIM_HALF_HEIGHT_M = 0.0060
BEAKER_RIM_Z_M = 0.0730
BEAKER_SEGMENTS = 8
BEAKER_MASS_KG = 0.060
BEAKER_COM_Z_M = 0.032
BEAKER_INERTIA = (5.6e-5, 5.6e-5, 4.2e-5)

# --- repaired physical route (see docs/physics/lekiwi-provenance.md) ----------
HOME_XY_M = (0.0, 0.0)
SERVICE_XY_M = (0.274, -0.055)
SERVICE_YAW_RAD = -math.pi / 2

BASE_SPAWN_Z_M = 0.0                 # base body frame sits at wheel-axle height


def f(v: float) -> str:
    return G.f(v)


def beaker_body() -> list[str]:
    lines = [
        f'    <body name="empty_beaker" pos="{f(BEAKER_PICKUP_XY_M[0])} {f(BEAKER_PICKUP_XY_M[1])}'
        f' {f(WORKTOP_TOP_Z_M)}">',
        '      <freejoint name="empty_beaker_free"/>',
        f'      <inertial pos="0 0 {f(BEAKER_COM_Z_M)}" mass="{f(BEAKER_MASS_KG)}"'
        f' diaginertia="{G.vec(BEAKER_INERTIA)}"/>',
        f'      <geom name="beaker_bottom" class="beaker" type="cylinder"'
        f' size="{f(BEAKER_OUTER_RADIUS_M)} 0.0015" pos="0 0 0.0015" mass="0"/>',
    ]
    for index in range(BEAKER_SEGMENTS):
        angle = 2 * math.pi * index / BEAKER_SEGMENTS
        cx, cy = BEAKER_WALL_RADIUS_M * math.cos(angle), BEAKER_WALL_RADIUS_M * math.sin(angle)
        lines.append(
            f'      <geom name="beaker_wall_{index}" class="beaker" type="box"'
            f' size="{f(BEAKER_WALL_HALF_THICK_M)} {f(BEAKER_WALL_HALF_TANGENT_M)} {f(BEAKER_WALL_HALF_HEIGHT_M)}"'
            f' pos="{f(cx)} {f(cy)} {f(0.003 + BEAKER_WALL_HALF_HEIGHT_M)}" euler="0 0 {f(angle)}" mass="0"/>')
    for index in range(BEAKER_SEGMENTS):
        angle = 2 * math.pi * index / BEAKER_SEGMENTS
        cx, cy = BEAKER_RIM_RADIUS_M * math.cos(angle), BEAKER_RIM_RADIUS_M * math.sin(angle)
        lines.append(
            f'      <geom name="beaker_rim_{index}" class="beaker_grip" type="box"'
            f' size="{f(BEAKER_RIM_HALF_THICK_M)} {f(BEAKER_WALL_HALF_TANGENT_M)} {f(BEAKER_RIM_HALF_HEIGHT_M)}"'
            f' pos="{f(cx)} {f(cy)} {f(BEAKER_RIM_Z_M)}" euler="0 0 {f(angle)}" mass="0"/>')
    lines.append('    </body>')
    return lines


def courier_xml() -> str:
    lines = G.header("robobuddy_lekiwi_courier", G.NOMINAL_FLOOR_FRICTION)
    # extra defaults for the workcell and the SO-ARM101 arm
    lines = lines[:-1] + [
        '    <default class="so101">',
        '      <joint damping="1" frictionloss="0.1" armature="0.005"/>',
        '      <geom group="3" condim="3" contype="1" conaffinity="1" rgba="0.72 0.60 0.18 1"/>',
        '    </default>',
        '    <default class="sts3215">',
        '      <joint damping="0.60" frictionloss="0.052" armature="0.028"/>',
        '      <position kp="998.22" kv="2.731" forcerange="-2.94 2.94"/>',
        '    </default>',
        '    <default class="gripper_collision">',
        '      <geom group="3" condim="6" contype="1" conaffinity="1" friction="1 5e-3 5e-4"'
        ' solref="0.01 1" priority="1" rgba="0.78 0.58 0.12 1"/>',
        '    </default>',
        '    <default class="workcell">',
        '      <geom group="3" condim="3" contype="1" conaffinity="7" friction="0.8 0.005 0.0001"'
        ' solref="0.01 1" rgba="0.52 0.56 0.60 1"/>',
        '    </default>',
        '    <default class="beaker">',
        '      <geom group="3" condim="6" contype="1" conaffinity="1" friction="0.9 0.01 0.0005"'
        ' solref="0.01 1" rgba="0.62 0.78 0.88 0.85"/>',
        '    </default>',
        '    <default class="beaker_grip">',
        '      <geom group="3" condim="6" contype="1" conaffinity="1" friction="1.1 0.02 0.001"'
        ' solref="0.008 1" priority="1" rgba="0.55 0.74 0.86 0.95"/>',
        '    </default>',
        '  </default>',
    ]
    lines += [
        '  <worldbody>',
        '    <light pos="0.3 -0.3 2" dir="0 0 -1"/>',
        f'    <geom name="lekiwi_floor" class="lekiwi_floor" size="6 6 0.1" pos="0 0 0"/>',
        '    <!-- Configured 290 x 290 mm laboratory transfer bench, legacy lekiwi-01-beaker-courier. -->',
        f'    <geom name="lekiwi_transfer_worktop" class="workcell" type="box"'
        f' size="{G.vec(WORKTOP_HALF_M)}" pos="{G.vec(WORKTOP_CENTER_M)}"/>',
        f'    <geom name="lekiwi_transfer_back_panel" class="workcell" type="box"'
        f' size="{G.vec(BACK_PANEL_HALF_M)}" pos="{G.vec(BACK_PANEL_CENTER_M)}" rgba="0.40 0.43 0.47 1"/>',
        '    <!-- Painted receiving-zone marker. It carries no load and has no collision geometry. -->',
        f'    <geom name="lekiwi_delivery_marker" type="box" size="{G.vec(DELIVERY_MARKER_HALF_M)}"'
        f' pos="{f(BEAKER_DELIVERY_XY_M[0])} {f(BEAKER_DELIVERY_XY_M[1])} {f(WORKTOP_TOP_Z_M + 0.0005)}"'
        ' contype="0" conaffinity="0" group="2" rgba="0.16 0.72 0.36 0.55"/>',
    ]
    lines += beaker_body()
    arm = G.ARM_BLOCK.format(mount=G.vec(G.ARM_MOUNT_M))
    lines += G.base_body(BASE_SPAWN_Z_M + G.WHEEL_RADIUS_M, extra=arm.split("\n"))
    lines += ['  </worldbody>']
    lines += ['  <actuator>']
    for name, _pos, _drive in G.WHEELS:
        lines.append(f'    <velocity class="drive_wheel" name="{name}" joint="{name}"/>')
    for joint, ctrl in (
        ("arm_shoulder_pan", "-1.91986 1.91986"),
        ("arm_shoulder_lift", "-1.74533 1.74533"),
        ("arm_elbow_flex", "-1.69 1.69"),
        ("arm_wrist_flex", "-1.65806 1.65806"),
        ("arm_wrist_roll", "-2.74385 2.74385"),
        ("arm_gripper", "-0.17453 1.74533"),
    ):
        lines.append(f'    <position class="sts3215" name="{joint}" joint="{joint}" ctrlrange="{ctrl}"/>')
    lines.append('  </actuator>')
    lines += ['</mujoco>', '']
    return "\n".join(lines)
