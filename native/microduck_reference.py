#!/usr/bin/env python3
"""MicroDuck Phase 5C native MuJoCo reference runner.

It executes the same pinned model assets and the same deployed controller contract as the
browser workspace, and emits a machine-readable run report.

The controller here is derived independently of the browser implementation, from the pinned
deployed sources:

    pollen-robotics/microduck@590b986  duck-control/src/obs.rs      the 61-value observation
                                       duck-control/src/model.rs    joint order, home pose
                                       robotd/src/control.rs        scaling, filters, skills

so a conformance fixture generated here is an independent reference, not the browser's own
output played back at it.

Scope of the evidence it produces:
  * software verification - observation assembly, action mapping, actuator output and
    controller state are implemented correctly;
  * numerical/model checks - the formulation behaves coherently and survives a tighter
    timestep;
  * native/browser conformance - the same model and controller agree across two backends.

It is NOT hardware validation. Native and browser agreement is implementation evidence only,
because both run the same model and the same approximations.

Usage:
    python native/microduck_reference.py --trial walk
    python native/microduck_reference.py --trial all --json out.json
    python native/microduck_reference.py --fixture assets/microduck/fixtures/controller-conformance.json
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

EXPECTED_MUJOCO_VERSION = "3.11.0"
ROOT = Path(__file__).resolve().parents[1]
POLICY_DIR = ROOT / "assets" / "microduck" / "policies"

# ---------------------------------------------------------------------------
# Deployed controller contract (duck-control/src/model.rs, obs.rs; robotd/control.rs)
# ---------------------------------------------------------------------------
WIRE_JOINT_ORDER = (
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
)
MOUTH_WIRE_INDEX = 9
POLICY_JOINT_ORDER = tuple(n for i, n in enumerate(WIRE_JOINT_ORDER) if i != MOUTH_WIRE_INDEX)
HOME_RAD = np.array([
    0.0, -0.0873, -0.4579, -0.0049, 0.4530,
    0.3491, 0.3491, 0.0, 0.0,
    0.0, 0.0873, 0.4579, 0.0049, -0.4530,
], dtype=np.float64)
HEAD_SLOTS = (5, 6, 7, 8)

OBS_LEN = 61
ACTION_LEN = 14

PHYSICS_TIMESTEP = 0.005
CONTROL_DECIMATION = 4

ACTION_SCALE = 0.9            # robotd Tuning::action_scale
STANDING_ACTION_SCALE = 1.0   # robotd Tuning::standing_action_scale
HEAD_LOWPASS = 0.5
LEGS_LOWPASS = 0.7
STANDING_THRESHOLD = 0.05

# The deployed daemon writes a firmware position-P-gain register on every servo every tick
# (duck-control/src/bus.rs `write_position_p_gain`), and it is not constant: standing, kicks
# and the sit/rise cycle run at `standing_gain_ratio` 0.8 of the running gain.
#
# microduck_rl pins the correspondence in the model itself. The `chosen_actuator` class the
# robot's 29 joints use carries `<!-- 200 kp -->` directly above `kp="0.55"`, and the
# commented-out alternative carries `<!-- 125 kp -->` above `kp="0.35"`. Two points, and the
# relationship through them is proportional to within 2%, so a firmware gain maps onto the
# identified stiffness by simple ratio. The same pair shows the torque limit does NOT move
# with the gain: the 125 kp variant keeps `forcerange="-0.96 0.96"`.
NOMINAL_FIRMWARE_GAIN = 200
IDENTIFIED_KP = 0.55
STANDING_GAIN_RATIO = 0.8


def kp_for_firmware_gain(gain):
    """The identified MuJoCo stiffness for a deployed firmware gain."""
    return IDENTIFIED_KP * float(gain) / float(NOMINAL_FIRMWARE_GAIN)

POLICY_FILES = {
    "walking": "alpha_walking.onnx",
    "stand": "alpha_stand.onnx",
    "sitstand": "alpha_sitstand.onnx",
    "ground_pick": "alpha_ground_pick.onnx",
    "kick_left": "ball_kick_left.onnx",
    "kick_right": "ball_kick_right.onnx",
    "roulade": "roulade.onnx",
    "roller": "roller.onnx",
    "roller_crouch": "roller_crouch.onnx",
}

MODELS = {
    "walk": ROOT / "models" / "microduck" / "walk.xml",
    "lowtraction": ROOT / "models" / "microduck" / "walk_lowtraction.xml",
    "groundcontact": ROOT / "models" / "microduck" / "groundcontact.xml",
    "kick": ROOT / "models" / "microduck" / "kick.xml",
}


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def quat_rotate_inverse(quat, vec):
    """Rotate `vec` by the inverse of `quat` (w, x, y, z) - infer_policy.py's helper."""
    w = quat[0]
    xyz = np.asarray(quat[1:4], dtype=np.float64)
    t = np.cross(xyz, vec) * 2.0
    return vec - w * t + np.cross(xyz, t)


def projected_gravity(quat):
    """World -Z in the trunk frame, normalised.

    duck-control/src/imu.rs builds the observation's gravity block as
    `normalise(rotate_inverse(quat, [0, 0, -1]))` and says why: "in steady state gravity
    must be a unit vector at any orientation - the policy observes it directly and was
    trained on normalised input". For a unit quaternion the normalisation is a no-op, but
    a declared setup perturbation can hand us a quaternion that is not quite unit, and
    then the policy would see a short gravity vector it has never been trained on.
    """
    g = quat_rotate_inverse(np.asarray(quat, dtype=np.float64), np.array([0.0, 0.0, -1.0]))
    norm = float(np.linalg.norm(g))
    return g / norm if norm > 0.0 else g


def build_observation(gyro, projected_gravity, joint_pos, joint_vel, last_action, command):
    """duck-control/src/obs.rs Observation::build, verbatim in layout and order."""
    obs = np.zeros(OBS_LEN, dtype=np.float32)
    obs[0:3] = gyro
    obs[3:6] = projected_gravity
    obs[6:20] = np.asarray(joint_pos, dtype=np.float64) - HOME_RAD
    obs[20:34] = joint_vel
    obs[34:48] = last_action
    twist = command.get("twist", (0.0, 0.0, 0.0))
    head = command.get("head", (0.0, 0.0, 0.0, 0.0))
    body = command.get("body", {})
    obs[48:51] = twist
    obs[51:55] = head
    obs[55] = 0.0                          # body x - unbound in training
    obs[56] = 0.0                          # body y - unbound in training
    obs[57] = float(body.get("z", 0.0))
    obs[58] = float(body.get("roll", 0.0))
    obs[59] = float(body.get("pitch", 0.0))
    obs[60] = 0.0                          # body yaw - unbound in training
    return obs


def scatter_action(action):
    """Map fourteen policy outputs onto the fifteen wire slots, skipping the mouth."""
    wire = np.zeros(len(WIRE_JOINT_ORDER), dtype=np.float64)
    for slot, value in enumerate(action):
        wire[slot if slot < MOUTH_WIRE_INDEX else slot + 1] = value
    return wire


class ReferenceController:
    """The deployed controller state: previous raw action and previous filtered targets."""

    def __init__(self, sessions, action_scale=ACTION_SCALE, filters=True):
        self.sessions = sessions
        self.action_scale = action_scale
        self.filters = filters
        self.reset()

    def reset(self):
        self.last_action = np.zeros(ACTION_LEN, dtype=np.float32)
        self.previous_targets = None

    def infer(self, policy_id, obs):
        session = self.sessions[policy_id]
        name_in = session.get_inputs()[0].name
        name_out = session.get_outputs()[0].name
        out = session.run([name_out], {name_in: obs.reshape(1, OBS_LEN)})[0]
        return out.squeeze(0).astype(np.float32)

    def step(self, policy_id, gyro, gravity, joint_pos, joint_vel, command, standing_tuned=False):
        obs = build_observation(gyro, gravity, joint_pos, joint_vel, self.last_action, command)
        action = self.infer(policy_id, obs)
        self.last_action = action.copy()
        scale = STANDING_ACTION_SCALE if standing_tuned else self.action_scale
        wire = scatter_action(action)
        targets = np.array([
            HOME_RAD[slot] + scale * wire[slot if slot < MOUTH_WIRE_INDEX else slot + 1]
            for slot in range(ACTION_LEN)
        ], dtype=np.float64)
        if self.filters and self.previous_targets is not None:
            for slot in range(ACTION_LEN):
                alpha = HEAD_LOWPASS if slot in HEAD_SLOTS else LEGS_LOWPASS
                targets[slot] = alpha * targets[slot] + (1.0 - alpha) * self.previous_targets[slot]
        self.previous_targets = targets.copy()
        return obs, action, targets, scale


class Plant:
    """The authoritative physics. Nothing outside declared setup writes state."""

    def __init__(self, model_key, timestep=PHYSICS_TIMESTEP):
        path = MODELS[model_key]
        self.path = path
        self.sha256 = sha256_file(path)
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.model.opt.timestep = timestep
        self.data = mujoco.MjData(self.model)
        m, d = self.model, self.data
        self.trunk = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "trunk_base")
        self.gyro_adr = m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "imu_ang_vel")]
        j0 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, POLICY_JOINT_ORDER[0])
        self.qadr = m.jnt_qposadr[j0]
        self.vadr = m.jnt_dofadr[j0]
        self.left_foot = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "left_foot_collision")
        self.right_foot = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "right_foot_collision")
        self.ball_geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "microduck_ball_geom")
        self.ball_joint = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "microduck_ball_free")
        self.setup_log = []
        self.nominal_kp = float(m.actuator_gainprm[0, 0]) if m.nu else IDENTIFIED_KP
        self.applied_kp = self.nominal_kp
        self.reset()

    # -- actuation --------------------------------------------------------------
    def set_actuator_kp(self, kp):
        """Set the position-servo stiffness on every joint, as the daemon sets the register.

        A MuJoCo `position` actuator applies `gainprm[0] * ctrl + biasprm[1] * qpos`, so kp
        lives in both. `forcerange` is deliberately left alone: the source's own 125 kp
        variant keeps the same torque limit, so the gain moves and the limit does not.

        kp = 0 is a true torque-off: the actuator produces no force whatever ctrl says. That
        is a different thing from commanding zero radians, which is a full-strength command
        to a straight-legged pose.
        """
        kp = float(kp)
        for i in range(self.model.nu):
            self.model.actuator_gainprm[i, 0] = kp
            self.model.actuator_biasprm[i, 1] = -kp
        self.applied_kp = kp

    def actuator_force_total(self):
        return float(np.abs(np.asarray(self.data.actuator_force)).sum())

    # -- declared setup / reset -------------------------------------------------
    def reset(self):
        key = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "HOME")
        mujoco.mj_resetDataKeyframe(self.model, self.data, key)
        self.set_actuator_kp(self.nominal_kp)
        mujoco.mj_forward(self.model, self.data)
        self.setup_log.append({"event": "reset", "simulationTime": float(self.data.time)})

    def setup_orientation(self, quat_wxyz, label):
        """A declared pre-trial perturbation. Logged as setup; it is not task progress."""
        self.data.qpos[3:7] = quat_wxyz
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)
        self.setup_log.append({"event": "setup_trunk_orientation", "label": label,
                               "quaternionWxyz": [float(v) for v in quat_wxyz],
                               "simulationTime": float(self.data.time)})

    def setup_ball(self, xy, label="ball_placement"):
        if self.ball_joint < 0:
            raise SystemExit("this model carries no ball")
        adr = self.model.jnt_qposadr[self.ball_joint]
        dof = self.model.jnt_dofadr[self.ball_joint]
        self.data.qpos[adr:adr + 3] = [xy[0], xy[1], 0.035]
        self.data.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
        self.data.qvel[dof:dof + 6] = 0
        mujoco.mj_forward(self.model, self.data)
        self.setup_log.append({"event": "setup_ball", "label": label,
                               "positionM": [float(xy[0]), float(xy[1]), 0.035],
                               "simulationTime": float(self.data.time)})

    def settle(self, seconds, hold_home=True):
        """Let a declared perturbation settle with the servos holding the home pose."""
        if hold_home:
            self.data.ctrl[:] = HOME_RAD
        for _ in range(int(round(seconds / self.model.opt.timestep))):
            mujoco.mj_step(self.model, self.data)
        self.setup_log.append({"event": "settle", "seconds": seconds,
                               "simulationTime": float(self.data.time)})

    # -- observation ------------------------------------------------------------
    def observe(self):
        d = self.data
        return dict(
            gyro=d.sensordata[self.gyro_adr:self.gyro_adr + 3].astype(np.float64).copy(),
            gravity=projected_gravity(d.xquat[self.trunk].astype(np.float64)),
            joint_pos=d.qpos[self.qadr:self.qadr + ACTION_LEN].astype(np.float64).copy(),
            joint_vel=d.qvel[self.vadr:self.vadr + ACTION_LEN].astype(np.float64).copy(),
        )

    def trunk_state(self):
        d = self.data
        return dict(positionM=[float(v) for v in d.qpos[0:3]],
                    quaternionWxyz=[float(v) for v in d.qpos[3:7]])

    def tilt_deg(self):
        g = quat_rotate_inverse(self.data.xquat[self.trunk].astype(np.float64),
                                np.array([0.0, 0.0, -1.0]))
        return math.degrees(math.acos(max(-1.0, min(1.0, -g[2]))))

    def ball_position(self):
        if self.ball_joint < 0:
            return None
        adr = self.model.jnt_qposadr[self.ball_joint]
        return np.array(self.data.qpos[adr:adr + 3], dtype=np.float64)

    def advance(self, steps, contact_sink=None):
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
            if contact_sink is not None:
                self._collect_contacts(contact_sink)

    def _collect_contacts(self, sink):
        d = self.data
        touching = {"left": False, "right": False}
        for c in range(d.ncon):
            g1, g2 = int(d.contact[c].geom1), int(d.contact[c].geom2)
            pair = (g1, g2)
            if self.left_foot in pair:
                touching["left"] = True
            if self.right_foot in pair:
                touching["right"] = True
            if self.ball_geom >= 0 and self.ball_geom in pair and (self.left_foot in pair or self.right_foot in pair):
                force = np.zeros(6)
                mujoco.mj_contactForce(self.model, d, c, force)
                sink["foot_ball_contacts"].append({
                    "simulationTime": round(float(d.time), 5),
                    "foot": "right" if self.right_foot in pair else "left",
                    "normalForceN": round(float(abs(force[0])), 5),
                    "geoms": [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g1),
                              mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g2)],
                })
        sink["steps"] += 1
        for side in ("left", "right"):
            if touching[side]:
                sink[f"{side}_stance_steps"] += 1
            else:
                sink[f"{side}_air_steps"] += 1
        if touching["left"] != sink["last_left"]:
            sink["left_transitions"] += 1
            sink["last_left"] = touching["left"]
        if touching["right"] != sink["last_right"]:
            sink["right_transitions"] += 1
            sink["last_right"] = touching["right"]


def new_contact_sink():
    return {"steps": 0, "left_stance_steps": 0, "right_stance_steps": 0,
            "left_air_steps": 0, "right_air_steps": 0,
            "left_transitions": 0, "right_transitions": 0,
            "last_left": True, "last_right": True, "foot_ball_contacts": []}


def summarise_contacts(sink):
    steps = max(sink["steps"], 1)
    return {
        "physicsSteps": sink["steps"],
        "leftStanceFraction": round(sink["left_stance_steps"] / steps, 4),
        "rightStanceFraction": round(sink["right_stance_steps"] / steps, 4),
        "leftAirFraction": round(sink["left_air_steps"] / steps, 4),
        "rightAirFraction": round(sink["right_air_steps"] / steps, 4),
        "leftContactTransitions": sink["left_transitions"],
        "rightContactTransitions": sink["right_transitions"],
        "footBallContacts": sink["foot_ball_contacts"],
    }


def load_sessions(policy_ids):
    import onnxruntime as ort
    sessions = {}
    hashes = {}
    for pid in policy_ids:
        path = POLICY_DIR / POLICY_FILES[pid]
        sessions[pid] = ort.InferenceSession(str(path))
        hashes[pid] = sha256_file(path)
        shape_in = sessions[pid].get_inputs()[0].shape
        shape_out = sessions[pid].get_outputs()[0].shape
        if list(shape_in[-1:]) != [OBS_LEN] or list(shape_out[-1:]) != [ACTION_LEN]:
            raise SystemExit(f"{pid}: policy declares {shape_in} -> {shape_out}, expected [1,61] -> [1,14]")
    return sessions, hashes


def run_trial(*, model_key, policy_id, seconds, command, timestep=PHYSICS_TIMESTEP,
              decimation=CONTROL_DECIMATION, actuation=True, standing_tuned=False,
              perturbation=None, settle_seconds=0.0, ball_xy=None, action_scale=ACTION_SCALE,
              filters=True, command_fn=None, sessions=None, hashes=None):
    plant = Plant(model_key, timestep=timestep)
    if ball_xy is not None:
        plant.setup_ball(ball_xy)
    if perturbation is not None:
        plant.setup_orientation(perturbation, "declared pre-trial orientation")
        if settle_seconds:
            plant.settle(settle_seconds)
    controller = ReferenceController(sessions, action_scale=action_scale, filters=filters)

    start = np.array(plant.data.qpos[0:3], dtype=np.float64)
    start_yaw = plant.data.qpos[3:7].copy()
    ball_start = plant.ball_position()
    sink = new_contact_sink()
    trunk_z = []
    effort_sum = 0.0
    effort_peak = 0.0
    ticks = int(round(seconds / (timestep * decimation)))
    for _ in range(ticks):
        obs = plant.observe()
        cmd = command_fn(plant.data.time) if command_fn else command
        _, _, targets, _ = controller.step(policy_id, obs["gyro"], obs["gravity"],
                                           obs["joint_pos"], obs["joint_vel"], cmd,
                                           standing_tuned=standing_tuned)
        # The controller's own gain schedule reaches the servos, exactly as the daemon writes
        # it to the firmware register: standing tuning runs at 0.8 of the running gain.
        #
        # A declared torque-off zeroes that gain. It does NOT zero ctrl: commanding 0 rad is
        # a full-strength command to a straight-legged pose, which is actuation, not its
        # absence. The targets stay published so the trial still shows what the policy asked
        # for while the motors produced nothing.
        gain = round(NOMINAL_FIRMWARE_GAIN * STANDING_GAIN_RATIO) if standing_tuned else NOMINAL_FIRMWARE_GAIN
        plant.set_actuator_kp(kp_for_firmware_gain(gain) if actuation else 0.0)
        plant.data.ctrl[:] = targets
        plant.advance(decimation, sink)
        effort = plant.actuator_force_total()
        effort_sum += effort
        effort_peak = max(effort_peak, effort)
        trunk_z.append(float(plant.data.qpos[2]))

    end = np.array(plant.data.qpos[0:3], dtype=np.float64)
    disp = end - start
    # Distance along the commanded direction. On a reduced-traction surface a robot can slide
    # a long way sideways while making no commanded progress at all, so the raw planar
    # distance would understate how completely the propulsion has gone.
    base_twist = (command_fn(0.0) if command_fn else command).get("twist", (0.0, 0.0, 0.0))
    axis_norm = math.hypot(base_twist[0], base_twist[1])
    commanded_axis = (
        round(float((disp[0] * base_twist[0] + disp[1] * base_twist[1]) / axis_norm), 6)
        if axis_norm > 1e-9 else None
    )
    yaw0 = 2 * math.atan2(start_yaw[3], start_yaw[0])
    yaw1 = 2 * math.atan2(plant.data.qpos[6], plant.data.qpos[3])
    result = {
        "collisionPlant": model_key,
        "modelPackageAsset": str(plant.path.relative_to(ROOT)),
        "modelSha256": plant.sha256,
        "policy": policy_id,
        "policySha256": hashes[policy_id],
        "engine": {"name": "mujoco", "version": mujoco.__version__,
                   "timestepSeconds": timestep, "controlDecimation": decimation,
                   "controlIntervalSeconds": timestep * decimation},
        "controller": {"actionScale": action_scale, "standingTuned": standing_tuned,
                       "headLowpassAlpha": HEAD_LOWPASS if filters else None,
                       "legsLowpassAlpha": LEGS_LOWPASS if filters else None,
                       "firmwareGain": 0 if not actuation else (
                           round(NOMINAL_FIRMWARE_GAIN * STANDING_GAIN_RATIO) if standing_tuned
                           else NOMINAL_FIRMWARE_GAIN),
                       "appliedKp": None if plant.applied_kp is None else round(plant.applied_kp, 6)},
        "requested": {"command": command if command_fn is None else "time-varying",
                      "actuationEnabled": actuation, "durationSeconds": seconds},
        "setupLog": plant.setup_log,
        "measured": {
            "simulationSeconds": round(float(plant.data.time), 6),
            "displacementM": [round(float(v), 6) for v in disp],
            "planarDistanceM": round(float(math.hypot(disp[0], disp[1])), 6),
            "averageSpeedMS": round(float(math.hypot(disp[0], disp[1]) / seconds), 6),
            "yawChangeRad": round(float(math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))), 6),
            # Summed |actuator_force| over the run. A declared torque-off must be exactly
            # zero here; commanding zero radians instead leaves this firmly non-zero, which
            # is how the two were told apart in the first place.
            "meanActuatorForceNm": round(effort_sum / max(ticks, 1), 6),
            "peakActuatorForceNm": round(effort_peak, 6),
            "finalTrunkHeightM": round(float(end[2]), 6),
            "minTrunkHeightM": round(float(min(trunk_z)), 6),
            "maxTrunkHeightM": round(float(max(trunk_z)), 6),
            "commandedAxisDistanceM": commanded_axis,
            "finalTiltDeg": round(plant.tilt_deg(), 4),
            # The upstream velstand gate: trunk below 0.10 m OR tilt beyond 40 deg. A
            # deliberate sit trips it by design, which is why it is reported rather than
            # interpreted here.
            "upstreamFallenGate": bool(plant.tilt_deg() > 40.0 or end[2] < 0.10),
            # The upstream velstand "recovery complete" definition.
            "uprightRecovered": bool(plant.tilt_deg() < 25.0 and end[2] > 0.09),
        },
        "contacts": summarise_contacts(sink),
    }
    if ball_start is not None:
        ball_end = plant.ball_position()
        bd = ball_end - ball_start
        result["ball"] = {
            "startM": [round(float(v), 6) for v in ball_start],
            "endM": [round(float(v), 6) for v in ball_end],
            "displacementM": [round(float(v), 6) for v in bd],
            "planarDistanceM": round(float(math.hypot(bd[0], bd[1])), 6),
            "footContactCount": len(sink["foot_ball_contacts"]),
        }
    return result


ZERO_CMD = {"twist": (0.0, 0.0, 0.0), "head": (0.0, 0.0, 0.0, 0.0), "body": {}}


def cmd(vx=0.0, vy=0.0, vyaw=0.0):
    return {"twist": (vx, vy, vyaw), "head": (0.0, 0.0, 0.0, 0.0), "body": {}}


def ground_pick_command(period=4.0, end_phase=0.7):
    def fn(t):
        phase = min(t / period, end_phase)
        angle = 2 * math.pi * phase
        return {"twist": (math.cos(angle), math.sin(angle), 0.0), "head": (0.0,) * 4, "body": {}}
    return fn


FACE_DOWN = (math.sqrt(0.5), 0.0, -math.sqrt(0.5), 0.0)
FACE_UP = (math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0)
ON_SIDE = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)

TRIALS = {
    "stand": dict(model_key="groundcontact", policy_id="stand", seconds=6.0, command=ZERO_CMD, standing_tuned=True),
    "walk": dict(model_key="walk", policy_id="walking", seconds=6.0, command=cmd(vx=0.30)),
    "walk-fast": dict(model_key="walk", policy_id="walking", seconds=6.0, command=cmd(vx=0.40)),
    "walk-turn": dict(model_key="walk", policy_id="walking", seconds=6.0, command=cmd(vx=0.30, vyaw=0.8)),
    "walk-no-actuation": dict(model_key="walk", policy_id="walking", seconds=6.0, command=cmd(vx=0.30), actuation=False),
    "walk-low-traction": dict(model_key="lowtraction", policy_id="walking", seconds=6.0, command=cmd(vx=0.30)),
    "sit": dict(model_key="groundcontact", policy_id="sitstand", seconds=4.0, command=cmd(vx=1.0)),
    "ground-pick": dict(model_key="groundcontact", policy_id="ground_pick", seconds=3.0, command=ZERO_CMD, standing_tuned=True, command_fn=ground_pick_command()),
    "roulade": dict(model_key="groundcontact", policy_id="roulade", seconds=3.0, command=ZERO_CMD, standing_tuned=True),
    "kick-right": dict(model_key="kick", policy_id="kick_right", seconds=3.0, command=ZERO_CMD, standing_tuned=True, ball_xy=(0.09, -0.042)),
    "kick-left": dict(model_key="kick", policy_id="kick_left", seconds=3.0, command=ZERO_CMD, standing_tuned=True, ball_xy=(0.09, 0.042)),
    "kick-miss": dict(model_key="kick", policy_id="kick_right", seconds=3.0, command=ZERO_CMD, standing_tuned=True, ball_xy=(0.55, -0.042)),
    "recover-face-down": dict(model_key="groundcontact", policy_id="stand", seconds=8.0, command=ZERO_CMD, standing_tuned=True, perturbation=FACE_DOWN, settle_seconds=1.5),
    "recover-face-up": dict(model_key="groundcontact", policy_id="stand", seconds=8.0, command=ZERO_CMD, standing_tuned=True, perturbation=FACE_UP, settle_seconds=1.5),
    "recover-on-side": dict(model_key="groundcontact", policy_id="stand", seconds=8.0, command=ZERO_CMD, standing_tuned=True, perturbation=ON_SIDE, settle_seconds=1.5),
    # Negative controls for the recovery capability. A physical recovery has to be able to
    # fail, and a failure has to be reported as one.
    "recover-no-actuation": dict(model_key="groundcontact", policy_id="stand", seconds=8.0, command=ZERO_CMD, standing_tuned=True, perturbation=FACE_DOWN, settle_seconds=1.5, actuation=False),
    "recover-short-budget": dict(model_key="groundcontact", policy_id="stand", seconds=1.5, command=ZERO_CMD, standing_tuned=True, perturbation=FACE_DOWN, settle_seconds=1.5),
}


def make_fixture(sessions, hashes):
    """An independently derived controller-conformance fixture.

    At each sampled state it records the complete controller pipeline: the physical inputs,
    the assembled 61-value observation, the raw 14-value policy output, the post-processed
    targets, and the controller state carried into the next tick. A browser port that shifts
    a block, drops the mouth exclusion, feeds back a scaled action, or applies the filters in
    the wrong order fails against it immediately.
    """
    plant = Plant("walk")
    controller = ReferenceController(sessions)
    samples = []
    command = cmd(vx=0.30)
    for tick in range(24):
        obs_in = plant.observe()
        previous_raw = controller.last_action.copy()
        previous_targets = None if controller.previous_targets is None else controller.previous_targets.copy()
        observation, action, targets, scale = controller.step(
            "walking", obs_in["gyro"], obs_in["gravity"], obs_in["joint_pos"], obs_in["joint_vel"], command)
        if tick in (0, 1, 2, 7, 15, 23):
            samples.append({
                "tick": tick,
                "simulationTimeSeconds": round(float(plant.data.time), 6),
                "input": {
                    "gyroRadS": [float(v) for v in obs_in["gyro"]],
                    "projectedGravity": [float(v) for v in obs_in["gravity"]],
                    "jointPositionRad": [float(v) for v in obs_in["joint_pos"]],
                    "jointVelocityRadS": [float(v) for v in obs_in["joint_vel"]],
                    "previousRawAction": [float(v) for v in previous_raw],
                    "previousTargetsRad": None if previous_targets is None else [float(v) for v in previous_targets],
                    "command": {"twist": list(command["twist"]), "head": list(command["head"]), "body": {"z": 0.0, "roll": 0.0, "pitch": 0.0}},
                },
                "observation": [float(v) for v in observation],
                "rawAction": [float(v) for v in action],
                "actionScale": scale,
                "targetsRad": [float(v) for v in targets],
                "wireTargets": {
                    "order": list(WIRE_JOINT_ORDER),
                    "mouthWireIndex": MOUTH_WIRE_INDEX,
                    "mouthIsPolicyControlled": False,
                },
            })
        plant.data.ctrl[:] = targets
        plant.advance(CONTROL_DECIMATION)
    return {
        "schema": "robobuddy.microduck-controller-conformance.v1",
        "generatedBy": "python native/microduck_reference.py --fixture",
        "independentOf": "the browser implementation; derived from the pinned deployed Rust controller sources",
        "pins": {
            "microduck": "590b986bd8c0d50ae02cb3ea2f59c463b6828168",
            "microduck_rl": "519142b1f5bf59fdfd44d06c205119e7fff8e3cb",
            "mujoco": mujoco.__version__,
        },
        "model": {"asset": str(plant.path.relative_to(ROOT)), "sha256": plant.sha256},
        "policy": {"id": "walking", "file": POLICY_FILES["walking"], "sha256": hashes["walking"]},
        "contract": {
            "observationWidth": OBS_LEN,
            "actionWidth": ACTION_LEN,
            "layout": {"gyro": [0, 3], "projectedGravity": [3, 6], "jointPositionOffset": [6, 20],
                       "jointVelocity": [20, 34], "previousRawAction": [34, 48], "command": [48, 61]},
            "policyJointOrder": list(POLICY_JOINT_ORDER),
            "homePositionRad": [float(v) for v in HOME_RAD],
            "actionScale": ACTION_SCALE,
            "headLowpassAlpha": HEAD_LOWPASS,
            "legsLowpassAlpha": LEGS_LOWPASS,
            "headJointSlots": list(HEAD_SLOTS),
            "physicsTimestepSeconds": PHYSICS_TIMESTEP,
            "controlDecimation": CONTROL_DECIMATION,
        },
        "samples": samples,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial", default="walk", help=f"one of {sorted(TRIALS)} or 'all'")
    parser.add_argument("--timestep", type=float, default=None, help="override the physics timestep (numerical-sensitivity runs)")
    parser.add_argument("--json", default=None, help="write the report to this path")
    parser.add_argument("--fixture", default=None, help="write the controller-conformance fixture to this path")
    args = parser.parse_args()

    if mujoco.__version__ != EXPECTED_MUJOCO_VERSION:
        print(f"warning: MuJoCo {mujoco.__version__}, expected {EXPECTED_MUJOCO_VERSION}", file=sys.stderr)

    if args.fixture:
        sessions, hashes = load_sessions(["walking"])
        fixture = make_fixture(sessions, hashes)
        path = Path(args.fixture)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(fixture, indent=1) + "\n")
        print(f"wrote {path} with {len(fixture['samples'])} sampled controller states")
        return 0

    names = sorted(TRIALS) if args.trial == "all" else [args.trial]
    for name in names:
        if name not in TRIALS:
            print(f"unknown trial {name}; choose from {sorted(TRIALS)} or 'all'", file=sys.stderr)
            return 2
    needed = sorted({TRIALS[n]["policy_id"] for n in names})
    sessions, hashes = load_sessions(needed)

    reports = {}
    for name in names:
        spec = dict(TRIALS[name])
        if args.timestep:
            spec["timestep"] = args.timestep
            spec["decimation"] = max(1, int(round((PHYSICS_TIMESTEP * CONTROL_DECIMATION) / args.timestep)))
        report = run_trial(sessions=sessions, hashes=hashes, **spec)
        reports[name] = report
        m = report["measured"]
        c = report["contacts"]
        extra = ""
        if "ball" in report:
            extra = f"  ball={report['ball']['planarDistanceM']:.4f} m via {report['ball']['footContactCount']} foot contacts"
        axis = "  n/a" if m["commandedAxisDistanceM"] is None else f"{m['commandedAxisDistanceM']:+.4f}"
        print(f"{name:20s} dist={m['planarDistanceM']:.4f} m  cmd-axis={axis}  v={m['averageSpeedMS']:.4f} m/s  "
              f"z={m['finalTrunkHeightM']:.4f}  tilt={m['finalTiltDeg']:.2f} deg  upright={m['uprightRecovered']}  "
              f"air L/R={c['leftAirFraction']:.3f}/{c['rightAirFraction']:.3f}  "
              f"steps L/R={c['leftContactTransitions']}/{c['rightContactTransitions']}{extra}")

    if args.json:
        path = Path(args.json)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(reports, indent=1) + "\n")
        print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
