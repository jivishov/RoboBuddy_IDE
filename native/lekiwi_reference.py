#!/usr/bin/env python3
"""LeKiwi Phase 5B native MuJoCo reference runner.

It executes the same pinned model assets and the same bounded controller as the browser
workspace, and emits a machine-readable run report.

Scope of the evidence it produces:
  * software verification - the implementation loads, maps, actuates and observes correctly;
  * numerical/model checks - the chosen formulation behaves coherently and survives a tighter
    timestep;
  * native/browser conformance - the same model and controller agree across two backends.

It is NOT hardware validation. Native and browser agreement is implementation evidence only,
because both run the same model and the same approximations.

Usage:
    python native/lekiwi_reference.py --trial courier
    python native/lekiwi_reference.py --trial base-motion --timestep 0.001
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

REPO = Path(__file__).resolve().parent.parent
MODELS = REPO / "models" / "lekiwi"

# --- shared controller contract -------------------------------------------------------------
# This block is the single numeric contract shared with src/physics/lekiwi-scene.js and
# src/physics/lekiwi-kinematics.js. tests/physics/lekiwi-phase5b-core.mjs parses it out of this
# file and deep-compares it with the browser modules, so the two backends cannot drift apart.
SHARED_CONTROLLER_JSON = r"""
{
  "wheelOrder": ["base_left_wheel", "base_back_wheel", "base_right_wheel"],
  "wheelRadiusM": 0.0508,
  "driveDirections": [[-0.866025, 0.5], [0.0, -1.0], [0.866025, 0.5]],
  "momentArmsM": [0.109842, 0.119842, 0.110242],
  "maxWheelRadS": 4.60061,
  "lerobotDefaults": {"wheelRadiusM": 0.05, "baseRadiusM": 0.125, "angleDeg": [150.0, -90.0, 30.0]},
  "driveLimits": {
    "maxLinearMS": 0.22,
    "maxYawRadS": 0.9,
    "linearGain": 1.6,
    "yawGain": 2.2,
    "positionToleranceM": 0.015,
    "yawToleranceRad": 0.06,
    "arrivalSpeedMS": 0.03,
    "arrivalYawRateRadS": 0.15
  },
  "gripperOpenRad": 1.2,
  "gripperCloseRad": -0.17,
  "controllerPeriodSeconds": 0.02,
  "observationPeriodSeconds": 0.01,
  "stowPoseRad": {
    "arm_shoulder_pan": 0.0,
    "arm_shoulder_lift": -1.53,
    "arm_elbow_flex": 0.19,
    "arm_wrist_flex": 1.04,
    "arm_wrist_roll": 0.0,
    "arm_gripper": 1.2
  },
  "armPosesRad": {
    "pick_approach": [-0.1281, -0.7472, -0.0251, 1.6452, 0.025],
    "pick_grip": [-0.124, -0.4377, -0.2109, 1.5215, 0.0279],
    "pick_lift": [-0.124, -0.3292, -0.4542, 1.6563, 0.0279],
    "carry": [-0.0118, 0.0084, -0.8149, 1.65806, 0.1109],
    "drop_over": [0.0938, 0.0549, -0.9539, 1.65806, 0.0602],
    "drop_place": [0.094, -0.2226, -0.256, 1.3514, 0.061],
    "drop_retreat": [0.0969, -0.0811, -0.6608, 1.6146, 0.0632]
  },
  "workcell": {
    "worktopTopZM": 0.211,
    "pickupXYM": [0.305, -0.292],
    "deliveryXYM": [0.242928, -0.370144],
    "homeXYM": [0.0, 0.0],
    "serviceStopXYM": [0.274, 0.03],
    "serviceStopYawRad": -1.5707963267948966,
    "restrictedStopXYM": [0.1, -0.65],
    "restrictedStopRadiusM": 0.15
  },
  "taskGoal": {
    "deliveryHalfExtentsXYM": [0.02, 0.02],
    "restXYToleranceM": 0.02,
    "supportZToleranceM": 0.004,
    "liftClearanceM": 0.01,
    "carryHorizontalM": 0.06,
    "settleSeconds": 0.25,
    "maxSettleDriftM": 0.0015,
    "maxSettleLinearSpeedMS": 0.03,
    "maxSettleAngularSpeedRadS": 0.8,
    "maxSettleTiltRad": 0.2,
    "baseTravelM": 0.2,
    "homeToleranceM": 0.03,
    "serviceStopToleranceM": 0.03,
    "stoppedSpeedMS": 0.02
  },
  "stageDurationsSeconds": {
    "settle": 0.6,
    "stow_arm": 0.6,
    "stop_at_service": 1.0,
    "approach_beaker": 2.0,
    "reach_rim": 1.4,
    "close_gripper": 1.0,
    "lift_beaker": 1.4,
    "carry_beaker": 1.6,
    "over_delivery": 1.6,
    "lower_beaker": 1.6,
    "release_beaker": 1.2,
    "retreat_arm": 1.6,
    "restow_arm": 2.2,
    "stop_home": 1.2
  },
  "driveTimeoutSeconds": {"drive_to_service": 14.0, "drive_home": 16.0}
}
"""
SHARED = json.loads(SHARED_CONTROLLER_JSON)

ARM_JOINTS = ["arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex", "arm_wrist_flex", "arm_wrist_roll"]
WHEELS = SHARED["wheelOrder"]
BEAKER_PREFIX = "beaker_"
FIXED_JAW = ("fixed_jaw_pad1", "fixed_jaw_pad2", "fixed_jaw_tip")
MOVING_JAW = ("moving_jaw_pad1", "moving_jaw_pad2", "moving_jaw_tip")
SUPPORT_GEOM = "lekiwi_transfer_worktop"

MODEL_FILES = {
    "wheel_reference": "wheel_reference.xml",
    "base": "base.xml",
    "base_stand": "base_stand.xml",
    "base_lowtraction": "base_lowtraction.xml",
    "courier": "courier.xml",
}
MODEL_PACKAGE_IDS = {
    "wheel_reference": ("lekiwi-wheel-reference-efa608d-v1", "robobuddy-lekiwi-wheel-reference-v1"),
    "base": ("lekiwi-base-efa608d-v1", "robobuddy-lekiwi-base-v1"),
    "base_stand": ("lekiwi-base-stand-efa608d-v1", "robobuddy-lekiwi-base-stand-v1"),
    "base_lowtraction": ("lekiwi-base-lowtraction-efa608d-v1", "robobuddy-lekiwi-base-lowtraction-v1"),
    "courier": ("lekiwi-courier-efa608d-v1", "robobuddy-lekiwi-courier-v1"),
}


def sha256_of(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def controller_sha256() -> str:
    return hashlib.sha256(SHARED_CONTROLLER_JSON.encode()).hexdigest()


def load_model(name: str, timestep: float | None = None, iterations: int | None = None):
    path = MODELS / MODEL_FILES[name]
    model = mujoco.MjModel.from_xml_path(str(path))
    if timestep is not None:
        model.opt.timestep = timestep
    if iterations is not None:
        model.opt.iterations = iterations
    return model, mujoco.MjData(model), path


def jid(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)


def qadr(model, name):
    return model.jnt_qposadr[jid(model, name)]


def dofadr(model, name):
    return model.jnt_dofadr[jid(model, name)]


def aid(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)


def bid(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)


def yaw_of(quat):
    w, x, y, z = quat
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def body_to_wheel(vx: float, vy: float, omega: float, wheel_radius=None, moment_arms=None, drive=None, max_wheel=None):
    wheel_radius = SHARED["wheelRadiusM"] if wheel_radius is None else wheel_radius
    moment_arms = SHARED["momentArmsM"] if moment_arms is None else moment_arms
    drive = SHARED["driveDirections"] if drive is None else drive
    max_wheel = SHARED["maxWheelRadS"] if max_wheel is None else max_wheel
    raw = [(d[0] * vx + d[1] * vy + arm * omega) / wheel_radius for d, arm in zip(drive, moment_arms)]
    peak = max(abs(v) for v in raw)
    scale = max_wheel / peak if peak > max_wheel else 1.0
    return [v * scale for v in raw], scale < 1.0


def base_pose(model, data, body="lekiwi_base"):
    b = bid(model, body)
    pos = data.xpos[b].copy()
    quat = data.xquat[b].copy()
    adr = dofadr(model, "lekiwi_base_free") if jid(model, "lekiwi_base_free") >= 0 else None
    lin = data.qvel[adr:adr + 3].copy() if adr is not None else np.zeros(3)
    ang = data.qvel[adr + 3:adr + 6].copy() if adr is not None else np.zeros(3)
    return dict(xM=float(pos[0]), yM=float(pos[1]), zM=float(pos[2]), yawRad=float(yaw_of(quat)),
                speedMS=float(math.hypot(lin[0], lin[1])), yawRateRadS=float(ang[2]),
                positionM=[float(v) for v in pos], quaternionWxyz=[float(v) for v in quat])


def geom_name(model, index):
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index) or ""


def contact_pairs(model, data):
    pairs = []
    for i in range(data.ncon):
        c = data.contact[i]
        pairs.append((geom_name(model, c.geom[0]), geom_name(model, c.geom[1])))
    return pairs


def beaker_contacts(model, data):
    fixed = moving = support = False
    for a, b in contact_pairs(model, data):
        names = (a, b)
        if not any(n.startswith(BEAKER_PREFIX) for n in names):
            continue
        other = b if a.startswith(BEAKER_PREFIX) else a
        if other in FIXED_JAW:
            fixed = True
        if other in MOVING_JAW:
            moving = True
        if other == SUPPORT_GEOM:
            support = True
    return fixed, moving, support


def tilt_of(quat):
    w, x, y, z = quat
    return math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y))))


# --- courier evaluator (mirrors src/physics/lekiwi-task-evaluator.js) -------------------------
class CourierEvaluator:
    def __init__(self, goal, workcell):
        self.goal = goal
        self.workcell = workcell
        self.s = dict(initial_beaker=None, last_beaker=None, last_base=None, path=0.0,
                      grasp=False, grasp_t=None, lift=False, lift_t=None, carry=False,
                      carry_anchor=None, max_held=0.0, support_held=False, support_held_t=None,
                      release=False, release_t=None, release_invalidations=0,
                      settle_anchor=None, settle_anchor_t=None, settle_drift=None,
                      settled=False, settle_t=None, service=False, service_t=None,
                      home=False, home_t=None, restricted=False, lost=False,
                      jaw=False, support=False, max_z=-1e9, samples=0)

    def observe(self, model, data):
        s = self.s
        goal = self.goal
        wc = self.workcell
        t = float(data.time)
        s["samples"] += 1
        pose = base_pose(model, data)
        base_xy = (pose["xM"], pose["yM"])
        if s["last_base"] is not None:
            s["path"] += math.dist(base_xy, s["last_base"])
        s["last_base"] = base_xy
        if math.dist(base_xy, tuple(wc["restrictedStopXYM"])) <= wc["restrictedStopRadiusM"]:
            s["restricted"] = True
        stopped = pose["speedMS"] <= goal["stoppedSpeedMS"] and abs(pose["yawRateRadS"]) <= 0.15
        if not s["service"] and stopped and math.dist(base_xy, tuple(wc["serviceStopXYM"])) <= goal["serviceStopToleranceM"]:
            s["service"] = True
            s["service_t"] = t

        bb = bid(model, "empty_beaker")
        pos = data.xpos[bb].copy()
        quat = data.xquat[bb].copy()
        badr = dofadr(model, "empty_beaker_free")
        lin = data.qvel[badr:badr + 3]
        ang = data.qvel[badr + 3:badr + 6]
        if s["initial_beaker"] is None:
            s["initial_beaker"] = pos.copy()
        s["last_beaker"] = pos.copy()
        s["max_z"] = max(s["max_z"], float(pos[2]))
        if pos[2] < s["initial_beaker"][2] - 0.05:
            s["lost"] = True

        previous_jaw = s["jaw"]
        fixed, moving, support = beaker_contacts(model, data)
        grasp = fixed and moving
        s["jaw"] = fixed or moving
        s["support"] = support
        if grasp and not s["grasp"]:
            s["grasp"] = True
            s["grasp_t"] = t
        if s["grasp"] and grasp and not support and pos[2] > s["initial_beaker"][2] + goal["liftClearanceM"]:
            if not s["lift"]:
                s["lift"] = True
                s["lift_t"] = t
            if not s["carry"] and s["carry_anchor"] is None:
                s["carry_anchor"] = pos.copy()
        if not s["carry"] and s["carry_anchor"] is not None:
            if not s["jaw"]:
                s["carry_anchor"] = None
                s["max_held"] = 0.0
            elif grasp:
                travel = math.hypot(pos[0] - s["carry_anchor"][0], pos[1] - s["carry_anchor"][1])
                s["max_held"] = max(s["max_held"], travel)
                if travel >= goal["carryHorizontalM"]:
                    s["carry"] = True

        dx = abs(pos[0] - wc["deliveryXYM"][0])
        dy = abs(pos[1] - wc["deliveryXYM"][1])
        in_delivery = dx <= goal["deliveryHalfExtentsXYM"][0] and dy <= goal["deliveryHalfExtentsXYM"][1]
        at_support = abs(pos[2] - s["initial_beaker"][2]) <= goal["supportZToleranceM"]

        if not s["support_held"] and s["carry"] and grasp and support and in_delivery:
            s["support_held"] = True
            s["support_held_t"] = t
        if s["release"] and s["jaw"]:
            s["release"] = False
            s["release_invalidations"] += 1
            s["release_t"] = None
            s["settle_anchor"] = None
            s["settle_anchor_t"] = None
            s["settled"] = False
            s["settle_t"] = None
            s["home"] = False
            s["home_t"] = None
        if not s["release"] and s["support_held"] and previous_jaw and not s["jaw"] and support and in_delivery:
            s["release"] = True
            s["release_t"] = t

        eligible = (s["release"] and support and not s["jaw"] and in_delivery and at_support
                    and float(np.linalg.norm(lin)) <= goal["maxSettleLinearSpeedMS"]
                    and float(np.linalg.norm(ang)) <= goal["maxSettleAngularSpeedRadS"]
                    and tilt_of(quat) <= goal["maxSettleTiltRad"])
        if not eligible:
            s["settle_anchor"] = None
            s["settle_anchor_t"] = None
            s["settle_drift"] = None
            s["settled"] = False
            s["settle_t"] = None
        elif s["settle_anchor"] is None:
            s["settle_anchor"] = pos.copy()
            s["settle_anchor_t"] = t
            s["settle_drift"] = 0.0
        else:
            drift = float(np.linalg.norm(pos - s["settle_anchor"]))
            if drift > goal["maxSettleDriftM"]:
                s["settle_anchor"] = pos.copy()
                s["settle_anchor_t"] = t
                s["settle_drift"] = 0.0
                s["settled"] = False
            else:
                s["settle_drift"] = max(s["settle_drift"] or 0.0, drift)
                if t - s["settle_anchor_t"] + 1e-12 >= goal["settleSeconds"]:
                    if not s["settled"]:
                        s["settle_t"] = t
                    s["settled"] = True

        if s["settled"] and stopped and math.dist(base_xy, tuple(wc["homeXYM"])) <= goal["homeToleranceM"] and not s["home"]:
            s["home"] = True
            s["home_t"] = t

    def metrics(self):
        s = self.s
        goal = self.goal
        wc = self.workcell
        rest_err = None
        if s["last_beaker"] is not None:
            rest_err = float(math.hypot(s["last_beaker"][0] - wc["deliveryXYM"][0], s["last_beaker"][1] - wc["deliveryXYM"][1]))
        times = [s["service_t"], s["grasp_t"], s["lift_t"], s["support_held_t"], s["release_t"], s["settle_t"], s["home_t"]]
        causal = all(v is not None for v in times) and (
            s["service_t"] <= s["grasp_t"] <= s["lift_t"] <= s["support_held_t"] < s["release_t"] < s["settle_t"] <= s["home_t"])
        travel_ok = s["path"] >= goal["baseTravelM"]
        success = bool(not s["restricted"] and not s["lost"] and s["service"] and s["grasp"] and s["lift"]
                       and s["carry"] and s["support_held"] and s["release"] and s["settled"] and s["home"]
                       and travel_ok and causal and s["support"] and not s["jaw"]
                       and rest_err is not None and rest_err <= goal["restXYToleranceM"])
        return dict(
            success=success, causalOrder=bool(causal), serviceStopReached=s["service"], graspSeen=s["grasp"],
            liftSeen=s["lift"], carrySeen=s["carry"], supportWhileHeldSeen=s["support_held"],
            releaseSeen=s["release"], releaseInvalidations=s["release_invalidations"], settled=s["settled"],
            homeReturned=s["home"], restrictedViolation=s["restricted"], payloadLost=s["lost"],
            basePathLengthM=float(s["path"]), baseTravelSufficient=bool(travel_ok),
            maxHeldTravelM=float(s["max_held"]),
            liftHeightM=float(s["max_z"] - s["initial_beaker"][2]) if s["initial_beaker"] is not None else None,
            settleDriftM=s["settle_drift"], restXYErrorM=rest_err,
            initialBeakerM=[float(v) for v in s["initial_beaker"]] if s["initial_beaker"] is not None else None,
            finalBeakerM=[float(v) for v in s["last_beaker"]] if s["last_beaker"] is not None else None,
            finalBaseM=[float(v) for v in s["last_base"]] if s["last_base"] is not None else None,
            serviceStopTimeSeconds=s["service_t"], graspTimeSeconds=s["grasp_t"], liftTimeSeconds=s["lift_t"],
            supportWhileHeldTimeSeconds=s["support_held_t"], releaseTimeSeconds=s["release_t"],
            settleTimeSeconds=s["settle_t"], homeTimeSeconds=s["home_t"], observationCount=s["samples"],
        )


class Runner:
    """Fixed-simulation-time execution with a declared observation cadence."""

    def __init__(self, model, data, evaluator=None):
        self.model = model
        self.data = data
        self.evaluator = evaluator
        self.sample_every = max(1, int(round(SHARED["observationPeriodSeconds"] / model.opt.timestep)))
        self.steps = 0
        self.commands = []
        self.trajectory = []
        self.budget = None

    def set_budget(self, steps):
        self.budget = steps

    def wheel_command(self, vx, vy, omega):
        targets, saturated = body_to_wheel(vx, vy, omega)
        for name, value in zip(WHEELS, targets):
            self.data.ctrl[aid(self.model, name)] = float(value)
        return targets, saturated

    def arm_command(self, pose, gripper=None):
        for name, value in pose.items():
            index = aid(self.model, name)
            if index >= 0:
                self.data.ctrl[index] = float(value)
        if gripper is not None:
            self.data.ctrl[aid(self.model, "arm_gripper")] = float(gripper)

    def advance(self, seconds, tag=None):
        steps = int(round(seconds / self.model.opt.timestep))
        for _ in range(steps):
            if self.budget is not None and self.steps >= self.budget:
                return False
            mujoco.mj_step(self.model, self.data)
            self.steps += 1
            if self.steps % self.sample_every == 0:
                self.sample(tag)
        return True

    def sample(self, tag=None):
        if self.evaluator is not None:
            self.evaluator.observe(self.model, self.data)
        if len(self.trajectory) < 4000 and self.steps % (self.sample_every * 10) == 0:
            pose = base_pose(self.model, self.data)
            entry = dict(t=round(float(self.data.time), 4), stage=tag,
                         baseXYM=[round(pose["xM"], 5), round(pose["yM"], 5)],
                         baseYawRad=round(pose["yawRad"], 5),
                         wheelRadS=[round(float(self.data.qvel[dofadr(self.model, w)]), 4) for w in WHEELS])
            if bid(self.model, "empty_beaker") >= 0:
                entry["beakerM"] = [round(float(v), 5) for v in self.data.xpos[bid(self.model, "empty_beaker")]]
            self.trajectory.append(entry)


def drive_to(runner, waypoints, timeout_seconds, arm_pose=None, gripper=None, tag=None):
    limits = SHARED["driveLimits"]
    dt = runner.model.opt.timestep
    period_steps = max(1, int(round(SHARED["controllerPeriodSeconds"] / dt)))
    reached = []
    for waypoint in waypoints:
        elapsed = 0.0
        arrived = False
        while elapsed < timeout_seconds:
            pose = base_pose(runner.model, runner.data)
            dx = waypoint["xM"] - pose["xM"]
            dy = waypoint["yM"] - pose["yM"]
            cos, sin = math.cos(pose["yawRad"]), math.sin(pose["yawRad"])
            vx = limits["linearGain"] * (dx * cos + dy * sin)
            vy = limits["linearGain"] * (-dx * sin + dy * cos)
            speed = math.hypot(vx, vy)
            if speed > limits["maxLinearMS"]:
                vx *= limits["maxLinearMS"] / speed
                vy *= limits["maxLinearMS"] / speed
            yaw_error = wrap(waypoint["yawRad"] - pose["yawRad"]) if waypoint.get("yawRad") is not None else 0.0
            omega = max(-limits["maxYawRadS"], min(limits["maxYawRadS"], limits["yawGain"] * yaw_error))
            position_error = math.hypot(dx, dy)
            if (position_error <= limits["positionToleranceM"] and abs(yaw_error) <= limits["yawToleranceRad"]
                    and pose["speedMS"] <= limits["arrivalSpeedMS"] and abs(pose["yawRateRadS"]) <= limits["arrivalYawRateRadS"]):
                runner.wheel_command(0.0, 0.0, 0.0)
                arrived = True
                break
            targets, saturated = runner.wheel_command(vx, vy, omega)
            runner.commands.append(dict(t=round(float(runner.data.time), 4), waypoint=waypoint["id"],
                                        desiredBodyVel=[round(vx, 5), round(vy, 5), round(omega, 5)],
                                        wheelTargetsRadS=[round(v, 5) for v in targets], saturated=saturated))
            if arm_pose is not None:
                runner.arm_command(arm_pose, gripper)
            if not runner.advance(period_steps * dt, tag):
                return reached, False
            elapsed += period_steps * dt
        runner.wheel_command(0.0, 0.0, 0.0)
        reached.append(dict(waypoint=waypoint["id"], arrived=arrived,
                            baseXYM=[round(base_pose(runner.model, runner.data)["xM"], 5),
                                     round(base_pose(runner.model, runner.data)["yM"], 5)]))
        if not arrived:
            return reached, False
    return reached, True


def arm_pose_dict(values):
    return {name: value for name, value in zip(ARM_JOINTS, values)}


# --- trials ----------------------------------------------------------------------------------
def trial_wheel_reference(args):
    results = {}
    interventions = []

    def rig(friction=None, disable_motor=False):
        model, data, path = load_model("wheel_reference", args.timestep, args.iterations)
        if friction is not None:
            for g in range(model.ngeom):
                name = geom_name(model, g)
                if "roller" in name or name == "reference_floor":
                    model.geom_friction[g, 0] = friction
        if disable_motor:
            index = aid(model, "base_back_wheel")
            model.actuator_forcerange[index] = [0.0, 0.0]
            model.actuator_gainprm[index, 0] = 0.0
        return model, data, path

    def carriage(model, data):
        return data.xpos[bid(model, "wheel_carriage")].copy()

    # grounded drive along the wheel drive direction
    model, data, path = rig()
    for _ in range(int(0.8 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    start = carriage(model, data)
    data.ctrl[aid(model, "base_back_wheel")] = 3.0
    for _ in range(int(3.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    wheel_rad_s = float(data.qvel[dofadr(model, "base_back_wheel")])
    delta = carriage(model, data) - start
    results["grounded"] = dict(driveDirectionM=float(-delta[1]), crossTrackM=float(delta[0]),
                               wheelRadS=wheel_rad_s, rollingSpeedMS=wheel_rad_s * SHARED["wheelRadiusM"],
                               carriageSpeedMS=float(math.hypot(data.qvel[0], data.qvel[1])), contacts=int(data.ncon))

    # suspended: an explicit external hold keeps the rig airborne; no ground contact exists
    model, data, path = rig()
    total_mass = float(model.body_subtreemass[0])
    data.qpos[qadr(model, "carriage_z")] = 0.08
    data.ctrl[aid(model, "base_back_wheel")] = 3.0
    mujoco.mj_forward(model, data)
    start = None
    for i in range(int(2.6 / model.opt.timestep)):
        data.xfrc_applied[bid(model, "wheel_carriage")][2] = total_mass * 9.81
        mujoco.mj_step(model, data)
        if i == int(0.4 / model.opt.timestep):
            start = carriage(model, data)
    interventions.append("suspended trial applies an explicit external support force to the reference carriage so it stays airborne; no contact exists and no state is assigned")
    delta = carriage(model, data) - start
    results["suspended"] = dict(horizontalTravelM=float(math.hypot(delta[0], delta[1])),
                                wheelRadS=float(data.qvel[dofadr(model, "base_back_wheel")]), contacts=int(data.ncon))

    # passive roller direction vs driven direction, at equal applied force
    for label, force in (("passiveDirection", [3.0, 0.0, 0.0]), ("drivenDirection", [0.0, -3.0, 0.0])):
        model, data, path = rig()
        for _ in range(int(0.8 / model.opt.timestep)):
            mujoco.mj_step(model, data)
        start = carriage(model, data)
        for _ in range(int(2.0 / model.opt.timestep)):
            data.xfrc_applied[bid(model, "wheel_carriage")][:3] = force
            mujoco.mj_step(model, data)
        delta = carriage(model, data) - start
        results[label] = dict(travelM=float(math.hypot(delta[0], delta[1])), appliedForceN=force)
    results["anisotropyRatio"] = (results["passiveDirection"]["travelM"] / results["drivenDirection"]["travelM"]
                                  if results["drivenDirection"]["travelM"] > 1e-9 else None)

    # reduced traction
    model, data, path = rig(friction=0.02)
    interventions.append("traction trial sets roller and floor sliding friction to 0.02 as a declared adverse surface")
    for _ in range(int(0.8 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    start = carriage(model, data)
    data.ctrl[aid(model, "base_back_wheel")] = 3.0
    for _ in range(int(1.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    results["reducedTraction"] = dict(travelM=float(math.hypot(*(carriage(model, data) - start)[:2])))

    model, data, path = rig()
    for _ in range(int(0.8 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    start = carriage(model, data)
    data.ctrl[aid(model, "base_back_wheel")] = 3.0
    for _ in range(int(1.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    results["nominalTractionSameWindow"] = dict(travelM=float(math.hypot(*(carriage(model, data) - start)[:2])))

    # zero-drive: servo braking, then a free-wheel coast with the motor disabled
    for label, disable in (("servoBrakedStop", False), ("freeWheelCoast", True)):
        model, data, path = rig()
        for _ in range(int(0.8 / model.opt.timestep)):
            mujoco.mj_step(model, data)
        data.ctrl[aid(model, "base_back_wheel")] = 3.0
        for _ in range(int(5.0 / model.opt.timestep)):
            mujoco.mj_step(model, data)
        speed_before = float(math.hypot(data.qvel[0], data.qvel[1]))
        mark = carriage(model, data)
        if disable:
            index = aid(model, "base_back_wheel")
            model.actuator_forcerange[index] = [0.0, 0.0]
            model.actuator_gainprm[index, 0] = 0.0
            interventions.append("free-wheel coast trial zeroes the wheel actuator gain and force range as a declared motor-disabled condition")
        data.ctrl[aid(model, "base_back_wheel")] = 0.0
        for _ in range(int(5.0 / model.opt.timestep)):
            mujoco.mj_step(model, data)
        results[label] = dict(speedBeforeMS=speed_before, speedAfterMS=float(math.hypot(data.qvel[0], data.qvel[1])),
                              coastTravelM=float(math.hypot(*(carriage(model, data) - mark)[:2])))
    return "wheel_reference", results, interventions, [], []


def run_base_command(model_name, command, seconds, args, settle=1.0, disable_wheels=(), friction=None):
    model, data, path = load_model(model_name, args.timestep, args.iterations)
    interventions = []
    if friction is not None:
        for g in range(model.ngeom):
            name = geom_name(model, g)
            if "roller" in name or "floor" in name:
                model.geom_friction[g, 0] = friction
        interventions.append(f"floor and roller sliding friction set to {friction}")
    for name in disable_wheels:
        index = aid(model, name)
        model.actuator_forcerange[index] = [0.0, 0.0]
        model.actuator_gainprm[index, 0] = 0.0
        interventions.append(f"wheel actuator {name} gain and force range zeroed (declared motor-disabled condition)")
    for _ in range(int(settle / model.opt.timestep)):
        mujoco.mj_step(model, data)
    start = base_pose(model, data)
    targets, saturated = body_to_wheel(*command)
    for name, value in zip(WHEELS, targets):
        data.ctrl[aid(model, name)] = float(value)
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)
    end = base_pose(model, data)
    dx = end["xM"] - start["xM"]
    dy = end["yM"] - start["yM"]
    cos, sin = math.cos(start["yawRad"]), math.sin(start["yawRad"])
    return dict(
        command=dict(xVelMS=command[0], yVelMS=command[1], thetaVelRadS=command[2]),
        wheelTargetsRadS=[round(v, 5) for v in targets], saturated=saturated,
        actualWheelRadS=[round(float(data.qvel[dofadr(model, w)]), 5) for w in WHEELS],
        forwardM=float(dx * cos + dy * sin), lateralM=float(-dx * sin + dy * cos),
        yawRad=float(wrap(end["yawRad"] - start["yawRad"])),
        finalSpeedMS=end["speedMS"], contacts=int(data.ncon), baseHeightM=end["zM"],
        durationSeconds=seconds,
    ), interventions, model, data


def trial_base_motion(args):
    results = {}
    interventions = []
    cases = [("forward", (0.15, 0, 0), 3.0), ("reverse", (-0.15, 0, 0), 3.0),
             ("lateralLeft", (0, 0.15, 0), 3.0), ("lateralRight", (0, -0.15, 0), 3.0),
             ("yawPositive", (0, 0, 0.8), 3.0), ("yawNegative", (0, 0, -0.8), 3.0),
             ("mixed", (0.12, 0.08, 0.4), 3.0), ("zeroCommand", (0, 0, 0), 3.0)]
    for name, command, seconds in cases:
        record, extra, _, _ = run_base_command("base", command, seconds, args)
        results[name] = record
        interventions.extend(extra)
    # stop/coast behaviour: commanded stop, then a declared motor-disabled coast
    model, data, path = load_model("base", args.timestep, args.iterations)
    for _ in range(int(1.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    targets, _ = body_to_wheel(0.20, 0, 0)
    for name, value in zip(WHEELS, targets):
        data.ctrl[aid(model, name)] = float(value)
    for _ in range(int(4.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    before = base_pose(model, data)
    for name in WHEELS:
        data.ctrl[aid(model, name)] = 0.0
    for _ in range(int(4.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    after = base_pose(model, data)
    results["servoBrakedStop"] = dict(speedBeforeMS=before["speedMS"], speedAfterMS=after["speedMS"],
                                      travelM=float(math.hypot(after["xM"] - before["xM"], after["yM"] - before["yM"])))
    model, data, path = load_model("base", args.timestep, args.iterations)
    for _ in range(int(1.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    for name, value in zip(WHEELS, targets):
        data.ctrl[aid(model, name)] = float(value)
    for _ in range(int(4.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    before = base_pose(model, data)
    for name in WHEELS:
        index = aid(model, name)
        model.actuator_forcerange[index] = [0.0, 0.0]
        model.actuator_gainprm[index, 0] = 0.0
        data.ctrl[index] = 0.0
    interventions.append("free-wheel coast trial zeroes all three wheel actuator gains and force ranges (declared motor-disabled condition)")
    for _ in range(int(6.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    after = base_pose(model, data)
    results["freeWheelCoast"] = dict(speedBeforeMS=before["speedMS"], speedAfterMS=after["speedMS"],
                                     travelM=float(math.hypot(after["xM"] - before["xM"], after["yM"] - before["yM"])))
    return "base", results, interventions, [], []


def trial_lifted_base(args):
    record, extra, model, data = run_base_command("base_stand", (0.15, 0, 0), 3.0, args)
    record["standContacts"] = [f"{a}|{b}" for a, b in contact_pairs(model, data) if "test_stand" in (a + b)]
    record["floorRollerContacts"] = [f"{a}|{b}" for a, b in contact_pairs(model, data) if "floor" in (a + b) and "roller" in (a + b)]
    reference, _, _, _ = run_base_command("base", (0.15, 0, 0), 3.0, args)
    return "base_stand", dict(lifted=record, groundedReference=reference,
                              propulsionRatio=abs(record["forwardM"]) / max(1e-9, abs(reference["forwardM"]))), extra, [], []


def trial_traction_reduced(args):
    results = {}
    for label, model_name in (("nominal", "base"), ("reduced", "base_lowtraction")):
        for window in (0.5, 1.0, 3.0):
            record, _, model, data = run_base_command(model_name, (0.26, 0, 0), window, args)
            wheel = [float(data.qvel[dofadr(model, w)]) for w in WHEELS]
            predicted = abs(np.linalg.solve(
                np.array([[d[0], d[1], a] for d, a in zip(SHARED["driveDirections"], SHARED["momentArmsM"])]),
                np.array(wheel) * SHARED["wheelRadiusM"])[0])
            record["predictedFromWheelsMS"] = float(predicted)
            record["slipRatio"] = float(1 - record["finalSpeedMS"] / predicted) if predicted > 1e-9 else None
            results[f"{label}_{window}s"] = record
    results["travelRatio_0.5s"] = results["reduced_0.5s"]["forwardM"] / max(1e-9, results["nominal_0.5s"]["forwardM"])
    return "base_lowtraction", results, [], [], []


def trial_motor_disabled(args):
    results = {}
    all_off, extra_a, _, _ = run_base_command("base", (0.15, 0, 0), 3.0, args, disable_wheels=WHEELS)
    one_off, extra_b, _, _ = run_base_command("base", (0.15, 0, 0), 3.0, args, disable_wheels=["base_left_wheel"])
    nominal, _, _, _ = run_base_command("base", (0.15, 0, 0), 3.0, args)
    results.update(allWheelsDisabled=all_off, oneWheelDisabled=one_off, nominal=nominal)
    return "base", results, extra_a + extra_b, [], []


def place_base_at(model, data, x, y, yaw):
    adr = qadr(model, "lekiwi_base_free")
    data.qpos[adr:adr + 3] = [x, y, 0.0508]
    data.qpos[adr + 3:adr + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    mujoco.mj_forward(model, data)


def trial_known_poses(args):
    model, data, path = load_model("courier", args.timestep, args.iterations)
    wc = SHARED["workcell"]
    place_base_at(model, data, wc["serviceStopXYM"][0], wc["serviceStopXYM"][1], wc["serviceStopYawRad"])
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "arm_tool_site")
    poses = {}
    for name, values in SHARED["armPosesRad"].items():
        for joint, value in zip(ARM_JOINTS, values):
            data.qpos[qadr(model, joint)] = value
        data.qpos[qadr(model, "arm_gripper")] = SHARED["gripperOpenRad"]
        mujoco.mj_forward(model, data)
        poses[name] = dict(toolSiteM=[round(float(v), 5) for v in data.site_xpos[site]],
                           toolApproachAxis=[round(float(v), 5) for v in data.site_xmat[site].reshape(3, 3)[:, 0]])
    for joint, value in SHARED["stowPoseRad"].items():
        data.qpos[qadr(model, joint)] = value
    mujoco.mj_forward(model, data)
    arm_bodies = ["arm_shoulder", "arm_upper", "arm_lower", "arm_wrist", "arm_gripper_body", "arm_moving_jaw"]
    base = base_pose(model, data)
    stow_radius = max(math.hypot(float(data.xpos[bid(model, b)][0]) - base["xM"],
                                 float(data.xpos[bid(model, b)][1]) - base["yM"]) for b in arm_bodies)
    poses["stow"] = dict(toolSiteM=[round(float(v), 5) for v in data.site_xpos[site]],
                         maxArmBodyRadiusM=round(float(stow_radius), 5))
    rim = dict(pickupRimM=[wc["pickupXYM"][0], wc["pickupXYM"][1], 0.284],
               deliveryRimM=[wc["deliveryXYM"][0], wc["deliveryXYM"][1], 0.284])
    grip = np.array(poses["pick_grip"]["toolSiteM"])
    rim_point = np.array(rim["pickupRimM"])
    place = np.array(poses["drop_place"]["toolSiteM"])
    delivery_point = np.array(rim["deliveryRimM"])
    poses["referenceCheck"] = dict(
        gripToRimDistanceM=float(np.linalg.norm(grip[:2] - rim_point[:2])),
        placeToDeliveryDistanceM=float(np.linalg.norm(place[:2] - delivery_point[:2])),
        rimReference=rim)
    return "courier", poses, [], [], []


def build_courier(args, weak_grip=False, miss=False, low_traction_floor=False, budget_steps=None,
                  drop_after_lift=False, lateral_only=False):
    model, data, path = load_model("courier", args.timestep, args.iterations)
    interventions = []
    if low_traction_floor:
        for g in range(model.ngeom):
            name = geom_name(model, g)
            if "roller" in name or name == "lekiwi_floor":
                model.geom_friction[g, 0] = 0.02
        interventions.append("courier floor and roller sliding friction reduced to 0.02")
    if weak_grip:
        index = aid(model, "arm_gripper")
        model.actuator_forcerange[index] = [-0.02, 0.02]
        interventions.append("gripper actuator force range reduced to +/-0.02 N m (declared weak-grip condition)")
    for joint, value in SHARED["stowPoseRad"].items():
        data.qpos[qadr(model, joint)] = value
        index = aid(model, joint)
        if index >= 0:
            data.ctrl[index] = value
    mujoco.mj_forward(model, data)
    evaluator = CourierEvaluator(SHARED["taskGoal"], SHARED["workcell"])
    runner = Runner(model, data, evaluator)
    if budget_steps is not None:
        runner.set_budget(budget_steps)
    return model, data, path, runner, evaluator, interventions


def courier_sequence(runner, args, miss=False, drop_after_lift=False):
    wc = SHARED["workcell"]
    durations = SHARED["stageDurationsSeconds"]
    stow = SHARED["stowPoseRad"]
    poses = {name: arm_pose_dict(values) for name, values in SHARED["armPosesRad"].items()}
    stow_arm = {k: v for k, v in stow.items() if k != "arm_gripper"}
    open_g, close_g = SHARED["gripperOpenRad"], SHARED["gripperCloseRad"]
    grip_pose = poses["pick_grip"]
    if miss:
        # A deliberately mistimed grasp: the jaws close 45 mm short of the rim.
        grip_pose = poses["pick_approach"]
    runner.arm_command(stow_arm, open_g)
    if not runner.advance(durations["settle"], "settle"):
        return False
    if not runner.advance(durations["stow_arm"], "stow_arm"):
        return False
    outbound = [dict(id="route_service_stop", xM=wc["serviceStopXYM"][0], yM=wc["serviceStopXYM"][1], yawRad=wc["serviceStopYawRad"])]
    reached, ok = drive_to(runner, outbound, SHARED["driveTimeoutSeconds"]["drive_to_service"], stow_arm, open_g, "drive_to_service")
    runner.route = list(reached)
    if not ok:
        return False
    runner.wheel_command(0.0, 0.0, 0.0)
    if not runner.advance(durations["stop_at_service"], "stop_at_service"):
        return False
    for stage, pose, gripper in (("approach_beaker", poses["pick_approach"], open_g),
                                 ("reach_rim", grip_pose, open_g),
                                 ("close_gripper", grip_pose, close_g),
                                 ("lift_beaker", poses["pick_lift"], close_g)):
        runner.arm_command(pose, gripper)
        if not runner.advance(durations[stage], stage):
            return False
    if drop_after_lift:
        runner.arm_command(poses["carry"], open_g)
        if not runner.advance(durations["carry_beaker"], "carry_beaker"):
            return False
        runner.arm_command(poses["drop_over"], open_g)
        if not runner.advance(durations["over_delivery"], "over_delivery"):
            return False
    else:
        for stage, pose, gripper in (("carry_beaker", poses["carry"], close_g),
                                     ("over_delivery", poses["drop_over"], close_g),
                                     ("lower_beaker", poses["drop_place"], close_g),
                                     ("release_beaker", poses["drop_place"], open_g),
                                     ("retreat_arm", poses["drop_retreat"], open_g)):
            runner.arm_command(pose, gripper)
            if not runner.advance(durations[stage], stage):
                return False
    runner.arm_command(stow_arm, open_g)
    if not runner.advance(durations["restow_arm"], "restow_arm"):
        return False
    inbound = [dict(id="route_home", xM=wc["homeXYM"][0], yM=wc["homeXYM"][1], yawRad=wc["serviceStopYawRad"]),
               dict(id="route_home_heading", xM=wc["homeXYM"][0], yM=wc["homeXYM"][1], yawRad=0.0)]
    reached, ok = drive_to(runner, inbound, SHARED["driveTimeoutSeconds"]["drive_home"], stow_arm, open_g, "drive_home")
    runner.route = getattr(runner, "route", []) + list(reached)
    if not ok:
        return False
    runner.wheel_command(0.0, 0.0, 0.0)
    return runner.advance(durations["stop_home"], "stop_home")


def trial_courier(args, **kwargs):
    model, data, path, runner, evaluator, interventions = build_courier(args, **kwargs)
    completed = courier_sequence(runner, args, miss=kwargs.get("miss", False), drop_after_lift=kwargs.get("drop_after_lift", False))
    metrics = evaluator.metrics()
    metrics["programCompleted"] = bool(completed)
    if not completed:
        metrics["success"] = False
    return "courier", metrics, interventions, runner.commands, runner.trajectory


def trial_manipulation(args):
    """Manipulation gate at a fixed base pose, separated from route planning."""
    model, data, path, runner, evaluator, interventions = build_courier(args)
    wc = SHARED["workcell"]
    place_base_at(model, data, wc["serviceStopXYM"][0], wc["serviceStopXYM"][1], wc["serviceStopYawRad"])
    interventions.append("manipulation gate places the base at the configured service stop as an explicit logged setup, then runs only physical manipulation")
    durations = SHARED["stageDurationsSeconds"]
    poses = {name: arm_pose_dict(values) for name, values in SHARED["armPosesRad"].items()}
    stow_arm = {k: v for k, v in SHARED["stowPoseRad"].items() if k != "arm_gripper"}
    open_g, close_g = SHARED["gripperOpenRad"], SHARED["gripperCloseRad"]
    runner.arm_command(stow_arm, open_g)
    runner.advance(durations["settle"], "settle")
    for stage, pose, gripper in (("approach_beaker", poses["pick_approach"], open_g),
                                 ("reach_rim", poses["pick_grip"], open_g),
                                 ("close_gripper", poses["pick_grip"], close_g),
                                 ("lift_beaker", poses["pick_lift"], close_g),
                                 ("carry_beaker", poses["carry"], close_g),
                                 ("over_delivery", poses["drop_over"], close_g),
                                 ("lower_beaker", poses["drop_place"], close_g),
                                 ("release_beaker", poses["drop_place"], open_g),
                                 ("retreat_arm", poses["drop_retreat"], open_g),
                                 ("restow_arm", stow_arm, open_g)):
        runner.arm_command(pose, gripper)
        runner.advance(durations[stage], stage)
    metrics = evaluator.metrics()
    metrics["manipulationSuccess"] = bool(metrics["graspSeen"] and metrics["liftSeen"] and metrics["carrySeen"]
                                          and metrics["supportWhileHeldSeen"] and metrics["releaseSeen"] and metrics["settled"])
    return "courier", metrics, interventions, runner.commands, runner.trajectory


def trial_lateral_carry(args):
    """Holonomic lateral base motion while a physically held payload stays gripped."""
    model, data, path, runner, evaluator, interventions = build_courier(args)
    wc = SHARED["workcell"]
    place_base_at(model, data, wc["serviceStopXYM"][0], wc["serviceStopXYM"][1], wc["serviceStopYawRad"])
    interventions.append("lateral-carry gate places the base at the configured service stop as an explicit logged setup")
    durations = SHARED["stageDurationsSeconds"]
    poses = {name: arm_pose_dict(values) for name, values in SHARED["armPosesRad"].items()}
    stow_arm = {k: v for k, v in SHARED["stowPoseRad"].items() if k != "arm_gripper"}
    open_g, close_g = SHARED["gripperOpenRad"], SHARED["gripperCloseRad"]
    runner.arm_command(stow_arm, open_g)
    runner.advance(durations["settle"], "settle")
    for stage, pose, gripper in (("approach_beaker", poses["pick_approach"], open_g),
                                 ("reach_rim", poses["pick_grip"], open_g),
                                 ("close_gripper", poses["pick_grip"], close_g),
                                 ("lift_beaker", poses["pick_lift"], close_g)):
        runner.arm_command(pose, gripper)
        runner.advance(durations[stage], stage)
    held_before = beaker_contacts(model, data)
    start = base_pose(model, data)
    beaker_start = data.xpos[bid(model, "empty_beaker")].copy()
    # Pure body-lateral command in both directions while the payload stays gripped.
    segments = []
    for direction in (1.0, -1.0):
        runner.wheel_command(0.0, direction * 0.12, 0.0)
        runner.advance(1.6, "lateral_carry")
        runner.wheel_command(0.0, 0.0, 0.0)
        runner.advance(0.6, "lateral_settle")
        pose = base_pose(model, data)
        dx = pose["xM"] - start["xM"]
        dy = pose["yM"] - start["yM"]
        cos, sin = math.cos(start["yawRad"]), math.sin(start["yawRad"])
        fixed, moving, support = beaker_contacts(model, data)
        segments.append(dict(direction=direction,
                             forwardM=float(dx * cos + dy * sin), lateralM=float(-dx * sin + dy * cos),
                             stillGrasped=bool(fixed and moving), supportContact=bool(support),
                             beakerM=[round(float(v), 5) for v in data.xpos[bid(model, "empty_beaker")]]))
    beaker_end = data.xpos[bid(model, "empty_beaker")].copy()
    metrics = evaluator.metrics()
    metrics.update(
        lateralSegments=segments,
        heldBeforeLateral=bool(held_before[0] and held_before[1]),
        stillGraspedAfter=bool(beaker_contacts(model, data)[0] and beaker_contacts(model, data)[1]),
        beakerTravelWithBaseM=float(np.linalg.norm(beaker_end[:2] - beaker_start[:2])),
        maxLateralM=float(max(abs(s["lateralM"]) for s in segments)),
        maxForwardDuringLateralM=float(max(abs(s["forwardM"]) for s in segments)),
    )
    metrics["lateralCarrySuccess"] = bool(metrics["heldBeforeLateral"] and metrics["stillGraspedAfter"]
                                          and metrics["maxLateralM"] >= 0.10)
    return "courier", metrics, interventions, runner.commands, runner.trajectory


def trial_restricted_route(args):
    """A route that drives through the visible restricted stop must fail the evaluator."""
    model, data, path, runner, evaluator, interventions = build_courier(args)
    wc = SHARED["workcell"]
    stow_arm = {k: v for k, v in SHARED["stowPoseRad"].items() if k != "arm_gripper"}
    runner.arm_command(stow_arm, SHARED["gripperOpenRad"])
    runner.advance(0.6, "settle")
    waypoints = [dict(id="restricted_stop", xM=wc["restrictedStopXYM"][0], yM=wc["restrictedStopXYM"][1], yawRad=0.0)]
    interventions.append("restricted-route trial deliberately commands the base into the visible restricted stop region")
    drive_to(runner, waypoints, 18.0, stow_arm, SHARED["gripperOpenRad"], "restricted")
    metrics = evaluator.metrics()
    return "courier", metrics, interventions, runner.commands, runner.trajectory


TRIALS = {
    "wheel-reference": trial_wheel_reference,
    "base-motion": trial_base_motion,
    "lifted-base": trial_lifted_base,
    "traction-reduced": trial_traction_reduced,
    "motor-disabled": trial_motor_disabled,
    "known-poses": trial_known_poses,
    "manipulation": trial_manipulation,
    "lateral-carry": trial_lateral_carry,
    "courier": lambda args: trial_courier(args),
    "missed-grasp": lambda args: trial_courier(args, miss=True),
    "weak-grip": lambda args: trial_courier(args, weak_grip=True),
    "dropped-payload": lambda args: trial_courier(args, drop_after_lift=True),
    "insufficient-budget": lambda args: trial_courier(args, budget_steps=4000),
    "restricted-route": trial_restricted_route,
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trial", default="courier", choices=sorted(TRIALS))
    parser.add_argument("--timestep", type=float, default=None)
    parser.add_argument("--iterations", type=int, default=None)
    parser.add_argument("--trace", action="store_true", help="include the sampled trajectory in the report")
    args = parser.parse_args()

    model_name, metrics, interventions, commands, trajectory = TRIALS[args.trial](args)
    model, _, path = load_model(model_name, args.timestep, args.iterations)
    package_id, model_id = MODEL_PACKAGE_IDS[model_name]
    report = dict(
        schema="robobuddy.lekiwi.native-reference.v1",
        classification="native/browser model implementation comparison; software and numerical verification only, not hardware validation",
        trial=args.trial,
        model=dict(packageId=package_id, id=model_id, asset=f"models/lekiwi/{MODEL_FILES[model_name]}", sha256=sha256_of(path)),
        source=dict(lekiwi="SIGRobotics-UIUC/LeKiwi@efa608d7ee5a495a4803b1d28cd0c955b4f1e033",
                    lerobot="huggingface/lerobot@7e241bd630a3719a56157a497ce5d08f244784f1",
                    soArm101="google-deepmind/mujoco_menagerie@8161bba264d7fa7c99ca301e91e7fb44737676ad",
                    legacyTask="jivishov/RoboBuddy_AI@75fe2669c0ab0b029986de424c69162071174df8"),
        controller=dict(id="lekiwi-beaker-courier-v1", sha256=controller_sha256(),
                        wheelMapping="pinned LeRobot Kiwi form with LeKiwi URDF-derived per-wheel geometry",
                        actuator="bounded velocity servo per wheel, bounded position servo per arm joint"),
        engine=dict(name="MuJoCo", version=mujoco.__version__, timestepSeconds=float(model.opt.timestep),
                    integrator=int(model.opt.integrator), iterations=int(model.opt.iterations),
                    lsIterations=int(model.opt.ls_iterations), cone=int(model.opt.cone)),
        observationPeriodSeconds=SHARED["observationPeriodSeconds"],
        setupInterventions=interventions,
        metrics=metrics,
    )
    if commands:
        report["boundedWheelCommands"] = commands[:400]
        report["commandCount"] = len(commands)
    if trajectory and args.trace:
        report["trajectory"] = trajectory
    elif trajectory:
        report["trajectorySampleCount"] = len(trajectory)
        report["trajectory"] = trajectory[:: max(1, len(trajectory) // 40)]
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
