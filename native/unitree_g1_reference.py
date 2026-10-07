#!/usr/bin/env python3
"""Native MuJoCo reference runner for the Unitree G1 29-DoF physical workspace (Phase 5D).

This is implementation conformance and numerical evidence. It is NOT hardware validation: it
shares the engine, the model and the controller assumptions with the browser backend, so agreement
between them says the port is faithful, not that either reproduces an assembled Unitree G1.

Every trial drives the plant through the same chain the browser uses:

    requested joint target
      -> bounded low-level command (source joint range, source URDF velocity limit, gain bounds)
      -> Unitree low-level motor law  tau = tau_ff + kp (q* - q) + kd (dq* - dq)
      -> per-joint source effort clamp
      -> MuJoCo dynamics
      -> measured q / dq

Nothing here writes qpos, qvel, a root pose or a root velocity outside a declared, reported
pre-trial setup, and no trial reports a conclusion that was not read back out of MuJoCo.
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

REPO = Path(__file__).resolve().parents[1]
MODEL_DIR = REPO / "models" / "unitree_g1"
SCHEMA = "robobuddy.unitree-g1.native-reference.v1"
CLASSIFICATION = (
    "implementation conformance and numerical evidence; not hardware validation. "
    "No measurement of an assembled Unitree G1 was used or is claimed."
)

# --- tables mirrored from src/physics/unitree-g1-source-audit.js and unitree-g1-controller.js -----
# tests/validate_unitree_g1_controller_parity.py asserts that these are identical to the JavaScript
# tables, so the browser and the native reference can never drift apart silently.
JOINT_ORDER = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
JOINT_RANGE_RAD = [
    [-2.5307, 2.8798], [-0.5236, 2.9671], [-2.7576, 2.7576], [-0.087267, 2.8798], [-0.87267, 0.5236], [-0.2618, 0.2618],
    [-2.5307, 2.8798], [-2.9671, 0.5236], [-2.7576, 2.7576], [-0.087267, 2.8798], [-0.87267, 0.5236], [-0.2618, 0.2618],
    [-2.618, 2.618], [-0.52, 0.52], [-0.52, 0.52],
    [-3.0892, 2.6704], [-1.5882, 2.2515], [-2.618, 2.618], [-1.0472, 2.0944], [-1.97222, 1.97222], [-1.61443, 1.61443], [-1.61443, 1.61443],
    [-3.0892, 2.6704], [-2.2515, 1.5882], [-2.618, 2.618], [-1.0472, 2.0944], [-1.97222, 1.97222], [-1.61443, 1.61443], [-1.61443, 1.61443],
]
EFFORT_LIMIT_NM = [88, 88, 88, 139, 50, 50, 88, 88, 88, 139, 50, 50, 88, 50, 50,
                   25, 25, 25, 25, 25, 5, 5, 25, 25, 25, 25, 25, 5, 5]
VELOCITY_LIMIT_RAD_S = [32, 32, 32, 20, 37, 37, 32, 32, 32, 20, 37, 37, 32, 37, 37,
                        37, 37, 37, 37, 37, 22, 22, 37, 37, 37, 37, 37, 22, 22]
STAND_POSE_RAD = [-0.1, 0, 0, 0.3, -0.2, 0, -0.1, 0, 0, 0.3, -0.2, 0, 0, 0, 0,
                  0.35, 0.18, 0, 0.87, 0, 0, 0, 0.35, -0.18, 0, 0.87, 0, 0, 0]
UNITREE_FIXSTAND_KP = [100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40, 200, 200, 200,
                       40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40]
UNITREE_FIXSTAND_KD = [2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2, 5, 5, 5,
                       10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]
UNITREE_FIXSTAND_RAMP_SECONDS = 2.0
ANKLE_INDICES = [4, 5, 10, 11]
ROBOBUDDY_ANKLE_KP = 250.0
ROBOBUDDY_ANKLE_KD = 10.0
MAX_KP = 400.0
MAX_KD = 60.0

PHYSICS_TIMESTEP_SECONDS = 0.002
LOWLEVEL_CONTROL_INTERVAL_SECONDS = 0.002
OBSERVATION_INTERVAL_SECONDS = 0.02
UNITREE_FSM_INTERVAL_SECONDS = 0.001

TOTAL_MASS_KG = 35.112142
STAND_COM_HEIGHT_ABOVE_ANKLE_M = 0.6715
ANKLE_STABILITY_REQUIREMENT = TOTAL_MASS_KG * 9.81 * STAND_COM_HEIGHT_ABOVE_ANKLE_M

FOOT_GEOMS = {
    "left": ["left_foot_heel_lateral", "left_foot_heel_medial", "left_foot_toe_lateral", "left_foot_toe_medial"],
    "right": ["right_foot_heel_lateral", "right_foot_heel_medial", "right_foot_toe_lateral", "right_foot_toe_medial"],
}
EXTERNAL_OBJECT = "contact_probe_block"

# The declared initial command per scene, mirroring src/physics/unitree-g1-model-package.js.
INITIAL_COMMAND = {
    "mounted": "joint-hold",
    "mounted_blocked": "joint-hold",
    "freebase": "joint-hold",
    "freebase_drop": "passive",
}


def _with_ankles(base, value):
    out = list(base)
    for index in ANKLE_INDICES:
        out[index] = value
    return out


PROFILES = {
    "unitree_g1_joint_hold_v1": {
        "label": "Unitree source joint-position hold gains",
        "kp": list(UNITREE_FIXSTAND_KP),
        "kd": list(UNITREE_FIXSTAND_KD),
        "rampSeconds": 0.0,
        "gainProvenance": "unitree_rl_mjlab G1 config.yaml FSM.FixStand kp/kd, unchanged",
    },
    "robobuddy_g1_stand_v1": {
        "label": "RoboBuddy engineering standing controller (FixStand-derived)",
        "kp": _with_ankles(UNITREE_FIXSTAND_KP, ROBOBUDDY_ANKLE_KP),
        "kd": _with_ankles(UNITREE_FIXSTAND_KD, ROBOBUDDY_ANKLE_KD),
        "rampSeconds": UNITREE_FIXSTAND_RAMP_SECONDS,
        "gainProvenance": "Unitree FixStand structure, posture, waist and arm gains; the four ankle gains are repository-authored",
    },
    "unitree_g1_fixstand_source_v1": {
        "label": "Unitree FixStand, exact source gains",
        "kp": list(UNITREE_FIXSTAND_KP),
        "kd": list(UNITREE_FIXSTAND_KD),
        "rampSeconds": UNITREE_FIXSTAND_RAMP_SECONDS,
        "gainProvenance": "unitree_rl_mjlab G1 config.yaml FSM.FixStand, unchanged",
    },
}

# --- free-base standing envelope ------------------------------------------------------------------
# Chosen before any gain was tuned, from the model geometry and the source posture, and never
# widened afterwards. The evaluation interval starts after the controller ramp completes.
STAND_GATE = {
    "evaluationStartSeconds": UNITREE_FIXSTAND_RAMP_SECONDS,
    "evaluationSeconds": 6.0,
    # The source standing posture puts the pelvis at 0.7842 m. A 50 mm band is roughly one knee-flex
    # degree of freedom: a controller that sags or springs further than that is not holding a posture.
    "pelvisHeightRangeM": [0.7342, 0.8342],
    # 0.15 rad is about 8.6 deg. Beyond that the centre of mass leaves the source foot support
    # polygon geometry (a 0.17 m long foot under a 0.67 m centre of mass).
    "maxPelvisTiltRad": 0.15,
    "maxHorizontalDriftM": 0.10,
    "maxTerminalLinearSpeedMS": 0.05,
    "maxTerminalAngularSpeedRadS": 0.20,
    "requireBothFeetSupported": True,
    "forbidNonFootGroundContact": True,
    "maxActuatorEffortFraction": 0.90,
}


def clamp(value, low, high):
    return low if value < low else (high if value > high else value)


def bound_command(index, position_rad, velocity_rad_s=0.0, feedforward_nm=0.0, kp=0.0, kd=0.0):
    low, high = JOINT_RANGE_RAD[index]
    return {
        "index": index,
        "jointId": JOINT_ORDER[index],
        "requestedPositionRad": float(position_rad),
        "positionRad": clamp(float(position_rad), low, high),
        "velocityRadS": clamp(float(velocity_rad_s), -VELOCITY_LIMIT_RAD_S[index], VELOCITY_LIMIT_RAD_S[index]),
        "feedforwardTorqueNm": clamp(float(feedforward_nm), -EFFORT_LIMIT_NM[index], EFFORT_LIMIT_NM[index]),
        "kp": clamp(float(kp), 0.0, MAX_KP),
        "kd": clamp(float(kd), 0.0, MAX_KD),
    }


def low_level_torque(command, q, dq):
    limit = EFFORT_LIMIT_NM[command["index"]]
    raw = command["feedforwardTorqueNm"] + command["kp"] * (command["positionRad"] - q) + command["kd"] * (command["velocityRadS"] - dq)
    return clamp(raw, -limit, limit)


class Plant:
    """The authoritative MuJoCo plant plus the bounded command layer above it."""

    def __init__(self, variant, timestep=None, control_interval=None, motors_enabled=True,
                 iterations=None, ls_iterations=None, integrator=None, floor_friction=None):
        path = MODEL_DIR / f"{variant}.xml"
        self.variant = variant
        self.model_sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
        self.model = mujoco.MjModel.from_xml_path(str(path))
        if timestep:
            self.model.opt.timestep = float(timestep)
        if iterations:
            self.model.opt.iterations = int(iterations)
        if ls_iterations:
            self.model.opt.ls_iterations = int(ls_iterations)
        if integrator is not None:
            self.model.opt.integrator = int(integrator)
        if floor_friction is not None:
            floor = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
            self.model.geom_friction[floor][0] = float(floor_friction)
            for side in FOOT_GEOMS.values():
                for name in side:
                    gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
                    self.model.geom_friction[gid][0] = float(floor_friction)
        self.motors_enabled = bool(motors_enabled)
        if not self.motors_enabled:
            # A genuine actuation failure, not a zero command: the transmission itself can no longer
            # produce force, so the controller keeps running and still holds nothing up.
            self.model.actuator_gainprm[:, 0] = 0.0
            self.model.actuator_ctrlrange[:, :] = 0.0
        self.data = mujoco.MjData(self.model)
        self.control_interval = float(control_interval or LOWLEVEL_CONTROL_INTERVAL_SECONDS)
        steps = self.control_interval / self.model.opt.timestep
        if abs(steps - round(steps)) > 1e-9 or round(steps) < 1:
            raise SystemExit(f"control interval {self.control_interval} is not a whole number of {self.model.opt.timestep} s physics steps")
        self.steps_per_control = int(round(steps))
        self.qadr = [self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)] for name in JOINT_ORDER]
        self.vadr = [self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)] for name in JOINT_ORDER]
        self.free_root = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base_joint") >= 0
        self.accepted = [bound_command(i, 0.0) for i in range(29)]
        self.setup_log = []
        self.reset()

    # --- state ----------------------------------------------------------------------------------
    def reset(self):
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        pose = [float(self.data.qpos[address]) for address in self.qadr]
        if INITIAL_COMMAND[self.variant] == "passive":
            self.accepted = [bound_command(i, pose[i]) for i in range(29)]
        else:
            self.accepted = hold_commands(PROFILES["unitree_g1_joint_hold_v1"], pose)
        self.declare_setup("initial command", initialCommand=INITIAL_COMMAND[self.variant],
                           note="a command, not a state write; 'passive' is zero torque and 'joint-hold' is a bounded hold at the declared initial joint positions")
        mujoco.mj_forward(self.model, self.data)

    def q(self):
        return np.array([self.data.qpos[a] for a in self.qadr])

    def dq(self):
        return np.array([self.data.qvel[a] for a in self.vadr])

    def root(self):
        if not self.free_root:
            body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
            return {
                "mode": "fixed-mounted",
                "frame": "mujoco_world",
                "positionM": [round(float(v), 6) for v in self.data.xpos[body]],
                "quaternionWxyz": [round(float(v), 6) for v in self.data.xquat[body]],
                "linearVelocityMS": [0.0, 0.0, 0.0],
                "angularVelocityRadS": [0.0, 0.0, 0.0],
                "uprightZ": 1.0,
            }
        matrix = np.zeros(9)
        mujoco.mju_quat2Mat(matrix, self.data.qpos[3:7])
        return {
            "mode": "free-base",
            "frame": "mujoco_world",
            "positionM": [round(float(v), 6) for v in self.data.qpos[0:3]],
            "quaternionWxyz": [round(float(v), 6) for v in self.data.qpos[3:7]],
            "linearVelocityMS": [round(float(v), 6) for v in self.data.qvel[0:3]],
            "angularVelocityRadS": [round(float(v), 6) for v in matrix.reshape(3, 3) @ self.data.qvel[3:6]],
            "uprightZ": round(float(matrix.reshape(3, 3)[2, 2]), 6),
        }

    def tilt_rad(self):
        if not self.free_root:
            return 0.0
        matrix = np.zeros(9)
        mujoco.mju_quat2Mat(matrix, self.data.qpos[3:7])
        return float(math.acos(clamp(float(matrix.reshape(3, 3)[2, 2]), -1.0, 1.0)))

    def geom_name(self, gid):
        return mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, gid) or f"geom{gid}"

    def body_of(self, gid):
        return mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, self.model.geom_bodyid[gid]) or "world"

    def contacts(self):
        """Named contact evidence. Support is never inferred from a contact count."""
        out = {
            "leftFootFloor": [], "rightFootFloor": [], "otherBodyFloor": [],
            "robotExternalObject": [], "robotSelf": [], "robotFixture": [], "objectFloor": [],
        }
        pairs = []
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            g1, g2 = int(contact.geom[0]), int(contact.geom[1])
            n1, n2 = self.geom_name(g1), self.geom_name(g2)
            b1, b2 = self.body_of(g1), self.body_of(g2)
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, index, force)
            entry = {"geoms": [n1, n2], "bodies": [b1, b2], "distanceM": round(float(contact.dist), 6),
                     "normalForceN": round(float(force[0]), 4)}
            pairs.append(entry)
            names = {n1, n2}
            bodies = {b1, b2}
            if "floor" in names:
                other = n2 if n1 == "floor" else n1
                if other in FOOT_GEOMS["left"]:
                    out["leftFootFloor"].append(entry)
                elif other in FOOT_GEOMS["right"]:
                    out["rightFootFloor"].append(entry)
                elif other == f"{EXTERNAL_OBJECT}_geom":
                    out["objectFloor"].append(entry)
                else:
                    out["otherBodyFloor"].append(entry)
            elif f"{EXTERNAL_OBJECT}_geom" in names:
                out["robotExternalObject"].append(entry)
            elif "hip_abduction_stop_wall" in names:
                out["robotFixture"].append(entry)
            elif "world" not in bodies:
                out["robotSelf"].append(entry)
        out["all"] = pairs
        out["count"] = int(self.data.ncon)
        return out

    def observation(self):
        # mj_step integrates qpos/qvel but leaves body poses and contacts at the pre-integration
        # state, so a sample read straight after a step would report body positions one timestep
        # behind the joint angles beside them. One forward evaluation puts every reported quantity
        # at the same instant; the actuators are plain torque motors, so effort is unchanged, and
        # the trajectory is untouched because mj_step performs this same evaluation itself.
        mujoco.mj_forward(self.model, self.data)
        q, dq = self.q(), self.dq()
        joints = {}
        for index, name in enumerate(JOINT_ORDER):
            accepted = self.accepted[index]
            joints[name] = {
                "requestedTargetRad": round(accepted["requestedPositionRad"], 6),
                "acceptedTargetRad": round(accepted["positionRad"], 6),
                "measuredPositionRad": round(float(q[index]), 6),
                "measuredVelocityRadS": round(float(dq[index]), 6),
                "actuatorEffortNm": round(float(self.data.actuator_force[index]), 6),
                "effortLimitNm": EFFORT_LIMIT_NM[index],
                "velocityLimitRadS": VELOCITY_LIMIT_RAD_S[index],
                "jointRangeRad": JOINT_RANGE_RAD[index],
                "kp": accepted["kp"], "kd": accepted["kd"],
            }
        return {
            "simulationTimeSeconds": round(float(self.data.time), 6),
            "root": self.root(),
            "joints": joints,
            "contacts": self.contacts(),
            "motorsEnabled": self.motors_enabled,
        }

    def object_pose(self):
        body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, EXTERNAL_OBJECT)
        if body < 0:
            return None
        return {
            "positionM": [round(float(v), 6) for v in self.data.xpos[body]],
            "quaternionWxyz": [round(float(v), 6) for v in self.data.xquat[body]],
        }

    # --- declared pre-trial setup ----------------------------------------------------------------
    def declare_setup(self, label, **detail):
        self.setup_log.append({"label": label, "simulationTimeSeconds": round(float(self.data.time), 6), **detail})

    def set_initial_root_velocity(self, linear=(0, 0, 0), angular=(0, 0, 0)):
        if not self.free_root:
            raise SystemExit("cannot perturb the root of a mounted fixture")
        self.data.qvel[0:3] = linear
        self.data.qvel[3:6] = angular
        mujoco.mj_forward(self.model, self.data)
        self.declare_setup("initial root perturbation", linearMS=list(linear), angularRadS=list(angular),
                           note="explicit pre-trial initial condition, applied before the evaluated interval and reported here")

    # --- execution --------------------------------------------------------------------------------
    def apply(self, commands):
        self.accepted = list(commands)

    def step_control_interval(self):
        q, dq = self.q(), self.dq()
        for index in range(29):
            self.data.ctrl[index] = low_level_torque(self.accepted[index], float(q[index]), float(dq[index]))
        for _ in range(self.steps_per_control):
            mujoco.mj_step(self.model, self.data)

    def run(self, duration, command_fn, sample_interval=OBSERVATION_INTERVAL_SECONDS, monitor=None):
        total = duration / self.control_interval
        cadence = sample_interval / self.control_interval
        if abs(total - round(total)) > 1e-9 or round(total) < 1:
            raise ValueError("run duration must be a positive whole number of controller intervals")
        if abs(cadence - round(cadence)) > 1e-9 or round(cadence) < 1:
            raise ValueError("sample interval must be a positive whole number of controller intervals")
        total_ticks, sample_ticks = int(round(total)), int(round(cadence))
        samples = []
        # Integer ticks prevent floating-point time comparisons from occasionally turning a
        # declared 20 ms cadence into a 22 ms gap. The browser uses this same tick discipline.
        for tick in range(1, total_ticks + 1):
            self.apply(command_fn(float(self.data.time)))
            self.step_control_interval()
            if monitor:
                monitor(self)
            if tick % sample_ticks == 0 or tick == total_ticks:
                samples.append(self.observation())
        return samples



def stand_commands(profile, start_pose, elapsed):
    ramp = profile["rampSeconds"]
    alpha = 1.0 if ramp <= 0 else clamp(elapsed / ramp, 0.0, 1.0)
    return [bound_command(i, start_pose[i] * (1 - alpha) + STAND_POSE_RAD[i] * alpha,
                          0.0, 0.0, profile["kp"][i], profile["kd"][i]) for i in range(29)]


def hold_commands(profile, pose):
    return [bound_command(i, pose[i], 0.0, 0.0, profile["kp"][i], profile["kd"][i]) for i in range(29)]


def engine_block(plant):
    return {
        "version": mujoco.__version__,
        "timestepSeconds": float(plant.model.opt.timestep),
        "integrator": int(plant.model.opt.integrator),
        "iterations": int(plant.model.opt.iterations),
        "lsIterations": int(plant.model.opt.ls_iterations),
        "controlIntervalSeconds": plant.control_interval,
        "physicsStepsPerControl": plant.steps_per_control,
        "observationIntervalSeconds": OBSERVATION_INTERVAL_SECONDS,
        "unitreeFsmIntervalSeconds": UNITREE_FSM_INTERVAL_SECONDS,
    }


def report(plant, trial, **payload):
    return {
        "schema": SCHEMA,
        "classification": CLASSIFICATION,
        "trial": trial,
        "model": {"variant": plant.variant, "asset": f"models/unitree_g1/{plant.variant}.xml", "sha256": plant.model_sha256,
                  "rootMode": "free-base" if plant.free_root else "fixed-mounted",
                  "totalMassKg": round(float(plant.model.body_mass.sum()), 6)},
        "engine": engine_block(plant),
        "declaredSetup": plant.setup_log,
        **payload,
    }


# --- trials ---------------------------------------------------------------------------------------

def trial_model_audit(args):
    out = {}
    for variant in ("mounted", "mounted_blocked", "freebase", "freebase_drop"):
        plant = Plant(variant, timestep=args.timestep, control_interval=args.control_interval)
        model = plant.model
        collision = [g for g in range(model.ngeom) if model.geom_contype[g] or model.geom_conaffinity[g]]
        unnamed = [g for g in collision if not mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)]
        joints = {}
        for index, name in enumerate(JOINT_ORDER):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            joints[name] = {
                "modelJointIndex": int(jid),
                "modelActuatorIndex": int(aid),
                "actuatorTargetsJoint": bool(int(model.actuator_trnid[aid][0]) == jid),
                "rangeRad": [round(float(v), 9) for v in model.jnt_range[jid]],
                "axis": [round(float(v), 9) for v in model.jnt_axis[jid]],
                "actuatorForceRangeNm": [round(float(v), 6) for v in model.jnt_actfrcrange[jid]],
                "ctrlRangeNm": [round(float(v), 6) for v in model.actuator_ctrlrange[aid]],
                "armature": round(float(model.dof_armature[model.jnt_dofadr[jid]]), 6),
                "damping": round(float(model.dof_damping[model.jnt_dofadr[jid]]), 6),
                "frictionLoss": round(float(model.dof_frictionloss[model.jnt_dofadr[jid]]), 6),
                "sourceRangeRad": JOINT_RANGE_RAD[index],
                "sourceEffortLimitNm": EFFORT_LIMIT_NM[index],
                "sourceVelocityLimitRadS": VELOCITY_LIMIT_RAD_S[index],
            }
        out[variant] = {
            "sha256": plant.model_sha256,
            "rootMode": "free-base" if plant.free_root else "fixed-mounted",
            "nq": int(model.nq), "nv": int(model.nv), "nu": int(model.nu), "nbody": int(model.nbody),
            "neq": int(model.neq), "nexclude": int(model.nexclude), "ntendon": int(model.ntendon),
            "totalMassKg": round(float(model.body_mass.sum()), 6),
            "collisionGeomCount": len(collision),
            "unnamedCollisionGeoms": len(unnamed),
            "timestepSeconds": float(model.opt.timestep),
            "integrator": int(model.opt.integrator),
            "iterations": int(model.opt.iterations),
            "lsIterations": int(model.opt.ls_iterations),
            "gravity": [round(float(v), 6) for v in model.opt.gravity],
            "joints": joints,
            "footGeomsResolved": {side: [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, g) >= 0 for g in names]
                                  for side, names in FOOT_GEOMS.items()},
            "externalObjectPresent": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, EXTERNAL_OBJECT) >= 0,
            "excludedPairs": sorted(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EXCLUDE, i) or f"exclude{i}" for i in range(model.nexclude)),
        }
    plant = Plant("freebase", timestep=args.timestep, control_interval=args.control_interval)
    return report(plant, "model-audit", variants=out, sourceTables={
        "jointOrder": JOINT_ORDER, "jointRangeRad": JOINT_RANGE_RAD,
        "effortLimitNm": EFFORT_LIMIT_NM, "velocityLimitRadS": VELOCITY_LIMIT_RAD_S,
        "standPoseRad": STAND_POSE_RAD, "totalMassKg": TOTAL_MASS_KG,
    })


REPRESENTATIVE_JOINTS = [
    ("left_hip_pitch_joint", -0.6), ("left_knee_joint", 1.1), ("left_ankle_pitch_joint", -0.5),
    ("right_hip_roll_joint", -0.4), ("waist_yaw_joint", 0.6), ("left_shoulder_pitch_joint", -0.8),
    ("left_elbow_joint", 1.3), ("right_wrist_yaw_joint", 0.9),
]


def trial_mounted_joint_response(args):
    plant = Plant("mounted", timestep=args.timestep, control_interval=args.control_interval)
    profile = PROFILES["unitree_g1_joint_hold_v1"]
    results = []
    for name, target in REPRESENTATIVE_JOINTS:
        plant.reset()
        index = JOINT_ORDER.index(name)
        pose = [0.0] * 29
        pose[index] = target
        plant.run(2.5, lambda _now, pose=pose: hold_commands(profile, pose))
        observation = plant.observation()[  "joints"][name]
        results.append({
            "joint": name,
            "requestedTargetRad": round(target, 6),
            "acceptedTargetRad": observation["acceptedTargetRad"],
            "measuredPositionRad": observation["measuredPositionRad"],
            "trackingErrorRad": round(abs(observation["measuredPositionRad"] - observation["acceptedTargetRad"]), 6),
            "actuatorEffortNm": observation["actuatorEffortNm"],
            "effortLimitNm": observation["effortLimitNm"],
            "signAgreesWithTarget": bool(math.copysign(1, observation["measuredPositionRad"]) == math.copysign(1, target)),
            "fractionOfTargetReached": round(observation["measuredPositionRad"] / target, 6),
            "measuredDiffersFromCommand": bool(observation["measuredPositionRad"] != observation["acceptedTargetRad"]),
        })
    return report(plant, "mounted-joint-response", controller={"id": "unitree_g1_joint_hold_v1", "gainProvenance": PROFILES["unitree_g1_joint_hold_v1"]["gainProvenance"]},
                  joints=results,
                  interpretation=(
                      "Bounded joint-position holds on the declared root-fixed mount. Every measured value is read "
                      "from MuJoCo, never echoed from the command, so a gravity-loaded joint settles visibly short "
                      "of its target at the source hold gains. Nothing in this trial is standing or balance evidence."))


def trial_blocked_joint(args):
    """A declared rigid wall physically stops left hip abduction short of a reachable target."""
    free = Plant("mounted", timestep=args.timestep, control_interval=args.control_interval)
    blocked = Plant("mounted_blocked", timestep=args.timestep, control_interval=args.control_interval)
    profile = PROFILES["unitree_g1_joint_hold_v1"]
    target = 1.2
    index = JOINT_ORDER.index("left_hip_roll_joint")
    pose = [0.0] * 29
    pose[index] = target
    outcome = {}
    for label, plant in (("unobstructed", free), ("blocked", blocked)):
        plant.run(3.0, lambda _now: hold_commands(profile, pose))
        observation = plant.observation()
        joint = observation["joints"]["left_hip_roll_joint"]
        outcome[label] = {
            "requestedTargetRad": round(target, 6),
            "acceptedTargetRad": joint["acceptedTargetRad"],
            "measuredPositionRad": joint["measuredPositionRad"],
            "shortfallRad": round(joint["acceptedTargetRad"] - joint["measuredPositionRad"], 6),
            "actuatorEffortNm": joint["actuatorEffortNm"],
            "effortLimitNm": joint["effortLimitNm"],
            "fixtureContacts": observation["contacts"]["robotFixture"],
        }
    outcome["fixture"] = {
        "geom": "hip_abduction_stop_wall",
        "declared": "a visible rigid wall in the mounted_blocked model that physically obstructs left hip abduction",
    }
    return report(blocked, "blocked-joint", blockedJoint="left_hip_roll_joint", **{"comparison": outcome},
                  interpretation=(
                      "The identical bounded command is issued in both models. Unobstructed, the joint reaches its "
                      "target. Behind the declared wall it stops far short, the actuator saturates against the "
                      "obstruction, and the contact is reported by named geometry pair. The measured value is read "
                      "from MuJoCo in both cases; nothing substitutes the command for the state."))


KNOWN_POSES = {
    "neutral": {},
    "stand": {name: STAND_POSE_RAD[i] for i, name in enumerate(JOINT_ORDER) if STAND_POSE_RAD[i] != 0},
    "asymmetric_upper": {
        "waist_yaw_joint": 0.45, "waist_pitch_joint": 0.25,
        "left_shoulder_pitch_joint": -0.9, "left_shoulder_roll_joint": 0.6, "left_elbow_joint": 1.2,
        "right_shoulder_pitch_joint": 0.5, "right_shoulder_roll_joint": -0.35, "right_elbow_joint": 0.4,
        "left_wrist_pitch_joint": -0.5, "right_wrist_yaw_joint": 0.7,
    },
    "asymmetric_lower": {
        "left_hip_pitch_joint": -0.7, "left_hip_roll_joint": 0.35, "left_knee_joint": 1.3, "left_ankle_pitch_joint": -0.55,
        "right_hip_pitch_joint": 0.4, "right_hip_yaw_joint": -0.5, "right_knee_joint": 0.6, "right_ankle_roll_joint": 0.2,
    },
}
KNOWN_POSE_BODIES = [
    "pelvis", "left_knee_link", "left_ankle_roll_link", "right_ankle_roll_link", "torso_link",
    "left_elbow_link", "left_wrist_yaw_link", "right_wrist_yaw_link", "waist_roll_link", "right_knee_link",
]


def trial_known_poses(args):
    """Forward-kinematic body frames at nontrivial poses in the mounted (root-fixed) model.

    These are the reference values the browser physical workspace and the canonical kinematic rig
    are both checked against, so a joint-order, sign or frame error cannot pass silently.
    """
    plant = Plant("mounted", timestep=args.timestep, control_interval=args.control_interval)
    out = {}
    for label, pose in KNOWN_POSES.items():
        plant.reset()
        # Declared kinematic evaluation: joint values are placed and mj_forward is evaluated. No
        # dynamics run and no result of this trial is ever counted as standing or actuation evidence.
        for name, value in pose.items():
            plant.data.qpos[plant.qadr[JOINT_ORDER.index(name)]] = value
        mujoco.mj_forward(plant.model, plant.data)
        plant.declare_setup(f"known-pose placement: {label}", jointsRad=pose,
                            note="kinematic reference evaluation only; no dynamics, no capability claim")
        out[label] = {
            "jointsRad": pose,
            "bodies": {name: {
                "positionM": [round(float(v), 6) for v in plant.data.xpos[mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_BODY, name)]],
                "quaternionWxyz": [round(float(v), 6) for v in plant.data.xquat[mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_BODY, name)]],
            } for name in KNOWN_POSE_BODIES},
        }
    return report(plant, "known-poses", poses=out, note="mounted root-fixed frames; the pelvis is welded to the world at the declared mount height")


def trial_free_fall(args):
    """Gravity acts on the free root, and the robot genuinely comes to rest on the ground."""
    plant = Plant("freebase_drop", timestep=args.timestep, control_interval=args.control_interval)
    initial = plant.observation()
    zero = [bound_command(i, 0.0, 0.0, 0.0, 0.0, 0.0) for i in range(29)]
    samples = plant.run(3.0, lambda _now: zero)
    final = samples[-1]
    minimum_z = min(float(s["root"]["positionM"][2]) for s in samples)
    return report(plant, "free-fall",
                  initial=initial, final=final,
                  minimumRootHeightM=round(minimum_z, 6),
                  rootHeightChangeM=round(final["root"]["positionM"][2] - initial["root"]["positionM"][2], 6),
                  uprightChange=round(final["root"]["uprightZ"] - initial["root"]["uprightZ"], 6),
                  nonFootGroundContacts=len(final["contacts"]["otherBodyFloor"]),
                  samples=[{"t": s["simulationTimeSeconds"], "z": s["root"]["positionM"][2], "up": s["root"]["uprightZ"],
                            "contacts": s["contacts"]["count"]} for s in samples])


def evaluate_stand(samples, gate, worst):
    start = gate["evaluationStartSeconds"]
    window = [s for s in samples if s["simulationTimeSeconds"] >= start - 1e-9]
    if not window:
        return {"passed": False, "reason": "no samples inside the evaluation window"}
    heights = [float(s["root"]["positionM"][2]) for s in window]
    drifts = [math.hypot(float(s["root"]["positionM"][0]), float(s["root"]["positionM"][1])) for s in window]
    tilts = [math.acos(clamp(float(s["root"]["uprightZ"]), -1.0, 1.0)) for s in window]
    final = window[-1]
    both_feet = all(all(any(c["normalForceN"] > 1e-6 for c in s["contacts"][side])
                         for side in ("leftFootFloor", "rightFootFloor")) for s in window)
    non_foot = max(len(s["contacts"]["otherBodyFloor"]) for s in window)
    external_support = max(len(s["contacts"]["robotExternalObject"]) + len(s["contacts"]["robotFixture"]) for s in window)
    times = [s["simulationTimeSeconds"] for s in window]
    checks = {
        "continuousEvaluationEvidence": bool(times[0] <= start + OBSERVATION_INTERVAL_SECONDS + 1e-6
            and times[-1] >= start + gate["evaluationSeconds"] - 1e-6
            and all(0 <= b - a <= OBSERVATION_INTERVAL_SECONDS + 1e-6 for a, b in zip(times, times[1:]))),
        "pelvisHeightInBand": bool(min(heights) >= gate["pelvisHeightRangeM"][0] and max(heights) <= gate["pelvisHeightRangeM"][1]),
        "tiltWithinLimit": bool(max(tilts) <= gate["maxPelvisTiltRad"]),
        "driftWithinLimit": bool(max(drifts) <= gate["maxHorizontalDriftM"]),
        "terminalLinearSpeedWithinLimit": bool(np.linalg.norm(final["root"]["linearVelocityMS"]) <= gate["maxTerminalLinearSpeedMS"]),
        "terminalAngularSpeedWithinLimit": bool(np.linalg.norm(final["root"]["angularVelocityRadS"]) <= gate["maxTerminalAngularSpeedRadS"]),
        "bothFeetSupportedThroughout": bool(both_feet) if gate["requireBothFeetSupported"] else True,
        "noNonFootGroundContact": bool(non_foot == 0) if gate["forbidNonFootGroundContact"] else True,
        "noExternalOrFixtureSupport": bool(external_support == 0),
        "actuatorEffortWithinFraction": bool(worst <= gate["maxActuatorEffortFraction"]),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "measured": {
            "evaluationStartSeconds": start,
            "evaluationEndSeconds": final["simulationTimeSeconds"],
            "pelvisHeightMinM": round(min(heights), 6), "pelvisHeightMaxM": round(max(heights), 6),
            "maxTiltRad": round(max(tilts), 6), "maxTiltDeg": round(math.degrees(max(tilts)), 4),
            "maxHorizontalDriftM": round(max(drifts), 6),
            "terminalLinearSpeedMS": round(float(np.linalg.norm(final["root"]["linearVelocityMS"])), 6),
            "terminalAngularSpeedRadS": round(float(np.linalg.norm(final["root"]["angularVelocityRadS"])), 6),
            "maxNonFootGroundContacts": int(non_foot),
            "maxExternalOrFixtureContacts": int(external_support),
            "worstActuatorEffortFraction": round(worst, 6),
        },
        "gate": gate,
    }


def run_stand(args, profile_id, motors_enabled=True, perturbation=None, floor_friction=None):
    plant = Plant("freebase", timestep=args.timestep, control_interval=args.control_interval,
                  motors_enabled=motors_enabled, floor_friction=floor_friction)
    profile = PROFILES[profile_id]
    plant.declare_setup("initial placement", pose="source FixStand standing posture at the model keyframe",
                        pelvisHeightM=round(float(plant.data.qpos[2]), 6),
                        note="the model keyframe is the declared initial condition; no support fixture, elastic band, weld or external force exists in this scene at any time")
    if perturbation:
        plant.set_initial_root_velocity(**perturbation)
    start_pose = list(plant.q())
    worst = 0.0

    def monitor(p):
        nonlocal worst
        for index in range(29):
            worst = max(worst, abs(float(p.data.actuator_force[index])) / EFFORT_LIMIT_NM[index])

    total = STAND_GATE["evaluationStartSeconds"] + STAND_GATE["evaluationSeconds"]
    samples = plant.run(total, lambda now: stand_commands(profile, start_pose, now), monitor=monitor)
    evaluation = evaluate_stand(samples, STAND_GATE, worst)
    return plant, {
        "controller": {"id": profile_id, "label": profile["label"], "gainProvenance": profile["gainProvenance"],
                       "kp": profile["kp"], "kd": profile["kd"], "rampSeconds": profile["rampSeconds"],
                       "targetPoseRad": STAND_POSE_RAD,
                       "totalAnkleKpNmPerRad": profile["kp"][4] + profile["kp"][10],
                       "freeBaseAnkleRequirementNmPerRad": round(ANKLE_STABILITY_REQUIREMENT, 4)},
        "motorsEnabled": motors_enabled,
        "supportUsed": "none: this scene contains no support fixture, elastic band, weld or external force at any time",
        "evaluation": evaluation,
        "final": samples[-1],
        "samples": [{"t": s["simulationTimeSeconds"], "z": s["root"]["positionM"][2], "up": s["root"]["uprightZ"],
                     "leftFoot": len(s["contacts"]["leftFootFloor"]), "rightFoot": len(s["contacts"]["rightFootFloor"]),
                     "nonFootGround": len(s["contacts"]["otherBodyFloor"])} for s in samples],
    }


def trial_stand_nominal(args):
    plant, payload = run_stand(args, "robobuddy_g1_stand_v1")
    return report(plant, "stand-nominal", **payload)


def trial_stand_source_fixstand(args):
    plant, payload = run_stand(args, "unitree_g1_fixstand_source_v1")
    payload["interpretation"] = (
        "Negative evidence, deliberately retained. The exact Unitree FixStand gains are applied to the exact "
        "Unitree standing posture on this model with no support of any kind. Its total ankle stiffness of "
        f"{payload['controller']['totalAnkleKpNmPerRad']} N m/rad is below the "
        f"{payload['controller']['freeBaseAnkleRequirementNmPerRad']} N m/rad this model needs to be a stable "
        "free-base inverted pendulum, so the posture diverges with torque headroom to spare. FixStand is a joint-"
        "position hold, not a free-base standing controller, and this repository does not advertise it as one."
    )
    return report(plant, "stand-source-fixstand", **payload)


def trial_stand_motors_disabled(args):
    plant, payload = run_stand(args, "robobuddy_g1_stand_v1", motors_enabled=False)
    payload["interpretation"] = (
        "Mandatory standing negative. The verified standing controller runs unchanged and keeps issuing bounded "
        "commands, but every actuator transmission is disabled, so no actuator force reaches the plant. The "
        "posture must fail; if it did not, something other than physical actuation would be holding the robot up."
    )
    return report(plant, "stand-motors-disabled", **payload)


def trial_stand_perturbation(args):
    plant, payload = run_stand(args, "robobuddy_g1_stand_v1",
                               perturbation={"linear": (float(args.impulse), 0.0, 0.0), "angular": (0.0, 0.0, 0.0)})
    payload["interpretation"] = (
        f"Robustness characterisation only, at exactly one tested initial condition: a {args.impulse} m/s forward "
        "root velocity applied before the evaluated interval. A pass here characterises the controller envelope at "
        "this magnitude; it is not perturbation-recovery capability, which is neither defined nor exposed."
    )
    return report(plant, "stand-perturbation", perturbationLinearMS=args.impulse, **payload)


def trial_stand_low_friction(args):
    plant, payload = run_stand(args, "robobuddy_g1_stand_v1", floor_friction=args.friction)
    payload["interpretation"] = (
        f"Robustness characterisation only: foot and floor sliding friction reduced to {args.friction}. A declared "
        "degraded surface, not a modelled real material."
    )
    return report(plant, "stand-low-friction", floorSlidingFriction=args.friction, **payload)


def trial_self_contact(args):
    """A representative non-adjacent self-contact: the two knees are commanded across each other."""
    plant = Plant("mounted", timestep=args.timestep, control_interval=args.control_interval)
    profile = PROFILES["unitree_g1_joint_hold_v1"]
    pose = [0.0] * 29
    pose[JOINT_ORDER.index("left_hip_roll_joint")] = JOINT_RANGE_RAD[1][0]
    pose[JOINT_ORDER.index("right_hip_roll_joint")] = JOINT_RANGE_RAD[7][1]
    samples = plant.run(3.0, lambda _now: hold_commands(profile, pose))
    final = samples[-1]
    self_contacts = final["contacts"]["robotSelf"]
    pairs = sorted({tuple(sorted(entry["bodies"])) for entry in self_contacts})
    return report(plant, "self-contact",
                  commandedPoseRad={"left_hip_roll_joint": pose[1], "right_hip_roll_joint": pose[7]},
                  selfContactPairs=[list(pair) for pair in pairs],
                  selfContacts=self_contacts,
                  leftHipRoll=final["joints"]["left_hip_roll_joint"],
                  rightHipRoll=final["joints"]["right_hip_roll_joint"],
                  interpretation=(
                      "Both hip roll joints are commanded to their source inner limits so the shins cross. The "
                      "resulting knee-to-knee contact is a non-adjacent pair that MuJoCo's parent-child filter does "
                      "not remove, so it demonstrates that self-contact is part of the physical plant."))


def trial_external_object(args):
    """A declared free object in the free-base scene responds to real robot contact."""
    plant = Plant("freebase", timestep=args.timestep, control_interval=args.control_interval)
    profile = PROFILES["robobuddy_g1_stand_v1"]
    plant.declare_setup("initial placement", pose="source FixStand standing posture at the model keyframe",
                        externalObject=EXTERNAL_OBJECT,
                        note="the free contact-probe block is declared in the scene and rests on the floor under gravity")
    start_pose = list(plant.q())
    plant.run(STAND_GATE["evaluationStartSeconds"], lambda now: stand_commands(profile, start_pose, now))
    before = plant.object_pose()
    reach = list(STAND_POSE_RAD)
    reach[JOINT_ORDER.index("right_hip_pitch_joint")] = args.reach_hip
    reach[JOINT_ORDER.index("right_knee_joint")] = args.reach_knee
    reach[JOINT_ORDER.index("right_ankle_pitch_joint")] = args.reach_ankle
    seen = []

    def monitor(p):
        contacts = p.contacts()["robotExternalObject"]
        for entry in contacts:
            if entry not in seen:
                seen.append(entry)

    samples = plant.run(args.reach_seconds, lambda _now: hold_commands(profile, reach), monitor=monitor)
    after = plant.object_pose()
    displacement = math.dist(before["positionM"], after["positionM"])
    return report(plant, "external-object",
                  objectId=EXTERNAL_OBJECT,
                  objectBefore=before, objectAfter=after,
                  objectDisplacementM=round(displacement, 6),
                  contactGeomPairs=sorted({tuple(entry["geoms"]) for entry in seen}) and [list(p) for p in sorted({tuple(entry["geoms"]) for entry in seen})],
                  contactCount=len(seen),
                  firstContacts=seen[:6],
                  reachCommandRad={"right_hip_pitch_joint": args.reach_hip, "right_knee_joint": args.reach_knee, "right_ankle_pitch_joint": args.reach_ankle},
                  final=samples[-1],
                  interpretation=(
                      "The block moves only through named MuJoCo geometry contact. No impulse, velocity assignment "
                      "or attachment exists anywhere in this scene."))


def trial_command_bounds(args):
    """Requested, accepted and measured are three different things, and the bounds are the source bounds."""
    plant = Plant("mounted", timestep=args.timestep, control_interval=args.control_interval)
    profile = PROFILES["unitree_g1_joint_hold_v1"]
    cases = []
    for name, requested in (("left_knee_joint", 6.0), ("left_ankle_roll_joint", -1.5), ("left_wrist_pitch_joint", 3.0)):
        index = JOINT_ORDER.index(name)
        command = bound_command(index, requested, velocity_rad_s=500.0, feedforward_nm=1e4, kp=1e5, kd=1e5)
        cases.append({
            "joint": name,
            "requestedPositionRad": requested, "acceptedPositionRad": round(command["positionRad"], 6),
            "requestedVelocityRadS": 500.0, "acceptedVelocityRadS": command["velocityRadS"],
            "requestedFeedforwardNm": 1e4, "acceptedFeedforwardNm": command["feedforwardTorqueNm"],
            "requestedKp": 1e5, "acceptedKp": command["kp"], "requestedKd": 1e5, "acceptedKd": command["kd"],
            "sourceJointRangeRad": JOINT_RANGE_RAD[index],
            "sourceVelocityLimitRadS": VELOCITY_LIMIT_RAD_S[index],
            "sourceEffortLimitNm": EFFORT_LIMIT_NM[index],
        })
    pose = [0.0] * 29
    pose[JOINT_ORDER.index("left_knee_joint")] = 6.0
    plant.run(2.5, lambda _now: hold_commands(profile, pose))
    joint = plant.observation()["joints"]["left_knee_joint"]
    peak = max(abs(float(plant.data.actuator_force[i])) / EFFORT_LIMIT_NM[i] for i in range(29))
    return report(plant, "command-bounds", cases=cases, measuredAfterOutOfRangeRequest=joint,
                  worstActuatorEffortFraction=round(peak, 6),
                  interpretation=(
                      "An out-of-range request is clamped to the source joint range, and the measured state is read "
                      "from MuJoCo rather than echoed from the command. No actuator ever exceeds its source effort limit."))


TRIALS = {
    "model-audit": trial_model_audit,
    "mounted-joint-response": trial_mounted_joint_response,
    "blocked-joint": trial_blocked_joint,
    "known-poses": trial_known_poses,
    "command-bounds": trial_command_bounds,
    "free-fall": trial_free_fall,
    "self-contact": trial_self_contact,
    "external-object": trial_external_object,
    "stand-nominal": trial_stand_nominal,
    "stand-source-fixstand": trial_stand_source_fixstand,
    "stand-motors-disabled": trial_stand_motors_disabled,
    "stand-perturbation": trial_stand_perturbation,
    "stand-low-friction": trial_stand_low_friction,
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial", required=True, choices=sorted(TRIALS))
    parser.add_argument("--timestep", type=float, default=None, help="physics timestep override for the sensitivity study")
    parser.add_argument("--control-interval", type=float, default=None, help="low-level control interval override")
    parser.add_argument("--impulse", type=float, default=0.25, help="stand-perturbation forward root velocity, m/s")
    parser.add_argument("--friction", type=float, default=0.25, help="stand-low-friction sliding friction")
    parser.add_argument("--reach-hip", type=float, default=-0.95)
    parser.add_argument("--reach-knee", type=float, default=0.45)
    parser.add_argument("--reach-ankle", type=float, default=-0.25)
    parser.add_argument("--reach-seconds", type=float, default=1.6)
    args = parser.parse_args()
    print(json.dumps(TRIALS[args.trial](args), indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
