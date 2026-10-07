#!/usr/bin/env python3
"""Native MuJoCo reference for the synthetic SO-101 contact-transfer benchmark.

Only setup writes qpos. Ordinary task motion is produced by bounded actuator targets + mj_step.
Low-grip/heavy-payload negative controls modify model parameters before the run as explicit test profiles.
"""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "so101" / "manipulation.xml"
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
INITIAL = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -0.18,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": math.pi / 2,
    "gripper": 0.60,
}
TARGET_CENTER = np.array([0.358, -0.156], dtype=float)
TARGET_HALF = np.array([0.020, 0.030], dtype=float)
SUPPORT_Z = 0.227
BLOCK_HALF_Z = 0.007
APPROACH_PAN_RAD = 0.005
LOW_GRIP_FORCE_NM = 0.0001


def ids(model):
    joints = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINTS}
    acts = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in JOINTS}
    block_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "benchmark_block")
    block_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "benchmark_block_geom")
    free_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "benchmark_block_free")
    fixed_tip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "fixed_jaw_sph_tip1")
    moving_tip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "moving_jaw_sph_tip1")
    camera_box2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "camera_box2")
    assert min(*joints.values(), *acts.values(), block_body, block_geom, free_joint, fixed_tip, moving_tip, camera_box2) >= 0
    return joints, acts, block_body, block_geom, free_joint, fixed_tip, moving_tip, camera_box2


def geom_name(model, geom_id):
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id)) or f"geom:{geom_id}"


def all_contact_pairs(model, data):
    pairs = set()
    for i in range(data.ncon):
        c = data.contact[i]
        a = geom_name(model, c.geom1)
        b = geom_name(model, c.geom2)
        pairs.add(tuple(sorted((a, b))))
    return [list(pair) for pair in sorted(pairs)]


def apply_test_profile(model, trial, block_body, gripper_act):
    profile = {
        "trial": trial,
        "lowGripForceNm": None,
        "lowGripForceRangeNm": None,
        "lowGripForceLimited": None,
        "payloadMassKg": float(model.body_mass[block_body]),
    }
    if trial == "low-grip":
        # actuator_forcerange is shaped (nu, 2): index the gripper actuator row,
        # not a flattened pair. Explicitly enable force limiting for this negative-only profile.
        model.actuator_forcerange[gripper_act, :] = (-LOW_GRIP_FORCE_NM, LOW_GRIP_FORCE_NM)
        model.actuator_forcelimited[gripper_act] = 1
        actual_range = np.array(model.actuator_forcerange[gripper_act, :], dtype=float)
        assert np.allclose(actual_range, [-LOW_GRIP_FORCE_NM, LOW_GRIP_FORCE_NM], atol=0.0, rtol=0.0)
        assert int(model.actuator_forcelimited[gripper_act]) == 1
        profile["lowGripForceNm"] = LOW_GRIP_FORCE_NM
        profile["lowGripForceRangeNm"] = actual_range.tolist()
        profile["lowGripForceLimited"] = True
    if trial == "heavy":
        factor = 37.5
        model.body_mass[block_body] *= factor
        model.body_inertia[block_body, :] *= factor
        profile["payloadMassKg"] = float(model.body_mass[block_body])
    return profile


def setup(model, data, joints, acts):
    mujoco.mj_resetData(model, data)
    for name, value in INITIAL.items():
        jid = joints[name]
        qadr = int(model.jnt_qposadr[jid])
        data.qpos[qadr] = value
        data.ctrl[acts[name]] = value
    mujoco.mj_forward(model, data)


def contact_names(model, data, block_geom):
    names = set()
    for i in range(data.ncon):
        c = data.contact[i]
        if int(c.geom1) == block_geom:
            names.add(geom_name(model, c.geom2))
        elif int(c.geom2) == block_geom:
            names.add(geom_name(model, c.geom1))
    return sorted(names)


def is_gripper_contact(names):
    return any(name.startswith("fixed_jaw_") or name.startswith("moving_jaw_") for name in names)


def block_state(model, data, block_body, free_joint, block_geom):
    qv = int(model.jnt_dofadr[free_joint])
    pos = np.array(data.xpos[block_body], dtype=float)
    vel = np.array(data.qvel[qv : qv + 6], dtype=float)
    contacts = contact_names(model, data, block_geom)
    return {
        "time": float(data.time),
        "positionM": pos.tolist(),
        "speedNorm": float(np.linalg.norm(vel)),
        "contacts": contacts,
        "gripperContact": bool(is_gripper_contact(contacts)),
    }


def joint_positions(model, data, joints):
    return {name: float(data.qpos[int(model.jnt_qposadr[jid])]) for name, jid in joints.items()}


def run_trial(trial: str, *, timestep=None, iterations=None):
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    if timestep is not None:
        model.opt.timestep = float(timestep)
    if iterations is not None:
        model.opt.iterations = int(iterations)
    data = mujoco.MjData(model)
    joints, acts, block_body, block_geom, free_joint, fixed_tip, moving_tip, camera_box2 = ids(model)
    profile = apply_test_profile(model, trial, block_body, acts["gripper"])
    setup(model, data, joints, acts)

    samples = []
    stages = []
    max_z = float(data.xpos[block_body][2])
    gripper_contact_samples = 0
    carried_contact_samples = 0

    def stage(name, targets, seconds, sample_stride=4):
        nonlocal max_z, gripper_contact_samples, carried_contact_samples
        for joint, value in targets.items():
            data.ctrl[acts[joint]] = float(value)
        steps = int(round(float(seconds) / float(model.opt.timestep)))
        assert abs(steps * float(model.opt.timestep) - float(seconds)) < 1e-9
        start = block_state(model, data, block_body, free_joint, block_geom)
        for step in range(steps):
            mujoco.mj_step(model, data)
            if step % sample_stride == 0 or step == steps - 1:
                state = block_state(model, data, block_body, free_joint, block_geom)
                max_z = max(max_z, state["positionM"][2])
                if state["gripperContact"]:
                    gripper_contact_samples += 1
                    if state["positionM"][2] > SUPPORT_Z + BLOCK_HALF_Z + 0.020:
                        carried_contact_samples += 1
                samples.append({"stage": name, **state})
        end = block_state(model, data, block_body, free_joint, block_geom)
        stages.append({
            "name": name,
            "targetsRad": dict(targets),
            "seconds": float(seconds),
            "start": start,
            "end": end,
            "actualJointsRad": joint_positions(model, data, joints),
            "fixedTipPositionM": np.array(data.geom_xpos[fixed_tip], dtype=float).tolist(),
            "movingTipPositionM": np.array(data.geom_xpos[moving_tip], dtype=float).tolist(),
            "cameraBox2PositionM": np.array(data.geom_xpos[camera_box2], dtype=float).tolist(),
            "blockContacts": end["contacts"],
            "allContacts": all_contact_pairs(model, data),
        })

    stage("settle_initial", {}, 0.20)
    if trial == "miss":
        stage("approach_misaligned", {"shoulder_pan": 0.20, "shoulder_lift": 0.0}, 0.60)
    else:
        stage("approach", {"shoulder_pan": APPROACH_PAN_RAD, "shoulder_lift": 0.0}, 0.60)
    stage("close", {"gripper": -0.04}, 0.50)
    stage("lift", {"shoulder_lift": -0.35}, 0.80)
    move_pan = 0.15 if trial == "outside" else 0.45
    stage("move", {"shoulder_pan": move_pan}, 0.90)
    stage("lower", {"shoulder_lift": 0.0}, 0.80)
    stage("release", {"gripper": 0.60}, 0.50)
    stage("settle_final", {}, 0.80)

    final = block_state(model, data, block_body, free_joint, block_geom)
    initial_pos = np.array(stages[0]["start"]["positionM"], dtype=float)
    final_pos = np.array(final["positionM"], dtype=float)
    lifted = bool(max_z > SUPPORT_Z + BLOCK_HALF_Z + 0.030)
    horizontal_travel = float(np.linalg.norm(final_pos[:2] - initial_pos[:2]))
    in_target = bool(np.all(np.abs(final_pos[:2] - TARGET_CENTER) <= TARGET_HALF))
    resting = bool(abs(float(final_pos[2]) - (SUPPORT_Z + BLOCK_HALF_Z)) < 0.012 and final["speedNorm"] < 0.08)
    released = bool(not final["gripperContact"])
    physically_carried = bool(lifted and carried_contact_samples >= 4 and horizontal_travel > 0.05)
    success = bool(gripper_contact_samples >= 4 and physically_carried and in_target and resting and released)
    return {
        "trial": trial,
        "profile": profile,
        "engine": {"version": mujoco.__version__, "timestepSeconds": float(model.opt.timestep), "iterations": int(model.opt.iterations), "lsIterations": int(model.opt.ls_iterations)},
        "controller": {"type": "bounded position-target stage controller", "ordinaryControlWrites": "data.ctrl only", "initialJointPositionsRad": INITIAL, "approachPanRad": APPROACH_PAN_RAD},
        "benchmark": {"blockMassKg": profile["payloadMassKg"], "blockHalfExtentsM": [0.008, 0.006, 0.007], "surfaceFriction": 0.8, "targetCenterXYM": TARGET_CENTER.tolist(), "targetHalfExtentsXYM": TARGET_HALF.tolist()},
        "metrics": {"success": success, "lifted": lifted, "physicallyCarried": physically_carried, "inTarget": in_target, "resting": resting, "released": released, "maxBlockZM": float(max_z), "horizontalTravelM": horizontal_travel, "gripperContactSamples": int(gripper_contact_samples), "carriedContactSamples": int(carried_contact_samples), "finalPositionM": final["positionM"], "finalSpeedNorm": final["speedNorm"], "stageDiagnostics": stages},
        "stages": stages,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trial", choices=("positive", "low-grip", "heavy", "miss", "outside"), default="positive")
    parser.add_argument("--timestep", type=float)
    parser.add_argument("--iterations", type=int)
    args = parser.parse_args()
    print(json.dumps(run_trial(args.trial, timestep=args.timestep, iterations=args.iterations), indent=2, sort_keys=True))

if __name__ == "__main__":
    main()
