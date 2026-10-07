from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import mujoco

EXPECTED_MUJOCO_VERSION = "3.11.0"
ROOT = Path(__file__).resolve().parents[1]
INTEGRATOR_CODES = {
    "Euler": int(mujoco.mjtIntegrator.mjINT_EULER),
    "RK4": int(mujoco.mjtIntegrator.mjINT_RK4),
    "implicit": int(mujoco.mjtIntegrator.mjINT_IMPLICIT),
    "implicitfast": int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST),
}

MODEL_PACKAGES: dict[str, dict[str, object]] = {
    "phase1": {
        "model_id": "phase1-vertical-slice-v1",
        "asset": "models/vertical-slice/model.xml",
        "path": ROOT / "models" / "vertical-slice" / "model.xml",
        "timestep": 0.002,
        "integrator": "RK4",
        "iterations": None,
        "ls_iterations": None,
        "joints": ["hinge"],
        "actuators": {"hinge": "hinge_position"},
        "bodies": ["free_box"],
        "default_joint": "hinge",
    },
    "so101": {
        "model_id": "robobuddy-so101-phase2a-v1",
        "asset": "models/so101/model.xml",
        "path": ROOT / "models" / "so101" / "model.xml",
        "timestep": 0.005,
        "integrator": "implicitfast",
        "iterations": 10,
        "ls_iterations": 20,
        "joints": ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"],
        "actuators": {
            "shoulder_pan": "shoulder_pan",
            "shoulder_lift": "shoulder_lift",
            "elbow_flex": "elbow_flex",
            "wrist_flex": "wrist_flex",
            "wrist_roll": "wrist_roll",
            "gripper": "gripper",
        },
        "bodies": ["base", "gripper", "moving_jaw_so101_v1"],
        "default_joint": "shoulder_pan",
    },
}


def _name_id(model: mujoco.MjModel, obj_type: mujoco.mjtObj, name: str) -> int:
    index = int(mujoco.mj_name2id(model, obj_type, name))
    if index < 0:
        raise RuntimeError(f"Required MuJoCo object is missing: {obj_type.name}:{name}")
    return index


def _geom_name(model: mujoco.MjModel, geom_id: int) -> str | None:
    if geom_id < 0:
        return None
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)


def _contacts(model: mujoco.MjModel, data: mujoco.MjData) -> list[dict[str, object]]:
    result: list[dict[str, object]] = []
    for contact in data.contact:
        if hasattr(contact, "geom"):
            geom1, geom2 = int(contact.geom[0]), int(contact.geom[1])
        else:
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
        result.append(
            {
                "geom1": geom1,
                "geom2": geom2,
                "geom1Name": _geom_name(model, geom1),
                "geom2Name": _geom_name(model, geom2),
                "distanceM": float(contact.dist),
            }
        )
    return result


def _finite_state(data: mujoco.MjData) -> bool:
    return all(math.isfinite(float(value)) for value in data.qpos) and all(
        math.isfinite(float(value)) for value in data.qvel
    )


def _joint_snapshot(data: mujoco.MjData, joint_meta: dict[str, dict[str, object]]) -> dict[str, dict[str, float]]:
    return {
        name: {
            "positionRad": float(data.qpos[int(meta["qpos_addr"])]),
            "velocityRadS": float(data.qvel[int(meta["dof_addr"])]),
        }
        for name, meta in joint_meta.items()
    }


def run(
    target_rad: float,
    steps: int,
    allow_version_mismatch: bool = False,
    model_key: str = "phase1",
    joint_name: str | None = None,
    targets: dict[str, float] | None = None,
) -> dict[str, object]:
    if model_key not in MODEL_PACKAGES:
        raise ValueError(f"Unknown model package key: {model_key}")
    package = MODEL_PACKAGES[model_key]
    model_path = package["path"]
    if not isinstance(model_path, Path) or not model_path.is_file():
        raise FileNotFoundError(model_path)
    if not allow_version_mismatch and mujoco.__version__ != EXPECTED_MUJOCO_VERSION:
        raise RuntimeError(
            f"Native reference requires MuJoCo {EXPECTED_MUJOCO_VERSION}, got {mujoco.__version__}. "
            "Use --allow-version-mismatch only for exploratory diagnostics, not conformance evidence."
        )
    if steps < 1 or steps > 100_000:
        raise ValueError("steps must be between 1 and 100000")

    model_sha256 = hashlib.sha256(model_path.read_bytes()).hexdigest()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    expected_timestep = float(package["timestep"])
    if abs(float(model.opt.timestep) - expected_timestep) > 1e-12:
        raise RuntimeError(f"Unexpected timestep {model.opt.timestep}; expected {expected_timestep}")
    expected_integrator_name = str(package["integrator"])
    expected_integrator = INTEGRATOR_CODES[expected_integrator_name]
    if int(model.opt.integrator) != expected_integrator:
        raise RuntimeError(
            f"Unexpected integrator {int(model.opt.integrator)}; "
            f"expected {expected_integrator_name} ({expected_integrator})"
        )
    expected_iterations = package["iterations"]
    if expected_iterations is not None and int(model.opt.iterations) != int(expected_iterations):
        raise RuntimeError(
            f"Unexpected solver iterations {int(model.opt.iterations)}; expected {int(expected_iterations)}"
        )
    expected_ls_iterations = package["ls_iterations"]
    if expected_ls_iterations is not None and int(model.opt.ls_iterations) != int(expected_ls_iterations):
        raise RuntimeError(
            f"Unexpected solver line-search iterations {int(model.opt.ls_iterations)}; "
            f"expected {int(expected_ls_iterations)}"
        )

    joint_names = list(package["joints"])
    actuator_names = dict(package["actuators"])
    selected_joint = joint_name or str(package["default_joint"])
    if selected_joint not in joint_names:
        raise ValueError(f"Joint {selected_joint} is not declared for {model_key}")

    joint_meta: dict[str, dict[str, object]] = {}
    for name in joint_names:
        joint_id = _name_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        actuator_id = _name_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_names[name])
        qpos_addr = int(model.jnt_qposadr[joint_id])
        dof_addr = int(model.jnt_dofadr[joint_id])
        actuator_joint = int(model.actuator_trnid[actuator_id, 0])
        if actuator_joint != joint_id:
            raise RuntimeError(f"Actuator {actuator_names[name]} is not mapped to joint {name}")
        control_min, control_max = map(float, model.actuator_ctrlrange[actuator_id])
        joint_min, joint_max = map(float, model.jnt_range[joint_id])
        joint_meta[name] = {
            "joint_id": joint_id,
            "actuator_id": actuator_id,
            "qpos_addr": qpos_addr,
            "dof_addr": dof_addr,
            "control_range": [control_min, control_max],
            "joint_range": [joint_min, joint_max],
        }

    mujoco.mj_forward(model, data)
    initial = _joint_snapshot(data, joint_meta)

    requested_targets = dict(targets or {selected_joint: target_rad})
    for name, target in requested_targets.items():
        if name not in joint_meta:
            raise ValueError(f"Target references undeclared joint {name}")
        if not math.isfinite(float(target)):
            raise ValueError(f"Target for {name} must be finite")
        meta = joint_meta[name]
        control_min, control_max = meta["control_range"]
        joint_min, joint_max = meta["joint_range"]
        if not control_min <= target <= control_max:
            raise ValueError(f"target_rad {target} is outside actuator control range {control_min}..{control_max}")
        if not joint_min <= target <= joint_max:
            raise ValueError(f"target_rad {target} is outside joint {name} range {joint_min}..{joint_max}")
        data.ctrl[int(meta["actuator_id"])] = target

    sample_steps = sorted({max(1, steps // 4), max(1, steps // 2), max(1, (3 * steps) // 4), steps})
    trajectory: list[dict[str, object]] = []
    selected_meta = joint_meta[selected_joint]
    for step_index in range(1, steps + 1):
        mujoco.mj_step(model, data)
        if not _finite_state(data):
            raise RuntimeError("MuJoCo produced non-finite joint state")
        if step_index in sample_steps:
            trajectory.append(
                {
                    "step": step_index,
                    "simulationTimeSeconds": float(data.time),
                    "positionRad": float(data.qpos[int(selected_meta["qpos_addr"])]),
                    "velocityRadS": float(data.qvel[int(selected_meta["dof_addr"])]),
                }
            )

    contacts = _contacts(model, data)
    final_simulation_time = float(data.time)
    joints: dict[str, dict[str, object]] = {}
    for name, meta in joint_meta.items():
        actuator_id = int(meta["actuator_id"])
        joints[name] = {
            "positionRad": float(data.qpos[int(meta["qpos_addr"])]),
            "velocityRadS": float(data.qvel[int(meta["dof_addr"])]),
            "targetRad": float(data.ctrl[actuator_id]),
            "controlRangeRad": list(meta["control_range"]),
            "jointRangeRad": list(meta["joint_range"]),
        }

    bodies: dict[str, dict[str, object]] = {}
    for name in package["bodies"]:
        body_id = _name_id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        bodies[name] = {
            "frame": "mujoco_world",
            "positionM": [float(value) for value in data.xpos[body_id]],
        }

    finite_final = _finite_state(data)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    reset = _joint_snapshot(data, joint_meta)
    reset_matches_initial = all(
        abs(reset[name][key] - initial[name][key]) <= 1e-12
        for name in joint_names
        for key in ("positionRad", "velocityRadS")
    )

    return {
        "simulationTimeSeconds": final_simulation_time,
        "frames": {
            "world": {
                "id": "mujoco_world",
                "handedness": "right-handed",
                "upAxis": "+Z",
                "linearUnit": "m",
                "angularUnit": "rad",
            }
        },
        "model": {
            "id": package["model_id"],
            "asset": package["asset"],
            "sha256": model_sha256,
        },
        "engine": {
            "name": "MuJoCo",
            "version": mujoco.__version__,
            "versionEvidence": "mujoco.__version__ native package introspection",
            "timestepSeconds": float(model.opt.timestep),
            "integrator": expected_integrator_name,
            "iterations": int(model.opt.iterations),
            "lsIterations": int(model.opt.ls_iterations),
        },
        "selectedJoint": selected_joint,
        "initialJoints": initial,
        "resetJoints": reset,
        "resetMatchesInitial": reset_matches_initial,
        "trajectory": trajectory,
        "joints": joints,
        "bodies": bodies,
        "contactCount": len(contacts),
        "contactsReadable": True,
        "contacts": contacts,
        "finiteState": finite_final,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a registered RoboBuddy native MuJoCo reference plant.")
    parser.add_argument("--model", choices=sorted(MODEL_PACKAGES), default="phase1", help="Registered reference model.")
    parser.add_argument("--joint", default=None, help="Joint to actuate; defaults to the package reference joint.")
    parser.add_argument("--target", type=float, default=0.8, help="Joint position target in radians.")
    parser.add_argument("--steps", type=int, default=500, help="Number of fixed MuJoCo steps.")
    parser.add_argument(
        "--allow-version-mismatch",
        action="store_true",
        help="Allow a native MuJoCo version other than 3.11.0 for exploratory diagnostics only.",
    )
    args = parser.parse_args()
    print(
        json.dumps(
            run(
                args.target,
                args.steps,
                args.allow_version_mismatch,
                model_key=args.model,
                joint_name=args.joint,
            ),
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
