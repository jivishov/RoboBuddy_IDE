#!/usr/bin/env python3
"""Independent MicroDuck BAM actuator conformance reference.

This file intentionally uses the installed better-actuator-models==1.0.1 package instead of
re-implementing BAM. It emits source-library outputs that the browser JavaScript plant must
match. It is software/model conformance evidence, not hardware calibration.
"""
from __future__ import annotations

import argparse
import importlib.metadata
import json
from pathlib import Path

from bam.model import load_model

EXPECTED_PACKAGE_VERSION = "1.0.1"

CASES = (
    {"name": "small-positive", "target": 0.10, "q": 0.0, "dq": 0.0, "gain": 200.0, "vin": 7.4, "external": 0.0},
    {"name": "moving-positive", "target": 0.35, "q": 0.10, "dq": 1.2, "gain": 200.0, "vin": 7.1, "external": -0.18},
    {"name": "standing-gain", "target": -0.20, "q": -0.05, "dq": -0.7, "gain": 160.0, "vin": 7.4, "external": 0.12},
    {"name": "saturated-positive", "target": 4.0, "q": 0.0, "dq": 0.4, "gain": 200.0, "vin": 8.2, "external": -0.35},
    {"name": "saturated-negative", "target": -4.0, "q": 0.0, "dq": -0.4, "gain": 200.0, "vin": 6.5, "external": 0.35},
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", type=Path, required=True)
    args = parser.parse_args()

    version = importlib.metadata.version("better-actuator-models")
    if version != EXPECTED_PACKAGE_VERSION:
        raise SystemExit(f"better-actuator-models {version} loaded; expected {EXPECTED_PACKAGE_VERSION}")

    model = load_model(motor_name="xl330", model="m6")
    act = model.actuator

    params = {
        "ktNmPerA": model.kt.value,
        "resistanceOhm": model.R.value,
        "armatureKgM2": model.armature.value,
        "qOffsetRad": model.q_offset.value,
        "frictionBaseNm": model.friction_base.value,
        "frictionStribeckNm": model.friction_stribeck.value,
        "loadFrictionMotor": model.load_friction_motor.value,
        "loadFrictionExternal": model.load_friction_external.value,
        "loadFrictionMotorStribeck": model.load_friction_motor_stribeck.value,
        "loadFrictionExternalStribeck": model.load_friction_external_stribeck.value,
        "loadFrictionMotorQuad": model.load_friction_motor_quad.value,
        "loadFrictionExternalQuad": model.load_friction_external_quad.value,
        "dthetaStribeckRadS": model.dtheta_stribeck.value,
        "stribeckAlpha": model.alpha.value,
        "frictionViscousNmPerRadS": model.friction_viscous.value,
        "errorGain": act.error_gain,
        "maxPwm": act.max_pwm,
    }

    samples = []
    for case in CASES:
        act.kp = case["gain"]
        act.vin = case["vin"]
        control_v = float(act.compute_control(case["target"], case["q"], case["dq"], 0.005))
        motor_torque = float(act.compute_torque(control_v, True, case["q"], case["dq"]))
        frictionloss, damping = model.compute_frictions(motor_torque, case["external"], case["dq"])
        samples.append({
            **case,
            "controlV": control_v,
            "motorTorqueNm": float(motor_torque),
            "frictionLossNm": float(frictionloss),
            "dampingNmPerRadS": float(damping),
        })

    report = {
        "package": "better-actuator-models",
        "version": version,
        "motor": "xl330",
        "model": "m6",
        "params": params,
        "trainingForceCeilingNm": float(8.2 * model.kt.value / model.R.value),
        "samples": samples,
    }
    args.json.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(f"wrote {args.json} with {len(samples)} BAM source-library samples")


if __name__ == "__main__":
    main()
