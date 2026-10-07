#!/usr/bin/env python3
"""Assert that the native Unitree G1 reference and the browser modules share identical tables.

The native runner produces the numerical evidence and the browser worker produces what learners
and agents actually run. If their joint order, limits, standing posture, gains, timing or command
law ever drifted apart, the native evidence would stop describing the shipped simulator. This test
reads both sides and compares them value for value, so that drift fails CI instead of passing
quietly.
"""
from __future__ import annotations

import ast
import importlib.util
import json
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
AUDIT_JS = REPO / "src" / "physics" / "unitree-g1-source-audit.js"
CONTROLLER_JS = REPO / "src" / "physics" / "unitree-g1-controller.js"


def js_array(text: str, name: str):
    """Read `export const NAME = Object.freeze([ ... ])` as JSON, matching nested brackets."""
    anchor = text.find(f"export const {name} = Object.freeze([")
    if anchor < 0:
        raise SystemExit(f"{name} not found in the JavaScript source")
    start = text.index("[", anchor)
    depth = 0
    for index in range(start, len(text)):
        if text[index] == "[":
            depth += 1
        elif text[index] == "]":
            depth -= 1
            if depth == 0:
                body = text[start:index + 1]
                break
    else:
        raise SystemExit(f"{name} has an unbalanced array literal")
    body = re.sub(r"//[^\n]*", "", body).replace("'", '"')
    body = re.sub(r",(\s*[\]}])", r"\1", body)
    return json.loads(body)


def js_number(text: str, name: str) -> float:
    match = re.search(rf"export const {name} = ([-\d.eE+]+);", text)
    if not match:
        raise SystemExit(f"{name} not found in the JavaScript source")
    return float(match.group(1))


def load_native():
    spec = importlib.util.spec_from_file_location("g1_reference", REPO / "native" / "unitree_g1_reference.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    audit = AUDIT_JS.read_text(encoding="utf-8")
    controller = CONTROLLER_JS.read_text(encoding="utf-8")
    native = load_native()
    failures = []

    def check(label, js_value, py_value):
        if js_value != py_value:
            failures.append(f"{label}: javascript {js_value!r} != python {py_value!r}")

    check("G1_JOINT_ORDER", js_array(audit, "G1_JOINT_ORDER"), native.JOINT_ORDER)
    check("G1_JOINT_RANGE_RAD", js_array(audit, "G1_JOINT_RANGE_RAD"), native.JOINT_RANGE_RAD)
    check("G1_EFFORT_LIMIT_NM", js_array(audit, "G1_EFFORT_LIMIT_NM"), native.EFFORT_LIMIT_NM)
    check("G1_VELOCITY_LIMIT_RAD_S", js_array(audit, "G1_VELOCITY_LIMIT_RAD_S"), native.VELOCITY_LIMIT_RAD_S)
    check("G1_STAND_POSE_RAD", js_array(controller, "G1_STAND_POSE_RAD"), native.STAND_POSE_RAD)
    check("UNITREE_FIXSTAND_KP", js_array(controller, "UNITREE_FIXSTAND_KP"), native.UNITREE_FIXSTAND_KP)
    check("UNITREE_FIXSTAND_KD", js_array(controller, "UNITREE_FIXSTAND_KD"), native.UNITREE_FIXSTAND_KD)
    check("UNITREE_FIXSTAND_RAMP_SECONDS", js_number(controller, "UNITREE_FIXSTAND_RAMP_SECONDS"), native.UNITREE_FIXSTAND_RAMP_SECONDS)
    check("G1_ROBOBUDDY_ANKLE_KP", js_number(controller, "G1_ROBOBUDDY_ANKLE_KP"), native.ROBOBUDDY_ANKLE_KP)
    check("G1_ROBOBUDDY_ANKLE_KD", js_number(controller, "G1_ROBOBUDDY_ANKLE_KD"), native.ROBOBUDDY_ANKLE_KD)
    check("G1_MAX_KP", js_number(controller, "G1_MAX_KP"), native.MAX_KP)
    check("G1_MAX_KD", js_number(controller, "G1_MAX_KD"), native.MAX_KD)
    check("G1_PHYSICS_TIMESTEP_SECONDS", js_number(controller, "G1_PHYSICS_TIMESTEP_SECONDS"), native.PHYSICS_TIMESTEP_SECONDS)
    check("G1_LOWLEVEL_CONTROL_INTERVAL_SECONDS", js_number(controller, "G1_LOWLEVEL_CONTROL_INTERVAL_SECONDS"), native.LOWLEVEL_CONTROL_INTERVAL_SECONDS)
    check("G1_OBSERVATION_INTERVAL_SECONDS", js_number(controller, "G1_OBSERVATION_INTERVAL_SECONDS"), native.OBSERVATION_INTERVAL_SECONDS)
    check("G1_UNITREE_FSM_INTERVAL_SECONDS", js_number(controller, "G1_UNITREE_FSM_INTERVAL_SECONDS"), native.UNITREE_FSM_INTERVAL_SECONDS)
    check("G1_TOTAL_MASS_KG", js_number(audit, "G1_TOTAL_MASS_KG"), native.TOTAL_MASS_KG)
    check("G1_STAND_COM_HEIGHT_ABOVE_ANKLE_M", js_number(audit, "G1_STAND_COM_HEIGHT_ABOVE_ANKLE_M"), native.STAND_COM_HEIGHT_ABOVE_ANKLE_M)

    # The two standing profiles must be built the same way on both sides.
    js_ankles = ast.literal_eval(re.search(r"const ANKLE_INDICES = Object\.freeze\((\[[^\]]*\])\)", controller).group(1))
    check("ankle indices", js_ankles, native.ANKLE_INDICES)
    expected_kp = list(native.UNITREE_FIXSTAND_KP)
    expected_kd = list(native.UNITREE_FIXSTAND_KD)
    for index in native.ANKLE_INDICES:
        expected_kp[index] = native.ROBOBUDDY_ANKLE_KP
        expected_kd[index] = native.ROBOBUDDY_ANKLE_KD
    check("robobuddy stand kp", expected_kp, native.PROFILES["robobuddy_g1_stand_v1"]["kp"])
    check("robobuddy stand kd", expected_kd, native.PROFILES["robobuddy_g1_stand_v1"]["kd"])
    check("source fixstand kp", list(native.UNITREE_FIXSTAND_KP), native.PROFILES["unitree_g1_fixstand_source_v1"]["kp"])
    check("joint hold kp", list(native.UNITREE_FIXSTAND_KP), native.PROFILES["unitree_g1_joint_hold_v1"]["kp"])

    # The command law itself must agree, evaluated over a spread of states.
    for index, (position, velocity, target, dq_target, tau_ff, kp, kd) in enumerate([
        (0.0, 0.0, 0.5, 0.0, 0.0, 100.0, 2.0),
        (0.4, -1.2, -0.3, 0.5, 3.0, 40.0, 10.0),
        (-2.0, 30.0, 2.5, -20.0, -5.0, 250.0, 10.0),
        (0.1, 0.0, 6.0, 500.0, 1e4, 1e5, 1e5),
    ]):
        command = native.bound_command(3, target, dq_target, tau_ff, kp, kd)
        torque = native.low_level_torque(command, position, velocity)
        low, high = native.JOINT_RANGE_RAD[3]
        expected_target = min(high, max(low, target))
        expected_dq = min(native.VELOCITY_LIMIT_RAD_S[3], max(-native.VELOCITY_LIMIT_RAD_S[3], dq_target))
        expected_tau = min(native.EFFORT_LIMIT_NM[3], max(-native.EFFORT_LIMIT_NM[3], tau_ff))
        expected_kp = min(native.MAX_KP, max(0.0, kp))
        expected_kd = min(native.MAX_KD, max(0.0, kd))
        raw = expected_tau + expected_kp * (expected_target - position) + expected_kd * (expected_dq - velocity)
        expected = min(native.EFFORT_LIMIT_NM[3], max(-native.EFFORT_LIMIT_NM[3], raw))
        if abs(torque - expected) > 1e-12:
            failures.append(f"low-level law case {index}: {torque} != {expected}")

    # The JavaScript law must be the same expression, read from the source rather than assumed.
    if "command.feedforwardTorqueNm + command.kp * (command.positionRad - q) + command.kd * (command.velocityRadS - dq)" not in controller:
        failures.append("the JavaScript low-level law is not the audited Unitree bridge expression")

    for failure in failures:
        print(failure, file=sys.stderr)
    if failures:
        return 1
    print("Unitree G1 controller parity (JavaScript vs native reference): OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
