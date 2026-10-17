from __future__ import annotations
import hashlib
import json
import math
import re

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
JOINT_IDS = {name: index + 1 for index, name in enumerate(JOINTS)}
API_VERSION = "robobuddy.so101.hardware.v1"
LEROBOT_VERSION = "0.6.0"
MAX_BODY = 32768

class BridgeError(Exception):
    def __init__(self, code, message, status=409):
        super().__init__(message)
        self.code, self.status = code, status
    def payload(self):
        return {"error": {"code": self.code, "message": str(self)}}

def require(condition, code, message, status=409):
    if not condition:
        raise BridgeError(code, message, status)

def number(value, name):
    require(type(value) in (int, float) and math.isfinite(value), "INVALID_ARGUMENT", f"{name} must be a finite number.", 400)
    return float(value)

def arm_id(value):
    require(isinstance(value, str) and re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}", value), "INVALID_ARGUMENT", "Arm name must contain 1–64 letters, numbers, dots, underscores or hyphens.", 400)
    return value

def calibration_dict(data):
    require(isinstance(data, dict) and set(data) == set(JOINTS), "INVALID_CALIBRATION", "Calibration must describe exactly the six SO-101 joints.", 400)
    result = {}
    for joint in JOINTS:
        c = data[joint]
        require(isinstance(c, dict) and set(c) == {"id", "drive_mode", "homing_offset", "range_min", "range_max"}, "INVALID_CALIBRATION", f"Unexpected calibration fields for {joint}.", 400)
        require(all(type(v) is int for v in c.values()), "INVALID_CALIBRATION", f"{joint}: calibration values must be integers.", 400)
        require(c["id"] == JOINT_IDS[joint] and c["drive_mode"] in (0, 1), "INVALID_CALIBRATION", f"{joint}: wrong motor ID or drive mode.", 400)
        require(0 <= c["range_min"] < c["range_max"] <= 4095 and c["range_max"] - c["range_min"] >= 64 and -2047 <= c["homing_offset"] <= 2047, "INVALID_CALIBRATION", f"{joint}: invalid 12-bit encoder range, insufficient span or homing offset.", 400)
        result[joint] = dict(c)
    return result

def calibration_hash(data):
    return hashlib.sha256(json.dumps(data, sort_keys=True, separators=(",", ":")).encode()).hexdigest()

def units(joint):
    return "percent" if joint == "gripper" else "degrees"

def normalized(raw, joint, c):
    if joint == "gripper":
        pct = (raw - c["range_min"]) * 100 / (c["range_max"] - c["range_min"])
        return 100 - pct if c["drive_mode"] else pct
    # Matches LeRobot use_degrees=True: encoder range midpoint, 4095 steps/turn.
    return (raw - (c["range_min"] + c["range_max"]) / 2) * 360 / 4095

def raw_target(value, joint, c):
    if joint == "gripper":
        pct = 100 - value if c["drive_mode"] else value
        return round(c["range_min"] + pct * (c["range_max"] - c["range_min"]) / 100)
    return round(value * 4095 / 360 + (c["range_min"] + c["range_max"]) / 2)

def control_limits(joint, c):
    if joint == "gripper":
        return [2.0, 98.0]
    a = normalized(c["range_min"], joint, c) + 2.0
    b = normalized(c["range_max"], joint, c) - 2.0
    require(a < b, "INVALID_CALIBRATION", f"{joint}: calibrated range is too narrow for a two-degree margin.")
    return [a, b]

