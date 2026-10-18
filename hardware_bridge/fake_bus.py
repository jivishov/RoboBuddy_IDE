from __future__ import annotations
import copy
from .common import JOINTS, JOINT_IDS, calibration_dict

class FakeMotorBus:
    """Explicit demo bus. No imports or calls to any real serial implementation."""
    is_demo = True
    driver = "demo-only"
    def __init__(self, port="DEMO", robot_id="demo_arm"):
        self.port, self.robot_id = port, robot_id
        self.connected = False
        self.calibration = {j: {"id": JOINT_IDS[j], "drive_mode": 0, "homing_offset": 0,
                              "range_min": 0 if j == "wrist_roll" else 512,
                              "range_max": 4095 if j == "wrist_roll" else 3583} for j in JOINTS}
        self.positions = {j: 2047 for j in JOINTS}
        self.torque = {j: False for j in JOINTS}
        self.goals = dict(self.positions)
        self.writes = []
        self.fail_reads = False
        self.calibration_matches = True
    def connect(self):
        self.connected = True
    def close(self):
        self.connected = False  # Closing is not a torque-off operation.
    def read(self):
        if not self.connected or self.fail_reads:
            raise OSError("Demo motor bus is unavailable")
        for j in JOINTS:
            if self.torque[j]:
                self.positions[j] = self.goals[j]
        return {"raw": dict(self.positions), "torque": dict(self.torque),
                "calibration": copy.deepcopy(self.calibration),
                "calibrated": self.calibration_matches, "model": "sts3215",
                "motor_ids": dict(JOINT_IDS), "firmware": "demo"}
    def set_position_mode(self):
        self.writes.append(("mode", "position"))
    def write_raw_goals(self, values):
        self.writes.append(("goal", dict(values)))
        self.goals.update(values)
    def enable(self):
        self.writes.append(("torque", True))
        self.torque = {j: True for j in JOINTS}
    def disable(self):
        self.writes.append(("torque", False))
        self.torque = {j: False for j in JOINTS}
    def home_midpoint(self):
        self.writes.append(("homing", "midpoint"))
        self.positions = {j: 2047 for j in JOINTS}
        self.calibration_matches = False
        return {j: 0 for j in JOINTS}
    def apply_calibration(self, data):
        self.calibration = calibration_dict(data)
        self.calibration_matches = True
        self.writes.append(("calibration", copy.deepcopy(data)))
    def backup(self):
        pass
    def set_manual_pose(self, data):
        for j, value in data.items():
            if j in JOINTS:
                self.positions[j] = int(value)

