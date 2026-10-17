from __future__ import annotations
import dataclasses
import importlib.metadata
import json
from pathlib import Path
import shutil
import time
from .common import BridgeError, JOINTS, JOINT_IDS, LEROBOT_VERSION, arm_id, calibration_dict, require

class LeRobotMotorBus:
    """Thin audited adapter. Never calls robot.connect(), configure() or torque_disabled()."""
    is_demo = False
    driver = f"LeRobot {LEROBOT_VERSION} · SO101Follower motor bus"
    def __init__(self, port, robot_id, calibration_dir=None, robot_factory=None):
        self.port, self.robot_id = port, arm_id(robot_id)
        if robot_factory is None:
            try:
                version = importlib.metadata.version("lerobot")
                require(version == LEROBOT_VERSION, "DRIVER_VERSION", f"Expected LeRobot {LEROBOT_VERSION}; installed {version}. Run the hardware setup script.", 503)
                from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
                config = SO101FollowerConfig(port=port, id=self.robot_id, use_degrees=True, cameras={},
                                            max_relative_target=3.0,
                                            **({"calibration_dir": Path(calibration_dir)} if calibration_dir else {}))
                self.robot = SO101Follower(config)
            except BridgeError:
                raise
            except (ImportError, importlib.metadata.PackageNotFoundError) as exc:
                raise BridgeError("DRIVER_MISSING", f"Install the pinned hardware environment using Setup RoboBuddy Hardware.ps1. ({exc})", 503) from exc
        else:
            self.robot = robot_factory()
        self.bus = self.robot.bus
        self.connected = False
        self._offsets = None
    def connect(self):
        # Both LeRobot bus handshake and this diagnostic phase are read-only.
        # High-level SO101Follower.connect() must never be used here.
        try:
            self.bus.connect()
            self.connected = True
            self.read()
        except Exception:
            self.close()
            raise
    def close(self):
        if self.connected or getattr(self.bus, "is_connected", False):
            # Do not drop an unsupported arm by disabling torque as a side effect of close.
            self.bus.disconnect(disable_torque=False)
        self.connected = False
    def read(self):
        raw = self.bus.sync_read("Present_Position", normalize=False, num_retry=0)
        torque = self.bus.sync_read("Torque_Enable", normalize=False, num_retry=0)
        require(set(raw) == set(JOINTS) and set(torque) == set(JOINTS), "MOTOR_MISMATCH", "All six expected motors must answer.")
        calibration = {}
        if getattr(self.robot, "calibration", None):
            calibration = calibration_dict({j: dataclasses.asdict(c) for j, c in self.robot.calibration.items()})
        return {"raw": {j: int(raw[j]) for j in JOINTS},
                "torque": {j: bool(torque[j]) for j in JOINTS},
                "calibration": calibration,
                "calibrated": bool(calibration and self.bus.is_calibrated),
                "model": "sts3215", "motor_ids": dict(JOINT_IDS),
                "firmware": "handshake verified"}
    def set_position_mode(self):
        if hasattr(self.bus, "set_operating_mode"):
            self.bus.set_operating_mode("position")
        else:
            from lerobot.motors.feetech import OperatingMode
            for joint in JOINTS:
                self.bus.write("Operating_Mode", joint, OperatingMode.POSITION.value, normalize=False)
    def write_raw_goals(self, values):
        self.bus.sync_write("Goal_Position", values, normalize=False, num_retry=0)
    def enable(self):
        self.bus.enable_torque(num_retry=0)
    def disable(self):
        self.bus.disable_torque(num_retry=0)
    def backup(self):
        path = Path(self.robot.calibration_fpath)
        if path.exists():
            backup = path.with_name(path.name + f".backup-{time.time_ns()}")
            shutil.copy2(path, backup)
    def home_midpoint(self):
        self.backup()
        # This writes calibration only after the explicit supported-arm wizard step.
        return self.bus.set_half_turn_homings()
    def apply_calibration(self, data):
        from lerobot.motors import MotorCalibration
        clean = calibration_dict(data)
        self.backup()
        calib = {j: MotorCalibration(**c) for j, c in clean.items()}
        self.bus.write_calibration(calib)
        self.robot.calibration = calib
        # The pinned upstream serializer retains standard LeRobot file compatibility.
        self.robot._save_calibration()

def ports():
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [{"device": p.device, "description": p.description or p.device,
             "vid": p.vid, "pid": p.pid} for p in list_ports.comports()]

