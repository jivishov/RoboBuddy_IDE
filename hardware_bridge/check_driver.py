"""Import/signature check only. Does not construct a robot or open any serial port."""
import importlib.metadata
import inspect
from .common import LEROBOT_VERSION

def main():
    version = importlib.metadata.version("lerobot")
    if version != LEROBOT_VERSION:
        raise SystemExit(f"Expected LeRobot {LEROBOT_VERSION}, found {version}.")
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors import MotorCalibration
    assert "use_degrees" in SO101FollowerConfig.__dataclass_fields__
    assert "disable_torque" in inspect.signature(FeetechMotorsBus.disconnect).parameters
    for method in ("connect", "sync_read", "sync_write", "enable_torque", "disable_torque", "set_half_turn_homings", "write_calibration"):
        assert callable(getattr(FeetechMotorsBus, method, None)), method
    assert set(MotorCalibration.__dataclass_fields__) == {"id", "drive_mode", "homing_offset", "range_min", "range_max"}
    # Constructing the pinned adapter loads its actual vendor SDK but never opens a port.
    import tempfile
    from pathlib import Path
    with tempfile.TemporaryDirectory() as folder:
        robot = SO101Follower(SO101FollowerConfig(port="UNOPENED", id="api_check", calibration_dir=Path(folder), use_degrees=True, cameras={}))
        assert not robot.bus.is_connected
    print(f"LeRobot {version} hardware API imports and signatures passed. No USB connection attempted.")

if __name__ == "__main__":
    main()

