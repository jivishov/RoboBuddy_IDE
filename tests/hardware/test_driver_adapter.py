import dataclasses
from pathlib import Path
import tempfile
import unittest
from hardware_bridge.common import JOINTS, JOINT_IDS
from hardware_bridge.lerobot_bus import LeRobotMotorBus

class DriverTests(unittest.TestCase):
    def test_real_lerobot_robot_uses_only_its_bus_for_read_only_connect(self):
        # Uses the installed released class, but replaces the wire before any connection.
        from lerobot.robots.so_follower import SO101Follower,SO101FollowerConfig
        from lerobot.motors import MotorCalibration
        with tempfile.TemporaryDirectory() as temp:
            robot=SO101Follower(SO101FollowerConfig(port="UNOPENED",id="driver_test",calibration_dir=Path(temp),use_degrees=True,cameras={}))
            class SpyBus:
                is_connected=False
                is_calibrated=True
                def __init__(self):self.calls=[]
                def connect(self):self.calls.append("connect_handshake");self.is_connected=True
                def sync_read(self,register,**kwargs):
                    self.calls.append(("read",register))
                    return {j:2047 if register=="Present_Position" else 0 for j in JOINTS}
                def disconnect(self,disable_torque=True):self.calls.append(("close",disable_torque));self.is_connected=False
            spy=SpyBus()
            robot.bus=spy
            robot.calibration={j:MotorCalibration(id=JOINT_IDS[j],drive_mode=0,homing_offset=0,range_min=512,range_max=3583) for j in JOINTS}
            def forbidden(*args,**kwargs):raise AssertionError("High-level robot connect/configure was called")
            robot.connect=forbidden;robot.configure=forbidden
            adapter=LeRobotMotorBus("UNOPENED","driver_test",robot_factory=lambda:robot)
            adapter.connect();adapter.read();adapter.close()
            self.assertEqual(spy.calls[0],"connect_handshake")
            self.assertEqual(spy.calls[-1],("close",False))
            self.assertTrue(all(call=="connect_handshake" or call[0] in ("read","close") for call in spy.calls))
    def test_normalization_matches_the_released_lerobot_motor_bus(self):
        from lerobot.motors.feetech import FeetechMotorsBus
        from lerobot.motors import Motor,MotorCalibration,MotorNormMode
        from hardware_bridge.common import normalized
        calib={j:MotorCalibration(id=JOINT_IDS[j],drive_mode=1 if j=="gripper" else 0,homing_offset=0,range_min=512,range_max=3583) for j in JOINTS}
        bus=FeetechMotorsBus(port="UNOPENED",motors={j:Motor(JOINT_IDS[j],"sts3215",MotorNormMode.RANGE_0_100 if j=="gripper" else MotorNormMode.DEGREES) for j in JOINTS},calibration=calib)
        for raw in (512,1024,2047,3000,3583):
            result=bus._normalize({JOINT_IDS[j]:raw for j in JOINTS})
            for j in JOINTS:self.assertAlmostEqual(result[JOINT_IDS[j]],normalized(raw,j,dataclasses.asdict(calib[j])),places=7)
        self.assertFalse(bus.is_connected)

if __name__=="__main__":unittest.main()

