import unittest
from hardware_bridge.common import BridgeError
from test_controller import Clock
from hardware_bridge.controller import HardwareController


class GestureTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.ctl = HardwareController(demo=True, clock=self.clock, wall_clock=self.clock)
        self.session = self.ctl.claim()["session"]
        self.seq = 0
    def op(self, operation, args=None):
        self.seq += 1
        return self.ctl.mutate(self.session, self.seq, self.clock() * 1000, operation, args or {})
    def connect(self):
        self.op("connect", {"port": "DEMO", "armId": "gesture_test"})
    def arm(self):
        self.connect()
        self.op("arm", {"supported": True})
    def start(self, joint="shoulder_pan"):
        self.arm()
        return self.op("gesture_start", {"joint": joint, "supported": True})["gesture"]["id"]
    def move(self, token, direction=1, joint="shoulder_pan"):
        return self.op("gesture_move", {"gestureId": token, "joint": joint, "direction": direction})
    def test_camera_start_requires_enable_support_and_one_joint(self):
        self.connect()
        with self.assertRaises(BridgeError): self.op("gesture_start", {"joint": "shoulder_pan", "supported": True})
        self.op("arm", {"supported": True})
        for args in ({"joint": "root_x", "supported": True}, {"joint": "shoulder_pan"}):
            with self.assertRaises(BridgeError): self.op("gesture_start", args)
        self.assertIsNone(self.ctl.gesture)
    def test_start_has_no_motor_write_and_packets_are_bounded(self):
        token = self.start()
        writes = len(self.ctl.bus.writes)
        self.assertEqual(self.ctl.events[-1]["event"], "gesture_enabled")
        self.move(token)
        self.assertEqual(len(self.ctl.bus.writes), writes)
        self.assertLessEqual(abs(self.ctl.target["shoulder_pan"] - self.ctl.snapshot()["joints"]["shoulder_pan"]["position"]), .5)
        self.clock.step(.1); self.ctl.tick()
        self.assertLessEqual(abs(self.ctl.last_sent["shoulder_pan"]), .5)
    def test_heartbeat_does_not_renew_camera_deadline(self):
        token = self.start(); self.move(token)
        self.clock.step(.36); self.ctl.heartbeat(self.session)
        self.assertIsNone(self.ctl.gesture); self.assertIsNone(self.ctl.target)
        self.assertEqual(self.ctl.state, "holding")
        self.assertTrue(all(self.ctl.bus.torque.values()))
        with self.assertRaises(BridgeError): self.move(token)
    def test_late_packet_cannot_revive_expired_capture_between_ticks(self):
        token = self.start(); self.clock.step(.36)
        with self.assertRaises(BridgeError): self.move(token)
        self.assertIsNone(self.ctl.gesture)
    def test_stalled_browser_is_stopped_by_helper_tick(self):
        self.move(self.start()); self.clock.step(.36); self.ctl.tick()
        self.assertIsNone(self.ctl.target); self.assertIsNone(self.ctl.gesture)
        self.assertEqual(self.ctl.bus.writes[-1][1], self.ctl.bus.positions)
    def test_center_holds_and_stop_invalidates_old_camera_packets(self):
        token = self.start(); self.move(token)
        self.move(token, 0)
        self.assertIsNone(self.ctl.target); self.assertIsNotNone(self.ctl.gesture)
        self.op("stop")
        with self.assertRaises(BridgeError): self.move(token)
        self.assertIsNone(self.ctl.target)
    def test_other_joint_manual_moves_and_nonfinite_directions_are_rejected(self):
        token = self.start()
        for direction in (True, float("nan"), 2, -2):
            with self.assertRaises(BridgeError): self.move(token, direction)
        with self.assertRaises(BridgeError): self.move(token, 1, "elbow_flex")
        with self.assertRaises(BridgeError): self.op("move", {"targets": {"shoulder_pan": 1}})
        self.assertIsNone(self.ctl.target)
    def test_delayed_camera_packet_is_rejected_before_motion(self):
        token = self.start(); self.seq += 1
        with self.assertRaises(BridgeError):
            self.ctl.mutate(self.session, self.seq, (self.clock() - .26) * 1000, "gesture_move", {"gestureId": token, "joint": "shoulder_pan", "direction": 1})
        self.assertIsNone(self.ctl.target)
    def test_gripper_increment_and_limit_are_percent_open(self):
        token = self.start("gripper")
        self.move(token, -1, "gripper")
        actual = self.ctl.snapshot()["joints"]["gripper"]["position"]
        self.assertAlmostEqual(self.ctl.target["gripper"], actual - 1)
    def test_stream_keeps_progress_and_speed_bound(self):
        token = self.start()
        for _ in range(20):
            self.move(token); self.clock.step(.1); self.ctl.tick(); self.ctl.heartbeat(self.session)
        self.assertGreater(self.ctl.snapshot()["joints"]["shoulder_pan"]["position"], 5)
        goals = [item[1]["shoulder_pan"] for item in self.ctl.bus.writes if item[0] == "goal"]
        self.assertTrue(all(abs(b-a) <= 7 for a, b in zip(goals, goals[1:])))
    def test_stop_io_failure_clears_lease_and_reports_unconfirmed(self):
        self.move(self.start()); self.ctl.bus.fail_reads = True
        self.clock.step(.36); self.ctl.tick()
        self.assertIsNone(self.ctl.gesture); self.assertIsNone(self.ctl.target)
        self.assertFalse(self.ctl.fault["physicalStopConfirmed"])


if __name__ == "__main__": unittest.main()
