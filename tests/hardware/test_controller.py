import copy
import unittest
from hardware_bridge.common import BridgeError, JOINTS, normalized, raw_target, calibration_dict
from hardware_bridge.controller import HardwareController
from hardware_bridge.fake_bus import FakeMotorBus

class Clock:
    def __init__(self): self.t=1000.0
    def __call__(self): return self.t
    def step(self, seconds=.1): self.t+=seconds

class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock()
        self.ctl=HardwareController(demo=True,clock=self.clock,wall_clock=self.clock)
        self.session=self.ctl.claim()["session"]
        self.seq=0
    def op(self, operation, args=None):
        self.seq+=1
        return self.ctl.mutate(self.session,self.seq,self.clock()*1000,operation,args or {})
    def connect(self):
        self.op("connect",{"port":"DEMO","armId":"test_arm"})
        return self.ctl.bus
    def arm(self):
        self.connect()
        return self.op("arm",{"supported":True})
    def test_start_and_pair_do_not_construct_a_motor_bus(self):
        self.assertIsNone(self.ctl.bus)
        self.assertEqual(self.ctl.snapshot()["state"],"disconnected")
    def test_connect_and_poll_do_not_write_any_motor_register(self):
        bus=self.connect()
        for _ in range(5): self.clock.step();self.ctl.tick()
        self.assertEqual(bus.writes,[])
        self.assertFalse(self.ctl.snapshot()["operatorEnabled"])
    def test_goals_are_actual_encoder_ticks_before_torque_enable(self):
        bus=self.connect();bus.positions["shoulder_pan"]=2200
        self.op("arm",{"supported":True})
        self.assertEqual(bus.writes[0],("mode","position"))
        self.assertEqual(bus.writes[1][1]["shoulder_pan"],2200)
        self.assertEqual(bus.writes[2],("torque",True))
    def test_enabling_without_support_is_rejected_before_write(self):
        bus=self.connect()
        with self.assertRaises(BridgeError): self.op("arm")
        self.assertEqual(bus.writes,[])
    def test_enabling_resamples_after_position_mode_before_seeding_goals(self):
        bus=self.connect()
        previous=bus.set_position_mode
        def mode_change():
            previous();bus.positions["shoulder_pan"]=2250
        bus.set_position_mode=mode_change
        self.op("arm",{"supported":True})
        self.assertEqual(bus.writes[1][1]["shoulder_pan"],2250)
    def test_invalid_calibrated_pose_cannot_enable_motors(self):
        bus=self.connect();bus.positions["shoulder_pan"]=20
        with self.assertRaises(BridgeError):self.op("arm",{"supported":True})
        self.assertEqual(bus.writes,[])
    def test_slow_diagnostic_cannot_enable_or_configure_after_lease_expiry(self):
        bus=self.connect();read=bus.read
        def slow_read():
            self.clock.step(3.1);return read()
        bus.read=slow_read
        with self.assertRaises(BridgeError):self.op("arm",{"supported":True})
        self.assertEqual(bus.writes,[])
    def test_preexisting_torque_cannot_be_silently_taken_over(self):
        bus=self.connect();bus.torque["elbow_flex"]=True
        with self.assertRaises(BridgeError): self.op("arm",{"supported":True})
        self.assertEqual(bus.writes,[])
    def test_mismatched_calibration_blocks_enable(self):
        bus=self.connect();bus.calibration_matches=False;self.ctl.tick()
        with self.assertRaises(BridgeError): self.op("arm",{"supported":True})
        self.assertEqual(bus.writes,[])
    def test_motion_requires_explicit_enable(self):
        bus=self.connect()
        with self.assertRaises(BridgeError): self.op("move",{"targets":{"shoulder_pan":1}})
        self.assertEqual(bus.writes,[])
    def test_nonfinite_bool_unknown_and_multijoint_targets_are_rejected(self):
        self.arm();before=len(self.ctl.bus.writes)
        for target in ({"shoulder_pan":float("nan")},{"shoulder_pan":True},{"root_x":1},{"shoulder_pan":1,"elbow_flex":1}):
            with self.subTest(target=target),self.assertRaises(BridgeError):self.op("move",{"targets":target})
        self.assertEqual(len(self.ctl.bus.writes),before)
    def test_large_jog_and_calibration_endpoint_are_rejected(self):
        self.arm()
        for value in (4,999,-999):
            with self.subTest(value=value),self.assertRaises(BridgeError):self.op("move",{"targets":{"shoulder_pan":value}})
    def test_jog_has_a_velocity_bound_and_settles_from_measured_readback(self):
        self.arm();self.op("move",{"targets":{"shoulder_pan":2}})
        for _ in range(10):
            self.clock.step();self.ctl.heartbeat(self.session);self.ctl.tick()
        state=self.ctl.snapshot()
        self.assertEqual(state["state"],"armed")
        self.assertAlmostEqual(state["joints"]["shoulder_pan"]["position"],2,delta=.15)
        goals=[x[1]["shoulder_pan"] for x in self.ctl.bus.writes if x[0]=="goal"]
        self.assertTrue(all(abs(b-a)<=7 for a,b in zip(goals,goals[1:])))
    def test_motion_cannot_accumulate_a_queue(self):
        self.arm();self.op("move",{"targets":{"shoulder_pan":2}})
        with self.assertRaises(BridgeError):self.op("move",{"targets":{"elbow_flex":1}})
    def test_lease_expiry_clears_pending_movement_and_does_not_remove_torque(self):
        self.arm();self.op("move",{"targets":{"shoulder_pan":2}})
        self.clock.step(3.1);self.ctl.tick()
        self.assertIsNone(self.ctl.target);self.assertIsNone(self.ctl.owner)
        self.assertFalse(self.ctl.operator_enabled)
        self.assertTrue(all(self.ctl.bus.torque.values()))
        self.assertEqual(self.ctl.state,"read_only")
    def test_old_operator_cannot_move_after_expiry_or_reclaim(self):
        self.arm();self.clock.step(3.1);self.ctl.tick();new=self.ctl.claim()["session"]
        self.assertNotEqual(new,self.session)
        with self.assertRaises(BridgeError):self.op("move",{"targets":{"shoulder_pan":1}})
    def test_duplicate_sequence_does_not_repeat_a_side_effect(self):
        bus=self.connect();seq=self.seq
        with self.assertRaises(BridgeError):self.ctl.mutate(self.session,seq,self.clock()*1000,"disable",{"supported":True})
        self.assertEqual(bus.writes,[])
    def test_delayed_and_future_commands_are_rejected(self):
        self.connect()
        for seconds in (-3,3):
            with self.assertRaises(BridgeError):self.ctl.mutate(self.session,999,(self.clock()+seconds)*1000,"arm",{"supported":True})
    def test_stale_telemetry_prevents_a_new_jog(self):
        self.arm();self.clock.step(.7)
        with self.assertRaises(BridgeError):self.op("move",{"targets":{"shoulder_pan":1}})
    def test_usb_loss_fault_has_no_confirmed_stop_and_blocks_motion(self):
        self.arm();self.ctl.bus.fail_reads=True;self.ctl.tick()
        state=self.ctl.snapshot()
        self.assertEqual(state["state"],"fault");self.assertFalse(state["fresh"])
        self.assertFalse(state["fault"]["physicalStopConfirmed"])
        with self.assertRaises(BridgeError):self.op("move",{"targets":{"shoulder_pan":1}})
    def test_faulted_port_can_be_closed_without_claiming_a_stop(self):
        self.arm();self.ctl.bus.fail_reads=True;self.ctl.tick();state=self.op("disconnect")
        self.assertFalse(state["connected"]);self.assertIsNotNone(state["fault"])
    def test_stop_clears_target_and_uses_current_ticks_not_requested_target(self):
        self.arm();self.op("move",{"targets":{"shoulder_pan":2}})
        self.op("stop")
        self.assertIsNone(self.ctl.target)
        self.assertEqual(self.ctl.bus.writes[-1][1],self.ctl.bus.positions)
        self.assertTrue(all(self.ctl.bus.torque.values()))
    def test_disable_requires_support_and_checks_motor_readback(self):
        self.arm()
        with self.assertRaises(BridgeError):self.op("disable")
        state=self.op("disable",{"supported":True})
        self.assertFalse(any(j["torqueEnabled"] for j in state["joints"].values()))
        self.assertEqual(state["state"],"read_only")
    def test_calibration_start_does_not_write_homing_before_middle_confirmation(self):
        bus=self.connect();self.op("calibration_start",{"supported":True})
        self.assertEqual(bus.writes,[("mode","position")])
        with self.assertRaises(BridgeError):self.op("calibration_middle")
        self.assertEqual(bus.writes,[("mode","position")])
    def test_calibration_wizard_records_ranges_and_keeps_torque_off(self):
        self.connect();self.op("calibration_start",{"supported":True})
        self.op("calibration_middle",{"middleConfirmed":True})
        self.ctl.bus.set_manual_pose({j:1000 for j in JOINTS});self.ctl.tick()
        self.ctl.bus.set_manual_pose({j:3000 for j in JOINTS});self.ctl.tick()
        state=self.op("calibration_finish",{"rangesConfirmed":True})
        self.assertTrue(state["calibrated"]);self.assertEqual(state["state"],"read_only")
        self.assertFalse(any(self.ctl.bus.torque.values()))
        self.assertEqual(self.ctl.bus.calibration["wrist_roll"]["range_max"],4095)
    def test_calibration_without_moving_ranges_cannot_be_saved(self):
        self.connect();self.op("calibration_start",{"supported":True});self.op("calibration_middle",{"middleConfirmed":True})
        with self.assertRaises(BridgeError):self.op("calibration_finish",{"rangesConfirmed":True})
        self.assertFalse(self.ctl.bus.calibration_matches)
    def test_import_rejects_missing_joints_and_wrong_ids_before_writes(self):
        bus=self.connect()
        for data in ({}, {**bus.calibration,"shoulder_pan":{**bus.calibration["shoulder_pan"],"id":6}}):
            with self.assertRaises(BridgeError):self.op("calibration_import",{"supported":True,"calibration":data})
        self.assertEqual(bus.writes,[])
    def test_second_browser_cannot_claim_the_same_operator_lease(self):
        with self.assertRaises(BridgeError):self.ctl.claim()
    def test_gripper_reverse_mapping_round_trip_is_not_radians(self):
        c={"id":6,"drive_mode":1,"homing_offset":0,"range_min":512,"range_max":3583}
        for v in (0,20,50,100):
            self.assertAlmostEqual(normalized(raw_target(v,"gripper",c),"gripper",c),v,delta=.1)

if __name__=="__main__": unittest.main()

