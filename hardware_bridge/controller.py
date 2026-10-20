from __future__ import annotations
import copy
import math
import secrets
import threading
import time
from .common import API_VERSION, BridgeError, JOINTS, JOINT_IDS, arm_id, calibration_dict, calibration_hash, control_limits, normalized, number, raw_target, require, units
from .fake_bus import FakeMotorBus
from .lerobot_bus import LeRobotMotorBus, ports

class HardwareController:
    """One motor owner, bounded jogs, monotonic motion scheduling and a browser lease."""
    LEASE_SECONDS = 3.0
    SAMPLE_MAX_AGE = 0.6
    COMMAND_SECONDS = 3.0
    PERIOD = 0.1
    GESTURE_SECONDS = 0.35
    def __init__(self, demo=False, bus_factory=None, clock=time.monotonic, wall_clock=time.time):
        self.demo, self.clock, self.wall_clock = demo, clock, wall_clock
        self.factory = bus_factory or (FakeMotorBus if demo else LeRobotMotorBus)
        self.lock = threading.RLock()
        self.bus = None
        self.owner = None
        self.sequence = 0
        self.lease_until = 0.0
        self.state = "disconnected"
        self.sample = None
        self.sample_at = 0.0
        self.target = None
        self.last_sent = None
        self.command_until = 0.0
        self.last_tick = self.clock()
        self.fault = None
        self.calibration_stage = None
        self.range_mins = {}
        self.range_maxes = {}
        self.homing_offsets = {}
        self.operator_enabled = False
        self.events = []
        self.gesture = None
        self.gesture_until = 0.0
        self.gesture_moving = False
        self.gesture_hold = None
    def _event(self, event, **details):
        self.events.append({"time": self.wall_clock(), "event": event, **details})
        self.events = self.events[-100:]
    def claim(self):
        with self.lock:
            self._expire()
            require(self.owner is None, "OPERATOR_BUSY", "Another panel owns this helper. Disconnect it, or wait for its lease to expire.")
            self.owner = secrets.token_urlsafe(24)
            self.sequence = 0
            self.lease_until = self.clock() + self.LEASE_SECONDS
            return {"session": self.owner, "leaseSeconds": self.LEASE_SECONDS, "state": self.snapshot()}
    def _owner(self, owner):
        self._expire()
        require(self.owner is not None and secrets.compare_digest(str(owner or ""), self.owner), "SESSION_EXPIRED", "This panel no longer owns the arm. Reconnect explicitly.", 401)
    def heartbeat(self, owner):
        with self.lock:
            self._owner(owner)
            self.lease_until = self.clock() + self.LEASE_SECONDS
            return self.snapshot()
    def mutate(self, owner, sequence, issued_at_ms, operation, args):
        with self.lock:
            self._owner(owner)
            require(type(sequence) is int and self.sequence < sequence <= 2147483647, "STALE_COMMAND", "Duplicate or out-of-order command rejected.", 400)
            timestamp = number(issued_at_ms, "issuedAtMs") / 1000
            require(abs(self.wall_clock() - timestamp) <= 2.0, "EXPIRED_COMMAND", "The command expired before reaching the helper.", 400)
            if operation == "gesture_move":
                require(abs(self.wall_clock() - timestamp) <= 0.25, "EXPIRED_COMMAND", "The camera command arrived too late. Enable gestures again.", 400)
            self.sequence = sequence
            require(isinstance(args, dict), "INVALID_ARGUMENT", "Command arguments must be an object.", 400)
            operations = {"connect": self._connect, "disconnect": self._disconnect, "arm": self._arm,
                          "move": self._move, "stop": self._stop, "disable": self._disable,
                          "gesture_start": self._gesture_start, "gesture_move": self._gesture_move,
                          "calibration_start": self._calibration_start, "calibration_middle": self._calibration_middle,
                          "calibration_finish": self._calibration_finish, "calibration_cancel": self._calibration_cancel,
                          "calibration_import": self._calibration_import}
            require(operation in operations, "INVALID_ARGUMENT", "Unsupported hardware operation.", 400)
            self._wrote_in_operation = False
            try:
                operations[operation](args)
                return self.snapshot()
            except BridgeError as exc:
                if self._wrote_in_operation:
                    self._fault(exc.code, str(exc))
                raise
            except Exception as exc:
                self._fault("MOTOR_IO", str(exc))
                raise BridgeError("MOTOR_IO", "Motor communication failed; physical stop is unconfirmed. Check the arm locally.", 503) from exc
    def _read(self):
        require(self.bus is not None, "NOT_CONNECTED", "Connect a motor bus first.")
        started = self.clock()
        sample = self.bus.read()
        require(set(sample["raw"]) == set(JOINTS), "MOTOR_MISMATCH", "All six motors must answer.")
        require(all(type(v) is int and 0 <= v <= 4095 for v in sample["raw"].values()), "ENCODER_RANGE", "An encoder returned an unsupported position.")
        self.sample, self.sample_at = sample, self.clock()
        self._last_io_ms = max(0, (self.sample_at - started) * 1000)
        return sample
    def _fresh(self):
        require(self.sample is not None and self.clock() - self.sample_at <= self.SAMPLE_MAX_AGE, "STALE_TELEMETRY", "Fresh encoder readings are required before movement.")
        return self.sample
    def _ready(self):
        sample = self._fresh()
        require(sample["calibrated"], "CALIBRATION_REQUIRED", "Load matching calibration or complete the calibration wizard before enabling motors.")
        calibration_dict(sample["calibration"])
        require(all(sample["calibration"][j]["range_min"] <= sample["raw"][j] <= sample["calibration"][j]["range_max"] for j in JOINTS), "ENCODER_OUTSIDE_CALIBRATION", "A joint is outside its calibrated encoder range. Recheck calibration with torque disabled.")
        return sample
    def _lease_alive(self):
        require(self.owner is not None and self.clock() <= self.lease_until, "SESSION_EXPIRED", "The operator lease expired before the motor write.", 401)
    def _connect(self, args):
        require(self.bus is None, "ALREADY_CONNECTED", "Disconnect before selecting a different port.")
        port = args.get("port")
        require(isinstance(port, str) and len(port) <= 128, "INVALID_ARGUMENT", "Select a serial port.", 400)
        valid_ports = {"DEMO"} if self.demo else {p["device"] for p in ports()}
        require(port in valid_ports, "PORT_NOT_FOUND", "The selected serial port is no longer available.")
        robot_id = arm_id(args.get("armId", ""))
        self.bus = self.factory(port, robot_id)
        try:
            self.bus.connect()
            self._read()
        except Exception:
            try:
                self.bus.close()
            finally:
                self.bus = None
                self.sample = None
            raise
        self.state = "read_only"
        self.operator_enabled = False
        self.fault = None
        self._event("connected_read_only", port=port, armId=robot_id)
    def _arm(self, args):
        require(args.get("supported") is True, "SUPPORT_REQUIRED", "Confirm the arm is supported, mounted and clear.")
        require(self.state == "read_only", "INVALID_STATE", "Enable motors only from a read-only connection.")
        sample = self._read()
        self._ready()
        require(not any(sample["torque"].values()), "TORQUE_ALREADY_ON", "Motors are already enabled. Support the arm and explicitly disable them before enabling control.")
        self._lease_alive()
        # Set goals from actual ticks BEFORE torque. No rest pose, cached command or PID change.
        self._wrote_in_operation = True
        self.bus.set_position_mode()
        sample = self._read()
        self._ready()
        require(not any(sample["torque"].values()), "TORQUE_ALREADY_ON", "A motor became enabled during arming; stop and check the arm locally.")
        self._lease_alive()
        self.bus.write_raw_goals(sample["raw"])
        require(self.owner is not None and self.clock() <= self.lease_until, "SESSION_EXPIRED", "The operator lease expired before motor enable.", 401)
        self.bus.enable()
        self.operator_enabled = True
        sample = self._read()
        require(all(sample["torque"].values()), "TORQUE_UNCONFIRMED", "Not all motors confirmed enabled torque.")
        self.last_sent = self._positions(sample)
        self.state = "armed"
        self._event("motors_enabled_at_measured_pose")
    def _positions(self, sample):
        return {j: normalized(sample["raw"][j], j, sample["calibration"][j]) for j in JOINTS}
    def _move(self, args):
        require(self.gesture is None, "GESTURE_ACTIVE", "Pause gesture control before manual jogging.")
        require(self.state in ("armed", "moving", "holding") and self.operator_enabled, "NOT_ARMED", "Explicitly enable the motors before jogging.")
        sample = self._ready()
        require(all(sample["torque"].values()), "TORQUE_LOST", "A motor is no longer enabled.")
        values = args.get("targets")
        require(isinstance(values, dict) and len(values) == 1 and set(values) <= set(JOINTS), "INVALID_ARGUMENT", "The first release accepts one joint per manual command.", 400)
        positions = self._positions(sample)
        joint, requested = next(iter(values.items()))
        requested = number(requested, joint)
        lo, hi = control_limits(joint, sample["calibration"][joint])
        require(lo <= requested <= hi, "JOINT_LIMIT", f"{joint}: target must stay inside {lo:.2f}..{hi:.2f} {units(joint)}.", 400)
        max_delta = 5.0 if joint == "gripper" else 3.0
        require(abs(requested - positions[joint]) <= max_delta + 0.01, "JOG_TOO_LARGE", f"Use increments no larger than {max_delta:g} {units(joint)} from the measured position.", 400)
        require(self.target is None, "MOTION_BUSY", "Wait for this jog to settle or press Stop.")
        self.target = {**positions, joint: requested}
        self.last_sent = dict(positions)
        self.command_until = self.clock() + self.COMMAND_SECONDS
        self.last_tick = self.clock()
        self.state = "moving"
        self._event("manual_jog", joint=joint, requested=requested)
    def _gesture_start(self, args):
        require(args.get("supported") is True, "SUPPORT_REQUIRED", "Confirm the arm is supported, mounted and clear.")
        require(self.state in ("armed", "holding") and self.operator_enabled and self.target is None and self.gesture is None, "NOT_ARMED", "Enable motors and finish any manual jog before enabling gestures.")
        sample = self._ready()
        require(all(sample["torque"].values()), "TORQUE_LOST", "A motor is no longer enabled.")
        joint = args.get("joint")
        require(isinstance(joint, str) and joint in JOINTS, "INVALID_ARGUMENT", "Choose one SO-101 joint.", 400)
        self.gesture = {"id": secrets.token_urlsafe(18), "joint": joint}
        self.gesture_until = self.clock() + self.GESTURE_SECONDS
        self.gesture_moving = False
        self.last_sent = self._positions(sample)
        self.gesture_hold = dict(self.last_sent)
        self.last_tick = self.clock()
        self._event("gesture_enabled", joint=joint)
    def _gesture_move(self, args):
        require(self.gesture is not None and secrets.compare_digest(str(args.get("gestureId", "")), self.gesture["id"]), "GESTURE_EXPIRED", "Gesture control expired. Set neutral and explicitly enable it again.")
        joint = args.get("joint")
        require(joint == self.gesture["joint"], "INVALID_ARGUMENT", "Pause gestures before selecting another joint.", 400)
        direction = number(args.get("direction"), "direction")
        require(-1 <= direction <= 1, "INVALID_ARGUMENT", "Gesture direction must be between -1 and 1.", 400)
        require(self.operator_enabled and self.state in ("armed", "holding", "moving"), "NOT_ARMED", "Motors must be explicitly enabled.")
        sample = self._ready()
        require(all(sample["torque"].values()), "TORQUE_LOST", "A motor is no longer enabled.")
        positions = self._positions(sample)
        self.gesture_until = self.clock() + self.GESTURE_SECONDS
        if direction == 0:
            # A settled command can still leave a small residual servo goal.
            # Capture fresh encoders once on the transition back to neutral;
            # repeated centered frames must not continually rebase the hold.
            if self.gesture_moving:
                sample = self._read()
                self._ready()
                require(all(sample["torque"].values()), "TORQUE_LOST", "A motor is no longer enabled.")
                self._lease_alive()
                self.gesture_hold[joint] = self._positions(sample)[joint]
                self._wrote_in_operation = True
                self.bus.write_raw_goals({j: raw_target(v, j, sample["calibration"][j]) for j, v in self.gesture_hold.items()})
            self.target = None
            self.gesture_moving = False
            self.last_sent = dict(self.gesture_hold)
            self.state = "holding"
            return
        lo, hi = control_limits(joint, sample["calibration"][joint])
        # At most half a degree (or 1% opening) ahead of measured state, no queue.
        target = min(hi, max(lo, positions[joint] + direction * (1.0 if joint == "gripper" else 0.5)))
        require(abs(target - positions[joint]) <= (1.01 if joint == "gripper" else 0.51), "JOINT_LIMIT", "The measured joint is outside its usable range.")
        self.target = {**self.gesture_hold, joint: target}
        self.gesture_moving = True
        self.command_until = self.gesture_until
        self.state = "moving"
    def _stop(self, args=None):
        self.gesture = None
        self.gesture_moving = False
        self.target = None
        if self.bus is not None and self.operator_enabled:
            sample = self._read()
            self.bus.write_raw_goals(sample["raw"])
            self.last_sent = self._positions(sample)
            self.state = "holding"
            self._event("hold_requested", confirmed=False)
        elif self.bus is not None and self.state not in ("fault", "calibrating"):
            self.state = "read_only"
    def _disable(self, args):
        require(args.get("supported") is True, "SUPPORT_REQUIRED", "Support the arm before removing motor torque.")
        require(self.bus is not None, "NOT_CONNECTED", "Connect before disabling motors.")
        self.target = None
        self.gesture = None
        self._wrote_in_operation = True
        self.bus.disable()
        sample = self._read()
        require(not any(sample["torque"].values()), "TORQUE_UNCONFIRMED", "Some motors did not confirm disabled torque.")
        self.operator_enabled = False
        self.state = "read_only"
        self.fault = None
        self._event("torque_disabled_confirmed")
    def _disconnect(self, args=None):
        try:
            self._stop()
        except Exception as exc:
            self._fault("DISCONNECT_STOP_UNCONFIRMED", str(exc))
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception as exc:
                self._fault("PORT_CLOSE_UNCONFIRMED", str(exc))
        self.bus, self.sample = None, None
        self.operator_enabled = False
        self.target = None
        self.calibration_stage = None
        self.state = "disconnected"
        self.owner = None
        self._event("disconnected", torque="not_monitored")
    def _calibration_start(self, args):
        require(self.state == "read_only", "INVALID_STATE", "Calibration starts from read-only mode.")
        require(args.get("supported") is True, "SUPPORT_REQUIRED", "Support the arm for torque-disabled calibration.")
        self._read()
        require(not any(self.sample["torque"].values()), "TORQUE_ALREADY_ON", "Explicitly disable the supported arm first.")
        self.bus.backup()
        self._wrote_in_operation = True
        self.bus.set_position_mode()
        self.state, self.calibration_stage = "calibrating", "middle"
        self._event("calibration_started")
    def _calibration_middle(self, args):
        require(self.state == "calibrating" and self.calibration_stage == "middle", "INVALID_STATE", "Start calibration first.")
        require(args.get("middleConfirmed") is True, "MIDDLE_REQUIRED", "Place all joints at the documented middle pose first.")
        self._read()
        require(not any(self.sample["torque"].values()), "TORQUE_ALREADY_ON", "Calibration requires disabled torque.")
        self._wrote_in_operation = True
        self.homing_offsets = self.bus.home_midpoint()
        self._read()
        self.range_mins = dict(self.sample["raw"])
        self.range_maxes = dict(self.sample["raw"])
        self.calibration_stage = "ranges"
    def _calibration_finish(self, args):
        require(self.state == "calibrating" and self.calibration_stage == "ranges", "INVALID_STATE", "Record ranges first.")
        require(args.get("rangesConfirmed") is True, "RANGES_REQUIRED", "Confirm each joint's full usable range was sampled.")
        self._read()
        self._record_ranges()
        require(all(self.range_maxes[j] - self.range_mins[j] >= 64 for j in JOINTS if j != "wrist_roll"), "RANGE_NOT_RECORDED", "Move each joint except wrist roll through its usable range before saving.")
        data = {j: {"id": JOINT_IDS[j], "drive_mode": 0, "homing_offset": int(self.homing_offsets[j]),
                    "range_min": 0 if j == "wrist_roll" else self.range_mins[j],
                    "range_max": 4095 if j == "wrist_roll" else self.range_maxes[j]} for j in JOINTS}
        self._wrote_in_operation = True
        self.bus.apply_calibration(calibration_dict(data))
        self._read()
        require(self.sample["calibrated"] and not any(self.sample["torque"].values()), "CALIBRATION_UNCONFIRMED", "Saved calibration or disabled torque could not be confirmed.")
        self.state, self.calibration_stage = "read_only", None
        self._event("calibration_saved", hash=calibration_hash(data))
    def _calibration_cancel(self, args):
        require(self.state == "calibrating", "INVALID_STATE", "No calibration is in progress.")
        self.calibration_stage = None
        self.state = "read_only"
        self._read()
        self._event("calibration_cancelled", note="Use backup/import if homing offsets were changed.")
    def _calibration_import(self, args):
        require(self.state == "read_only", "INVALID_STATE", "Import calibration in read-only mode.")
        require(args.get("supported") is True, "SUPPORT_REQUIRED", "Support the arm before applying saved calibration.")
        data = calibration_dict(args.get("calibration"))
        self._read()
        require(not any(self.sample["torque"].values()), "TORQUE_ALREADY_ON", "Explicitly disable the supported arm first.")
        self._wrote_in_operation = True
        self.bus.apply_calibration(data)
        self._read()
        require(self.sample["calibrated"], "CALIBRATION_UNCONFIRMED", "Motor calibration does not match the imported file.")
    def _record_ranges(self):
        for j in JOINTS:
            if j != "wrist_roll":
                value = self.sample["raw"][j]
                self.range_mins[j] = min(self.range_mins[j], value)
                self.range_maxes[j] = max(self.range_maxes[j], value)
    def _expire(self):
        if self.gesture is not None and self.clock() > self.gesture_until:
            try:
                self._stop()
                self._event("gesture_deadline_hold")
            except Exception as exc:
                self._fault("GESTURE_STOP_UNCONFIRMED", str(exc))
        if self.owner is not None and self.clock() > self.lease_until:
            try:
                self._stop()
            except Exception as exc:
                self._fault("LEASE_STOP_UNCONFIRMED", str(exc))
            if self.state == "calibrating":
                self.calibration_stage = None
                self.state = "read_only"
            self.owner = None
            # A new session must explicitly disable/re-arm; do not inherit torque authority.
            self.operator_enabled = False
            if self.state != "fault" and self.bus:
                self.state = "read_only"
            self._event("operator_lease_expired")
    def _fault(self, code, detail):
        self.gesture = None
        self.target = None
        self.state = "fault"
        self.fault = {"code": code, "message": detail, "physicalStopConfirmed": False}
        self._event("fault", code=code)
    def tick(self):
        with self.lock:
            self._expire()
            if self.bus is None or self.state == "fault":
                return
            try:
                sample = self._read()
                # A slow bus read may consume the remaining camera/operator lease.
                # Never send an old moving goal after that read completes.
                self._expire()
                if self.state == "calibrating" and self.calibration_stage == "ranges":
                    require(not any(sample["torque"].values()), "TORQUE_LOST", "A motor became enabled during calibration.")
                    self._record_ranges()
                if self.target is not None:
                    if self.clock() > self.command_until:
                        self._stop()
                        self._event("jog_deadline_hold")
                        return
                    require(sample["calibrated"] and all(sample["torque"].values()), "TORQUE_LOST", "Motor torque or calibration changed during movement.")
                    actual = self._positions(sample)
                    dt = min(self.PERIOD, max(0.0, self.clock() - self.last_tick))
                    self.last_tick = self.clock()
                    next_values = {}
                    for j in JOINTS:
                        speed = 10.0 if j == "gripper" else 5.0
                        delta = self.target[j] - self.last_sent[j]
                        next_values[j] = self.last_sent[j] + math.copysign(min(abs(delta), speed * dt), delta)
                    self._lease_alive()
                    self.bus.write_raw_goals({j: raw_target(v, j, sample["calibration"][j]) for j, v in next_values.items()})
                    self.last_sent = next_values
                    if all(abs(actual[j] - self.target[j]) <= (0.2 if j == "gripper" else 0.15) for j in JOINTS):
                        self.target = None
                        self.state = "armed"
                        self._event("manual_jog_settled")
            except Exception as exc:
                self._fault(getattr(exc, "code", "MOTOR_IO"), str(exc))
    def snapshot(self):
        with self.lock:
            sample = self.sample
            age = None if sample is None else max(0.0, self.clock() - self.sample_at)
            calibrated = bool(sample and sample["calibrated"])
            joints = {}
            if sample:
                for j in JOINTS:
                    c = sample["calibration"].get(j)
                    joints[j] = {"motorId": JOINT_IDS[j], "rawTicks": sample["raw"][j],
                                 "position": normalized(sample["raw"][j], j, c) if calibrated else None,
                                 "unit": units(j), "torqueEnabled": sample["torque"][j],
                                 "limits": control_limits(j, c) if calibrated else None,
                                 "requested": self.target.get(j) if self.target else None}
            return {"apiVersion": API_VERSION, "mode": "demo" if self.demo else "hardware",
                    "capabilities": {"webcamGestures": True}, "gesture": copy.deepcopy(self.gesture),
                    "state": self.state, "connected": self.bus is not None,
                    "port": self.bus.port if self.bus else None,
                    "armId": self.bus.robot_id if self.bus else None,
                    "driver": self.bus.driver if self.bus else None,
                    "ownerActive": self.owner is not None, "operatorEnabled": self.operator_enabled,
                    "calibrated": calibrated, "calibrationHash": calibration_hash(sample["calibration"]) if calibrated else None,
                    "sampleAgeMs": age * 1000 if age is not None else None,
                    "fresh": age is not None and age <= self.SAMPLE_MAX_AGE and self.state != "fault",
                    "ioMs": getattr(self, "_last_io_ms", None),
                    "joints": joints, "fault": copy.deepcopy(self.fault),
                    "calibrationStage": self.calibration_stage,
                    "recordedRanges": {j: [self.range_mins.get(j), self.range_maxes.get(j)] for j in JOINTS},
                    "events": copy.deepcopy(self.events[-12:]),
                    "hardwareCommissioned": False}

