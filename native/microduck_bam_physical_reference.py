#!/usr/bin/env python3
"""Run the existing MicroDuck controller/evidence harness on a BAM M6 physical plant.

The controller, policy loading, task definitions and evidence format remain independently
maintained in microduck_reference.py. This file replaces only its old XML-position-actuator
Plant with the BAM v1.0.1 XL330/M6 dynamics used by the upstream CPU deployment rehearsal:
voltage-domain P control, back-EMF, identified armature, load/Stribeck friction and one-step
load-dependent battery sag. It also applies the deployed median-of-three IMU preprocessing.

Electrical values 7.4 V and 0.1 V/Nm are the pinned upstream CPU-regression reference
condition. They are not claimed to be measurements of a particular assembled robot.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import microduck_reference as ref  # noqa: E402

LegacyPlant = ref.Plant

KT = 0.36601349688984386
R = 2.8113923539223227
ARMATURE = 0.0018077432831600838
FRICTION_BASE = 0.004771183165566
FRICTION_STRIBECK = 0.004676345799486616
LOAD_MOTOR = 0.2667860954283698
LOAD_EXTERNAL = 8.515871897059342e-06
LOAD_MOTOR_STRIBECK = 1.0722918395099123e-05
LOAD_EXTERNAL_STRIBECK = 0.08077928978935671
LOAD_MOTOR_QUAD = 0.009972471242139415
LOAD_EXTERNAL_QUAD = 0.004902565732332559
DTHETA_STRIBECK = 2.890372094130307
STRIBECK_ALPHA = 8.683259907618984
FRICTION_VISCOUS = 0.005359668274599504
ERROR_GAIN = (4096.0 / (2.0 * math.pi)) / (256.0 * 885.0)
NOMINAL_VIN = 7.4
VIN_DROP_GAIN = 0.1
VIN_MIN = 6.0
TRAINING_VIN_MAX = 8.2
FORCE_CEILING = TRAINING_VIN_MAX * KT / R


def _normalise(v):
    a = np.asarray(v, dtype=np.float64)
    mag = float(np.linalg.norm(a))
    return a / mag if mag > 0.1 else np.array([0.0, 0.0, -1.0])


class DeploymentImuFilter:
    """pollen-robotics/microduck@590b986 duck-control/src/imu.rs."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.gyro_history = [np.zeros(3), np.zeros(3)]
        self.gravity_history = [np.array([0.0, 0.0, -1.0]), np.array([0.0, 0.0, -1.0])]
        self.last_gyro = np.zeros(3)
        self.last_gravity = np.array([0.0, 0.0, -1.0])

    def sample(self, gyro, gravity):
        gyro = np.asarray(gyro, dtype=np.float64)
        gravity = _normalise(gravity)
        self.last_gyro = np.median(np.stack([self.gyro_history[0], self.gyro_history[1], gyro]), axis=0)
        self.last_gravity = np.median(np.stack([self.gravity_history[0], self.gravity_history[1], gravity]), axis=0)
        self.gyro_history = [self.gyro_history[1], gyro.copy()]
        self.gravity_history = [self.gravity_history[1], gravity.copy()]


def bam_frictionloss(motor_torque, external_torque, velocity):
    """BAM Model.compute_frictions M6 at v1.0.1 (CPU sign-gated quadratic term)."""
    gearbox = abs(external_torque * LOAD_EXTERNAL - motor_torque * LOAD_MOTOR)
    gearbox_s = abs(external_torque * LOAD_EXTERNAL_STRIBECK - motor_torque * LOAD_MOTOR_STRIBECK)
    stribeck = math.exp(-(abs(velocity / DTHETA_STRIBECK) ** STRIBECK_ALPHA))
    loss = FRICTION_BASE + gearbox + stribeck * FRICTION_STRIBECK + gearbox_s * stribeck
    sign_mismatch = np.sign(external_torque) != np.sign(motor_torque)
    if sign_mismatch:
        if abs(external_torque) < abs(motor_torque):
            quad = LOAD_EXTERNAL_QUAD * abs(external_torque) ** 2
        elif abs(external_torque) > abs(motor_torque):
            quad = LOAD_MOTOR_QUAD * abs(motor_torque) ** 2
        else:
            quad = 0.0
        loss += stribeck * quad
    return max(0.0, loss)


class BamPlant(LegacyPlant):
    BAM_VERSION = "1.0.1"
    BAM_REVISION = "ab81512c44f1f709b99ef332addb5e51568cd51c"

    def __init__(self, model_key, timestep=ref.PHYSICS_TIMESTEP):
        self._bam_ready = False
        super().__init__(model_key, timestep=timestep)
        m = self.model
        m.actuator_gainprm[:, :] = 0.0
        m.actuator_gainprm[:, 0] = 1.0
        m.actuator_biasprm[:, :] = 0.0
        m.actuator_forcerange[:, 0] = -FORCE_CEILING
        m.actuator_forcerange[:, 1] = FORCE_CEILING
        m.dof_armature[self.vadr:self.vadr + ref.ACTION_LEN] = ARMATURE
        # bam.mjlab.BamActuator.edit_spec zeros XML damping/friction. Dynamic BAM
        # friction is written immediately before every mj_step below.
        m.dof_frictionloss[self.vadr:self.vadr + ref.ACTION_LEN] = 0.0
        m.dof_damping[self.vadr:self.vadr + ref.ACTION_LEN] = 0.0
        mujoco.mj_setConst(m, self.data)

        self._bam_ready = True
        self.firmware_gain = ref.NOMINAL_FIRMWARE_GAIN
        self.actuation_enabled = True
        self.previous_motor_torque = np.zeros(ref.ACTION_LEN, dtype=np.float64)
        self.last_motor_torque = np.zeros(ref.ACTION_LEN, dtype=np.float64)
        self.effective_vin = NOMINAL_VIN
        self.imu_filter = DeploymentImuFilter()
        self.imu_sample_steps = max(1, int(round(0.02 / self.model.opt.timestep)))
        self.steps_since_imu = 0
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self._sample_imu()
        self.setup_log[-1].update({
            "plant": "BAM XL330/M6 deployment-reference",
            "bamVersion": self.BAM_VERSION,
            "nominalVinV": NOMINAL_VIN,
            "vinDropGainVPerNm": VIN_DROP_GAIN,
        })

    def set_actuator_kp(self, kp):
        # LegacyPlant.__init__ calls this through reset before BAM conversion is ready.
        if not getattr(self, "_bam_ready", False):
            return LegacyPlant.set_actuator_kp(self, kp)
        kp = float(kp)
        # Compatibility adapter for the independent legacy controller harness. `kp` is
        # converted back to the firmware register value; MuJoCo stiffness is never mutated.
        self.applied_kp = None
        self.firmware_gain = max(0.0, kp / ref.IDENTIFIED_KP * ref.NOMINAL_FIRMWARE_GAIN)
        self.actuation_enabled = kp > 0.0
        if not self.actuation_enabled:
            self.previous_motor_torque[:] = 0.0
            self.last_motor_torque[:] = 0.0

    def _friction_constraint_force(self, joint_id):
        total = 0.0
        for i in range(int(self.data.nefc)):
            if (int(self.data.efc_type[i]) == int(mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF)
                    and int(self.data.efc_id[i]) == int(joint_id)):
                total += float(self.data.efc_force[i])
        return total

    def _bam_step(self, targets):
        load = float(np.abs(self.previous_motor_torque).sum())
        self.effective_vin = max(VIN_MIN, NOMINAL_VIN - VIN_DROP_GAIN * load)
        next_motor = np.zeros(ref.ACTION_LEN, dtype=np.float64)

        for slot in range(ref.ACTION_LEN):
            q = float(self.data.qpos[self.qadr + slot])
            dq = float(self.data.qvel[self.vadr + slot])
            error = float(targets[slot]) - q
            duty = float(np.clip(error * self.firmware_gain * ERROR_GAIN, -1.0, 1.0))
            voltage = self.effective_vin * duty
            motor = KT * voltage / R - KT * KT * dq / R
            if not self.actuation_enabled:
                motor = 0.0
            next_motor[slot] = motor

            dof = self.vadr + slot
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, ref.POLICY_JOINT_ORDER[slot])
            previous_solved_actuator = float(self.data.qfrc_actuator[dof])
            external = (-float(self.data.qfrc_bias[dof]) + float(self.data.qfrc_constraint[dof])
                        - self._friction_constraint_force(joint_id))
            self.model.dof_frictionloss[dof] = bam_frictionloss(previous_solved_actuator, external, dq)
            self.model.dof_damping[dof] = FRICTION_VISCOUS
            self.data.ctrl[slot] = float(np.clip(motor, -FORCE_CEILING, FORCE_CEILING))

        mujoco.mj_step(self.model, self.data)
        self.previous_motor_torque = next_motor
        self.last_motor_torque = next_motor.copy()
        self.steps_since_imu += 1
        if self.steps_since_imu >= self.imu_sample_steps:
            self.steps_since_imu = 0
            self._sample_imu()

    def _raw_imu(self):
        d = self.data
        return (
            d.sensordata[self.gyro_adr:self.gyro_adr + 3].astype(np.float64).copy(),
            ref.projected_gravity(d.xquat[self.trunk].astype(np.float64)),
        )

    def _sample_imu(self):
        gyro, gravity = self._raw_imu()
        self.imu_filter.sample(gyro, gravity)

    def observe(self):
        d = self.data
        return dict(
            gyro=self.imu_filter.last_gyro.copy(),
            gravity=self.imu_filter.last_gravity.copy(),
            joint_pos=d.qpos[self.qadr:self.qadr + ref.ACTION_LEN].astype(np.float64).copy(),
            joint_vel=d.qvel[self.vadr:self.vadr + ref.ACTION_LEN].astype(np.float64).copy(),
        )

    def setup_orientation(self, quat_wxyz, label):
        super().setup_orientation(quat_wxyz, label)
        self.imu_filter.reset()
        self.steps_since_imu = 0
        self._sample_imu()
        self.setup_log[-1]["imuHistoryReset"] = True

    def settle(self, seconds, hold_home=True):
        targets = np.asarray(ref.HOME_RAD if hold_home else self.data.qpos[self.qadr:self.qadr + ref.ACTION_LEN], dtype=np.float64)
        for _ in range(int(round(seconds / self.model.opt.timestep))):
            self._bam_step(targets)
        self.setup_log.append({"event": "settle", "seconds": seconds,
                               "simulationTime": float(self.data.time), "plant": "BAM M6"})

    def advance(self, steps, contact_sink=None):
        targets = np.asarray(self.data.ctrl[:ref.ACTION_LEN], dtype=np.float64).copy()
        for _ in range(steps):
            self._bam_step(targets)
            if contact_sink is not None:
                self._collect_contacts(contact_sink)


ref.Plant = BamPlant

if __name__ == "__main__":
    raise SystemExit(ref.main())
