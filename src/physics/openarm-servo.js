// Simulation controller settings, not measured OpenArm firmware/motor limits.
export const OPENARM_CONTROL_PROFILE = Object.freeze({
  id: 'openarm-gravity-assisted-quintic-v3',
  armReferenceSpeedRadS: 0.75,
  fingerReferenceSpeedRadS: 0.60,
  gripperOperatingEffortNm: 1.2,
  maximumMotionSeconds: 12,
  gravityFeedforward: true,
  hardwareValidated: false,
});
export const smoothStep5 = (u) => { const t = Math.max(0, Math.min(1, u)); return t * t * t * (10 + t * (-15 + 6 * t)); };

export class OpenArmServo {
  constructor(model, data, actuators, joints) {
    this.model = model; this.data = data; this.actuators = actuators; this.joints = joints;
    this.paths = new Map();
    for (const actuator of actuators.values()) {
      const q = Number(data.qpos[joints.get(actuator.jointId).qpos]);
      this.paths.set(actuator.jointId, { start: q, target: q, time: Number(data.time), duration: 0 });
    }
  }
  reference(jointId, time = Number(this.data.time)) {
    const p = this.paths.get(jointId);
    return p.duration === 0 ? p.target : p.start + (p.target - p.start) * smoothStep5((time - p.time) / p.duration);
  }
  target(jointId) { return this.paths.get(jointId)?.target ?? null; }
  request(targets, durationSeconds = null) {
    const time = Number(this.data.time);
    const entries = Object.entries(targets).map(([id, value]) => ({ id, value, start: this.reference(id, time) }));
    const minimum = Math.max(0.04, ...entries.map(({ id, value, start }) => 1.875 * Math.abs(value - start) / (id.includes('finger') ? OPENARM_CONTROL_PROFILE.fingerReferenceSpeedRadS : OPENARM_CONTROL_PROFILE.armReferenceSpeedRadS)));
    const duration = durationSeconds == null ? Math.max(0.40, minimum) : durationSeconds;
    if (typeof duration !== 'number' || !Number.isFinite(duration) || duration < minimum - 1e-9 || duration > OPENARM_CONTROL_PROFILE.maximumMotionSeconds) {
      throw new RangeError(`Motion duration must be ${minimum.toFixed(3)}..${OPENARM_CONTROL_PROFILE.maximumMotionSeconds} simulation seconds for these targets (quintic reference-speed limit)`);
    }
    // Validate the complete request before changing any trajectory.
    for (const { id, value, start } of entries) this.paths.set(id, { start, target: value, time, duration });
    return { durationSeconds: duration, minimumDurationSeconds: minimum };
  }
  apply() {
    for (const actuator of this.actuators.values()) {
      const joint = this.joints.get(actuator.jointId);
      const kp = Number(this.model.actuator_gainprm[actuator.id * 10]);
      const reference = this.reference(actuator.jointId);
      // Bias compensation is inserted THROUGH the bounded position actuator.
      // No additional qfrc_applied/xfrc_applied force can bypass its effort cap.
      const bias = actuator.jointId.includes('finger') ? 0 : Number(this.data.qfrc_bias[joint.dof]) / kp;
      this.data.ctrl[actuator.id] = Math.max(actuator.controlRange[0], Math.min(actuator.controlRange[1], reference + bias));
    }
  }
}
