// MicroDuck Phase 5C physical controller.
//
// This is the deployed controller contract, reproduced exactly:
//
//   pollen-robotics/microduck@590b986  duck-control/src/obs.rs      the 61-value observation
//                                      duck-control/src/model.rs    joint order, home pose
//                                      duck-control/src/policy.rs   standing threshold
//                                      robotd/src/control.rs        skills, scaling, filters
//
// It holds no MuJoCo handle and no ONNX session. It turns an authoritative physical
// observation plus a bounded command into a policy input, and a policy output into fourteen
// joint targets. Everything that decides how the robot moves lives here, so it can be
// compared value-for-value against an independently derived reference fixture.
//
// It cannot move the robot by itself: it never produces a root pose, a root velocity, an
// object velocity, or a task outcome. The only thing it emits is actuator targets.

export const MICRODUCK_OBSERVATION_WIDTH = 61;
export const MICRODUCK_ACTION_WIDTH = 14;

// The fifteen wire slots, from duck-ipc-proto JOINT_NAMES. The physical model carries no
// mouth joint - neither does the upstream RL model - but the wire order is preserved so the
// fourteen policy slots keep their published identity and a mouth off-by-one is impossible
// to introduce silently.
export const MICRODUCK_WIRE_JOINT_ORDER = Object.freeze([
  'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee', 'left_ankle',
  'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll', 'mouth',
  'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle',
]);
export const MICRODUCK_MOUTH_WIRE_INDEX = 9;
export const MICRODUCK_POLICY_JOINT_ORDER = Object.freeze(
  MICRODUCK_WIRE_JOINT_ORDER.filter((_, index) => index !== MICRODUCK_MOUTH_WIRE_INDEX),
);

// duck-control/src/model.rs DEFAULT_POSITION, mouth slot removed. Identical to
// microduck_rl HOME_FRAME and to the source scene STAND keyframe.
export const MICRODUCK_HOME_POSITION_RAD = Object.freeze([
  0.0, -0.0873, -0.4579, -0.0049, 0.4530,
  0.3491, 0.3491, 0.0, 0.0,
  0.0, 0.0873, 0.4579, 0.0049, -0.4530,
]);

// duck-control/src/obs.rs, the layout table.
export const MICRODUCK_OBSERVATION_LAYOUT = Object.freeze({
  gyro: Object.freeze([0, 3]),
  projectedGravity: Object.freeze([3, 6]),
  jointPositionOffset: Object.freeze([6, 20]),
  jointVelocity: Object.freeze([20, 34]),
  previousRawAction: Object.freeze([34, 48]),
  command: Object.freeze([48, 61]),
});
// Command sub-layout. Body x, y and yaw are unbound in training and are always zero.
export const MICRODUCK_COMMAND_LAYOUT = Object.freeze({
  twist: Object.freeze([48, 51]),
  head: Object.freeze([51, 55]),
  bodyXY: Object.freeze([55, 57]),
  bodyZ: 57,
  bodyRoll: 58,
  bodyPitch: 59,
  bodyYaw: 60,
});

// robotd/src/control.rs Tuning + SkillTuning defaults, and duck-control/src/policy.rs.
export const MICRODUCK_TUNING = Object.freeze({
  actionScale: 0.9,
  standingActionScale: 1.0,
  standingGainRatio: 0.8,
  gain: 200,
  headLowpassAlpha: 0.5,
  legsLowpassAlpha: 0.7,
  standingThreshold: 0.05,
});
export const MICRODUCK_SKILL_TUNING = Object.freeze({
  groundPickPeriodSeconds: 4.0,
  groundPickActionScale: 1.0,
  groundPickGainRatio: 1.0,
  groundPickEndPhase: 0.7,
  kickDurationSeconds: 0.5,
  rouladeDurationSeconds: 1.0,
  rouladeActionScale: 1.0,
  rouladeGainRatio: 1.0,
  riseSeconds: 1.0,
});
// robotd/src/control.rs HEAD_JOINTS: neck_pitch, head_pitch, head_yaw, head_roll.
export const MICRODUCK_HEAD_JOINT_SLOTS = Object.freeze([5, 6, 7, 8]);

// The controller cadence, and the physics decimation that realises it.
export const MICRODUCK_PHYSICS_TIMESTEP_SECONDS = 0.005;
export const MICRODUCK_CONTROL_DECIMATION = 4;
export const MICRODUCK_CONTROL_INTERVAL_SECONDS = MICRODUCK_PHYSICS_TIMESTEP_SECONDS * MICRODUCK_CONTROL_DECIMATION;
export const MICRODUCK_CONTROL_HZ = 1 / MICRODUCK_CONTROL_INTERVAL_SECONDS;

// The networks robotd's scheduler can select, and the policy each one names.
export const MICRODUCK_NETS = Object.freeze({
  WALK: 'walk',
  STAND: 'stand',
  SIT: 'sit',
  RISE: 'rise',
  GROUND_PICK: 'ground_pick',
  KICK_LEFT: 'kick_left',
  KICK_RIGHT: 'kick_right',
  ROULADE: 'roulade',
});
const NET_POLICY = Object.freeze({
  walk: 'walking',
  stand: 'stand',
  sit: 'sitstand',
  rise: 'sitstand',
  ground_pick: 'ground_pick',
  kick_left: 'kick_left',
  kick_right: 'kick_right',
  roulade: 'roulade',
});

export function policyIdForNet(net) {
  const id = NET_POLICY[net];
  if (!id) throw new Error(`Unknown MicroDuck net: ${net}`);
  return id;
}

function finite(value, label) {
  const number = Number(value);
  if (!Number.isFinite(number)) throw new TypeError(`${label} must be finite`);
  return number;
}

function fixedVector(values, width, label) {
  if (!values || values.length !== width) throw new TypeError(`${label} must contain exactly ${width} values`);
  return Array.from(values, (value, index) => finite(value, `${label}[${index}]`));
}

/**
 * Map a policy's fourteen outputs onto the fifteen wire slots, leaving the mouth untouched.
 * The mirror of the observation's mouth exclusion, written once so the two directions cannot
 * disagree about where the policy's n-th value belongs.
 */
export function scatterPolicyAction(action, mouthValue = 0) {
  const values = fixedVector(action, MICRODUCK_ACTION_WIDTH, 'Policy action');
  const wire = new Array(MICRODUCK_WIRE_JOINT_ORDER.length);
  for (let slot = 0; slot < MICRODUCK_ACTION_WIDTH; slot += 1) {
    wire[slot < MICRODUCK_MOUTH_WIRE_INDEX ? slot : slot + 1] = values[slot];
  }
  wire[MICRODUCK_MOUTH_WIRE_INDEX] = finite(mouthValue, 'Mouth value');
  return wire;
}

/** The inverse: read fourteen policy slots out of fifteen wire slots, skipping the mouth. */
export function gatherPolicyValues(wire) {
  const values = fixedVector(wire, MICRODUCK_WIRE_JOINT_ORDER.length, 'Wire values');
  return values.filter((_, index) => index !== MICRODUCK_MOUTH_WIRE_INDEX);
}

/**
 * Normalise a bounded command into the physical units the observation carries.
 * Nothing here clamps to a policy-friendly range: bounds belong to the command surface.
 */
export function makeCommand({ twist = [0, 0, 0], head = [0, 0, 0, 0], body = {} } = {}) {
  return Object.freeze({
    twist: Object.freeze(fixedVector(twist, 3, 'Command twist')),
    head: Object.freeze(fixedVector(head, 4, 'Command head')),
    body: Object.freeze({
      z: finite(body.z ?? 0, 'Command body.z'),
      roll: finite(body.roll ?? 0, 'Command body.roll'),
      pitch: finite(body.pitch ?? 0, 'Command body.pitch'),
    }),
  });
}

export function twistMagnitude(command) {
  const [x, y, z] = command.twist;
  return Math.sqrt(x * x + y * y + z * z);
}

/**
 * Assemble the 61-value observation.
 *
 * `jointPositionRad` and `jointVelocityRadS` are the fourteen policy joints, in policy order,
 * read from the authoritative physics. `previousRawAction` is the previous policy output,
 * raw - before action scaling - because that is what the policy was trained observing.
 */
export function buildPolicyObservation({
  gyroRadS,
  projectedGravity,
  jointPositionRad,
  jointVelocityRadS,
  previousRawAction,
  command,
} = {}) {
  const gyro = fixedVector(gyroRadS, 3, 'Gyro');
  const gravity = fixedVector(projectedGravity, 3, 'Projected gravity');
  const positions = fixedVector(jointPositionRad, MICRODUCK_ACTION_WIDTH, 'Joint position');
  const velocities = fixedVector(jointVelocityRadS, MICRODUCK_ACTION_WIDTH, 'Joint velocity');
  const previous = fixedVector(previousRawAction, MICRODUCK_ACTION_WIDTH, 'Previous raw action');
  const cmd = command && command.twist ? command : makeCommand(command || {});

  const values = new Float32Array(MICRODUCK_OBSERVATION_WIDTH);
  values.set(gyro, MICRODUCK_OBSERVATION_LAYOUT.gyro[0]);
  values.set(gravity, MICRODUCK_OBSERVATION_LAYOUT.projectedGravity[0]);
  for (let index = 0; index < MICRODUCK_ACTION_WIDTH; index += 1) {
    values[MICRODUCK_OBSERVATION_LAYOUT.jointPositionOffset[0] + index] = positions[index] - MICRODUCK_HOME_POSITION_RAD[index];
  }
  values.set(velocities, MICRODUCK_OBSERVATION_LAYOUT.jointVelocity[0]);
  values.set(previous, MICRODUCK_OBSERVATION_LAYOUT.previousRawAction[0]);
  const base = MICRODUCK_OBSERVATION_LAYOUT.command[0];
  values[base + 0] = cmd.twist[0];
  values[base + 1] = cmd.twist[1];
  values[base + 2] = cmd.twist[2];
  values[base + 3] = cmd.head[0];
  values[base + 4] = cmd.head[1];
  values[base + 5] = cmd.head[2];
  values[base + 6] = cmd.head[3];
  values[base + 7] = 0; // body x - unbound in training
  values[base + 8] = 0; // body y - unbound in training
  values[base + 9] = cmd.body.z;
  values[base + 10] = cmd.body.roll;
  values[base + 11] = cmd.body.pitch;
  values[base + 12] = 0; // body yaw - unbound in training
  return values;
}

/**
 * Rotate world -Z into the trunk body frame and normalise it, exactly as
 * duck-control/src/imu.rs builds the observation's gravity block:
 * `normalise(rotate_inverse(quat, [0, 0, -1]))`.
 *
 * The normalisation is not decoration. imu.rs states the reason outright: "in steady state
 * gravity must be a unit vector at any orientation - the policy observes it directly and was
 * trained on normalised input." For a unit quaternion it is a no-op, but a declared setup
 * perturbation can hand us a quaternion that is not quite unit, and the policy would then
 * see a short gravity vector it has never been trained on.
 *
 * This is the single implementation. The authoritative worker imports it rather than
 * carrying its own copy, because two copies of a rotation are two chances to disagree.
 */
export function projectedGravityFromQuaternion(quaternionWxyz) {
  const [w, x, y, z] = fixedVector(quaternionWxyz, 4, 'Quaternion');
  const v = [0, 0, -1];
  const t = [
    2 * (y * v[2] - z * v[1]),
    2 * (z * v[0] - x * v[2]),
    2 * (x * v[1] - y * v[0]),
  ];
  const g = [
    v[0] - w * t[0] + (y * t[2] - z * t[1]),
    v[1] - w * t[1] + (z * t[0] - x * t[2]),
    v[2] - w * t[2] + (x * t[1] - y * t[0]),
  ];
  const norm = Math.hypot(g[0], g[1], g[2]);
  return norm > 0 ? [g[0] / norm, g[1] / norm, g[2] / norm] : g;
}

/**
 * The deployed controller, including its scheduler and its state.
 *
 * State it owns, and that reset() clears, per robotd's Controller::reset:
 *   * the previous raw policy action, which the observation feeds back;
 *   * the previous filtered targets, which anchor the low-pass filters;
 *   * the skill windows.
 *
 * It is deliberately synchronous and pure apart from that state: the caller supplies the
 * authoritative physical observation and an inference function, and receives targets.
 */
export class MicroDuckController {
  constructor({ tuning = MICRODUCK_TUNING, skills = MICRODUCK_SKILL_TUNING, availablePolicies = null } = {}) {
    this.tuning = Object.freeze({ ...MICRODUCK_TUNING, ...tuning });
    this.skills = Object.freeze({ ...MICRODUCK_SKILL_TUNING, ...skills });
    // Which nets may be selected at all. A capability the physical package does not support
    // is absent here, so the scheduler cannot fall through to it.
    this.availablePolicies = new Set(availablePolicies || ['walking', 'stand', 'sitstand', 'ground_pick', 'kick_left', 'kick_right', 'roulade']);
    this.reset();
  }

  reset() {
    this.previousRawAction = new Array(MICRODUCK_ACTION_WIDTH).fill(0);
    this.previousTargets = null;
    this.groundPickPhase = null;
    this.kick = null;
    this.roulade = null;
    this.sit = 'up';
    this.riseRemaining = 0;
    this.lastStep = null;
  }

  hasPolicy(id) { return this.availablePolicies.has(id); }
  get busy() { return this.groundPickPhase != null || this.kick != null || this.roulade != null || this.sit !== 'up'; }
  get sitting() { return this.sit === 'sitting'; }

  willStand(magnitude) {
    // duck-control/src/policy.rs: `twist_magnitude <= self.standing_threshold`. The
    // comparison is inclusive, so a twist of exactly 0.05 stands rather than walks. A strict
    // `<` here would walk on the one command a person is most likely to type exactly.
    return this.hasPolicy('stand') && Number(magnitude) <= this.tuning.standingThreshold;
  }

  /** Begin a skill window. Returns false when the physical package does not carry that policy. */
  requestSkill(skill) {
    if (skill === 'kick_left' || skill === 'kick_right') {
      if (!this.hasPolicy(skill)) return false;
      this.kick = { left: skill === 'kick_left', remaining: this.skills.kickDurationSeconds };
      return true;
    }
    if (skill === 'ground_pick') {
      if (!this.hasPolicy('ground_pick')) return false;
      this.groundPickPhase = 0;
      return true;
    }
    if (skill === 'roulade') {
      if (!this.hasPolicy('roulade')) return false;
      this.roulade = this.skills.rouladeDurationSeconds;
      return true;
    }
    if (skill === 'sit' || skill === 'stand_up') {
      if (!this.hasPolicy('sitstand')) return false;
      if (skill === 'sit') this.sit = 'sitting';
      else { this.sit = 'rising'; this.riseRemaining = this.skills.riseSeconds; }
      return true;
    }
    return false;
  }

  /** Select the network and the effective command for this tick. */
  selectNet(command, { bodyActive = false } = {}) {
    if (this.roulade != null) return { net: MICRODUCK_NETS.ROULADE, effective: makeCommand({}) };
    if (this.kick != null) {
      // The kick networks are trained with every command slot at zero, head and body included.
      return { net: this.kick.left ? MICRODUCK_NETS.KICK_LEFT : MICRODUCK_NETS.KICK_RIGHT, effective: makeCommand({}) };
    }
    if (this.groundPickPhase != null) {
      // The twist slots carry the phase encoding; head and body are zero-padded.
      const angle = 2 * Math.PI * this.groundPickPhase;
      return { net: MICRODUCK_NETS.GROUND_PICK, effective: makeCommand({ twist: [Math.cos(angle), Math.sin(angle), 0] }) };
    }
    if (this.sit === 'sitting') {
      // The posture flag rides the twist vx slot: 1 = sit, 0 = stand. Head and body stay live.
      return { net: MICRODUCK_NETS.SIT, effective: makeCommand({ twist: [1, 0, 0], head: command.head, body: command.body }) };
    }
    if (this.sit === 'rising') {
      return { net: MICRODUCK_NETS.RISE, effective: makeCommand({ twist: [0, 0, 0], head: command.head, body: command.body }) };
    }
    const effective = bodyActive ? makeCommand({ twist: [0, 0, 0], head: command.head, body: command.body }) : command;
    const standing = this.willStand(twistMagnitude(effective)) || (bodyActive && this.hasPolicy('stand'));
    return { net: standing ? MICRODUCK_NETS.STAND : MICRODUCK_NETS.WALK, effective };
  }

  /**
   * The scale and gain for a tick. Recomputed from the active state every tick, so a
   * sit -> stand cycle cannot leave a stale scale behind.
   */
  tuningFor(net, effective) {
    const standingTuned = net === MICRODUCK_NETS.STAND
      || ((net === MICRODUCK_NETS.KICK_LEFT || net === MICRODUCK_NETS.KICK_RIGHT || net === MICRODUCK_NETS.SIT || net === MICRODUCK_NETS.RISE)
        && this.willStand(twistMagnitude(effective)));
    if (net === MICRODUCK_NETS.ROULADE) {
      return { scale: this.skills.rouladeActionScale, gain: Math.round(this.tuning.gain * this.skills.rouladeGainRatio), standingTuned };
    }
    if (net === MICRODUCK_NETS.GROUND_PICK) {
      return { scale: this.skills.groundPickActionScale, gain: Math.round(this.tuning.gain * this.skills.groundPickGainRatio), standingTuned };
    }
    if (net === MICRODUCK_NETS.SIT || net === MICRODUCK_NETS.RISE) {
      // The prototype pins the scale at 1.0 for the whole sit/rise cycle.
      return { scale: 1.0, gain: standingTuned ? Math.round(this.tuning.gain * this.tuning.standingGainRatio) : this.tuning.gain, standingTuned };
    }
    if (standingTuned) {
      return { scale: this.tuning.standingActionScale, gain: Math.round(this.tuning.gain * this.tuning.standingGainRatio), standingTuned };
    }
    return { scale: this.tuning.actionScale, gain: this.tuning.gain, standingTuned };
  }

  /** Home pose + scale x action, then the first-order low-pass the deployed daemon applies. */
  targetsFor(rawAction, scale) {
    const wire = scatterPolicyAction(rawAction, 0);
    const targets = MICRODUCK_POLICY_JOINT_ORDER.map((_, slot) => {
      const wireIndex = slot < MICRODUCK_MOUTH_WIRE_INDEX ? slot : slot + 1;
      return MICRODUCK_HOME_POSITION_RAD[slot] + scale * wire[wireIndex];
    });
    if (this.previousTargets) {
      const head = this.tuning.headLowpassAlpha;
      const legs = this.tuning.legsLowpassAlpha;
      for (let slot = 0; slot < targets.length; slot += 1) {
        const alpha = MICRODUCK_HEAD_JOINT_SLOTS.includes(slot) ? head : legs;
        if (alpha == null) continue;
        targets[slot] = alpha * targets[slot] + (1 - alpha) * this.previousTargets[slot];
      }
    }
    return targets;
  }

  /**
   * The first half of a controller tick: expire finished skill windows, select the network,
   * and assemble the 61-value observation.
   *
   * It is split from the second half only because browser ONNX inference is asynchronous.
   * No controller state that the observation depends on is mutated here, so an inference that
   * is cancelled between the two halves leaves the controller exactly as it was.
   */
  beginTick({ observation, command, bodyActive = false }) {
    const cmd = command && command.twist ? command : makeCommand(command || {});

    // Expire finished windows first, so a tick after the deadline runs the next thing rather
    // than one more frame of a finished move.
    if (this.kick && this.kick.remaining <= 0) this.kick = null;
    if (this.roulade != null && this.roulade <= 0) this.roulade = null;
    if (this.sit === 'rising' && this.riseRemaining <= 0) this.sit = 'up';

    const { net, effective } = this.selectNet(cmd, { bodyActive });
    const policyId = policyIdForNet(net);
    if (!this.hasPolicy(policyId)) throw new Error(`MicroDuck physical package does not support policy ${policyId}`);

    const policyInput = buildPolicyObservation({
      gyroRadS: observation.gyroRadS,
      projectedGravity: observation.projectedGravity,
      jointPositionRad: observation.jointPositionRad,
      jointVelocityRadS: observation.jointVelocityRadS,
      previousRawAction: this.previousRawAction,
      command: effective,
    });
    return { net, policyId, effective, observation: policyInput };
  }

  /**
   * The second half: post-process the raw policy output into joint targets and advance the
   * skill windows. `pending` is what beginTick returned.
   */
  completeTick(pending, rawAction, dtSeconds = MICRODUCK_CONTROL_INTERVAL_SECONDS) {
    const { net, policyId, effective, observation: policyInput } = pending;
    const action = fixedVector(rawAction, MICRODUCK_ACTION_WIDTH, 'Policy output');
    this.previousRawAction = action;

    const { scale, gain, standingTuned } = this.tuningFor(net, effective);
    const targets = this.targetsFor(action, scale);
    this.previousTargets = targets;

    // Advance the windows after the tick that used them.
    const dt = finite(dtSeconds, 'dtSeconds');
    if (this.groundPickPhase != null) {
      this.groundPickPhase += dt / this.skills.groundPickPeriodSeconds;
      if (this.groundPickPhase >= this.skills.groundPickEndPhase) this.groundPickPhase = null;
    }
    if (this.kick) this.kick.remaining -= dt;
    if (this.roulade != null) this.roulade -= dt;
    if (this.sit === 'rising') this.riseRemaining -= dt;

    this.lastStep = Object.freeze({
      net, policyId, scale, gain, standingTuned,
      busy: this.busy,
      commandTwist: Object.freeze([...effective.twist]),
      targetsRad: Object.freeze(
        Object.fromEntries(MICRODUCK_POLICY_JOINT_ORDER.map((id, slot) => [id, targets[slot]])),
      ),
      rawAction: Object.freeze([...action]),
      observation: policyInput,
    });
    return this.lastStep;
  }

  /**
   * One controller tick, for a caller with synchronous inference.
   *
   * `infer(policyId, observationFloat32) -> Float32Array(14)`. The browser path uses
   * beginTick/completeTick instead, around an async session; both go through the same
   * observation assembly and the same post-processing.
   */
  step({ observation, command, bodyActive = false, dtSeconds = MICRODUCK_CONTROL_INTERVAL_SECONDS, infer }) {
    if (typeof infer !== 'function') throw new TypeError('MicroDuckController.step requires an infer(policyId, observation) function');
    const pending = this.beginTick({ observation, command, bodyActive });
    return this.completeTick(pending, infer(pending.policyId, pending.observation), dtSeconds);
  }
}
