import {
  G1_EFFORT_LIMIT_NM, G1_JOINT_ORDER, G1_JOINT_RANGE_RAD, G1_STAND_COM_HEIGHT_ABOVE_ANKLE_M,
  G1_TOTAL_MASS_KG, G1_VELOCITY_LIMIT_RAD_S,
} from './unitree-g1-source-audit.js';

// Unitree G1 low-level actuation and standing control.
//
// Everything in this module is pure: it turns a requested target into a *bounded command*, and a
// bounded command into an actuator torque. It never touches simulation state. MuJoCo integrates
// the torque and owns q and dq; nothing here writes qpos, qvel or a root pose.

// --- timing ------------------------------------------------------------------------------------
// Recorded separately and deliberately. See UNITREE_G1_RECONCILIATION for the provenance of each.
export const G1_PHYSICS_TIMESTEP_SECONDS = 0.002;
export const G1_LOWLEVEL_CONTROL_INTERVAL_SECONDS = 0.002;
export const G1_STAND_CONTROL_INTERVAL_SECONDS = 0.002;
export const G1_OBSERVATION_INTERVAL_SECONDS = 0.02;
export const G1_UNITREE_FSM_INTERVAL_SECONDS = 0.001;

// --- bounded low-level command surface ----------------------------------------------------------
// The internal layer can represent the full Unitree low-level motor command. The learner and agent
// surfaces above it expose only bounded abstractions.
export const G1_MAX_KP = 400;
export const G1_MAX_KD = 60;

// --- Tier 3 source controller: unitree_rl_mjlab G1 FixStand ---------------------------------------
export const UNITREE_FIXSTAND_KP = Object.freeze([
  100, 100, 100, 150, 40, 40,
  100, 100, 100, 150, 40, 40,
  200, 200, 200,
  40, 40, 40, 40, 40, 40, 40,
  40, 40, 40, 40, 40, 40, 40,
]);
export const UNITREE_FIXSTAND_KD = Object.freeze([
  2, 2, 2, 4, 2, 2,
  2, 2, 2, 4, 2, 2,
  5, 5, 5,
  10, 10, 10, 10, 10, 10, 10,
  10, 10, 10, 10, 10, 10, 10,
]);
export const UNITREE_FIXSTAND_RAMP_SECONDS = 2;
export const G1_STAND_POSE_RAD = Object.freeze([
  -0.1, 0, 0, 0.3, -0.2, 0,
  -0.1, 0, 0, 0.3, -0.2, 0,
  0, 0, 0,
  0.35, 0.18, 0, 0.87, 0, 0, 0,
  0.35, -0.18, 0, 0.87, 0, 0, 0,
]);

// --- free-base ankle stability criterion ----------------------------------------------------------
// A rigid inverted-pendulum approximation gives an ankle-pitch stiffness scale m*g*h.
// This is a gain-selection heuristic, not a necessary/sufficient stability theorem for the
// articulated, contacting robot. Only the actual free-base trials establish the standing claim.
export const G1_ANKLE_STABILITY_REQUIREMENT_NM_PER_RAD = G1_TOTAL_MASS_KG * 9.81 * G1_STAND_COM_HEIGHT_ABOVE_ANKLE_M;
export const G1_ROBOBUDDY_ANKLE_KP = 250;
// Bounded above by explicit-integration stability on the *unloaded* foot: with the foot's own
// inertia of about 0.0105 kg m^2 (0.608 kg ankle-roll link plus the source 0.01 armature) a
// damping gain of kd needs kd * dt < 2 I, i.e. kd < ~10.5 at the nominal 2 ms step, or a
// commanded free-hanging ankle chatters instead of tracking. Measured: kd 30 limit-cycles a
// free foot at 4 rad/s while kd 10 tracks it to 0.9 mrad, and both hold the stand identically.
export const G1_ROBOBUDDY_ANKLE_KD = 10;

const ANKLE_INDICES = Object.freeze([4, 5, 10, 11]);

function withAnkleGains(base, value) {
  const out = [...base];
  for (const index of ANKLE_INDICES) out[index] = value;
  return Object.freeze(out);
}

// The repository standing controller: Unitree's FixStand structure, Unitree's standing posture, and
// Unitree's waist and arm gains, with only the four ankle gains replaced. Those replacements are
// repository-authored simulator gains derived from the criterion above; they are not, and must
// never be presented as, Unitree hardware settings.
export const ROBOBUDDY_STAND_KP = withAnkleGains(UNITREE_FIXSTAND_KP, G1_ROBOBUDDY_ANKLE_KP);
export const ROBOBUDDY_STAND_KD = withAnkleGains(UNITREE_FIXSTAND_KD, G1_ROBOBUDDY_ANKLE_KD);

export const G1_CONTROLLERS = Object.freeze({
  LOWLEVEL: 'unitree_g1_lowlevel_motor',
  JOINT_HOLD: 'unitree_g1_joint_hold_v1',
  STAND: 'robobuddy_g1_stand_v1',
  SOURCE_FIXSTAND: 'unitree_g1_fixstand_source_v1',
});

// The default gains behind a plain joint-target request, used wherever no standing claim is being
// made: joint inspection, the blocked-joint fixture, self-contact probing. They are Unitree's own
// FixStand gains, unchanged, which is exactly what that controller is - a joint-position hold.
// The identity of an explicit per-joint low-level command. It exists separately from the joint-hold
// profile because such a command may carry caller-supplied kp/kd, so reporting it as the source-gain
// hold would misdescribe the gains that are actually running.
export const G1_LOWLEVEL_COMMAND_PROFILE = Object.freeze({
  id: G1_CONTROLLERS.LOWLEVEL,
  label: 'Unitree low-level motor command at caller-supplied gains',
  claim: 'bounded per-joint low-level motor command: tau = kp*(q_target-q) + kd*(dq_target-dq) + tau_ff, clamped to the source effort limit. It makes no standing, balance or locomotion claim, and a gravity-loaded joint will visibly settle short of its target.',
});

export const G1_JOINT_HOLD_PROFILE = Object.freeze({
  id: G1_CONTROLLERS.JOINT_HOLD,
  label: 'Unitree source joint-position hold gains',
  kp: UNITREE_FIXSTAND_KP,
  kd: UNITREE_FIXSTAND_KD,
  rampSeconds: 0,
  targetPoseRad: G1_STAND_POSE_RAD,
  gainProvenance: 'unitree_rl_mjlab G1 config.yaml FSM.FixStand kp/kd, unchanged',
  claim: 'bounded joint-position hold. It makes no standing, balance or locomotion claim, and a gravity-loaded joint will visibly settle short of its target.',
});

export const G1_STAND_CONTROLLER_PROFILES = Object.freeze({
  [G1_CONTROLLERS.STAND]: Object.freeze({
    id: G1_CONTROLLERS.STAND,
    label: 'RoboBuddy engineering standing controller (FixStand-derived)',
    kp: ROBOBUDDY_STAND_KP,
    kd: ROBOBUDDY_STAND_KD,
    rampSeconds: UNITREE_FIXSTAND_RAMP_SECONDS,
    targetPoseRad: G1_STAND_POSE_RAD,
    gainProvenance: 'Unitree FixStand structure, posture, waist and arm gains; the four ankle gains are repository-authored',
    claim: 'physical standing / posture hold. Not dynamic balance, not perturbation recovery, not locomotion-ready.',
  }),
  [G1_CONTROLLERS.SOURCE_FIXSTAND]: Object.freeze({
    id: G1_CONTROLLERS.SOURCE_FIXSTAND,
    label: 'Unitree FixStand, exact source gains',
    kp: UNITREE_FIXSTAND_KP,
    kd: UNITREE_FIXSTAND_KD,
    rampSeconds: UNITREE_FIXSTAND_RAMP_SECONDS,
    targetPoseRad: G1_STAND_POSE_RAD,
    gainProvenance: 'unitree_rl_mjlab G1 config.yaml FSM.FixStand, unchanged',
    claim: 'source joint-position hold. Measured NOT to maintain free-base posture on this model; retained as evidence, not as a standing capability.',
  }),
});

const clamp = (value, low, high) => Math.min(high, Math.max(low, value));

export function jointIndex(jointId) {
  const index = G1_JOINT_ORDER.indexOf(jointId);
  if (index < 0) throw new Error(`Unknown Unitree G1 joint: ${jointId}`);
  return index;
}

/**
 * Bound one requested low-level motor command against the source limits.
 *
 * Returns both what was requested and what was accepted, so a caller can always see that the two
 * are different things. Nothing here promises the joint will reach the target.
 */
export function boundLowLevelCommand(jointId, requested = {}) {
  const index = jointIndex(jointId);
  const [low, high] = G1_JOINT_RANGE_RAD[index];
  const velocityLimit = G1_VELOCITY_LIMIT_RAD_S[index];
  const effortLimit = G1_EFFORT_LIMIT_NM[index];
  const requestedPositionRad = Number(requested.positionRad ?? 0);
  const requestedVelocityRadS = Number(requested.velocityRadS ?? 0);
  const requestedTorqueNm = Number(requested.feedforwardTorqueNm ?? 0);
  const requestedKp = Number(requested.kp ?? 0);
  const requestedKd = Number(requested.kd ?? 0);
  for (const [label, value] of Object.entries({ positionRad: requestedPositionRad, velocityRadS: requestedVelocityRadS, feedforwardTorqueNm: requestedTorqueNm, kp: requestedKp, kd: requestedKd })) {
    if (!Number.isFinite(value)) throw new TypeError(`Unitree G1 command ${jointId}.${label} must be finite`);
  }
  const accepted = {
    jointId,
    index,
    positionRad: clamp(requestedPositionRad, low, high),
    velocityRadS: clamp(requestedVelocityRadS, -velocityLimit, velocityLimit),
    feedforwardTorqueNm: clamp(requestedTorqueNm, -effortLimit, effortLimit),
    kp: clamp(requestedKp, 0, G1_MAX_KP),
    kd: clamp(requestedKd, 0, G1_MAX_KD),
  };
  return Object.freeze({
    ...accepted,
    requested: Object.freeze({
      positionRad: requestedPositionRad,
      velocityRadS: requestedVelocityRadS,
      feedforwardTorqueNm: requestedTorqueNm,
      kp: requestedKp,
      kd: requestedKd,
    }),
    bounded: accepted.positionRad !== requestedPositionRad
      || accepted.velocityRadS !== requestedVelocityRadS
      || accepted.feedforwardTorqueNm !== requestedTorqueNm
      || accepted.kp !== requestedKp
      || accepted.kd !== requestedKd,
    limits: Object.freeze({ jointRangeRad: G1_JOINT_RANGE_RAD[index], velocityLimitRadS: velocityLimit, effortLimitNm: effortLimit }),
  });
}

/**
 * The Unitree low-level motor law, exactly as the official unitree_mujoco SDK bridge writes it:
 *   tau = tau_ff + kp (q_target - q_measured) + kd (dq_target - dq_measured)
 * clamped to the source per-joint effort limit. This is the only path from a command to a torque.
 */
export function lowLevelTorqueNm(command, measured) {
  const q = Number(measured?.positionRad);
  const dq = Number(measured?.velocityRadS);
  if (!Number.isFinite(q) || !Number.isFinite(dq)) throw new TypeError('Unitree G1 low-level law needs finite measured q and dq');
  const limit = G1_EFFORT_LIMIT_NM[command.index];
  const raw = command.feedforwardTorqueNm + command.kp * (command.positionRad - q) + command.kd * (command.velocityRadS - dq);
  return clamp(raw, -limit, limit);
}

/** Linear interpolation between two 29-vectors, exactly Unitree's LinearInterpolator over ts=[0, ramp]. */
export function standTargetRad(profile, startPoseRad, elapsedSeconds) {
  const ramp = Number(profile.rampSeconds);
  const alpha = ramp <= 0 ? 1 : clamp(Number(elapsedSeconds) / ramp, 0, 1);
  return G1_JOINT_ORDER.map((_, index) => startPoseRad[index] * (1 - alpha) + profile.targetPoseRad[index] * alpha);
}

/**
 * One standing-controller update. Produces 29 bounded low-level commands; the caller turns them
 * into torques through lowLevelTorqueNm and hands those to MuJoCo.
 */
export function standCommands(profile, startPoseRad, elapsedSeconds) {
  const targets = standTargetRad(profile, startPoseRad, elapsedSeconds);
  return G1_JOINT_ORDER.map((jointId, index) => boundLowLevelCommand(jointId, {
    positionRad: targets[index],
    velocityRadS: 0,
    feedforwardTorqueNm: 0,
    kp: profile.kp[index],
    kd: profile.kd[index],
  }));
}

/** Total ankle-pitch stiffness a gain profile provides, against the free-base requirement. */
export function ankleStiffnessAudit(profile) {
  const total = Number(profile.kp[4]) + Number(profile.kp[10]);
  return Object.freeze({
    totalAnkleKpNmPerRad: total,
    requiredNmPerRad: G1_ANKLE_STABILITY_REQUIREMENT_NM_PER_RAD,
    marginRatio: total / G1_ANKLE_STABILITY_REQUIREMENT_NM_PER_RAD,
    passesRigidPendulumHeuristic: total > G1_ANKLE_STABILITY_REQUIREMENT_NM_PER_RAD,
  });
}

export function assertControllerTables() {
  const tables = {
    G1_JOINT_ORDER, G1_JOINT_RANGE_RAD, G1_EFFORT_LIMIT_NM, G1_VELOCITY_LIMIT_RAD_S,
    G1_STAND_POSE_RAD, UNITREE_FIXSTAND_KP, UNITREE_FIXSTAND_KD, ROBOBUDDY_STAND_KP, ROBOBUDDY_STAND_KD,
  };
  for (const [name, table] of Object.entries(tables)) {
    if (!Array.isArray(table) || table.length !== 29) throw new Error(`${name} must declare exactly 29 entries, found ${table?.length}`);
  }
  if (new Set(G1_JOINT_ORDER).size !== 29) throw new Error('G1_JOINT_ORDER contains duplicates');
  for (const [index, [low, high]] of G1_JOINT_RANGE_RAD.entries()) {
    if (!(low < high)) throw new Error(`G1_JOINT_RANGE_RAD[${index}] is not an increasing range`);
    const stand = G1_STAND_POSE_RAD[index];
    if (stand < low || stand > high) throw new Error(`Standing posture for ${G1_JOINT_ORDER[index]} is outside its source joint range`);
  }
  for (const limits of [G1_EFFORT_LIMIT_NM, G1_VELOCITY_LIMIT_RAD_S]) {
    if (limits.some((value) => !(Number(value) > 0))) throw new Error('Every source limit must be strictly positive');
  }
  for (const gains of [UNITREE_FIXSTAND_KP, ROBOBUDDY_STAND_KP]) {
    if (gains.some((value) => Number(value) > G1_MAX_KP)) throw new Error('A standing gain exceeds the bounded command surface');
  }
  for (const gains of [UNITREE_FIXSTAND_KD, ROBOBUDDY_STAND_KD]) {
    if (gains.some((value) => Number(value) > G1_MAX_KD)) throw new Error('A standing damping gain exceeds the bounded command surface');
  }
  // The two profiles must differ only in the four ankle gains: any other drift would silently turn
  // a documented minimal deviation into an undocumented custom controller.
  const differing = ROBOBUDDY_STAND_KP.map((value, index) => (value === UNITREE_FIXSTAND_KP[index] && ROBOBUDDY_STAND_KD[index] === UNITREE_FIXSTAND_KD[index] ? null : index)).filter((index) => index != null);
  if (differing.join(',') !== ANKLE_INDICES.join(',')) throw new Error(`The repository standing profile may differ from source FixStand only at the ankles, found indices ${differing.join(',')}`);
  return true;
}
