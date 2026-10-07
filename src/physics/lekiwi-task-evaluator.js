import { LEKIWI_COURIER_SCENE, wrapAngle, yawFromQuaternion } from './lekiwi-scene.js';

// The LeKiwi courier evaluator consumes authoritative MuJoCo observations only. It never reads a
// command, a controller stage, a timer, or the learner program's own output. Every predicate is a
// statement about the observed base body, the observed free beaker body, or a named geometry
// contact pair reported by the engine.

function vec3(value) {
  return Array.isArray(value) && value.length === 3 && value.every((item) => Number.isFinite(Number(item))) ? value.map(Number) : null;
}
function norm3(value) { const v = vec3(value); return v ? Math.hypot(...v) : Number.POSITIVE_INFINITY; }
function horizontal(a, b) { return Math.hypot(a[0] - b[0], a[1] - b[1]); }
function distance3(a, b) { return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]); }

function contactsWith(observation, objectPrefix, geomNames) {
  const wanted = new Set(geomNames);
  const hits = new Set();
  for (const contact of observation?.contacts || []) {
    const a = contact?.geom1Name;
    const b = contact?.geom2Name;
    if (typeof a !== 'string' || typeof b !== 'string') continue;
    if (a.startsWith(objectPrefix) && wanted.has(b)) hits.add(b);
    else if (b.startsWith(objectPrefix) && wanted.has(a)) hits.add(a);
  }
  return hits;
}
function objectTouchesGeom(observation, objectPrefix, geomName) {
  return (observation?.contacts || []).some((contact) => {
    const a = contact?.geom1Name;
    const b = contact?.geom2Name;
    if (typeof a !== 'string' || typeof b !== 'string') return false;
    return (a.startsWith(objectPrefix) && b === geomName) || (b.startsWith(objectPrefix) && a === geomName);
  });
}

function tiltRad(quaternionWxyz) {
  const q = Array.isArray(quaternionWxyz) ? quaternionWxyz.map(Number) : [1, 0, 0, 0];
  const [w, x, y, z] = q;
  // Angle between the body +Z axis and world +Z.
  const upZ = 1 - 2 * (x * x + y * y);
  return Math.acos(Math.max(-1, Math.min(1, upZ)));
}

export class LeKiwiCourierEvaluator {
  constructor({ goal = LEKIWI_COURIER_SCENE.taskGoal } = {}) {
    this.goal = structuredClone(goal);
    this.reset();
  }

  reset(initialObservation = null) {
    this.state = {
      initialBeakerM: null,
      lastBeakerM: null,
      initialBaseM: null,
      lastBaseM: null,
      basePathLengthM: 0,
      maxBaseTravelFromHomeM: 0,
      currentFixedJawContact: false,
      currentMovingJawContact: false,
      currentGrasp: false,
      currentAnyJawContact: false,
      currentSupportContact: false,
      graspSeen: false,
      graspTimeSeconds: null,
      liftSeen: false,
      liftTimeSeconds: null,
      maxBeakerZM: Number.NEGATIVE_INFINITY,
      carryAnchorM: null,
      maxHeldTravelM: 0,
      carrySeen: false,
      carriedWhileBaseMovedM: 0,
      supportWhileHeldSeen: false,
      supportWhileHeldTimeSeconds: null,
      releaseSeen: false,
      releaseInvalidations: 0,
      releaseTimeSeconds: null,
      settleAnchorM: null,
      settleAnchorTimeSeconds: null,
      settleDriftM: null,
      settled: false,
      settleTimeSeconds: null,
      serviceStopReached: false,
      serviceStopTimeSeconds: null,
      homeReturned: false,
      homeTimeSeconds: null,
      restrictedViolation: false,
      payloadLost: false,
      lastSimulationTimeSeconds: null,
      observationCount: 0,
    };
    if (initialObservation) this.observe(initialObservation);
    return this.snapshot();
  }

  observe(observation) {
    const t = Number(observation?.simulationTimeSeconds);
    if (!Number.isFinite(t)) return this.snapshot();
    const s = this.state;
    const goal = this.goal;
    s.observationCount += 1;

    const base = observation?.bodies?.[goal.baseBodyId];
    const basePos = vec3(base?.positionM);
    if (basePos) {
      if (!s.initialBaseM) s.initialBaseM = [...basePos];
      if (s.lastBaseM) s.basePathLengthM += horizontal(basePos, s.lastBaseM);
      s.lastBaseM = [...basePos];
      s.maxBaseTravelFromHomeM = Math.max(s.maxBaseTravelFromHomeM, horizontal(basePos, [goal.homeXYM[0], goal.homeXYM[1], 0]));
      const baseSpeed = norm3(base?.linearVelocityMS);
      const baseYawRate = Math.abs(Number(base?.angularVelocityRadS?.[2] ?? 0));
      if (horizontal(basePos, [goal.restrictedStopXYM[0], goal.restrictedStopXYM[1], 0]) <= Number(goal.restrictedStopRadiusM)) s.restrictedViolation = true;
      const stopped = baseSpeed <= Number(goal.stoppedSpeedMS) && baseYawRate <= 0.15;
      if (!s.serviceStopReached && stopped && horizontal(basePos, [goal.serviceStopXYM[0], goal.serviceStopXYM[1], 0]) <= Number(goal.serviceStopToleranceM)) {
        s.serviceStopReached = true;
        s.serviceStopTimeSeconds = t;
      }
      if (s.settled && stopped && horizontal(basePos, [goal.homeXYM[0], goal.homeXYM[1], 0]) <= Number(goal.homeToleranceM)) {
        if (!s.homeReturned) { s.homeReturned = true; s.homeTimeSeconds = t; }
      }
    }

    const beaker = observation?.bodies?.[goal.objectId];
    const beakerPos = vec3(beaker?.positionM);
    if (!beakerPos) { s.lastSimulationTimeSeconds = t; return this.snapshot(); }
    if (!s.initialBeakerM) s.initialBeakerM = [...beakerPos];
    s.lastBeakerM = [...beakerPos];
    s.maxBeakerZM = Math.max(s.maxBeakerZM, beakerPos[2]);
    if (beakerPos[2] < Number(s.initialBeakerM[2]) - 0.05) s.payloadLost = true;

    const previousJawContact = s.currentAnyJawContact;
    const fixedHits = contactsWith(observation, goal.objectGeomPrefix, goal.fixedJawGeoms);
    const movingHits = contactsWith(observation, goal.objectGeomPrefix, goal.movingJawGeoms);
    s.currentFixedJawContact = fixedHits.size > 0;
    s.currentMovingJawContact = movingHits.size > 0;
    s.currentGrasp = s.currentFixedJawContact && s.currentMovingJawContact;
    s.currentAnyJawContact = s.currentFixedJawContact || s.currentMovingJawContact;
    s.currentSupportContact = objectTouchesGeom(observation, goal.objectGeomPrefix, goal.supportGeom);
    if (s.currentGrasp && !s.graspSeen) { s.graspSeen = true; s.graspTimeSeconds = t; }

    const liftThreshold = Number(s.initialBeakerM[2]) + Number(goal.liftClearanceM);
    if (s.graspSeen && s.currentGrasp && !s.currentSupportContact && beakerPos[2] > liftThreshold) {
      if (!s.liftSeen) { s.liftSeen = true; s.liftTimeSeconds = t; }
      if (!s.carrySeen && !s.carryAnchorM) s.carryAnchorM = [...beakerPos];
    }
    if (!s.carrySeen && s.carryAnchorM) {
      if (!s.currentAnyJawContact) { s.carryAnchorM = null; s.maxHeldTravelM = 0; }
      else if (s.currentGrasp) {
        const travel = horizontal(beakerPos, s.carryAnchorM);
        s.maxHeldTravelM = Math.max(s.maxHeldTravelM, travel);
        if (travel >= Number(goal.carryHorizontalM)) s.carrySeen = true;
      }
    }

    const inDelivery = Math.abs(beakerPos[0] - goal.deliveryXYM[0]) <= Number(goal.deliveryHalfExtentsXYM[0])
      && Math.abs(beakerPos[1] - goal.deliveryXYM[1]) <= Number(goal.deliveryHalfExtentsXYM[1]);
    const atSupportHeight = Math.abs(beakerPos[2] - Number(s.initialBeakerM[2])) <= Number(goal.supportZToleranceM);

    if (!s.supportWhileHeldSeen && s.carrySeen && s.currentGrasp && s.currentSupportContact && inDelivery) {
      s.supportWhileHeldSeen = true;
      s.supportWhileHeldTimeSeconds = t;
    }

    // A release stays credited only while it remains a release: any later jaw re-contact
    // invalidates it, and a later clean release can establish new evidence.
    if (s.releaseSeen && s.currentAnyJawContact) {
      s.releaseSeen = false;
      s.releaseInvalidations += 1;
      s.releaseTimeSeconds = null;
      s.settleAnchorM = null;
      s.settleAnchorTimeSeconds = null;
      s.settled = false;
      s.settleTimeSeconds = null;
      s.homeReturned = false;
      s.homeTimeSeconds = null;
    }
    if (!s.releaseSeen && s.supportWhileHeldSeen && previousJawContact && !s.currentAnyJawContact && s.currentSupportContact && inDelivery) {
      s.releaseSeen = true;
      s.releaseTimeSeconds = t;
    }

    const linearSpeed = norm3(beaker?.linearVelocityMS);
    const angularSpeed = norm3(beaker?.angularVelocityRadS);
    const tilt = tiltRad(beaker?.quaternionWxyz);
    const settleEligible = s.releaseSeen && s.currentSupportContact && !s.currentAnyJawContact && inDelivery && atSupportHeight
      && linearSpeed <= Number(goal.maxSettleLinearSpeedMS)
      && angularSpeed <= Number(goal.maxSettleAngularSpeedRadS)
      && tilt <= Number(goal.maxSettleTiltRad);
    if (!settleEligible) {
      s.settleAnchorM = null;
      s.settleAnchorTimeSeconds = null;
      s.settleDriftM = null;
      s.settled = false;
      s.settleTimeSeconds = null;
    } else if (!s.settleAnchorM) {
      s.settleAnchorM = [...beakerPos];
      s.settleAnchorTimeSeconds = t;
      s.settleDriftM = 0;
    } else {
      const drift = distance3(beakerPos, s.settleAnchorM);
      if (drift > Number(goal.maxSettleDriftM)) {
        s.settleAnchorM = [...beakerPos];
        s.settleAnchorTimeSeconds = t;
        s.settleDriftM = 0;
        s.settled = false;
      } else {
        s.settleDriftM = Math.max(Number(s.settleDriftM || 0), drift);
        if (t - s.settleAnchorTimeSeconds + 1e-12 >= Number(goal.settleSeconds)) {
          if (!s.settled) s.settleTimeSeconds = t;
          s.settled = true;
        }
      }
    }

    s.lastSimulationTimeSeconds = t;
    return this.snapshot();
  }

  snapshot() {
    const s = this.state;
    const goal = this.goal;
    const restXYErrorM = s.lastBeakerM ? Math.hypot(s.lastBeakerM[0] - goal.deliveryXYM[0], s.lastBeakerM[1] - goal.deliveryXYM[1]) : null;
    const baseTravelSufficient = s.basePathLengthM >= Number(goal.baseTravelM);
    const causalOrder = Boolean(
      s.serviceStopTimeSeconds != null && s.graspTimeSeconds != null && s.liftTimeSeconds != null
      && s.supportWhileHeldTimeSeconds != null && s.releaseTimeSeconds != null && s.settleTimeSeconds != null
      && s.homeTimeSeconds != null
      && s.serviceStopTimeSeconds <= s.graspTimeSeconds
      && s.graspTimeSeconds <= s.liftTimeSeconds
      && s.liftTimeSeconds <= s.supportWhileHeldTimeSeconds
      && s.supportWhileHeldTimeSeconds < s.releaseTimeSeconds
      && s.releaseTimeSeconds < s.settleTimeSeconds
      && s.settleTimeSeconds <= s.homeTimeSeconds,
    );
    const success = Boolean(
      !s.restrictedViolation && !s.payloadLost
      && s.serviceStopReached && s.graspSeen && s.liftSeen && s.carrySeen
      && s.supportWhileHeldSeen && s.releaseSeen && s.settled && s.homeReturned
      && baseTravelSufficient && causalOrder
      && s.currentSupportContact && !s.currentAnyJawContact
      && restXYErrorM != null && restXYErrorM <= Number(goal.restXYToleranceM),
    );
    return Object.freeze({
      task: 'LeKiwi Physical Beaker Courier',
      physical: true,
      success,
      causalOrder,
      serviceStopReached: s.serviceStopReached,
      graspSeen: s.graspSeen,
      liftSeen: s.liftSeen,
      carrySeen: s.carrySeen,
      supportWhileHeldSeen: s.supportWhileHeldSeen,
      releaseSeen: s.releaseSeen,
      releaseInvalidations: s.releaseInvalidations,
      settled: s.settled,
      homeReturned: s.homeReturned,
      restrictedViolation: s.restrictedViolation,
      payloadLost: s.payloadLost,
      currentGrasp: s.currentGrasp,
      currentJawContact: s.currentAnyJawContact,
      currentSupportContact: s.currentSupportContact,
      basePathLengthM: s.basePathLengthM,
      maxBaseTravelFromHomeM: s.maxBaseTravelFromHomeM,
      baseTravelSufficient,
      maxHeldTravelM: s.maxHeldTravelM,
      maxBeakerZM: Number.isFinite(s.maxBeakerZM) ? s.maxBeakerZM : null,
      liftHeightM: Number.isFinite(s.maxBeakerZM) && s.initialBeakerM ? s.maxBeakerZM - s.initialBeakerM[2] : null,
      settleDriftM: s.settleDriftM,
      restXYErrorM,
      initialBeakerM: s.initialBeakerM ? [...s.initialBeakerM] : null,
      finalBeakerM: s.lastBeakerM ? [...s.lastBeakerM] : null,
      finalBaseM: s.lastBaseM ? [...s.lastBaseM] : null,
      serviceStopTimeSeconds: s.serviceStopTimeSeconds,
      graspTimeSeconds: s.graspTimeSeconds,
      liftTimeSeconds: s.liftTimeSeconds,
      supportWhileHeldTimeSeconds: s.supportWhileHeldTimeSeconds,
      releaseTimeSeconds: s.releaseTimeSeconds,
      settleTimeSeconds: s.settleTimeSeconds,
      homeTimeSeconds: s.homeTimeSeconds,
      observationCount: s.observationCount,
      evidence: 'MuJoCo base and free-beaker body poses/velocities, named fixed-jaw and moving-jaw contact pairs, worktop support contact while still physically gripped, a sustained release transition invalidated by any later jaw re-contact, contact-supported simulation-time dwell, measured base path length, and an observed stopped return to the home region. No weld, parenting, attachment flag, snap, teleport, upright lock, timer, command echo, or program-reported success is used.',
    });
  }
}

/** Base-only physical gate helper shared by the base scenes and the browser/native gates. */
export function basePoseFromObservation(observation, bodyId = 'lekiwi_base') {
  const body = observation?.bodies?.[bodyId];
  const position = vec3(body?.positionM);
  if (!position) return null;
  const linear = vec3(body?.linearVelocityMS) || [0, 0, 0];
  const angular = vec3(body?.angularVelocityRadS) || [0, 0, 0];
  const yawRad = yawFromQuaternion(body?.quaternionWxyz || [1, 0, 0, 0]);
  return Object.freeze({
    xM: position[0], yM: position[1], zM: position[2], yawRad,
    speedMS: Math.hypot(linear[0], linear[1]),
    yawRateRadS: angular[2],
    linearVelocityMS: Object.freeze([...linear]),
    tiltRad: tiltRad(body?.quaternionWxyz),
  });
}

/** Body-frame displacement between two observed base poses. */
export function bodyFrameDisplacement(startPose, endPose) {
  const dx = endPose.xM - startPose.xM;
  const dy = endPose.yM - startPose.yM;
  const cos = Math.cos(startPose.yawRad);
  const sin = Math.sin(startPose.yawRad);
  return Object.freeze({
    forwardM: dx * cos + dy * sin,
    leftM: -dx * sin + dy * cos,
    yawRad: wrapAngle(endPose.yawRad - startPose.yawRad),
  });
}
