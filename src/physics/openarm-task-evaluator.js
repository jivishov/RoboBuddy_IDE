import { OPENARM_V2_PHASE5A_SCENE } from './openarm-scene.js';

function finiteVec3(value) {
  return Array.isArray(value) && value.length === 3 && value.every((item) => Number.isFinite(Number(item)));
}
function norm3(value) { return finiteVec3(value) ? Math.hypot(...value.map(Number)) : Number.POSITIVE_INFINITY; }
function distance3(a, b) { return Math.hypot(Number(a[0]) - Number(b[0]), Number(a[1]) - Number(b[1]), Number(a[2]) - Number(b[2])); }
function horizontalDistance(a, b) { return Math.hypot(Number(a[0]) - Number(b[0]), Number(a[1]) - Number(b[1])); }
function contactPair(observation, first, second) {
  return (observation?.contacts || []).some((contact) => {
    const a = contact?.geom1Name;
    const b = contact?.geom2Name;
    const matches = (name, spec) => typeof name === 'string' && (spec.endsWith('_') ? name.startsWith(spec) : name === spec);
    const force = contact.normalForceN;
    const valid = Number.isFinite(force) && force > 0.01 && Number.isFinite(contact.distanceM) && contact.distanceM >= -0.002;
    return valid && ((matches(a, first) && matches(b, second)) || (matches(a, second) && matches(b, first)));
  });
}
function objectTouchesGeom(observation, objectGeoms, otherGeom) {
  return objectGeoms.some((objectGeom) => contactPair(observation, objectGeom, otherGeom));
}
function objectPosition(observation, objectId) {
  const value = observation?.bodies?.[objectId]?.positionM;
  return finiteVec3(value) ? value.map(Number) : null;
}
function bodyVelocity(observation, objectId, key) {
  const value = observation?.bodies?.[objectId]?.[key];
  return finiteVec3(value) ? value.map(Number) : null;
}
function endEffectorPosition(observation, side) {
  const value = observation?.bodies?.[`openarm_${side}_ee_base_link`]?.positionM;
  return finiteVec3(value) ? value.map(Number) : null;
}

function newObjectState(spec) {
  return {
    spec: structuredClone(spec),
    initialPositionM: null,
    lastPositionM: null,
    currentInnerContact: false,
    currentOuterContact: false,
    currentBilateralContact: false,
    currentGripperContact: false,
    currentSupportContact: false,
    innerContactSeen: false,
    outerContactSeen: false,
    graspSeen: false,
    graspCandidateTimeSeconds: null,
    maximumBilateralDurationSeconds: 0,
    liftSeen: false,
    carrySeen: false,
    supportWhileHeldSeen: false,
    releaseSeen: false,
    releaseInvalidationCount: 0,
    settled: false,
    retreated: false,
    carryAnchorPositionM: null,
    maxHeldHorizontalTravelM: 0,
    maxZM: Number.NEGATIVE_INFINITY,
    settleCandidatePositionM: null,
    settleCandidateTimeSeconds: null,
    settleDriftM: null,
    supportWhileHeldTimeSeconds: null,
    releaseTimeSeconds: null,
    releaseEePositionM: null,
    settleTimeSeconds: null,
    settledEePositionM: null,
    retreatTimeSeconds: null,
    contactObservationCount: 0,
    bilateralContactObservationCount: 0,
    supportObservationCount: 0,
  };
}

function invalidateReleaseEvidence(state) {
  state.releaseSeen = false;
  state.releaseInvalidationCount += 1;
  state.releaseTimeSeconds = null;
  state.releaseEePositionM = null;
  state.settleCandidatePositionM = null;
  state.settleCandidateTimeSeconds = null;
  state.settleDriftM = null;
  state.settleTimeSeconds = null;
  state.settled = false;
  state.settledEePositionM = null;
  state.retreatTimeSeconds = null;
  state.retreated = false;
}

export class OpenArmBimanualStackEvaluator {
  constructor({ goal = OPENARM_V2_PHASE5A_SCENE.taskGoal } = {}) {
    this.goal = structuredClone(goal);
    this.reset();
  }

  reset(initialObservation = null) {
    this.objects = {
      flask: newObjectState(this.goal.flask),
      beaker: newObjectState(this.goal.beaker),
    };
    this.orderViolation = false;
    this.maximumPenetrationM = 0;
    this.excessivePenetrationSeen = false;
    this.palmContactSeen = false;
    this.lastSimulationTimeSeconds = null;
    this.lastObservation = null;
    if (initialObservation) this.observe(initialObservation);
    return this.snapshot();
  }

  observe(observation) {
    const simulationTimeSeconds = Number(observation?.simulationTimeSeconds);
    if (!Number.isFinite(simulationTimeSeconds)) return this.snapshot();
    // Re-reading the same state cannot manufacture dwell evidence.
    if (this.lastSimulationTimeSeconds != null && simulationTimeSeconds <= this.lastSimulationTimeSeconds) return this.snapshot();
    if (this.lastSimulationTimeSeconds != null && simulationTimeSeconds - this.lastSimulationTimeSeconds > 0.08) {
      for (const state of Object.values(this.objects)) { state.graspCandidateTimeSeconds = null; state.settleCandidateTimeSeconds = null; }
    }
    for (const contact of observation.contacts || []) {
      const names = [contact.geom1Name || '', contact.geom2Name || ''];
      if (!names.some(n => /^(?:flask_|beaker_)/.test(n))) continue;
      this.maximumPenetrationM = Math.max(this.maximumPenetrationM, -(Number(contact.distanceM) || 0));
      if (Number(contact.distanceM) < -Number(this.goal.maximumPenetrationM || .002)) this.excessivePenetrationSeen = true;
      if (names.some(n => n.startsWith('ee_base_link_')) && contact.normalForceN > .01) this.palmContactSeen = true;
    }
    this.#observeObject('flask', observation, simulationTimeSeconds);
    this.#observeObject('beaker', observation, simulationTimeSeconds);

    const left = this.objects.flask;
    const right = this.objects.beaker;
    if ((right.graspSeen || right.liftSeen || right.carrySeen) && !left.retreated) this.orderViolation = true;

    this.lastSimulationTimeSeconds = simulationTimeSeconds;
    this.lastObservation = observation;
    return this.snapshot();
  }

  #observeObject(key, observation, simulationTimeSeconds) {
    const state = this.objects[key];
    const spec = state.spec;
    const positionM = objectPosition(observation, spec.objectId);
    if (!positionM) return;
    if (!state.initialPositionM) state.initialPositionM = [...positionM];
    state.lastPositionM = [...positionM];
    state.maxZM = Math.max(state.maxZM, positionM[2]);

    const previousGripperContact = state.currentGripperContact;
    const inner = objectTouchesGeom(observation, spec.objectGeoms, spec.gripperGeoms[0]);
    const outer = objectTouchesGeom(observation, spec.objectGeoms, spec.gripperGeoms[1]);
    const bilateralContact = inner && outer;
    const gripperContact = inner || outer;
    const supportContact = objectTouchesGeom(observation, spec.objectGeoms, spec.supportGeom);
    state.currentInnerContact = inner;
    state.currentOuterContact = outer;
    state.currentBilateralContact = bilateralContact;
    state.currentGripperContact = gripperContact;
    state.currentSupportContact = supportContact;
    if (inner) state.innerContactSeen = true;
    if (outer) state.outerContactSeen = true;
    if (gripperContact) state.contactObservationCount += 1;
    if (bilateralContact) {
      state.bilateralContactObservationCount += 1;
      state.graspCandidateTimeSeconds ??= simulationTimeSeconds;
      const duration = simulationTimeSeconds - state.graspCandidateTimeSeconds;
      state.maximumBilateralDurationSeconds = Math.max(state.maximumBilateralDurationSeconds, duration);
      if (duration + 1e-9 >= Number(this.goal.minimumGraspSeconds || .06)) state.graspSeen = true;
    }
    if (!bilateralContact) state.graspCandidateTimeSeconds = null;
    if (supportContact) state.supportObservationCount += 1;

    const liftThreshold = Number(spec.initialBodyZM) + Number(this.goal.liftClearanceM);
    if (state.graspSeen && bilateralContact && positionM[2] > liftThreshold) {
      state.liftSeen = true;
      if (!state.carrySeen && !state.carryAnchorPositionM) state.carryAnchorPositionM = [...positionM];
    }

    if (!state.carrySeen && state.carryAnchorPositionM) {
      if (!gripperContact) {
        state.carryAnchorPositionM = null;
        state.maxHeldHorizontalTravelM = 0;
      } else if (bilateralContact) {
        const heldTravel = horizontalDistance(positionM, state.carryAnchorPositionM);
        state.maxHeldHorizontalTravelM = Math.max(state.maxHeldHorizontalTravelM, heldTravel);
        if (heldTravel >= Number(this.goal.carryHorizontalM)) state.carrySeen = true;
      }
    }

    const [targetX, targetY] = spec.targetCenterXYM;
    const [halfX, halfY] = spec.targetHalfExtentsXYM;
    const inTarget = Math.abs(positionM[0] - targetX) <= halfX && Math.abs(positionM[1] - targetY) <= halfY;
    const nearSupportHeight = Math.abs(positionM[2] - Number(spec.initialBodyZM)) <= 0.015;

    if (!state.supportWhileHeldSeen && state.carrySeen && bilateralContact && supportContact && inTarget && nearSupportHeight) {
      state.supportWhileHeldSeen = true;
      state.supportWhileHeldTimeSeconds = simulationTimeSeconds;
    }

    // A release is only causal evidence while it remains a release. If the
    // gripper touches the vessel again, a prior transient contact loss cannot
    // remain credited toward settle/retreat success. A later clean release can
    // establish new evidence after the re-contact.
    if (state.releaseSeen && gripperContact) invalidateReleaseEvidence(state);

    if (!state.releaseSeen
      && state.supportWhileHeldSeen
      && previousGripperContact
      && !gripperContact
      && supportContact
      && inTarget
      && nearSupportHeight) {
      state.releaseSeen = true;
      state.releaseTimeSeconds = simulationTimeSeconds;
      const ee = endEffectorPosition(observation, spec.side);
      if (ee) state.releaseEePositionM = [...ee];
    }

    const linearSpeed = norm3(bodyVelocity(observation, spec.objectId, 'linearVelocityMS'));
    const angularSpeed = norm3(bodyVelocity(observation, spec.objectId, 'angularVelocityRadS'));
    const quat = observation?.bodies?.[spec.objectId]?.quaternionWxyz;
    const upright = Array.isArray(quat) && quat.length === 4 && quat.every(Number.isFinite)
      && (1 - 2 * (quat[1] ** 2 + quat[2] ** 2)) >= Math.cos(Math.PI / 12);
    const settleEligible = upright && state.releaseSeen
      && supportContact
      && !gripperContact
      && inTarget
      && nearSupportHeight
      && linearSpeed <= Number(this.goal.maxSettleLinearSpeedMS)
      && angularSpeed <= Number(this.goal.maxSettleAngularSpeedRadS);

    if (!settleEligible) {
      state.settleCandidatePositionM = null;
      state.settleCandidateTimeSeconds = null;
      state.settleDriftM = null;
      state.settled = false;
      state.settledEePositionM = null;
      state.retreated = false;
      state.retreatTimeSeconds = null;
    } else if (!state.settleCandidatePositionM || state.settleCandidateTimeSeconds == null) {
      state.settleCandidatePositionM = [...positionM];
      state.settleCandidateTimeSeconds = simulationTimeSeconds;
      state.settleDriftM = 0;
      state.settled = false;
      state.settledEePositionM = null;
      state.retreated = false;
      state.retreatTimeSeconds = null;
    } else {
      const drift = distance3(positionM, state.settleCandidatePositionM);
      if (drift > Number(this.goal.maxSettleDriftM)) {
        state.settleCandidatePositionM = [...positionM];
        state.settleCandidateTimeSeconds = simulationTimeSeconds;
        state.settleDriftM = 0;
        state.settled = false;
        state.settledEePositionM = null;
        state.retreated = false;
        state.retreatTimeSeconds = null;
      } else {
        state.settleDriftM = Math.max(Number(state.settleDriftM || 0), drift);
        if (simulationTimeSeconds - state.settleCandidateTimeSeconds + 1e-12 >= Number(this.goal.settleSeconds)) {
          if (!state.settled) {
            state.settleTimeSeconds = simulationTimeSeconds;
            const ee = endEffectorPosition(observation, spec.side);
            state.settledEePositionM = ee ? [...ee] : null;
          }
          state.settled = true;
        }
      }
    }

    if (state.settled && supportContact && !gripperContact && state.settledEePositionM) {
      const ee = endEffectorPosition(observation, spec.side);
      if (ee && distance3(ee, state.settledEePositionM) >= Number(this.goal.retreatDistanceM)) {
        state.retreated = true;
        state.retreatTimeSeconds ??= simulationTimeSeconds;
      }
    }
  }

  snapshot() {
    const snapshotObject = (state) => Object.freeze({
      objectId: state.spec.objectId,
      side: state.spec.side,
      graspSeen: state.graspSeen,
      maximumBilateralDurationSeconds: state.maximumBilateralDurationSeconds,
      innerContactSeen: state.innerContactSeen,
      outerContactSeen: state.outerContactSeen,
      currentBilateralContact: state.currentBilateralContact,
      liftSeen: state.liftSeen,
      carrySeen: state.carrySeen,
      supportWhileHeldSeen: state.supportWhileHeldSeen,
      releaseSeen: state.releaseSeen,
      releaseInvalidationCount: state.releaseInvalidationCount,
      settled: state.settled,
      retreated: state.retreated,
      currentGripperContact: state.currentGripperContact,
      currentSupportContact: state.currentSupportContact,
      contactObservationCount: state.contactObservationCount,
      bilateralContactObservationCount: state.bilateralContactObservationCount,
      supportObservationCount: state.supportObservationCount,
      initialPositionM: state.initialPositionM ? [...state.initialPositionM] : null,
      finalPositionM: state.lastPositionM ? [...state.lastPositionM] : null,
      maxZM: Number.isFinite(state.maxZM) ? state.maxZM : null,
      maxHeldHorizontalTravelM: state.maxHeldHorizontalTravelM,
      settleEvidenceDurationSeconds: state.settleCandidateTimeSeconds == null || this.lastSimulationTimeSeconds == null
        ? 0 : Math.max(0, this.lastSimulationTimeSeconds - state.settleCandidateTimeSeconds),
      settleDriftM: state.settleDriftM,
      supportWhileHeldTimeSeconds: state.supportWhileHeldTimeSeconds,
      releaseTimeSeconds: state.releaseTimeSeconds,
      releaseEePositionM: state.releaseEePositionM ? [...state.releaseEePositionM] : null,
      settleTimeSeconds: state.settleTimeSeconds,
      settledEePositionM: state.settledEePositionM ? [...state.settledEePositionM] : null,
      retreatTimeSeconds: state.retreatTimeSeconds,
    });
    const flask = snapshotObject(this.objects.flask);
    const beaker = snapshotObject(this.objects.beaker);
    const success = Boolean(
      !this.orderViolation && !this.excessivePenetrationSeen && !this.palmContactSeen
      && flask.graspSeen && flask.liftSeen && flask.carrySeen && flask.supportWhileHeldSeen && flask.releaseSeen && flask.settled && flask.retreated
      && beaker.graspSeen && beaker.liftSeen && beaker.carrySeen && beaker.supportWhileHeldSeen && beaker.releaseSeen && beaker.settled && beaker.retreated
      && flask.currentSupportContact && beaker.currentSupportContact
      && !flask.currentGripperContact && !beaker.currentGripperContact
    );
    return Object.freeze({
      maximumPenetrationM: this.maximumPenetrationM,
      excessivePenetrationSeen: this.excessivePenetrationSeen,
      palmContactSeen: this.palmContactSeen,
      task: 'OpenArm V2 Physical Bimanual Heater and Ring-Stand Stack',
      physical: true,
      success,
      orderViolation: this.orderViolation,
      flask,
      beaker,
      evidence: 'MuJoCo body motion, free-joint velocities, simultaneous named bilateral fingertip/vessel contact, intended support contact while still physically held, a sustained observed release transition that is invalidated by any later re-contact, contact-supported simulation-time dwell, and measured post-settle end-effector retreat. No object weld, parenting, snap, teleport or synthetic success event is used. Primitive collision surrogates and dry workcell parameters are simulator estimates, not hardware calibration.',
    });
  }
}
