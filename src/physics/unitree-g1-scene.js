import { PHYSICS_BACKEND_API_VERSION } from './backend-contract.js';
import {
  UNITREE_G1_EXTERNAL_OBJECT, UNITREE_G1_FREEBASE_DROP_PACKAGE, UNITREE_G1_FREEBASE_PACKAGE,
  UNITREE_G1_MOUNTED_BLOCKED_PACKAGE, UNITREE_G1_MOUNTED_PACKAGE, UNITREE_G1_STAND_PELVIS_Z_M,
} from './unitree-g1-model-package.js';
import { G1_EFFORT_LIMIT_NM, G1_JOINT_ORDER } from './unitree-g1-source-audit.js';
import { G1_CONTROLLERS, G1_OBSERVATION_INTERVAL_SECONDS, UNITREE_FIXSTAND_RAMP_SECONDS } from './unitree-g1-controller.js';

const scene = (id, revision, modelPackage, { taskGoal = null } = {}) => Object.freeze({
  schemaVersion: PHYSICS_BACKEND_API_VERSION,
  id,
  revision,
  robotId: modelPackage.robotId,
  modelPackage: modelPackage.id,
  legacyTaskId: null,
  physics: Object.freeze({ ...modelPackage.physics }),
  fixtures: Object.freeze((modelPackage.sceneConstraints?.fixtures || []).map((fixtureId) => Object.freeze({ id: fixtureId }))),
  objects: Object.freeze((modelPackage.sceneConstraints?.objects || []).map((objectId) => Object.freeze({ id: objectId }))),
  controllers: Object.freeze([...modelPackage.controllers]),
  taskGoal,
});

/**
 * The free-base standing envelope.
 *
 * Every threshold is fixed from the model geometry and the source posture, and none of them was
 * widened afterwards to make a run pass:
 *   - the height band is +/- 50 mm around the source standing pelvis height, roughly one knee-flex
 *     degree of freedom; a controller that sags or springs further is not holding a posture;
 *   - 0.15 rad of tilt is about 8.6 deg, the point at which a centre of mass 0.67 m above the ankle
 *     leaves the 0.17 m source foot support polygon;
 *   - the evaluated interval begins only after the controller's source 2 s ramp has completed.
 */
export const UNITREE_G1_STAND_GATE = Object.freeze({
  evaluationStartSeconds: UNITREE_FIXSTAND_RAMP_SECONDS,
  evaluationSeconds: 6,
  maxObservationGapSeconds: G1_OBSERVATION_INTERVAL_SECONDS + 1e-9,
  pelvisHeightRangeM: Object.freeze([UNITREE_G1_STAND_PELVIS_Z_M - 0.05, UNITREE_G1_STAND_PELVIS_Z_M + 0.05]),
  maxPelvisTiltRad: 0.15,
  maxHorizontalDriftM: 0.10,
  maxTerminalLinearSpeedMS: 0.05,
  maxTerminalAngularSpeedRadS: 0.20,
  maxActuatorEffortFraction: 0.90,
  requireBothFeetSupported: true,
  forbidNonFootGroundContact: true,
  forbidExternalSupport: true,
});

export const UNITREE_G1_MOUNTED_SCENE = scene('p5d-unitree-g1-mounted', 'p5d-unitree-g1-mounted-v1', UNITREE_G1_MOUNTED_PACKAGE);
export const UNITREE_G1_MOUNTED_BLOCKED_SCENE = scene('p5d-unitree-g1-mounted-blocked', 'p5d-unitree-g1-mounted-blocked-v1', UNITREE_G1_MOUNTED_BLOCKED_PACKAGE);
export const UNITREE_G1_FREEBASE_DROP_SCENE = scene('p5d-unitree-g1-freebase-drop', 'p5d-unitree-g1-freebase-drop-v1', UNITREE_G1_FREEBASE_DROP_PACKAGE);
export const UNITREE_G1_FREEBASE_SCENE = scene('p5d-unitree-g1-freebase', 'p5d-unitree-g1-freebase-v1', UNITREE_G1_FREEBASE_PACKAGE, {
  taskGoal: Object.freeze({
    type: 'unitree-g1-physical-standing',
    controllerId: G1_CONTROLLERS.STAND,
    externalObjectId: UNITREE_G1_EXTERNAL_OBJECT,
    gate: UNITREE_G1_STAND_GATE,
    evidence: 'MuJoCo root pose and velocity plus named foot, body, self and external-object contacts',
    syntheticSuccessEvents: false,
  }),
});

export const UNITREE_G1_SCENES = Object.freeze({
  mounted: UNITREE_G1_MOUNTED_SCENE,
  mountedBlocked: UNITREE_G1_MOUNTED_BLOCKED_SCENE,
  freebase: UNITREE_G1_FREEBASE_SCENE,
  freebaseDrop: UNITREE_G1_FREEBASE_DROP_SCENE,
});

const finiteVector = (vector, length) => Array.isArray(vector) && vector.length === length && vector.every(Number.isFinite);
const norm = (vector) => finiteVector(vector, 3) ? Math.hypot(...vector) : Infinity;
const supportingContacts = (contacts) => (contacts || []).filter((contact) => Number.isFinite(contact.normalForceN) && contact.normalForceN > 1e-6).length;

// Missing data is not evidence of zero velocity/effort or of a free, gravity-loaded base.
function validStandObservation(observation, seconds) {
  return Number.isFinite(seconds) && seconds >= 0 && observation.root?.mode === 'free-base'
    && observation.root.free === true && observation.contactsReadable === true
    && finiteVector(observation.root.positionM, 3) && finiteVector(observation.root.quaternionWxyz, 4)
    && Number.isFinite(observation.root.tiltRad) && observation.root.tiltRad >= 0
    && finiteVector(observation.root.linearVelocityMS, 3) && finiteVector(observation.root.angularVelocityRadS, 3)
    && finiteVector(observation.engine?.gravity, 3) && observation.engine.gravity[2] < -9
    && G1_JOINT_ORDER.every((id, index) => Number.isFinite(observation.joints?.[id]?.effortNm)
      && observation.joints[id].effortLimitNm === G1_EFFORT_LIMIT_NM[index])
    && ['leftFootFloor', 'rightFootFloor', 'otherBodyFloor', 'robotExternalObject', 'robotFixture'].every((key) =>
      Array.isArray(observation.contactClasses?.[key]) && observation.contactClasses[key].every((entry) =>
        Array.isArray(entry.geoms) && entry.geoms.length === 2 && Number.isFinite(entry.normalForceN)));
}

/**
 * Free-base standing evaluator.
 *
 * It reads only MuJoCo observations. It never sees a command, it never sees a controller's opinion
 * of its own success, and it never treats a contact count as support: a foot is supporting the
 * robot only when a named foot geom has a positive normal contact force against the named floor geom.
 */
export class UnitreeG1StandEvaluator {
  constructor(gate = UNITREE_G1_STAND_GATE) {
    this.gate = gate;
    this.reset();
  }

  reset() {
    this.samples = 0;
    this.windowSamples = 0;
    this.startedSeconds = null;
    this.lastSeconds = null;
    this.pelvisHeightMinM = Infinity;
    this.pelvisHeightMaxM = -Infinity;
    this.maxTiltRad = 0;
    this.maxHorizontalDriftM = 0;
    this.maxNonFootGroundContacts = 0;
    this.maxExternalSupportContacts = 0;
    this.worstActuatorEffortFraction = 0;
    this.bothFeetSupportedThroughout = true;
    this.selfContactSeen = false;
    this.externalObjectContactSeen = false;
    this.everLeftUprightBand = false;
    this.terminal = null;
    this.actuationDisabledSeen = false;
    this.controllerId = null;
    this.controllerEngagedAtSeconds = null;
    this.observationsValid = true;
    this.observationsContinuous = true;
    this.lastWindowSeconds = null;
  }

  /**
   * The evaluated interval opens one controller ramp after the standing controller was engaged,
   * not one ramp after the simulation clock started. Anchoring it to absolute time would let the
   * tail of the ramp count as held posture whenever a caller settled the robot first.
   */
  evaluationOpensAtSeconds() {
    const engaged = this.controllerEngagedAtSeconds;
    return (engaged == null ? 0 : engaged) + this.gate.evaluationStartSeconds;
  }

  observe(observation) {
    if (!observation?.root) { this.observationsValid = false; return; }
    const seconds = observation.simulationTimeSeconds ?? observation.simulationTime;
    const classes = observation.contactClasses || {};
    this.samples += 1;
    this.lastSeconds = seconds;
    const engagedAt = observation.controller?.engagedAtSeconds;
    if (observation.controller?.id !== this.controllerId || (engagedAt != null && engagedAt !== this.controllerEngagedAtSeconds)) {
      // A controller change restarts the evaluated interval: the previous window described a
      // different controller and may not be carried over.
      if (this.controllerId != null) this.#resetWindow();
      this.controllerEngagedAtSeconds = engagedAt ?? null;
    }
    this.controllerId = observation.controller?.id ?? this.controllerId;
    this.observationsValid &&= validStandObservation(observation, seconds);
    if (observation.actuationEnabled === false) this.actuationDisabledSeen = true;
    if ((classes.robotSelf?.length || 0) > 0) this.selfContactSeen = true;
    if ((classes.robotExternalObject?.length || 0) > 0) this.externalObjectContactSeen = true;

    let worst = 0;
    for (const joint of Object.values(observation.joints || {})) {
      const limit = Number(joint.effortLimitNm);
      const effort = Math.abs(Number(joint.effortNm));
      if (Number.isFinite(limit) && limit > 0 && Number.isFinite(effort)) worst = Math.max(worst, effort / limit);
    }
    this.worstActuatorEffortFraction = Math.max(this.worstActuatorEffortFraction, worst);

    const position = observation.root.positionM || [0, 0, 0];
    const tilt = Number(observation.root.tiltRad ?? 0);
    this.terminal = {
      simulationTimeSeconds: seconds,
      pelvisHeightM: Number(position[2]),
      tiltRad: tilt,
      horizontalDriftM: Math.hypot(Number(position[0]), Number(position[1])),
      linearSpeedMS: norm(observation.root.linearVelocityMS),
      angularSpeedRadS: norm(observation.root.angularVelocityRadS),
      leftFootContacts: supportingContacts(classes.leftFootFloor),
      rightFootContacts: supportingContacts(classes.rightFootFloor),
      nonFootGroundContacts: classes.otherBodyFloor?.length || 0,
      selfContacts: classes.robotSelf?.length || 0,
      externalObjectContacts: classes.robotExternalObject?.length || 0,
      fixtureContacts: classes.robotFixture?.length || 0,
    };
    if (tilt > this.gate.maxPelvisTiltRad) this.everLeftUprightBand = true;

    if (seconds + 1e-9 < this.evaluationOpensAtSeconds()) return;
    if (this.startedSeconds == null) this.startedSeconds = seconds;
    if (this.lastWindowSeconds != null && (seconds < this.lastWindowSeconds || seconds - this.lastWindowSeconds > this.gate.maxObservationGapSeconds)) {
      this.observationsContinuous = false;
    }
    this.lastWindowSeconds = seconds;
    this.windowSamples += 1;
    this.pelvisHeightMinM = Math.min(this.pelvisHeightMinM, Number(position[2]));
    this.pelvisHeightMaxM = Math.max(this.pelvisHeightMaxM, Number(position[2]));
    this.maxTiltRad = Math.max(this.maxTiltRad, tilt);
    this.maxHorizontalDriftM = Math.max(this.maxHorizontalDriftM, this.terminal.horizontalDriftM);
    this.maxNonFootGroundContacts = Math.max(this.maxNonFootGroundContacts, this.terminal.nonFootGroundContacts);
    this.maxExternalSupportContacts = Math.max(this.maxExternalSupportContacts, this.terminal.externalObjectContacts + this.terminal.fixtureContacts);
    if (!(this.terminal.leftFootContacts > 0 && this.terminal.rightFootContacts > 0)) this.bothFeetSupportedThroughout = false;
  }

  #resetWindow() {
    this.observationsValid = true;
    this.observationsContinuous = true;
    this.lastWindowSeconds = null;
    this.worstActuatorEffortFraction = 0;
    this.windowSamples = 0;
    this.startedSeconds = null;
    this.pelvisHeightMinM = Infinity;
    this.pelvisHeightMaxM = -Infinity;
    this.maxTiltRad = 0;
    this.maxHorizontalDriftM = 0;
    this.maxNonFootGroundContacts = 0;
    this.maxExternalSupportContacts = 0;
    this.bothFeetSupportedThroughout = true;
  }

  snapshot() {
    const gate = this.gate;
    const evaluated = this.windowSamples > 0;
    const elapsed = evaluated && this.lastSeconds != null ? this.lastSeconds - this.startedSeconds : 0;
    const checks = {
      completePhysicalObservations: this.observationsValid,
      continuousEvaluationEvidence: Boolean(evaluated && this.observationsContinuous),
      evaluationWindowReached: Boolean(evaluated && elapsed >= gate.evaluationSeconds - 1e-6),
      pelvisHeightInBand: Boolean(evaluated && this.pelvisHeightMinM >= gate.pelvisHeightRangeM[0] && this.pelvisHeightMaxM <= gate.pelvisHeightRangeM[1]),
      tiltWithinLimit: Boolean(evaluated && this.maxTiltRad <= gate.maxPelvisTiltRad),
      driftWithinLimit: Boolean(evaluated && this.maxHorizontalDriftM <= gate.maxHorizontalDriftM),
      terminalLinearSpeedWithinLimit: Boolean(this.terminal && this.terminal.linearSpeedMS <= gate.maxTerminalLinearSpeedMS),
      terminalAngularSpeedWithinLimit: Boolean(this.terminal && this.terminal.angularSpeedRadS <= gate.maxTerminalAngularSpeedRadS),
      bothFeetSupportedThroughout: gate.requireBothFeetSupported ? Boolean(evaluated && this.bothFeetSupportedThroughout) : true,
      noNonFootGroundContact: gate.forbidNonFootGroundContact ? this.maxNonFootGroundContacts === 0 : true,
      noExternalOrFixtureSupport: gate.forbidExternalSupport ? this.maxExternalSupportContacts === 0 : true,
      actuatorEffortWithinFraction: this.worstActuatorEffortFraction <= gate.maxActuatorEffortFraction,
    };
    return Object.freeze({
      standing: Object.values(checks).every(Boolean),
      checks: Object.freeze(checks),
      controllerId: this.controllerId,
      actuationDisabledSeen: this.actuationDisabledSeen,
      selfContactSeen: this.selfContactSeen,
      externalObjectContactSeen: this.externalObjectContactSeen,
      fellDuringRun: this.everLeftUprightBand,
      measured: Object.freeze({
        samples: this.samples,
        evaluatedSamples: this.windowSamples,
        controllerEngagedAtSeconds: this.controllerEngagedAtSeconds,
        evaluationOpensAtSeconds: this.evaluationOpensAtSeconds(),
        evaluationStartSeconds: this.startedSeconds,
        evaluationEndSeconds: this.lastSeconds,
        evaluatedSeconds: elapsed,
        pelvisHeightMinM: evaluated ? this.pelvisHeightMinM : null,
        pelvisHeightMaxM: evaluated ? this.pelvisHeightMaxM : null,
        maxTiltRad: evaluated ? this.maxTiltRad : null,
        maxHorizontalDriftM: evaluated ? this.maxHorizontalDriftM : null,
        maxNonFootGroundContacts: this.maxNonFootGroundContacts,
        maxExternalOrFixtureContacts: this.maxExternalSupportContacts,
        worstActuatorEffortFraction: this.worstActuatorEffortFraction,
      }),
      terminal: this.terminal ? Object.freeze({ ...this.terminal }) : null,
      gate,
    });
  }
}
