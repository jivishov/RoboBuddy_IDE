import { MICRODUCK_BALL_BODY, MICRODUCK_FOOT_GEOMS, MICRODUCK_TRUNK_BODY } from './microduck-model-package.js';

// Causal task evaluation for the MicroDuck physical workspace.
//
// Every outcome below is read from authoritative MuJoCo state: trunk pose, named foot-floor
// contacts, named sole-ball contacts, and free-body ball pose. Nothing here infers a contact
// from a total contact count or from an object's speed changing, and nothing here can be set
// by a command, a program's print statement, or a WebMCP call.

// The upstream velstand gates, reused so a fall and a recovery mean the same thing here as
// they do in the training environment.
export const MICRODUCK_FALLEN_TILT_DEG = 40;
export const MICRODUCK_FALLEN_TRUNK_M = 0.10;
export const MICRODUCK_RECOVERED_TILT_DEG = 25;
export const MICRODUCK_RECOVERED_TRUNK_M = 0.09;

export function trunkPose(observation) {
  const body = observation?.bodies?.[MICRODUCK_TRUNK_BODY];
  if (!body?.positionM || !body?.quaternionWxyz) return null;
  return { positionM: body.positionM.map(Number), quaternionWxyz: body.quaternionWxyz.map(Number) };
}

export function tiltDegrees(quaternionWxyz) {
  const [w, x, y, z] = (quaternionWxyz || [1, 0, 0, 0]).map(Number);
  // The body-frame z component of world up.
  const upZ = 1 - 2 * (x * x + y * y);
  return (Math.acos(Math.max(-1, Math.min(1, upZ))) * 180) / Math.PI;
}

export function yawRadians(quaternionWxyz) {
  const [w, x, y, z] = (quaternionWxyz || [1, 0, 0, 0]).map(Number);
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

function shortestAngle(delta) {
  return Math.atan2(Math.sin(delta), Math.cos(delta));
}

/**
 * Accumulates physical evidence across a run.
 *
 * It is fed every authoritative observation the session publishes - including the ones
 * sampled inside a long advance - so a transient foot-ball contact cannot be missed just
 * because a caller asked for a long interval.
 */
export class MicroDuckPhysicalEvaluator {
  constructor({ commandedTwist = [0, 0, 0] } = {}) {
    this.commandedTwist = commandedTwist.map(Number);
    this.reset();
  }

  reset() {
    this.samples = 0;
    this.start = null;
    this.latest = null;
    this.startSimulationTime = null;
    this.latestSimulationTime = null;
    this.footFloor = { left_foot_collision: { stance: 0, transitions: 0, last: null }, right_foot_collision: { stance: 0, transitions: 0, last: null } };
    this.footBallContacts = [];
    this.ballStart = null;
    this.ballLatest = null;
    this.minTrunkZ = Infinity;
    this.maxTrunkZ = -Infinity;
    this.maxTiltDeg = 0;
    this.everFallen = false;
    this.actuationDisabledSamples = 0;
    this.setupLog = [];
  }

  observe(observation) {
    const pose = trunkPose(observation);
    if (!pose) return;
    const time = Number(observation.simulationTimeSeconds ?? observation.simulationTime ?? 0);
    if (this.start == null) {
      this.start = pose;
      this.startSimulationTime = time;
      const ball = observation?.bodies?.[MICRODUCK_BALL_BODY];
      if (ball?.positionM) this.ballStart = ball.positionM.map(Number);
    }
    this.latest = pose;
    this.latestSimulationTime = time;
    this.samples += 1;

    const z = pose.positionM[2];
    this.minTrunkZ = Math.min(this.minTrunkZ, z);
    this.maxTrunkZ = Math.max(this.maxTrunkZ, z);
    const tilt = tiltDegrees(pose.quaternionWxyz);
    this.maxTiltDeg = Math.max(this.maxTiltDeg, tilt);
    if (tilt > MICRODUCK_FALLEN_TILT_DEG || z < MICRODUCK_FALLEN_TRUNK_M) this.everFallen = true;
    if (observation.actuationEnabled === false) this.actuationDisabledSamples += 1;
    if (Array.isArray(observation.setupLog) && observation.setupLog.length > this.setupLog.length) {
      this.setupLog = observation.setupLog.map((item) => ({ ...item }));
    }

    const floor = observation?.footContacts?.floor;
    for (const foot of MICRODUCK_FOOT_GEOMS) {
      const touching = Boolean(floor?.[foot]);
      const record = this.footFloor[foot];
      if (touching) record.stance += 1;
      if (record.last !== null && record.last !== touching) record.transitions += 1;
      record.last = touching;
    }

    for (const contact of observation?.footContacts?.ball || []) {
      this.footBallContacts.push({
        simulationTimeSeconds: time,
        foot: contact.foot,
        normalForceN: contact.normalForceN == null ? null : Number(contact.normalForceN),
        geoms: Array.isArray(contact.geoms) ? [...contact.geoms] : [],
      });
    }

    const ball = observation?.bodies?.[MICRODUCK_BALL_BODY];
    if (ball?.positionM) this.ballLatest = ball.positionM.map(Number);
  }

  report() {
    if (!this.start || !this.latest) {
      return { samples: this.samples, complete: false, reason: 'no authoritative observation was recorded' };
    }
    const dx = this.latest.positionM[0] - this.start.positionM[0];
    const dy = this.latest.positionM[1] - this.start.positionM[1];
    const dz = this.latest.positionM[2] - this.start.positionM[2];
    const planar = Math.hypot(dx, dy);
    const duration = Math.max(1e-9, this.latestSimulationTime - this.startSimulationTime);
    const [vx, vy] = this.commandedTwist;
    const axis = Math.hypot(vx, vy);
    const commandedAxisM = axis > 1e-9 ? (dx * vx + dy * vy) / axis : null;
    const tilt = tiltDegrees(this.latest.quaternionWxyz);
    const report = {
      samples: this.samples,
      complete: true,
      simulationSeconds: Number(duration.toFixed(6)),
      displacementM: [dx, dy, dz].map((value) => Number(value.toFixed(6))),
      planarDistanceM: Number(planar.toFixed(6)),
      commandedAxisDistanceM: commandedAxisM == null ? null : Number(commandedAxisM.toFixed(6)),
      averageSpeedMS: Number((planar / duration).toFixed(6)),
      yawChangeRad: Number(shortestAngle(yawRadians(this.latest.quaternionWxyz) - yawRadians(this.start.quaternionWxyz)).toFixed(6)),
      finalTrunkHeightM: Number(this.latest.positionM[2].toFixed(6)),
      minTrunkHeightM: Number(this.minTrunkZ.toFixed(6)),
      maxTrunkHeightM: Number(this.maxTrunkZ.toFixed(6)),
      finalTiltDeg: Number(tilt.toFixed(4)),
      maxTiltDeg: Number(this.maxTiltDeg.toFixed(4)),
      fallen: tilt > MICRODUCK_FALLEN_TILT_DEG || this.latest.positionM[2] < MICRODUCK_FALLEN_TRUNK_M,
      everFallen: this.everFallen,
      upright: tilt < MICRODUCK_RECOVERED_TILT_DEG && this.latest.positionM[2] > MICRODUCK_RECOVERED_TRUNK_M,
      actuationDisabledSamples: this.actuationDisabledSamples,
      setupLog: this.setupLog.map((item) => ({ ...item })),
      contacts: {
        leftStanceFraction: Number((this.footFloor.left_foot_collision.stance / Math.max(1, this.samples)).toFixed(4)),
        rightStanceFraction: Number((this.footFloor.right_foot_collision.stance / Math.max(1, this.samples)).toFixed(4)),
        leftContactTransitions: this.footFloor.left_foot_collision.transitions,
        rightContactTransitions: this.footFloor.right_foot_collision.transitions,
        footBallContacts: this.footBallContacts.map((item) => ({ ...item })),
      },
    };
    if (this.ballStart && this.ballLatest) {
      const bdx = this.ballLatest[0] - this.ballStart[0];
      const bdy = this.ballLatest[1] - this.ballStart[1];
      report.ball = {
        startM: this.ballStart.map((value) => Number(value.toFixed(6))),
        endM: this.ballLatest.map((value) => Number(value.toFixed(6))),
        planarDistanceM: Number(Math.hypot(bdx, bdy).toFixed(6)),
        footContactCount: this.footBallContacts.length,
        // A kick outcome requires a named sole/ball contact. Ball motion without one is
        // reported as unattributed, never as a kick.
        movedByFootContact: this.footBallContacts.length > 0 && Math.hypot(bdx, bdy) > 1e-4,
        unattributedMotion: this.footBallContacts.length === 0 && Math.hypot(bdx, bdy) > 1e-4,
      };
    }
    return report;
  }

  /** Did contact-driven locomotion actually happen? Contact evidence, not a distance alone. */
  locomotionVerdict({ minimumCommandedAxisM = 0.05, minimumTransitionsPerFoot = 4 } = {}) {
    const report = this.report();
    if (!report.complete) return { walked: false, reason: report.reason, report };
    const axis = report.commandedAxisDistanceM;
    const transitions = Math.min(report.contacts.leftContactTransitions, report.contacts.rightContactTransitions);
    const reasons = [];
    if (axis == null) reasons.push('no linear command was issued');
    else if (axis < minimumCommandedAxisM) reasons.push(`commanded-axis displacement ${axis} m is below ${minimumCommandedAxisM} m`);
    if (transitions < minimumTransitionsPerFoot) reasons.push(`only ${transitions} contact transitions on the quieter foot, below ${minimumTransitionsPerFoot}`);
    if (report.fallen) reasons.push('the robot ended fallen');
    return { walked: reasons.length === 0, reasons, report };
  }
}
