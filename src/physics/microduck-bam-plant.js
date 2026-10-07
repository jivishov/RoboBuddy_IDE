// MicroDuck BAM M6 actuator and deployed-IMU reference primitives.
//
// Clean-room JavaScript implementation of the public equations/constants in:
//   Rhoban/bam v1.0.1 (Apache-2.0), commit ab81512c44f1f709b99ef332addb5e51568cd51c
//   pollen-robotics/microduck_rl@519142b1f5bf59fdfd44d06c205119e7fff8e3cb
//   pollen-robotics/microduck@590b986 (deployed controller / IMU path)
//
// TRAINING and DEPLOYMENT are deliberately separate. The training plant randomises voltage,
// sag, actuator latency, friction and inertia. The deployed controller changes firmware gain
// by skill and median-filters its real IMU stream. A single hybrid profile would be less true.

export const MICRODUCK_BAM_VERSION = '1.0.1';
export const MICRODUCK_BAM_REVISION = 'ab81512c44f1f709b99ef332addb5e51568cd51c';
export const MICRODUCK_RL_REVISION = '519142b1f5bf59fdfd44d06c205119e7fff8e3cb';
export const MICRODUCK_DEPLOYED_REVISION = '590b986';

export const MICRODUCK_BAM_M6 = Object.freeze({
  motor: 'xl330', model: 'm6',
  ktNmPerA: 0.36601349688984386,
  resistanceOhm: 2.8113923539223227,
  armatureKgM2: 0.0018077432831600838,
  qOffsetRad: 0.0271132870444849,
  frictionBaseNm: 0.004771183165566,
  frictionStribeckNm: 0.004676345799486616,
  loadFrictionMotor: 0.2667860954283698,
  loadFrictionExternal: 8.515871897059342e-06,
  loadFrictionMotorStribeck: 1.0722918395099123e-05,
  loadFrictionExternalStribeck: 0.08077928978935671,
  loadFrictionMotorQuad: 0.009972471242139415,
  loadFrictionExternalQuad: 0.004902565732332559,
  dthetaStribeckRadS: 2.890372094130307,
  stribeckAlpha: 8.683259907618984,
  frictionViscousNmPerRadS: 0.005359668274599504,
});

export const MICRODUCK_XL330 = Object.freeze({ encoderCountsPerRev: 4096, kpDivisor: 256, pwmLimit: 885, maxPwm: 1.0 });
export const MICRODUCK_XL330_ERROR_GAIN =
  (MICRODUCK_XL330.encoderCountsPerRev / (2 * Math.PI)) /
  (MICRODUCK_XL330.kpDivisor * MICRODUCK_XL330.pwmLimit);

export const MICRODUCK_TRAINING_PLANT_PROFILE = Object.freeze({
  id: 'training-reference',
  firmwareGain: 200,
  vinRangeV: Object.freeze([6.5, 8.2]),
  vinDropGainRangeVPerNm: Object.freeze([0.0, 0.2]),
  vinMinV: 6.0,
  targetDelayPhysicsSteps: Object.freeze([3, 6]),
  frictionScaleRange: Object.freeze([0.9, 1.1]),
  armatureScaleRange: Object.freeze([0.9, 1.1]),
  massInertiaScaleRange: Object.freeze([0.95, 1.05]),
  encoderBiasRangeRad: Object.freeze([-0.015, 0.015]),
  imuMountRandomizationDeg: 6.0,
  quadraticSignGate: false,
  frictionApplication: 'explicit-static-friction-clipping',
  imuPipeline: 'training-observation-pipeline',
});

export const MICRODUCK_DEPLOYMENT_PLANT_PROFILE = Object.freeze({
  id: 'deployment-reference',
  nominalVinV: 7.4,
  vinDropGainVPerNm: 0.1,
  vinMinV: 6.0,
  targetDelayPhysicsSteps: Object.freeze([0, 0]),
  frictionScale: 1.0,
  armatureScale: 1.0,
  quadraticSignGate: true,
  frictionApplication: 'mujoco-friction-constraint',
  imuPipeline: 'deployed-median3',
});

export function microDuckBamForceCeilingNm(vinV = MICRODUCK_TRAINING_PLANT_PROFILE.vinRangeV[1]) {
  return Number(vinV) * MICRODUCK_BAM_M6.ktNmPerA / MICRODUCK_BAM_M6.resistanceOhm;
}

export function microDuckBamSmallSignalKpNmRad(firmwareGain, vinV) {
  return Number(firmwareGain) * MICRODUCK_XL330_ERROR_GAIN * Number(vinV)
    * MICRODUCK_BAM_M6.ktNmPerA / MICRODUCK_BAM_M6.resistanceOhm;
}

export function clamp(value, minimum, maximum) { return Math.min(maximum, Math.max(minimum, value)); }

export function normaliseProjectedGravity(vector) {
  const v = [Number(vector?.[0]), Number(vector?.[1]), Number(vector?.[2])];
  const mag = Math.hypot(...v);
  return Number.isFinite(mag) && mag > 0.1 ? v.map((x) => x / mag) : [0, 0, -1];
}

function median3(a, b, c) {
  // Rust source: a.max(b).min(c).max(a.min(b)). Algebraically this is the middle value.
  return Math.max(Math.min(Math.max(a, b), c), Math.min(a, b));
}

export function median3Vector(previous2, previous1, current) {
  return [0, 1, 2].map((i) => median3(Number(previous2[i]), Number(previous1[i]), Number(current[i])));
}

export class MicroDuckDeploymentImuFilter {
  constructor() { this.reset(); }
  reset() {
    this.gyroHistory = [[0, 0, 0], [0, 0, 0]];
    this.gravityHistory = [[0, 0, -1], [0, 0, -1]];
    this.last = { gyroRadS: [0, 0, 0], projectedGravity: [0, 0, -1] };
  }
  sample(gyroRadS, projectedGravity) {
    const gyro = [0, 1, 2].map((i) => Number(gyroRadS?.[i]));
    const gravity = normaliseProjectedGravity(projectedGravity);
    const filtered = {
      gyroRadS: median3Vector(this.gyroHistory[0], this.gyroHistory[1], gyro),
      projectedGravity: median3Vector(this.gravityHistory[0], this.gravityHistory[1], gravity),
    };
    this.gyroHistory = [this.gyroHistory[1], gyro];
    this.gravityHistory = [this.gravityHistory[1], gravity];
    this.last = filtered;
    return { gyroRadS: [...filtered.gyroRadS], projectedGravity: [...filtered.projectedGravity] };
  }
}

export class MicroDuckDeterministicRng {
  constructor(seed = 0x4d445543) { this.state = (Number(seed) >>> 0) || 1; }
  next() {
    let x = this.state >>> 0;
    x ^= x << 13; x ^= x >>> 17; x ^= x << 5;
    this.state = x >>> 0;
    return this.state / 0x100000000;
  }
  uniform(minimum, maximum) { return Number(minimum) + (Number(maximum) - Number(minimum)) * this.next(); }
  integer(minimum, maximum) { return minimum + Math.floor(this.next() * (maximum - minimum + 1)); }
}

export class MicroDuckTargetDelay {
  constructor({ minLag = 0, maxLag = 0, seed = 0x4d445543 } = {}) {
    if (!Number.isInteger(minLag) || !Number.isInteger(maxLag) || minLag < 0 || maxLag < minLag) throw new RangeError('invalid MicroDuck delay range');
    this.minLag = minLag; this.maxLag = maxLag; this.rng = new MicroDuckDeterministicRng(seed); this.history = [];
  }
  reset(initialTarget) { this.history = [Array.from(initialTarget, Number)]; }
  push(target) {
    const frame = Array.from(target, Number);
    if (!this.history.length) this.reset(frame); else this.history.push(frame);
    const limit = this.maxLag + 1;
    if (this.history.length > limit) this.history.splice(0, this.history.length - limit);
    const lag = this.maxLag > 0 ? this.rng.integer(this.minLag, this.maxLag) : 0;
    const validLag = Math.min(lag, this.history.length - 1);
    return { lag, validLag, target: [...this.history[this.history.length - 1 - validLag]] };
  }
}

export function microDuckBamMotorTorqueNm({ targetRad, positionRad, velocityRadS, firmwareGain, vinV, enabled = true }) {
  if (!enabled) return 0;
  const positionError = Number(targetRad) - Number(positionRad);
  const dutyCycle = clamp(positionError * Number(firmwareGain) * MICRODUCK_XL330_ERROR_GAIN, -MICRODUCK_XL330.maxPwm, MICRODUCK_XL330.maxPwm);
  const voltage = Number(vinV) * dutyCycle;
  return MICRODUCK_BAM_M6.ktNmPerA * voltage / MICRODUCK_BAM_M6.resistanceOhm
    - (MICRODUCK_BAM_M6.ktNmPerA ** 2) * Number(velocityRadS) / MICRODUCK_BAM_M6.resistanceOhm;
}

export function microDuckBamFrictionLossNm({ motorTorqueNm, externalTorqueNm, velocityRadS, frictionScale = 1, quadraticSignGate = true }) {
  const p = MICRODUCK_BAM_M6;
  const motor = Number(motorTorqueNm); const external = Number(externalTorqueNm); const absVelocity = Math.abs(Number(velocityRadS));
  const stribeck = Math.exp(-((absVelocity / p.dthetaStribeckRadS) ** p.stribeckAlpha));
  const gearbox = Math.abs(external * p.loadFrictionExternal - motor * p.loadFrictionMotor);
  const gearboxStribeck = Math.abs(external * p.loadFrictionExternalStribeck - motor * p.loadFrictionMotorStribeck);
  let loss = p.frictionBaseNm + gearbox + stribeck * (p.frictionStribeckNm + gearboxStribeck);
  const driveSide = Math.abs(external) < Math.abs(motor);
  const quad = driveSide ? p.loadFrictionExternalQuad * Math.abs(external) ** 2 : p.loadFrictionMotorQuad * Math.abs(motor) ** 2;
  if (!quadraticSignGate || Math.sign(external) !== Math.sign(motor)) loss += stribeck * quad;
  return Math.max(0, loss * Number(frictionScale));
}

export function microDuckTrainingOutputTorqueNm({ motorTorqueNm, externalTorqueNm, velocityRadS, effectiveInertiaKgM2, timestepSeconds, frictionScale = 1 }) {
  const frictionLoss = microDuckBamFrictionLossNm({ motorTorqueNm, externalTorqueNm, velocityRadS, frictionScale, quadraticSignGate: false });
  const budget = frictionLoss + MICRODUCK_BAM_M6.frictionViscousNmPerRadS * Math.abs(Number(velocityRadS));
  const netNoFriction = Number(motorTorqueNm) - Number(externalTorqueNm);
  const tauStop = (Number(effectiveInertiaKgM2) / Number(timestepSeconds)) * Number(velocityRadS) + netNoFriction;
  return Number(motorTorqueNm) - Math.sign(tauStop) * Math.min(Math.abs(tauStop), budget);
}

export function sampleMicroDuckTrainingPlant(seed = 0x4d445543) {
  const rng = new MicroDuckDeterministicRng(seed);
  return Object.freeze({
    seed: Number(seed) >>> 0,
    vinV: rng.uniform(...MICRODUCK_TRAINING_PLANT_PROFILE.vinRangeV),
    vinDropGainVPerNm: rng.uniform(...MICRODUCK_TRAINING_PLANT_PROFILE.vinDropGainRangeVPerNm),
    frictionScale: rng.uniform(...MICRODUCK_TRAINING_PLANT_PROFILE.frictionScaleRange),
    armatureScale: rng.uniform(...MICRODUCK_TRAINING_PLANT_PROFILE.armatureScaleRange),
    massInertiaScale: rng.uniform(...MICRODUCK_TRAINING_PLANT_PROFILE.massInertiaScaleRange),
  });
}
