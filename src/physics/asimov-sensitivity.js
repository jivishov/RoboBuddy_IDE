/** Immutable, predeclared experiments. No value is a fitted hardware parameter. */
function freeze(value) {
  if (value && typeof value === 'object') { Object.values(value).forEach(freeze); Object.freeze(value); }
  return value;
}
export const ASIMOV_BODY_SENSOR_PROFILE = freeze({
  id: 'asimov-body-sensors-v2', sampleIntervalSeconds: 0.005,
  frame: 'pelvis_link', seed: 1729, imuDelaySeconds: 0.01,
  positionNoiseRad: 0.01, velocityNoiseRadS: 0.1,
  angularVelocityNoiseRadS: 0.01, gravityNoise: 0.05,
  groupingEvidence: 'Explicit joint-ID assignment retaining the old assumed source-order groups; not verified CAN order.',
  evidence: 'Synthetic body-frame gyro and perturbed normalized projected gravity; no raw accelerometer or hardware calibration.',
});
export const ASIMOV_SENSOR_STANDING_CONTROLLER = freeze({
  id: 'asimov-sensor-stance-v2', label: 'Experimental sensor-driven standing',
  claim: 'Balance feedback consumes only synthetic body-frame sensors. Inner joint PD remains an ideal local encoder loop. No walking or hardware calibration.',
  orientationKp: 200, angularKd: 20, controlIntervalSeconds: 0.005,
  filterTimeConstantSeconds: 0.03, maximumSensorAgeSeconds: 0.04,
  warmupSeconds: 0.05,
});
const make = (id, sensor, ankle = null) => ({
  id, evidence: 'Hypothetical sensitivity experiment; not a confidence interval or identified hardware model.',
  sensor: { ...ASIMOV_BODY_SENSOR_PROFILE, ...sensor },
  actuator: { commandDelaySeconds: 0.005, ...(ankle ? { ankleStress: ankle } : {}) },
  balanceController: ASIMOV_SENSOR_STANDING_CONTROLLER,
});
export const ASIMOV_SENSITIVITY_PROFILES = freeze({
  'sensor-standing': make('sensor-standing', {}),
  'sensor-standing-delay': make('sensor-standing-delay', { imuDelaySeconds: 0.025 }),
  'sensor-standing-ankle-stress': make('sensor-standing-ankle-stress', {}, {
    torqueScale: 0.7, speedRadS: 9.32, staticNm: 0.4, dynamicNm: 0.015,
    evidence: 'Assumed independent-axis ankle losses and 70% inherited joint torque caps. NOT physical A/B motor parameters or paired-motor feasibility.',
  }),
});
export function asimovSensitivity(id) {
  if (typeof id !== 'string' || !Object.hasOwn(ASIMOV_SENSITIVITY_PROFILES, id)) throw new RangeError('Unknown declared Asimov sensitivity profile');
  return ASIMOV_SENSITIVITY_PROFILES[id];
}
export const ASIMOV_MASS_RECONCILIATION = freeze({
  sourceModelMassKg: 32.22491296549718, publishedNominalMassKg: 35,
  differenceKg: 35 - 32.22491296549718,
  status: 'unresolved-assembly-configuration; no automatic model mass scaling',
  source: 'https://docs.menlo.ai/asimov/1/overview/system-tour/mechanical',
  retrievedOn: '2026-09-12',
});
