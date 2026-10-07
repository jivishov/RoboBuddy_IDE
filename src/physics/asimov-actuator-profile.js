import { ASIMOV_SOURCE } from './asimov-generated.js';

function freeze(value) {
  if (value && typeof value === 'object') { Object.values(value).forEach(freeze); Object.freeze(value); }
  return value;
}
// Transcribed scalar specifications, retrieved 2026-09-11. These are NOT measured
// calibration data. The conflicting legs-only release is deliberately not merged.
export const ASIMOV_ACTUATOR_AUDIT = freeze({
  id: 'asimov-actuator-audit-20260911-v1',
  sourceRevision: ASIMOV_SOURCE.revision,
  mechanicalSource: 'https://docs.menlo.ai/asimov/1/overview/system-tour/mechanical',
  trainingSource: 'https://docs.menlo.ai/guides/locomotion-training/reinforcement-learning-simulation-training-environment',
  incompatibleLegsRevision: '98870d12f079c0b6313bb0fe459aa591a5e7f251',
  retrievedOn: '2026-09-11',
  families: {
    'EC-A6416-P2-25': { ratedNm: 40, peakNm: 120, speedRadS: 12.57, ktNmA: 2.75, armatureKgM2: .095625, staticNm: .70, dynamicNm: .050 },
    'EC-A5013-H17-100': { ratedNm: 30, peakNm: 90, speedRadS: 3.98, ktNmA: 7.15, armatureKgM2: .11, staticNm: .20, dynamicNm: .020 },
    'EC-A3814-H14-107': { ratedNm: 20, peakNm: 60, speedRadS: 5.45, ktNmA: 5.70, armatureKgM2: .038, staticNm: .70, dynamicNm: .050 },
    'EC-A4315-P2-36': { ratedNm: 25, peakNm: 75, speedRadS: 12.25, ktNmA: 2.53, armatureKgM2: .0339552, staticNm: .70, dynamicNm: .020 },
    'EC-A4310-P2-36': { ratedNm: 12, peakNm: 36, speedRadS: 9.32, ktNmA: 1.81, armatureKgM2: .0282528, staticNm: .40, dynamicNm: .015 },
  },
  ankle: {
    status: 'unresolved-motor-to-joint-mapping',
    publishedRows: { A: { ratedNm: 40, peakNm: 145.4, armatureKgM2: .0565056 }, B: { ratedNm: 17, peakNm: 57.6, armatureKgM2: .0565056 } },
    pitchRatio: null, rollRatio: null,
    decision: 'Keep four source-equivalent ankle axes. Do not reinterpret effective ankle ratings as individual A/B motor limits, guess linkage ratios, or double the already-reflected inertia.',
  },
});
function familyFor(id) {
  if (id.includes('ankle')) return null;
  if (id.includes('hip_pitch') || id === 'waist_yaw_joint') return 'EC-A6416-P2-25';
  if (id.includes('hip_roll') || id.includes('shoulder_pitch')) return 'EC-A5013-H17-100';
  if (id.includes('hip_yaw') || id.includes('shoulder_yaw')) return 'EC-A3814-H14-107';
  if (id.includes('knee') || id.includes('shoulder_roll')) return 'EC-A4315-P2-36';
  return 'EC-A4310-P2-36';
}
export const ASIMOV_ACTUATOR_PROFILE = freeze({
  id: 'asimov-spec-informed-actuation-v1',
  label: 'Spec-informed actuator experiment — uncalibrated',
  controlIntervalSeconds: .005,
  commandDelaySeconds: .005,
  // No published full torque/speed curve or thermal constants: this taper is an
  // explicit conservative EXPERIMENT, not a fitted DC motor curve.
  speedEnvelope: 'estimated-linear-motoring-taper; continuous braking cap',
  peakOperation: 'disabled-no-duty-cycle-data',
  frictionLaw: 'smoothed-Stribeck; no exact stiction or backlash',
  frictionTransitionRadS: .10, frictionRegularizationRadS: .02,
  evidence: 'source-informed-estimates; no hardware validation',
  joints: ASIMOV_SOURCE.joints.map(j => {
    const family = familyFor(j.id), spec = ASIMOV_ACTUATOR_AUDIT.families[family];
    if (spec && Math.abs(spec.armatureKgM2 - j.armature) > 1e-9) throw new Error(`Unreconciled Asimov armature: ${j.id}`);
    return { id: j.id, family, continuousLimitNm: spec ? Math.min(j.effortLimitNm, spec.ratedNm) : j.effortLimitNm,
      speedRadS: spec ? Math.min(j.velocityLimitRadS, spec.speedRadS) : j.velocityLimitRadS,
      staticNm: spec?.staticNm ?? 0, dynamicNm: spec?.dynamicNm ?? 0,
      sourceEffortNm: j.effortLimitNm, armatureKgM2: j.armature,
      status: spec ? 'role-and-armature-matched; curve-and-friction-law-estimated' : 'source-equivalent-ankle; motor-space-model-blocked' };
  }),
});
export const ASIMOV_SENSOR_PROFILE = freeze({
  id: 'asimov-hardware-like-stress-v1', label: 'Hardware-like sensitivity profile — not calibrated',
  sampleIntervalSeconds: .005, seed: 1729,
  // Amplitudes are illustrative legs-training values, NOT measured full-body noise.
  positionNoiseRad: .01, velocityNoiseRadS: .1, angularVelocityNoiseRadS: .01, gravityNoise: .05,
  jointDelaySeconds: ASIMOV_SOURCE.joints.map((_, i) => i < 8 ? .01 : i < 16 ? .005 : 0),
  groupingEvidence: 'assumed contiguous source-order groups; not verified CAN polling order',
  distribution: 'seeded independent bounded uniform',
});
