import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import {
  MICRODUCK_BAM_M6,
  MICRODUCK_BAM_REVISION,
  MICRODUCK_BAM_VERSION,
  MICRODUCK_DEPLOYMENT_PLANT_PROFILE,
  MICRODUCK_TRAINING_PLANT_PROFILE,
  MICRODUCK_XL330_ERROR_GAIN,
  MicroDuckDeploymentImuFilter,
  MicroDuckTargetDelay,
  microDuckBamForceCeilingNm,
  microDuckBamFrictionLossNm,
  microDuckBamMotorTorqueNm,
  microDuckBamSmallSignalKpNmRad,
  sampleMicroDuckTrainingPlant,
} from '../../src/physics/microduck-bam-plant.js';
import { MICRODUCK_WALK_PACKAGE } from '../../src/physics/microduck-model-package.js';

const ROOT = resolve(new URL('../..', import.meta.url).pathname);
function close(actual, expected, tolerance = 1e-12, label = 'value') {
  assert(Number.isFinite(actual), `${label} is not finite`);
  assert(Math.abs(actual - expected) <= tolerance, `${label}: got ${actual}, expected ${expected}`);
}

assert.equal(MICRODUCK_BAM_VERSION, '1.0.1');
assert.equal(MICRODUCK_BAM_REVISION, 'ab81512c44f1f709b99ef332addb5e51568cd51c');
close(MICRODUCK_BAM_M6.ktNmPerA, 0.36601349688984386, 1e-15, 'Kt');
close(MICRODUCK_BAM_M6.resistanceOhm, 2.8113923539223227, 1e-15, 'R');
close(MICRODUCK_BAM_M6.armatureKgM2, 0.0018077432831600838, 1e-15, 'armature');
close(MICRODUCK_XL330_ERROR_GAIN, (4096 / (2 * Math.PI)) / (256 * 885), 1e-18, 'XL330 error gain');
close(microDuckBamForceCeilingNm(), 8.2 * MICRODUCK_BAM_M6.ktNmPerA / MICRODUCK_BAM_M6.resistanceOhm, 1e-15, 'BAM force ceiling');

assert.deepEqual(MICRODUCK_TRAINING_PLANT_PROFILE.vinRangeV, [6.5, 8.2]);
assert.deepEqual(MICRODUCK_TRAINING_PLANT_PROFILE.vinDropGainRangeVPerNm, [0, 0.2]);
assert.deepEqual(MICRODUCK_TRAINING_PLANT_PROFILE.targetDelayPhysicsSteps, [3, 6]);
assert.deepEqual(MICRODUCK_TRAINING_PLANT_PROFILE.frictionScaleRange, [0.9, 1.1]);
assert.deepEqual(MICRODUCK_TRAINING_PLANT_PROFILE.armatureScaleRange, [0.9, 1.1]);
assert.equal(MICRODUCK_DEPLOYMENT_PLANT_PROFILE.imuPipeline, 'deployed-median3');

const sampled = sampleMicroDuckTrainingPlant(12345);
assert(sampled.vinV >= 6.5 && sampled.vinV <= 8.2);
assert(sampled.vinDropGainVPerNm >= 0 && sampled.vinDropGainVPerNm <= 0.2);
assert(sampled.frictionScale >= 0.9 && sampled.frictionScale <= 1.1);
assert(sampled.armatureScale >= 0.9 && sampled.armatureScale <= 1.1);
assert(sampled.massInertiaScale >= 0.95 && sampled.massInertiaScale <= 1.05);

// mjlab 1.3.0 delay semantics: append current target, sample an integer lag on each physics
// step, and clamp only while history is still shorter than the requested lag.
const delay = new MicroDuckTargetDelay({ minLag: 3, maxLag: 6, seed: 42 });
delay.reset([0]);
for (let i = 1; i <= 32; i += 1) {
  const result = delay.push([i]);
  assert(result.lag >= 3 && result.lag <= 6, `lag ${result.lag} escaped source range`);
  assert(result.validLag <= result.lag);
  assert.equal(result.target.length, 1);
}

// Deployed imu.rs initializes two gyro samples at zero and two gravity samples upright. A
// one-frame spike is rejected; a persistent value appears on the second call. Gravity is
// normalised before, not after, the component-wise median.
const imu = new MicroDuckDeploymentImuFilter();
let filtered = imu.sample([1, -2, 3], [0.6, 0, -0.8]);
assert.deepEqual(filtered.gyroRadS, [0, 0, 0]);
assert.deepEqual(filtered.projectedGravity, [0, 0, -1]);
filtered = imu.sample([1, -2, 3], [0.6, 0, -0.8]);
assert.deepEqual(filtered.gyroRadS, [1, -2, 3]);
close(filtered.projectedGravity[0], 0.6, 1e-12, 'median gravity x');
close(filtered.projectedGravity[2], -0.8, 1e-12, 'median gravity z');

// Motor-off removes electromagnetic torque even at non-zero velocity. Passive BAM friction
// is a separate plant term and remains available to the MuJoCo constraint solver.
assert.equal(microDuckBamMotorTorqueNm({ targetRad: 1, positionRad: 0, velocityRadS: 2, firmwareGain: 200, vinV: 7.4, enabled: false }), 0);
assert(microDuckBamFrictionLossNm({ motorTorqueNm: 0, externalTorqueNm: 0.2, velocityRadS: 0.1, frictionScale: 1, quadraticSignGate: true }) > 0);
close(microDuckBamSmallSignalKpNmRad(200, 7.4), 0.5544140148169708, 1e-12, 'BAM 7.4V small-signal stiffness');

// Metadata must no longer claim the BAM actuator is absent.
assert.equal(MICRODUCK_WALK_PACKAGE.plant.actuator.family, 'BAM');
assert.equal(MICRODUCK_WALK_PACKAGE.plant.actuator.version, '1.0.1');
assert.equal(MICRODUCK_WALK_PACKAGE.plant.interactiveProfile, 'deployment-reference');
assert(!MICRODUCK_WALK_PACKAGE.limitations.some((text) => /BAM M6 .*NOT reproduced/i.test(text)));
assert(MICRODUCK_WALK_PACKAGE.limitations.some((text) => /hardware comparison/i.test(text)));
assert(MICRODUCK_WALK_PACKAGE.limitations.some((text) => /Creative Commons BY-SA-NC/i.test(text)), 'source 3D collision asset license is not disclosed');
assert(MICRODUCK_WALK_PACKAGE.limitations.some((text) => /outside .*MIT scope|outside RoboBuddy original-code MIT scope/i.test(text)), 'source 3D collision assets are not scoped outside RoboBuddy MIT code');

const worker = readFileSync(resolve(ROOT, 'src/physics/microduck-mujoco-worker.js'), 'utf8');
assert(/configureBamMotorPlant/.test(worker), 'browser worker no longer converts the XML fallback to BAM motor mode');
assert(/updateBamBeforePhysicsStep/.test(worker), 'BAM is not applied at every physics step');
assert(/MicroDuckDeploymentImuFilter/.test(worker), 'deployed IMU median filter disappeared');
assert(!/applyFirmwareGain\(/.test(worker), 'legacy position-kp actuation path returned');
assert(/passiveBamFrictionRetained: true/.test(worker), 'motor-off stopped declaring retained passive gearbox friction');
// Scheduled controller gain and effective physical gain are deliberately separate. A
// torque-off must not continue to advertise 200/160 as the gain physically in effect.
assert(/firmwareGain:\s*actuationEnabled\s*\?\s*appliedFirmwareGain\s*:\s*0/.test(worker), 'torque-off no longer reports zero effective firmware gain');
assert(/microDuckBamSmallSignalKpNmRad\(actuationEnabled\s*\?\s*appliedFirmwareGain\s*:\s*0,\s*effectiveVinV\)/.test(worker), 'torque-off no longer reports zero effective BAM small-signal gain');

const referencePath = process.argv[2];
if (referencePath) {
  const reference = JSON.parse(readFileSync(referencePath, 'utf8'));
  assert.equal(reference.package, 'better-actuator-models');
  assert.equal(reference.version, '1.0.1');
  for (const [key, expected] of Object.entries(reference.params)) {
    if (key === 'errorGain') close(MICRODUCK_XL330_ERROR_GAIN, expected, 1e-15, key);
    else if (key === 'maxPwm') close(1, expected, 1e-15, key);
    else if (key in MICRODUCK_BAM_M6) close(MICRODUCK_BAM_M6[key], expected, 1e-12, key);
  }
  close(microDuckBamForceCeilingNm(), reference.trainingForceCeilingNm, 1e-12, 'native force ceiling');
  for (const sample of reference.samples) {
    const motor = microDuckBamMotorTorqueNm({
      targetRad: sample.target, positionRad: sample.q, velocityRadS: sample.dq,
      firmwareGain: sample.gain, vinV: sample.vin, enabled: true,
    });
    close(motor, sample.motorTorqueNm, 1e-12, `${sample.name} motor torque`);
    const friction = microDuckBamFrictionLossNm({
      motorTorqueNm: sample.motorTorqueNm, externalTorqueNm: sample.external,
      velocityRadS: sample.dq, frictionScale: 1, quadraticSignGate: true,
    });
    close(friction, sample.frictionLossNm, 1e-12, `${sample.name} friction`);
    close(MICRODUCK_BAM_M6.frictionViscousNmPerRadS, sample.dampingNmPerRadS, 1e-12, `${sample.name} damping`);
  }
}

console.log('MicroDuck BAM/deployment plant contract: OK');