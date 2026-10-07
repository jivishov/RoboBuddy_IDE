import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFileSync } from 'node:fs';
import { assertPhysicalScene } from '../../src/physics/backend-contract.js';
import { PARAMETER_EVIDENCE, getModelPackage, listModelPackages, requireModelPackage, validateModelPackage } from '../../src/physics/model-registry.js';
import { PHASE1_MODEL_PACKAGE, SO101_PHASE2A_MODEL_PACKAGE } from '../../src/physics/model-packages.js';
import { SO101_PHASE2A_SCENE } from '../../src/physics/so101-scene.js';

assert.equal(getModelPackage('phase1-vertical-slice'), PHASE1_MODEL_PACKAGE);
assert.equal(requireModelPackage('phase1-vertical-slice').asset, 'models/vertical-slice/model.xml');
assert.equal(requireModelPackage('phase1-vertical-slice').modelId, 'phase1-vertical-slice-v1');
assert.equal(listModelPackages().filter((item) => item.id === 'phase1-vertical-slice').length, 1);

assert.equal(getModelPackage(SO101_PHASE2A_MODEL_PACKAGE.id), SO101_PHASE2A_MODEL_PACKAGE);
assert.equal(SO101_PHASE2A_MODEL_PACKAGE.robotId, 'so101_follower');
assert.equal(SO101_PHASE2A_MODEL_PACKAGE.source.revision, '8161bba264d7fa7c99ca301e91e7fb44737676ad');
assert.equal(SO101_PHASE2A_MODEL_PACKAGE.license, 'Apache-2.0');
assert.equal(SO101_PHASE2A_MODEL_PACKAGE.physics.timestepSeconds, 0.005);
assert.equal(SO101_PHASE2A_MODEL_PACKAGE.physics.integrator, 'implicitfast');
assert.deepEqual(SO101_PHASE2A_MODEL_PACKAGE.joints.map(({ id }) => id), ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']);
assert.deepEqual(SO101_PHASE2A_MODEL_PACKAGE.actuators.map(({ jointId }) => jointId), SO101_PHASE2A_MODEL_PACKAGE.joints.map(({ id }) => id));
assert.ok(SO101_PHASE2A_MODEL_PACKAGE.actuators.every(({ evidence }) => evidence === PARAMETER_EVIDENCE.SOURCE_DERIVED), 'SO-101 actuator names, mappings and control ranges must remain source-derived');
assert.equal(SO101_PHASE2A_MODEL_PACKAGE.evidence.servoControllerParameters, PARAMETER_EVIDENCE.ESTIMATED, 'hardware-facing servo controller parameters must remain explicitly estimated');
assert.equal(SO101_PHASE2A_MODEL_PACKAGE.evidence.hardwareAlignment, PARAMETER_EVIDENCE.CALIBRATION_REQUIRED, 'hardware alignment must remain calibration-required');
assert.doesNotThrow(() => assertPhysicalScene(structuredClone(SO101_PHASE2A_SCENE)));
assert.equal(SO101_PHASE2A_SCENE.modelPackage, SO101_PHASE2A_MODEL_PACKAGE.id);
assert.equal(SO101_PHASE2A_SCENE.robotId, SO101_PHASE2A_MODEL_PACKAGE.robotId);

const so101Bytes = readFileSync(new URL('../../models/so101/model.xml', import.meta.url));
const so101Sha256 = createHash('sha256').update(so101Bytes).digest('hex');
assert.equal(so101Sha256, SO101_PHASE2A_MODEL_PACKAGE.sha256, 'SO-101 registered SHA-256 must identify the repository model bytes');
const so101Source = so101Bytes.toString('utf8');
for (const name of SO101_PHASE2A_MODEL_PACKAGE.joints.map(({ id }) => id)) {
  assert.ok(so101Source.includes(`name="${name}"`), `SO-101 MJCF must contain declared joint/actuator ${name}`);
}
assert.ok(!so101Source.includes('<freejoint'), 'Phase 2A SO-101 validation plant must not acquire a hidden free root');
assert.ok(
  so101Source.includes('<geom name="base_proxy" type="box" size="0.045 0.045 0.055" pos="0 0 0.025" contype="0" conaffinity="0" group="2"/>'),
  'Phase 2A diagnostic base proxy must remain visual-only and non-colliding',
);

const playwrightConfigSource = readFileSync(new URL('../../playwright.config.mjs', import.meta.url), 'utf8');
assert.ok(
  playwrightConfigSource.includes('so101-physics-browser'),
  'Playwright discovery must include tests/so101-physics-browser.spec.mjs so CI actually executes the Phase 2A browser/WASM acceptance test',
);
assert.ok(
  playwrightConfigSource.includes('physics-load-failure-browser'),
  'Playwright discovery must include tests/physics-load-failure-browser.spec.mjs so CI actually executes the physical load-failure/no-fallback regression',
);

assert.ok(Object.isFrozen(SO101_PHASE2A_MODEL_PACKAGE), 'registered package root must be immutable');
assert.ok(Object.isFrozen(SO101_PHASE2A_MODEL_PACKAGE.physics), 'registered package physics settings must be immutable');
assert.ok(Object.isFrozen(SO101_PHASE2A_MODEL_PACKAGE.joints), 'registered package joint list must be immutable');
assert.ok(Object.isFrozen(SO101_PHASE2A_MODEL_PACKAGE.joints[0]), 'registered package joint descriptors must be immutable');
assert.throws(() => { SO101_PHASE2A_MODEL_PACKAGE.physics.timestepSeconds = 0.01; }, TypeError);
assert.throws(() => { SO101_PHASE2A_MODEL_PACKAGE.joints[0].id = 'tampered_joint'; }, TypeError);

assert.throws(() => requireModelPackage('https://example.invalid/model.xml'), /Unknown physical model package/);
assert.throws(() => validateModelPackage({ ...structuredClone(PHASE1_MODEL_PACKAGE), id: 'bad-url-package', asset: 'https://example.invalid/model.xml' }), /repository-local/);
assert.throws(() => validateModelPackage({ ...structuredClone(PHASE1_MODEL_PACKAGE), id: 'bad-traversal-package', asset: 'models/../secret/model.xml' }), /repository-local/);
assert.throws(() => validateModelPackage({ ...structuredClone(PHASE1_MODEL_PACKAGE), id: 'bad-sha-package', sha256: PHASE1_MODEL_PACKAGE.sha256.toUpperCase() }), /lowercase SHA-256/);
assert.throws(() => validateModelPackage({ ...structuredClone(PHASE1_MODEL_PACKAGE), id: 'bad-integrator-package', physics: { ...PHASE1_MODEL_PACKAGE.physics, integrator: 'not-an-integrator' } }), /unsupported physics.integrator/);
assert.throws(() => validateModelPackage({ ...structuredClone(SO101_PHASE2A_MODEL_PACKAGE), id: 'bad-iterations-package', physics: { ...SO101_PHASE2A_MODEL_PACKAGE.physics, iterations: 0 } }), /iterations/);
assert.throws(() => validateModelPackage({ ...structuredClone(SO101_PHASE2A_MODEL_PACKAGE), id: 'bad-range-package', joints: SO101_PHASE2A_MODEL_PACKAGE.joints.map((joint, index) => index === 0 ? { ...joint, rangeRad: [1, -1] } : structuredClone(joint)) }), /minimum must be less than maximum/);
assert.throws(() => validateModelPackage({ ...structuredClone(SO101_PHASE2A_MODEL_PACKAGE), id: 'bad-axis-package', joints: SO101_PHASE2A_MODEL_PACKAGE.joints.map((joint, index) => index === 0 ? { ...joint, axis: [0, 0, 0] } : structuredClone(joint)) }), /zero vector/);
assert.throws(() => validateModelPackage({ ...structuredClone(SO101_PHASE2A_MODEL_PACKAGE), id: 'bad-evidence-package', actuators: SO101_PHASE2A_MODEL_PACKAGE.actuators.map((actuator, index) => index === 0 ? { ...actuator, evidence: 'authoritative-ish' } : structuredClone(actuator)) }), /unknown parameter evidence/);
assert.throws(() => validateModelPackage({ ...structuredClone(PHASE1_MODEL_PACKAGE), id: 'bad-command-package', actuators: [{ id: 'bad', jointId: 'hinge', controllerId: 'hinge_position' }] }), /requires command/);
assert.throws(() => validateModelPackage({ ...structuredClone(PHASE1_MODEL_PACKAGE), id: 'bad-actuator-package', actuators: [{ id: 'bad', jointId: 'missing', controllerId: 'hinge_position', command: 'position-rad' }] }), /unknown joint/);
console.log('Model registry/package contract checks: OK');
