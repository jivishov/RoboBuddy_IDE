export const MODEL_EVIDENCE = Object.freeze({
  MODEL_DERIVED: 'model-derived',
  NUMERICALLY_VERIFIED: 'numerically-verified',
  HARDWARE_COMPARED: 'hardware-compared',
});

export const PARAMETER_EVIDENCE = Object.freeze({
  SOURCE_DERIVED: 'source-derived',
  ESTIMATED: 'estimated',
  CALIBRATION_REQUIRED: 'calibration-required',
});

export const EXECUTION_BACKENDS = Object.freeze({
  LEGACY: 'legacy',
  BROWSER_MUJOCO: 'browser-mujoco',
  NATIVE_MUJOCO: 'native-mujoco',
});

const registry = new Map();
const MODEL_ASSET_RE = /^models\/[A-Za-z0-9._/-]+\.xml$/;
const SHA256_RE = /^[0-9a-f]{64}$/;
const MODEL_INTEGRATORS = new Set(['Euler', 'RK4', 'implicit', 'implicitfast']);
const PARAMETER_EVIDENCE_VALUES = new Set(Object.values(PARAMETER_EVIDENCE));

function stringArray(value, label) {
  if (!Array.isArray(value) || value.some((item) => typeof item !== 'string' || !item)) {
    throw new TypeError(`${label} must be an array of non-empty strings`);
  }
  if (new Set(value).size !== value.length) throw new TypeError(`${label} must contain unique values`);
  return value;
}

function finiteRange(value, label) {
  if (!Array.isArray(value) || value.length !== 2 || value.some((item) => !Number.isFinite(Number(item)))) {
    throw new TypeError(`${label} must be a two-value finite numeric range`);
  }
  if (Number(value[0]) >= Number(value[1])) throw new RangeError(`${label} minimum must be less than maximum`);
  return value;
}

function finiteAxis(value, label) {
  if (!Array.isArray(value) || value.length !== 3 || value.some((item) => !Number.isFinite(Number(item)))) {
    throw new TypeError(`${label} must be a three-value finite numeric axis`);
  }
  const normSquared = value.reduce((sum, item) => sum + Number(item) ** 2, 0);
  if (normSquared <= 0) throw new RangeError(`${label} must not be the zero vector`);
  return value;
}

function parameterEvidence(value, label) {
  if (value != null && !PARAMETER_EVIDENCE_VALUES.has(value)) throw new TypeError(`${label} has unknown parameter evidence ${value}`);
}

function validateDescriptorList(items, label, requiredKeys) {
  if (!Array.isArray(items) || items.length === 0) throw new TypeError(`${label} must be a non-empty array`);
  const ids = new Set();
  for (const [index, item] of items.entries()) {
    if (!item || typeof item !== 'object' || Array.isArray(item)) throw new TypeError(`${label}[${index}] must be an object`);
    for (const key of requiredKeys) {
      if (typeof item[key] !== 'string' || !item[key]) throw new TypeError(`${label}[${index}] requires ${key}`);
    }
    if (ids.has(item.id)) throw new TypeError(`${label} contains duplicate id ${item.id}`);
    ids.add(item.id);
  }
}

function deepFreeze(value) {
  if (!value || typeof value !== 'object' || Object.isFrozen(value)) return value;
  for (const child of Object.values(value)) deepFreeze(child);
  return Object.freeze(value);
}

export function validateModelPackage(manifest) {
  if (!manifest || typeof manifest !== 'object' || Array.isArray(manifest)) throw new TypeError('Model package must be an object');
  if (!manifest.id || !manifest.robotId || !manifest.modelId || !manifest.source?.url || !manifest.source?.revision) {
    throw new TypeError('Model package requires id, robotId, modelId, source.url, and source.revision');
  }
  if (!MODEL_ASSET_RE.test(manifest.asset || '') || String(manifest.asset).includes('..')) {
    throw new TypeError('Model package asset must be a repository-local models/*.xml path');
  }
  if (!SHA256_RE.test(String(manifest.sha256 || ''))) throw new TypeError('Model package requires a lowercase SHA-256');
  if (typeof manifest.license !== 'string' || !manifest.license) throw new TypeError('Model package requires license');
  if (!manifest.physics || !Number.isFinite(Number(manifest.physics.timestepSeconds)) || Number(manifest.physics.timestepSeconds) <= 0) {
    throw new TypeError('Model package requires a positive physics.timestepSeconds');
  }
  if (!MODEL_INTEGRATORS.has(manifest.physics.integrator)) throw new TypeError(`Model package has unsupported physics.integrator ${manifest.physics.integrator}`);
  if (manifest.physics.iterations != null && (!Number.isInteger(manifest.physics.iterations) || manifest.physics.iterations < 1)) {
    throw new RangeError('Model package physics.iterations must be a positive integer when provided');
  }
  if (manifest.physics.lsIterations != null && (!Number.isInteger(manifest.physics.lsIterations) || manifest.physics.lsIterations < 0)) {
    throw new RangeError('Model package physics.lsIterations must be a non-negative integer when provided');
  }

  validateDescriptorList(manifest.joints, 'joints', ['id']);
  for (const [index, joint] of manifest.joints.entries()) {
    if (joint.rangeRad != null) finiteRange(joint.rangeRad, `joints[${index}].rangeRad`);
    if (joint.axis != null) finiteAxis(joint.axis, `joints[${index}].axis`);
    parameterEvidence(joint.evidence, `joints[${index}].evidence`);
  }
  const jointIds = new Set(manifest.joints.map((item) => item.id));
  if (manifest.initialJointPositionsRad != null) {
    if (!manifest.initialJointPositionsRad || typeof manifest.initialJointPositionsRad !== 'object' || Array.isArray(manifest.initialJointPositionsRad)) {
      throw new TypeError('initialJointPositionsRad must be an object when provided');
    }
    for (const [jointId, raw] of Object.entries(manifest.initialJointPositionsRad)) {
      if (!jointIds.has(jointId)) throw new TypeError(`initialJointPositionsRad references unknown joint ${jointId}`);
      const value = Number(raw);
      if (!Number.isFinite(value)) throw new TypeError(`initialJointPositionsRad.${jointId} must be finite radians`);
      const range = manifest.joints.find((joint) => joint.id === jointId)?.rangeRad;
      if (range && (value < Number(range[0]) || value > Number(range[1]))) throw new RangeError(`initialJointPositionsRad.${jointId} is outside the declared joint range`);
    }
  }

  validateDescriptorList(manifest.actuators, 'actuators', ['id', 'jointId', 'controllerId', 'command']);
  for (const [index, actuator] of manifest.actuators.entries()) {
    if (!jointIds.has(actuator.jointId)) throw new TypeError(`Actuator ${actuator.id} maps unknown joint ${actuator.jointId}`);
    if (actuator.controlRangeRad != null) finiteRange(actuator.controlRangeRad, `actuators[${index}].controlRangeRad`);
    parameterEvidence(actuator.evidence, `actuators[${index}].evidence`);
  }

  if (manifest.initialJointPositionsRad != null) {
    const actuatorByJoint = new Map(manifest.actuators.filter((actuator) => actuator.command === 'position-rad').map((actuator) => [actuator.jointId, actuator]));
    for (const [jointId, raw] of Object.entries(manifest.initialJointPositionsRad)) {
      const actuator = actuatorByJoint.get(jointId);
      if (!actuator) continue;
      const value = Number(raw);
      if (actuator.controlRangeRad && (value < Number(actuator.controlRangeRad[0]) || value > Number(actuator.controlRangeRad[1]))) {
        throw new RangeError(`initialJointPositionsRad.${jointId} is outside actuator ${actuator.id} control range`);
      }
    }
  }

  if (manifest.mechanicalCouplings != null) {
    if (!Array.isArray(manifest.mechanicalCouplings)) throw new TypeError('mechanicalCouplings must be an array when provided');
    const ids = new Set();
    for (const [index, coupling] of manifest.mechanicalCouplings.entries()) {
      if (!coupling || typeof coupling !== 'object' || Array.isArray(coupling)) throw new TypeError(`mechanicalCouplings[${index}] must be an object`);
      for (const key of ['id', 'driverJointId', 'followerJointId']) {
        if (typeof coupling[key] !== 'string' || !coupling[key]) throw new TypeError(`mechanicalCouplings[${index}] requires ${key}`);
      }
      if (ids.has(coupling.id)) throw new TypeError(`mechanicalCouplings contains duplicate id ${coupling.id}`);
      ids.add(coupling.id);
      if (!jointIds.has(coupling.driverJointId)) throw new TypeError(`Mechanical coupling ${coupling.id} driver must be a declared controllable joint`);
      if (jointIds.has(coupling.followerJointId)) throw new TypeError(`Mechanical coupling ${coupling.id} follower must remain a passive source-model joint, not a second command surface`);
      const multiplier = Number(coupling.multiplier ?? 1);
      const offsetRad = Number(coupling.offsetRad ?? 0);
      if (!Number.isFinite(multiplier) || !Number.isFinite(offsetRad)) throw new TypeError(`Mechanical coupling ${coupling.id} requires finite multiplier/offsetRad`);
      parameterEvidence(coupling.evidence, `mechanicalCouplings[${index}].evidence`);
    }
  }

  if (manifest.bodies != null) validateDescriptorList(manifest.bodies, 'bodies', ['id']);
  stringArray(manifest.controllers, 'controllers');
  for (const actuator of manifest.actuators) {
    if (!manifest.controllers.includes(actuator.controllerId)) throw new TypeError(`Actuator ${actuator.id} uses undeclared controller ${actuator.controllerId}`);
  }
  if (manifest.sceneConstraints?.fixtures != null) stringArray(manifest.sceneConstraints.fixtures, 'sceneConstraints.fixtures');
  if (manifest.sceneConstraints?.objects != null) stringArray(manifest.sceneConstraints.objects, 'sceneConstraints.objects');
  if (manifest.limitations != null) stringArray(manifest.limitations, 'limitations');
  if (manifest.evidence != null) {
    if (typeof manifest.evidence !== 'object' || Array.isArray(manifest.evidence)) throw new TypeError('evidence must be an object when provided');
    for (const [key, value] of Object.entries(manifest.evidence)) parameterEvidence(value, `evidence.${key}`);
  }
  return manifest;
}

export function registerModelPackage(manifest) {
  validateModelPackage(manifest);
  if (registry.has(manifest.id)) throw new Error(`Duplicate model package: ${manifest.id}`);
  const frozen = deepFreeze(structuredClone(manifest));
  registry.set(frozen.id, frozen);
  return frozen;
}

export function getModelPackage(id) {
  return registry.get(id) || null;
}

export function requireModelPackage(id) {
  const modelPackage = getModelPackage(id);
  if (!modelPackage) throw new Error(`Unknown physical model package: ${id}`);
  return modelPackage;
}

export function listModelPackages() {
  return [...registry.values()];
}

export function capabilityRecord({ backend, capability, evidence, limitations = [] }) {
  if (!Object.values(EXECUTION_BACKENDS).includes(backend)) throw new TypeError(`Unknown backend: ${backend}`);
  if (!Object.values(MODEL_EVIDENCE).includes(evidence)) throw new TypeError(`Unknown evidence label: ${evidence}`);
  return Object.freeze({ backend, capability, evidence, limitations: [...limitations] });
}

// Only disposable in-memory workcells may leave the registry. Bundled packages are immutable.
export function unregisterTransientModelPackage(id) {
  const entry = registry.get(id);
  if (!entry) return false;
  if (entry.transient !== true) throw new Error('Bundled model packages cannot be unregistered');
  return registry.delete(id);
}
