export const PHYSICS_BACKEND_API_VERSION = '0.2.0-alpha.1';

export const PhysicsBackendState = Object.freeze({
  IDLE: 'idle',
  LOADING: 'loading',
  READY: 'ready',
  RUNNING: 'running',
  PAUSED: 'paused',
  FAILED: 'failed',
  DISPOSED: 'disposed',
});

const SCENE_KEYS = new Set([
  'schemaVersion', 'id', 'revision', 'robotId', 'modelPackage', 'legacyTaskId',
  'physics', 'fixtures', 'objects', 'controllers', 'taskGoal',
]);
const PHYSICS_KEYS = new Set(['timestepSeconds', 'integrator', 'iterations', 'lsIterations']);
const INTEGRATORS = new Set(['Euler', 'RK4', 'implicit', 'implicitfast']);

// A backend may optionally advance many authoritative physics steps in one request and
// return the ground-truth observations captured at a declared step cadence. This keeps a
// fine observation resolution affordable without one cross-thread round trip per sample.
// Backends without the capability keep their original single-observation advanceSteps.
export const SAMPLED_ADVANCE_METHOD = 'advanceStepsObserved';
export const MAX_ADVANCE_STEPS_PER_REQUEST = 100000;
export const MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE = 2048;

export function supportsSampledAdvance(backend) {
  return typeof backend?.[SAMPLED_ADVANCE_METHOD] === 'function';
}

export function sampledObservationCount(stepCount, sampleEverySteps) {
  if (!Number.isInteger(stepCount) || stepCount < 1) throw new RangeError('stepCount must be a positive integer');
  if (!Number.isInteger(sampleEverySteps) || sampleEverySteps < 1) throw new RangeError('sampleEverySteps must be a positive integer');
  const aligned = Math.floor(stepCount / sampleEverySteps);
  // The true post-request state is always the last sample, even when the requested step
  // count is not a whole number of sampling periods.
  return aligned * sampleEverySteps === stepCount ? aligned : aligned + 1;
}

export function assertSampledObservations(result, { stepCount, sampleEverySteps, previousSimulationTimeSeconds = null } = {}) {
  if (!result || typeof result !== 'object' || Array.isArray(result)) throw new TypeError('Sampled advance must return a result object');
  if (!Number.isInteger(result.executedSteps) || result.executedSteps < 0) throw new TypeError('Sampled advance must report an integer executedSteps count');
  if (result.executedSteps > stepCount) throw new RangeError(`Sampled advance executed ${result.executedSteps} steps for a request of ${stepCount}`);
  const observations = result.observations;
  if (!Array.isArray(observations) || !observations.length) throw new TypeError('Sampled advance must return at least one authoritative observation');
  const maximum = sampledObservationCount(stepCount, sampleEverySteps);
  if (observations.length > maximum) throw new RangeError(`Sampled advance returned ${observations.length} observations for a declared ${maximum}-sample cadence`);
  let cursor = previousSimulationTimeSeconds == null ? null : Number(previousSimulationTimeSeconds);
  for (const observation of observations) {
    const time = Number(observation?.simulationTimeSeconds);
    if (!Number.isFinite(time) || time < 0) throw new TypeError('Every sampled observation must carry a finite simulation time');
    if (cursor != null) {
      const ordered = result.executedSteps === 0 ? time === cursor : time > cursor;
      if (!ordered) throw new RangeError('Sampled observations must be ordered by strictly increasing simulation time');
    }
    cursor = time;
  }
  const finalObservation = result.finalObservation ?? observations[observations.length - 1];
  if (finalObservation !== observations[observations.length - 1]) {
    throw new RangeError('The final sampled observation must be the last returned sample');
  }
  return { observations, finalObservation };
}

export function assertPhysicsBackend(backend) {
  const required = [
    'loadScene',
    'reset',
    'acceptCommand',
    'advanceSteps',
    'getObservation',
    'getDiagnostics',
    'pause',
    'resume',
    'cancelRun',
    'exportTrace',
    'dispose',
  ];
  const missing = required.filter((name) => typeof backend?.[name] !== 'function');
  if (missing.length) throw new TypeError(`Invalid physics backend: missing ${missing.join(', ')}`);
  return backend;
}

export function assertPhysicalScene(scene) {
  if (!scene || typeof scene !== 'object' || Array.isArray(scene)) throw new TypeError('Physical scene must be an object');
  for (const key of Object.keys(scene)) {
    if (!SCENE_KEYS.has(key)) throw new TypeError(`Unknown physical scene field: ${key}`);
  }
  for (const key of ['schemaVersion', 'id', 'revision', 'robotId', 'modelPackage']) {
    if (typeof scene[key] !== 'string' || !scene[key]) throw new TypeError(`Physical scene requires non-empty ${key}`);
  }
  if (scene.schemaVersion !== PHYSICS_BACKEND_API_VERSION) {
    throw new Error(`Unsupported physical scene schema ${scene.schemaVersion}`);
  }
  if (scene.legacyTaskId != null && (typeof scene.legacyTaskId !== 'string' || !scene.legacyTaskId)) {
    throw new TypeError('legacyTaskId must be null or a non-empty string');
  }
  if (!scene.physics || typeof scene.physics !== 'object' || Array.isArray(scene.physics)) {
    throw new TypeError('Physical scene requires physics settings');
  }
  for (const key of Object.keys(scene.physics)) {
    if (!PHYSICS_KEYS.has(key)) throw new TypeError(`Unknown physics setting: ${key}`);
  }
  const timestep = Number(scene.physics.timestepSeconds);
  if (!Number.isFinite(timestep) || timestep <= 0) throw new RangeError('physics.timestepSeconds must be positive');
  if (!INTEGRATORS.has(scene.physics.integrator)) throw new TypeError(`Unsupported physics integrator: ${scene.physics.integrator}`);
  if (scene.physics.iterations != null && (!Number.isInteger(scene.physics.iterations) || scene.physics.iterations < 1)) {
    throw new RangeError('physics.iterations must be a positive integer when provided');
  }
  if (scene.physics.lsIterations != null && (!Number.isInteger(scene.physics.lsIterations) || scene.physics.lsIterations < 0)) {
    throw new RangeError('physics.lsIterations must be a non-negative integer when provided');
  }
  for (const key of ['fixtures', 'objects', 'controllers']) {
    if (scene[key] != null && !Array.isArray(scene[key])) throw new TypeError(`${key} must be an array when provided`);
  }
  for (const key of ['fixtures', 'objects']) {
    for (const [index, item] of (scene[key] || []).entries()) {
      if (!item || typeof item !== 'object' || Array.isArray(item)) throw new TypeError(`${key}[${index}] must be an object`);
    }
  }
  if (Array.isArray(scene.controllers)) {
    if (scene.controllers.some((value) => typeof value !== 'string' || !value)) throw new TypeError('controllers must contain non-empty strings');
    if (new Set(scene.controllers).size !== scene.controllers.length) throw new TypeError('controllers must contain unique values');
  }
  if (scene.taskGoal != null && (typeof scene.taskGoal !== 'object' || Array.isArray(scene.taskGoal))) {
    throw new TypeError('taskGoal must be null or an object');
  }
  return scene;
}

export function makeCommandEnvelope({ sessionId, epoch, commandId, sceneRevision, robotId, command, maxSteps = null }) {
  if (!sessionId || !commandId || !sceneRevision || !robotId || !command?.type) {
    throw new TypeError('Incomplete physics command envelope');
  }
  if (!Number.isInteger(epoch) || epoch < 1) throw new RangeError('Physics command epoch must be a positive integer');
  if (maxSteps != null && (!Number.isInteger(maxSteps) || maxSteps < 1)) {
    throw new RangeError('maxSteps must be null or a positive integer');
  }
  return Object.freeze({
    schemaVersion: PHYSICS_BACKEND_API_VERSION,
    sessionId: String(sessionId),
    epoch,
    commandId: String(commandId),
    sceneRevision: String(sceneRevision),
    robotId: String(robotId),
    command: structuredClone(command),
    maxSteps,
  });
}
