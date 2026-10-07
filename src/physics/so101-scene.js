import { PHYSICS_BACKEND_API_VERSION } from './backend-contract.js';
import { SO101_MANIPULATION_MODEL_PACKAGE, SO101_PHASE2A_MODEL_PACKAGE } from './model-packages.js';

export const SO101_PHASE2A_SCENE = Object.freeze({
  schemaVersion: PHYSICS_BACKEND_API_VERSION,
  id: 'phase2a-so101-articulation',
  revision: 'phase2a-so101-articulation-v1',
  robotId: SO101_PHASE2A_MODEL_PACKAGE.robotId,
  modelPackage: SO101_PHASE2A_MODEL_PACKAGE.id,
  legacyTaskId: null,
  physics: Object.freeze({
    timestepSeconds: SO101_PHASE2A_MODEL_PACKAGE.physics.timestepSeconds,
    integrator: SO101_PHASE2A_MODEL_PACKAGE.physics.integrator,
    iterations: SO101_PHASE2A_MODEL_PACKAGE.physics.iterations,
    lsIterations: SO101_PHASE2A_MODEL_PACKAGE.physics.lsIterations,
  }),
  fixtures: Object.freeze([]),
  objects: Object.freeze([]),
  controllers: Object.freeze([...SO101_PHASE2A_MODEL_PACKAGE.controllers]),
  taskGoal: null,
});

export const SO101_MANIPULATION_SCENE = Object.freeze({
  schemaVersion: PHYSICS_BACKEND_API_VERSION,
  id: 'p4-so101-benchmark-transfer',
  revision: 'p4-so101-benchmark-transfer-v3',
  robotId: SO101_MANIPULATION_MODEL_PACKAGE.robotId,
  modelPackage: SO101_MANIPULATION_MODEL_PACKAGE.id,
  legacyTaskId: null,
  physics: Object.freeze({
    timestepSeconds: SO101_MANIPULATION_MODEL_PACKAGE.physics.timestepSeconds,
    integrator: SO101_MANIPULATION_MODEL_PACKAGE.physics.integrator,
    iterations: SO101_MANIPULATION_MODEL_PACKAGE.physics.iterations,
    lsIterations: SO101_MANIPULATION_MODEL_PACKAGE.physics.lsIterations,
  }),
  fixtures: Object.freeze([
    Object.freeze({ id: 'benchmark_source_support' }),
    Object.freeze({ id: 'benchmark_target_support' }),
    Object.freeze({ id: 'benchmark_target_region' }),
  ]),
  objects: Object.freeze([Object.freeze({ id: 'benchmark_block' })]),
  controllers: Object.freeze([...SO101_MANIPULATION_MODEL_PACKAGE.controllers]),
  taskGoal: Object.freeze({
    type: 'physical-block-transfer',
    objectId: 'benchmark_block',
    targetFrame: 'mujoco_world',
    targetCenterXYM: Object.freeze([0.358, -0.156]),
    targetHalfExtentsXYM: Object.freeze([0.020, 0.030]),
    requireContact: true,
    requireLift: true,
    requireCarry: true,
    requireRelease: true,
    requireFinalRest: true,
  }),
});

export const SO101_BENCHMARK_TRANSFER_CONTROLLER = Object.freeze({
  id: 'so101-benchmark-transfer-v1',
  controllerPeriodSeconds: 0.02,
  stages: Object.freeze([
    Object.freeze({ name: 'settle_initial', durationSeconds: 0.20, targetsRad: Object.freeze({}) }),
    Object.freeze({ name: 'approach', durationSeconds: 0.60, targetsRad: Object.freeze({ shoulder_pan: 0.005, shoulder_lift: 0.0 }) }),
    Object.freeze({ name: 'close', durationSeconds: 0.50, targetsRad: Object.freeze({ gripper: -0.04 }) }),
    Object.freeze({ name: 'lift', durationSeconds: 0.80, targetsRad: Object.freeze({ shoulder_lift: -0.35 }) }),
    Object.freeze({ name: 'move', durationSeconds: 0.90, targetsRad: Object.freeze({ shoulder_pan: 0.45 }) }),
    Object.freeze({ name: 'lower', durationSeconds: 0.80, targetsRad: Object.freeze({ shoulder_lift: 0.0 }) }),
    Object.freeze({ name: 'release', durationSeconds: 0.50, targetsRad: Object.freeze({ gripper: 0.60 }) }),
    Object.freeze({ name: 'settle_final', durationSeconds: 0.80, targetsRad: Object.freeze({}) }),
  ]),
});
