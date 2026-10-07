import { PHYSICS_BACKEND_API_VERSION } from './backend-contract.js';

export const PHASE1_SCENE = Object.freeze({
  schemaVersion: PHYSICS_BACKEND_API_VERSION,
  id: 'phase1-vertical-slice',
  revision: 'phase1-vertical-slice-v1',
  robotId: 'phase1_articulated_joint',
  modelPackage: 'phase1-vertical-slice',
  legacyTaskId: null,
  physics: Object.freeze({
    timestepSeconds: 0.002,
    integrator: 'RK4',
  }),
  fixtures: Object.freeze([
    Object.freeze({ id: 'floor', role: 'support-plane' }),
    Object.freeze({ id: 'table', role: 'support-surface' }),
  ]),
  objects: Object.freeze([
    Object.freeze({ id: 'free_box', dynamic: true, role: 'gravity-and-contact-probe' }),
  ]),
  controllers: Object.freeze(['hinge_position']),
  taskGoal: null,
});
