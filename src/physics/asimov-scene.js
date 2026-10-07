import { PHYSICS_BACKEND_API_VERSION } from './backend-contract.js';
import { ASIMOV_ALL_PACKAGES } from './asimov-model-package.js';
export const ASIMOV_SCENES=Object.freeze(Object.fromEntries(Object.entries(ASIMOV_ALL_PACKAGES).map(([variant,p])=>[variant,Object.freeze({
  schemaVersion:PHYSICS_BACKEND_API_VERSION,id:`asimov-${variant}`,revision:`asimov-${variant}-v1`,
  robotId:p.robotId,modelPackage:p.id,legacyTaskId:null,physics:{...p.physics},
  fixtures:p.sceneConstraints.fixtures.map(id=>({id})),objects:[],controllers:[...p.controllers],
  taskGoal:{type:'physical-observation-laboratory',syntheticSuccessEvents:false},
})])));
export function asimovSceneById(id) {
  const scene=Object.values(ASIMOV_SCENES).find(s=>s.id===id);
  if(!scene) throw new RangeError(`Unknown physical Asimov scene ${id}`);
  return scene;
}
