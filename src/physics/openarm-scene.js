import { PHYSICS_BACKEND_API_VERSION } from './backend-contract.js';
import { OPENARM_V2_PHASE5A_MODEL_PACKAGE } from './openarm-model-package.js';

export const OPENARM_V2_PHASE5A_SCENE = Object.freeze({
  schemaVersion: PHYSICS_BACKEND_API_VERSION,
  id: 'phase5a-openarm-v2-bimanual-stack',
  revision: 'phase5a-openarm-v2-bimanual-stack-v3',
  robotId: OPENARM_V2_PHASE5A_MODEL_PACKAGE.robotId,
  modelPackage: OPENARM_V2_PHASE5A_MODEL_PACKAGE.id,
  legacyTaskId: 'openarm-04-filtration-workcell',
  physics: Object.freeze({
    timestepSeconds: OPENARM_V2_PHASE5A_MODEL_PACKAGE.physics.timestepSeconds,
    integrator: OPENARM_V2_PHASE5A_MODEL_PACKAGE.physics.integrator,
  }),
  fixtures: Object.freeze(OPENARM_V2_PHASE5A_MODEL_PACKAGE.sceneConstraints.fixtures.map((id) => Object.freeze({ id }))),
  objects: Object.freeze(OPENARM_V2_PHASE5A_MODEL_PACKAGE.sceneConstraints.objects.map((id) => Object.freeze({ id }))),
  controllers: Object.freeze([...OPENARM_V2_PHASE5A_MODEL_PACKAGE.controllers]),
  taskGoal: Object.freeze({
    type: 'openarm-v2-bimanual-dry-stack',
    order: Object.freeze(['flask', 'beaker']),
    flask: Object.freeze({
      objectId: 'flask',
      side: 'left',
      gripperGeoms: Object.freeze(['finger_inner_left_collision_', 'finger_outer_left_collision_']),
      objectGeoms: Object.freeze(['flask_body_geom', 'flask_shoulder_geom', 'flask_grip_geom']),
      supportGeom: 'left_hotplate',
      targetCenterXYM: Object.freeze([0.67, 0.1535]),
      targetHalfExtentsXYM: Object.freeze([0.017, 0.013]),
      initialBodyZM: 1.092,
    }),
    beaker: Object.freeze({
      objectId: 'beaker',
      side: 'right',
      gripperGeoms: Object.freeze(['finger_inner_right_collision_', 'finger_outer_right_collision_']),
      objectGeoms: Object.freeze(['beaker_grip_geom']),
      supportGeom: 'right_ring_gauze',
      targetCenterXYM: Object.freeze([0.67, -0.1535]),
      targetHalfExtentsXYM: Object.freeze([0.021, 0.021]),
      initialBodyZM: 1.105,
    }),
    minimumGraspSeconds: 0.06,
    maximumPenetrationM: 0.002,
    liftClearanceM: 0.020,
    carryHorizontalM: 0.060,
    settleSeconds: 0.20,
    maxSettleDriftM: 0.001,
    maxSettleLinearSpeedMS: 0.035,
    maxSettleAngularSpeedRadS: 0.8,
    retreatDistanceM: 0.050,
  }),
});

export const OPENARM_V2_BIMANUAL_CONTROLLER = Object.freeze({
  id: "openarm-v2-bimanual-stack-v3",
  controllerPeriodSeconds: 0.02,
  stages: Object.freeze([
  {
    "name": "settle_initial",
    "targetsRad": {},
    "durationSeconds": 0.5,
    "label": "Settle both vessels on their source supports"
  },
  {
    "name": "left_close",
    "targetsRad": {
      "openarm_left_finger_joint1": 0.1
    },
    "durationSeconds": 2.5,
    "label": "Close slowly until bilateral contact is established"
  },
  {
    "name": "left_lift",
    "targetsRad": {
      "openarm_left_joint1": -0.05253966506920369,
      "openarm_left_joint2": 0,
      "openarm_left_joint3": 0,
      "openarm_left_joint4": 1.8468042348970615,
      "openarm_left_joint5": 0,
      "openarm_left_joint6": -0.3285475731713685,
      "openarm_left_joint7": 0,
      "openarm_left_finger_joint1": 0.1
    },
    "durationSeconds": 1.4,
    "label": "Lift the contacted dry vessel"
  },
  {
    "name": "left_transfer",
    "targetsRad": {
      "openarm_left_joint1": -0.5905285334129174,
      "openarm_left_joint2": 0,
      "openarm_left_joint3": 0,
      "openarm_left_joint4": 1.132438798768539,
      "openarm_left_joint5": 0,
      "openarm_left_joint6": -0.1521710053865597,
      "openarm_left_joint7": 0,
      "openarm_left_finger_joint1": 0.1
    },
    "durationSeconds": 1.8,
    "label": "Carry above the receiving support"
  },
  {
    "name": "left_lower",
    "targetsRad": {
      "openarm_left_joint1": -0.5815591182839464,
      "openarm_left_joint2": 0,
      "openarm_left_joint3": 0,
      "openarm_left_joint4": 0.90038694388924,
      "openarm_left_joint5": 0,
      "openarm_left_joint6": 0.0888502646217102,
      "openarm_left_joint7": 0,
      "openarm_left_finger_joint1": 0.1
    },
    "durationSeconds": 1.4,
    "label": "Lower onto the support"
  },
  {
    "name": "left_support",
    "targetsRad": {},
    "durationSeconds": 0.3,
    "label": "Hold for support contact before releasing"
  },
  {
    "name": "left_release",
    "targetsRad": {
      "openarm_left_finger_joint1": 0.65
    },
    "durationSeconds": 2.5,
    "label": "Open the gripper slowly"
  },
  {
    "name": "left_settle",
    "targetsRad": {},
    "durationSeconds": 0.5,
    "label": "Allow the free vessel to settle"
  },
  {
    "name": "left_retreat",
    "targetsRad": {
      "openarm_left_joint1": -0.05253966506920369,
      "openarm_left_joint2": 0,
      "openarm_left_joint3": 0,
      "openarm_left_joint4": 1.8468042348970615,
      "openarm_left_joint5": 0,
      "openarm_left_joint6": -0.3285475731713685,
      "openarm_left_joint7": 0,
      "openarm_left_finger_joint1": 0.65
    },
    "durationSeconds": 2.5,
    "label": "Retreat clear of the released vessel"
  },
  {
    "name": "right_close",
    "targetsRad": {
      "openarm_right_finger_joint1": -0.1
    },
    "durationSeconds": 2.5,
    "label": "Close slowly until bilateral contact is established"
  },
  {
    "name": "right_lift",
    "targetsRad": {
      "openarm_right_joint1": 0.05253966506920369,
      "openarm_right_joint2": 0,
      "openarm_right_joint3": 0,
      "openarm_right_joint4": 1.8468042348970615,
      "openarm_right_joint5": 0,
      "openarm_right_joint6": 0.3285475731713685,
      "openarm_right_joint7": 0,
      "openarm_right_finger_joint1": -0.1
    },
    "durationSeconds": 1.4,
    "label": "Lift the contacted dry vessel"
  },
  {
    "name": "right_transfer",
    "targetsRad": {
      "openarm_right_joint1": 0.5905285334129174,
      "openarm_right_joint2": 0,
      "openarm_right_joint3": 0,
      "openarm_right_joint4": 1.132438798768539,
      "openarm_right_joint5": 0,
      "openarm_right_joint6": 0.1521710053865597,
      "openarm_right_joint7": 0,
      "openarm_right_finger_joint1": -0.1
    },
    "durationSeconds": 1.8,
    "label": "Carry above the receiving support"
  },
  {
    "name": "right_lower",
    "targetsRad": {
      "openarm_right_joint1": 0.5815591182839464,
      "openarm_right_joint2": 0,
      "openarm_right_joint3": 0,
      "openarm_right_joint4": 0.90038694388924,
      "openarm_right_joint5": 0,
      "openarm_right_joint6": -0.0888502646217102,
      "openarm_right_joint7": 0,
      "openarm_right_finger_joint1": -0.1
    },
    "durationSeconds": 1.4,
    "label": "Lower onto the support"
  },
  {
    "name": "right_support",
    "targetsRad": {},
    "durationSeconds": 0.3,
    "label": "Hold for support contact before releasing"
  },
  {
    "name": "right_release",
    "targetsRad": {
      "openarm_right_finger_joint1": -0.65
    },
    "durationSeconds": 2.5,
    "label": "Open the gripper slowly"
  },
  {
    "name": "right_settle",
    "targetsRad": {},
    "durationSeconds": 0.5,
    "label": "Allow the free vessel to settle"
  },
  {
    "name": "right_retreat",
    "targetsRad": {
      "openarm_right_joint1": 0.05253966506920369,
      "openarm_right_joint2": 0,
      "openarm_right_joint3": 0,
      "openarm_right_joint4": 1.8468042348970615,
      "openarm_right_joint5": 0,
      "openarm_right_joint6": 0.3285475731713685,
      "openarm_right_joint7": 0,
      "openarm_right_finger_joint1": -0.65
    },
    "durationSeconds": 2.5,
    "label": "Retreat clear of the released vessel"
  }
].map(stage => Object.freeze({...stage, targetsRad: Object.freeze(stage.targetsRad)}))),
  referencePoses: Object.freeze({ sourceHomeEeM: OPENARM_V2_PHASE5A_MODEL_PACKAGE.benchmark.sourceHomeEeM }),
});
