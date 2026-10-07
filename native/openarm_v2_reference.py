#!/usr/bin/env python3
"""Native MuJoCo implementation-comparison reference for RoboBuddy OpenArm V2 Phase 5A.

Setup writes only the declared initial robot qpos/ctrl values plus the passive
finger qpos required to place the source-defined mechanical coupling on its
constraint manifold. Ordinary task execution writes actuator controls and
advances MuJoCo. Free flask/beaker qpos are never written in the nominal run.
Negative profiles explicitly alter test parameters before execution and are
reported as such.

This is native/browser model-implementation evidence, not hardware validation.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / 'models' / 'openarm_v2' / 'manipulation.xml'
JOINTS = [
    *(f'openarm_left_joint{i}' for i in range(1, 8)), 'openarm_left_finger_joint1',
    *(f'openarm_right_joint{i}' for i in range(1, 8)), 'openarm_right_finger_joint1',
]
PASSIVE_COUPLED_JOINTS = {
    'openarm_left_finger_joint2': 'openarm_left_finger_joint1',
    'openarm_right_finger_joint2': 'openarm_right_finger_joint1',
}
ACTUATORS = [
    *(f'left_joint{i}_ctrl' for i in range(1, 8)), 'left_finger1_ctrl',
    *(f'right_joint{i}_ctrl' for i in range(1, 8)), 'right_finger1_ctrl',
]
INITIAL = {
    **{f'openarm_left_joint{i}': 0.0 for i in range(1, 8)},
    'openarm_left_joint1': -0.004216999605119041,
    'openarm_left_joint4': 1.6592956429571402,
    'openarm_left_joint6': -0.09271631576736272,
    'openarm_left_finger_joint1': 0.65,
    **{f'openarm_right_joint{i}': 0.0 for i in range(1, 8)},
    'openarm_right_joint1': 0.004216999605119041,
    'openarm_right_joint4': 1.6592956429571402,
    'openarm_right_joint6': 0.09271631576736272,
    'openarm_right_finger_joint1': -0.65,
}


def d2r(value):
    return math.radians(value)


def arm(side, values, finger=None):
    out = {f'openarm_{side}_joint{i + 1}': d2r(value) for i, value in enumerate(values)}
    if finger is not None:
        out[f'openarm_{side}_finger_joint1'] = float(finger)
    return out


# Shared reference trajectory specification; servo and evaluation below are native implementations.
# This runner checks numerical conformance, not independent hardware truth.
REFERENCE_PATH = ROOT / 'models' / 'openarm_v2' / 'reference-controller.json'
STAGES = [(s['name'], s['targetsRad'], s['durationSeconds']) for s in json.loads(REFERENCE_PATH.read_text())['stages']]

TARGETS = {
    'flask': {
        'center': np.array([0.67, 0.1535]),
        'half': np.array([0.017, 0.013]),
        'support': 'left_hotplate',
        'initial_z': 1.092,
        'geoms': {'flask_body_geom', 'flask_shoulder_geom', 'flask_grip_geom'},
        'fingers': {'finger_inner_left_collision_', 'finger_outer_left_collision_'},
        'side': 'left',
    },
    'beaker': {
        'center': np.array([0.67, -0.1535]),
        'half': np.array([0.021, 0.021]),
        'support': 'right_ring_gauze',
        'initial_z': 1.105,
        'geoms': {'beaker_grip_geom'},
        'fingers': {'finger_inner_right_collision_', 'finger_outer_right_collision_'},
        'side': 'right',
    },
}
CRITERIA = {
    'lift_clearance_m': 0.020,
    'carry_horizontal_m': 0.060,
    'settle_seconds': 0.20,
    'max_settle_drift_m': 0.001,
    'max_settle_linear_speed_m_s': 0.035,
    'max_settle_angular_speed_rad_s': 0.8,
    'retreat_distance_m': 0.050,
    'support_height_tolerance_m': 0.015,
}


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def canonical_json_hash(value) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(',', ':')).encode()
    return sha256_bytes(payload)


def controller_descriptor():
    return {
        'initialJointPositionsRad': INITIAL,
        'passiveMechanicalCouplings': PASSIVE_COUPLED_JOINTS,
        'stages': [
            {'name': name, 'targetsRad': targets, 'durationSeconds': duration}
            for name, targets, duration in STAGES
        ],
        'criteria': CRITERIA,
        'targets': {
            key: {
                'centerXYM': spec['center'].tolist(),
                'halfExtentsXYM': spec['half'].tolist(),
                'supportGeom': spec['support'],
                'initialBodyZM': spec['initial_z'],
                'objectGeoms': sorted(spec['geoms']),
                'gripperGeoms': sorted(spec['fingers']),
                'side': spec['side'],
            }
            for key, spec in TARGETS.items()
        },
    }


def name(model, typ, idx):
    return mujoco.mj_id2name(model, typ, int(idx)) or f'id:{idx}'


def ids(model):
    joints = {joint: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint) for joint in JOINTS}
    acts = {joint: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator) for joint, actuator in zip(JOINTS, ACTUATORS)}
    bodies = {
        body: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
        for body in ('flask', 'beaker', 'openarm_left_ee_base_link', 'openarm_right_ee_base_link')
    }
    free = {obj: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'{obj}_free') for obj in ('flask', 'beaker')}
    assert min(*joints.values(), *acts.values(), *bodies.values(), *free.values()) >= 0
    return joints, acts, bodies, free


def setup(model, data, joints, acts, trial):
    mujoco.mj_resetData(model, data)
    for joint, value in INITIAL.items():
        data.qpos[int(model.jnt_qposadr[joints[joint]])] = value
        data.ctrl[acts[joint]] = value
    # Source equality constraints mechanically couple finger2 to finger1. Setup
    # must begin on that constraint manifold; the passive followers remain
    # non-commandable during ordinary execution.
    for follower, driver in PASSIVE_COUPLED_JOINTS.items():
        follower_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, follower)
        assert follower_id >= 0
        driver_value = float(data.qpos[int(model.jnt_qposadr[joints[driver]])])
        data.qpos[int(model.jnt_qposadr[follower_id])] = driver_value
    # Explicit negative setup intervention only; nominal free-object qpos is untouched.
    if trial == 'miss-left':
        free_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'flask_free')
        qpos_address = int(model.jnt_qposadr[free_joint])
        data.qpos[qpos_address + 1] += 0.05
    mujoco.mj_forward(model, data)


def contacts(model, data):
    out = []
    for index in range(data.ncon):
        contact = data.contact[index]
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, force)
        if force[0] <= 0.01 or contact.dist < -0.002: continue
        out.append((
            name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1),
            name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2),
        ))
    return out


def object_contact_state(model, data, obj, pairs=None):
    spec = TARGETS[obj]
    pairs = contacts(model, data) if pairs is None else pairs
    touched = set()
    support = False
    for first, second in pairs:
        first = next((prefix for prefix in spec['fingers'] if first.startswith(prefix)), first)
        second = next((prefix for prefix in spec['fingers'] if second.startswith(prefix)), second)
        if first in spec['geoms'] and second in spec['fingers']:
            touched.add(second)
        if second in spec['geoms'] and first in spec['fingers']:
            touched.add(first)
        if (first in spec['geoms'] and second == spec['support']) or (second in spec['geoms'] and first == spec['support']):
            support = True
    return touched, support


def body_position(data, body_id):
    return np.array(data.xpos[body_id], dtype=float)


def initial_evidence(model, data, joints, bodies):
    passive = {}
    for follower in PASSIVE_COUPLED_JOINTS:
        follower_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, follower)
        passive[follower] = float(data.qpos[int(model.jnt_qposadr[follower_id])])
    return {
        'simulationTimeSeconds': float(data.time),
        'jointsRad': {
            joint: float(data.qpos[int(model.jnt_qposadr[joint_id])])
            for joint, joint_id in joints.items()
        },
        'passiveCoupledJointsRad': passive,
        'bodiesM': {body: body_position(data, body_id).tolist() for body, body_id in bodies.items()},
    }


def run_trial(trial='nominal', timestep=None):
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    if timestep is not None:
        model.opt.timestep = float(timestep)
    data = mujoco.MjData(model)
    joints, acts, bodies, free = ids(model)

    # Negative-only parameter profiles are explicit and never used by nominal execution.
    if trial == 'weak-grip':
        for key in ('openarm_left_finger_joint1', 'openarm_right_finger_joint1'):
            actuator_id = acts[key]
            model.actuator_forcerange[actuator_id, :] = (-0.02, 0.02)
            model.actuator_forcelimited[actuator_id] = 1
    if trial == 'blocked-left':
        actuator_id = acts['openarm_left_joint1']
        model.actuator_forcerange[actuator_id, :] = (-0.001, 0.001)
        model.actuator_forcelimited[actuator_id] = 1

    setup(model, data, joints, acts, trial)
    equality_types = [int(value) for value in model.eq_type]
    weld_code = int(mujoco.mjtEq.mjEQ_WELD)
    assert weld_code not in equality_types, equality_types
    assert len(equality_types) == 2, equality_types

    initial_state = initial_evidence(model, data, joints, bodies)
    home_ee = {
        'left': body_position(data, bodies['openarm_left_ee_base_link']).tolist(),
        'right': body_position(data, bodies['openarm_right_ee_base_link']).tolist(),
    }
    state = {
        obj: {
            'initial': body_position(data, bodies[obj]),
            'max_z': float(data.xpos[bodies[obj]][2]),
            'contact_seen': set(),
            'grasp': False,
            'lift': False,
            'carry_anchor': None,
            'carry': False,
            'support_while_held': False,
            'support_while_held_time': None,
            'previous_held': False,
            'release': False,
            'release_time': None,
            'settle_candidate': None,
            'settle_start': None,
            'settle_drift': None,
            'settled': False,
            'settled_ee': None,
            'settle_time': None,
            'retreat': False,
            'retreat_distance': 0.0,
            'retreat_time': None,
            'current_support': False,
            'current_held': False,
            'current_bilateral': False,
        }
        for obj in ('flask', 'beaker')
    }

    stage_diag = []
    command_sequence = []
    trajectory = []
    contact_pairs_seen = set()
    next_sample_time = 0.0
    references = dict(INITIAL)
    maximum_penetration = 0.0
    palm_contact_seen = False
    grasp_since = {obj: None for obj in TARGETS}

    for stage_name, targets, seconds in STAGES:
        if trial == 'insufficient-budget' and stage_name == 'left_transfer':
            break
        actual_targets = dict(targets)
        if trial == 'outside-left' and stage_name in ('left_transfer', 'left_lower', 'left_retreat'):
            actual_targets = arm('left', [-8.0, 0, 0, 80.0, 0, -2.0, 0], 0.05 if stage_name != 'left_retreat' else 0.65)
        start_references = dict(references)
        references.update(actual_targets)
        command_sequence.append({'stage': stage_name, 'targetsRad': actual_targets, 'durationSeconds': seconds})

        step_count = int(round(seconds / model.opt.timestep))
        assert abs(step_count * model.opt.timestep - seconds) < 1e-9
        for step in range(step_count):
            u = step * float(model.opt.timestep) / seconds
            h = u ** 3 * (10 + u * (-15 + 6 * u))
            for joint, value in references.items():
                actuator_id = acts[joint]
                ref = start_references[joint] + (value - start_references[joint]) * h
                dof = int(model.jnt_dofadr[joints[joint]])
                bias = 0.0 if 'finger' in joint else float(data.qfrc_bias[dof]) / float(model.actuator_gainprm[actuator_id, 0])
                data.ctrl[actuator_id] = np.clip(ref + bias, *model.actuator_ctrlrange[actuator_id])
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            for contact in data.contact:
                names = [name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1), name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)]
                if any(n.startswith(('flask_', 'beaker_')) for n in names):
                    maximum_penetration = max(maximum_penetration, -float(contact.dist))
                    if any(n.startswith('ee_base_link_') for n in names) and contact.dist < 0:
                        palm_contact_seen = True
            pair_list = contacts(model, data)
            contact_pairs_seen.update(tuple(sorted(pair)) for pair in pair_list)

            for obj in ('flask', 'beaker'):
                st = state[obj]
                spec = TARGETS[obj]
                position = body_position(data, bodies[obj])
                touched, support = object_contact_state(model, data, obj, pair_list)
                bilateral = spec['fingers'].issubset(touched)
                held = bool(touched)
                st['max_z'] = max(st['max_z'], float(position[2]))
                st['contact_seen'].update(touched)
                st['current_support'] = support
                st['current_held'] = held
                st['current_bilateral'] = bilateral
                if bilateral:
                    if grasp_since[obj] is None: grasp_since[obj] = float(data.time)
                    if float(data.time) - grasp_since[obj] >= 0.06 - 1e-9: st['grasp'] = True
                else:
                    grasp_since[obj] = None

                if st['grasp'] and bilateral and position[2] > spec['initial_z'] + CRITERIA['lift_clearance_m']:
                    st['lift'] = True
                    if st['carry_anchor'] is None and not st['carry']:
                        st['carry_anchor'] = position.copy()

                if st['carry_anchor'] is not None and not st['carry']:
                    if not held:
                        st['carry_anchor'] = None
                    elif bilateral and np.linalg.norm(position[:2] - st['carry_anchor'][:2]) >= CRITERIA['carry_horizontal_m']:
                        st['carry'] = True

                inside = bool(np.all(np.abs(position[:2] - spec['center']) <= spec['half']))
                near = abs(position[2] - spec['initial_z']) <= CRITERIA['support_height_tolerance_m']
                if st['carry'] and bilateral and support and inside and near and not st['support_while_held']:
                    st['support_while_held'] = True
                    st['support_while_held_time'] = float(data.time)

                # Match the browser evaluator: any post-release gripper re-contact invalidates
                # release/settle/retreat evidence until a new causal release is observed.
                if st['release'] and held:
                    st['release'] = False
                    st['release_time'] = None
                    st['settle_candidate'] = None
                    st['settle_start'] = None
                    st['settle_drift'] = None
                    st['settled'] = False
                    st['settled_ee'] = None
                    st['settle_time'] = None
                    st['retreat'] = False
                    st['retreat_distance'] = 0.0
                    st['retreat_time'] = None
                if st['support_while_held'] and st['previous_held'] and not held and support and inside and near and not st['release']:
                    st['release'] = True
                    st['release_time'] = float(data.time)

                qvel_address = int(model.jnt_dofadr[free[obj]])
                linear_speed = float(np.linalg.norm(data.qvel[qvel_address:qvel_address + 3]))
                angular_speed = float(np.linalg.norm(data.qvel[qvel_address + 3:qvel_address + 6]))
                settle_eligible = (
                    st['release'] and support and not held and inside and near
                    and linear_speed <= CRITERIA['max_settle_linear_speed_m_s']
                    and angular_speed <= CRITERIA['max_settle_angular_speed_rad_s']
                )
                ee = body_position(data, bodies[f"openarm_{spec['side']}_ee_base_link"])
                if not settle_eligible:
                    st['settle_candidate'] = None
                    st['settle_start'] = None
                    st['settle_drift'] = None
                    st['settled'] = False
                    st['settled_ee'] = None
                    st['retreat'] = False
                    st['retreat_distance'] = 0.0
                    st['retreat_time'] = None
                elif st['settle_candidate'] is None or st['settle_start'] is None:
                    st['settle_candidate'] = position.copy()
                    st['settle_start'] = float(data.time)
                    st['settle_drift'] = 0.0
                    st['settled'] = False
                    st['settled_ee'] = None
                    st['retreat'] = False
                    st['retreat_distance'] = 0.0
                    st['retreat_time'] = None
                else:
                    drift = float(np.linalg.norm(position - st['settle_candidate']))
                    if drift > CRITERIA['max_settle_drift_m']:
                        st['settle_candidate'] = position.copy()
                        st['settle_start'] = float(data.time)
                        st['settle_drift'] = 0.0
                        st['settled'] = False
                        st['settled_ee'] = None
                        st['retreat'] = False
                        st['retreat_distance'] = 0.0
                        st['retreat_time'] = None
                    else:
                        st['settle_drift'] = max(float(st['settle_drift'] or 0.0), drift)
                        if float(data.time) - st['settle_start'] + 1e-12 >= CRITERIA['settle_seconds']:
                            if not st['settled']:
                                st['settle_time'] = float(data.time)
                                st['settled_ee'] = ee.copy()
                            st['settled'] = True

                if st['settled'] and support and not held and st['settled_ee'] is not None:
                    retreat_distance = float(np.linalg.norm(ee - st['settled_ee']))
                    st['retreat_distance'] = max(st['retreat_distance'], retreat_distance)
                    if retreat_distance >= CRITERIA['retreat_distance_m']:
                        st['retreat'] = True
                        st['retreat_time'] = st['retreat_time'] or float(data.time)

                st['previous_held'] = held

            if float(data.time) + 1e-12 >= next_sample_time:
                trajectory.append({
                    'timeSeconds': float(data.time),
                    'flaskM': body_position(data, bodies['flask']).tolist(),
                    'beakerM': body_position(data, bodies['beaker']).tolist(),
                })
                next_sample_time += 0.05

        stage_diag.append({
            'stage': stage_name,
            'time': float(data.time),
            'flask': body_position(data, bodies['flask']).tolist(),
            'beaker': body_position(data, bodies['beaker']).tolist(),
            'contacts': contacts(model, data),
        })

    def metrics(obj):
        st = state[obj]
        spec = TARGETS[obj]
        position = body_position(data, bodies[obj])
        touched, support = object_contact_state(model, data, obj)
        inside = bool(np.all(np.abs(position[:2] - spec['center']) <= spec['half']))
        bilateral = spec['fingers'].issubset(touched)
        return {
            'bilateralContact': st['grasp'],
            'lifted': st['lift'],
            'carried': st['carry'],
            'supportWhileHeld': st['support_while_held'],
            'released': st['release'],
            'support': support,
            'settled': st['settled'],
            'settleDriftM': st['settle_drift'],
            'retreated': st['retreat'],
            'retreatDistanceM': st['retreat_distance'],
            'insideTarget': inside,
            'currentGripperContact': bool(touched),
            'currentBilateralContact': bilateral,
            'maxZM': st['max_z'],
            'finalPositionM': position.tolist(),
            'contactGeoms': sorted(st['contact_seen']),
            'supportWhileHeldTimeSeconds': st['support_while_held_time'],
            'releaseTimeSeconds': st['release_time'],
            'settleTimeSeconds': st['settle_time'],
            'retreatTimeSeconds': st['retreat_time'],
        }

    flask = metrics('flask')
    beaker = metrics('beaker')
    required = ('bilateralContact', 'lifted', 'carried', 'supportWhileHeld', 'released', 'support', 'settled', 'retreated', 'insideTarget')
    success = all(flask[key] for key in required) and all(beaker[key] for key in required) and maximum_penetration <= .002 and not palm_contact_seen
    blocked_actual = float(data.qpos[int(model.jnt_qposadr[joints['openarm_left_joint1']])])
    final_state = {
        'simulationTimeSeconds': float(data.time),
        'jointsRad': {
            joint: float(data.qpos[int(model.jnt_qposadr[joint_id])])
            for joint, joint_id in joints.items()
        },
        'bodiesM': {body: body_position(data, body_id).tolist() for body, body_id in bodies.items()},
    }
    descriptor = controller_descriptor()
    return {
        'classification': 'native/browser model implementation comparison; not hardware validation',
        'trial': trial,
        'trialProfile': {
            'negativeSetupObjectQposOffset': trial == 'miss-left',
            'modifiedActuatorLimits': trial in ('weak-grip', 'blocked-left'),
            'modifiedTargets': trial == 'outside-left',
            'truncatedExecutionBudget': trial == 'insufficient-budget',
        },
        'engine': {
            'version': mujoco.__version__,
            'timestepSeconds': float(model.opt.timestep),
            'integratorCode': int(model.opt.integrator),
            'solverCode': int(model.opt.solver),
            'iterations': int(model.opt.iterations),
            'lsIterations': int(model.opt.ls_iterations),
            'coneCode': int(model.opt.cone),
            'impratio': float(model.opt.impratio),
        },
        'model': {
            'id': 'robobuddy-openarm-v2-phase5a-v3',
            'packageId': 'openarm-v2-phase5a-a8c9796-v3',
            'asset': str(MODEL_PATH.relative_to(ROOT)).replace('\\', '/'),
            'sha256': sha256_bytes(MODEL_PATH.read_bytes()),
            'nq': int(model.nq),
            'nv': int(model.nv),
            'nu': int(model.nu),
            'neq': int(model.neq),
            'hasObjectWeld': False,
        },
        'controller': {
            'id': 'openarm-v2-bimanual-stack-v3',
            'sha256': canonical_json_hash(descriptor),
            'descriptor': descriptor,
        },
        'initialState': initial_state,
        'homeEeM': home_ee,
        'commandSequence': command_sequence,
        'contactPairsSeen': [list(pair) for pair in sorted(contact_pairs_seen)],
        'objectTrajectories': trajectory,
        'finalState': final_state,
        'metrics': {
            'success': bool(success),
            'maximumPenetrationM': maximum_penetration,
            'palmContactSeen': palm_contact_seen,
            'flask': flask,
            'beaker': beaker,
            'blockedLeftJoint1ActualRad': blocked_actual,
        },
        'taskVerdict': 'success' if success else 'failure',
        'stages': stage_diag,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trial', choices=('nominal', 'miss-left', 'weak-grip', 'blocked-left', 'outside-left', 'insufficient-budget'), default='nominal')
    parser.add_argument('--timestep', type=float)
    args = parser.parse_args()
    print(json.dumps(run_trial(args.trial, timestep=args.timestep), indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
