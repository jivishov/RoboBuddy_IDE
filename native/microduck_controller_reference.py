#!/usr/bin/env python3
"""Validate the committed MicroDuck controller-conformance fixture independently of plant dynamics.

The fixture's physical-looking gyro/gravity/joint values are treated only as frozen bounded
controller input vectors. They are not physical evidence. Physical fidelity is validated
separately by microduck_bam_physical_reference.py against the BAM/MuJoCo plant.

This validator independently derives the deployed 61-value observation, evaluates the exact
pinned walking ONNX policy, applies mouth exclusion/action scaling/low-pass target processing,
and checks model/policy byte identities.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import onnxruntime as ort

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / 'assets/microduck/fixtures/controller-conformance.json'
POLICY = ROOT / 'assets/microduck/policies/alpha_walking.onnx'
MODEL = ROOT / 'models/microduck/walk.xml'

MICRODUCK_PIN = '590b986bd8c0d50ae02cb3ea2f59c463b6828168'
MICRODUCK_RL_PIN = '519142b1f5bf59fdfd44d06c205119e7fff8e3cb'
WIRE_ORDER = (
    'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee', 'left_ankle',
    'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll', 'mouth',
    'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle',
)
MOUTH_INDEX = 9
POLICY_ORDER = tuple(name for index, name in enumerate(WIRE_ORDER) if index != MOUTH_INDEX)
HOME = np.array([
    0.0, -0.0873, -0.4579, -0.0049, 0.4530,
    0.3491, 0.3491, 0.0, 0.0,
    0.0, 0.0873, 0.4579, 0.0049, -0.4530,
], dtype=np.float64)
HEAD_SLOTS = {5, 6, 7, 8}
OBS_WIDTH = 61
ACTION_WIDTH = 14
ACTION_SCALE = 0.9
HEAD_ALPHA = 0.5
LEGS_ALPHA = 0.7


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def close_array(actual, expected, tolerance: float, label: str) -> None:
    a = np.asarray(actual, dtype=np.float64)
    e = np.asarray(expected, dtype=np.float64)
    if a.shape != e.shape:
        raise AssertionError(f'{label}: shape {a.shape} != {e.shape}')
    delta = np.max(np.abs(a - e)) if a.size else 0.0
    if not np.isfinite(delta) or delta > tolerance:
        raise AssertionError(f'{label}: max delta {delta} > {tolerance}')


def build_observation(sample_input: dict) -> np.ndarray:
    obs = np.zeros(OBS_WIDTH, dtype=np.float32)
    obs[0:3] = sample_input['gyroRadS']
    obs[3:6] = sample_input['projectedGravity']
    obs[6:20] = np.asarray(sample_input['jointPositionRad'], dtype=np.float64) - HOME
    obs[20:34] = sample_input['jointVelocityRadS']
    obs[34:48] = sample_input['previousRawAction']
    command = sample_input['command']
    obs[48:51] = command['twist']
    obs[51:55] = command['head']
    body = command.get('body') or {}
    obs[55] = 0.0
    obs[56] = 0.0
    obs[57] = float(body.get('z', 0.0))
    obs[58] = float(body.get('roll', 0.0))
    obs[59] = float(body.get('pitch', 0.0))
    obs[60] = 0.0
    return obs


def targets_for(action, scale: float, previous_targets) -> np.ndarray:
    action = np.asarray(action, dtype=np.float64)
    if action.shape != (ACTION_WIDTH,):
        raise AssertionError(f'action shape {action.shape} != ({ACTION_WIDTH},)')
    targets = HOME + float(scale) * action
    if previous_targets is not None:
        previous = np.asarray(previous_targets, dtype=np.float64)
        if previous.shape != (ACTION_WIDTH,):
            raise AssertionError('previous target shape mismatch')
        for slot in range(ACTION_WIDTH):
            alpha = HEAD_ALPHA if slot in HEAD_SLOTS else LEGS_ALPHA
            targets[slot] = alpha * targets[slot] + (1.0 - alpha) * previous[slot]
    return targets


def validate(path: Path) -> dict:
    fixture = json.loads(path.read_text())
    assert fixture['schema'] == 'robobuddy.microduck-controller-conformance.v1'
    assert fixture['pins']['microduck'] == MICRODUCK_PIN
    assert fixture['pins']['microduck_rl'] == MICRODUCK_RL_PIN
    assert fixture['model']['asset'] == 'models/microduck/walk.xml'
    assert fixture['model']['sha256'] == sha256(MODEL)
    assert fixture['policy']['id'] == 'walking'
    assert fixture['policy']['file'] == POLICY.name
    assert fixture['policy']['sha256'] == sha256(POLICY)

    contract = fixture['contract']
    assert contract['observationWidth'] == OBS_WIDTH
    assert contract['actionWidth'] == ACTION_WIDTH
    assert tuple(contract['policyJointOrder']) == POLICY_ORDER
    close_array(contract['homePositionRad'], HOME, 1e-12, 'home pose')
    assert abs(float(contract['actionScale']) - ACTION_SCALE) <= 1e-12
    assert abs(float(contract['headLowpassAlpha']) - HEAD_ALPHA) <= 1e-12
    assert abs(float(contract['legsLowpassAlpha']) - LEGS_ALPHA) <= 1e-12
    assert set(contract['headJointSlots']) == HEAD_SLOTS
    assert abs(float(contract['physicsTimestepSeconds']) - 0.005) <= 1e-12
    assert int(contract['controlDecimation']) == 4

    session = ort.InferenceSession(str(POLICY), providers=['CPUExecutionProvider'])
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    assert list(session.get_inputs()[0].shape[-1:]) == [OBS_WIDTH]
    assert list(session.get_outputs()[0].shape[-1:]) == [ACTION_WIDTH]

    for sample in fixture['samples']:
        tick = sample['tick']
        obs = build_observation(sample['input'])
        close_array(obs, sample['observation'], 1e-6, f'tick {tick} observation')
        raw = session.run([output_name], {input_name: obs.reshape(1, OBS_WIDTH)})[0].reshape(ACTION_WIDTH)
        close_array(raw, sample['rawAction'], 1e-6, f'tick {tick} ONNX raw action')
        targets = targets_for(raw, sample['actionScale'], sample['input']['previousTargetsRad'])
        close_array(targets, sample['targetsRad'], 1e-6, f'tick {tick} filtered targets')
        wire = sample['wireTargets']
        assert tuple(wire['order']) == WIRE_ORDER
        assert int(wire['mouthWireIndex']) == MOUTH_INDEX
        assert wire['mouthIsPolicyControlled'] is False

    return {
        'samples': len(fixture['samples']),
        'modelSha256': fixture['model']['sha256'],
        'policySha256': fixture['policy']['sha256'],
        'inputSemantics': 'frozen prescribed controller test vectors; not physical evidence',
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture', default=str(FIXTURE))
    parser.add_argument('--json', default=None)
    args = parser.parse_args()
    result = validate(Path(args.fixture))
    if args.json:
        Path(args.json).write_text(json.dumps(result, indent=2) + '\n')
    print(f"controller fixture independent validation: OK ({result['samples']} samples)")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
