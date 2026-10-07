"""Release gates for the contact-aligned OpenArm v3 model.

Numerical implementation checks, not hardware calibration. The tolerances below
are task acceptance criteria; changing model identifiers must not weaken them.
Usage: python tests/validate_openarm_evidence.py test-results/native
"""
from __future__ import annotations

import hashlib
import json
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
NEGATIVES = ('miss-left', 'weak-grip', 'blocked-left', 'outside-left', 'insufficient-budget')


def finite_tree(value):
    if isinstance(value, float):
        assert math.isfinite(value), 'Non-finite value in native evidence'
    elif isinstance(value, dict):
        for item in value.values():
            finite_tree(item)
    elif isinstance(value, list):
        for item in value:
            finite_tree(item)


def main(directory: Path):
    xml_path = ROOT / 'models/openarm_v2/manipulation.xml'
    xml = ET.parse(xml_path).getroot()
    digest = hashlib.sha256(xml_path.read_bytes()).hexdigest()
    reference = json.loads((ROOT / 'models/openarm_v2/reference-controller.json').read_text())
    assert len(xml.findall('.//freejoint')) == 2
    assert not xml.findall('.//weld'), 'A free grasp must not use an object weld'
    equality = xml.find('equality')
    assert equality is not None and len(equality) == 2
    assert all(item.tag == 'joint' and 'finger' in item.attrib['joint1'] and 'finger' in item.attrib['joint2'] for item in equality)

    def load(suffix, trial, timestep):
        report = json.loads((directory / f'openarm-{suffix}.json').read_text())
        finite_tree(report)
        assert report['trial'] == trial
        assert report['classification'] == 'native/browser model implementation comparison; not hardware validation'
        assert report['engine']['version'] == '3.11.0'
        assert abs(report['engine']['timestepSeconds'] - timestep) < 1e-12
        assert report['engine']['iterations'] > 0
        assert report['model']['id'] == 'robobuddy-openarm-v2-phase5a-v3'
        assert report['model']['packageId'] == 'openarm-v2-phase5a-a8c9796-v3'
        assert report['model']['sha256'] == digest
        assert report['model']['neq'] == 2 and report['model']['hasObjectWeld'] is False
        assert report['model']['nu'] == 16 and report['model']['nq'] == 32 and report['model']['nv'] == 30
        assert report['controller']['id'] == 'openarm-v2-bimanual-stack-v3'
        descriptor = report['controller']['descriptor']
        controller_hash = hashlib.sha256(json.dumps(descriptor, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
        assert report['controller']['sha256'] == controller_hash
        expected_stages = [{key: stage[key] for key in ('name', 'targetsRad', 'durationSeconds')} for stage in reference['stages']]
        assert descriptor['stages'] == expected_stages
        assert len(expected_stages) == 17
        assert report['initialState']['simulationTimeSeconds'] == 0
        assert len(report['objectTrajectories']) > 10
        return report

    nominal = load('nominal', 'nominal', .001)
    tight = load('tight', 'nominal', .0005)
    negatives = {trial: load(trial, trial, .001) for trial in NEGATIVES}
    for report in (nominal, tight):
        assert report['taskVerdict'] == 'success' and report['metrics']['success'] is True
        assert len(report['commandSequence']) == 17
        assert report['metrics']['palmContactSeen'] is False
        assert 0 <= report['metrics']['maximumPenetrationM'] <= .002
        for side, y in (('left', .1535), ('right', -.1535)):
            assert math.dist(report['homeEeM'][side], [.401, y, 1.140]) < .003
        for obj, y, z, half_x, half_y in (('flask', .1535, 1.092, .017, .013), ('beaker', -.1535, 1.105, .021, .021)):
            metric = report['metrics'][obj]
            for key in ('bilateralContact', 'lifted', 'carried', 'supportWhileHeld', 'released', 'support', 'settled', 'retreated', 'insideTarget'):
                assert metric[key] is True, (obj, key, metric)
            assert metric['currentGripperContact'] is False and metric['currentBilateralContact'] is False
            assert metric['retreatDistanceM'] >= .05
            assert metric['settleDriftM'] is not None and 0 <= metric['settleDriftM'] <= .001
            assert metric['maxZM'] > z + .02
            final = metric['finalPositionM']
            assert abs(final[0] - .67) <= half_x and abs(final[1] - y) <= half_y
            assert abs(final[2] - z) <= .015
            assert metric['supportWhileHeldTimeSeconds'] < metric['releaseTimeSeconds'] < metric['settleTimeSeconds'] < metric['retreatTimeSeconds']
        assert not any(report['trialProfile'].values()), report['trialProfile']
    for trial, report in negatives.items():
        assert report['taskVerdict'] == 'failure' and report['metrics']['success'] is False, (trial, report['metrics'])
        assert any(report['trialProfile'].values()), (trial, 'Negative setup must be declared')
    deltas = {obj: math.dist(nominal['metrics'][obj]['finalPositionM'], tight['metrics'][obj]['finalPositionM']) for obj in ('flask', 'beaker')}
    assert all(delta <= .002 for delta in deltas.values()), deltas
    summary = {
        'modelSha256': digest,
        'classification': nominal['classification'],
        'nominalSuccess': True, 'halfTimestepSuccess': True,
        'negativeSuccess': {trial: report['metrics']['success'] for trial, report in negatives.items()},
        'nominalMaximumPenetrationM': nominal['metrics']['maximumPenetrationM'],
        'nominalPalmContactSeen': nominal['metrics']['palmContactSeen'],
        'halfTimestepFinalPositionDeltaM': deltas,
    }
    (directory / 'openarm-release-summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print('OpenArm v3 native nominal/negative/sensitivity gates: OK')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main(Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'test-results/native')
