#!/usr/bin/env python3
"""Validate source-pinned MicroDuck BAM physical evidence.

Thresholds intentionally test qualitative physical discrimination rather than
preserving numbers from the superseded XML position-servo approximation.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ALL = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/microduck-all.json")
TIGHT_WALK = Path(sys.argv[2] if len(sys.argv) > 2 else "/tmp/microduck-walk-tight.json")
TIGHT_STAND = Path(sys.argv[3] if len(sys.argv) > 3 else "/tmp/microduck-stand-tight.json")
reports = json.loads(ALL.read_text())
tight_walk = json.loads(TIGHT_WALK.read_text())["walk"]
tight_stand = json.loads(TIGHT_STAND.read_text())["stand"]


def m(name):
    return reports[name]["measured"]


def c(name):
    return reports[name]["contacts"]


# Engine / provenance authority.
for name, report in reports.items():
    assert report["engine"]["timestepSeconds"] == 0.005, (name, report["engine"])
    assert report["engine"]["controlDecimation"] == 4, (name, report["engine"])
    assert abs(report["engine"]["controlIntervalSeconds"] - 0.02) < 1e-12, (name, report["engine"])
    assert report["modelSha256"], name
    assert report["policySha256"], name

# Collision routing is part of the evidence, not an incidental filename choice.
for name in ('walk', 'walk-fast', 'walk-no-actuation'):
    assert reports[name]['collisionPlant'] == 'walk', (name, reports[name]['collisionPlant'])
assert reports['walk-low-traction']['collisionPlant'] == 'lowtraction'
for name in ('stand', 'sit', 'ground-pick', 'roulade', 'recover-face-down', 'recover-face-up', 'recover-on-side', 'recover-no-actuation', 'recover-short-budget'):
    assert reports[name]['collisionPlant'] == 'groundcontact', (name, reports[name]['collisionPlant'])
for name in ('kick-right', 'kick-left', 'kick-miss'):
    assert reports[name]['collisionPlant'] == 'kick', (name, reports[name]['collisionPlant'])

# Deployment-reference gain is fed into BAM, not converted into MuJoCo stiffness.
assert reports["walk"]["controller"]["firmwareGain"] == 200, reports["walk"]["controller"]
assert reports["stand"]["controller"]["firmwareGain"] == 160, reports["stand"]["controller"]
assert m("walk")["meanActuatorForceNm"] > 0.02, m("walk")

# Stand: physically supported and upright.
stand = m("stand")
assert stand["finalTiltDeg"] < 10.0, stand
assert 0.09 < stand["finalTrunkHeightM"] < 0.14, stand
assert c("stand")["leftStanceFraction"] > 0.80, c("stand")
assert c("stand")["rightStanceFraction"] > 0.80, c("stand")

# Walk: non-trivial displacement plus alternating contact, not a root translation.
walk = m("walk")
assert walk["commandedAxisDistanceM"] > 0.20, walk
assert not walk["upstreamFallenGate"], walk
assert c("walk")["leftContactTransitions"] >= 8, c("walk")
assert c("walk")["rightContactTransitions"] >= 8, c("walk")
assert c("walk")["leftAirFraction"] > 0.10, c("walk")
assert c("walk")["rightAirFraction"] > 0.10, c("walk")
assert m("walk-fast")["commandedAxisDistanceM"] > walk["commandedAxisDistanceM"], m("walk-fast")

# Motor disabled: no electromagnetic torque/actuator force, while passive BAM friction remains.
off = m("walk-no-actuation")
assert off["upstreamFallenGate"], off
assert off["finalTrunkHeightM"] < 0.08, off
assert off["commandedAxisDistanceM"] < walk["commandedAxisDistanceM"] * 0.5, off
for name in ("walk-no-actuation", "recover-no-actuation"):
    assert reports[name]["controller"]["firmwareGain"] == 0, reports[name]["controller"]
    assert m(name)["meanActuatorForceNm"] == 0.0, (name, m(name))
    assert m(name)["peakActuatorForceNm"] == 0.0, (name, m(name))

# Deliberately degraded surface must materially alter forward locomotion.
slip = m("walk-low-traction")
assert abs(slip["commandedAxisDistanceM"]) < abs(walk["commandedAxisDistanceM"]) * 0.8, (slip, walk)

# Kicks require named foot-ball contact; a miss cannot be promoted to success.
for side in ("kick-right", "kick-left"):
    hit = reports[side]
    assert hit["ball"]["footContactCount"] > 0, (side, hit["ball"])
    assert hit["ball"]["planarDistanceM"] > 0.05, (side, hit["ball"])
    for contact in c(side)["footBallContacts"]:
        assert "microduck_ball_geom" in contact["geoms"], contact
miss = reports["kick-miss"]
assert miss["ball"]["footContactCount"] == 0, miss["ball"]
assert miss["ball"]["planarDistanceM"] < 0.01, miss["ball"]

# Recovery policy must demonstrate a genuine positive case and genuine negatives.
face_up = m("recover-face-up")
assert face_up["uprightRecovered"], face_up
assert face_up["finalTrunkHeightM"] > 0.095, face_up
for name in ("recover-no-actuation", "recover-short-budget"):
    assert not m(name)["uprightRecovered"], (name, m(name))

# Sit must actually lower the body.
sit = m("sit")
assert sit["finalTrunkHeightM"] < 0.085, sit

# Five-times smaller timestep may perturb a contact-rich gait, but must not change the conclusion.
tight_walk_m = tight_walk["measured"]
ratio = tight_walk_m["commandedAxisDistanceM"] / walk["commandedAxisDistanceM"]
assert 0.70 <= ratio <= 1.30, (ratio, tight_walk_m, walk)
assert not tight_walk_m["upstreamFallenGate"], tight_walk_m
assert abs(tight_stand["measured"]["finalTrunkHeightM"] - stand["finalTrunkHeightM"]) < 0.015, (tight_stand["measured"], stand)

print("MicroDuck BAM physical/negative/sensitivity gates: OK")
print(f"  walk {walk['commandedAxisDistanceM']:+.4f} m; tight ratio {ratio:.3f}")
print(f"  stand z={stand['finalTrunkHeightM']:.4f} m tilt={stand['finalTiltDeg']:.2f} deg")
print(f"  motor-off peak actuator force={off['peakActuatorForceNm']:.6f} N m")
print(f"  low traction {slip['commandedAxisDistanceM']:+.4f} m")
print(f"  kick R/L={reports['kick-right']['ball']['planarDistanceM']:.3f}/{reports['kick-left']['ball']['planarDistanceM']:.3f} m")
