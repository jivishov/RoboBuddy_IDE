#!/usr/bin/env python3
"""Gate the native Unitree G1 Phase 5D evidence.

The reports produced by native/unitree_g1_reference.py are only useful if something asserts what
they must contain. This validator does that, so CI cannot go green on a run whose standing trial
silently stopped standing, whose negative controls silently started passing, whose actuation
silently exceeded a source effort limit, or whose evidence quietly acquired a hardware claim.

Usage:
    python tests/validate_unitree_g1_evidence.py <report-directory>

Every report listed in REQUIRED must be present in that directory.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

SCHEMA = "robobuddy.unitree-g1.native-reference.v1"
MODEL_SHA256 = {
    "mounted": "78b99a41889c15f677b723349f4f6f478725bc39a4efa5d7dcf37730d50d0a01",
    "mounted_blocked": "5f0af3108fc496fae98a4d1eb2c59d752423668363939a0eb020519d9cc31065",
    "freebase": "3aad0869bf2a83d1f9acd84f5c627745a9a56dc1ab426be8a1f82463a6a76bf3",
    "freebase_drop": "e222c0b8bf12f51352373f31965752893f5cbb987dc28a172300fcaa25165ab5",
}
TOTAL_MASS_KG = 35.112142
STAND_CONTROLLER = "robobuddy_g1_stand_v1"
SOURCE_FIXSTAND = "unitree_g1_fixstand_source_v1"

REQUIRED = (
    "model-audit",
    "mounted-joint-response",
    "mounted-joint-response-tight",
    "blocked-joint",
    "blocked-joint-tight",
    "known-poses",
    "command-bounds",
    "free-fall",
    "free-fall-tight",
    "self-contact",
    "external-object",
    "stand-nominal",
    "stand-nominal-tight",
    "stand-source-fixstand",
    "stand-motors-disabled",
    "stand-perturbation",
    "stand-perturbation-over",
    "stand-low-friction",
)

failures: list[str] = []
notes: list[str] = []


def check(condition: object, message: str) -> None:
    if not condition:
        failures.append(message)


def load(directory: Path, trial: str) -> dict:
    path = directory / f"unitree-g1-{trial}.json"
    if not path.exists():
        failures.append(f"missing report: {path}")
        return {}
    report = json.loads(path.read_text())
    check(report.get("schema") == SCHEMA, f"{trial}: schema must be {SCHEMA}, got {report.get('schema')!r}")
    classification = str(report.get("classification", ""))
    check("not hardware validation" in classification, f"{trial}: evidence must be classified as not hardware validation")
    check("hardware-compared" not in classification.lower(), f"{trial}: evidence must not claim a hardware comparison")
    engine = report.get("engine") or {}
    check(engine.get("version") == "3.11.0", f"{trial}: engine version must be 3.11.0, got {engine.get('version')!r}")
    model = report.get("model") or {}
    variant = model.get("variant")
    if variant in MODEL_SHA256:
        check(model.get("sha256") == MODEL_SHA256[variant],
              f"{trial}: {variant} model sha256 {model.get('sha256')} does not match the committed MJCF")
    return report


def gate_measured(trial: str, report: dict) -> dict:
    evaluation = report.get("evaluation") or {}
    measured = evaluation.get("measured") or {}
    check(measured, f"{trial}: report carries no evaluated measurements")
    return measured


def expect_stand_pass(trial: str, report: dict, *, max_effort_fraction: float) -> None:
    evaluation = report.get("evaluation") or {}
    check(evaluation.get("passed") is True, f"{trial}: the standing gate must pass")
    for name, value in (evaluation.get("checks") or {}).items():
        check(value is True, f"{trial}: standing check {name} must hold")
    measured = gate_measured(trial, report)
    check(report.get("motorsEnabled") is True, f"{trial}: motors must be enabled")
    support = str(report.get("supportUsed") or "none")
    check(support.startswith("none"), f"{trial}: no external support may be used, got {support!r}")
    check(measured.get("maxNonFootGroundContacts") == 0, f"{trial}: nothing but a foot may touch the ground")
    check(measured.get("maxExternalOrFixtureContacts") == 0, f"{trial}: no external object or fixture may support the stand")
    effort = float(measured.get("worstActuatorEffortFraction", 1.0))
    check(effort <= max_effort_fraction, f"{trial}: worst effort fraction {effort} exceeds {max_effort_fraction}")
    controller = (report.get("controller") or {}).get("id")
    check(controller == STAND_CONTROLLER, f"{trial}: must run {STAND_CONTROLLER}, got {controller!r}")
    notes.append(
        f"{trial}: pass z=[{measured.get('pelvisHeightMinM')},{measured.get('pelvisHeightMaxM')}] "
        f"tilt={measured.get('maxTiltDeg')} deg drift={measured.get('maxHorizontalDriftM')} m effort={effort}"
    )


def expect_stand_fail(trial: str, report: dict, *, reason: str) -> None:
    evaluation = report.get("evaluation") or {}
    check(evaluation.get("passed") is False, f"{trial}: this negative control must fail ({reason})")
    measured = gate_measured(trial, report)
    check(float(measured.get("maxTiltRad", 0)) > 0.5, f"{trial}: a failed stand must actually fall over, not merely miss the gate")
    check(int(measured.get("maxNonFootGroundContacts", 0)) > 0, f"{trial}: a failed stand must put something other than a foot on the ground")
    notes.append(
        f"{trial}: fail tilt={measured.get('maxTiltDeg')} deg nonFoot={measured.get('maxNonFootGroundContacts')} "
        f"effort={measured.get('worstActuatorEffortFraction')}"
    )


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__)
        return 2
    directory = Path(argv[1])
    reports = {trial: load(directory, trial) for trial in REQUIRED}

    # --- model identity -----------------------------------------------------------------------
    audit = reports["model-audit"]
    tables = audit.get("sourceTables") or {}
    check(len(tables.get("jointOrder") or []) == 29, "model-audit: the source joint order must have 29 entries")
    for variant, info in (audit.get("variants") or {}).items():
        check(info.get("nq") is not None, f"model-audit: {variant} reports no nq")
        check(info.get("neq") == 0, f"model-audit: {variant} must contain no equality constraint, got neq={info.get('neq')}")
        check(info.get("ntendon") == 0, f"model-audit: {variant} must contain no tendon, got ntendon={info.get('ntendon')}")
        check(info.get("nu") == 29, f"model-audit: {variant} must expose 29 actuators, got {info.get('nu')}")
        check(info.get("unnamedCollisionGeoms") == 0, f"model-audit: {variant} has unnamed collision geometry, so contacts cannot be identified")
        if variant != "freebase":
            check(abs(float(info.get("totalMassKg", 0)) - TOTAL_MASS_KG) < 1e-4,
                  f"model-audit: {variant} total mass {info.get('totalMassKg')} kg must be the source mass {TOTAL_MASS_KG} kg")
        if variant in MODEL_SHA256:
            check(info.get("sha256") == MODEL_SHA256[variant], f"model-audit: {variant} sha256 does not match the committed MJCF")

    # --- mounted dynamics: a commanded joint moves and settles short of its target -------------
    for trial in ("mounted-joint-response", "mounted-joint-response-tight"):
        joints = reports[trial].get("joints") or []
        check(joints, f"{trial}: no commanded joints reported")
        for row in joints:
            check(row.get("signAgreesWithTarget") is True, f"{trial}: {row.get('joint')} moved the wrong way")
            check(row.get("measuredDiffersFromCommand") is True,
                  f"{trial}: {row.get('joint')} measured value equals its command, which would mean the target is being echoed")
            check(abs(float(row.get("actuatorEffortNm", 0))) <= float(row.get("effortLimitNm", 0)) + 1e-9,
                  f"{trial}: {row.get('joint')} exceeded its source effort limit")

    # --- blocked joint: the same command, obstructed ------------------------------------------
    for trial in ("blocked-joint", "blocked-joint-tight"):
        comparison = reports[trial].get("comparison") or {}
        free = comparison.get("unobstructed") or {}
        blocked = comparison.get("blocked") or {}
        check(free.get("requestedTargetRad") == blocked.get("requestedTargetRad"),
              f"{trial}: the obstructed and unobstructed cases must issue the identical command")
        check(float(blocked.get("measuredPositionRad", 0)) < float(free.get("measuredPositionRad", 0)) - 0.5,
              f"{trial}: the obstructed joint must stop far short of the unobstructed one")
        contacts = blocked.get("fixtureContacts") or []
        check(contacts, f"{trial}: the obstruction must be reported as a named contact")
        check(any("stop_wall" in str(name) for pair in contacts for name in (pair.get("geoms") if isinstance(pair, dict) else pair)),
              f"{trial}: the obstruction contact must name the declared stop wall")
        check(abs(float(blocked.get("actuatorEffortNm", 0))) > 0.9 * float(blocked.get("effortLimitNm", 1)),
              f"{trial}: the blocked actuator must be loaded against the obstruction")

    # --- root-fixed known poses ---------------------------------------------------------------
    poses = reports["known-poses"].get("poses") or {}
    check(len(poses) >= 2, "known-poses: at least a neutral and one commanded pose must be reported")
    neutral = (poses.get("neutral") or {}).get("bodies") or {}
    check("pelvis" in neutral, "known-poses: the neutral pose must report the pelvis frame")
    for name, pose in poses.items():
        pelvis = ((pose.get("bodies") or {}).get("pelvis") or {}).get("positionM")
        check(pelvis == (neutral.get("pelvis") or {}).get("positionM"),
              f"known-poses: the {name} pose moved the welded pelvis, so the mount is not root-fixed")

    # --- command bounds -----------------------------------------------------------------------
    bounds = reports["command-bounds"]
    for case in bounds.get("cases") or []:
        low, high = case.get("sourceJointRangeRad") or [0, 0]
        check(low - 1e-9 <= float(case.get("acceptedPositionRad", 0)) <= high + 1e-9,
              f"command-bounds: {case.get('joint')} accepted a position outside its source range")
        check(abs(float(case.get("acceptedVelocityRadS", 0))) <= float(case.get("sourceVelocityLimitRadS", 0)) + 1e-9,
              f"command-bounds: {case.get('joint')} accepted a velocity above its source limit")
        check(abs(float(case.get("acceptedFeedforwardNm", 0))) <= float(case.get("sourceEffortLimitNm", 0)) + 1e-9,
              f"command-bounds: {case.get('joint')} accepted a feedforward torque above its source effort limit")
        check(float(case.get("acceptedKp", 0)) <= 400.0 + 1e-9, f"command-bounds: {case.get('joint')} accepted kp above the bound")
        check(float(case.get("acceptedKd", 0)) <= 60.0 + 1e-9, f"command-bounds: {case.get('joint')} accepted kd above the bound")
    measured = bounds.get("measuredAfterOutOfRangeRequest") or {}
    check(float(measured.get("requestedTargetRad", 0)) != float(measured.get("acceptedTargetRad", 0)),
          "command-bounds: the out-of-range request must be visibly different from the accepted command")
    check(float(bounds.get("worstActuatorEffortFraction", 1)) <= 1.0 + 1e-9,
          "command-bounds: no actuator may exceed its source effort limit")

    # --- a genuine fall -----------------------------------------------------------------------
    for trial in ("free-fall", "free-fall-tight"):
        fall = reports[trial]
        check(float(fall.get("rootHeightChangeM", 0)) < -0.3, f"{trial}: the root must actually fall")
        check(float(fall.get("uprightChange", 0)) < -1.0, f"{trial}: the robot must actually go over, not settle upright")
        check(int(fall.get("nonFootGroundContacts", 0)) > 0, f"{trial}: a fall must put something other than a foot on the ground")
    base = float(reports["free-fall"].get("rootHeightChangeM", 0))
    tight = float(reports["free-fall-tight"].get("rootHeightChangeM", 0))
    check(abs(base - tight) < 0.25, f"free-fall: halving the timestep changed the drop by {abs(base - tight)} m")
    notes.append(f"free-fall: dz={base} m at 2 ms, {tight} m at 1 ms")

    # --- self contact -------------------------------------------------------------------------
    self_contact = reports["self-contact"]
    pairs = self_contact.get("selfContactPairs") or []
    check(pairs, "self-contact: no self-contact pair was reported")
    check(all(len(set(pair)) == 2 for pair in pairs), "self-contact: a reported pair must name two different bodies")
    for entry in self_contact.get("selfContacts") or []:
        check(len(entry.get("geoms") or []) == 2, "self-contact: a contact must be identified by a named geometry pair")
        check(float(entry.get("normalForceN", 0)) > 0, "self-contact: a reported self-contact must carry a real normal force")
    notes.append(f"self-contact: {pairs}")

    # --- external object ----------------------------------------------------------------------
    external = reports["external-object"]
    check(float(external.get("objectDisplacementM", 0)) > 0.02, "external-object: the free object must respond to contact")
    check(external.get("contactGeomPairs"), "external-object: the contact must be identified by named geometry pair")
    check(any("contact_probe_block" in str(name) for pair in external.get("contactGeomPairs") or [] for name in pair),
          "external-object: the contact must name the declared external object")
    notes.append(f"external-object: displacement={external.get('objectDisplacementM')} m pairs={external.get('contactGeomPairs')}")

    # --- standing: the positive result and every negative control -----------------------------
    expect_stand_pass("stand-nominal", reports["stand-nominal"], max_effort_fraction=0.9)
    expect_stand_pass("stand-nominal-tight", reports["stand-nominal-tight"], max_effort_fraction=0.9)
    expect_stand_pass("stand-low-friction", reports["stand-low-friction"], max_effort_fraction=0.9)
    expect_stand_pass("stand-perturbation", reports["stand-perturbation"], max_effort_fraction=0.9)
    expect_stand_fail("stand-source-fixstand", reports["stand-source-fixstand"], reason="the source gains cannot hold free-base posture on this model")
    expect_stand_fail("stand-motors-disabled", reports["stand-motors-disabled"], reason="no actuation means no standing")
    expect_stand_fail("stand-perturbation-over", reports["stand-perturbation-over"], reason="a posture hold is not perturbation recovery")

    fixstand = (reports["stand-source-fixstand"].get("controller") or {}).get("id")
    check(fixstand == SOURCE_FIXSTAND, f"stand-source-fixstand: must run {SOURCE_FIXSTAND}, got {fixstand!r}")
    disabled = gate_measured("stand-motors-disabled", reports["stand-motors-disabled"])
    check(reports["stand-motors-disabled"].get("motorsEnabled") is False, "stand-motors-disabled: motors must really be off")
    check(float(disabled.get("worstActuatorEffortFraction", 1)) == 0.0,
          "stand-motors-disabled: a motors-off trial must report exactly zero actuator effort")

    # The perturbation pair must bracket the boundary: the survivable impulse is smaller than the
    # one that topples it, or the pair proves nothing about where the limit lies.
    survived = float(reports["stand-perturbation"].get("perturbationLinearMS") or 0)
    toppled = float(reports["stand-perturbation-over"].get("perturbationLinearMS") or 0)
    check(survived < toppled, f"stand-perturbation: the survived impulse {survived} must be smaller than the toppling impulse {toppled}")
    notes.append(f"perturbation boundary: held at {survived} m/s, fell at {toppled} m/s")

    # --- timestep sensitivity -----------------------------------------------------------------
    loose = gate_measured("stand-nominal", reports["stand-nominal"])
    fine = gate_measured("stand-nominal-tight", reports["stand-nominal-tight"])
    for key, tolerance in (("pelvisHeightMinM", 0.01), ("maxTiltRad", 0.05), ("maxHorizontalDriftM", 0.03)):
        delta = abs(float(loose.get(key, 0)) - float(fine.get(key, 0)))
        check(delta < tolerance, f"stand-nominal: {key} moved by {delta} when the timestep was halved")
        notes.append(f"sensitivity {key}: delta={delta:.3e} over 2 ms vs 1 ms")

    for line in notes:
        print(f"  {line}")
    if failures:
        print(f"\nUnitree G1 native evidence: {len(failures)} FAILURE(S)")
        for failure in failures:
            print(f"  - {failure}")
        return 1
    print(f"\nUnitree G1 native evidence: OK ({len(REQUIRED)} reports)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
