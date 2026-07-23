from __future__ import annotations

import json

from blue_drake.cli import main
from blue_drake.validation import run_validation_suite


def test_analytical_validation_suite_passes() -> None:
    report = run_validation_suite()
    assert report.benchmark_schema_version == 2
    assert len(report.checks) == 13
    assert report.passed
    assert {check.check_id for check in report.checks} == {
        "submerged-archimedes",
        "free-surface-immersion",
        "air-drag-envelope",
        "surge-drag-polynomial",
        "surface-heave-stiffness",
        "diagonal-added-mass",
        "glider-speed-squared",
        "hydrostatic-pressure",
        "acoustic-ideal-latency",
        "rov-thruster-geometry",
        "uuv-stern-propeller-geometry",
        "station-keeping-position-feedback",
        "parallel-gripper-symmetric-feedback",
    }
    subjects = {check.subject_id for check in report.checks}
    assert {"rov", "uuv", "glider", "usv"} <= subjects
    assert all(check.evidence_type == "analytical" for check in report.checks)
    assert all(check.parameter_provenance for check in report.checks)


def test_benchmark_cli_emits_machine_readable_evidence(capsys) -> None:
    assert main(["benchmark", "--json"]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["benchmark_schema_version"] == 2
    assert result["passed"] is True
    assert result["check_count"] == 13
    assert all(check["passed"] for check in result["checks"])
    assert all("parameter_provenance" in check for check in result["checks"])
