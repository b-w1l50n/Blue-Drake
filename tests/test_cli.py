from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from blue_drake import __version__
from blue_drake.cli import _parse_args, main

REPO_ROOT = Path(__file__).resolve().parents[1]
MIXED_SCENARIO = REPO_ROOT / "scenarios" / "mixed_marine.toml"


def test_legacy_scenario_argument_maps_to_run_command() -> None:
    args = _parse_args([str(MIXED_SCENARIO), "--no-visualizer"])
    assert args.command == "run"
    assert args.scenario == str(MIXED_SCENARIO)
    assert args.meshcat_host == "localhost"
    assert args.meshcat_port is None


def test_meshcat_network_binding_requires_explicit_run_options() -> None:
    args = _parse_args(
        [
            "run",
            str(MIXED_SCENARIO),
            "--meshcat-host",
            "*",
            "--meshcat-port",
            "7000",
        ]
    )
    assert args.meshcat_host == "*"
    assert args.meshcat_port == 7000


def test_meshcat_rejects_privileged_port(capsys) -> None:
    with pytest.raises(SystemExit) as exc_info:
        _parse_args(["run", str(MIXED_SCENARIO), "--meshcat-port", "80"])
    assert exc_info.value.code == 2
    assert "--meshcat-port" in capsys.readouterr().err


def test_package_version_has_release_candidate_value() -> None:
    assert __version__ == "0.1.0"


def test_version_option_reports_single_source_of_truth(capsys) -> None:
    with pytest.raises(SystemExit) as exc_info:
        _parse_args(["--version"])
    assert exc_info.value.code == 0
    assert capsys.readouterr().out.strip().endswith(f" {__version__}")


def test_validate_does_not_need_simulation(capsys) -> None:
    assert main(["validate", str(MIXED_SCENARIO)]) == 0
    output = capsys.readouterr()
    assert "Valid: mixed-marine-foundation (schema 1)" in output.out
    assert output.err == ""


def test_validate_subprocess_does_not_import_pydrake() -> None:
    code = (
        "import sys; "
        "from blue_drake.cli import main; "
        f"assert main(['validate', {str(MIXED_SCENARIO)!r}]) == 0; "
        "assert not any(n == 'pydrake' or n.startswith('pydrake.') "
        "for n in sys.modules)"
    )
    completed = subprocess.run(
        [sys.executable, "-c", code],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr


def test_inspect_json_is_machine_readable(capsys) -> None:
    assert main(["inspect", str(MIXED_SCENARIO), "--json"]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["name"] == "mixed-marine-foundation"
    assert len(result["vehicles"]) == 4


def test_catalog_lists_avior_and_sealink(capsys) -> None:
    assert main(["catalog", "--json"]) == 0
    result = json.loads(capsys.readouterr().out)
    assert any(
        sensor["profile_id"] == "xsens-avior-ahrs"
        for sensor in result["sensors"]
    )
    assert result["modems"][0]["profile_id"] == "divenet-sealink-3km-oem"


def test_invalid_scenario_returns_nonzero_without_traceback(
    tmp_path: Path, capsys
) -> None:
    path = tmp_path / "bad.toml"
    path.write_text("schema_version = 99", encoding="utf-8")
    assert main(["validate", str(path)]) == 2
    output = capsys.readouterr()
    assert output.out == ""
    assert "blue-drake: error:" in output.err


def test_run_writes_non_overwriting_csv_artifacts(
    tmp_path: Path, capsys
) -> None:
    output_dir = tmp_path / "run"
    custom = REPO_ROOT / "scenarios" / "custom_sensors.toml"
    arguments = [
        "run",
        str(custom),
        "--no-visualizer",
        "--duration",
        "0.05",
        "--realtime-rate",
        "0",
        "--output-dir",
        str(output_dir),
        "--log-period",
        "0.02",
    ]
    assert main(arguments) == 0
    manifest = json.loads(
        (output_dir / "manifest.json").read_text(encoding="utf-8")
    )
    assert manifest["artifact_schema_version"] == 1
    assert manifest["simulated_duration_s"] == 0.05
    assert manifest["scenario"]["vehicles"][0]["sensors"][0][
        "supplied_value"
    ] == [34.8, 1.2]
    assert {item["log_id"] for item in manifest["logs"]} == {
        "rov_1_state",
        "rov_1_student_depth_measurement",
        "rov_1_water_quality_measurement",
    }
    assert (
        (output_dir / "rov_1_state.csv")
        .read_text(encoding="utf-8")
        .startswith("time_s,quaternion_w_WB")
    )
    capsys.readouterr()
    assert main(arguments) == 2
    assert "already exists" in capsys.readouterr().err
