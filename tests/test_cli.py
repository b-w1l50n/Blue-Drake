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


def test_package_version_has_release_value() -> None:
    assert __version__ == "1.0.1"


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


def test_doctor_reports_runtime_without_importing_pydrake() -> None:
    code = (
        "import json, sys; "
        "from blue_drake.cli import main; "
        "status = main(['doctor', '--json']); "
        "assert status in (0, 1); "
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
    result = json.loads(completed.stdout)
    assert result["doctor_schema_version"] == 1
    assert result["blue_drake_version"] == __version__
    assert result["meshcat"]["default_host"] == "localhost"
    assert {check["id"] for check in result["checks"]} == {
        "python",
        "numpy",
        "pydrake",
        "release-platform",
    }


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
    assert manifest["artifact_schema_version"] == 2
    assert manifest["software"]["blue_drake_version"] == __version__
    assert manifest["software"]["drake_version"]
    assert manifest["software"]["numpy_version"]
    assert manifest["software"]["python_version"]
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


def test_identical_runs_produce_byte_identical_artifacts(
    tmp_path: Path, capsys
) -> None:
    custom = REPO_ROOT / "scenarios" / "custom_sensors.toml"
    destinations = [tmp_path / "first", tmp_path / "second"]
    for destination in destinations:
        assert (
            main(
                [
                    "run",
                    str(custom),
                    "--no-visualizer",
                    "--duration",
                    "0.05",
                    "--realtime-rate",
                    "0",
                    "--output-dir",
                    str(destination),
                    "--log-period",
                    "0.02",
                ]
            )
            == 0
        )
        capsys.readouterr()

    first_files = sorted(path.name for path in destinations[0].iterdir())
    second_files = sorted(path.name for path in destinations[1].iterdir())
    assert first_files == second_files
    for filename in first_files:
        assert (destinations[0] / filename).read_bytes() == (
            destinations[1] / filename
        ).read_bytes()


def test_runtime_failure_is_reported_without_traceback(
    monkeypatch, capsys
) -> None:
    def fail_run(args, scenario) -> int:
        raise RuntimeError("simulator failed cleanly")

    monkeypatch.setattr("blue_drake.cli._run", fail_run)
    assert main(["run", str(MIXED_SCENARIO), "--no-visualizer"]) == 2
    output = capsys.readouterr()
    assert output.out == ""
    assert output.err == "blue-drake: error: simulator failed cleanly\n"
