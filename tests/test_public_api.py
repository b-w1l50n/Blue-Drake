from __future__ import annotations

import subprocess
import sys

import blue_drake


def test_lightweight_public_api_is_explicit_and_stable() -> None:
    assert blue_drake.__all__ == [
        "ActuatorBankConfig",
        "ActuatorKind",
        "CustomVectorSensorProfile",
        "FixedActuatorConfig",
        "GliderControlConfig",
        "GliderWingConfig",
        "GridPath",
        "HydrostaticMode",
        "MarineVehicleConfig",
        "MarineGrid",
        "MountedSensorConfig",
        "ParameterProvenance",
        "SensorKind",
        "StationKeepingGains",
        "VehicleKind",
        "WrenchAllocation",
        "__version__",
        "allocate_wrench",
        "glider_preset",
        "plan_grid_path",
        "rov_preset",
        "sensor_profile",
        "station_keeping_wrench",
        "usv_preset",
        "uuv_preset",
    ]


def test_public_api_does_not_eagerly_import_drake() -> None:
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; import blue_drake; "
            "assert 'pydrake' not in sys.modules",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
