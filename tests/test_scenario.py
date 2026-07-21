from __future__ import annotations

from pathlib import Path

import pytest

from blue_drake.scenario import load_scenario
from blue_drake.vehicles import VehicleKind

REPO_ROOT = Path(__file__).resolve().parents[1]


def test_mixed_scenario_loads_all_public_vehicle_categories() -> None:
    scenario = load_scenario(REPO_ROOT / "scenarios" / "mixed_marine.toml")
    assert scenario.name == "mixed-marine-foundation"
    assert {vehicle.config.kind for vehicle in scenario.vehicles} == set(
        VehicleKind
    )
    assert scenario.acoustic.modem.profile_id == "divenet-sealink-3km-oem"
    surface = next(
        vehicle
        for vehicle in scenario.vehicles
        if vehicle.vehicle_id == "surface_1"
    )
    assert surface.wrench_command_B[3] == 8.0
    assert {sensor.profile.profile_id for sensor in surface.sensors} == {
        "xsens-avior-ahrs",
        "cerulean-surveyor-240-16",
    }
    assert scenario.seafloor_z_W_m == -20.0
    assert scenario.world_extent_m == 80.0
    assert len(scenario.acoustic.transmissions) == 4
    assert scenario.acoustic.transmissions[0].source_id == "surface_1"
    glider = next(
        vehicle
        for vehicle in scenario.vehicles
        if vehicle.vehicle_id == "glider_1"
    )
    assert glider.glider_command == (-2.0, 0.0)


def test_unknown_scenario_keys_are_rejected(tmp_path: Path) -> None:
    path = tmp_path / "bad.toml"
    path.write_text(
        """
name = "bad"
secret_switch = true
[network]
modem = "divenet-sealink-3km-oem"
[[vehicles]]
id = "rov_1"
preset = "rov"
position_W_m = [0, 0, -1]
""",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="unknown scenario keys"):
        load_scenario(path)


def test_subsea_vehicle_cannot_start_above_water(tmp_path: Path) -> None:
    path = tmp_path / "above.toml"
    path.write_text(
        """
name = "above"
[[vehicles]]
id = "rov_1"
preset = "rov"
position_W_m = [0, 0, 0.1]
""",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="below the waterline"):
        load_scenario(path)


def test_glider_rejects_unmodeled_wrench_command(tmp_path: Path) -> None:
    path = tmp_path / "glider_command.toml"
    path.write_text(
        """
name = "glider-command"
[[vehicles]]
id = "glider_1"
preset = "glider"
position_W_m = [0, 0, -2]
wrench_command_B = [0, 0, 0, 1, 0, 0]
""",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="no milestone-2 actuator model"):
        load_scenario(path)


def test_sensor_tables_are_strictly_validated(tmp_path: Path) -> None:
    path = tmp_path / "bad_sensor.toml"
    path.write_text(
        """
name = "bad-sensor"
[[vehicles]]
id = "rov_1"
preset = "rov"
position_W_m = [0, 0, -2]
[[vehicles.sensors]]
id = "depth"
profile = "blue-robotics-bar30"
imaginary_setting = true
""",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="unknown vehicle 1 sensor 1 keys"):
        load_scenario(path)


def test_duplicate_sensor_ids_are_rejected(tmp_path: Path) -> None:
    path = tmp_path / "duplicate_sensor.toml"
    path.write_text(
        """
name = "duplicate-sensor"
[[vehicles]]
id = "rov_1"
preset = "rov"
position_W_m = [0, 0, -2]
[[vehicles.sensors]]
id = "depth"
profile = "blue-robotics-bar30"
[[vehicles.sensors]]
id = "depth"
profile = "blue-robotics-bar02"
""",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="sensor IDs for rov_1 must be unique"):
        load_scenario(path)


def test_acoustic_transmission_references_known_vehicles(
    tmp_path: Path,
) -> None:
    path = tmp_path / "bad_transmission.toml"
    path.write_text(
        """
name = "bad-transmission"
duration_s = 2
[network]
[[network.transmissions]]
id = "missing-node"
source = "rov_1"
destination = "ghost"
start_time_s = 0.5
payload_bytes = 8
[[vehicles]]
id = "rov_1"
preset = "rov"
position_W_m = [0, 0, -2]
""",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="references unknown vehicle"):
        load_scenario(path)


def test_acoustic_integer_fields_do_not_silently_truncate(
    tmp_path: Path,
) -> None:
    path = tmp_path / "fractional_payload.toml"
    path.write_text(
        """
name = "fractional-payload"
[network]
[[network.transmissions]]
id = "bad-payload"
source = "rov_1"
destination = "rov_2"
start_time_s = 0.5
payload_bytes = 8.5
[[vehicles]]
id = "rov_1"
preset = "rov"
position_W_m = [0, 0, -2]
[[vehicles]]
id = "rov_2"
preset = "rov"
position_W_m = [1, 0, -2]
""",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="payload_bytes must be an integer"):
        load_scenario(path)
