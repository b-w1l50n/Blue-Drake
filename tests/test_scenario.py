from __future__ import annotations

from pathlib import Path

import pytest

from blue_drake.scenario import load_scenario
from blue_drake.sensors import (
    CustomVectorSensorProfile,
    PressureSensorProfile,
)
from blue_drake.vehicles import VehicleKind

REPO_ROOT = Path(__file__).resolve().parents[1]
SHOWCASE_SCENARIO = REPO_ROOT / "scenarios" / "fleet_showcase.toml"


def test_fleet_showcase_declares_long_running_trimmed_configuration() -> None:
    scenario = load_scenario(SHOWCASE_SCENARIO)
    assert scenario.duration_s == 300.0
    assert len(scenario.vehicles) == 4
    rov = next(
        vehicle
        for vehicle in scenario.vehicles
        if vehicle.vehicle_id == "rov_1"
    )
    uuv = next(
        vehicle
        for vehicle in scenario.vehicles
        if vehicle.vehicle_id == "uuv_1"
    )
    assert rov.wrench_command_B[5] == pytest.approx(-4.905)
    assert uuv.applied_wrench_B[5] == pytest.approx(-1.962)


def test_mixed_scenario_loads_all_public_vehicle_categories() -> None:
    scenario = load_scenario(REPO_ROOT / "scenarios" / "mixed_marine.toml")
    assert scenario.name == "mixed-marine-foundation"
    assert scenario.schema_version == 1
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
    assert scenario.air_density_kg_m3 == pytest.approx(1.225)
    assert surface.wind_velocity_W_mps == (2.0, 0.0, 0.0)
    assert len(scenario.acoustic.transmissions) == 4
    assert scenario.acoustic.transmissions[0].source_id == "surface_1"
    glider = next(
        vehicle
        for vehicle in scenario.vehicles
        if vehicle.vehicle_id == "glider_1"
    )
    assert glider.glider_command == (-2.0, 0.0)


def test_custom_sensor_scenario_resolves_profiles_and_values() -> None:
    scenario = load_scenario(REPO_ROOT / "scenarios" / "custom_sensors.toml")
    assert len(scenario.sensor_profiles) == 2
    assert isinstance(scenario.sensor_profiles[0], CustomVectorSensorProfile)
    assert isinstance(scenario.sensor_profiles[1], PressureSensorProfile)
    water_quality = scenario.vehicles[0].sensors[0]
    assert water_quality.profile.channel_names == ("salinity", "turbidity")
    assert water_quality.supplied_value == (34.8, 1.2)
    assert water_quality.bias == (0.1, 0.0)


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


def test_unsupported_schema_version_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / "future.toml"
    path.write_text(
        """
schema_version = 99
name = "future"
[[vehicles]]
id = "rov_1"
preset = "rov"
position_W_m = [0, 0, -1]
""",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="unsupported scenario schema_version"):
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


def test_custom_profile_cannot_override_builtin_profile(tmp_path: Path) -> None:
    path = tmp_path / "override.toml"
    path.write_text(
        """
name = "override"
[[sensor_profiles]]
id = "blue-robotics-bar30"
display_name = "Not really a Bar30"
kind = "pressure"
maximum_pressure_Pa = 1
approximate_depth_rating_m = 1
nominal_depth_resolution_m = 1
temperature_accuracy_C = 1
[[vehicles]]
id = "rov_1"
preset = "rov"
position_W_m = [0, 0, -1]
""",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="cannot override built-in"):
        load_scenario(path)


def test_sourced_custom_profile_requires_source_url(tmp_path: Path) -> None:
    path = tmp_path / "missing_source.toml"
    path.write_text(
        """
name = "missing-source"
[[sensor_profiles]]
id = "lab-pressure"
display_name = "Lab pressure"
kind = "pressure"
provenance = "measured"
maximum_pressure_Pa = 200000
approximate_depth_rating_m = 10
nominal_depth_resolution_m = 0.01
temperature_accuracy_C = 1
[[vehicles]]
id = "rov_1"
preset = "rov"
position_W_m = [0, 0, -1]
""",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="require a source_url"):
        load_scenario(path)


def test_custom_imu_and_sonar_profiles_parse_in_si_units(
    tmp_path: Path,
) -> None:
    path = tmp_path / "custom_physical.toml"
    path.write_text(
        """
name = "custom-physical"
[[sensor_profiles]]
id = "lab-imu"
display_name = "Lab IMU"
kind = "imu"
gyroscope_range_radps = [10, 10, 10]
accelerometer_range_mps2 = [100, 100, 100]
gyroscope_noise_density_radps_per_sqrt_hz = 0.001
accelerometer_noise_density_mps2_per_sqrt_hz = 0.01
maximum_output_rate_hz = 100
roll_pitch_accuracy_rad_rms = 0.01
heading_accuracy_rad_rms = 0.02
[[sensor_profiles]]
id = "lab-altimeter"
display_name = "Lab Altimeter"
kind = "echosounder"
frequency_hz = 200000
minimum_range_m = 0.2
maximum_range_m = 75
horizontal_field_of_view_rad = 0.4
vertical_field_of_view_rad = 0.4
range_resolution_fraction = 0.01
depth_rating_m = 200
maximum_ping_rate_hz = 10
[[vehicles]]
id = "rov_1"
preset = "rov"
position_W_m = [0, 0, -2]
[[vehicles.sensors]]
id = "imu"
profile = "lab-imu"
[[vehicles.sensors]]
id = "altimeter"
profile = "lab-altimeter"
rpy_BS_deg = [0, 90, 0]
""",
        encoding="utf-8",
    )
    scenario = load_scenario(path)
    assert scenario.sensor_profiles[0].kind.value == "imu"
    assert scenario.sensor_profiles[1].kind.value == "echosounder"
    assert scenario.vehicles[0].sensors[0].profile.profile_id == "lab-imu"


def test_value_is_reserved_for_custom_vector_sensors(tmp_path: Path) -> None:
    path = tmp_path / "physical_value.toml"
    path.write_text(
        """
name = "physical-value"
[[vehicles]]
id = "rov_1"
preset = "rov"
position_W_m = [0, 0, -1]
[[vehicles.sensors]]
id = "depth"
profile = "blue-robotics-bar30"
value = [123]
""",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="requires a custom_vector"):
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


def test_vehicle_and_sensor_ids_are_artifact_safe(tmp_path: Path) -> None:
    path = tmp_path / "unsafe_id.toml"
    path.write_text(
        """
name = "unsafe-id"
[[vehicles]]
id = "../rov"
preset = "rov"
position_W_m = [0, 0, -1]
""",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="vehicle_id must start with a letter"):
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
