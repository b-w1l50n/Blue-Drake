from __future__ import annotations

import math

import numpy as np
import pytest

from blue_drake.sensors import (
    GRAVITY_MPS2,
    CustomVectorSensorProfile,
    MountedSensorConfig,
    SensorKind,
    bar02_profile,
    bar30_profile,
    custom_vector_measurement,
    flat_seafloor_range,
    omniscan_450fs_300m_profile,
    ping_sonar_profile,
    pressure_measurement,
    raw_imu_measurement,
    sensor_profile,
    surveyor_240_16_profile,
    xsens_avior_ahrs_profile,
    xsens_mti_630r_profile,
)


def test_public_profiles_cover_requested_manufacturers() -> None:
    profiles = (
        bar02_profile(),
        bar30_profile(),
        ping_sonar_profile(),
        surveyor_240_16_profile(),
        omniscan_450fs_300m_profile(),
        xsens_avior_ahrs_profile(),
        xsens_mti_630r_profile(),
    )
    assert all(
        profile.source_url.startswith("https://") for profile in profiles
    )
    assert {profile.kind for profile in profiles} >= {
        SensorKind.PRESSURE,
        SensorKind.IMU,
        SensorKind.ECHOSOUNDER,
        SensorKind.MULTIBEAM_ECHOSOUNDER,
        SensorKind.FORWARD_LOOKING_SONAR,
    }


def test_profile_lookup_is_stable_and_rejects_unknown_names() -> None:
    assert sensor_profile("XSENS_MTI_630R").profile_id == "xsens-mti-630r"
    with pytest.raises(ValueError, match="unknown sensor profile"):
        sensor_profile("imaginary-dvl")


def test_custom_vector_preserves_metadata_and_reports_bounds() -> None:
    profile = CustomVectorSensorProfile(
        profile_id="water-quality",
        display_name="Student Water Quality Package",
        channel_names=("salinity", "turbidity"),
        units=("PSU", "NTU"),
        minimum_values=(0.0, 0.0),
        maximum_values=(50.0, 100.0),
        default_values=(35.0, 2.0),
    )
    assert profile.size == 2
    assert profile.provenance.value == "assumed"
    assert MountedSensorConfig("defaulted", profile).supplied_value == (
        35.0,
        2.0,
    )
    assert custom_vector_measurement(
        profile, values=(34.5, 3.0), error=(0.5, -1.0)
    ) == pytest.approx([35.0, 2.0, 1.0])
    assert custom_vector_measurement(
        profile, values=(55.0, 3.0)
    ) == pytest.approx([50.0, 3.0, 0.0])


def test_custom_vector_rejects_ambiguous_channel_metadata() -> None:
    with pytest.raises(ValueError, match="one nonempty unit"):
        CustomVectorSensorProfile(
            profile_id="bad",
            display_name="Bad",
            channel_names=("one", "two"),
            units=("m",),
            minimum_values=(0.0, 0.0),
            maximum_values=(1.0, 1.0),
            default_values=(0.5, 0.5),
        )


def test_xsens_avior_is_distinct_from_mti_630r() -> None:
    avior = sensor_profile("xsens-avior-ahrs")
    mti = sensor_profile("xsens-mti-630r")
    assert avior.profile_id == "xsens-avior-ahrs"
    assert avior.gyroscope_range_radps[0] == pytest.approx(math.radians(300.0))
    assert avior.accelerometer_range_mps2[0] == pytest.approx(
        8.0 * GRAVITY_MPS2
    )
    assert avior.gyroscope_noise_density_radps_per_sqrt_hz < (
        mti.gyroscope_noise_density_radps_per_sqrt_hz
    )


def test_bar30_hydrostatic_pressure_matches_analytic_result() -> None:
    measurement = pressure_measurement(
        bar30_profile(),
        sensor_z_W_m=-10.0,
        water_density_kg_m3=1025.0,
        gravity_mps2=9.81,
        surface_pressure_Pa=101_325.0,
        water_temperature_C=12.0,
    )
    expected = 101_325.0 + 1025.0 * 9.81 * 10.0
    assert measurement == pytest.approx([expected, 10.0, 12.0, 1.0])


def test_pressure_error_changes_pressure_and_inferred_depth() -> None:
    ideal = pressure_measurement(
        bar02_profile(),
        sensor_z_W_m=-2.0,
        water_density_kg_m3=1000.0,
        gravity_mps2=10.0,
        surface_pressure_Pa=100_000.0,
        water_temperature_C=20.0,
    )
    measured = pressure_measurement(
        bar02_profile(),
        sensor_z_W_m=-2.0,
        water_density_kg_m3=1000.0,
        gravity_mps2=10.0,
        surface_pressure_Pa=100_000.0,
        water_temperature_C=20.0,
        error=(1000.0, -0.5),
    )
    assert measured[:3] - ideal[:3] == pytest.approx([1000.0, 0.1, -0.5])


def test_pressure_sensor_uses_air_temperature_above_water() -> None:
    measurement = pressure_measurement(
        bar30_profile(),
        sensor_z_W_m=2.0,
        water_density_kg_m3=1025.0,
        gravity_mps2=9.81,
        surface_pressure_Pa=101_325.0,
        water_temperature_C=8.0,
        air_temperature_C=22.0,
    )
    assert measurement == pytest.approx([101_325.0, 0.0, 22.0, 1.0])


def test_stationary_level_imu_reports_upward_specific_force() -> None:
    measurement = raw_imu_measurement(
        xsens_mti_630r_profile(),
        rotation_WS=np.eye(3),
        angular_velocity_W_radps=np.zeros(3),
        translational_acceleration_WS_W_mps2=np.zeros(3),
        gravity_W_mps2=(0.0, 0.0, -GRAVITY_MPS2),
    )
    assert measurement[:6] == pytest.approx(
        [0.0, 0.0, 0.0, 0.0, 0.0, GRAVITY_MPS2]
    )
    assert measurement[6:] == pytest.approx([1.0, 1.0])


def test_imu_measurement_rotates_world_motion_into_sensor_frame() -> None:
    yaw = math.pi / 2.0
    rotation_WS = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    measurement = raw_imu_measurement(
        xsens_mti_630r_profile(),
        rotation_WS=rotation_WS,
        angular_velocity_W_radps=(1.0, 0.0, 0.0),
        translational_acceleration_WS_W_mps2=(0.0, 0.0, -9.81),
    )
    assert measurement[:3] == pytest.approx([0.0, -1.0, 0.0], abs=1e-12)
    assert measurement[3:6] == pytest.approx(np.zeros(3), abs=1e-12)


def test_downward_sonar_center_ray_intersects_flat_seafloor() -> None:
    result = flat_seafloor_range(
        ping_sonar_profile(),
        sensor_origin_W_m=(1.0, 2.0, -3.0),
        beam_direction_W=(0.0, 0.0, -1.0),
        seafloor_z_W_m=-13.0,
    )
    assert result == pytest.approx([10.0, 1.0])


def test_upward_or_out_of_range_sonar_ray_is_invalid() -> None:
    profile = ping_sonar_profile()
    upward = flat_seafloor_range(
        profile,
        sensor_origin_W_m=(0.0, 0.0, -2.0),
        beam_direction_W=(0.0, 0.0, 1.0),
        seafloor_z_W_m=-10.0,
    )
    too_far = flat_seafloor_range(
        profile,
        sensor_origin_W_m=(0.0, 0.0, 0.0),
        beam_direction_W=(0.0, 0.0, -1.0),
        seafloor_z_W_m=-200.0,
    )
    assert upward == pytest.approx([100.0, 0.0])
    assert too_far == pytest.approx([100.0, 0.0])


def test_sonar_center_ray_is_invalid_when_sensor_is_in_air() -> None:
    result = flat_seafloor_range(
        ping_sonar_profile(),
        sensor_origin_W_m=(0.0, 0.0, 0.01),
        beam_direction_W=(0.0, 0.0, -1.0),
        seafloor_z_W_m=-10.0,
    )
    assert result == pytest.approx([100.0, 0.0])


def test_sonar_is_invalid_below_profile_depth_rating() -> None:
    result = flat_seafloor_range(
        ping_sonar_profile(),
        sensor_origin_W_m=(0.0, 0.0, -301.0),
        beam_direction_W=(0.0, 0.0, -1.0),
        seafloor_z_W_m=-310.0,
    )
    assert result == pytest.approx([100.0, 0.0])


def test_range_error_cannot_create_target_outside_true_envelope() -> None:
    result = flat_seafloor_range(
        ping_sonar_profile(),
        sensor_origin_W_m=(0.0, 0.0, -2.0),
        beam_direction_W=(0.0, 0.0, -1.0),
        seafloor_z_W_m=-202.0,
        range_error_m=-150.0,
    )
    assert result == pytest.approx([50.0, 0.0])


def test_sonar_rejects_seafloor_above_water_surface() -> None:
    with pytest.raises(ValueError, match="seafloor must be below"):
        flat_seafloor_range(
            ping_sonar_profile(),
            sensor_origin_W_m=(0.0, 0.0, -2.0),
            beam_direction_W=(0.0, 0.0, -1.0),
            seafloor_z_W_m=1.0,
        )


def test_sonar_profile_rejects_nonphysical_angular_envelope() -> None:
    values = vars(ping_sonar_profile()) | {"horizontal_field_of_view_rad": 7.0}
    with pytest.raises(ValueError, match="cannot exceed"):
        type(ping_sonar_profile())(**values)


def test_mount_bias_dimension_depends_on_sensor_kind() -> None:
    config = MountedSensorConfig(
        sensor_id="imu",
        profile=xsens_mti_630r_profile(),
        bias=(0.0,) * 6,
    )
    assert config.error_size == 6
    with pytest.raises(ValueError, match="bias must contain 6"):
        MountedSensorConfig(
            sensor_id="bad_imu",
            profile=xsens_mti_630r_profile(),
            bias=(0.0, 0.0),
        )


def test_imu_rejects_improper_rotation() -> None:
    reflection = np.diag([1.0, 1.0, -1.0])
    with pytest.raises(ValueError, match="proper rotation"):
        raw_imu_measurement(
            xsens_mti_630r_profile(),
            rotation_WS=reflection,
            angular_velocity_W_radps=np.zeros(3),
            translational_acceleration_WS_W_mps2=np.zeros(3),
        )
