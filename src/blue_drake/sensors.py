"""Marine sensor profiles, mounting configuration, and measurement math."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from enum import StrEnum

import numpy as np
from numpy.typing import ArrayLike, NDArray

from blue_drake.identifiers import validate_identifier

Vector = NDArray[np.float64]
Vector3 = tuple[float, float, float]
GRAVITY_MPS2 = 9.80665


class SensorKind(StrEnum):
    """Physical measurement families represented by Blue Drake profiles."""

    PRESSURE = "pressure"
    IMU = "imu"
    ECHOSOUNDER = "echosounder"
    MULTIBEAM_ECHOSOUNDER = "multibeam_echosounder"
    FORWARD_LOOKING_SONAR = "forward_looking_sonar"
    CUSTOM_VECTOR = "custom_vector"


class ParameterProvenance(StrEnum):
    """Declared origin of user-configurable profile parameters."""

    PUBLISHED = "published"
    MEASURED = "measured"
    FITTED = "fitted"
    ASSUMED = "assumed"


def _vector(name: str, value: ArrayLike, size: int) -> Vector:
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {size} finite values")
    return result


def _positive(name: str, value: float) -> None:
    if value <= 0.0 or not math.isfinite(value):
        raise ValueError(f"{name} must be positive and finite")


def _profile_identity(
    profile_id: str,
    display_name: str,
    provenance: ParameterProvenance | str,
    source_url: str | None,
) -> ParameterProvenance:
    validate_identifier("profile_id", profile_id)
    if not display_name.strip():
        raise ValueError("sensor profile display_name is required")
    if source_url is not None and not source_url.strip():
        raise ValueError("source_url cannot be empty when supplied")
    try:
        normalized = ParameterProvenance(provenance)
    except ValueError as exc:
        raise ValueError(
            f"unsupported parameter provenance: {provenance}"
        ) from exc
    if normalized is not ParameterProvenance.ASSUMED and source_url is None:
        raise ValueError(f"{normalized} sensor parameters require a source_url")
    return normalized


@dataclass(frozen=True)
class PressureSensorProfile:
    """Operating envelope for a pressure/depth sensor."""

    profile_id: str
    display_name: str
    maximum_pressure_Pa: float
    approximate_depth_rating_m: float
    nominal_depth_resolution_m: float
    temperature_accuracy_C: float
    source_url: str | None
    source_retrieved: str = "2026-07-21"
    provenance: ParameterProvenance = ParameterProvenance.PUBLISHED
    kind: SensorKind = SensorKind.PRESSURE

    def __post_init__(self) -> None:
        if self.kind is not SensorKind.PRESSURE:
            raise ValueError("pressure profile kind must be pressure")
        object.__setattr__(
            self,
            "provenance",
            _profile_identity(
                self.profile_id,
                self.display_name,
                self.provenance,
                self.source_url,
            ),
        )
        for name in (
            "maximum_pressure_Pa",
            "approximate_depth_rating_m",
            "nominal_depth_resolution_m",
            "temperature_accuracy_C",
        ):
            _positive(name, getattr(self, name))


@dataclass(frozen=True)
class ImuSensorProfile:
    """Raw-sensor envelope for an inertial measurement unit."""

    profile_id: str
    display_name: str
    gyroscope_range_radps: Vector3
    accelerometer_range_mps2: Vector3
    gyroscope_noise_density_radps_per_sqrt_hz: float
    accelerometer_noise_density_mps2_per_sqrt_hz: float
    maximum_output_rate_hz: float
    roll_pitch_accuracy_rad_rms: float
    heading_accuracy_rad_rms: float
    source_url: str | None
    source_retrieved: str = "2026-07-21"
    provenance: ParameterProvenance = ParameterProvenance.PUBLISHED
    kind: SensorKind = SensorKind.IMU

    def __post_init__(self) -> None:
        if self.kind is not SensorKind.IMU:
            raise ValueError("IMU profile kind must be imu")
        object.__setattr__(
            self,
            "provenance",
            _profile_identity(
                self.profile_id,
                self.display_name,
                self.provenance,
                self.source_url,
            ),
        )
        for name in ("gyroscope_range_radps", "accelerometer_range_mps2"):
            values = _vector(name, getattr(self, name), 3)
            if np.any(values <= 0.0):
                raise ValueError(f"{name} must be positive")
            object.__setattr__(
                self, name, tuple(float(item) for item in values)
            )
        for name in (
            "gyroscope_noise_density_radps_per_sqrt_hz",
            "accelerometer_noise_density_mps2_per_sqrt_hz",
            "maximum_output_rate_hz",
            "roll_pitch_accuracy_rad_rms",
            "heading_accuracy_rad_rms",
        ):
            _positive(name, getattr(self, name))


@dataclass(frozen=True)
class SonarProfile:
    """Geometric and acoustic envelope for a sonar target."""

    profile_id: str
    display_name: str
    kind: SensorKind
    frequency_hz: float
    minimum_range_m: float
    maximum_range_m: float
    horizontal_field_of_view_rad: float
    vertical_field_of_view_rad: float
    range_resolution_fraction: float
    depth_rating_m: float
    maximum_ping_rate_hz: float | None
    source_url: str | None
    source_retrieved: str = "2026-07-21"
    provenance: ParameterProvenance = ParameterProvenance.PUBLISHED

    def __post_init__(self) -> None:
        if self.kind not in {
            SensorKind.ECHOSOUNDER,
            SensorKind.MULTIBEAM_ECHOSOUNDER,
            SensorKind.FORWARD_LOOKING_SONAR,
        }:
            raise ValueError("sonar profile must use a sonar sensor kind")
        object.__setattr__(
            self,
            "provenance",
            _profile_identity(
                self.profile_id,
                self.display_name,
                self.provenance,
                self.source_url,
            ),
        )
        for name in (
            "frequency_hz",
            "maximum_range_m",
            "horizontal_field_of_view_rad",
            "vertical_field_of_view_rad",
            "range_resolution_fraction",
            "depth_rating_m",
        ):
            _positive(name, getattr(self, name))
        if self.minimum_range_m < 0.0 or not math.isfinite(
            self.minimum_range_m
        ):
            raise ValueError("minimum_range_m must be finite and nonnegative")
        if self.minimum_range_m >= self.maximum_range_m:
            raise ValueError("minimum sonar range must be below maximum range")
        if self.maximum_ping_rate_hz is not None:
            _positive("maximum_ping_rate_hz", self.maximum_ping_rate_hz)


@dataclass(frozen=True)
class CustomVectorSensorProfile:
    """Metadata and bounds for an explicitly supplied numeric sensor vector."""

    profile_id: str
    display_name: str
    channel_names: tuple[str, ...]
    units: tuple[str, ...]
    minimum_values: tuple[float, ...]
    maximum_values: tuple[float, ...]
    default_values: tuple[float, ...]
    provenance: ParameterProvenance = ParameterProvenance.ASSUMED
    source_url: str | None = None
    source_retrieved: str = ""
    kind: SensorKind = SensorKind.CUSTOM_VECTOR

    def __post_init__(self) -> None:
        if self.kind is not SensorKind.CUSTOM_VECTOR:
            raise ValueError("custom vector profile kind must be custom_vector")
        object.__setattr__(
            self,
            "provenance",
            _profile_identity(
                self.profile_id,
                self.display_name,
                self.provenance,
                self.source_url,
            ),
        )
        channel_names = tuple(str(item).strip() for item in self.channel_names)
        units = tuple(str(item).strip() for item in self.units)
        if not channel_names or len(channel_names) > 64:
            raise ValueError("custom vector must contain 1 to 64 channels")
        if any(not item for item in channel_names):
            raise ValueError("custom vector channel names cannot be empty")
        if len(set(channel_names)) != len(channel_names):
            raise ValueError("custom vector channel names must be unique")
        if len(units) != len(channel_names) or any(not item for item in units):
            raise ValueError(
                "custom vector requires one nonempty unit per channel"
            )
        object.__setattr__(self, "channel_names", channel_names)
        object.__setattr__(self, "units", units)
        size = len(channel_names)
        for name in (
            "minimum_values",
            "maximum_values",
            "default_values",
        ):
            values = _vector(name, getattr(self, name), size)
            object.__setattr__(
                self, name, tuple(float(item) for item in values)
            )
        minimum = np.asarray(self.minimum_values)
        maximum = np.asarray(self.maximum_values)
        defaults = np.asarray(self.default_values)
        if np.any(minimum >= maximum):
            raise ValueError("custom vector minimums must be below maximums")
        if np.any(defaults < minimum) or np.any(defaults > maximum):
            raise ValueError("custom vector defaults must be within bounds")

    @property
    def size(self) -> int:
        """Return the number of user-supplied value channels."""

        return len(self.channel_names)


SensorProfile = (
    PressureSensorProfile
    | ImuSensorProfile
    | SonarProfile
    | CustomVectorSensorProfile
)


@dataclass(frozen=True)
class MountedSensorConfig:
    """A named sensor profile rigidly mounted to one vehicle body."""

    sensor_id: str
    profile: SensorProfile
    position_B_m: Vector3 = (0.0, 0.0, 0.0)
    rpy_BS_deg: Vector3 = (0.0, 0.0, 0.0)
    bias: tuple[float, ...] = ()
    supplied_value: tuple[float, ...] = ()

    def __post_init__(self) -> None:
        validate_identifier("sensor_id", self.sensor_id)
        for name in ("position_B_m", "rpy_BS_deg"):
            values = _vector(name, getattr(self, name), 3)
            object.__setattr__(
                self, name, tuple(float(item) for item in values)
            )
        raw_bias = self.bias if len(self.bias) else np.zeros(self.error_size)
        bias = _vector("bias", raw_bias, self.error_size)
        object.__setattr__(self, "bias", tuple(float(item) for item in bias))
        if isinstance(self.profile, CustomVectorSensorProfile):
            raw_value = (
                self.supplied_value
                if len(self.supplied_value)
                else self.profile.default_values
            )
            supplied_value = _vector(
                "supplied_value", raw_value, self.profile.size
            )
            object.__setattr__(
                self,
                "supplied_value",
                tuple(float(item) for item in supplied_value),
            )
        elif self.supplied_value:
            raise ValueError("supplied_value requires a custom vector profile")

    @property
    def error_size(self) -> int:
        if self.profile.kind is SensorKind.PRESSURE:
            return 2
        if self.profile.kind is SensorKind.IMU:
            return 6
        if isinstance(self.profile, CustomVectorSensorProfile):
            return self.profile.size
        return 1


def bar02_profile() -> PressureSensorProfile:
    """Blue Robotics Bar02 published envelope; not a calibrated error model."""

    return PressureSensorProfile(
        profile_id="blue-robotics-bar02",
        display_name="Blue Robotics Bar02",
        maximum_pressure_Pa=200_000.0,
        approximate_depth_rating_m=10.0,
        nominal_depth_resolution_m=0.00016,
        temperature_accuracy_C=2.0,
        source_url=(
            "https://bluerobotics.com/store/sensors-cameras/sensors/"
            "bar-depth-pressure-sensor/"
        ),
    )


def bar30_profile() -> PressureSensorProfile:
    """Blue Robotics Bar30 published envelope; not a calibrated error model."""

    return PressureSensorProfile(
        profile_id="blue-robotics-bar30",
        display_name="Blue Robotics Bar30",
        maximum_pressure_Pa=3_000_000.0,
        approximate_depth_rating_m=300.0,
        nominal_depth_resolution_m=0.002,
        temperature_accuracy_C=4.0,
        source_url=(
            "https://bluerobotics.com/store/sensors-cameras/sensors/"
            "bar-depth-pressure-sensor/"
        ),
    )


def xsens_mti_630r_profile() -> ImuSensorProfile:
    """Return published raw-sensor and AHRS headline values for MTi-630R."""

    return ImuSensorProfile(
        profile_id="xsens-mti-630r",
        display_name="Xsens MTi-630R",
        gyroscope_range_radps=(
            math.radians(2000.0),
            math.radians(2000.0),
            math.radians(2000.0),
        ),
        accelerometer_range_mps2=(
            10.0 * GRAVITY_MPS2,
            10.0 * GRAVITY_MPS2,
            15.0 * GRAVITY_MPS2,
        ),
        gyroscope_noise_density_radps_per_sqrt_hz=math.radians(0.007),
        accelerometer_noise_density_mps2_per_sqrt_hz=(60e-6 * GRAVITY_MPS2),
        maximum_output_rate_hz=2000.0,
        roll_pitch_accuracy_rad_rms=math.radians(0.2),
        heading_accuracy_rad_rms=math.radians(1.0),
        source_url="https://www.movella.com/hubfs/Downloads/Leaflets/MTi-630R.pdf",
    )


def xsens_avior_ahrs_profile() -> ImuSensorProfile:
    """Return published raw-sensor and AHRS headline values for Avior."""

    gyro_range = math.radians(300.0)
    accel_range = 8.0 * GRAVITY_MPS2
    return ImuSensorProfile(
        profile_id="xsens-avior-ahrs",
        display_name="Xsens Avior AHRS",
        gyroscope_range_radps=(gyro_range, gyro_range, gyro_range),
        accelerometer_range_mps2=(
            accel_range,
            accel_range,
            accel_range,
        ),
        gyroscope_noise_density_radps_per_sqrt_hz=math.radians(0.004),
        accelerometer_noise_density_mps2_per_sqrt_hz=(15e-6 * GRAVITY_MPS2),
        maximum_output_rate_hz=2000.0,
        roll_pitch_accuracy_rad_rms=math.radians(0.2),
        heading_accuracy_rad_rms=math.radians(1.0),
        source_url=(
            "https://www.movella.com/hubfs/A-and-M-Avior/"
            "A%26M%20-%20Datasheet%20Avior%20LR.pdf"
        ),
    )


def ping_sonar_profile() -> SonarProfile:
    """Return the current Blue Robotics Ping echosounder envelope."""

    beam = math.radians(25.0)
    return SonarProfile(
        profile_id="blue-robotics-ping-sonar",
        display_name="Blue Robotics Ping Sonar",
        kind=SensorKind.ECHOSOUNDER,
        frequency_hz=115_000.0,
        minimum_range_m=0.3,
        maximum_range_m=100.0,
        horizontal_field_of_view_rad=beam,
        vertical_field_of_view_rad=beam,
        range_resolution_fraction=0.005,
        depth_rating_m=300.0,
        maximum_ping_rate_hz=None,
        source_url=(
            "https://bluerobotics.com/store/sonars/echosounders/"
            "ping-sonar-r2-rp/"
        ),
    )


def surveyor_240_16_profile() -> SonarProfile:
    """Return the Cerulean Surveyor 240-16 published operating envelope."""

    return SonarProfile(
        profile_id="cerulean-surveyor-240-16",
        display_name="Cerulean Surveyor 240-16",
        kind=SensorKind.MULTIBEAM_ECHOSOUNDER,
        frequency_hz=240_000.0,
        minimum_range_m=0.0,
        maximum_range_m=50.0,
        horizontal_field_of_view_rad=math.radians(80.0),
        vertical_field_of_view_rad=math.radians(4.0),
        range_resolution_fraction=0.005,
        depth_rating_m=300.0,
        maximum_ping_rate_hz=20.0,
        source_url=(
            "https://docs.ceruleansonar.com/c/surveyor-240-16/specifications"
        ),
    )


def omniscan_450fs_300m_profile() -> SonarProfile:
    """Return the Cerulean Omniscan 450FS 300 m model envelope."""

    return SonarProfile(
        profile_id="cerulean-omniscan-450fs-300m",
        display_name="Cerulean Omniscan 450FS 300 m",
        kind=SensorKind.FORWARD_LOOKING_SONAR,
        frequency_hz=450_000.0,
        minimum_range_m=0.0,
        maximum_range_m=120.0,
        horizontal_field_of_view_rad=math.radians(0.8),
        vertical_field_of_view_rad=math.radians(50.0),
        range_resolution_fraction=1.0 / 1200.0,
        depth_rating_m=300.0,
        maximum_ping_rate_hz=20.0,
        source_url=(
            "https://ceruleansonar.com/product/omniscan-450fs-100m-300m/"
        ),
    )


PROFILE_FACTORIES = {
    "blue-robotics-bar02": bar02_profile,
    "blue-robotics-bar30": bar30_profile,
    "blue-robotics-ping-sonar": ping_sonar_profile,
    "cerulean-omniscan-450fs-300m": omniscan_450fs_300m_profile,
    "cerulean-surveyor-240-16": surveyor_240_16_profile,
    "xsens-avior-ahrs": xsens_avior_ahrs_profile,
    "xsens-mti-630r": xsens_mti_630r_profile,
}


def sensor_profile(
    profile_id: str,
    *,
    custom_profiles: Mapping[str, SensorProfile] | None = None,
) -> SensorProfile:
    """Resolve a stable, case-insensitive sensor profile identifier."""

    normalized = profile_id.strip().lower().replace("_", "-")
    if custom_profiles is not None and normalized in custom_profiles:
        return custom_profiles[normalized]
    try:
        return PROFILE_FACTORIES[normalized]()
    except KeyError as exc:
        choices = ", ".join(sorted(PROFILE_FACTORIES))
        raise ValueError(
            f"unknown sensor profile {profile_id!r}; choose {choices}"
        ) from exc


def custom_vector_measurement(
    profile: CustomVectorSensorProfile,
    *,
    values: ArrayLike,
    error: ArrayLike | None = None,
) -> Vector:
    """Return bounded supplied values followed by a collective valid flag."""

    supplied = _vector("values", values, profile.size)
    errors = (
        np.zeros(profile.size)
        if error is None
        else _vector("error", error, profile.size)
    )
    measured = supplied + errors
    minimum = np.asarray(profile.minimum_values)
    maximum = np.asarray(profile.maximum_values)
    valid = np.all((measured >= minimum) & (measured <= maximum))
    return np.concatenate((np.clip(measured, minimum, maximum), [float(valid)]))


def pressure_measurement(
    profile: PressureSensorProfile,
    *,
    sensor_z_W_m: float,
    water_density_kg_m3: float,
    gravity_mps2: float,
    surface_pressure_Pa: float,
    water_temperature_C: float,
    error: ArrayLike = (0.0, 0.0),
) -> Vector:
    """Return pressure, inferred depth, temperature, and valid flag."""

    for name, value in (
        ("water_density_kg_m3", water_density_kg_m3),
        ("gravity_mps2", gravity_mps2),
        ("surface_pressure_Pa", surface_pressure_Pa),
    ):
        _positive(name, value)
    if not math.isfinite(sensor_z_W_m) or not math.isfinite(
        water_temperature_C
    ):
        raise ValueError("sensor position and temperature must be finite")
    error = _vector("error", error, 2)
    true_depth = max(0.0, -sensor_z_W_m)
    true_pressure = surface_pressure_Pa + (
        water_density_kg_m3 * gravity_mps2 * true_depth
    )
    measured_pressure = true_pressure + error[0]
    measured_temperature = water_temperature_C + error[1]
    valid = 0.0 <= measured_pressure <= profile.maximum_pressure_Pa
    clipped_pressure = float(
        np.clip(measured_pressure, 0.0, profile.maximum_pressure_Pa)
    )
    inferred_depth = max(
        0.0,
        (clipped_pressure - surface_pressure_Pa)
        / (water_density_kg_m3 * gravity_mps2),
    )
    return np.array(
        [clipped_pressure, inferred_depth, measured_temperature, float(valid)]
    )


def raw_imu_measurement(
    profile: ImuSensorProfile,
    *,
    rotation_WS: ArrayLike,
    angular_velocity_W_radps: ArrayLike,
    translational_acceleration_WS_W_mps2: ArrayLike,
    gravity_W_mps2: ArrayLike = (0.0, 0.0, -9.81),
    error: ArrayLike = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
) -> Vector:
    """Return clipped angular velocity, specific force, and validity flags."""

    rotation = np.asarray(rotation_WS, dtype=float)
    if rotation.shape != (3, 3) or not np.allclose(
        rotation.T @ rotation, np.eye(3), atol=1e-8
    ):
        raise ValueError("rotation_WS must be a 3x3 orthonormal matrix")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-8):
        raise ValueError("rotation_WS must be a proper rotation")
    angular_W = _vector("angular_velocity_W_radps", angular_velocity_W_radps, 3)
    acceleration_W = _vector(
        "translational_acceleration_WS_W_mps2",
        translational_acceleration_WS_W_mps2,
        3,
    )
    gravity_W = _vector("gravity_W_mps2", gravity_W_mps2, 3)
    error = _vector("error", error, 6)
    rotation_SW = rotation.T
    gyro = rotation_SW @ angular_W + error[:3]
    specific_force = rotation_SW @ (acceleration_W - gravity_W) + error[3:]
    gyro_limits = np.asarray(profile.gyroscope_range_radps)
    accel_limits = np.asarray(profile.accelerometer_range_mps2)
    gyro_valid = np.all(np.abs(gyro) <= gyro_limits)
    accel_valid = np.all(np.abs(specific_force) <= accel_limits)
    return np.concatenate(
        (
            np.clip(gyro, -gyro_limits, gyro_limits),
            np.clip(specific_force, -accel_limits, accel_limits),
            [float(gyro_valid), float(accel_valid)],
        )
    )


def flat_seafloor_range(
    profile: SonarProfile,
    *,
    sensor_origin_W_m: ArrayLike,
    beam_direction_W: ArrayLike,
    seafloor_z_W_m: float,
    range_error_m: float = 0.0,
) -> Vector:
    """Intersect a sonar center ray with a horizontal seafloor plane.

    The result is ``[range_m, valid]``. This geometric abstraction does not
    model beam footprints, echoes, confidence, imagery, or acoustic propagation.
    """

    origin = _vector("sensor_origin_W_m", sensor_origin_W_m, 3)
    direction = _vector("beam_direction_W", beam_direction_W, 3)
    norm = np.linalg.norm(direction)
    if not np.isclose(norm, 1.0, atol=1e-9):
        raise ValueError("beam_direction_W must be a unit vector")
    if not math.isfinite(seafloor_z_W_m) or not math.isfinite(range_error_m):
        raise ValueError("seafloor and range error must be finite")
    if direction[2] >= -1e-12:
        return np.array([profile.maximum_range_m, 0.0])
    distance = (seafloor_z_W_m - origin[2]) / direction[2]
    measured = distance + range_error_m
    valid = profile.minimum_range_m <= measured <= profile.maximum_range_m
    return np.array(
        [
            float(
                np.clip(
                    measured, profile.minimum_range_m, profile.maximum_range_m
                )
            ),
            float(valid),
        ]
    )
