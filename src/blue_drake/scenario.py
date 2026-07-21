"""Strict TOML scenarios for reproducible Blue Drake experiments."""

from __future__ import annotations

import math
import tomllib
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from blue_drake.acoustics import (
    AcousticChannelConfig,
    AcousticTransmissionRequest,
    ModemProfile,
    modem_profile,
)
from blue_drake.identifiers import validate_identifier
from blue_drake.sensors import (
    PROFILE_FACTORIES,
    CustomVectorSensorProfile,
    ImuSensorProfile,
    MountedSensorConfig,
    ParameterProvenance,
    PressureSensorProfile,
    SensorKind,
    SensorProfile,
    SonarProfile,
    sensor_profile,
)
from blue_drake.vehicles import MarineVehicleConfig, VehicleKind, vehicle_preset

Vector3 = tuple[float, float, float]
Vector2 = tuple[float, float]
Vector6 = tuple[float, float, float, float, float, float]
CURRENT_SCENARIO_SCHEMA_VERSION = 1


def _mapping(name: str, value: Any) -> dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a TOML table")
    return dict(value)


def _reject_unknown(
    values: Mapping[str, Any], allowed: set[str], context: str
) -> None:
    unknown = set(values) - allowed
    if unknown:
        raise ValueError(
            f"unknown {context} keys: " + ", ".join(sorted(unknown))
        )


def _vector(name: str, value: Any, size: int) -> tuple[float, ...]:
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain {size} numbers") from exc
    if len(result) != size or not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain {size} finite numbers")
    return result


def _integer(name: str, value: Any) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    return value


def _string_vector(name: str, value: Any) -> tuple[str, ...]:
    if not isinstance(value, list) or not all(
        isinstance(item, str) for item in value
    ):
        raise ValueError(f"{name} must be an array of strings")
    result = tuple(item.strip() for item in value)
    if any(not item for item in result):
        raise ValueError(f"{name} cannot contain empty strings")
    return result


def _numeric_vector(name: str, value: Any) -> tuple[float, ...]:
    if not isinstance(value, list) or not all(
        isinstance(item, int | float) and not isinstance(item, bool)
        for item in value
    ):
        raise ValueError(f"{name} must be an array of numbers")
    result = tuple(float(item) for item in value)
    if not result or not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain finite numbers")
    return result


def _number(name: str, value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True)
class ScenarioVehicle:
    """One configured vehicle and its initial simulation inputs."""

    vehicle_id: str
    config: MarineVehicleConfig
    initial_position_W_m: Vector3
    initial_rpy_deg: Vector3 = (0.0, 0.0, 0.0)
    water_current_W_mps: Vector3 = (0.0, 0.0, 0.0)
    wind_velocity_W_mps: Vector3 = (0.0, 0.0, 0.0)
    wrench_command_B: Vector6 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    applied_wrench_B: Vector6 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    glider_command: Vector2 = (0.0, 0.0)
    sensors: tuple[MountedSensorConfig, ...] = ()

    def __post_init__(self) -> None:
        validate_identifier("vehicle_id", self.vehicle_id)
        for name, size in (
            ("initial_position_W_m", 3),
            ("initial_rpy_deg", 3),
            ("water_current_W_mps", 3),
            ("wind_velocity_W_mps", 3),
            ("wrench_command_B", 6),
            ("applied_wrench_B", 6),
            ("glider_command", 2),
        ):
            object.__setattr__(
                self, name, _vector(name, getattr(self, name), size)
            )
        if (
            self.config.kind is VehicleKind.USV
            and abs(self.initial_position_W_m[2]) > 0.25
        ):
            raise ValueError("USV body origin must start near the waterline")
        if (
            self.config.kind is not VehicleKind.USV
            and self.initial_position_W_m[2] >= 0.0
        ):
            raise ValueError("subsea vehicles must start below the waterline")
        if self.config.actuator_bank is None and any(self.wrench_command_B):
            raise ValueError(
                f"{self.config.kind} preset has no milestone-2 actuator model"
            )
        if self.config.glider_control is None and any(self.glider_command):
            raise ValueError("glider_command requires a glider vehicle")
        object.__setattr__(self, "sensors", tuple(self.sensors))
        sensor_ids = [sensor.sensor_id for sensor in self.sensors]
        if len(sensor_ids) != len(set(sensor_ids)):
            raise ValueError(f"sensor IDs for {self.vehicle_id} must be unique")


@dataclass(frozen=True)
class AcousticScenario:
    """Selected modem and idealized channel configuration."""

    modem: ModemProfile
    channel: AcousticChannelConfig
    transmissions: tuple[AcousticTransmissionRequest, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "transmissions", tuple(self.transmissions))
        transmission_ids = [
            request.transmission_id for request in self.transmissions
        ]
        if len(transmission_ids) != len(set(transmission_ids)):
            raise ValueError("acoustic transmission IDs must be unique")
        for request in self.transmissions:
            if request.payload_bytes > self.modem.maximum_payload_bytes:
                raise ValueError(
                    f"transmission {request.transmission_id} exceeds modem "
                    "payload"
                )
            if request.code_channel >= self.modem.code_channel_count:
                raise ValueError(
                    f"transmission {request.transmission_id} uses "
                    "unsupported channel"
                )


@dataclass(frozen=True)
class MarineScenario:
    """Complete deterministic simulation configuration."""

    name: str
    vehicles: tuple[ScenarioVehicle, ...]
    acoustic: AcousticScenario
    sensor_profiles: tuple[SensorProfile, ...] = ()
    schema_version: int = CURRENT_SCENARIO_SCHEMA_VERSION
    duration_s: float = 10.0
    time_step_s: float = 0.005
    gravity_mps2: float = 9.81
    water_density_kg_m3: float = 1025.0
    air_density_kg_m3: float = 1.225
    surface_pressure_Pa: float = 101_325.0
    water_temperature_C: float = 10.0
    air_temperature_C: float = 15.0
    seafloor_z_W_m: float = -50.0
    world_extent_m: float = 100.0

    def __post_init__(self) -> None:
        if (
            isinstance(self.schema_version, bool)
            or self.schema_version != CURRENT_SCENARIO_SCHEMA_VERSION
        ):
            raise ValueError(
                "unsupported scenario schema_version; expected "
                f"{CURRENT_SCENARIO_SCHEMA_VERSION}"
            )
        if not self.name.strip():
            raise ValueError("scenario name cannot be empty")
        if not self.vehicles:
            raise ValueError("scenario must contain at least one vehicle")
        vehicle_ids = [vehicle.vehicle_id for vehicle in self.vehicles]
        if len(set(vehicle_ids)) != len(vehicle_ids):
            raise ValueError("scenario vehicle IDs must be unique")
        object.__setattr__(self, "sensor_profiles", tuple(self.sensor_profiles))
        profile_ids = [profile.profile_id for profile in self.sensor_profiles]
        if len(profile_ids) != len(set(profile_ids)):
            raise ValueError("custom sensor profile IDs must be unique")
        for name in (
            "duration_s",
            "time_step_s",
            "gravity_mps2",
            "water_density_kg_m3",
            "air_density_kg_m3",
            "surface_pressure_Pa",
        ):
            value = getattr(self, name)
            if value <= 0.0 or not math.isfinite(value):
                raise ValueError(f"{name} must be positive and finite")
        if not math.isfinite(self.water_temperature_C) or not math.isfinite(
            self.air_temperature_C
        ):
            raise ValueError("air and water temperature must be finite")
        if not math.isfinite(self.seafloor_z_W_m) or self.seafloor_z_W_m >= 0.0:
            raise ValueError("seafloor_z_W_m must be finite and below zero")
        if self.world_extent_m <= 0.0 or not math.isfinite(self.world_extent_m):
            raise ValueError("world_extent_m must be positive and finite")
        vehicle_id_set = set(vehicle_ids)
        for request in self.acoustic.transmissions:
            if (
                request.source_id not in vehicle_id_set
                or request.destination_id not in vehicle_id_set
            ):
                raise ValueError(
                    f"transmission {request.transmission_id} references "
                    "unknown vehicle"
                )
            if request.start_time_s > self.duration_s:
                raise ValueError(
                    f"transmission {request.transmission_id} starts after "
                    "scenario"
                )


def _parse_transmission(value: Any, index: int) -> AcousticTransmissionRequest:
    context = f"network transmission {index}"
    data = _mapping(context, value)
    _reject_unknown(
        data,
        {
            "id",
            "source",
            "destination",
            "start_time_s",
            "payload_bytes",
            "code_channel",
        },
        context,
    )
    required = {"id", "source", "destination", "start_time_s", "payload_bytes"}
    missing = required - set(data)
    if missing:
        raise ValueError(
            f"missing {context} keys: " + ", ".join(sorted(missing))
        )
    return AcousticTransmissionRequest(
        transmission_id=str(data["id"]),
        source_id=str(data["source"]),
        destination_id=str(data["destination"]),
        start_time_s=float(data["start_time_s"]),
        payload_bytes=_integer(
            f"{context} payload_bytes", data["payload_bytes"]
        ),
        code_channel=_integer(
            f"{context} code_channel", data.get("code_channel", 0)
        ),
    )


_PROFILE_COMMON_KEYS = {
    "id",
    "display_name",
    "kind",
    "provenance",
    "source_url",
    "source_retrieved",
}


def _profile_common(data: Mapping[str, Any], context: str) -> dict[str, Any]:
    required = {"id", "display_name", "kind"}
    missing = required - set(data)
    if missing:
        raise ValueError(
            f"missing {context} keys: " + ", ".join(sorted(missing))
        )
    for name in ("id", "display_name", "kind"):
        if not isinstance(data[name], str):
            raise ValueError(f"{context} {name} must be a string")
    profile_id = data["id"].strip().lower().replace("_", "-")
    if profile_id in PROFILE_FACTORIES:
        raise ValueError(
            f"{context} cannot override built-in profile {profile_id}"
        )
    try:
        provenance = ParameterProvenance(data.get("provenance", "assumed"))
    except ValueError as exc:
        raise ValueError(f"{context} has unsupported provenance") from exc
    source_url = data.get("source_url")
    if source_url is not None and not isinstance(source_url, str):
        raise ValueError(f"{context} source_url must be a string")
    source_retrieved = data.get("source_retrieved", "")
    if not isinstance(source_retrieved, str):
        raise ValueError(f"{context} source_retrieved must be a string")
    return {
        "profile_id": profile_id,
        "display_name": data["display_name"],
        "provenance": provenance,
        "source_url": source_url,
        "source_retrieved": source_retrieved,
    }


def _parse_sensor_profile(value: Any, index: int) -> SensorProfile:
    context = f"sensor profile {index}"
    data = _mapping(context, value)
    if "kind" not in data:
        raise ValueError(f"missing {context} keys: kind")
    try:
        kind = SensorKind(data["kind"])
    except ValueError as exc:
        raise ValueError(f"{context} has unsupported kind") from exc
    common = _profile_common(data, context)
    if kind is SensorKind.PRESSURE:
        specific = {
            "maximum_pressure_Pa",
            "approximate_depth_rating_m",
            "nominal_depth_resolution_m",
            "temperature_accuracy_C",
        }
        _reject_unknown(data, _PROFILE_COMMON_KEYS | specific, context)
        missing = specific - set(data)
        if missing:
            raise ValueError(
                f"missing {context} keys: " + ", ".join(sorted(missing))
            )
        return PressureSensorProfile(
            **common,
            maximum_pressure_Pa=_number(
                f"{context} maximum_pressure_Pa",
                data["maximum_pressure_Pa"],
            ),
            approximate_depth_rating_m=_number(
                f"{context} approximate_depth_rating_m",
                data["approximate_depth_rating_m"],
            ),
            nominal_depth_resolution_m=_number(
                f"{context} nominal_depth_resolution_m",
                data["nominal_depth_resolution_m"],
            ),
            temperature_accuracy_C=_number(
                f"{context} temperature_accuracy_C",
                data["temperature_accuracy_C"],
            ),
        )
    if kind is SensorKind.IMU:
        specific = {
            "gyroscope_range_radps",
            "accelerometer_range_mps2",
            "gyroscope_noise_density_radps_per_sqrt_hz",
            "accelerometer_noise_density_mps2_per_sqrt_hz",
            "maximum_output_rate_hz",
            "roll_pitch_accuracy_rad_rms",
            "heading_accuracy_rad_rms",
        }
        _reject_unknown(data, _PROFILE_COMMON_KEYS | specific, context)
        missing = specific - set(data)
        if missing:
            raise ValueError(
                f"missing {context} keys: " + ", ".join(sorted(missing))
            )
        return ImuSensorProfile(
            **common,
            gyroscope_range_radps=_vector(
                f"{context} gyroscope_range_radps",
                data["gyroscope_range_radps"],
                3,
            ),
            accelerometer_range_mps2=_vector(
                f"{context} accelerometer_range_mps2",
                data["accelerometer_range_mps2"],
                3,
            ),
            gyroscope_noise_density_radps_per_sqrt_hz=_number(
                f"{context} gyroscope_noise_density_radps_per_sqrt_hz",
                data["gyroscope_noise_density_radps_per_sqrt_hz"],
            ),
            accelerometer_noise_density_mps2_per_sqrt_hz=_number(
                f"{context} accelerometer_noise_density_mps2_per_sqrt_hz",
                data["accelerometer_noise_density_mps2_per_sqrt_hz"],
            ),
            maximum_output_rate_hz=_number(
                f"{context} maximum_output_rate_hz",
                data["maximum_output_rate_hz"],
            ),
            roll_pitch_accuracy_rad_rms=_number(
                f"{context} roll_pitch_accuracy_rad_rms",
                data["roll_pitch_accuracy_rad_rms"],
            ),
            heading_accuracy_rad_rms=_number(
                f"{context} heading_accuracy_rad_rms",
                data["heading_accuracy_rad_rms"],
            ),
        )
    if kind in {
        SensorKind.ECHOSOUNDER,
        SensorKind.MULTIBEAM_ECHOSOUNDER,
        SensorKind.FORWARD_LOOKING_SONAR,
    }:
        required = {
            "frequency_hz",
            "minimum_range_m",
            "maximum_range_m",
            "horizontal_field_of_view_rad",
            "vertical_field_of_view_rad",
            "range_resolution_fraction",
            "depth_rating_m",
        }
        specific = required | {"maximum_ping_rate_hz"}
        _reject_unknown(data, _PROFILE_COMMON_KEYS | specific, context)
        missing = required - set(data)
        if missing:
            raise ValueError(
                f"missing {context} keys: " + ", ".join(sorted(missing))
            )
        ping_rate = data.get("maximum_ping_rate_hz")
        return SonarProfile(
            **common,
            kind=kind,
            frequency_hz=_number(
                f"{context} frequency_hz", data["frequency_hz"]
            ),
            minimum_range_m=_number(
                f"{context} minimum_range_m", data["minimum_range_m"]
            ),
            maximum_range_m=_number(
                f"{context} maximum_range_m", data["maximum_range_m"]
            ),
            horizontal_field_of_view_rad=_number(
                f"{context} horizontal_field_of_view_rad",
                data["horizontal_field_of_view_rad"],
            ),
            vertical_field_of_view_rad=_number(
                f"{context} vertical_field_of_view_rad",
                data["vertical_field_of_view_rad"],
            ),
            range_resolution_fraction=_number(
                f"{context} range_resolution_fraction",
                data["range_resolution_fraction"],
            ),
            depth_rating_m=_number(
                f"{context} depth_rating_m", data["depth_rating_m"]
            ),
            maximum_ping_rate_hz=(
                None
                if ping_rate is None
                else _number(f"{context} maximum_ping_rate_hz", ping_rate)
            ),
        )
    specific = {
        "channel_names",
        "units",
        "minimum_values",
        "maximum_values",
        "default_values",
    }
    _reject_unknown(data, _PROFILE_COMMON_KEYS | specific, context)
    missing = specific - set(data)
    if missing:
        raise ValueError(
            f"missing {context} keys: " + ", ".join(sorted(missing))
        )
    return CustomVectorSensorProfile(
        **common,
        channel_names=_string_vector(
            f"{context} channel_names", data["channel_names"]
        ),
        units=_string_vector(f"{context} units", data["units"]),
        minimum_values=_numeric_vector(
            f"{context} minimum_values", data["minimum_values"]
        ),
        maximum_values=_numeric_vector(
            f"{context} maximum_values", data["maximum_values"]
        ),
        default_values=_numeric_vector(
            f"{context} default_values", data["default_values"]
        ),
    )


def _parse_sensor(
    value: Any,
    vehicle_index: int,
    sensor_index: int,
    custom_profiles: Mapping[str, SensorProfile],
) -> MountedSensorConfig:
    context = f"vehicle {vehicle_index} sensor {sensor_index}"
    data = _mapping(context, value)
    _reject_unknown(
        data,
        {
            "id",
            "profile",
            "position_B_m",
            "rpy_BS_deg",
            "bias",
            "value",
        },
        context,
    )
    missing = {"id", "profile"} - set(data)
    if missing:
        raise ValueError(
            f"missing {context} keys: " + ", ".join(sorted(missing))
        )
    profile = sensor_profile(
        str(data["profile"]), custom_profiles=custom_profiles
    )
    if "value" in data and not isinstance(profile, CustomVectorSensorProfile):
        raise ValueError(f"{context} value requires a custom_vector profile")
    if profile.kind is SensorKind.PRESSURE:
        error_size = 2
    elif profile.kind is SensorKind.IMU:
        error_size = 6
    elif profile.kind is SensorKind.CUSTOM_VECTOR:
        error_size = profile.size
    else:
        error_size = 1
    return MountedSensorConfig(
        sensor_id=str(data["id"]),
        profile=profile,
        position_B_m=_vector(
            f"{context} position_B_m",
            data.get("position_B_m", (0, 0, 0)),
            3,
        ),
        rpy_BS_deg=_vector(
            f"{context} rpy_BS_deg",
            data.get("rpy_BS_deg", (0, 0, 0)),
            3,
        ),
        bias=_vector(
            f"{context} bias",
            data.get("bias", (0.0,) * error_size),
            error_size,
        ),
        supplied_value=(
            _vector(
                f"{context} value",
                data["value"],
                profile.size,
            )
            if isinstance(profile, CustomVectorSensorProfile)
            and "value" in data
            else ()
        ),
    )


def _parse_vehicle(
    value: Any,
    index: int,
    custom_profiles: Mapping[str, SensorProfile],
) -> ScenarioVehicle:
    data = _mapping(f"vehicle {index}", value)
    allowed = {
        "id",
        "preset",
        "position_W_m",
        "rpy_deg",
        "water_current_W_mps",
        "wind_velocity_W_mps",
        "wrench_command_B",
        "applied_wrench_B",
        "glider_command",
        "sensors",
    }
    _reject_unknown(data, allowed, f"vehicle {index}")
    missing = {"id", "preset", "position_W_m"} - set(data)
    if missing:
        raise ValueError(
            f"missing vehicle {index} keys: " + ", ".join(sorted(missing))
        )
    sensors_raw = data.get("sensors", [])
    if not isinstance(sensors_raw, list):
        raise ValueError(f"vehicle {index} sensors must be an array of tables")
    return ScenarioVehicle(
        vehicle_id=str(data["id"]),
        config=vehicle_preset(str(data["preset"])),
        initial_position_W_m=_vector(
            f"vehicle {index} position_W_m", data["position_W_m"], 3
        ),
        initial_rpy_deg=_vector(
            f"vehicle {index} rpy_deg", data.get("rpy_deg", (0, 0, 0)), 3
        ),
        water_current_W_mps=_vector(
            f"vehicle {index} water_current_W_mps",
            data.get("water_current_W_mps", (0, 0, 0)),
            3,
        ),
        wind_velocity_W_mps=_vector(
            f"vehicle {index} wind_velocity_W_mps",
            data.get("wind_velocity_W_mps", (0, 0, 0)),
            3,
        ),
        wrench_command_B=_vector(
            f"vehicle {index} wrench_command_B",
            data.get("wrench_command_B", (0, 0, 0, 0, 0, 0)),
            6,
        ),
        applied_wrench_B=_vector(
            f"vehicle {index} applied_wrench_B",
            data.get("applied_wrench_B", (0, 0, 0, 0, 0, 0)),
            6,
        ),
        glider_command=_vector(
            f"vehicle {index} glider_command",
            data.get("glider_command", (0, 0)),
            2,
        ),
        sensors=tuple(
            _parse_sensor(sensor, index, sensor_index, custom_profiles)
            for sensor_index, sensor in enumerate(sensors_raw, start=1)
        ),
    )


def load_scenario(path: str | Path) -> MarineScenario:
    """Load and strictly validate one TOML scenario file."""

    source = Path(path)
    with source.open("rb") as stream:
        raw = tomllib.load(stream)
    allowed = {
        "schema_version",
        "name",
        "duration_s",
        "time_step_s",
        "gravity_mps2",
        "water_density_kg_m3",
        "air_density_kg_m3",
        "surface_pressure_Pa",
        "water_temperature_C",
        "air_temperature_C",
        "seafloor_z_W_m",
        "world_extent_m",
        "sensor_profiles",
        "network",
        "vehicles",
    }
    _reject_unknown(raw, allowed, "scenario")
    vehicles_raw = raw.get("vehicles")
    if not isinstance(vehicles_raw, list):
        raise ValueError("scenario vehicles must be an array of tables")
    profiles_raw = raw.get("sensor_profiles", [])
    if not isinstance(profiles_raw, list):
        raise ValueError("sensor_profiles must be an array of tables")
    parsed_profiles = tuple(
        _parse_sensor_profile(value, index)
        for index, value in enumerate(profiles_raw, start=1)
    )
    custom_profiles = {
        profile.profile_id: profile for profile in parsed_profiles
    }
    if len(custom_profiles) != len(parsed_profiles):
        raise ValueError("custom sensor profile IDs must be unique")

    network = _mapping("network", raw.get("network"))
    _reject_unknown(
        network,
        {"modem", "sound_speed_mps", "transmissions"},
        "network",
    )
    transmissions_raw = network.get("transmissions", [])
    if not isinstance(transmissions_raw, list):
        raise ValueError("network transmissions must be an array of tables")
    selected_modem = modem_profile(
        str(network.get("modem", "divenet-sealink-3km-oem"))
    )
    acoustic = AcousticScenario(
        modem=selected_modem,
        channel=AcousticChannelConfig(
            sound_speed_mps=float(network.get("sound_speed_mps", 1500.0))
        ),
        transmissions=tuple(
            _parse_transmission(value, index)
            for index, value in enumerate(transmissions_raw, start=1)
        ),
    )
    return MarineScenario(
        schema_version=_integer(
            "schema_version",
            raw.get("schema_version", CURRENT_SCENARIO_SCHEMA_VERSION),
        ),
        name=str(raw.get("name", source.stem)),
        vehicles=tuple(
            _parse_vehicle(value, index, custom_profiles)
            for index, value in enumerate(vehicles_raw, start=1)
        ),
        acoustic=acoustic,
        sensor_profiles=parsed_profiles,
        duration_s=float(raw.get("duration_s", 10.0)),
        time_step_s=float(raw.get("time_step_s", 0.005)),
        gravity_mps2=float(raw.get("gravity_mps2", 9.81)),
        water_density_kg_m3=float(raw.get("water_density_kg_m3", 1025.0)),
        air_density_kg_m3=float(raw.get("air_density_kg_m3", 1.225)),
        surface_pressure_Pa=float(raw.get("surface_pressure_Pa", 101_325.0)),
        water_temperature_C=float(raw.get("water_temperature_C", 10.0)),
        air_temperature_C=float(raw.get("air_temperature_C", 15.0)),
        seafloor_z_W_m=float(raw.get("seafloor_z_W_m", -50.0)),
        world_extent_m=float(raw.get("world_extent_m", 100.0)),
    )
