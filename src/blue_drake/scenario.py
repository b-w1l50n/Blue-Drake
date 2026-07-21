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
from blue_drake.sensors import MountedSensorConfig, SensorKind, sensor_profile
from blue_drake.vehicles import MarineVehicleConfig, VehicleKind, vehicle_preset

Vector3 = tuple[float, float, float]
Vector2 = tuple[float, float]
Vector6 = tuple[float, float, float, float, float, float]


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


@dataclass(frozen=True)
class ScenarioVehicle:
    """One configured vehicle and its initial simulation inputs."""

    vehicle_id: str
    config: MarineVehicleConfig
    initial_position_W_m: Vector3
    initial_rpy_deg: Vector3 = (0.0, 0.0, 0.0)
    water_current_W_mps: Vector3 = (0.0, 0.0, 0.0)
    wrench_command_B: Vector6 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    applied_wrench_B: Vector6 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    glider_command: Vector2 = (0.0, 0.0)
    sensors: tuple[MountedSensorConfig, ...] = ()

    def __post_init__(self) -> None:
        if not self.vehicle_id.strip():
            raise ValueError("vehicle_id cannot be empty")
        for name, size in (
            ("initial_position_W_m", 3),
            ("initial_rpy_deg", 3),
            ("water_current_W_mps", 3),
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
    duration_s: float = 10.0
    time_step_s: float = 0.005
    gravity_mps2: float = 9.81
    water_density_kg_m3: float = 1025.0
    surface_pressure_Pa: float = 101_325.0
    water_temperature_C: float = 10.0
    seafloor_z_W_m: float = -50.0
    world_extent_m: float = 100.0

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("scenario name cannot be empty")
        if not self.vehicles:
            raise ValueError("scenario must contain at least one vehicle")
        vehicle_ids = [vehicle.vehicle_id for vehicle in self.vehicles]
        if len(set(vehicle_ids)) != len(vehicle_ids):
            raise ValueError("scenario vehicle IDs must be unique")
        for name in (
            "duration_s",
            "time_step_s",
            "gravity_mps2",
            "water_density_kg_m3",
            "surface_pressure_Pa",
        ):
            value = getattr(self, name)
            if value <= 0.0 or not math.isfinite(value):
                raise ValueError(f"{name} must be positive and finite")
        if not math.isfinite(self.water_temperature_C):
            raise ValueError("water_temperature_C must be finite")
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


def _parse_sensor(
    value: Any, vehicle_index: int, sensor_index: int
) -> MountedSensorConfig:
    context = f"vehicle {vehicle_index} sensor {sensor_index}"
    data = _mapping(context, value)
    _reject_unknown(
        data,
        {"id", "profile", "position_B_m", "rpy_BS_deg", "bias"},
        context,
    )
    missing = {"id", "profile"} - set(data)
    if missing:
        raise ValueError(
            f"missing {context} keys: " + ", ".join(sorted(missing))
        )
    profile = sensor_profile(str(data["profile"]))
    if profile.kind is SensorKind.PRESSURE:
        error_size = 2
    elif profile.kind is SensorKind.IMU:
        error_size = 6
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
    )


def _parse_vehicle(value: Any, index: int) -> ScenarioVehicle:
    data = _mapping(f"vehicle {index}", value)
    allowed = {
        "id",
        "preset",
        "position_W_m",
        "rpy_deg",
        "water_current_W_mps",
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
            _parse_sensor(sensor, index, sensor_index)
            for sensor_index, sensor in enumerate(sensors_raw, start=1)
        ),
    )


def load_scenario(path: str | Path) -> MarineScenario:
    """Load and strictly validate one TOML scenario file."""

    source = Path(path)
    with source.open("rb") as stream:
        raw = tomllib.load(stream)
    allowed = {
        "name",
        "duration_s",
        "time_step_s",
        "gravity_mps2",
        "water_density_kg_m3",
        "surface_pressure_Pa",
        "water_temperature_C",
        "seafloor_z_W_m",
        "world_extent_m",
        "network",
        "vehicles",
    }
    _reject_unknown(raw, allowed, "scenario")
    vehicles_raw = raw.get("vehicles")
    if not isinstance(vehicles_raw, list):
        raise ValueError("scenario vehicles must be an array of tables")

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
        name=str(raw.get("name", source.stem)),
        vehicles=tuple(
            _parse_vehicle(value, index)
            for index, value in enumerate(vehicles_raw, start=1)
        ),
        acoustic=acoustic,
        duration_s=float(raw.get("duration_s", 10.0)),
        time_step_s=float(raw.get("time_step_s", 0.005)),
        gravity_mps2=float(raw.get("gravity_mps2", 9.81)),
        water_density_kg_m3=float(raw.get("water_density_kg_m3", 1025.0)),
        surface_pressure_Pa=float(raw.get("surface_pressure_Pa", 101_325.0)),
        water_temperature_C=float(raw.get("water_temperature_C", 10.0)),
        seafloor_z_W_m=float(raw.get("seafloor_z_W_m", -50.0)),
        world_extent_m=float(raw.get("world_extent_m", 100.0)),
    )
