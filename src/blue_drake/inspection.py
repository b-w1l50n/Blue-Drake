"""JSON-ready inspection data for Blue Drake scenarios and catalogs."""

from __future__ import annotations

from dataclasses import asdict
from typing import Any

from blue_drake.acoustics import MODEM_PROFILES, schedule_transmissions
from blue_drake.scenario import MarineScenario
from blue_drake.sensors import PROFILE_FACTORIES
from blue_drake.vehicles import PRESETS


def scenario_summary(scenario: MarineScenario) -> dict[str, Any]:
    """Return stable, JSON-ready high-level scenario information."""

    acoustic_events = schedule_transmissions(
        scenario.acoustic.modem,
        node_positions_W_m={
            vehicle.vehicle_id: vehicle.initial_position_W_m
            for vehicle in scenario.vehicles
        },
        requests=scenario.acoustic.transmissions,
        channel=scenario.acoustic.channel,
    )
    return {
        "schema_version": scenario.schema_version,
        "name": scenario.name,
        "duration_s": scenario.duration_s,
        "time_step_s": scenario.time_step_s,
        "environment": {
            "gravity_mps2": scenario.gravity_mps2,
            "water_density_kg_m3": scenario.water_density_kg_m3,
            "surface_pressure_Pa": scenario.surface_pressure_Pa,
            "water_temperature_C": scenario.water_temperature_C,
            "seafloor_z_W_m": scenario.seafloor_z_W_m,
            "world_extent_m": scenario.world_extent_m,
        },
        "vehicles": [
            {
                "id": vehicle.vehicle_id,
                "kind": vehicle.config.kind.value,
                "model_name": vehicle.config.name,
                "initial_position_W_m": list(vehicle.initial_position_W_m),
                "initial_rpy_deg": list(vehicle.initial_rpy_deg),
                "water_current_W_mps": list(vehicle.water_current_W_mps),
                "wrench_command_B": list(vehicle.wrench_command_B),
                "applied_wrench_B": list(vehicle.applied_wrench_B),
                "glider_command": list(vehicle.glider_command),
                "sensor_count": len(vehicle.sensors),
                "sensors": [
                    {
                        "id": sensor.sensor_id,
                        "profile": sensor.profile.profile_id,
                        "kind": sensor.profile.kind.value,
                        "position_B_m": list(sensor.position_B_m),
                        "rpy_BS_deg": list(sensor.rpy_BS_deg),
                        "bias": list(sensor.bias),
                        "supplied_value": list(sensor.supplied_value),
                    }
                    for sensor in vehicle.sensors
                ],
                "actuator_count": (
                    0
                    if vehicle.config.actuator_bank is None
                    else len(vehicle.config.actuator_bank.actuators)
                ),
            }
            for vehicle in scenario.vehicles
        ],
        "custom_sensor_profiles": [
            asdict(profile) for profile in scenario.sensor_profiles
        ],
        "network": {
            "modem": scenario.acoustic.modem.profile_id,
            "sound_speed_mps": scenario.acoustic.channel.sound_speed_mps,
            "transmission_count": len(scenario.acoustic.transmissions),
            "transmissions": [
                asdict(request) for request in scenario.acoustic.transmissions
            ],
            "events": [
                {
                    "id": event.request.transmission_id,
                    "status": event.status.value,
                    "range_m": event.range_m,
                    "transmit_end_s": event.transmit_end_s,
                    "arrival_start_s": event.arrival_start_s,
                    "arrival_end_s": event.arrival_end_s,
                    "one_way_latency_s": event.one_way_latency_s,
                }
                for event in acoustic_events
            ],
        },
    }


def catalog_summary() -> dict[str, Any]:
    """Return built-in vehicle, sensor, and modem profiles as plain data."""

    vehicles = []
    for preset_id, factory in sorted(PRESETS.items()):
        config = factory()
        vehicles.append(
            {
                "preset": preset_id,
                "name": config.name,
                "kind": config.kind.value,
                "dry_mass_kg": config.dry_mass_kg,
                "dimensions_m": list(config.dimensions_m),
                "actuator_count": (
                    0
                    if config.actuator_bank is None
                    else len(config.actuator_bank.actuators)
                ),
            }
        )
    return {
        "vehicles": vehicles,
        "sensors": [
            asdict(factory())
            for _, factory in sorted(PROFILE_FACTORIES.items())
        ],
        "modems": [
            asdict(profile) for _, profile in sorted(MODEM_PROFILES.items())
        ],
    }
