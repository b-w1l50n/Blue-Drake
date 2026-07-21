"""Run a Blue Drake TOML scenario with optional Meshcat visualization."""

from __future__ import annotations

import argparse
import os
import tempfile

import numpy as np

from blue_drake.acoustics import schedule_transmissions
from blue_drake.scenario import load_scenario
from blue_drake.sensors import CustomVectorSensorProfile, SensorKind

os.environ.setdefault(
    "MPLCONFIGDIR",
    os.path.join(tempfile.gettempdir(), "blue-drake-matplotlib"),
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", help="Path to a Blue Drake TOML scenario")
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--realtime-rate", type=float, default=1.0)
    parser.add_argument("--no-visualizer", action="store_true")
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0.0:
        parser.error("--duration must be positive")
    if args.realtime_rate < 0.0:
        parser.error("--realtime-rate cannot be negative")
    return args


def main() -> None:
    """Load, run, and summarize a deterministic scenario."""

    args = _parse_args()
    scenario = load_scenario(args.scenario)

    from pydrake.geometry import StartMeshcat
    from pydrake.math import RigidTransform, RollPitchYaw
    from pydrake.systems.analysis import Simulator

    from blue_drake.simulation import build_marine_fleet_diagram

    meshcat = None if args.no_visualizer else StartMeshcat()
    model = build_marine_fleet_diagram(
        {vehicle.vehicle_id: vehicle.config for vehicle in scenario.vehicles},
        sensors={
            vehicle.vehicle_id: vehicle.sensors for vehicle in scenario.vehicles
        },
        time_step_s=scenario.time_step_s,
        water_density_kg_m3=scenario.water_density_kg_m3,
        gravity_mps2=scenario.gravity_mps2,
        surface_pressure_Pa=scenario.surface_pressure_Pa,
        water_temperature_C=scenario.water_temperature_C,
        seafloor_z_W_m=scenario.seafloor_z_W_m,
        world_extent_m=scenario.world_extent_m,
        meshcat=meshcat,
    )
    simulator = Simulator(model.diagram)
    context = simulator.get_mutable_context()
    plant_context = model.plant.GetMyMutableContextFromRoot(context)
    for configured in scenario.vehicles:
        vehicle = model.vehicle(configured.vehicle_id)
        model.plant.SetFreeBodyPose(
            plant_context,
            vehicle.body,
            RigidTransform(
                RollPitchYaw(
                    np.deg2rad(configured.initial_rpy_deg)
                ).ToRotationMatrix(),
                configured.initial_position_W_m,
            ),
        )
        model.diagram.GetInputPort(
            f"{configured.vehicle_id}_water_current_W_mps"
        ).FixValue(context, configured.water_current_W_mps)
        model.diagram.GetInputPort(
            f"{configured.vehicle_id}_applied_wrench_B"
        ).FixValue(context, configured.applied_wrench_B)
        if configured.config.actuator_bank is not None:
            model.diagram.GetInputPort(
                f"{configured.vehicle_id}_wrench_command_B"
            ).FixValue(context, configured.wrench_command_B)
        if configured.config.glider_control is not None:
            model.diagram.GetInputPort(
                f"{configured.vehicle_id}_glider_command"
            ).FixValue(context, configured.glider_command)
        for sensor in configured.sensors:
            if sensor.profile.kind is SensorKind.CUSTOM_VECTOR:
                model.diagram.GetInputPort(
                    f"{configured.vehicle_id}_{sensor.sensor_id}_value"
                ).FixValue(context, sensor.supplied_value)
            model.diagram.GetInputPort(
                f"{configured.vehicle_id}_{sensor.sensor_id}_error"
            ).FixValue(context, np.zeros(sensor.error_size))

    duration_s = args.duration or scenario.duration_s
    simulator.set_target_realtime_rate(args.realtime_rate)
    simulator.Initialize()
    print(f"Blue Drake scenario: {scenario.name}")
    print(
        f"Modem: {scenario.acoustic.modem.display_name} "
        f"({scenario.acoustic.modem.validation_status})"
    )
    if scenario.sensor_profiles:
        print("Custom sensor profiles:")
        for profile in scenario.sensor_profiles:
            print(
                f"  {profile.profile_id}: {profile.kind} ({profile.provenance})"
            )
    events = schedule_transmissions(
        scenario.acoustic.modem,
        node_positions_W_m={
            vehicle.vehicle_id: vehicle.initial_position_W_m
            for vehicle in scenario.vehicles
        },
        requests=scenario.acoustic.transmissions,
        channel=scenario.acoustic.channel,
    )
    if events:
        print("Acoustic schedule (stationary initial geometry):")
        for event in events:
            print(
                f"  {event.request.transmission_id}: {event.status} "
                f"range_m={event.range_m:.2f} "
                f"arrival_end_s={event.arrival_end_s:.3f}"
            )
    if meshcat is not None:
        print(f"Meshcat: {meshcat.web_url()}")
    simulator.AdvanceTo(duration_s)
    for vehicle in model.vehicles:
        pose = vehicle.body.EvalPoseInWorld(plant_context)
        position = np.round(pose.translation(), 3)
        print(f"{vehicle.vehicle_id}: xyz_W_m={position}")
        for sensor in vehicle.sensors:
            prefix = f"{vehicle.vehicle_id}_{sensor.config.sensor_id}"
            measurement = model.diagram.GetOutputPort(
                f"{prefix}_measurement"
            ).Eval(context)
            if isinstance(sensor.config.profile, CustomVectorSensorProfile):
                profile = sensor.config.profile
                fields = ", ".join(
                    f"{name}={value:.4g} {unit}"
                    for name, unit, value in zip(
                        profile.channel_names,
                        profile.units,
                        measurement[:-1],
                        strict=True,
                    )
                )
                print(
                    f"  {sensor.config.sensor_id}: {fields}, "
                    f"valid={bool(measurement[-1])}"
                )
            else:
                print(
                    f"  {sensor.config.sensor_id}: {np.round(measurement, 4)}"
                )


if __name__ == "__main__":
    main()
