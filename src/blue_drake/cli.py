"""Run, validate, and inspect Blue Drake marine simulation scenarios."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
from collections.abc import Sequence
from pathlib import Path

import numpy as np

from blue_drake._version import __version__
from blue_drake.acoustics import schedule_transmissions
from blue_drake.inspection import catalog_summary, scenario_summary
from blue_drake.scenario import MarineScenario, load_scenario
from blue_drake.sensors import CustomVectorSensorProfile, SensorKind

os.environ.setdefault(
    "MPLCONFIGDIR",
    os.path.join(tempfile.gettempdir(), "blue-drake-matplotlib"),
)

_COMMANDS = {"run", "validate", "inspect", "catalog", "benchmark"}


def _add_json_option(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--json",
        action="store_true",
        help="emit machine-readable JSON",
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--version",
        action="version",
        version=f"%(prog)s {__version__}",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    run = subparsers.add_parser("run", help="simulate a TOML scenario")
    run.add_argument("scenario", help="path to a Blue Drake TOML scenario")
    run.add_argument("--duration", type=float, default=None)
    run.add_argument("--realtime-rate", type=float, default=1.0)
    run.add_argument("--no-visualizer", action="store_true")
    run.add_argument(
        "--output-dir",
        help="create a new directory containing deterministic CSV logs",
    )
    run.add_argument(
        "--log-period",
        type=float,
        default=None,
        help="logging period in seconds; defaults to the scenario time step",
    )

    validate = subparsers.add_parser(
        "validate", help="validate TOML without importing Drake"
    )
    validate.add_argument("scenario")
    _add_json_option(validate)

    inspect = subparsers.add_parser(
        "inspect", help="summarize a validated scenario"
    )
    inspect.add_argument("scenario")
    _add_json_option(inspect)

    catalog = subparsers.add_parser(
        "catalog", help="list built-in vehicle, sensor, and modem profiles"
    )
    _add_json_option(catalog)
    benchmark = subparsers.add_parser(
        "benchmark", help="run analytical implementation benchmarks"
    )
    _add_json_option(benchmark)
    return parser


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if (
        arguments
        and arguments[0] not in _COMMANDS
        and not arguments[0].startswith("-")
    ):
        arguments.insert(0, "run")
    parser = _parser()
    args = parser.parse_args(arguments)
    if args.command == "run":
        if args.duration is not None and (
            args.duration <= 0.0 or not math.isfinite(args.duration)
        ):
            parser.error("--duration must be positive and finite")
        if args.realtime_rate < 0.0 or not math.isfinite(args.realtime_rate):
            parser.error("--realtime-rate must be finite and nonnegative")
        if args.log_period is not None and (
            args.log_period <= 0.0 or not math.isfinite(args.log_period)
        ):
            parser.error("--log-period must be positive and finite")
        if args.log_period is not None and args.output_dir is None:
            parser.error("--log-period requires --output-dir")
    return args


def _print_scenario_summary(summary: dict) -> None:
    print(f"Scenario: {summary['name']} (schema {summary['schema_version']})")
    print(
        f"Duration: {summary['duration_s']:.3f} s; "
        f"step: {summary['time_step_s']:.6f} s"
    )
    print("Vehicles:")
    for vehicle in summary["vehicles"]:
        print(
            f"  {vehicle['id']}: {vehicle['kind']}, "
            f"{vehicle['actuator_count']} actuator(s), "
            f"{vehicle['sensor_count']} sensor(s)"
        )
        for sensor in vehicle["sensors"]:
            print(f"    {sensor['id']}: {sensor['profile']} ({sensor['kind']})")
    network = summary["network"]
    print(
        f"Network: {network['modem']}; "
        f"{network['transmission_count']} scheduled transmission(s)"
    )
    for event in network["events"]:
        print(
            f"  {event['id']}: {event['status']}, "
            f"range={event['range_m']:.2f} m, "
            f"latency={event['one_way_latency_s']:.3f} s"
        )


def _print_catalog(catalog: dict) -> None:
    print("Vehicle presets:")
    for item in catalog["vehicles"]:
        print(
            f"  {item['preset']}: {item['name']} ({item['dry_mass_kg']:.3g} kg)"
        )
    print("Sensor profiles:")
    for item in catalog["sensors"]:
        print(
            f"  {item['profile_id']}: {item['display_name']} ({item['kind']})"
        )
    print("Modem profiles:")
    for item in catalog["modems"]:
        print(f"  {item['profile_id']}: {item['display_name']}")


def _print_benchmark(report: dict) -> None:
    status = "PASS" if report["passed"] else "FAIL"
    print(f"Analytical benchmarks: {status} ({report['check_count']} checks)")
    for check in report["checks"]:
        result = "PASS" if check["passed"] else "FAIL"
        print(
            f"  {result} {check['check_id']}: "
            f"observed={check['observed']:.12g} "
            f"expected={check['expected']:.12g} {check['unit']}"
        )


def _configure_context(model, scenario: MarineScenario, context) -> object:
    from pydrake.math import RigidTransform, RollPitchYaw

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
    return plant_context


def _print_initial_configuration(scenario: MarineScenario) -> None:
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


def _print_final_state(model, context, plant_context) -> None:
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


def _run(args: argparse.Namespace, scenario: MarineScenario) -> int:
    from pydrake.geometry import StartMeshcat
    from pydrake.systems.analysis import Simulator

    from blue_drake.run_artifacts import write_run_artifacts
    from blue_drake.simulation import build_marine_fleet_diagram

    if args.output_dir is not None and Path(args.output_dir).exists():
        raise FileExistsError(
            f"output directory already exists: {args.output_dir}"
        )
    meshcat = None if args.no_visualizer else StartMeshcat()
    logging_period_s = (
        None
        if args.output_dir is None
        else (args.log_period or scenario.time_step_s)
    )
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
        logging_period_s=logging_period_s,
        meshcat=meshcat,
    )
    simulator = Simulator(model.diagram)
    context = simulator.get_mutable_context()
    plant_context = _configure_context(model, scenario, context)
    duration_s = args.duration or scenario.duration_s
    simulator.set_target_realtime_rate(args.realtime_rate)
    simulator.Initialize()
    _print_initial_configuration(scenario)
    if meshcat is not None:
        print(f"Meshcat: {meshcat.web_url()}")
    simulator.AdvanceTo(duration_s)
    _print_final_state(model, context, plant_context)
    if args.output_dir is not None:
        output_dir = write_run_artifacts(
            args.output_dir,
            model=model,
            context=context,
            scenario=scenario,
            simulated_duration_s=duration_s,
            logging_period_s=logging_period_s,
        )
        print(f"Run artifacts: {output_dir}")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    """Dispatch a Blue Drake command and return a process exit status."""

    args = _parse_args(argv)
    try:
        if args.command == "benchmark":
            from blue_drake.validation import run_validation_suite

            report = run_validation_suite().to_dict()
            if args.json:
                print(json.dumps(report, indent=2, sort_keys=True))
            else:
                _print_benchmark(report)
            return 0 if report["passed"] else 1
        if args.command == "catalog":
            catalog = catalog_summary()
            if args.json:
                print(json.dumps(catalog, indent=2, sort_keys=True))
            else:
                _print_catalog(catalog)
            return 0
        scenario = load_scenario(args.scenario)
        if args.command == "validate":
            result = {
                "valid": True,
                "name": scenario.name,
                "schema_version": scenario.schema_version,
            }
            if args.json:
                print(json.dumps(result, indent=2, sort_keys=True))
            else:
                print(
                    f"Valid: {scenario.name} (schema {scenario.schema_version})"
                )
            return 0
        if args.command == "inspect":
            summary = scenario_summary(scenario)
            if args.json:
                print(json.dumps(summary, indent=2, sort_keys=True))
            else:
                _print_scenario_summary(summary)
            return 0
        return _run(args, scenario)
    except (OSError, ValueError) as exc:
        print(f"blue-drake: error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
