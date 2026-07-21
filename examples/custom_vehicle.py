"""Run a small assumed-parameter ROV and sensor experiment."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
from pydrake.systems.analysis import Simulator

from blue_drake import MountedSensorConfig, rov_preset
from blue_drake.acoustics import AcousticChannelConfig, modem_profile
from blue_drake.scenario import (
    AcousticScenario,
    MarineScenario,
    ScenarioVehicle,
)
from blue_drake.sensors import bar30_profile, xsens_mti_630r_profile
from blue_drake.simulation import (
    build_marine_scenario_diagram,
    configure_scenario_context,
)


def make_scenario() -> MarineScenario:
    """Create an explicitly assumed student ROV configuration."""

    reference = rov_preset()
    mass_scale = 36.0 / reference.dry_mass_kg
    student_rov = replace(
        reference,
        name="student_rov",
        dry_mass_kg=36.0,
        displaced_volume_m3=36.0 / 1025.0,
        dry_inertia_diagonal_kg_m2=tuple(
            value * mass_scale for value in reference.dry_inertia_diagonal_kg_m2
        ),
        parameter_provenance="assumed",
        parameter_source_urls=(),
    )
    sensors = (
        MountedSensorConfig("depth", bar30_profile()),
        MountedSensorConfig("imu", xsens_mti_630r_profile()),
    )
    return MarineScenario(
        name="student-custom-rov",
        vehicles=(
            ScenarioVehicle(
                vehicle_id="student_rov",
                config=student_rov,
                initial_position_W_m=(0.0, 0.0, -2.0),
                sensors=sensors,
            ),
        ),
        acoustic=AcousticScenario(
            modem=modem_profile("divenet-sealink-3km-oem"),
            channel=AcousticChannelConfig(),
        ),
        duration_s=0.2,
        seafloor_z_W_m=-10.0,
    )


def main() -> None:
    scenario = make_scenario()
    model = build_marine_scenario_diagram(scenario)
    simulator = Simulator(model.diagram)
    context = simulator.get_mutable_context()
    plant_context = configure_scenario_context(model, scenario, context)
    simulator.Initialize()
    simulator.AdvanceTo(scenario.duration_s)

    vehicle = model.vehicle("student_rov")
    position = vehicle.body.EvalPoseInWorld(plant_context).translation()
    depth = model.diagram.GetOutputPort("student_rov_depth_measurement").Eval(
        context
    )
    if not np.all(np.isfinite(position)) or not bool(depth[-1]):
        raise RuntimeError("custom vehicle example produced invalid output")
    print(f"student_rov xyz_W_m={np.round(position, 4)}")
    print(f"depth_m={depth[1]:.4f}")


if __name__ == "__main__":
    main()
