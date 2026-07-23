"""Compose a Blue Drake ROV and station-keeping controller in Drake."""

from __future__ import annotations

import numpy as np
from pydrake.math import RigidTransform
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource

from blue_drake.control_systems import StationKeepingController
from blue_drake.controls import StationKeepingGains
from blue_drake.simulation import build_marine_fleet_diagram
from blue_drake.vehicles import rov_preset


def _constant(builder, name: str, value) -> ConstantVectorSource:
    source = builder.AddSystem(ConstantVectorSource(value))
    source.set_name(name)
    return source


def main() -> None:
    fleet_model = build_marine_fleet_diagram({"student_rov": rov_preset()})
    builder = DiagramBuilder()
    fleet = builder.AddSystem(fleet_model.diagram)
    fleet.set_name("marine_fleet")
    controller = builder.AddSystem(
        StationKeepingController(
            StationKeepingGains(
                position_stiffness_N_per_m=(35.0, 35.0, 50.0),
                position_damping_N_per_mps=(30.0, 30.0, 35.0),
                rotation_stiffness_Nm_per_rad=(20.0, 20.0, 15.0),
                rotation_damping_Nm_per_radps=(8.0, 8.0, 6.0),
                maximum_force_N=55.0,
                maximum_torque_Nm=15.0,
            )
        )
    )
    controller.set_name("station_keeping")

    desired_pose = np.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -2.0])
    builder.Connect(
        fleet.GetOutputPort("student_rov_state"),
        controller.state_input,
    )
    builder.Connect(
        _constant(builder, "desired_pose", desired_pose).get_output_port(),
        controller.desired_pose_input,
    )
    builder.Connect(
        controller.wrench_command_output,
        fleet.GetInputPort("student_rov_wrench_command_B"),
    )
    for suffix, value in (
        ("water_current_W_mps", np.zeros(3)),
        ("wind_velocity_W_mps", np.zeros(3)),
        ("applied_wrench_B", np.zeros(6)),
    ):
        builder.Connect(
            _constant(builder, suffix, value).get_output_port(),
            fleet.GetInputPort(f"student_rov_{suffix}"),
        )

    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    plant_context = fleet_model.plant.GetMyMutableContextFromRoot(context)
    start_position_W_m = np.array([0.0, 0.0, -2.0])
    fleet_model.plant.SetFreeBodyPose(
        plant_context,
        fleet_model.vehicle("student_rov").body,
        RigidTransform(start_position_W_m),
    )
    simulator.set_target_realtime_rate(0.0)
    simulator.Initialize()
    simulator.AdvanceTo(8.0)

    final_position_W_m = (
        fleet_model.vehicle("student_rov")
        .body.EvalPoseInWorld(plant_context)
        .translation()
    )
    initial_error_m = np.linalg.norm(desired_pose[4:] - start_position_W_m)
    final_error_m = np.linalg.norm(desired_pose[4:] - final_position_W_m)
    print(f"initial_position_W_m={np.round(start_position_W_m, 4)}")
    print(f"final_position_W_m={np.round(final_position_W_m, 4)}")
    print(f"position_error_m={initial_error_m:.4f} -> {final_error_m:.4f}")


if __name__ == "__main__":
    main()
