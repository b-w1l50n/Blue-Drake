"""Close a generic articulated ROV gripper through Drake joint control."""

from __future__ import annotations

import numpy as np
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource, TrajectorySource
from pydrake.trajectories import PiecewisePolynomial

from blue_drake.manipulation import ParallelJawGripperConfig
from blue_drake.manipulation_systems import ParallelJawGripperController
from blue_drake.simulation import build_marine_fleet_diagram
from blue_drake.vehicles import rov_preset


def _constant(builder, name: str, value) -> ConstantVectorSource:
    source = builder.AddSystem(ConstantVectorSource(value))
    source.set_name(name)
    return source


def main() -> None:
    gripper_config = ParallelJawGripperConfig()
    fleet_model = build_marine_fleet_diagram(
        {"student_rov": rov_preset()},
        grippers={"student_rov": gripper_config},
    )
    builder = DiagramBuilder()
    fleet = builder.AddSystem(fleet_model.diagram)
    fleet.set_name("marine_fleet")
    controller = builder.AddSystem(ParallelJawGripperController(gripper_config))
    controller.set_name("gripper_controller")
    opening_trajectory = PiecewisePolynomial.FirstOrderHold(
        [0.0, 1.0, 3.0],
        np.array([[0.16, 0.16, 0.04]]),
    )
    desired_opening = builder.AddSystem(TrajectorySource(opening_trajectory))
    desired_opening.set_name("desired_opening")

    builder.Connect(
        fleet.GetOutputPort("student_rov_gripper_state"),
        controller.state_input,
    )
    builder.Connect(
        desired_opening.get_output_port(),
        controller.desired_opening_input,
    )
    builder.Connect(
        controller.actuation_output,
        fleet.GetInputPort("student_rov_gripper_actuation_N"),
    )
    for suffix, value in (
        ("water_current_W_mps", np.zeros(3)),
        ("wind_velocity_W_mps", np.zeros(3)),
        ("applied_wrench_B", np.zeros(6)),
        ("wrench_command_B", np.zeros(6)),
    ):
        builder.Connect(
            _constant(builder, suffix, value).get_output_port(),
            fleet.GetInputPort(f"student_rov_{suffix}"),
        )

    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    simulator.set_target_realtime_rate(0.0)
    simulator.Initialize()
    initial_state = fleet.GetOutputPort("student_rov_gripper_state").Eval(
        fleet.GetMyContextFromRoot(context)
    )
    simulator.AdvanceTo(4.0)
    final_state = fleet.GetOutputPort("student_rov_gripper_state").Eval(
        fleet.GetMyContextFromRoot(context)
    )
    initial_opening_m = initial_state[0] - initial_state[1]
    final_opening_m = final_state[0] - final_state[1]
    print(f"gripper_opening_m={initial_opening_m:.4f} -> {final_opening_m:.4f}")
    print(
        "joint_limits_enforced="
        f"{final_opening_m >= gripper_config.minimum_opening_m}"
    )


if __name__ == "__main__":
    main()
