from __future__ import annotations

import pytest

from blue_drake.control_systems import StationKeepingController
from blue_drake.controls import StationKeepingGains


def test_station_keeping_controller_has_explicit_drake_ports() -> None:
    from pydrake.systems.analysis import Simulator

    gains = StationKeepingGains(
        position_stiffness_N_per_m=(10.0, 10.0, 10.0),
        position_damping_N_per_mps=(2.0, 2.0, 2.0),
        rotation_stiffness_Nm_per_rad=(5.0, 5.0, 5.0),
        rotation_damping_Nm_per_radps=(1.0, 1.0, 1.0),
        maximum_force_N=20.0,
        maximum_torque_Nm=10.0,
    )
    controller = StationKeepingController(gains)
    simulator = Simulator(controller)
    context = simulator.get_mutable_context()
    controller.state_input.FixValue(
        context,
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )
    controller.desired_pose_input.FixValue(
        context, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -2.0]
    )
    assert controller.wrench_command_output.Eval(context) == pytest.approx(
        [0.0, 0.0, 0.0, 10.0, 0.0, 0.0]
    )
