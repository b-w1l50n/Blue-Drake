from __future__ import annotations

import pytest

from blue_drake.manipulation import ParallelJawGripperConfig
from blue_drake.manipulation_systems import ParallelJawGripperController


def test_parallel_jaw_controller_exports_explicit_drake_ports() -> None:
    from pydrake.systems.analysis import Simulator

    controller = ParallelJawGripperController(ParallelJawGripperConfig())
    simulator = Simulator(controller)
    context = simulator.get_mutable_context()
    controller.state_input.FixValue(context, [0.08, -0.08, 0.0, 0.0])
    controller.desired_opening_input.FixValue(context, [0.04])
    force = controller.actuation_output.Eval(context)
    assert force[0] == pytest.approx(-force[1])
    assert force[0] < 0.0
