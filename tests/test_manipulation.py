from __future__ import annotations

import numpy as np
import pytest

from blue_drake.manipulation import (
    ParallelJawGripperConfig,
    parallel_jaw_actuation,
)


def test_parallel_jaw_controller_closes_symmetrically() -> None:
    config = ParallelJawGripperConfig()
    force = parallel_jaw_actuation(
        config,
        state=np.array([0.08, -0.08, 0.0, 0.0]),
        desired_opening_m=0.04,
    )
    assert force[0] < 0.0
    assert force[1] > 0.0
    assert force[0] == pytest.approx(-force[1])


def test_parallel_jaw_command_and_force_are_bounded() -> None:
    config = ParallelJawGripperConfig(maximum_actuation_force_N=5.0)
    force = parallel_jaw_actuation(
        config,
        state=np.zeros(4),
        desired_opening_m=100.0,
    )
    assert force == pytest.approx([5.0, -5.0])


def test_parallel_jaw_config_rejects_overlapping_minimum() -> None:
    with pytest.raises(ValueError, match="at least the jaw width"):
        ParallelJawGripperConfig(minimum_opening_m=0.01)
