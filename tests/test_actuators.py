from __future__ import annotations

import numpy as np
import pytest

from blue_drake.actuators import (
    ActuatorBankConfig,
    ActuatorKind,
    FixedActuatorConfig,
    allocate_wrench,
    rov_actuator_preset,
    usv_actuator_preset,
    uuv_actuator_preset,
    wrench_from_thrusts,
)


def test_force_at_offset_produces_expected_moment() -> None:
    actuator = FixedActuatorConfig(
        name="offset",
        kind=ActuatorKind.THRUSTER,
        position_B_m=(0.0, 2.0, 0.0),
        direction_B=(1.0, 0.0, 0.0),
        minimum_thrust_N=-5.0,
        maximum_thrust_N=10.0,
    )
    bank = ActuatorBankConfig(name="one", actuators=(actuator,))
    assert wrench_from_thrusts(bank, [3.0]) == pytest.approx(
        [0.0, 0.0, -6.0, 3.0, 0.0, 0.0]
    )


def test_direction_must_be_unit_length() -> None:
    with pytest.raises(ValueError, match="unit vector"):
        FixedActuatorConfig(
            name="bad",
            kind=ActuatorKind.THRUSTER,
            position_B_m=(0.0, 0.0, 0.0),
            direction_B=(2.0, 0.0, 0.0),
            minimum_thrust_N=-1.0,
            maximum_thrust_N=1.0,
        )


def test_rov_layout_has_full_six_axis_rank() -> None:
    bank = rov_actuator_preset()
    assert len(bank.actuators) == 8
    assert np.linalg.matrix_rank(bank.allocation_matrix) == 6


def test_rov_allocator_reproduces_feasible_wrench() -> None:
    bank = rov_actuator_preset()
    desired = np.array([2.0, -3.0, 4.0, 12.0, -8.0, 10.0])
    result = allocate_wrench(bank, desired)
    assert result.achieved_wrench_B == pytest.approx(desired, abs=1e-9)
    assert result.residual_wrench_B == pytest.approx(np.zeros(6), abs=1e-9)


def test_allocator_saturates_and_reports_residual() -> None:
    bank = uuv_actuator_preset()
    result = allocate_wrench(bank, [0.0, 0.0, 0.0, 100.0, 0.0, 0.0])
    assert result.thrusts_N == pytest.approx([55.0])
    assert result.achieved_wrench_B == pytest.approx(
        [0.0, 0.0, 0.0, 55.0, 0.0, 0.0]
    )
    assert result.residual_wrench_B[3] == pytest.approx(45.0)


def test_underactuated_uuv_does_not_invent_lateral_force() -> None:
    result = allocate_wrench(
        uuv_actuator_preset(), [0.0, 0.0, 0.0, 0.0, 20.0, 0.0]
    )
    assert result.thrusts_N == pytest.approx([0.0])
    assert result.achieved_wrench_B == pytest.approx(np.zeros(6))
    assert result.residual_wrench_B[4] == pytest.approx(20.0)


def test_usv_differential_thrust_spans_surge_and_yaw() -> None:
    bank = usv_actuator_preset()
    assert np.linalg.matrix_rank(bank.allocation_matrix) == 2
    surge = allocate_wrench(bank, [0.0, 0.0, 0.0, 20.0, 0.0, 0.0])
    yaw = allocate_wrench(bank, [0.0, 0.0, 6.2, 0.0, 0.0, 0.0])
    assert surge.thrusts_N[0] == pytest.approx(surge.thrusts_N[1])
    assert surge.achieved_wrench_B[3] > 19.0
    assert surge.residual_wrench_B[1] > 0.0
    assert yaw.thrusts_N == pytest.approx([-10.0, 10.0])


@pytest.mark.parametrize(
    "bank",
    [rov_actuator_preset(), uuv_actuator_preset(), usv_actuator_preset()],
)
def test_allocator_satisfies_bound_constrained_optimality(bank) -> None:
    generator = np.random.default_rng(147)
    allocation = bank.allocation_matrix
    weights = np.asarray(bank.wrench_weights)
    weighted_matrix = weights[:, None] * allocation
    for _ in range(100):
        desired = generator.uniform(-200.0, 200.0, 6)
        result = allocate_wrench(bank, desired)
        thrusts = result.thrusts_N
        gradient = weighted_matrix.T @ (
            weighted_matrix @ thrusts - weights * desired
        )
        at_lower = np.isclose(thrusts, bank.minimum_thrusts_N, atol=1e-8)
        at_upper = np.isclose(thrusts, bank.maximum_thrusts_N, atol=1e-8)
        free = ~(at_lower | at_upper)
        assert np.all(gradient[at_lower] >= -1e-8)
        assert np.all(gradient[at_upper] <= 1e-8)
        assert gradient[free] == pytest.approx(0.0, abs=1e-8)
