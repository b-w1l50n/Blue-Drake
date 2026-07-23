from __future__ import annotations

import numpy as np
import pytest

from blue_drake.controls import StationKeepingGains, station_keeping_wrench


def _gains() -> StationKeepingGains:
    return StationKeepingGains(
        position_stiffness_N_per_m=(10.0, 20.0, 30.0),
        position_damping_N_per_mps=(2.0, 3.0, 4.0),
        rotation_stiffness_Nm_per_rad=(5.0, 6.0, 7.0),
        rotation_damping_Nm_per_radps=(1.0, 1.5, 2.0),
        maximum_force_N=100.0,
        maximum_torque_Nm=50.0,
    )


def _state() -> np.ndarray:
    return np.array(
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    )


def test_station_keeping_zero_error_produces_zero_wrench() -> None:
    state = _state()
    assert station_keeping_wrench(
        _gains(), state=state, desired_pose=state[:7]
    ) == pytest.approx(np.zeros(6))


def test_station_keeping_position_feedback_is_expressed_in_body_frame() -> None:
    state = _state()
    yaw_90 = np.sqrt(0.5)
    state[:4] = [yaw_90, 0.0, 0.0, yaw_90]
    desired = state[:7].copy()
    desired[4] = 1.0
    wrench = station_keeping_wrench(_gains(), state=state, desired_pose=desired)
    assert wrench[3:] == pytest.approx([0.0, -10.0, 0.0], abs=1e-12)


def test_station_keeping_shortest_rotation_has_correct_sign() -> None:
    state = _state()
    yaw_90 = np.sqrt(0.5)
    desired = state[:7].copy()
    desired[:4] = [yaw_90, 0.0, 0.0, yaw_90]
    wrench = station_keeping_wrench(_gains(), state=state, desired_pose=desired)
    assert wrench[:3] == pytest.approx([0.0, 0.0, 7.0 * np.pi / 2.0], abs=1e-12)


def test_station_keeping_is_invariant_to_quaternion_sign() -> None:
    state = _state()
    desired = state[:7].copy()
    desired[:4] = [np.sqrt(0.5), 0.0, np.sqrt(0.5), 0.0]
    positive = station_keeping_wrench(
        _gains(), state=state, desired_pose=desired
    )
    negative = station_keeping_wrench(
        _gains(), state=state, desired_pose=np.r_[-desired[:4], desired[4:]]
    )
    assert negative == pytest.approx(positive)


def test_station_keeping_damping_opposes_world_velocity() -> None:
    state = _state()
    state[7:10] = [1.0, -2.0, 3.0]
    state[10:13] = [-4.0, 5.0, -6.0]
    wrench = station_keeping_wrench(
        _gains(), state=state, desired_pose=state[:7]
    )
    assert wrench[:3] == pytest.approx([-1.0, 3.0, -6.0])
    assert wrench[3:] == pytest.approx([8.0, -15.0, 24.0])


def test_station_keeping_limits_force_and_torque_norms() -> None:
    gains = StationKeepingGains(
        position_stiffness_N_per_m=(100.0, 100.0, 100.0),
        position_damping_N_per_mps=(0.0, 0.0, 0.0),
        rotation_stiffness_Nm_per_rad=(100.0, 100.0, 100.0),
        rotation_damping_Nm_per_radps=(0.0, 0.0, 0.0),
        maximum_force_N=4.0,
        maximum_torque_Nm=3.0,
    )
    state = _state()
    desired = state[:7].copy()
    desired[4:] += 10.0
    desired[:4] = [np.sqrt(0.5), np.sqrt(0.5), 0.0, 0.0]
    wrench = station_keeping_wrench(gains, state=state, desired_pose=desired)
    assert np.linalg.norm(wrench[:3]) == pytest.approx(3.0)
    assert np.linalg.norm(wrench[3:]) == pytest.approx(4.0)


def test_station_keeping_rejects_invalid_gains_and_quaternion() -> None:
    with pytest.raises(ValueError, match="cannot contain negative"):
        StationKeepingGains(
            position_stiffness_N_per_m=(-1.0, 1.0, 1.0),
            position_damping_N_per_mps=(1.0, 1.0, 1.0),
            rotation_stiffness_Nm_per_rad=(1.0, 1.0, 1.0),
            rotation_damping_Nm_per_radps=(1.0, 1.0, 1.0),
            maximum_force_N=1.0,
            maximum_torque_Nm=1.0,
        )
    state = _state()
    state[:4] = 0.0
    with pytest.raises(ValueError, match="nonzero norm"):
        station_keeping_wrench(_gains(), state=state, desired_pose=_state()[:7])
