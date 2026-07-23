"""Drake-independent geometric marine control calculations."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

Vector3 = tuple[float, float, float]


def _finite_vector3(name: str, value: Vector3) -> Vector3:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain three finite values")
    return tuple(float(item) for item in vector)


@dataclass(frozen=True)
class StationKeepingGains:
    """Diagonal SE(3) proportional-derivative gains and wrench limits."""

    position_stiffness_N_per_m: Vector3
    position_damping_N_per_mps: Vector3
    rotation_stiffness_Nm_per_rad: Vector3
    rotation_damping_Nm_per_radps: Vector3
    maximum_force_N: float
    maximum_torque_Nm: float

    def __post_init__(self) -> None:
        for name in (
            "position_stiffness_N_per_m",
            "position_damping_N_per_mps",
            "rotation_stiffness_Nm_per_rad",
            "rotation_damping_Nm_per_radps",
        ):
            value = _finite_vector3(name, getattr(self, name))
            if any(item < 0.0 for item in value):
                raise ValueError(f"{name} cannot contain negative values")
            object.__setattr__(self, name, value)
        for name in ("maximum_force_N", "maximum_torque_Nm"):
            value = getattr(self, name)
            if value <= 0.0 or not math.isfinite(value):
                raise ValueError(f"{name} must be positive and finite")


def _unit_quaternion(name: str, value) -> np.ndarray:
    quaternion = np.asarray(value, dtype=float)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError(f"{name} must contain four finite values")
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise ValueError(f"{name} must have nonzero norm")
    return quaternion / norm


def _quaternion_product(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ]
    )


def _rotation_matrix_WB(quaternion_WB: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion_WB
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ]
    )


def _rotation_error_W(
    quaternion_WB: np.ndarray,
    desired_quaternion_WD: np.ndarray,
) -> np.ndarray:
    inverse_current = quaternion_WB * np.array([1.0, -1.0, -1.0, -1.0])
    error = _quaternion_product(desired_quaternion_WD, inverse_current)
    if error[0] < 0.0:
        error = -error
    vector_norm = float(np.linalg.norm(error[1:]))
    if vector_norm <= 1e-12:
        return np.zeros(3)
    angle = 2.0 * math.atan2(vector_norm, max(0.0, float(error[0])))
    return angle * error[1:] / vector_norm


def _limit_norm(vector: np.ndarray, maximum: float) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm <= maximum:
        return vector
    return vector * (maximum / norm)


def station_keeping_wrench(
    gains: StationKeepingGains,
    *,
    state: np.ndarray,
    desired_pose: np.ndarray,
) -> np.ndarray:
    """Return a bounded body wrench for a desired world pose.

    ``state`` uses Drake floating-base order: quaternion ``wxyz``, world
    position, world angular velocity, then world translational velocity.
    ``desired_pose`` contains quaternion ``wxyz`` followed by world position.
    The returned body wrench uses torque-then-force order.
    """

    state = np.asarray(state, dtype=float)
    desired_pose = np.asarray(desired_pose, dtype=float)
    if state.shape != (13,) or not np.all(np.isfinite(state)):
        raise ValueError("state must contain 13 finite values")
    if desired_pose.shape != (7,) or not np.all(np.isfinite(desired_pose)):
        raise ValueError("desired_pose must contain seven finite values")

    quaternion_WB = _unit_quaternion("state quaternion", state[:4])
    quaternion_WD = _unit_quaternion(
        "desired pose quaternion", desired_pose[:4]
    )
    rotation_WB = _rotation_matrix_WB(quaternion_WB)
    position_error_W_m = desired_pose[4:] - state[4:7]
    rotation_error_W_rad = _rotation_error_W(quaternion_WB, quaternion_WD)
    torque_W_Nm = (
        np.asarray(gains.rotation_stiffness_Nm_per_rad) * rotation_error_W_rad
        - np.asarray(gains.rotation_damping_Nm_per_radps) * state[7:10]
    )
    force_W_N = (
        np.asarray(gains.position_stiffness_N_per_m) * position_error_W_m
        - np.asarray(gains.position_damping_N_per_mps) * state[10:13]
    )
    torque_B_Nm = rotation_WB.T @ _limit_norm(
        torque_W_Nm, gains.maximum_torque_Nm
    )
    force_B_N = rotation_WB.T @ _limit_norm(force_W_N, gains.maximum_force_N)
    return np.concatenate((torque_B_Nm, force_B_N))
