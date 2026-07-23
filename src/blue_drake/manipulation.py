"""Generic subsea manipulation configuration and control calculations."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

Vector3 = tuple[float, float, float]


def _positive_vector3(name: str, value: Vector3) -> Vector3:
    vector = np.asarray(value, dtype=float)
    if (
        vector.shape != (3,)
        or not np.all(np.isfinite(vector))
        or np.any(vector <= 0.0)
    ):
        raise ValueError(f"{name} must contain three positive finite values")
    return tuple(float(item) for item in vector)


@dataclass(frozen=True)
class ParallelJawGripperConfig:
    """Generic symmetric two-jaw gripper attached to a marine vehicle."""

    palm_mass_kg: float = 1.0
    palm_dimensions_m: Vector3 = (0.12, 0.30, 0.10)
    jaw_mass_kg: float = 0.25
    jaw_dimensions_m: Vector3 = (0.24, 0.04, 0.08)
    minimum_opening_m: float = 0.04
    maximum_opening_m: float = 0.24
    default_opening_m: float = 0.16
    joint_damping_N_per_mps: float = 3.0
    position_gain_N_per_m: float = 300.0
    velocity_gain_N_per_mps: float = 20.0
    maximum_actuation_force_N: float = 40.0

    def __post_init__(self) -> None:
        for name in ("palm_dimensions_m", "jaw_dimensions_m"):
            object.__setattr__(
                self, name, _positive_vector3(name, getattr(self, name))
            )
        for name in (
            "palm_mass_kg",
            "jaw_mass_kg",
            "joint_damping_N_per_mps",
            "position_gain_N_per_m",
            "velocity_gain_N_per_mps",
            "maximum_actuation_force_N",
        ):
            value = getattr(self, name)
            if value <= 0.0 or not math.isfinite(value):
                raise ValueError(f"{name} must be positive and finite")
        if self.minimum_opening_m < self.jaw_dimensions_m[
            1
        ] or not math.isfinite(self.minimum_opening_m):
            raise ValueError(
                "minimum_opening_m must be finite and at least the jaw width"
            )
        if (
            self.maximum_opening_m <= self.minimum_opening_m
            or not math.isfinite(self.maximum_opening_m)
        ):
            raise ValueError(
                "maximum_opening_m must be finite and exceed the minimum"
            )
        if not (
            self.minimum_opening_m
            <= self.default_opening_m
            <= self.maximum_opening_m
        ):
            raise ValueError("default_opening_m must lie within opening limits")


def parallel_jaw_actuation(
    config: ParallelJawGripperConfig,
    *,
    state: np.ndarray,
    desired_opening_m: float,
) -> np.ndarray:
    """Return bounded left/right prismatic-joint forces.

    State order is left position, right position, left velocity, right
    velocity. Positive left and negative right translation open the jaws.
    """

    state = np.asarray(state, dtype=float)
    if state.shape != (4,) or not np.all(np.isfinite(state)):
        raise ValueError("state must contain four finite values")
    if not math.isfinite(desired_opening_m):
        raise ValueError("desired_opening_m must be finite")
    opening = float(
        np.clip(
            desired_opening_m,
            config.minimum_opening_m,
            config.maximum_opening_m,
        )
    )
    target_positions = np.array([0.5 * opening, -0.5 * opening])
    forces = (
        config.position_gain_N_per_m * (target_positions - state[:2])
        - config.velocity_gain_N_per_mps * state[2:]
    )
    return np.clip(
        forces,
        -config.maximum_actuation_force_N,
        config.maximum_actuation_force_N,
    )
