"""Fixed marine actuator geometry and bounded wrench allocation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum

import numpy as np
from numpy.typing import ArrayLike, NDArray

Vector3 = tuple[float, float, float]
Vector6 = tuple[float, float, float, float, float, float]
Vector = NDArray[np.float64]


class ActuatorKind(StrEnum):
    """Public categories for fixed-direction force actuators."""

    THRUSTER = "thruster"
    PROPELLER = "propeller"


def _finite_vector(name: str, value: ArrayLike, size: int) -> Vector:
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {size} finite values")
    return result


@dataclass(frozen=True)
class FixedActuatorConfig:
    """One force actuator fixed to a vehicle body.

    Positive thrust acts along ``direction_B`` at ``position_B_m``. Limits are
    signed because reverse thrust is commonly weaker than forward thrust.
    The time constant defines a first-order thrust response in the Drake
    adapter; it is not a motor, propeller, or fluid wake model.
    """

    name: str
    kind: ActuatorKind
    position_B_m: Vector3
    direction_B: Vector3
    minimum_thrust_N: float
    maximum_thrust_N: float
    time_constant_s: float = 0.1

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("actuator name cannot be empty")
        if not isinstance(self.kind, ActuatorKind):
            object.__setattr__(self, "kind", ActuatorKind(self.kind))
        position = _finite_vector("position_B_m", self.position_B_m, 3)
        direction = _finite_vector("direction_B", self.direction_B, 3)
        if not np.isclose(np.linalg.norm(direction), 1.0, atol=1e-9):
            raise ValueError("direction_B must be a unit vector")
        object.__setattr__(
            self, "position_B_m", tuple(float(item) for item in position)
        )
        object.__setattr__(
            self, "direction_B", tuple(float(item) for item in direction)
        )
        for name in (
            "minimum_thrust_N",
            "maximum_thrust_N",
            "time_constant_s",
        ):
            if not math.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite")
        if self.minimum_thrust_N >= self.maximum_thrust_N:
            raise ValueError("minimum thrust must be less than maximum thrust")
        if not self.minimum_thrust_N <= 0.0 <= self.maximum_thrust_N:
            raise ValueError("thrust limits must include zero")
        if self.time_constant_s <= 0.0:
            raise ValueError("time_constant_s must be positive")

    @property
    def wrench_column_B(self) -> Vector:
        """Return unit-thrust body wrench in torque-then-force order."""

        position = np.asarray(self.position_B_m)
        direction = np.asarray(self.direction_B)
        return np.concatenate((np.cross(position, direction), direction))


@dataclass(frozen=True)
class ActuatorBankConfig:
    """Named fixed actuators and weights for body-wrench allocation."""

    name: str
    actuators: tuple[FixedActuatorConfig, ...]
    wrench_weights: Vector6 = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("actuator bank name cannot be empty")
        object.__setattr__(self, "actuators", tuple(self.actuators))
        if not self.actuators:
            raise ValueError("actuator bank must contain at least one actuator")
        names = [actuator.name for actuator in self.actuators]
        if len(set(names)) != len(names):
            raise ValueError("actuator names must be unique within a bank")
        weights = _finite_vector("wrench_weights", self.wrench_weights, 6)
        if np.any(weights <= 0.0):
            raise ValueError("wrench_weights must be positive")
        object.__setattr__(
            self, "wrench_weights", tuple(float(item) for item in weights)
        )

    @property
    def allocation_matrix(self) -> Vector:
        """Map actuator thrust in newtons to body wrench."""

        return np.column_stack(
            [actuator.wrench_column_B for actuator in self.actuators]
        )

    @property
    def minimum_thrusts_N(self) -> Vector:
        return np.asarray(
            [actuator.minimum_thrust_N for actuator in self.actuators]
        )

    @property
    def maximum_thrusts_N(self) -> Vector:
        return np.asarray(
            [actuator.maximum_thrust_N for actuator in self.actuators]
        )

    @property
    def time_constants_s(self) -> Vector:
        return np.asarray(
            [actuator.time_constant_s for actuator in self.actuators]
        )


@dataclass(frozen=True)
class WrenchAllocation:
    """Feasible actuator demand and the wrench it can actually produce."""

    thrusts_N: Vector
    achieved_wrench_B: Vector
    residual_wrench_B: Vector


def wrench_from_thrusts(
    config: ActuatorBankConfig, thrusts_N: ArrayLike
) -> Vector:
    """Calculate the body wrench for actuator thrusts without clipping."""

    thrusts = _finite_vector("thrusts_N", thrusts_N, len(config.actuators))
    return config.allocation_matrix @ thrusts


def allocate_wrench(
    config: ActuatorBankConfig, desired_wrench_B: ArrayLike
) -> WrenchAllocation:
    """Solve a deterministic bounded, weighted least-squares allocation.

    The active-set algorithm solves ``min ||W (B u - tau)||`` subject to each
    actuator's signed thrust bounds. It has no temporal objective and is not an
    optimal controller. The returned residual makes infeasible requests
    explicit.
    """

    desired = _finite_vector("desired_wrench_B", desired_wrench_B, 6)
    allocation = config.allocation_matrix
    weights = np.asarray(config.wrench_weights)
    weighted_matrix = weights[:, None] * allocation
    weighted_desired = weights * desired
    lower = config.minimum_thrusts_N
    upper = config.maximum_thrusts_N
    actuator_count = len(config.actuators)
    thrusts = np.zeros(actuator_count)
    # -1 is fixed at the lower bound, +1 at the upper bound, 0 is free.
    status = np.zeros(actuator_count, dtype=np.int8)
    status[np.isclose(thrusts, lower, atol=1e-14)] = -1
    status[np.isclose(thrusts, upper, atol=1e-14)] = 1
    tolerance = 1e-10

    for _ in range(10 * actuator_count + 1):
        free = status == 0
        candidate = thrusts.copy()
        if np.any(free):
            fixed = ~free
            rhs = weighted_desired.copy()
            if np.any(fixed):
                rhs -= weighted_matrix[:, fixed] @ thrusts[fixed]
            candidate[free], *_ = np.linalg.lstsq(
                weighted_matrix[:, free], rhs, rcond=None
            )

            direction = candidate - thrusts
            step = 1.0
            for index in np.flatnonzero(free):
                if candidate[index] < lower[index] - tolerance:
                    step = min(
                        step,
                        (lower[index] - thrusts[index]) / direction[index],
                    )
                elif candidate[index] > upper[index] + tolerance:
                    step = min(
                        step,
                        (upper[index] - thrusts[index]) / direction[index],
                    )
            thrusts += step * direction
            if step < 1.0 - tolerance:
                at_lower = free & np.isclose(thrusts, lower, atol=tolerance)
                at_upper = free & np.isclose(thrusts, upper, atol=tolerance)
                status[at_lower] = -1
                status[at_upper] = 1
                thrusts = np.clip(thrusts, lower, upper)
                continue

        gradient = weighted_matrix.T @ (
            weighted_matrix @ thrusts - weighted_desired
        )
        lower_violation = np.where(status == -1, -gradient, -np.inf)
        upper_violation = np.where(status == 1, gradient, -np.inf)
        violations = np.maximum(lower_violation, upper_violation)
        worst = int(np.argmax(violations))
        if violations[worst] <= tolerance:
            break
        status[worst] = 0
    else:  # pragma: no cover - defensive convergence guard
        raise RuntimeError("bounded wrench allocation did not converge")

    thrusts = np.clip(thrusts, lower, upper)
    achieved = allocation @ thrusts
    return WrenchAllocation(
        thrusts_N=thrusts,
        achieved_wrench_B=achieved,
        residual_wrench_B=desired - achieved,
    )


def _actuator(
    name: str,
    kind: ActuatorKind,
    position: Vector3,
    direction: Vector3,
    reverse_N: float,
    forward_N: float,
    time_constant_s: float,
) -> FixedActuatorConfig:
    return FixedActuatorConfig(
        name=name,
        kind=kind,
        position_B_m=position,
        direction_B=direction,
        minimum_thrust_N=-reverse_N,
        maximum_thrust_N=forward_N,
        time_constant_s=time_constant_s,
    )


def rov_actuator_preset() -> ActuatorBankConfig:
    """Return an eight-thruster, six-axis inspection ROV layout."""

    diagonal = math.sqrt(0.5)
    actuators = (
        _actuator(
            "front_port",
            ActuatorKind.THRUSTER,
            (0.26, 0.20, 0.0),
            (diagonal, -diagonal, 0.0),
            32.0,
            40.0,
            0.08,
        ),
        _actuator(
            "front_starboard",
            ActuatorKind.THRUSTER,
            (0.26, -0.20, 0.0),
            (diagonal, diagonal, 0.0),
            32.0,
            40.0,
            0.08,
        ),
        _actuator(
            "rear_port",
            ActuatorKind.THRUSTER,
            (-0.26, 0.20, 0.0),
            (diagonal, diagonal, 0.0),
            32.0,
            40.0,
            0.08,
        ),
        _actuator(
            "rear_starboard",
            ActuatorKind.THRUSTER,
            (-0.26, -0.20, 0.0),
            (diagonal, -diagonal, 0.0),
            32.0,
            40.0,
            0.08,
        ),
        _actuator(
            "vertical_front_port",
            ActuatorKind.THRUSTER,
            (0.24, 0.18, 0.0),
            (0.0, 0.0, 1.0),
            28.0,
            35.0,
            0.08,
        ),
        _actuator(
            "vertical_front_starboard",
            ActuatorKind.THRUSTER,
            (0.24, -0.18, 0.0),
            (0.0, 0.0, 1.0),
            28.0,
            35.0,
            0.08,
        ),
        _actuator(
            "vertical_rear_port",
            ActuatorKind.THRUSTER,
            (-0.24, 0.18, 0.0),
            (0.0, 0.0, 1.0),
            28.0,
            35.0,
            0.08,
        ),
        _actuator(
            "vertical_rear_starboard",
            ActuatorKind.THRUSTER,
            (-0.24, -0.18, 0.0),
            (0.0, 0.0, 1.0),
            28.0,
            35.0,
            0.08,
        ),
    )
    return ActuatorBankConfig(
        name="inspection_rov_8_thruster", actuators=actuators
    )


def uuv_actuator_preset() -> ActuatorBankConfig:
    """Return a single stern propeller for the streamlined UUV preset."""

    actuator = _actuator(
        "stern_propeller",
        ActuatorKind.PROPELLER,
        (-0.82, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        18.0,
        55.0,
        0.18,
    )
    return ActuatorBankConfig(
        name="streamlined_uuv_propulsion", actuators=(actuator,)
    )


def usv_actuator_preset() -> ActuatorBankConfig:
    """Return differential twin propellers for the catamaran USV preset."""

    actuators = (
        _actuator(
            "port_propeller",
            ActuatorKind.PROPELLER,
            (-0.42, 0.31, -0.12),
            (1.0, 0.0, 0.0),
            12.0,
            32.0,
            0.14,
        ),
        _actuator(
            "starboard_propeller",
            ActuatorKind.PROPELLER,
            (-0.42, -0.31, -0.12),
            (1.0, 0.0, 0.0),
            12.0,
            32.0,
            0.14,
        ),
    )
    return ActuatorBankConfig(
        name="catamaran_differential_propulsion", actuators=actuators
    )
