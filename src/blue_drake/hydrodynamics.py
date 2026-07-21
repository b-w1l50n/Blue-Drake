"""Drake-independent hydrostatic and hydrodynamic wrench calculations."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from blue_drake.vehicles import HydrostaticMode, MarineVehicleConfig

Vector = NDArray[np.float64]


def _vector(name: str, value: ArrayLike, size: int) -> Vector:
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {size} finite values")
    return result


def _rotation(value: ArrayLike) -> NDArray[np.float64]:
    result = np.asarray(value, dtype=float)
    if result.shape != (3, 3):
        raise ValueError("rotation_WB must have shape (3, 3)")
    if not np.allclose(result.T @ result, np.eye(3), atol=1e-8):
        raise ValueError("rotation_WB must be orthonormal")
    if not np.isclose(np.linalg.det(result), 1.0, atol=1e-8):
        raise ValueError("rotation_WB must be a proper rotation")
    return result


@dataclass(frozen=True)
class MarineWrench:
    """Torque and force expressed in the world frame."""

    torque_W_Nm: Vector
    force_W_N: Vector

    @property
    def vector(self) -> Vector:
        """Return Drake spatial-wrench order: torque, then force."""

        return np.concatenate((self.torque_W_Nm, self.force_W_N))


def effective_inertia_wrench(
    config: MarineVehicleConfig,
    *,
    rotation_WB: ArrayLike,
    uncorrected_wrench: MarineWrench,
    gravity_mps2: float,
) -> MarineWrench:
    """Scale loads to approximate configured diagonal added inertia.

    ``MultibodyPlant`` owns the dry rigid-body inertia. This calculation scales
    the total body-axis load so its zero-rate acceleration matches a diagonal
    dry-plus-added inertia, then subtracts the gravity load that the plant will
    apply itself. It is exact for uncoupled translation at zero angular rate and
    is explicitly not a full six-degree-of-freedom added-mass formulation.
    """

    rotation_WB = _rotation(rotation_WB)
    if gravity_mps2 <= 0.0 or not np.isfinite(gravity_mps2):
        raise ValueError("gravity_mps2 must be positive and finite")
    rotation_BW = rotation_WB.T
    marine_force_B = rotation_BW @ _vector(
        "uncorrected force", uncorrected_wrench.force_W_N, 3
    )
    marine_torque_B = rotation_BW @ _vector(
        "uncorrected torque", uncorrected_wrench.torque_W_Nm, 3
    )
    gravity_force_W = np.array([0.0, 0.0, -config.dry_mass_kg * gravity_mps2])
    gravity_force_B = rotation_BW @ gravity_force_W

    dry_mass = np.full(3, config.dry_mass_kg)
    effective_mass = dry_mass + np.asarray(config.added_mass_diagonal_kg)
    dry_inertia = np.asarray(config.dry_inertia_diagonal_kg_m2)
    effective_inertia = dry_inertia + np.asarray(
        config.added_inertia_diagonal_kg_m2
    )
    corrected_force_B = (
        dry_mass / effective_mass * (marine_force_B + gravity_force_B)
        - gravity_force_B
    )
    corrected_torque_B = dry_inertia / effective_inertia * marine_torque_B
    return MarineWrench(
        torque_W_Nm=rotation_WB @ corrected_torque_B,
        force_W_N=rotation_WB @ corrected_force_B,
    )


def buoyancy_force_N(
    config: MarineVehicleConfig,
    *,
    body_origin_z_W_m: float,
    water_density_kg_m3: float,
    gravity_mps2: float,
) -> float:
    """Calculate upward hydrostatic support for the configured approximation."""

    if water_density_kg_m3 <= 0.0 or gravity_mps2 <= 0.0:
        raise ValueError("water density and gravity must be positive")
    maximum = water_density_kg_m3 * config.displaced_volume_m3 * gravity_mps2
    if config.hydrostatic_mode is HydrostaticMode.SUBMERGED:
        return maximum

    equilibrium = config.dry_mass_kg * gravity_mps2
    linearized = (
        equilibrium - config.surface_heave_stiffness_N_per_m * body_origin_z_W_m
    )
    return float(np.clip(linearized, 0.0, maximum))


def surface_restoring_torque_B(
    config: MarineVehicleConfig, *, rotation_WB: ArrayLike
) -> Vector:
    """Return linearized roll/pitch waterplane restoring torque in B."""

    if config.hydrostatic_mode is not HydrostaticMode.SURFACE_PIERCING:
        return np.zeros(3)
    rotation_WB = _rotation(rotation_WB)
    up_B = rotation_WB.T @ np.array([0.0, 0.0, 1.0])
    roll_rad = np.arctan2(up_B[1], up_B[2])
    pitch_rad = np.arctan2(-up_B[0], up_B[2])
    return np.array(
        [
            -config.surface_roll_stiffness_Nm_per_rad * roll_rad,
            -config.surface_pitch_stiffness_Nm_per_rad * pitch_rad,
            0.0,
        ]
    )


def glider_wing_force_B(
    config: MarineVehicleConfig,
    *,
    relative_velocity_B_mps: ArrayLike,
    water_density_kg_m3: float,
) -> Vector:
    """Return low-angle lift and induced drag in the body x-z plane."""

    velocity = _vector("relative_velocity_B_mps", relative_velocity_B_mps, 3)
    if water_density_kg_m3 <= 0.0 or not np.isfinite(water_density_kg_m3):
        raise ValueError("water_density_kg_m3 must be positive and finite")
    wing = config.glider_wing
    if wing is None or velocity[0] <= 1e-9:
        return np.zeros(3)
    planar_velocity = np.array([velocity[0], 0.0, velocity[2]])
    speed = np.linalg.norm(planar_velocity)
    if speed <= 1e-9:
        return np.zeros(3)
    # Blue Drake uses z-up; the common marine z-down formula has the opposite
    # sign for vertical velocity.
    angle = np.arctan2(-velocity[2], velocity[0])
    limited_angle = np.clip(
        angle, -wing.maximum_lift_angle_rad, wing.maximum_lift_angle_rad
    )
    lift_coefficient = wing.lift_curve_slope_per_rad * limited_angle
    dynamic_pressure = 0.5 * water_density_kg_m3 * speed**2
    lift_direction = (
        np.array([-planar_velocity[2], 0.0, planar_velocity[0]]) / speed
    )
    drag_direction = -planar_velocity / speed
    lift = (
        dynamic_pressure
        * wing.reference_area_m2
        * lift_coefficient
        * lift_direction
    )
    induced_drag = (
        dynamic_pressure
        * wing.reference_area_m2
        * wing.induced_drag_factor
        * lift_coefficient**2
        * drag_direction
    )
    return lift + induced_drag


def compute_marine_wrench(
    config: MarineVehicleConfig,
    *,
    rotation_WB: ArrayLike,
    body_origin_W_m: ArrayLike,
    angular_velocity_W_radps: ArrayLike,
    translational_velocity_W_mps: ArrayLike,
    water_current_W_mps: ArrayLike = (0.0, 0.0, 0.0),
    applied_wrench_B: ArrayLike = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    glider_control: ArrayLike = (0.0, 0.0),
    water_density_kg_m3: float = 1025.0,
    gravity_mps2: float = 9.81,
) -> MarineWrench:
    """Calculate buoyancy, drag, and caller-supplied actuation.

    Gravity is absent because Drake's ``MultibodyPlant`` applies dry-body
    gravity. Added-inertia correction is applied by the Drake adapter.
    """

    rotation_WB = _rotation(rotation_WB)
    rotation_BW = rotation_WB.T
    position_W = _vector("body_origin_W_m", body_origin_W_m, 3)
    angular_velocity_W = _vector(
        "angular_velocity_W_radps", angular_velocity_W_radps, 3
    )
    velocity_W = _vector(
        "translational_velocity_W_mps", translational_velocity_W_mps, 3
    )
    current_W = _vector("water_current_W_mps", water_current_W_mps, 3)
    applied_B = _vector("applied_wrench_B", applied_wrench_B, 6)
    glider_control = _vector("glider_control", glider_control, 2)
    if config.glider_control is None and np.any(glider_control != 0.0):
        raise ValueError("glider_control requires a configured glider")
    if config.glider_control is not None:
        control_limits = np.array(
            [
                config.glider_control.maximum_buoyancy_delta_N,
                config.glider_control.maximum_pitch_moment_Nm,
            ]
        )
        glider_control = np.clip(
            glider_control, -control_limits, control_limits
        )

    relative_linear_B = rotation_BW @ (velocity_W - current_W)
    angular_velocity_B = rotation_BW @ angular_velocity_W
    linear_drag = np.asarray(config.linear_drag_N_per_mps)
    quadratic_drag = np.asarray(config.quadratic_drag_N_per_mps2)
    drag_force_B = (
        -linear_drag * relative_linear_B
        - quadratic_drag * np.abs(relative_linear_B) * relative_linear_B
    )
    wing_force_B = glider_wing_force_B(
        config,
        relative_velocity_B_mps=relative_linear_B,
        water_density_kg_m3=water_density_kg_m3,
    )
    angular_linear_drag = np.asarray(config.angular_linear_drag_Nm_per_radps)
    angular_quadratic_drag = np.asarray(
        config.angular_quadratic_drag_Nm_per_radps2
    )
    drag_torque_B = (
        -angular_linear_drag * angular_velocity_B
        - angular_quadratic_drag
        * np.abs(angular_velocity_B)
        * angular_velocity_B
    )

    upward_force_N = (
        buoyancy_force_N(
            config,
            body_origin_z_W_m=float(position_W[2]),
            water_density_kg_m3=water_density_kg_m3,
            gravity_mps2=gravity_mps2,
        )
        + glider_control[0]
    )
    upward_force_N = max(0.0, upward_force_N)
    buoyancy_force_W = np.array([0.0, 0.0, upward_force_N])
    buoyancy_force_B = rotation_BW @ buoyancy_force_W
    buoyancy_torque_B = np.cross(
        np.asarray(config.center_of_buoyancy_B_m), buoyancy_force_B
    )

    restoring_torque_B = surface_restoring_torque_B(
        config, rotation_WB=rotation_WB
    )
    control_torque_B = np.array([0.0, glider_control[1], 0.0])
    torque_W = rotation_WB @ (
        drag_torque_B
        + buoyancy_torque_B
        + restoring_torque_B
        + control_torque_B
        + applied_B[:3]
    )
    force_W = rotation_WB @ (drag_force_B + wing_force_B + applied_B[3:])
    force_W += buoyancy_force_W
    return MarineWrench(torque_W_Nm=torque_W, force_W_N=force_W)
