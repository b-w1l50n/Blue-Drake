"""Immutable physical configuration for supported marine vehicle classes."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum

import numpy as np

from blue_drake.actuators import (
    ActuatorBankConfig,
    rov_actuator_preset,
    usv_actuator_preset,
    uuv_actuator_preset,
)

Vector3 = tuple[float, float, float]


class VehicleKind(StrEnum):
    """User-facing marine vehicle categories supported by Blue Drake."""

    UUV = "uuv"
    ROV = "rov"
    GLIDER = "glider"
    USV = "usv"


class HydrostaticMode(StrEnum):
    """Approximation used to calculate hydrostatic support."""

    SUBMERGED = "submerged"
    SURFACE_PIERCING = "surface_piercing"


@dataclass(frozen=True)
class GliderWingConfig:
    """Low-angle lifting-surface approximation for an underwater glider."""

    reference_area_m2: float
    lift_curve_slope_per_rad: float
    maximum_lift_angle_rad: float
    induced_drag_factor: float

    def __post_init__(self) -> None:
        for name in (
            "reference_area_m2",
            "lift_curve_slope_per_rad",
            "maximum_lift_angle_rad",
            "induced_drag_factor",
        ):
            value = getattr(self, name)
            if value <= 0.0 or not math.isfinite(value):
                raise ValueError(f"{name} must be positive and finite")
        if self.maximum_lift_angle_rad >= math.pi / 2.0:
            raise ValueError("maximum_lift_angle_rad must be below pi/2")


@dataclass(frozen=True)
class GliderControlConfig:
    """Bounds and response rates for buoyancy and movable-mass effects."""

    maximum_buoyancy_delta_N: float
    maximum_pitch_moment_Nm: float
    buoyancy_time_constant_s: float
    pitch_time_constant_s: float

    def __post_init__(self) -> None:
        for name in (
            "maximum_buoyancy_delta_N",
            "maximum_pitch_moment_Nm",
            "buoyancy_time_constant_s",
            "pitch_time_constant_s",
        ):
            value = getattr(self, name)
            if value <= 0.0 or not math.isfinite(value):
                raise ValueError(f"{name} must be positive and finite")


def _finite_vector3(name: str, value: Vector3) -> Vector3:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain three finite values")
    return tuple(float(item) for item in vector)


@dataclass(frozen=True)
class MarineVehicleConfig:
    """Physical parameters for one rigid marine vehicle.

    Added-inertia values are applied through the documented diagonal zero-rate
    approximation. See ``docs/dynamics.md`` and ``docs/fidelity.md``.
    """

    name: str
    kind: VehicleKind
    dry_mass_kg: float
    displaced_volume_m3: float
    dimensions_m: Vector3
    dry_inertia_diagonal_kg_m2: Vector3
    center_of_buoyancy_B_m: Vector3
    linear_drag_N_per_mps: Vector3
    quadratic_drag_N_per_mps2: Vector3
    angular_linear_drag_Nm_per_radps: Vector3
    angular_quadratic_drag_Nm_per_radps2: Vector3
    added_mass_diagonal_kg: Vector3 = (0.0, 0.0, 0.0)
    added_inertia_diagonal_kg_m2: Vector3 = (0.0, 0.0, 0.0)
    air_drag_coefficient_xyz: Vector3 = (1.0, 1.0, 1.0)
    hydrostatic_mode: HydrostaticMode = HydrostaticMode.SUBMERGED
    surface_heave_stiffness_N_per_m: float = 0.0
    surface_roll_stiffness_Nm_per_rad: float = 0.0
    surface_pitch_stiffness_Nm_per_rad: float = 0.0
    color_rgba: tuple[float, float, float, float] = (0.05, 0.55, 0.75, 1.0)
    actuator_bank: ActuatorBankConfig | None = None
    glider_wing: GliderWingConfig | None = None
    glider_control: GliderControlConfig | None = None

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("vehicle name cannot be empty")
        if self.dry_mass_kg <= 0.0 or not math.isfinite(self.dry_mass_kg):
            raise ValueError("dry_mass_kg must be positive and finite")
        if self.displaced_volume_m3 <= 0.0 or not math.isfinite(
            self.displaced_volume_m3
        ):
            raise ValueError("displaced_volume_m3 must be positive and finite")

        positive_vectors = (
            "dimensions_m",
            "dry_inertia_diagonal_kg_m2",
            "air_drag_coefficient_xyz",
        )
        nonnegative_vectors = (
            "linear_drag_N_per_mps",
            "quadratic_drag_N_per_mps2",
            "angular_linear_drag_Nm_per_radps",
            "angular_quadratic_drag_Nm_per_radps2",
            "added_mass_diagonal_kg",
            "added_inertia_diagonal_kg_m2",
        )
        for name in (
            positive_vectors + nonnegative_vectors + ("center_of_buoyancy_B_m",)
        ):
            value = _finite_vector3(name, getattr(self, name))
            object.__setattr__(self, name, value)
        for name in positive_vectors:
            if any(item <= 0.0 for item in getattr(self, name)):
                raise ValueError(f"{name} must contain positive values")
        for name in nonnegative_vectors:
            if any(item < 0.0 for item in getattr(self, name)):
                raise ValueError(f"{name} cannot contain negative values")

        color = tuple(float(item) for item in self.color_rgba)
        if len(color) != 4 or not all(0.0 <= item <= 1.0 for item in color):
            raise ValueError("color_rgba must contain four values in [0, 1]")
        object.__setattr__(self, "color_rgba", color)

        if self.hydrostatic_mode is HydrostaticMode.SURFACE_PIERCING:
            surface_stiffness = (
                self.surface_heave_stiffness_N_per_m,
                self.surface_roll_stiffness_Nm_per_rad,
                self.surface_pitch_stiffness_Nm_per_rad,
            )
            if any(
                value <= 0.0 or not math.isfinite(value)
                for value in surface_stiffness
            ):
                raise ValueError(
                    "surface-piercing vehicles require positive finite "
                    "stiffness"
                )
        elif any(
            value != 0.0
            for value in (
                self.surface_heave_stiffness_N_per_m,
                self.surface_roll_stiffness_Nm_per_rad,
                self.surface_pitch_stiffness_Nm_per_rad,
            )
        ):
            raise ValueError(
                "submerged vehicles must use zero surface stiffness"
            )
        if self.kind is VehicleKind.GLIDER:
            if self.glider_wing is None or self.glider_control is None:
                raise ValueError(
                    "glider vehicles require wing and control configs"
                )
        elif self.glider_wing is not None or self.glider_control is not None:
            raise ValueError(
                "glider dynamics configs require glider vehicle kind"
            )

    @property
    def maximum_buoyancy_mass_kg(self) -> float:
        """Return displaced freshwater mass before density scaling."""

        return 1000.0 * self.displaced_volume_m3


def _box_inertia(mass_kg: float, dimensions_m: Vector3) -> Vector3:
    length, width, height = dimensions_m
    scale = mass_kg / 12.0
    return (
        scale * (width**2 + height**2),
        scale * (length**2 + height**2),
        scale * (length**2 + width**2),
    )


def rov_preset() -> MarineVehicleConfig:
    """Return an inspection-class, fully actuated ROV foundation model."""

    mass = 35.0
    dimensions = (0.72, 0.52, 0.34)
    return MarineVehicleConfig(
        name="inspection_rov",
        kind=VehicleKind.ROV,
        dry_mass_kg=mass,
        displaced_volume_m3=35.5 / 1025.0,
        dimensions_m=dimensions,
        dry_inertia_diagonal_kg_m2=_box_inertia(mass, dimensions),
        center_of_buoyancy_B_m=(0.0, 0.0, 0.045),
        linear_drag_N_per_mps=(5.0, 7.0, 8.0),
        quadratic_drag_N_per_mps2=(18.0, 28.0, 36.0),
        angular_linear_drag_Nm_per_radps=(1.0, 1.4, 1.5),
        angular_quadratic_drag_Nm_per_radps2=(3.0, 5.0, 5.5),
        added_mass_diagonal_kg=(7.0, 11.0, 14.0),
        added_inertia_diagonal_kg_m2=(0.35, 0.75, 0.85),
        color_rgba=(0.02, 0.62, 0.75, 1.0),
        actuator_bank=rov_actuator_preset(),
    )


def uuv_preset() -> MarineVehicleConfig:
    """Return a generic streamlined UUV without autonomy semantics."""

    mass = 52.0
    dimensions = (1.8, 0.3, 0.3)
    return MarineVehicleConfig(
        name="streamlined_uuv",
        kind=VehicleKind.UUV,
        dry_mass_kg=mass,
        displaced_volume_m3=52.2 / 1025.0,
        dimensions_m=dimensions,
        dry_inertia_diagonal_kg_m2=_box_inertia(mass, dimensions),
        center_of_buoyancy_B_m=(0.0, 0.0, 0.025),
        linear_drag_N_per_mps=(3.0, 22.0, 22.0),
        quadratic_drag_N_per_mps2=(8.0, 62.0, 62.0),
        angular_linear_drag_Nm_per_radps=(0.5, 4.5, 4.5),
        angular_quadratic_drag_Nm_per_radps2=(1.2, 16.0, 16.0),
        added_mass_diagonal_kg=(4.0, 38.0, 38.0),
        added_inertia_diagonal_kg_m2=(0.12, 9.0, 9.0),
        color_rgba=(0.06, 0.48, 0.72, 1.0),
        actuator_bank=uuv_actuator_preset(),
    )


def glider_preset() -> MarineVehicleConfig:
    """Return a generic underwater-glider rigid-body foundation model."""

    mass = 58.0
    dimensions = (1.9, 1.15, 0.25)
    return MarineVehicleConfig(
        name="underwater_glider",
        kind=VehicleKind.GLIDER,
        dry_mass_kg=mass,
        displaced_volume_m3=58.0 / 1025.0,
        dimensions_m=dimensions,
        dry_inertia_diagonal_kg_m2=_box_inertia(mass, dimensions),
        center_of_buoyancy_B_m=(0.0, 0.0, 0.018),
        linear_drag_N_per_mps=(2.5, 18.0, 28.0),
        quadratic_drag_N_per_mps2=(7.0, 48.0, 75.0),
        angular_linear_drag_Nm_per_radps=(0.8, 3.0, 3.4),
        angular_quadratic_drag_Nm_per_radps2=(2.0, 9.0, 10.0),
        added_mass_diagonal_kg=(3.0, 31.0, 42.0),
        added_inertia_diagonal_kg_m2=(0.2, 7.5, 8.0),
        color_rgba=(0.66, 0.36, 0.08, 1.0),
        glider_wing=GliderWingConfig(
            reference_area_m2=0.65,
            lift_curve_slope_per_rad=4.0,
            maximum_lift_angle_rad=math.radians(15.0),
            induced_drag_factor=0.12,
        ),
        glider_control=GliderControlConfig(
            maximum_buoyancy_delta_N=12.0,
            maximum_pitch_moment_Nm=4.0,
            buoyancy_time_constant_s=2.0,
            pitch_time_constant_s=1.0,
        ),
    )


def usv_preset() -> MarineVehicleConfig:
    """Return a small displacement USV with linearized surface support."""

    mass = 22.0
    dimensions = (1.2, 0.93, 0.46)
    return MarineVehicleConfig(
        name="small_catamaran_usv",
        kind=VehicleKind.USV,
        dry_mass_kg=mass,
        displaced_volume_m3=0.050,
        dimensions_m=dimensions,
        dry_inertia_diagonal_kg_m2=_box_inertia(mass, dimensions),
        center_of_buoyancy_B_m=(0.0, 0.0, -0.10),
        linear_drag_N_per_mps=(7.0, 35.0, 80.0),
        quadratic_drag_N_per_mps2=(15.0, 95.0, 140.0),
        angular_linear_drag_Nm_per_radps=(8.0, 12.0, 6.0),
        angular_quadratic_drag_Nm_per_radps2=(22.0, 35.0, 18.0),
        added_mass_diagonal_kg=(3.0, 18.0, 25.0),
        added_inertia_diagonal_kg_m2=(2.5, 4.0, 5.0),
        hydrostatic_mode=HydrostaticMode.SURFACE_PIERCING,
        surface_heave_stiffness_N_per_m=2600.0,
        surface_roll_stiffness_Nm_per_rad=180.0,
        surface_pitch_stiffness_Nm_per_rad=250.0,
        color_rgba=(0.12, 0.42, 0.82, 1.0),
        actuator_bank=usv_actuator_preset(),
    )


PRESETS = {
    "uuv": uuv_preset,
    "rov": rov_preset,
    "glider": glider_preset,
    "usv": usv_preset,
}


def vehicle_preset(name: str) -> MarineVehicleConfig:
    """Resolve a supported user-facing preset name."""

    normalized = name.strip().lower().replace("-", "_")
    try:
        factory = PRESETS[normalized]
    except KeyError as exc:
        choices = ", ".join(sorted(PRESETS))
        raise ValueError(
            f"unknown vehicle preset {name!r}; choose {choices}"
        ) from exc
    return factory()
