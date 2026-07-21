"""Reproducible analytical benchmarks for Blue Drake's foundation models."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np

from blue_drake.acoustics import (
    DIVENET_SEALINK_3KM_OEM,
    AcousticChannelConfig,
    estimate_transmission,
)
from blue_drake.actuators import rov_actuator_preset, wrench_from_thrusts
from blue_drake.hydrodynamics import (
    MarineWrench,
    buoyancy_force_N,
    compute_marine_wrench,
    effective_inertia_wrench,
    glider_wing_force_B,
)
from blue_drake.sensors import bar30_profile, pressure_measurement
from blue_drake.vehicles import glider_preset, rov_preset, usv_preset


@dataclass(frozen=True)
class ValidationCheck:
    """One scalar comparison against an independently evaluated equation."""

    check_id: str
    description: str
    equation: str
    expected: float
    observed: float
    unit: str
    absolute_tolerance: float = 1e-10
    relative_tolerance: float = 1e-10

    @property
    def absolute_error(self) -> float:
        """Return the absolute difference between expected and observed."""

        return abs(self.observed - self.expected)

    @property
    def passed(self) -> bool:
        """Return whether the observation is within declared tolerances."""

        return math.isclose(
            self.observed,
            self.expected,
            abs_tol=self.absolute_tolerance,
            rel_tol=self.relative_tolerance,
        )

    def to_dict(self) -> dict[str, object]:
        """Return a stable JSON-ready representation."""

        return asdict(self) | {
            "absolute_error": self.absolute_error,
            "passed": self.passed,
        }


@dataclass(frozen=True)
class ValidationReport:
    """Aggregate result for the versioned analytical benchmark suite."""

    benchmark_schema_version: int
    checks: tuple[ValidationCheck, ...]

    @property
    def passed(self) -> bool:
        """Return true only when every benchmark passes."""

        return all(check.passed for check in self.checks)

    def to_dict(self) -> dict[str, object]:
        """Return a stable JSON-ready representation."""

        return {
            "benchmark_schema_version": self.benchmark_schema_version,
            "passed": self.passed,
            "check_count": len(self.checks),
            "checks": [check.to_dict() for check in self.checks],
        }


def _submerged_buoyancy_check() -> ValidationCheck:
    density, gravity = 1025.0, 9.81
    config = rov_preset()
    return ValidationCheck(
        check_id="submerged-archimedes",
        description="Submerged support equals displaced-fluid weight.",
        equation="F_b = rho * volume * g",
        expected=density * config.displaced_volume_m3 * gravity,
        observed=buoyancy_force_N(
            config,
            body_origin_z_W_m=-10.0,
            water_density_kg_m3=density,
            gravity_mps2=gravity,
        ),
        unit="N",
    )


def _surge_drag_check() -> ValidationCheck:
    density, gravity, speed = 1025.0, 9.81, 0.8
    config = rov_preset()
    wrench = compute_marine_wrench(
        config,
        rotation_WB=np.eye(3),
        body_origin_W_m=(0.0, 0.0, -2.0),
        angular_velocity_W_radps=(0.0, 0.0, 0.0),
        translational_velocity_W_mps=(speed, 0.0, 0.0),
        water_density_kg_m3=density,
        gravity_mps2=gravity,
    )
    expected = -config.linear_drag_N_per_mps[0] * speed
    expected -= config.quadratic_drag_N_per_mps2[0] * speed**2
    return ValidationCheck(
        check_id="surge-drag-polynomial",
        description="Body-axis surge drag matches its declared polynomial.",
        equation="F_x = -d_1 u - d_2 |u|u",
        expected=expected,
        observed=float(wrench.force_W_N[0]),
        unit="N",
    )


def _surface_heave_check() -> ValidationCheck:
    config, displacement = usv_preset(), 0.025
    gravity = 9.81
    support = buoyancy_force_N(
        config,
        body_origin_z_W_m=displacement,
        water_density_kg_m3=1025.0,
        gravity_mps2=gravity,
    )
    expected = config.dry_mass_kg * gravity
    expected -= config.surface_heave_stiffness_N_per_m * displacement
    return ValidationCheck(
        check_id="surface-heave-stiffness",
        description=(
            "USV hydrostatic support follows the linear waterline model."
        ),
        equation="F_b(z) = m*g - K_heave*z",
        expected=expected,
        observed=support,
        unit="N",
    )


def _added_mass_check() -> ValidationCheck:
    config, applied_force = rov_preset(), 20.0
    corrected = effective_inertia_wrench(
        config,
        rotation_WB=np.eye(3),
        uncorrected_wrench=MarineWrench(
            torque_W_Nm=np.zeros(3),
            force_W_N=np.array([applied_force, 0.0, 0.0]),
        ),
        gravity_mps2=9.81,
    )
    return ValidationCheck(
        check_id="diagonal-added-mass",
        description="Zero-rate surge acceleration uses dry plus added mass.",
        equation="a_x = F_x / (m + m_added,x)",
        expected=applied_force
        / (config.dry_mass_kg + config.added_mass_diagonal_kg[0]),
        observed=float(corrected.force_W_N[0] / config.dry_mass_kg),
        unit="m/s^2",
    )


def _glider_scaling_check() -> ValidationCheck:
    config = glider_preset()
    slow = glider_wing_force_B(
        config,
        relative_velocity_B_mps=(1.0, 0.0, -0.1),
        water_density_kg_m3=1025.0,
    )
    fast = glider_wing_force_B(
        config,
        relative_velocity_B_mps=(2.0, 0.0, -0.2),
        water_density_kg_m3=1025.0,
    )
    return ValidationCheck(
        check_id="glider-speed-squared",
        description=(
            "Wing-force magnitude scales with speed squared at fixed angle."
        ),
        equation="|F(2V)| / |F(V)| = 4",
        expected=4.0,
        observed=float(np.linalg.norm(fast) / np.linalg.norm(slow)),
        unit="ratio",
    )


def _pressure_check() -> ValidationCheck:
    density, gravity, depth = 1025.0, 9.81, 12.0
    surface_pressure = 101_325.0
    measurement = pressure_measurement(
        bar30_profile(),
        sensor_z_W_m=-depth,
        water_density_kg_m3=density,
        gravity_mps2=gravity,
        surface_pressure_Pa=surface_pressure,
        water_temperature_C=10.0,
    )
    return ValidationCheck(
        check_id="hydrostatic-pressure",
        description=(
            "Pressure output matches the constant-density column equation."
        ),
        equation="p = p_surface + rho*g*depth",
        expected=surface_pressure + density * gravity * depth,
        observed=float(measurement[0]),
        unit="Pa",
    )


def _acoustic_timing_check() -> ValidationCheck:
    payload_bytes, range_m, sound_speed = 64, 750.0, 1500.0
    estimate = estimate_transmission(
        DIVENET_SEALINK_3KM_OEM,
        payload_bytes=payload_bytes,
        range_m=range_m,
        channel=AcousticChannelConfig(sound_speed_mps=sound_speed),
    )
    expected = DIVENET_SEALINK_3KM_OEM.preamble_s
    expected += 8.0 * payload_bytes / DIVENET_SEALINK_3KM_OEM.data_rate_bps
    expected += range_m / sound_speed
    return ValidationCheck(
        check_id="acoustic-ideal-latency",
        description=(
            "Ideal packet latency combines serialization and propagation."
        ),
        equation="t = t_preamble + 8N/R + range/c",
        expected=expected,
        observed=estimate.latency_s,
        unit="s",
    )


def _actuator_geometry_check() -> ValidationCheck:
    config = rov_actuator_preset()
    thrusts = np.array([5.0, 5.0, 5.0, 5.0, 0.0, 0.0, 0.0, 0.0])
    wrench = wrench_from_thrusts(config, thrusts)
    expected = 4.0 * 5.0 * math.sqrt(0.5)
    return ValidationCheck(
        check_id="rov-thruster-geometry",
        description="Symmetric horizontal thrusters sum to pure surge.",
        equation="F_x = 4*T*cos(45 deg)",
        expected=expected,
        observed=float(wrench[3]),
        unit="N",
    )


def run_validation_suite() -> ValidationReport:
    """Evaluate Drake-independent analytical implementation benchmarks.

    These checks verify equations and wiring against independently calculated
    values. They are not empirical validation of a vehicle or vendor device.
    """

    return ValidationReport(
        benchmark_schema_version=1,
        checks=(
            _submerged_buoyancy_check(),
            _surge_drag_check(),
            _surface_heave_check(),
            _added_mass_check(),
            _glider_scaling_check(),
            _pressure_check(),
            _acoustic_timing_check(),
            _actuator_geometry_check(),
        ),
    )
