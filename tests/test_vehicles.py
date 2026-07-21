from __future__ import annotations

import pytest

from blue_drake.vehicles import (
    HydrostaticMode,
    MarineVehicleConfig,
    VehicleKind,
    glider_preset,
    rov_preset,
    usv_preset,
    uuv_preset,
    vehicle_preset,
)


def test_public_presets_cover_exactly_requested_vehicle_categories() -> None:
    presets = (uuv_preset(), rov_preset(), glider_preset(), usv_preset())
    assert {preset.kind for preset in presets} == set(VehicleKind)
    assert all("auv" not in preset.name.lower() for preset in presets)


def test_surface_and_subsea_presets_use_distinct_hydrostatics() -> None:
    assert usv_preset().hydrostatic_mode is HydrostaticMode.SURFACE_PIERCING
    assert rov_preset().hydrostatic_mode is HydrostaticMode.SUBMERGED
    assert glider_preset().hydrostatic_mode is HydrostaticMode.SUBMERGED
    assert all(
        coefficient > 0.0
        for coefficient in usv_preset().air_drag_coefficient_xyz
    )


def test_vehicle_preset_reports_supported_choices() -> None:
    assert vehicle_preset("USV").kind is VehicleKind.USV
    with pytest.raises(ValueError, match="unknown vehicle preset"):
        vehicle_preset("auv")


def test_surface_vehicle_requires_positive_heave_stiffness() -> None:
    values = vars(usv_preset()) | {"surface_heave_stiffness_N_per_m": 0.0}
    with pytest.raises(ValueError, match="positive finite stiffness"):
        MarineVehicleConfig(**values)


def test_glider_preset_declares_wing_and_buoyancy_control() -> None:
    config = glider_preset()
    assert config.glider_wing is not None
    assert config.glider_control is not None
    assert config.actuator_bank is None


def test_non_glider_rejects_glider_dynamics_configuration() -> None:
    values = vars(rov_preset()) | {
        "glider_wing": glider_preset().glider_wing,
        "glider_control": glider_preset().glider_control,
    }
    with pytest.raises(ValueError, match="require glider vehicle kind"):
        MarineVehicleConfig(**values)


def test_dry_inertia_must_be_physically_realizable() -> None:
    values = vars(rov_preset()) | {
        "dry_inertia_diagonal_kg_m2": (1.0, 1.0, 3.0)
    }
    with pytest.raises(ValueError, match="triangle inequalities"):
        MarineVehicleConfig(**values)


def test_center_of_buoyancy_must_be_inside_body_envelope() -> None:
    values = vars(rov_preset()) | {"center_of_buoyancy_B_m": (0.0, 0.0, 1.0)}
    with pytest.raises(ValueError, match="within the body envelope"):
        MarineVehicleConfig(**values)
