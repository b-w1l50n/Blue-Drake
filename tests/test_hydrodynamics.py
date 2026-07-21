from __future__ import annotations

import numpy as np
import pytest

from blue_drake.hydrodynamics import (
    MarineWrench,
    aerodynamic_drag_force_B,
    buoyancy_force_N,
    compute_marine_wrench,
    effective_inertia_wrench,
    glider_wing_force_B,
    submerged_box_fraction,
    surface_restoring_torque_B,
    water_load_fraction,
)
from blue_drake.vehicles import glider_preset, rov_preset, usv_preset


def _wrench(config, **overrides):
    values = {
        "rotation_WB": np.eye(3),
        "body_origin_W_m": (0.0, 0.0, -2.0),
        "angular_velocity_W_radps": (0.0, 0.0, 0.0),
        "translational_velocity_W_mps": (0.0, 0.0, 0.0),
    }
    values.update(overrides)
    return compute_marine_wrench(config, **values)


def test_submerged_buoyancy_matches_archimedes_principle() -> None:
    config = rov_preset()
    expected = 1025.0 * config.displaced_volume_m3 * 9.81
    assert buoyancy_force_N(
        config,
        body_origin_z_W_m=-2.0,
        water_density_kg_m3=1025.0,
        gravity_mps2=9.81,
    ) == pytest.approx(expected)


def test_submerged_box_fraction_transitions_across_waterline() -> None:
    config = rov_preset()
    height = config.dimensions_m[2]
    assert submerged_box_fraction(
        config, body_origin_z_W_m=-height
    ) == pytest.approx(1.0)
    assert submerged_box_fraction(
        config, body_origin_z_W_m=0.0
    ) == pytest.approx(0.5)
    assert submerged_box_fraction(
        config, body_origin_z_W_m=height
    ) == pytest.approx(0.0)


def test_emerged_submerged_vehicle_has_no_buoyancy_or_water_drag() -> None:
    config = rov_preset()
    wrench = _wrench(
        config,
        body_origin_W_m=(0.0, 0.0, 2.0),
        translational_velocity_W_mps=(1.0, 0.0, 0.0),
        wind_velocity_W_mps=(1.0, 0.0, 0.0),
    )
    assert wrench.force_W_N == pytest.approx(np.zeros(3))


def test_subsea_actuator_loses_authority_out_of_water() -> None:
    config = rov_preset()
    actuator_wrench = (0.0, 0.0, 4.0, 20.0, 0.0, 0.0)
    emerged = _wrench(
        config,
        body_origin_W_m=(0.0, 0.0, 2.0),
        actuator_wrench_B=actuator_wrench,
    )
    at_surface = _wrench(
        config,
        body_origin_W_m=(0.0, 0.0, 0.0),
        actuator_wrench_B=actuator_wrench,
    )
    fully_submerged = _wrench(
        config,
        actuator_wrench_B=actuator_wrench,
    )

    assert emerged.torque_W_Nm[2] == pytest.approx(0.0)
    assert emerged.force_W_N[0] == pytest.approx(0.0)
    assert at_surface.torque_W_Nm[2] == pytest.approx(2.0)
    assert at_surface.force_W_N[0] == pytest.approx(10.0)
    assert fully_submerged.torque_W_Nm[2] == pytest.approx(4.0)
    assert fully_submerged.force_W_N[0] == pytest.approx(20.0)


def test_external_wrench_remains_effective_out_of_water() -> None:
    config = rov_preset()
    wrench = _wrench(
        config,
        body_origin_W_m=(0.0, 0.0, 2.0),
        applied_wrench_B=(0.0, 0.0, 4.0, 20.0, 0.0, 0.0),
    )
    assert wrench.torque_W_Nm[2] == pytest.approx(4.0)
    assert wrench.force_W_N[0] == pytest.approx(20.0)


def test_surface_propulsor_has_full_authority_at_nominal_waterline() -> None:
    config = usv_preset()
    nominal = _wrench(
        config,
        body_origin_W_m=(0.0, 0.0, 0.0),
        actuator_wrench_B=(0.0, 0.0, 0.0, 20.0, 0.0, 0.0),
    )
    emerged = _wrench(
        config,
        body_origin_W_m=(0.0, 0.0, config.dimensions_m[2]),
        actuator_wrench_B=(0.0, 0.0, 0.0, 20.0, 0.0, 0.0),
    )
    assert nominal.force_W_N[0] == pytest.approx(20.0)
    assert emerged.force_W_N[0] == pytest.approx(0.0)


def test_surface_water_loads_taper_to_zero_as_hull_emerges() -> None:
    config = usv_preset()
    height = config.dimensions_m[2]
    assert water_load_fraction(config, body_origin_z_W_m=0.0) == pytest.approx(
        1.0
    )
    assert water_load_fraction(
        config, body_origin_z_W_m=0.25 * height
    ) == pytest.approx(0.5)
    assert water_load_fraction(
        config, body_origin_z_W_m=0.5 * height
    ) == pytest.approx(0.0)

    water_relative = _wrench(
        config,
        body_origin_W_m=(0.0, 0.0, height),
        translational_velocity_W_mps=(1.0, 0.0, 0.0),
        wind_velocity_W_mps=(1.0, 0.0, 0.0),
    )
    assert water_relative.force_W_N == pytest.approx(np.zeros(3))


def test_emerged_surface_vehicle_has_no_water_restoring_torque() -> None:
    config = usv_preset()
    roll = np.deg2rad(10.0)
    rotation_WB = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(roll), -np.sin(roll)],
            [0.0, np.sin(roll), np.cos(roll)],
        ]
    )
    wrench = _wrench(
        config,
        rotation_WB=rotation_WB,
        body_origin_W_m=(0.0, 0.0, config.dimensions_m[2]),
    )
    assert wrench.torque_W_Nm == pytest.approx(np.zeros(3), abs=1e-12)


def test_emerged_vehicle_receives_air_drag_not_water_drag() -> None:
    config = rov_preset()
    speed = 10.0
    force = aerodynamic_drag_force_B(
        config,
        relative_air_velocity_B_mps=(speed, 0.0, 0.0),
        air_density_kg_m3=1.225,
        exposed_fraction=1.0,
    )
    expected = (
        -0.5
        * 1.225
        * config.air_drag_coefficient_xyz[0]
        * config.dimensions_m[1]
        * config.dimensions_m[2]
        * speed**2
    )
    assert force == pytest.approx((expected, 0.0, 0.0))


def test_air_drag_uses_wind_relative_motion_and_exposed_fraction() -> None:
    config = rov_preset()
    calm = _wrench(
        config,
        body_origin_W_m=(0.0, 0.0, 2.0),
        translational_velocity_W_mps=(4.0, 0.0, 0.0),
    )
    matching_wind = _wrench(
        config,
        body_origin_W_m=(0.0, 0.0, 2.0),
        translational_velocity_W_mps=(4.0, 0.0, 0.0),
        wind_velocity_W_mps=(4.0, 0.0, 0.0),
    )
    half_exposed = _wrench(
        config,
        body_origin_W_m=(0.0, 0.0, 0.0),
        translational_velocity_W_mps=(4.0, 0.0, 0.0),
        water_current_W_mps=(4.0, 0.0, 0.0),
    )
    assert calm.force_W_N[0] < 0.0
    assert matching_wind.force_W_N[0] == pytest.approx(0.0)
    assert half_exposed.force_W_N[0] == pytest.approx(0.5 * calm.force_W_N[0])


def test_partly_emerged_buoyancy_scales_with_immersed_fraction() -> None:
    config = rov_preset()
    full = 1025.0 * config.displaced_volume_m3 * 9.81
    support = buoyancy_force_N(
        config,
        body_origin_z_W_m=0.0,
        water_density_kg_m3=1025.0,
        gravity_mps2=9.81,
    )
    assert support == pytest.approx(0.5 * full)


def test_surface_support_is_at_equilibrium_on_waterline() -> None:
    config = usv_preset()
    support = buoyancy_force_N(
        config,
        body_origin_z_W_m=0.0,
        water_density_kg_m3=1025.0,
        gravity_mps2=9.81,
    )
    assert support == pytest.approx(config.dry_mass_kg * 9.81)


def test_surface_support_decreases_when_body_rises() -> None:
    config = usv_preset()
    low = buoyancy_force_N(
        config,
        body_origin_z_W_m=-0.02,
        water_density_kg_m3=1025.0,
        gravity_mps2=9.81,
    )
    high = buoyancy_force_N(
        config,
        body_origin_z_W_m=0.02,
        water_density_kg_m3=1025.0,
        gravity_mps2=9.81,
    )
    assert low > high


def test_drag_dissipates_water_relative_motion() -> None:
    config = rov_preset()
    velocity = np.array([0.8, -0.4, 0.2])
    wrench = _wrench(
        config,
        translational_velocity_W_mps=velocity,
        water_current_W_mps=(0.0, 0.0, 0.0),
    )
    buoyancy = np.array([0.0, 0.0, 1025.0 * config.displaced_volume_m3 * 9.81])
    drag = wrench.force_W_N - buoyancy
    assert float(drag @ velocity) < 0.0


@pytest.mark.parametrize(
    "config", [rov_preset(), glider_preset(), usv_preset()]
)
def test_unforced_dynamic_loads_are_dissipative(config) -> None:
    position = (
        (0.0, 0.0, 0.0) if config.kind.value == "usv" else (0.0, 0.0, -2.0)
    )
    linear_velocity = np.array([0.7, -0.25, 0.12])
    angular_velocity = np.array([0.15, -0.09, 0.04])
    static = _wrench(config, body_origin_W_m=position)
    moving = _wrench(
        config,
        body_origin_W_m=position,
        translational_velocity_W_mps=linear_velocity,
        angular_velocity_W_radps=angular_velocity,
    )
    dynamic_power_W = float(
        (moving.force_W_N - static.force_W_N) @ linear_velocity
        + (moving.torque_W_Nm - static.torque_W_Nm) @ angular_velocity
    )
    assert dynamic_power_W < 0.0


def test_marine_wrench_is_equivariant_under_world_yaw() -> None:
    config = rov_preset()
    yaw = np.deg2rad(63.0)
    rotation_WQ = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    velocity_W = np.array([0.7, -0.4, 0.15])
    angular_W = np.array([0.2, -0.1, 0.05])
    current_W = np.array([0.1, 0.05, -0.02])
    wind_W = np.array([-0.3, 0.2, 0.0])
    original = _wrench(
        config,
        translational_velocity_W_mps=velocity_W,
        angular_velocity_W_radps=angular_W,
        water_current_W_mps=current_W,
        wind_velocity_W_mps=wind_W,
        applied_wrench_B=(0.4, -0.2, 0.1, 3.0, -1.0, 0.5),
    )
    rotated = _wrench(
        config,
        rotation_WB=rotation_WQ,
        translational_velocity_W_mps=rotation_WQ @ velocity_W,
        angular_velocity_W_radps=rotation_WQ @ angular_W,
        water_current_W_mps=rotation_WQ @ current_W,
        wind_velocity_W_mps=rotation_WQ @ wind_W,
        applied_wrench_B=(0.4, -0.2, 0.1, 3.0, -1.0, 0.5),
    )
    assert rotated.force_W_N == pytest.approx(rotation_WQ @ original.force_W_N)
    assert rotated.torque_W_Nm == pytest.approx(
        rotation_WQ @ original.torque_W_Nm
    )


@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_buoyancy_rejects_nonfinite_environment(bad: float) -> None:
    with pytest.raises(ValueError, match="positive and finite"):
        buoyancy_force_N(
            rov_preset(),
            body_origin_z_W_m=-1.0,
            water_density_kg_m3=bad,
            gravity_mps2=9.81,
        )


def test_matching_current_produces_no_translational_drag() -> None:
    velocity = (0.4, -0.15, 0.05)
    wrench = _wrench(
        rov_preset(),
        translational_velocity_W_mps=velocity,
        water_current_W_mps=velocity,
    )
    assert wrench.force_W_N[:2] == pytest.approx((0.0, 0.0))


def test_offset_buoyancy_produces_uprighting_roll_moment() -> None:
    roll = np.deg2rad(15.0)
    rotation_WB = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(roll), -np.sin(roll)],
            [0.0, np.sin(roll), np.cos(roll)],
        ]
    )
    wrench = _wrench(rov_preset(), rotation_WB=rotation_WB)
    assert wrench.torque_W_Nm[0] < 0.0


def test_body_wrench_rotates_into_world_frame() -> None:
    yaw = np.pi / 2.0
    rotation_WB = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    wrench = _wrench(
        rov_preset(),
        rotation_WB=rotation_WB,
        applied_wrench_B=(0.0, 0.0, 0.0, 10.0, 0.0, 0.0),
    )
    assert wrench.force_W_N[:2] == pytest.approx((0.0, 10.0), abs=1e-10)


def test_effective_inertia_scales_surge_to_dry_plus_added_mass() -> None:
    config = rov_preset()
    force_N = 20.0
    corrected = effective_inertia_wrench(
        config,
        rotation_WB=np.eye(3),
        uncorrected_wrench=MarineWrench(
            torque_W_Nm=np.zeros(3),
            force_W_N=np.array([force_N, 0.0, 0.0]),
        ),
        gravity_mps2=9.81,
    )
    plant_acceleration = corrected.force_W_N[0] / config.dry_mass_kg
    expected = force_N / (config.dry_mass_kg + config.added_mass_diagonal_kg[0])
    assert plant_acceleration == pytest.approx(expected)


def test_effective_inertia_preserves_static_neutral_buoyancy() -> None:
    config = rov_preset()
    gravity = 9.81
    upward = np.array([0.0, 0.0, config.dry_mass_kg * gravity])
    corrected = effective_inertia_wrench(
        config,
        rotation_WB=np.eye(3),
        uncorrected_wrench=MarineWrench(
            torque_W_Nm=np.zeros(3),
            force_W_N=upward,
        ),
        gravity_mps2=gravity,
    )
    assert corrected.force_W_N == pytest.approx(upward)


def test_zero_immersion_removes_added_inertia_correction() -> None:
    config = rov_preset()
    original = MarineWrench(
        torque_W_Nm=np.array([1.0, 2.0, 3.0]),
        force_W_N=np.array([4.0, 5.0, 6.0]),
    )
    corrected = effective_inertia_wrench(
        config,
        rotation_WB=np.eye(3),
        uncorrected_wrench=original,
        gravity_mps2=9.81,
        immersion_fraction=0.0,
    )
    assert corrected.vector == pytest.approx(original.vector)


def test_zero_added_inertia_leaves_wrench_unchanged() -> None:
    values = vars(rov_preset()) | {
        "added_mass_diagonal_kg": (0.0, 0.0, 0.0),
        "added_inertia_diagonal_kg_m2": (0.0, 0.0, 0.0),
    }
    config = type(rov_preset())(**values)
    original = MarineWrench(
        torque_W_Nm=np.array([1.0, 2.0, 3.0]),
        force_W_N=np.array([4.0, 5.0, 6.0]),
    )
    corrected = effective_inertia_wrench(
        config,
        rotation_WB=np.eye(3),
        uncorrected_wrench=original,
        gravity_mps2=9.81,
    )
    assert corrected.vector == pytest.approx(original.vector)


def test_glider_wing_lift_is_restoring_and_dissipative() -> None:
    velocity = np.array([1.0, 0.0, -0.1])
    force = glider_wing_force_B(
        glider_preset(),
        relative_velocity_B_mps=velocity,
        water_density_kg_m3=1025.0,
    )
    assert force[2] > 0.0
    assert float(force @ velocity) < 0.0


def test_glider_wing_force_scales_with_speed_squared() -> None:
    slow = glider_wing_force_B(
        glider_preset(),
        relative_velocity_B_mps=(1.0, 0.0, -0.1),
        water_density_kg_m3=1025.0,
    )
    fast = glider_wing_force_B(
        glider_preset(),
        relative_velocity_B_mps=(2.0, 0.0, -0.2),
        water_density_kg_m3=1025.0,
    )
    assert fast == pytest.approx(4.0 * slow)


def test_surface_attitude_stiffness_opposes_roll_and_pitch() -> None:
    roll = np.deg2rad(10.0)
    pitch = np.deg2rad(5.0)
    cosine_roll, sine_roll = np.cos(roll), np.sin(roll)
    cosine_pitch, sine_pitch = np.cos(pitch), np.sin(pitch)
    rotation_WB = np.array(
        [
            [cosine_pitch, sine_pitch * sine_roll, sine_pitch * cosine_roll],
            [0.0, cosine_roll, -sine_roll],
            [-sine_pitch, cosine_pitch * sine_roll, cosine_pitch * cosine_roll],
        ]
    )
    torque = surface_restoring_torque_B(usv_preset(), rotation_WB=rotation_WB)
    assert torque[0] < 0.0
    assert torque[1] < 0.0
    assert torque[2] == 0.0


def test_glider_buoyancy_delta_is_world_up_and_bounded() -> None:
    yaw = np.deg2rad(90.0)
    rotation_WB = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    baseline = _wrench(glider_preset(), rotation_WB=rotation_WB)
    controlled = _wrench(
        glider_preset(),
        rotation_WB=rotation_WB,
        glider_control=(1000.0, 0.0),
    )
    control = glider_preset().glider_control
    assert control is not None
    assert controlled.force_W_N - baseline.force_W_N == pytest.approx(
        [0.0, 0.0, control.maximum_buoyancy_delta_N]
    )
