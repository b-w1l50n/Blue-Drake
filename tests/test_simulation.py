from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from blue_drake.actuators import uuv_actuator_preset
from blue_drake.drake_systems import GliderControlSystem, MarineActuatorSystem
from blue_drake.scenario import load_scenario
from blue_drake.sensors import (
    CustomVectorSensorProfile,
    MountedSensorConfig,
    bar30_profile,
    ping_sonar_profile,
    xsens_mti_630r_profile,
)
from blue_drake.simulation import (
    build_marine_fleet_diagram,
    build_marine_scenario_diagram,
    configure_meshcat_marine_world,
    configure_scenario_context,
)
from blue_drake.vehicles import (
    glider_preset,
    rov_preset,
    usv_preset,
    uuv_preset,
)


def _fix_inputs(model, context) -> None:
    for vehicle in model.vehicles:
        model.diagram.GetInputPort(
            f"{vehicle.vehicle_id}_water_current_W_mps"
        ).FixValue(context, np.zeros(3))
        model.diagram.GetInputPort(
            f"{vehicle.vehicle_id}_wind_velocity_W_mps"
        ).FixValue(context, np.zeros(3))
        model.diagram.GetInputPort(
            f"{vehicle.vehicle_id}_applied_wrench_B"
        ).FixValue(context, np.zeros(6))
        if vehicle.actuators is not None:
            model.diagram.GetInputPort(
                f"{vehicle.vehicle_id}_wrench_command_B"
            ).FixValue(context, np.zeros(6))
        if vehicle.glider_controls is not None:
            model.diagram.GetInputPort(
                f"{vehicle.vehicle_id}_glider_command"
            ).FixValue(context, np.zeros(2))
        for sensor in vehicle.sensors:
            model.diagram.GetInputPort(
                f"{vehicle.vehicle_id}_{sensor.config.sensor_id}_error"
            ).FixValue(context, np.zeros(sensor.config.error_size))


class _RecordingMeshcat:
    def __init__(self) -> None:
        self.objects = []
        self.transforms = []

    def SetObject(self, path, shape, rgba) -> None:
        self.objects.append((path, shape, rgba))

    def SetTransform(self, path, transform) -> None:
        self.transforms.append((path, transform))


def test_marine_world_adds_water_surface_and_seafloor_context() -> None:
    meshcat = _RecordingMeshcat()
    configure_meshcat_marine_world(
        meshcat, seafloor_z_W_m=-20.0, world_extent_m=80.0
    )
    assert [item[0] for item in meshcat.objects] == [
        "blue_drake/environment/water_surface",
        "blue_drake/environment/seafloor",
    ]
    transforms = {path: transform for path, transform in meshcat.transforms}
    assert transforms[
        "blue_drake/environment/water_surface"
    ].translation() == pytest.approx([0.0, 0.0, -0.01])
    assert transforms[
        "blue_drake/environment/seafloor"
    ].translation() == pytest.approx([0.0, 0.0, -20.05])


def test_marine_world_rejects_invalid_extent() -> None:
    with pytest.raises(ValueError, match="world_extent_m"):
        configure_meshcat_marine_world(
            _RecordingMeshcat(), seafloor_z_W_m=-20.0, world_extent_m=0.0
        )


def test_scenario_build_and_context_configuration_are_public() -> None:
    from pydrake.systems.analysis import Simulator

    scenario = load_scenario(
        Path(__file__).resolve().parents[1] / "scenarios" / "mixed_marine.toml"
    )
    model = build_marine_scenario_diagram(scenario)
    simulator = Simulator(model.diagram)
    context = simulator.get_mutable_context()
    plant_context = configure_scenario_context(model, scenario, context)

    for configured in scenario.vehicles:
        pose = model.vehicle(configured.vehicle_id).body.EvalPoseInWorld(
            plant_context
        )
        assert pose.translation() == pytest.approx(
            configured.initial_position_W_m
        )
    simulator.Initialize()
    simulator.AdvanceTo(0.01)


def test_visualization_receives_explicit_plant_and_scene_graph(
    monkeypatch,
) -> None:
    received = {}

    def apply_visualization(
        config, *, builder, plant, scene_graph, meshcat
    ) -> None:
        received.update(
            config=config,
            builder=builder,
            plant=plant,
            scene_graph=scene_graph,
            meshcat=meshcat,
        )

    monkeypatch.setattr(
        "blue_drake.simulation.ApplyVisualizationConfig",
        apply_visualization,
    )
    meshcat = _RecordingMeshcat()
    model = build_marine_fleet_diagram(
        {"rov_1": rov_preset()},
        meshcat=meshcat,
    )

    assert received["plant"] is model.plant
    assert received["scene_graph"] is model.scene_graph
    assert received["meshcat"] is meshcat


def test_custom_vector_sensor_exports_supplied_value_port() -> None:
    from pydrake.systems.analysis import Simulator

    profile = CustomVectorSensorProfile(
        profile_id="water-quality",
        display_name="Water Quality",
        channel_names=("salinity", "turbidity"),
        units=("PSU", "NTU"),
        minimum_values=(0.0, 0.0),
        maximum_values=(50.0, 100.0),
        default_values=(35.0, 2.0),
    )
    sensor = MountedSensorConfig(
        "water_quality",
        profile,
        bias=(0.1, 0.0),
        supplied_value=(34.8, 1.2),
    )
    model = build_marine_fleet_diagram(
        {"rov_1": rov_preset()}, sensors={"rov_1": (sensor,)}
    )
    assert model.diagram.HasInputPort("rov_1_water_quality_value")
    simulator = Simulator(model.diagram)
    context = simulator.get_mutable_context()
    model.diagram.GetInputPort("rov_1_water_current_W_mps").FixValue(
        context, np.zeros(3)
    )
    model.diagram.GetInputPort("rov_1_wind_velocity_W_mps").FixValue(
        context, np.zeros(3)
    )
    model.diagram.GetInputPort("rov_1_applied_wrench_B").FixValue(
        context, np.zeros(6)
    )
    model.diagram.GetInputPort("rov_1_wrench_command_B").FixValue(
        context, np.zeros(6)
    )
    model.diagram.GetInputPort("rov_1_water_quality_value").FixValue(
        context, sensor.supplied_value
    )
    model.diagram.GetInputPort("rov_1_water_quality_error").FixValue(
        context, np.zeros(2)
    )
    ideal = model.diagram.GetOutputPort("rov_1_water_quality_ideal").Eval(
        context
    )
    measured = model.diagram.GetOutputPort(
        "rov_1_water_quality_measurement"
    ).Eval(context)
    assert ideal == pytest.approx([34.8, 1.2, 1.0])
    assert measured == pytest.approx([34.9, 1.2, 1.0])


def test_periodic_fleet_logs_capture_state_and_sensor_measurements() -> None:
    from pydrake.math import RigidTransform
    from pydrake.systems.analysis import Simulator

    sensor = MountedSensorConfig("depth", bar30_profile())
    model = build_marine_fleet_diagram(
        {"rov_1": rov_preset()},
        sensors={"rov_1": (sensor,)},
        logging_period_s=0.02,
    )
    assert {log.log_id for log in model.logs} == {
        "rov_1_state",
        "rov_1_depth_measurement",
    }
    simulator = Simulator(model.diagram)
    context = simulator.get_mutable_context()
    plant_context = model.plant.GetMyMutableContextFromRoot(context)
    model.plant.SetFreeBodyPose(
        plant_context,
        model.vehicle("rov_1").body,
        RigidTransform([0.0, 0.0, -2.0]),
    )
    _fix_inputs(model, context)
    simulator.Initialize()
    simulator.AdvanceTo(0.05)
    logs = {item.log_id: item for item in model.logs}
    state = logs["rov_1_state"].sink.FindLog(context)
    depth = logs["rov_1_depth_measurement"].sink.FindLog(context)
    assert len(logs["rov_1_state"].columns) == 13
    assert state.data().shape[0] == 13
    assert depth.data().shape[0] == 4
    assert state.data()[:7, 0] == pytest.approx(
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -2.0]
    )
    assert depth.data()[1, 0] == pytest.approx(2.0)
    assert state.sample_times() == pytest.approx([0.0, 0.02, 0.04])
    assert depth.sample_times() == pytest.approx([0.0, 0.02, 0.04])


def test_mixed_surface_and_subsea_diagram_builds_and_advances() -> None:
    from pydrake.math import RigidTransform
    from pydrake.systems.analysis import Simulator

    model = build_marine_fleet_diagram(
        {"usv_1": usv_preset(), "rov_1": rov_preset()}
    )
    assert model.diagram.HasInputPort("usv_1_water_current_W_mps")
    assert model.diagram.HasInputPort("usv_1_wind_velocity_W_mps")
    assert model.diagram.HasInputPort("rov_1_applied_wrench_B")
    assert model.diagram.HasInputPort("usv_1_wrench_command_B")

    simulator = Simulator(model.diagram)
    context = simulator.get_mutable_context()
    plant_context = model.plant.GetMyMutableContextFromRoot(context)
    model.plant.SetFreeBodyPose(
        plant_context,
        model.vehicle("usv_1").body,
        RigidTransform([-2.0, 0.0, 0.0]),
    )
    model.plant.SetFreeBodyPose(
        plant_context,
        model.vehicle("rov_1").body,
        RigidTransform([2.0, 0.0, -2.0]),
    )
    _fix_inputs(model, context)
    simulator.Initialize()
    simulator.AdvanceTo(0.25)

    for vehicle in model.vehicles:
        pose = vehicle.body.EvalPoseInWorld(plant_context)
        assert np.all(np.isfinite(pose.translation()))
    usv_z = (
        model.vehicle("usv_1")
        .body.EvalPoseInWorld(plant_context)
        .translation()[2]
    )
    assert abs(usv_z) < 0.02


def test_explicit_body_wrench_moves_vehicle_forward() -> None:
    from pydrake.math import RigidTransform
    from pydrake.systems.analysis import Simulator

    model = build_marine_fleet_diagram({"rov_1": rov_preset()})
    simulator = Simulator(model.diagram)
    context = simulator.get_mutable_context()
    plant_context = model.plant.GetMyMutableContextFromRoot(context)
    vehicle = model.vehicle("rov_1")
    model.plant.SetFreeBodyPose(
        plant_context,
        vehicle.body,
        RigidTransform([0.0, 0.0, -2.0]),
    )
    model.diagram.GetInputPort("rov_1_water_current_W_mps").FixValue(
        context, np.zeros(3)
    )
    model.diagram.GetInputPort("rov_1_wind_velocity_W_mps").FixValue(
        context, np.zeros(3)
    )
    model.diagram.GetInputPort("rov_1_applied_wrench_B").FixValue(
        context, [0.0, 0.0, 0.0, 20.0, 0.0, 0.0]
    )
    model.diagram.GetInputPort("rov_1_wrench_command_B").FixValue(
        context, np.zeros(6)
    )
    simulator.Initialize()
    simulator.AdvanceTo(0.5)
    position = vehicle.body.EvalPoseInWorld(plant_context).translation()
    assert position[0] > 0.04


def test_positive_buoyancy_settles_at_free_surface() -> None:
    from pydrake.math import RigidTransform
    from pydrake.systems.analysis import Simulator

    model = build_marine_fleet_diagram({"rov_1": rov_preset()})
    simulator = Simulator(model.diagram)
    context = simulator.get_mutable_context()
    plant_context = model.plant.GetMyMutableContextFromRoot(context)
    vehicle = model.vehicle("rov_1")
    model.plant.SetFreeBodyPose(
        plant_context,
        vehicle.body,
        RigidTransform([0.0, 0.0, -0.25]),
    )
    model.diagram.GetInputPort("rov_1_water_current_W_mps").FixValue(
        context, np.zeros(3)
    )
    model.diagram.GetInputPort("rov_1_wind_velocity_W_mps").FixValue(
        context, np.zeros(3)
    )
    model.diagram.GetInputPort("rov_1_applied_wrench_B").FixValue(
        context, np.zeros(6)
    )
    model.diagram.GetInputPort("rov_1_wrench_command_B").FixValue(
        context, np.zeros(6)
    )
    simulator.Initialize()
    simulator.AdvanceTo(5.0)

    z_W_m = vehicle.body.EvalPoseInWorld(plant_context).translation()[2]
    assert -0.25 < z_W_m < 0.0


def test_vehicle_cannot_fall_through_physical_seafloor() -> None:
    from pydrake.math import RigidTransform
    from pydrake.systems.analysis import Simulator

    config = rov_preset()
    seafloor_z_W_m = -2.0
    model = build_marine_fleet_diagram(
        {"rov_1": config},
        seafloor_z_W_m=seafloor_z_W_m,
    )
    simulator = Simulator(model.diagram)
    context = simulator.get_mutable_context()
    plant_context = model.plant.GetMyMutableContextFromRoot(context)
    vehicle = model.vehicle("rov_1")
    model.plant.SetFreeBodyPose(
        plant_context,
        vehicle.body,
        RigidTransform([0.0, 0.0, -1.0]),
    )
    model.diagram.GetInputPort("rov_1_water_current_W_mps").FixValue(
        context, np.zeros(3)
    )
    model.diagram.GetInputPort("rov_1_wind_velocity_W_mps").FixValue(
        context, np.zeros(3)
    )
    model.diagram.GetInputPort("rov_1_applied_wrench_B").FixValue(
        context, [0.0, 0.0, 0.0, 0.0, 0.0, -100.0]
    )
    model.diagram.GetInputPort("rov_1_wrench_command_B").FixValue(
        context, np.zeros(6)
    )
    simulator.Initialize()
    simulator.AdvanceTo(5.0)

    z_W_m = vehicle.body.EvalPoseInWorld(plant_context).translation()[2]
    minimum_center_z = seafloor_z_W_m + 0.5 * config.dimensions_m[2]
    assert z_W_m >= minimum_center_z - 0.02


def test_bounded_actuator_command_moves_vehicle_and_reports_thrust() -> None:
    from pydrake.math import RigidTransform
    from pydrake.systems.analysis import Simulator

    model = build_marine_fleet_diagram({"uuv_1": uuv_preset()})
    simulator = Simulator(model.diagram)
    context = simulator.get_mutable_context()
    plant_context = model.plant.GetMyMutableContextFromRoot(context)
    vehicle = model.vehicle("uuv_1")
    model.plant.SetFreeBodyPose(
        plant_context, vehicle.body, RigidTransform([0.0, 0.0, -2.0])
    )
    model.diagram.GetInputPort("uuv_1_water_current_W_mps").FixValue(
        context, np.zeros(3)
    )
    model.diagram.GetInputPort("uuv_1_wind_velocity_W_mps").FixValue(
        context, np.zeros(3)
    )
    model.diagram.GetInputPort("uuv_1_applied_wrench_B").FixValue(
        context, np.zeros(6)
    )
    model.diagram.GetInputPort("uuv_1_wrench_command_B").FixValue(
        context, [0.0, 0.0, 0.0, 100.0, 0.0, 0.0]
    )
    simulator.Initialize()
    simulator.AdvanceTo(0.5)

    thrust = model.diagram.GetOutputPort("uuv_1_actual_thrusts_N").Eval(context)
    position = vehicle.body.EvalPoseInWorld(plant_context).translation()
    assert 50.0 < thrust[0] < 55.0
    assert position[0] > 0.05


def test_first_order_actuator_step_matches_analytic_response() -> None:
    from pydrake.systems.analysis import Simulator

    bank = uuv_actuator_preset()
    system = MarineActuatorSystem(bank)
    simulator = Simulator(system)
    context = simulator.get_mutable_context()
    system.wrench_command_input.FixValue(
        context, [0.0, 0.0, 0.0, 40.0, 0.0, 0.0]
    )
    simulator.Initialize()
    simulator.AdvanceTo(bank.time_constants_s[0])
    thrust = system.actual_thrust_output.Eval(context)[0]
    assert thrust == pytest.approx(40.0 * (1.0 - np.exp(-1.0)), rel=1e-5)


def test_mounted_sensor_suite_exports_consistent_measurements() -> None:
    from pydrake.math import RigidTransform
    from pydrake.systems.analysis import Simulator

    sensors = (
        MountedSensorConfig(
            "depth", bar30_profile(), position_B_m=(0.0, 0.0, 0.1)
        ),
        MountedSensorConfig("imu", xsens_mti_630r_profile()),
        MountedSensorConfig(
            "altimeter",
            ping_sonar_profile(),
            rpy_BS_deg=(0.0, 90.0, 0.0),
        ),
    )
    model = build_marine_fleet_diagram(
        {"rov_1": rov_preset()},
        sensors={"rov_1": sensors},
        seafloor_z_W_m=-20.0,
        water_temperature_C=12.0,
    )
    simulator = Simulator(model.diagram)
    context = simulator.get_mutable_context()
    plant_context = model.plant.GetMyMutableContextFromRoot(context)
    model.plant.SetFreeBodyPose(
        plant_context,
        model.vehicle("rov_1").body,
        RigidTransform([0.0, 0.0, -2.0]),
    )
    _fix_inputs(model, context)
    simulator.Initialize()
    simulator.AdvanceTo(0.01)

    depth_ideal = model.diagram.GetOutputPort("rov_1_depth_ideal").Eval(context)
    depth_measured = model.diagram.GetOutputPort(
        "rov_1_depth_measurement"
    ).Eval(context)
    imu = model.diagram.GetOutputPort("rov_1_imu_measurement").Eval(context)
    altitude = model.diagram.GetOutputPort("rov_1_altimeter_measurement").Eval(
        context
    )
    assert depth_measured == pytest.approx(depth_ideal)
    assert depth_measured[1] == pytest.approx(1.9, abs=0.01)
    assert np.all(np.isfinite(imu))
    assert imu[6:] == pytest.approx([1.0, 1.0])
    assert altitude == pytest.approx([18.0, 1.0], abs=0.01)


def test_glider_control_is_bounded_and_has_first_order_response() -> None:
    from pydrake.systems.analysis import Simulator

    config = glider_preset().glider_control
    assert config is not None
    system = GliderControlSystem(config)
    simulator = Simulator(system)
    context = simulator.get_mutable_context()
    system.command_input.FixValue(context, [-100.0, 100.0])
    allocated = system.allocated_output.Eval(context)
    assert allocated == pytest.approx(
        [-config.maximum_buoyancy_delta_N, config.maximum_pitch_moment_Nm]
    )
    simulator.Initialize()
    simulator.AdvanceTo(config.pitch_time_constant_s)
    actual = system.actual_output.Eval(context)
    assert actual[1] == pytest.approx(
        config.maximum_pitch_moment_Nm * (1.0 - np.exp(-1.0)),
        rel=1e-5,
    )
    assert actual[0] < 0.0


def test_glider_fleet_exports_control_ports_and_advances() -> None:
    from pydrake.math import RigidTransform
    from pydrake.systems.analysis import Simulator

    model = build_marine_fleet_diagram({"glider_1": glider_preset()})
    assert model.diagram.HasInputPort("glider_1_glider_command")
    simulator = Simulator(model.diagram)
    context = simulator.get_mutable_context()
    plant_context = model.plant.GetMyMutableContextFromRoot(context)
    model.plant.SetFreeBodyPose(
        plant_context,
        model.vehicle("glider_1").body,
        RigidTransform([0.0, 0.0, -3.0]),
    )
    _fix_inputs(model, context)
    model.diagram.GetInputPort("glider_1_glider_command").FixValue(
        context, [-2.0, 0.5]
    )
    simulator.Initialize()
    simulator.AdvanceTo(0.1)
    actual = model.diagram.GetOutputPort("glider_1_glider_actual").Eval(context)
    assert actual[0] < 0.0
    assert actual[1] > 0.0


def test_fleet_environment_parameters_are_validated_at_build_time() -> None:
    with pytest.raises(ValueError, match="surface pressure"):
        build_marine_fleet_diagram(
            {"rov_1": rov_preset()}, surface_pressure_Pa=float("nan")
        )
    with pytest.raises(ValueError, match="below zero"):
        build_marine_fleet_diagram({"rov_1": rov_preset()}, seafloor_z_W_m=2.0)
