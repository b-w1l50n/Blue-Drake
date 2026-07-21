"""Construction helpers for mixed-domain Blue Drake fleet diagrams."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np

from blue_drake.drake_systems import (
    GliderControlSystem,
    MarineActuatorSystem,
    MarineHydrodynamicForceSystem,
    SpatialForceConcatenator,
)
from blue_drake.identifiers import validate_identifier
from blue_drake.sensor_systems import (
    CustomVectorSensorSystem,
    RawImuSensorSystem,
    add_sensor_system,
)
from blue_drake.sensors import MountedSensorConfig, SensorKind
from blue_drake.vehicles import MarineVehicleConfig

try:
    from pydrake.geometry import Box, Rgba
    from pydrake.math import RigidTransform
    from pydrake.multibody.plant import (
        AddMultibodyPlantSceneGraph,
        CoulombFriction,
    )
    from pydrake.multibody.tree import SpatialInertia, UnitInertia
    from pydrake.systems.framework import DiagramBuilder
    from pydrake.systems.primitives import Adder, LogVectorOutput
    from pydrake.visualization import (
        ApplyVisualizationConfig,
        VisualizationConfig,
    )
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "Drake is required for blue_drake.simulation; install the package"
    ) from exc


@dataclass(frozen=True)
class FleetVehicle:
    """Drake handles and configuration for one vehicle."""

    vehicle_id: str
    model_instance: object
    body: object
    hydrodynamics: MarineHydrodynamicForceSystem
    actuators: MarineActuatorSystem | None
    glider_controls: GliderControlSystem | None
    sensors: tuple[FleetSensor, ...]
    config: MarineVehicleConfig


@dataclass(frozen=True)
class FleetSensor:
    """Drake system and immutable configuration for one mounted sensor."""

    config: MountedSensorConfig
    system: object


@dataclass(frozen=True)
class FleetLog:
    """One deterministic in-memory vector log and its column contract."""

    log_id: str
    columns: tuple[str, ...]
    sink: object


@dataclass(frozen=True)
class MarineFleetModel:
    """Built Drake diagram and handles for a mixed marine fleet."""

    diagram: object
    plant: object
    scene_graph: object
    vehicles: tuple[FleetVehicle, ...]
    logs: tuple[FleetLog, ...] = ()

    def vehicle(self, vehicle_id: str) -> FleetVehicle:
        """Return one vehicle by its stable scenario ID."""

        for vehicle in self.vehicles:
            if vehicle.vehicle_id == vehicle_id:
                return vehicle
        raise KeyError(f"unknown vehicle_id: {vehicle_id}")


def _spatial_inertia(config: MarineVehicleConfig) -> SpatialInertia:
    mass = config.dry_mass_kg
    inertia = config.dry_inertia_diagonal_kg_m2
    unit_inertia = UnitInertia(
        Ixx=inertia[0] / mass,
        Iyy=inertia[1] / mass,
        Izz=inertia[2] / mass,
    )
    return SpatialInertia(
        mass=mass,
        p_PScm_E=np.zeros(3),
        G_SP_E=unit_inertia,
    )


def _register_vehicle_geometry(
    plant, body, config: MarineVehicleConfig
) -> None:
    shape = Box(*config.dimensions_m)
    identity = RigidTransform()
    plant.RegisterVisualGeometry(
        body,
        identity,
        shape,
        f"{config.name}_illustration",
        np.asarray(config.color_rgba),
    )
    plant.RegisterCollisionGeometry(
        body,
        identity,
        shape,
        f"{config.name}_collision",
        CoulombFriction(0.65, 0.5),
    )


def _register_seafloor_collision(
    plant,
    *,
    seafloor_z_W_m: float,
    world_extent_m: float,
) -> None:
    """Register the rendered flat seafloor as fixed collision geometry."""

    thickness_m = 0.1
    plant.RegisterCollisionGeometry(
        plant.world_body(),
        RigidTransform([0.0, 0.0, seafloor_z_W_m - 0.5 * thickness_m]),
        Box(world_extent_m, world_extent_m, thickness_m),
        "seafloor_collision",
        CoulombFriction(0.8, 0.6),
    )


def _state_columns() -> tuple[str, ...]:
    return (
        "quaternion_w_WB",
        "quaternion_x_WB",
        "quaternion_y_WB",
        "quaternion_z_WB",
        "position_x_W_m",
        "position_y_W_m",
        "position_z_W_m",
        "angular_velocity_x_W_radps",
        "angular_velocity_y_W_radps",
        "angular_velocity_z_W_radps",
        "translational_velocity_x_W_mps",
        "translational_velocity_y_W_mps",
        "translational_velocity_z_W_mps",
    )


def _sensor_columns(config: MountedSensorConfig) -> tuple[str, ...]:
    if config.profile.kind is SensorKind.PRESSURE:
        return ("pressure_Pa", "depth_m", "temperature_C", "valid")
    if config.profile.kind is SensorKind.IMU:
        return (
            "gyro_x_S_radps",
            "gyro_y_S_radps",
            "gyro_z_S_radps",
            "specific_force_x_S_mps2",
            "specific_force_y_S_mps2",
            "specific_force_z_S_mps2",
            "gyro_valid",
            "accel_valid",
        )
    if config.profile.kind is SensorKind.CUSTOM_VECTOR:
        return (*config.profile.channel_names, "valid")
    return ("center_ray_range_m", "valid")


def configure_meshcat_marine_world(
    meshcat,
    *,
    seafloor_z_W_m: float,
    world_extent_m: float = 100.0,
) -> None:
    """Add restrained water-surface and seafloor context to Meshcat."""

    if not math.isfinite(seafloor_z_W_m) or seafloor_z_W_m >= 0.0:
        raise ValueError("seafloor_z_W_m must be finite and below zero")
    if world_extent_m <= 0.0 or not math.isfinite(world_extent_m):
        raise ValueError("world_extent_m must be positive and finite")
    water_path = "blue_drake/environment/water_surface"
    seafloor_path = "blue_drake/environment/seafloor"
    meshcat.SetObject(
        water_path,
        Box(world_extent_m, world_extent_m, 0.02),
        Rgba(0.05, 0.45, 0.72, 0.12),
    )
    meshcat.SetTransform(water_path, RigidTransform([0.0, 0.0, -0.01]))
    meshcat.SetObject(
        seafloor_path,
        Box(world_extent_m, world_extent_m, 0.1),
        Rgba(0.48, 0.40, 0.27, 1.0),
    )
    meshcat.SetTransform(
        seafloor_path,
        RigidTransform([0.0, 0.0, seafloor_z_W_m - 0.05]),
    )


def build_marine_fleet_diagram(
    vehicles: Mapping[str, MarineVehicleConfig],
    *,
    sensors: Mapping[str, tuple[MountedSensorConfig, ...]] | None = None,
    time_step_s: float = 0.005,
    water_density_kg_m3: float = 1025.0,
    air_density_kg_m3: float = 1.225,
    gravity_mps2: float = 9.81,
    surface_pressure_Pa: float = 101_325.0,
    water_temperature_C: float = 10.0,
    air_temperature_C: float = 15.0,
    seafloor_z_W_m: float = -50.0,
    world_extent_m: float = 100.0,
    logging_period_s: float | None = None,
    meshcat=None,
) -> MarineFleetModel:
    """Build independently forced marine vehicles in one Drake world."""

    if not vehicles:
        raise ValueError("vehicles cannot be empty")
    for vehicle_id in vehicles:
        validate_identifier("vehicle_id", vehicle_id)
    if time_step_s <= 0.0 or not math.isfinite(time_step_s):
        raise ValueError("time_step_s must be positive and finite")
    if any(
        value <= 0.0 or not math.isfinite(value)
        for value in (
            water_density_kg_m3,
            air_density_kg_m3,
            gravity_mps2,
            surface_pressure_Pa,
        )
    ):
        raise ValueError(
            "air and water density, gravity, and surface pressure must be "
            "positive and finite"
        )
    if not math.isfinite(water_temperature_C) or not math.isfinite(
        air_temperature_C
    ):
        raise ValueError("air and water temperature must be finite")
    if not math.isfinite(seafloor_z_W_m) or seafloor_z_W_m >= 0.0:
        raise ValueError("seafloor_z_W_m must be finite and below zero")
    if world_extent_m <= 0.0 or not math.isfinite(world_extent_m):
        raise ValueError("world_extent_m must be positive and finite")
    if logging_period_s is not None and (
        logging_period_s <= 0.0 or not math.isfinite(logging_period_s)
    ):
        raise ValueError("logging_period_s must be positive and finite")
    sensors = {} if sensors is None else dict(sensors)
    unknown_sensor_vehicles = set(sensors) - set(vehicles)
    if unknown_sensor_vehicles:
        raise ValueError(
            "sensor suites reference unknown vehicles: "
            + ", ".join(sorted(unknown_sensor_vehicles))
        )

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder,
        time_step=time_step_s,
    )
    plant.set_name("marine_plant")
    plant.mutable_gravity_field().set_gravity_vector([0.0, 0.0, -gravity_mps2])

    bodies = []
    model_instances = []
    for vehicle_id, config in vehicles.items():
        model_instance = plant.AddModelInstance(vehicle_id)
        body = plant.AddRigidBody(
            "base_link",
            model_instance,
            _spatial_inertia(config),
        )
        _register_vehicle_geometry(plant, body, config)
        bodies.append(body)
        model_instances.append(model_instance)
    _register_seafloor_collision(
        plant,
        seafloor_z_W_m=seafloor_z_W_m,
        world_extent_m=world_extent_m,
    )
    plant.Finalize()

    concatenator = builder.AddSystem(SpatialForceConcatenator(len(vehicles)))
    concatenator.set_name("marine_force_concatenator")
    built_vehicles = []
    fleet_logs = []
    for index, ((vehicle_id, config), model_instance, body) in enumerate(
        zip(vehicles.items(), model_instances, bodies, strict=True)
    ):
        hydrodynamics = builder.AddSystem(
            MarineHydrodynamicForceSystem(
                config,
                body.index(),
                water_density_kg_m3=water_density_kg_m3,
                air_density_kg_m3=air_density_kg_m3,
                gravity_mps2=gravity_mps2,
            )
        )
        hydrodynamics.set_name(f"{vehicle_id}_marine_hydrodynamics")
        builder.Connect(
            plant.get_body_poses_output_port(),
            hydrodynamics.body_poses_input,
        )
        builder.Connect(
            plant.get_body_spatial_velocities_output_port(),
            hydrodynamics.body_velocities_input,
        )
        actuator_system = None
        glider_control_system = None
        applied_wrench_port = hydrodynamics.applied_wrench_input
        if config.actuator_bank is not None:
            actuator_system = builder.AddSystem(
                MarineActuatorSystem(config.actuator_bank)
            )
            actuator_system.set_name(f"{vehicle_id}_actuators")
            wrench_adder = builder.AddSystem(Adder(2, 6))
            wrench_adder.set_name(f"{vehicle_id}_wrench_adder")
            builder.Connect(
                actuator_system.actual_wrench_output,
                wrench_adder.get_input_port(0),
            )
            builder.Connect(
                wrench_adder.get_output_port(),
                hydrodynamics.applied_wrench_input,
            )
            applied_wrench_port = wrench_adder.get_input_port(1)
            builder.ExportInput(
                actuator_system.wrench_command_input,
                f"{vehicle_id}_wrench_command_B",
            )
            builder.ExportOutput(
                actuator_system.actual_wrench_output,
                f"{vehicle_id}_actuator_wrench_B",
            )
            builder.ExportOutput(
                actuator_system.actual_thrust_output,
                f"{vehicle_id}_actual_thrusts_N",
            )
            builder.ExportOutput(
                actuator_system.allocated_thrust_output,
                f"{vehicle_id}_allocated_thrusts_N",
            )
        if config.glider_control is not None:
            glider_control_system = builder.AddSystem(
                GliderControlSystem(config.glider_control)
            )
            glider_control_system.set_name(f"{vehicle_id}_glider_control")
            builder.Connect(
                glider_control_system.actual_output,
                hydrodynamics.glider_control_input,
            )
            builder.ExportInput(
                glider_control_system.command_input,
                f"{vehicle_id}_glider_command",
            )
            builder.ExportOutput(
                glider_control_system.actual_output,
                f"{vehicle_id}_glider_actual",
            )
            builder.ExportOutput(
                glider_control_system.allocated_output,
                f"{vehicle_id}_glider_allocated",
            )
        builder.Connect(
            hydrodynamics.spatial_force_output,
            concatenator.force_inputs[index],
        )
        builder.ExportInput(
            hydrodynamics.water_current_input,
            f"{vehicle_id}_water_current_W_mps",
        )
        builder.ExportInput(
            hydrodynamics.wind_velocity_input,
            f"{vehicle_id}_wind_velocity_W_mps",
        )
        builder.ExportInput(
            applied_wrench_port,
            f"{vehicle_id}_applied_wrench_B",
        )
        builder.ExportOutput(
            plant.get_state_output_port(model_instance),
            f"{vehicle_id}_state",
        )
        if logging_period_s is not None:
            state_logger = LogVectorOutput(
                plant.get_state_output_port(model_instance),
                builder,
                publish_period=logging_period_s,
            )
            state_logger.set_name(f"{vehicle_id}_state_logger")
            fleet_logs.append(
                FleetLog(
                    log_id=f"{vehicle_id}_state",
                    columns=_state_columns(),
                    sink=state_logger,
                )
            )
        fleet_sensors = []
        configured_sensors = tuple(sensors.get(vehicle_id, ()))
        sensor_ids = [sensor.sensor_id for sensor in configured_sensors]
        if len(sensor_ids) != len(set(sensor_ids)):
            raise ValueError(f"sensor IDs for {vehicle_id} must be unique")
        for sensor_config in configured_sensors:
            sensor_system = add_sensor_system(
                builder,
                sensor_config,
                body.index(),
                water_density_kg_m3=water_density_kg_m3,
                gravity_mps2=gravity_mps2,
                surface_pressure_Pa=surface_pressure_Pa,
                water_temperature_C=water_temperature_C,
                air_temperature_C=air_temperature_C,
                seafloor_z_W_m=seafloor_z_W_m,
            )
            sensor_system.set_name(
                f"{vehicle_id}_{sensor_config.sensor_id}_sensor"
            )
            if not isinstance(sensor_system, CustomVectorSensorSystem):
                builder.Connect(
                    plant.get_body_poses_output_port(),
                    sensor_system.body_poses_input,
                )
            if isinstance(sensor_system, RawImuSensorSystem):
                builder.Connect(
                    plant.get_body_spatial_velocities_output_port(),
                    sensor_system.body_velocities_input,
                )
                builder.Connect(
                    plant.get_body_spatial_accelerations_output_port(),
                    sensor_system.body_accelerations_input,
                )
            prefix = f"{vehicle_id}_{sensor_config.sensor_id}"
            if isinstance(sensor_system, CustomVectorSensorSystem):
                builder.ExportInput(
                    sensor_system.value_input,
                    f"{prefix}_value",
                )
            builder.ExportInput(sensor_system.error_input, f"{prefix}_error")
            builder.ExportOutput(sensor_system.ideal_output, f"{prefix}_ideal")
            builder.ExportOutput(
                sensor_system.measurement_output,
                f"{prefix}_measurement",
            )
            if logging_period_s is not None:
                sensor_logger = LogVectorOutput(
                    sensor_system.measurement_output,
                    builder,
                    publish_period=logging_period_s,
                )
                sensor_logger.set_name(f"{prefix}_measurement_logger")
                fleet_logs.append(
                    FleetLog(
                        log_id=f"{prefix}_measurement",
                        columns=_sensor_columns(sensor_config),
                        sink=sensor_logger,
                    )
                )
            fleet_sensors.append(
                FleetSensor(config=sensor_config, system=sensor_system)
            )
        built_vehicles.append(
            FleetVehicle(
                vehicle_id=vehicle_id,
                model_instance=model_instance,
                body=body,
                hydrodynamics=hydrodynamics,
                actuators=actuator_system,
                glider_controls=glider_control_system,
                sensors=tuple(fleet_sensors),
                config=config,
            )
        )

    builder.Connect(
        concatenator.spatial_force_output,
        plant.get_applied_spatial_force_input_port(),
    )
    builder.ExportOutput(plant.get_state_output_port(), "state")
    builder.ExportOutput(plant.get_body_poses_output_port(), "body_poses")
    if meshcat is not None:
        configure_meshcat_marine_world(
            meshcat,
            seafloor_z_W_m=seafloor_z_W_m,
            world_extent_m=world_extent_m,
        )
        ApplyVisualizationConfig(
            VisualizationConfig(),
            builder=builder,
            plant=plant,
            scene_graph=scene_graph,
            meshcat=meshcat,
        )
    return MarineFleetModel(
        diagram=builder.Build(),
        plant=plant,
        scene_graph=scene_graph,
        vehicles=tuple(built_vehicles),
        logs=tuple(fleet_logs),
    )
