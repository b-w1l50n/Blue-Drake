"""Drake adapters for deterministic Blue Drake sensor calculations."""

from __future__ import annotations

import numpy as np

from blue_drake.sensors import (
    ImuSensorProfile,
    MountedSensorConfig,
    PressureSensorProfile,
    SensorKind,
    SonarProfile,
    flat_seafloor_range,
    pressure_measurement,
    raw_imu_measurement,
)

try:
    from pydrake.common.value import AbstractValue
    from pydrake.math import RigidTransform, RollPitchYaw
    from pydrake.multibody.math import (
        SpatialAcceleration,
        SpatialVelocity,
    )
    from pydrake.systems.framework import BasicVector, LeafSystem
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "Drake is required for blue_drake.sensor_systems; install the package"
    ) from exc


class _MountedSensorSystem(LeafSystem):
    """Shared mounting and body-pose input behavior."""

    def __init__(self, config: MountedSensorConfig, body_index) -> None:
        super().__init__()
        self._config = config
        self._body_offset = int(body_index)
        self._X_BS = RigidTransform(
            RollPitchYaw(np.deg2rad(config.rpy_BS_deg)),
            config.position_B_m,
        )
        self.body_poses_input = self.DeclareAbstractInputPort(
            "body_poses", AbstractValue.Make([RigidTransform()])
        )

    def _sensor_pose(self, context) -> RigidTransform:
        X_WB = self.body_poses_input.Eval(context)[self._body_offset]
        return X_WB.multiply(self._X_BS)


class PressureSensorSystem(_MountedSensorSystem):
    """Measure hydrostatic pressure, inferred depth, and temperature."""

    def __init__(
        self,
        config: MountedSensorConfig,
        body_index,
        *,
        water_density_kg_m3: float,
        gravity_mps2: float,
        surface_pressure_Pa: float,
        water_temperature_C: float,
    ) -> None:
        if not isinstance(config.profile, PressureSensorProfile):
            raise TypeError("PressureSensorSystem requires a pressure profile")
        super().__init__(config, body_index)
        self._profile = config.profile
        self._water_density_kg_m3 = water_density_kg_m3
        self._gravity_mps2 = gravity_mps2
        self._surface_pressure_Pa = surface_pressure_Pa
        self._water_temperature_C = water_temperature_C
        self.error_input = self.DeclareVectorInputPort(
            "error_pressure_Pa_temperature_C", BasicVector(2)
        )
        self.ideal_output = self.DeclareVectorOutputPort(
            "ideal_pressure_depth_temperature_valid",
            BasicVector(4),
            self._calc_ideal,
        )
        self.measurement_output = self.DeclareVectorOutputPort(
            "pressure_depth_temperature_valid",
            BasicVector(4),
            self._calc_measurement,
        )

    def _measurement(self, context, error: np.ndarray) -> np.ndarray:
        return pressure_measurement(
            self._profile,
            sensor_z_W_m=self._sensor_pose(context).translation()[2],
            water_density_kg_m3=self._water_density_kg_m3,
            gravity_mps2=self._gravity_mps2,
            surface_pressure_Pa=self._surface_pressure_Pa,
            water_temperature_C=self._water_temperature_C,
            error=error,
        )

    def _calc_ideal(self, context, output) -> None:
        output.SetFromVector(self._measurement(context, np.zeros(2)))

    def _calc_measurement(self, context, output) -> None:
        error = np.asarray(self._config.bias) + self.error_input.Eval(context)
        output.SetFromVector(self._measurement(context, error))


class RawImuSensorSystem(_MountedSensorSystem):
    """Measure angular velocity and specific force at a rigid sensor mount."""

    def __init__(
        self,
        config: MountedSensorConfig,
        body_index,
        *,
        gravity_W_mps2: tuple[float, float, float],
    ) -> None:
        if not isinstance(config.profile, ImuSensorProfile):
            raise TypeError("RawImuSensorSystem requires an IMU profile")
        super().__init__(config, body_index)
        self._profile = config.profile
        self._gravity_W_mps2 = gravity_W_mps2
        self.body_velocities_input = self.DeclareAbstractInputPort(
            "body_spatial_velocities",
            AbstractValue.Make([SpatialVelocity()]),
        )
        self.body_accelerations_input = self.DeclareAbstractInputPort(
            "body_spatial_accelerations",
            AbstractValue.Make([SpatialAcceleration()]),
        )
        self.error_input = self.DeclareVectorInputPort(
            "error_gyro_radps_accel_mps2", BasicVector(6)
        )
        self.ideal_output = self.DeclareVectorOutputPort(
            "ideal_gyro_accel_valid", BasicVector(8), self._calc_ideal
        )
        self.measurement_output = self.DeclareVectorOutputPort(
            "gyro_accel_valid", BasicVector(8), self._calc_measurement
        )

    def _measurement(self, context, error: np.ndarray) -> np.ndarray:
        velocity = self.body_velocities_input.Eval(context)[self._body_offset]
        acceleration = self.body_accelerations_input.Eval(context)[
            self._body_offset
        ]
        X_WB = self.body_poses_input.Eval(context)[self._body_offset]
        X_WS = X_WB.multiply(self._X_BS)
        p_BS_W = X_WB.rotation().multiply(self._X_BS.translation())
        angular_velocity_W = velocity.rotational()
        acceleration_WS_W = (
            acceleration.translational()
            + np.cross(acceleration.rotational(), p_BS_W)
            + np.cross(
                angular_velocity_W,
                np.cross(angular_velocity_W, p_BS_W),
            )
        )
        return raw_imu_measurement(
            self._profile,
            rotation_WS=X_WS.rotation().matrix(),
            angular_velocity_W_radps=angular_velocity_W,
            translational_acceleration_WS_W_mps2=acceleration_WS_W,
            gravity_W_mps2=self._gravity_W_mps2,
            error=error,
        )

    def _calc_ideal(self, context, output) -> None:
        output.SetFromVector(self._measurement(context, np.zeros(6)))

    def _calc_measurement(self, context, output) -> None:
        error = np.asarray(self._config.bias) + self.error_input.Eval(context)
        output.SetFromVector(self._measurement(context, error))


class FlatSeafloorSonarSystem(_MountedSensorSystem):
    """Intersect a mounted sonar center ray with a horizontal seafloor."""

    def __init__(
        self,
        config: MountedSensorConfig,
        body_index,
        *,
        seafloor_z_W_m: float,
    ) -> None:
        if not isinstance(config.profile, SonarProfile):
            raise TypeError("FlatSeafloorSonarSystem requires a sonar profile")
        super().__init__(config, body_index)
        self._profile = config.profile
        self._seafloor_z_W_m = seafloor_z_W_m
        self.error_input = self.DeclareVectorInputPort(
            "range_error_m", BasicVector(1)
        )
        self.ideal_output = self.DeclareVectorOutputPort(
            "ideal_center_ray_range_valid", BasicVector(2), self._calc_ideal
        )
        self.measurement_output = self.DeclareVectorOutputPort(
            "center_ray_range_valid", BasicVector(2), self._calc_measurement
        )

    def _measurement(self, context, range_error_m: float) -> np.ndarray:
        X_WS = self._sensor_pose(context)
        beam_direction_W = X_WS.rotation().multiply([1.0, 0.0, 0.0])
        return flat_seafloor_range(
            self._profile,
            sensor_origin_W_m=X_WS.translation(),
            beam_direction_W=beam_direction_W,
            seafloor_z_W_m=self._seafloor_z_W_m,
            range_error_m=range_error_m,
        )

    def _calc_ideal(self, context, output) -> None:
        output.SetFromVector(self._measurement(context, 0.0))

    def _calc_measurement(self, context, output) -> None:
        error = self._config.bias[0] + self.error_input.Eval(context)[0]
        output.SetFromVector(self._measurement(context, error))


def add_sensor_system(
    builder,
    config: MountedSensorConfig,
    body_index,
    *,
    water_density_kg_m3: float,
    gravity_mps2: float,
    surface_pressure_Pa: float,
    water_temperature_C: float,
    seafloor_z_W_m: float,
):
    """Add the Drake adapter appropriate for a mounted sensor profile."""

    if config.profile.kind is SensorKind.PRESSURE:
        system = PressureSensorSystem(
            config,
            body_index,
            water_density_kg_m3=water_density_kg_m3,
            gravity_mps2=gravity_mps2,
            surface_pressure_Pa=surface_pressure_Pa,
            water_temperature_C=water_temperature_C,
        )
    elif config.profile.kind is SensorKind.IMU:
        system = RawImuSensorSystem(
            config,
            body_index,
            gravity_W_mps2=(0.0, 0.0, -gravity_mps2),
        )
    else:
        system = FlatSeafloorSonarSystem(
            config,
            body_index,
            seafloor_z_W_m=seafloor_z_W_m,
        )
    return builder.AddSystem(system)
