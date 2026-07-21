"""Marine robotics simulation systems built on Drake."""

from blue_drake.actuators import (
    ActuatorBankConfig,
    ActuatorKind,
    FixedActuatorConfig,
    WrenchAllocation,
    allocate_wrench,
)
from blue_drake.sensors import (
    MountedSensorConfig,
    SensorKind,
    sensor_profile,
)
from blue_drake.vehicles import (
    GliderControlConfig,
    GliderWingConfig,
    MarineVehicleConfig,
    VehicleKind,
    glider_preset,
    rov_preset,
    usv_preset,
    uuv_preset,
)

__all__ = [
    "ActuatorBankConfig",
    "ActuatorKind",
    "FixedActuatorConfig",
    "GliderControlConfig",
    "GliderWingConfig",
    "MarineVehicleConfig",
    "MountedSensorConfig",
    "SensorKind",
    "VehicleKind",
    "WrenchAllocation",
    "allocate_wrench",
    "glider_preset",
    "rov_preset",
    "sensor_profile",
    "usv_preset",
    "uuv_preset",
]
