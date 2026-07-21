"""Marine robotics simulation systems built on Drake."""

from blue_drake._version import __version__
from blue_drake.actuators import (
    ActuatorBankConfig,
    ActuatorKind,
    FixedActuatorConfig,
    WrenchAllocation,
    allocate_wrench,
)
from blue_drake.planning import GridPath, MarineGrid, plan_grid_path
from blue_drake.provenance import ParameterProvenance
from blue_drake.sensors import (
    CustomVectorSensorProfile,
    MountedSensorConfig,
    SensorKind,
    sensor_profile,
)
from blue_drake.vehicles import (
    GliderControlConfig,
    GliderWingConfig,
    HydrostaticMode,
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
    "CustomVectorSensorProfile",
    "FixedActuatorConfig",
    "GliderControlConfig",
    "GliderWingConfig",
    "GridPath",
    "HydrostaticMode",
    "MarineVehicleConfig",
    "MarineGrid",
    "MountedSensorConfig",
    "ParameterProvenance",
    "SensorKind",
    "VehicleKind",
    "WrenchAllocation",
    "__version__",
    "allocate_wrench",
    "glider_preset",
    "plan_grid_path",
    "rov_preset",
    "sensor_profile",
    "usv_preset",
    "uuv_preset",
]
