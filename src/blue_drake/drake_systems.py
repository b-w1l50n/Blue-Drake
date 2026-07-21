"""Small Drake systems that adapt Blue Drake's pure marine calculations."""

from __future__ import annotations

import numpy as np

from blue_drake.actuators import (
    ActuatorBankConfig,
    allocate_wrench,
    wrench_from_thrusts,
)
from blue_drake.hydrodynamics import (
    compute_marine_wrench,
    effective_inertia_wrench,
    submerged_box_fraction,
)
from blue_drake.vehicles import (
    GliderControlConfig,
    HydrostaticMode,
    MarineVehicleConfig,
)

try:
    from pydrake.common.value import AbstractValue
    from pydrake.math import RigidTransform
    from pydrake.multibody.math import SpatialForce, SpatialVelocity
    from pydrake.multibody.plant import ExternallyAppliedSpatialForce
    from pydrake.systems.framework import BasicVector, LeafSystem
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "Drake is required for blue_drake.drake_systems; install the package"
    ) from exc


class MarineActuatorSystem(LeafSystem):
    """Allocate a body-wrench command and model first-order thrust response."""

    def __init__(self, config: ActuatorBankConfig) -> None:
        super().__init__()
        self._config = config
        actuator_count = len(config.actuators)
        self.wrench_command_input = self.DeclareVectorInputPort(
            "wrench_command_B", BasicVector(6)
        )
        self.DeclareContinuousState(actuator_count)
        self.actual_wrench_output = self.DeclareVectorOutputPort(
            "actual_wrench_B", BasicVector(6), self._calc_actual_wrench
        )
        self.actual_thrust_output = self.DeclareVectorOutputPort(
            "actual_thrusts_N",
            BasicVector(actuator_count),
            self._calc_actual_thrust,
        )
        self.allocated_thrust_output = self.DeclareVectorOutputPort(
            "allocated_thrusts_N",
            BasicVector(actuator_count),
            self._calc_allocated_thrust,
        )

    def _actual_thrusts(self, context) -> np.ndarray:
        return context.get_continuous_state_vector().CopyToVector()

    def _allocated_thrusts(self, context) -> np.ndarray:
        command = self.wrench_command_input.Eval(context)
        return allocate_wrench(self._config, command).thrusts_N

    def _calc_actual_wrench(self, context, output) -> None:
        output.SetFromVector(
            wrench_from_thrusts(self._config, self._actual_thrusts(context))
        )

    def _calc_actual_thrust(self, context, output) -> None:
        output.SetFromVector(self._actual_thrusts(context))

    def _calc_allocated_thrust(self, context, output) -> None:
        output.SetFromVector(self._allocated_thrusts(context))

    def DoCalcTimeDerivatives(self, context, derivatives) -> None:
        error = self._allocated_thrusts(context) - self._actual_thrusts(context)
        rates = error / self._config.time_constants_s
        derivatives.get_mutable_vector().SetFromVector(rates)


class GliderControlSystem(LeafSystem):
    """Bound buoyancy-delta and pitch-moment commands with first-order lag."""

    def __init__(self, config: GliderControlConfig) -> None:
        super().__init__()
        self._config = config
        self.command_input = self.DeclareVectorInputPort(
            "command_buoyancy_N_pitch_moment_Nm", BasicVector(2)
        )
        self.DeclareContinuousState(2)
        self.actual_output = self.DeclareVectorOutputPort(
            "actual_buoyancy_N_pitch_moment_Nm",
            BasicVector(2),
            self._calc_actual,
        )
        self.allocated_output = self.DeclareVectorOutputPort(
            "allocated_buoyancy_N_pitch_moment_Nm",
            BasicVector(2),
            self._calc_allocated,
        )

    def _actual(self, context) -> np.ndarray:
        return context.get_continuous_state_vector().CopyToVector()

    def _allocated(self, context) -> np.ndarray:
        limits = np.array(
            [
                self._config.maximum_buoyancy_delta_N,
                self._config.maximum_pitch_moment_Nm,
            ]
        )
        return np.clip(self.command_input.Eval(context), -limits, limits)

    def _calc_actual(self, context, output) -> None:
        output.SetFromVector(self._actual(context))

    def _calc_allocated(self, context, output) -> None:
        output.SetFromVector(self._allocated(context))

    def DoCalcTimeDerivatives(self, context, derivatives) -> None:
        time_constants = np.array(
            [
                self._config.buoyancy_time_constant_s,
                self._config.pitch_time_constant_s,
            ]
        )
        derivatives.get_mutable_vector().SetFromVector(
            (self._allocated(context) - self._actual(context)) / time_constants
        )


class MarineHydrodynamicForceSystem(LeafSystem):
    """Apply buoyancy, drag, and an explicit body-frame wrench to one body."""

    def __init__(
        self,
        config: MarineVehicleConfig,
        body_index,
        *,
        water_density_kg_m3: float,
        air_density_kg_m3: float,
        gravity_mps2: float,
    ) -> None:
        super().__init__()
        self._config = config
        self._body_index = body_index
        self._body_offset = int(body_index)
        self._water_density_kg_m3 = float(water_density_kg_m3)
        self._air_density_kg_m3 = float(air_density_kg_m3)
        self._gravity_mps2 = float(gravity_mps2)

        self.body_poses_input = self.DeclareAbstractInputPort(
            "body_poses", AbstractValue.Make([RigidTransform()])
        )
        self.body_velocities_input = self.DeclareAbstractInputPort(
            "body_spatial_velocities",
            AbstractValue.Make([SpatialVelocity()]),
        )
        self.water_current_input = self.DeclareVectorInputPort(
            "water_current_W_mps", BasicVector(3)
        )
        self.wind_velocity_input = self.DeclareVectorInputPort(
            "wind_velocity_W_mps", BasicVector(3)
        )
        self.applied_wrench_input = self.DeclareVectorInputPort(
            "applied_wrench_B", BasicVector(6)
        )
        self.glider_control_input = None
        if config.glider_control is not None:
            self.glider_control_input = self.DeclareVectorInputPort(
                "glider_buoyancy_N_pitch_moment_Nm", BasicVector(2)
            )
        self.spatial_force_output = self.DeclareAbstractOutputPort(
            "spatial_forces",
            lambda: AbstractValue.Make([ExternallyAppliedSpatialForce()]),
            self._calc_spatial_force,
        )

    def _calc_spatial_force(self, context, output) -> None:
        poses = self.body_poses_input.Eval(context)
        velocities = self.body_velocities_input.Eval(context)
        pose = poses[self._body_offset]
        velocity = velocities[self._body_offset]
        glider_control = (
            np.zeros(2)
            if self.glider_control_input is None
            else self.glider_control_input.Eval(context)
        )
        wrench = compute_marine_wrench(
            self._config,
            rotation_WB=pose.rotation().matrix(),
            body_origin_W_m=pose.translation(),
            angular_velocity_W_radps=velocity.rotational(),
            translational_velocity_W_mps=velocity.translational(),
            water_current_W_mps=self.water_current_input.Eval(context),
            wind_velocity_W_mps=self.wind_velocity_input.Eval(context),
            applied_wrench_B=self.applied_wrench_input.Eval(context),
            glider_control=glider_control,
            water_density_kg_m3=self._water_density_kg_m3,
            air_density_kg_m3=self._air_density_kg_m3,
            gravity_mps2=self._gravity_mps2,
        )
        wrench = effective_inertia_wrench(
            self._config,
            rotation_WB=pose.rotation().matrix(),
            uncorrected_wrench=wrench,
            gravity_mps2=self._gravity_mps2,
            immersion_fraction=(
                submerged_box_fraction(
                    self._config,
                    body_origin_z_W_m=float(pose.translation()[2]),
                )
                if self._config.hydrostatic_mode is HydrostaticMode.SUBMERGED
                else 1.0
            ),
        )
        applied = ExternallyAppliedSpatialForce()
        applied.body_index = self._body_index
        applied.p_BoBq_B = np.zeros(3)
        applied.F_Bq_W = SpatialForce(
            tau=wrench.torque_W_Nm,
            f=wrench.force_W_N,
        )
        output.set_value([applied])


class SpatialForceConcatenator(LeafSystem):
    """Combine one externally applied force list per vehicle."""

    def __init__(self, input_count: int) -> None:
        super().__init__()
        if input_count <= 0:
            raise ValueError("input_count must be positive")
        model_value = AbstractValue.Make([ExternallyAppliedSpatialForce()])
        self.force_inputs = tuple(
            self.DeclareAbstractInputPort(
                f"spatial_forces_{index}", model_value
            )
            for index in range(input_count)
        )
        self.spatial_force_output = self.DeclareAbstractOutputPort(
            "spatial_forces",
            lambda: AbstractValue.Make([ExternallyAppliedSpatialForce()]),
            self._calc_output,
        )

    def _calc_output(self, context, output) -> None:
        forces = []
        for port in self.force_inputs:
            forces.extend(port.Eval(context))
        output.set_value(forces)
