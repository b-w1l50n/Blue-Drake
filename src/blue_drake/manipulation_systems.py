"""Drake systems for generic subsea manipulation."""

from __future__ import annotations

from blue_drake.manipulation import (
    ParallelJawGripperConfig,
    parallel_jaw_actuation,
)

try:
    from pydrake.systems.framework import BasicVector, LeafSystem
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "Drake is required for blue_drake.manipulation_systems; "
        "install the package"
    ) from exc


class ParallelJawGripperController(LeafSystem):
    """Bounded joint-space PD control for a symmetric parallel gripper."""

    def __init__(self, config: ParallelJawGripperConfig) -> None:
        super().__init__()
        self._config = config
        self.state_input = self.DeclareVectorInputPort(
            "gripper_state", BasicVector(4)
        )
        self.desired_opening_input = self.DeclareVectorInputPort(
            "desired_opening_m", BasicVector(1)
        )
        self.actuation_output = self.DeclareVectorOutputPort(
            "joint_actuation_N",
            BasicVector(2),
            self._calc_actuation,
        )

    def _calc_actuation(self, context, output) -> None:
        output.SetFromVector(
            parallel_jaw_actuation(
                self._config,
                state=self.state_input.Eval(context),
                desired_opening_m=float(
                    self.desired_opening_input.Eval(context)[0]
                ),
            )
        )
