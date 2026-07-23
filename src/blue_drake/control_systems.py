"""Drake systems for generic marine control experiments."""

from __future__ import annotations

from blue_drake.controls import StationKeepingGains, station_keeping_wrench

try:
    from pydrake.systems.framework import BasicVector, LeafSystem
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "Drake is required for blue_drake.control_systems; install the package"
    ) from exc


class StationKeepingController(LeafSystem):
    """Bounded geometric PD station keeping for one floating marine body."""

    def __init__(self, gains: StationKeepingGains) -> None:
        super().__init__()
        self._gains = gains
        self.state_input = self.DeclareVectorInputPort(
            "estimated_state", BasicVector(13)
        )
        self.desired_pose_input = self.DeclareVectorInputPort(
            "desired_pose_W", BasicVector(7)
        )
        self.wrench_command_output = self.DeclareVectorOutputPort(
            "wrench_command_B",
            BasicVector(6),
            self._calc_wrench_command,
        )

    def _calc_wrench_command(self, context, output) -> None:
        output.SetFromVector(
            station_keeping_wrench(
                self._gains,
                state=self.state_input.Eval(context),
                desired_pose=self.desired_pose_input.Eval(context),
            )
        )
