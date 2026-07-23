# Marine control systems

Blue Drake uses Drake's systems framework for low-level marine control
experiments. Controllers consume explicit state or sensor ports and produce the
same bounded body-wrench commands used by the actuator model. They do not make
mission decisions, execute paths, communicate with hardware, or hide vehicle
physics.

## Geometric station keeping

`StationKeepingController` is a double-only Drake `LeafSystem` around the pure
`station_keeping_wrench()` calculation. It has:

- a 13-element `estimated_state` input in Drake floating-body order:
  quaternion `wxyz`, world position, world angular velocity, and world
  translational velocity;
- a seven-element `desired_pose_W` input containing quaternion `wxyz` and world
  position; and
- a six-element `wrench_command_B` output in torque-then-force order.

The controller computes proportional position and shortest-path rotation errors
in world frame:

```text
force_W  = Kp_position (p_WD - p_WB) - Kd_position v_WB
torque_W = Kp_rotation e_R_W          - Kd_rotation omega_WB
```

Each world-frame vector is norm-limited, then rotated into body frame before it
is emitted. The fleet actuator system performs allocation, per-thruster
saturation, lag, and medium-authority limiting afterward. Keeping these stages
separate exposes both controller saturation and actuator residuals.

This is bounded geometric PD control, not a tuned controller for any physical
vehicle. It has no integral state, estimator, feedforward hydrostatic
compensation, allocation awareness, anti-windup, or robustness guarantee.
Positive buoyancy and steady disturbances therefore produce steady-state
position error.

## Drake composition

The tested [`station_keeping.py`](../examples/station_keeping.py) example adds a
Blue Drake fleet diagram as a subsystem in an outer Drake `DiagramBuilder`,
connects the ROV state to the controller, and connects the body-wrench command
back to the ROV. Desired pose and environment values use Drake constant-source
systems.

This pattern is intentional: applications can replace the example controller
with Drake controllers, estimators, trajectories, loggers, or optimization
systems without changing the marine plant. A controller is generic application
composition; it is not part of scenario TOML and does not create a mission or
autonomy interface.

## Frames and tuning

Position and velocity feedback are world-frame quantities. Commands are
body-frame wrenches because that is the actuator-allocation boundary. Gains are
diagonal and carry SI units in their field names. Force and torque limits bound
Euclidean norms rather than clipping individual axes.

Tune against an identified vehicle model and declared operating condition.
Passing this simulation example does not qualify gains for hardware.
