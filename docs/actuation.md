# Actuation and wrench allocation

Blue Drake provides a generic fixed-direction marine actuator model. It is a
developer-facing command boundary, not a vehicle controller or a
manufacturer-calibrated propulsor model.

## Model contract

For actuator `i`, `r_i` is its position from the body origin in body frame B,
`d_i` is a unit thrust direction in B, and `u_i` is signed thrust in newtons.
Its body wrench in Drake torque-then-force order is

```text
b_i = [r_i x d_i]
      [    d_i   ]

wrench_B = B u, where B = [b_1 ... b_n].
```

`allocate_wrench()` solves the bounded weighted least-squares problem

```text
minimize  || W (B u - commanded_wrench_B) ||_2
subject to minimum_thrust_N <= u <= maximum_thrust_N.
```

It returns the feasible thrust demand, achieved wrench, and residual. A
nonzero residual is normal for an underactuated vehicle or a saturated bank.
Weights are numerical priorities; callers must choose their scaling when
torque and force errors should not receive equal numeric weight.

The Drake adapter applies a separate first-order response to each allocated
thrust:

```text
du_actual / dt = (u_allocated - u_actual) / time_constant_s.
```

This response provides finite actuator bandwidth. It does not represent motor
current, shaft speed, advance ratio, inflow, wake interaction, ventilation, or
cavitation.

The resulting actuator wrench then passes through the documented
box-immersion authority envelope. Underwater propulsors lose authority as the
body emerges; surface propulsors retain full authority at the nominal
waterline and lose it above that point. This is a bounded phase approximation,
not an inflow or ventilation model.

## Fleet diagram ports

Actuated presets export these ports using the vehicle ID as a prefix:

- `<id>_wrench_command_B`: requested body wrench, torque then force,
- `<id>_actuator_wrench_B`: actual body wrench after actuator lag and before
  the medium-authority envelope,
- `<id>_allocated_thrusts_N`: bounded allocation target,
- `<id>_actual_thrusts_N`: lagged actuator state, and
- `<id>_applied_wrench_B`: additive external load that bypasses actuation.

The applied-wrench input is retained for disturbances, tethers, and future
model composition. It must not be used to claim actuator fidelity.

## Preset capability

| Vehicle | Reference layout | Achievable fixed-actuator subspace |
|---|---|---|
| ROV | Four horizontal and four vertical thrusters | Six-axis wrench |
| UUV | One stern propeller | Surge only |
| USV | Differential twin propellers | Coupled surge, pitch, and yaw |
| Glider | Buoyancy-delta and pitch-moment surrogate | No fixed-thruster subspace |

All geometry, limits, and time constants in these presets are illustrative
engineering assumptions. They are not Blue Robotics, Cerulean, Xsens,
DiveNET, or other vendor specifications. Hardware-calibrated presets require
citable curves or test data plus validation tests before they can be added.

## Deliberate omissions

- propeller speed, electrical, and shaft dynamics,
- speed- and inflow-dependent thrust curves,
- control surfaces and lift,
- detailed glider pump and movable-mass mechanics,
- thruster-thruster and thruster-hull interactions,
- faults, dead zones, hysteresis, and command transport, and
- closed-loop control or autonomy.

These omissions keep the command layer composable and prevent assumed values
from being presented as a validated digital twin.
