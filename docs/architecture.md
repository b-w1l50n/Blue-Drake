# Architecture

Blue Drake separates deterministic simulation semantics from applications and
external integrations.

```text
scenario files
    |
    v
validated configuration --> wrench allocation --> actuator dynamics
          |                                         |
          +--> acoustic event schedule              |
                                                    |
                                                    v
environment inputs --------------------------> marine rigid body
                                                    |
                                  +-----------------+----------------+
                                  v                 v                v
                             sensors       state/diagnostics   optional Meshcat
```

The core library must not open sockets, poll files, depend on wall time, or
require a visualization server. Application adapters may do those things while
remaining optional.

## Initial packages

- `blue_drake.vehicles`: immutable physical configurations and presets.
- `blue_drake.actuators`: fixed-actuator geometry and bounded allocation.
- `blue_drake.hydrodynamics`: Drake-independent marine wrench calculations.
- `blue_drake.drake_systems`: small systems that adapt calculations to Drake.
- `blue_drake.simulation`: fleet diagram construction.
- `blue_drake.scenario`: strict TOML loading and validation.
- `blue_drake.acoustics`: hardware-informed modem configuration and pure,
  stationary acoustic event scheduling.
- `blue_drake.sensors`: sourced profiles and Drake-independent measurement math.
- `blue_drake.sensor_systems`: mounted Drake sensor adapters.
- `blue_drake.cli`: a thin example runner, not simulation state.

Actuated vehicle presets export a six-element commanded body-wrench input and
per-actuator diagnostics. A separate applied body-wrench input remains an
explicit extension seam for external loads. See [actuation](actuation.md) for
the allocation equation, port contract, and limitations.

Sensors consume plant truth through explicit ports and never feed simulation
state. Each exports ideal and measured values; measured values depend on an
explicit error input. This makes deterministic replay the default and keeps
random source selection at the application boundary. See
[sensors](sensors.md).

Custom physical profiles alter only a validated operating envelope and reuse
those same adapters. A `custom_vector` is a separate source adapter: it consumes
an explicit value port rather than plant truth and exposes bounds and metadata
without claiming physical fidelity. See [custom sensors](custom_sensors.md).

Marine force calculations also contain the reviewable effective-inertia,
glider-wing, and surface-restoring foundations described in
[marine dynamics](dynamics.md). They remain pure calculations; Drake adapters
only obtain plant state, connect commands, and emit spatial forces.

Acoustic scheduling is a separate deterministic calculation over declared
transmissions and stationary node positions. The CLI currently evaluates it
from initial scenario geometry; it is not a live network or a Drake plant
state. See [acoustic communication semantics](acoustics.md).

Meshcat environment geometry is presentation only. The water-surface and
seafloor boxes provide spatial context but do not generate waves, contact, or
bathymetric sonar returns.

## Public API policy

Configuration values are immutable. Public names carry units and frame suffixes
where ambiguity is possible. World-frame names end in `_W`; body-frame names end
in `_B`. Breaking changes are acceptable before version 1.0 but must be noted in
release documentation.

The current Python `LeafSystem` adapters are double-only and do not declare
scalar conversion to AutoDiff or symbolic systems. Pure configuration and
measurement functions are likewise NumPy floating-point calculations. A future
scalar-conversion change requires an explicit design review and dedicated
tests; contributors must not infer conversion support merely because a system
derives from Drake `LeafSystem`.
