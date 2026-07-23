# Architecture

Blue Drake separates deterministic simulation semantics from applications and
external integrations.

```text
scenario files
    |
    v
validated configuration --> controller --> wrench allocation --> actuator dynamics
          |                                                        |
          +--> acoustic event schedule                             |
                                                                   |
                                                                   v
environment inputs -----------------------------------------> marine rigid body
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
- `blue_drake.controls`: pure geometric feedback calculations and gains.
- `blue_drake.control_systems`: explicit-port Drake controller adapters.
- `blue_drake.hydrodynamics`: Drake-independent marine wrench calculations.
- `blue_drake.drake_systems`: small systems that adapt calculations to Drake.
- `blue_drake.simulation`: fleet diagram construction.
- `blue_drake.scenario`: strict TOML loading and validation.
- `blue_drake.acoustics`: hardware-informed modem configuration and pure,
  stationary acoustic event scheduling.
- `blue_drake.sensors`: sourced profiles and Drake-independent measurement math.
- `blue_drake.sensor_systems`: mounted Drake sensor adapters.
- `blue_drake.inspection`: JSON-ready scenario and built-in catalog summaries.
- `blue_drake.run_artifacts`: non-overwriting application-side CSV/JSON export.
- `blue_drake.planning`: pure 2D/3D grid geometry and A* path search.
- `blue_drake.validation`: pure analytical implementation benchmarks.
- `blue_drake.cli`: a thin example runner, not simulation state.

Actuated vehicle presets export a six-element commanded body-wrench input and
per-actuator diagnostics. A separate applied body-wrench input remains an
explicit extension seam for external loads. See [actuation](actuation.md) for
the allocation equation, port contract, and limitations.

Generic controllers remain separate systems connected through that wrench
boundary. The controller does not bypass allocation or actuator dynamics. See
[marine control systems](controls.md) for the station-keeping contract and
outer-diagram composition pattern.

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

Air and water properties, currents, wind, immersion, and exposed-area semantics
are documented in [the environment contract](environment.md). Air drag and
water-relative loads are calculated separately before their world-frame forces
are combined.

Acoustic scheduling is a separate deterministic calculation over declared
transmissions and stationary node positions. The CLI currently evaluates it
from initial scenario geometry; it is not a live network or a Drake plant
state. See [acoustic communication semantics](acoustics.md).

The Meshcat water surface is presentation only and does not generate waves or
fluid contact. The rendered flat seafloor has matching fixed collision
geometry, and center-ray sonar uses the same analytical seafloor elevation.
General bathymetry and terrain-dependent returns are not modeled.

Vector log sinks are added to a diagram only when requested. They retain
simulation-time state and sensor samples in memory; filesystem export remains
in the CLI-side artifact module. Scenario validation, inspection, and catalog
queries do not import Drake.

Grid planning consumes caller-declared occupancy and returns waypoints. It has
no connection to plant inputs, controllers, acoustic delivery, or mission
state, preserving a hard boundary between generic algorithms and autonomy.

## Public API policy

Configuration values are immutable. Public names carry units and frame suffixes
where ambiguity is possible. World-frame names end in `_W`; body-frame names end
in `_B`. Breaking corrections during the 1.0 release-candidate series require
explicit review and release notes. Scenario, artifact, and benchmark formats
carry their own integer schema versions; see [compatibility](compatibility.md).

The current Python `LeafSystem` adapters are double-only and do not declare
scalar conversion to AutoDiff or symbolic systems. Pure configuration and
measurement functions are likewise NumPy floating-point calculations. A future
scalar-conversion change requires an explicit design review and dedicated
tests; contributors must not infer conversion support merely because a system
derives from Drake `LeafSystem`.
