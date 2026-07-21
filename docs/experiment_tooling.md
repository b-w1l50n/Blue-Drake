# Scenario and experiment tooling

Milestone 7 adds lightweight configuration tools and deterministic run
artifacts. Validation, inspection, and catalog commands do not import Drake,
start Meshcat, or run a simulation.

## Versioned scenarios

Scenario files should declare the current schema explicitly:

```toml
schema_version = 1
name = "my-experiment"
```

Omitting `schema_version` is interpreted as version 1 for compatibility with
earlier Blue Drake scenarios. Unknown versions are rejected rather than being
silently reinterpreted.

## CLI workflows

Validate a scenario:

```bash
blue-drake validate scenarios/mixed_marine.toml
```

Inspect vehicles, sensors, and calculated stationary acoustic events:

```bash
blue-drake inspect scenarios/mixed_marine.toml
blue-drake inspect scenarios/mixed_marine.toml --json
```

List built-in vehicle, sensor, and modem profiles:

```bash
blue-drake catalog
blue-drake catalog --json
```

Run explicitly or use the backward-compatible positional form:

```bash
blue-drake run scenarios/mixed_marine.toml --no-visualizer
blue-drake scenarios/mixed_marine.toml --no-visualizer
```

`mixed_marine.toml` is a two-second dynamics and communications example with
continuous propulsion commands. It is not a station-keeping demonstration.
For a long browser session, `fleet_showcase.toml` runs for five minutes with
explicit ROV and UUV trim loads and no continuous horizontal propulsion:

```bash
blue-drake scenarios/fleet_showcase.toml
```

Machine-readable output uses JSON with stable unit-bearing field names where
applicable. It is an inspection interface, not a vendor protocol.

## Deterministic run artifacts

Enable periodic state and sensor-measurement logging at the application
boundary:

```bash
blue-drake run scenarios/custom_sensors.toml \
  --no-visualizer --realtime-rate 0 \
  --output-dir runs/custom-sensor-001 \
  --log-period 0.02
```

The target directory must not exist. Blue Drake never replaces or appends to an
existing run directory. A successful run creates:

- `manifest.json`, containing artifact schema version 1, scenario inspection
  data, calculated acoustic events, simulated duration, logging period, and log
  column contracts;
- `<vehicle>_state.csv`, containing time, free-body quaternion, world position,
  angular velocity, and translational velocity; and
- `<vehicle>_<sensor>_measurement.csv`, containing the documented sensor output
  vector.

CSV values use round-trip-safe floating-point formatting. Times come from Drake
simulation time, not wall time. Logs remain in memory until the run completes;
large scenarios therefore require an explicit memory-budget review before very
high-rate, long-duration use.

The logger does not collect application messages, network traffic, credentials,
ROS data, or hardware interfaces. Acoustic events in the manifest are the same
stationary initial-geometry abstraction reported by `inspect`.
