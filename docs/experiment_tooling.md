# Scenario and experiment tooling

Blue Drake provides lightweight configuration tools and deterministic run
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

Initial geometry is validated before Drake is imported. A USV bounding box
must cross the mean waterline, and no oriented vehicle bounding box may begin
intersecting the configured flat seafloor. These checks turn common setup
mistakes into deterministic configuration errors instead of contact impulses.

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

Inspect the local Python, NumPy, Drake, platform, and Meshcat defaults without
starting a simulator or importing PyDrake:

```bash
blue-drake doctor
blue-drake doctor --json
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

Meshcat binds to localhost by default. For deliberate access from another
computer on a trusted LAN, use a fixed port:

```bash
blue-drake scenarios/fleet_showcase.toml \
  --meshcat-host '*' --meshcat-port 7000
```

Open `http://NUC_IP:7000` from the client. Do not forward that port to the
public internet. Across an untrusted network, leave the localhost default and
use an SSH tunnel instead.

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

- `manifest.json`, containing artifact schema version 2, Blue Drake, Drake,
  NumPy, and Python versions, scenario inspection data, calculated acoustic
  events, simulated duration, logging period, and log column contracts;
- `<vehicle>_state.csv`, containing time, free-body quaternion, world position,
  angular velocity, and translational velocity; and
- `<vehicle>_<sensor>_measurement.csv`, containing the documented sensor output
  vector.

CSV values use round-trip-safe floating-point formatting. Times come from Drake
simulation time, not wall time. Logs remain in memory until the run completes;
large scenarios therefore require an explicit memory-budget review before very
high-rate, long-duration use.

Two identical runs in one supported software environment are required to
produce byte-identical manifests and CSV files. Floating-point trajectories are
not promised to be byte-identical across different Drake, NumPy, Python, CPU,
or operating-system versions; the manifest records the relevant software
versions so comparisons remain reviewable.

The logger does not collect application messages, network traffic, credentials,
ROS data, or hardware interfaces. Acoustic events in the manifest are the same
stationary initial-geometry abstraction reported by `inspect`.
