# Blue Drake

Blue Drake is a developer-oriented marine robotics simulation toolkit built on
top of [Drake](https://drake.mit.edu/). It targets surface and subsea vehicles,
scenario-driven experiments, deterministic headless runs, and Meshcat
visualization.

> **Not affiliated with Drake Sim, we're a student edit meant to expand the sim
> to marine robotics (both surface and subsea). I'm grateful to the Drake Devs
> for making such a great sim.**

Blue Drake is an independent project and is not affiliated with, endorsed by,
or maintained by the Drake development team. "Drake" is used here to identify
the upstream software dependency.

## Scope

The intended public vehicle presets are:

- generic uncrewed underwater vehicles (UUVs),
- tethered remotely operated vehicles (ROVs),
- underwater gliders, and
- uncrewed surface vehicles (USVs).

Blue Drake will provide marine dynamics, actuators, hardware-informed sensors,
acoustic communications, generic control and path-planning primitives, scenario
files, and Meshcat visualization. It will not contain Erinyes mission autonomy,
C2 workflows, HIL orchestration, ROS application logic, or learned-model
services.

Initial hardware-informed targets are Blue Robotics and Cerulean sensors, Xsens
inertial sensors, and a DiveNET Sealink acoustic modem profile. Product names
identify simulation targets only; their manufacturers do not sponsor or endorse
this project.

## Version 0.1 release candidate

The initial release candidate combines experiment tooling and generic geometric
planning with acoustic, dynamics, actuation, and sensor foundations:

- validated scenario and vehicle configuration types,
- submerged and surface-piercing hydrostatic modes,
- linear and quadratic body-frame drag,
- buoyancy and center-of-buoyancy restoring torque,
- fixed thruster and propeller geometry in body coordinates,
- bounded weighted least-squares wrench allocation with explicit residuals,
- first-order actuator response and per-actuator diagnostics,
- commanded-wrench, current, and external-load ports in the Drake fleet,
- one preset each for UUV, ROV, glider, and USV,
- a separately sourced Xsens Avior AHRS profile,
- a provisional DiveNET Sealink profile with published rate, range, channel,
  and device-count metadata,
- deterministic acoustic airtime, propagation, range, collision, half-duplex,
  and transmitter-conflict diagnostics,
- strict TOML acoustic transmission schedules and CLI event summaries,
- translucent water-surface and seafloor context in Meshcat,
- scenario-defined pressure, IMU, and sonar operating envelopes that reuse the
  existing transparent physical calculations,
- bounded custom numeric-vector sensors with named channels and units,
- explicit custom-value, bias, error, ideal, measured, and validity semantics,
- declared `assumed`, `measured`, `fitted`, or `published` provenance for custom
  parameters,
- schema-versioned TOML with lightweight `validate` and `inspect` commands,
- machine-readable built-in vehicle, sensor, and modem catalogs,
- deterministic in-memory state and sensor logging with non-overwriting CSV and
  JSON run artifacts,
- deterministic optimal axis-connected A* paths over 2D and 3D marine grids,
- sourced Blue Robotics, Cerulean, and Xsens profiles,
- mounted pressure/depth, raw IMU, and center-ray sonar outputs,
- separate ideal, measured, and explicit error ports,
- diagonal zero-rate effective-inertia handling for configured added mass,
- low-angle glider lift plus bounded buoyancy and pitch controls,
- linearized USV roll and pitch hydrostatic stiffness,
- deterministic unit and diagram tests,
- a versioned, machine-readable analytical benchmark suite, and
- wheel, sdist, clean-install, and supported Python/Drake CI release gates.

This is not yet a validated marine digital twin. The preset actuator numbers
are illustrative, not vendor data. Coupled added-mass dynamics, wave excitation,
detailed glider mechanisms, tether mechanics, propeller curves, acoustic
channel physics, vendor protocols, and vendor onboard processing are
deliberately deferred and listed in
[the fidelity matrix](docs/fidelity.md).

## Quick start

The supported release path is CPython 3.12-3.14 on Ubuntu 24.04 with Drake
1.54.0. A frozen Drake 1.45 extra remains available for the original macOS
Sonoma development host; that platform is no longer supported upstream. See
[the compatibility policy](docs/compatibility.md).

```bash
python3 -m venv .venv
.venv/bin/pip install -e '.[dev,drake-current]'
.venv/bin/blue-drake scenarios/mixed_marine.toml
```

The command prints a local Meshcat URL. For a terminal-only deterministic run:

```bash
.venv/bin/blue-drake scenarios/mixed_marine.toml \
  --no-visualizer --duration 2
```

Validate or inspect a scenario without launching Drake:

```bash
.venv/bin/blue-drake validate scenarios/mixed_marine.toml
.venv/bin/blue-drake inspect scenarios/mixed_marine.toml --json
.venv/bin/blue-drake benchmark --json
```

The library API also exposes `build_marine_fleet_diagram()` so Drake users can
connect their own controllers, planners, loggers, and analysis systems.

## Coordinates and units

All physical quantities use SI units. World and body frames are right-handed,
with `x` forward, `y` left, and `z` up. Submerged positions therefore have
negative world `z`. Spatial wrench vectors use Drake order: torque followed by
force.

See [architecture](docs/architecture.md), [fidelity](docs/fidelity.md), and
[actuation](docs/actuation.md) before extending a model. Sensor contributors
should also read [sensor semantics and provenance](docs/sensors.md),
[acoustic communication semantics](docs/acoustics.md), and
[marine dynamics](docs/dynamics.md). Custom sensor authors should read
[custom sensor profiles and supplied values](docs/custom_sensors.md), plus
[scenario and experiment tooling](docs/experiment_tooling.md),
[generic grid path planning](docs/planning.md), and
[analytical validation evidence](docs/validation.md). Release users should also
read [compatibility and versioning](docs/compatibility.md),
[the changelog](CHANGELOG.md), and [contributing](CONTRIBUTING.md).

## License

Blue Drake is distributed under the
[BSD 3-Clause License](LICENSE). This license applies to Blue Drake's original
code only; Drake and referenced vendor products retain their own licenses,
copyrights, and trademarks.
