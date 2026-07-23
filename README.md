# Blue Drake

Blue Drake is a free marine robotics simulation toolkit built on
[Drake](https://drake.mit.edu/). It gives students an affordable place to test
sensor and vehicle ideas before getting water time, and gives professionals a
fast way to run an initial plausibility check.

It supports deterministic headless experiments and interactive
[Meshcat](https://github.com/meshcat-dev/meshcat) visualization for UUVs, ROVs,
underwater gliders, and USVs.

Not affiliated with Drake Sim, we're a student edit meant to expand the sim to
marine robotics (both surface and subsea). I'm grateful to the Drake Devs for
making such a great sim.

Blue Drake is independent and is not endorsed or maintained by the Drake team
or the referenced hardware manufacturers.

## What's included

- Marine rigid-body dynamics with distinct air and water environments
- Generic UUV, ROV, glider, and USV vehicle presets
- Blue Robotics and Cerulean pressure/sonar profiles
- Xsens MTi and Avior inertial-sensor profiles
- A provisional DiveNET Sealink acoustic-modem profile
- Custom vehicle and sensor composition
- Drake-native station keeping and an articulated ROV gripper
- TOML scenarios, CSV/JSON results, diagnostics, benchmarks, and grid planning

Blue Drake `1.0.1` is intentionally a transparent, moderate-fidelity tool.
It is not a validated digital twin or a substitute for bench, tank, lake, or
sea testing.

## Quick start

The supported release environment is CPython 3.12–3.14 on Ubuntu 24.04 with
Drake 1.54.

```bash
git clone https://github.com/b-w1l50n/Blue-Drake.git
cd Blue-Drake
python3 -m venv .venv
.venv/bin/pip install -e '.[drake-current]'
.venv/bin/blue-drake scenarios/mixed_marine.toml
```

The last command prints a local Meshcat URL. Run the all-vehicle showcase with:

```bash
.venv/bin/blue-drake scenarios/fleet_showcase.toml
```

For a terminal-only run or a quick installation check:

```bash
.venv/bin/blue-drake scenarios/mixed_marine.toml \
  --no-visualizer --realtime-rate 0
.venv/bin/blue-drake doctor
.venv/bin/blue-drake benchmark
```

## Documentation

- [Installation and first experiment](docs/getting_started.md)
- [What the models do and do not represent](docs/fidelity.md)
- [Scenario and experiment workflow](docs/experiment_tooling.md)
- [Sensors](docs/sensors.md), [custom sensors](docs/custom_sensors.md), and
  [custom vehicles](docs/custom_vehicles.md)
- [Dynamics](docs/dynamics.md), [actuation](docs/actuation.md), and
  [controls](docs/controls.md)
- [ROV grippers and manipulation](docs/manipulation.md), and
  [acoustics](docs/acoustics.md)
- [Public Python API](docs/public_api.md) and
  [compatibility policy](docs/compatibility.md)
- [1.0 release contract](docs/release_contract.md),
  [changelog](CHANGELOG.md), and [contributing guide](CONTRIBUTING.md)

All physical quantities use SI units. World and body frames are right-handed
with `x` forward, `y` left, and `z` up.

## Project boundaries

Blue Drake does not include mission autonomy, ROS application logic, HIL
orchestration, C2 workflows, credentials, operational endpoints, proprietary
vendor protocols, or live hardware control.

## License

[BSD 3-Clause](LICENSE). Drake and referenced products retain their own
licenses, copyrights, and trademarks.
