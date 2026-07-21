# Public API and compatibility

Blue Drake 1.x distinguishes supported interfaces from implementation details.
This keeps the stable surface small while leaving the models readable and
hackable.

## Supported Python surface

Names exported by `blue_drake.__all__` are the lightweight convenience API.
They do not import Drake and remain compatible throughout the 1.x release
line, subject to the deprecation policy below.

The following module entry points are also supported:

- `blue_drake.scenario.load_scenario()` and the scenario configuration types;
- `blue_drake.simulation.build_marine_fleet_diagram()`,
  `build_marine_scenario_diagram()`, `configure_scenario_context()`, and the
  returned model accessors;
- `blue_drake.hydrodynamics.compute_marine_wrench()` and documented pure force
  calculations;
- `blue_drake.sensors.sensor_profile()`, profile configuration types, and
  documented pure measurement functions;
- `blue_drake.acoustics.modem_profile()`, `estimate_transmission()`, and
  `schedule_transmissions()`;
- `blue_drake.actuators.allocate_wrench()`; and
- `blue_drake.inspection.scenario_summary()` and `catalog_summary()`.

Other importable names are implementation details unless another document
explicitly calls them public. A leading underscore is always private.

## Drake diagram contract

`build_marine_fleet_diagram()` returns a diagram whose exported ports use the
vehicle and sensor identifiers supplied by the scenario. Port names, vector
order, frames, and SI units documented in [architecture](architecture.md),
[actuation](actuation.md), and [sensors](sensors.md) are supported 1.x
interfaces. New optional ports may be added; removing, renaming, reordering, or
changing the meaning of an existing port requires deprecation or a major
release.

The systems are double-only in 1.0. AutoDiff and symbolic scalar conversion are
not implied by the fact that they are Drake systems.

## Data contracts

Scenario TOML, run manifests, and benchmark JSON have independent integer
schema versions. Readers reject unknown scenario versions. A breaking data
change requires a new schema version, parser and serialization tests, and a
migration example. New optional fields may retain a schema version only when an
old document keeps identical meaning.

JSON objects use unit-bearing field names where practical. Object key order is
not semantic. Lists whose documentation declares deterministic ordering retain
that ordering throughout 1.x.

## Errors and side effects

Invalid caller configuration raises `ValueError` or a documented subclass.
Missing optional Drake support raises `ImportError` when a simulation module is
imported or used. Run-artifact creation raises `FileExistsError` rather than
overwriting an existing directory.

Pure model, configuration, inspection, planning, and scheduling APIs do not
open sockets, read ambient files, use wall time, or draw from global random
state. Scenario loading reads only the explicitly supplied path. The CLI owns
Meshcat startup and run-artifact filesystem writes.

## Deprecation policy

An incompatible 1.x Python or diagram-API change should retain the old form for
at least one minor release and emit `DeprecationWarning` with a replacement.
Security or correctness defects that cannot safely preserve old behavior may
be fixed immediately and must be called out in the changelog.

The command-line interface follows the same intent: scripts may rely on
documented commands, flags, exit codes, and machine-readable JSON fields, but
not on human-readable prose or log formatting.
