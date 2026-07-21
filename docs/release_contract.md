# Blue Drake 1.0 release contract

Blue Drake 1.0 is a free, developer-oriented marine robotics simulation
foundation built on Drake. Its purpose is to make bounded engineering models
easy to inspect, compose, test, and replace. It is not intended to reproduce
CFD, a high-detail commercial simulator, or a validated vehicle digital twin.

This contract is the acceptance boundary for `1.0.0rc1`. A capability does not
enter the release merely because it can be rendered or demonstrated; it must
have a documented model, deterministic tests, and evidence appropriate to the
claim.

## Supported scope

The release supports:

- generic UUV, ROV, underwater-glider, and USV rigid bodies;
- distinct air and water properties around a flat mean free surface;
- documented hydrostatic, drag, added-inertia-foundation, glider,
  surface-restoring, and actuator approximations;
- flat fixed-seafloor contact and Meshcat presentation;
- mounted pressure, raw-IMU, and center-ray sonar calculations with
  hardware-informed Blue Robotics, Cerulean, Xsens MTi, and Xsens Avior
  envelopes;
- a provisional DiveNET Sealink envelope and deterministic, stationary
  acoustic event scheduling;
- custom physical sensor profiles and bounded caller-supplied vector sensors;
- schema-validated scenarios, deterministic headless execution, non-overwriting
  run artifacts, analytical and reference validation, and Meshcat viewing;
- deterministic axis-connected grid path planning that is independent of
  controllers and mission state; and
- documented Ubuntu 24.04, WSL2, and Linux NUC workflows.

The exact fidelity of each item is maintained in the
[fidelity matrix](fidelity.md), not inferred from this list.

## Explicit exclusions

The release does not contain or claim:

- mission autonomy, path execution, vehicle controllers, or behavior trees;
- ROS application logic, HIL orchestration, C2 workflows, operational
  endpoints, credentials, or learned-model services;
- vendor wire protocols, vendor firmware behavior, or live hardware control;
- proprietary Xsens fusion, sonar imagery, acoustic BER or multipath physics;
- general bathymetry, waves, CFD, seakeeping, flexible bodies, or a validated
  digital twin; or
- safety qualification for a physical vehicle.

These are boundaries, not a backlog that must be completed for 1.0.

## Release gates

`1.0.0rc1` may be stamped only when all of these gates pass:

1. Public Python entry points, Drake diagram ports, CLI behavior, scenario
   schema, artifact schema, benchmark schema, frames, units, and compatibility
   policy are documented and regression-tested.
2. Model tests cover signs, frame transformations, dimensional behavior,
   dissipative loads, equilibrium, air/water transitions, actuator authority,
   contact, and finite behavior under declared operating limits.
3. Every public vehicle class has a reproducible reference case whose assumed,
   fitted, measured, and published parameters remain distinguishable.
4. Sensors, acoustics, planning, logging, inspection, and error paths are
   deterministic and reject malformed input without hidden I/O or wall time.
5. All checked-in scenarios validate and run through their intended smoke or
   endurance path.
6. Formatting, lint, unit/integration tests, analytical/reference benchmarks,
   reproducibility checks, wheel and sdist construction, Twine validation,
   clean-wheel installation, and public CI pass.
7. A clean-environment user walkthrough can install the package, run a
   scenario, inspect artifacts, expose Meshcat deliberately on a trusted LAN,
   and extend a sensor or vehicle using only repository documentation.

## Change discipline

Release work follows the engineering spirit of Drake's review process. Design
intent precedes implementation, changes remain reviewable, and physical-model
changes include equations, frames, units, limitations, and evidence. Changes
should remain below 750 added or modified lines where practical and must not
exceed 1,500 without an explicit maintainer exception.

The acceptance criterion is deliberately human: a marine robotics developer
unfamiliar with the implementation can install, run, inspect, and extend Blue
Drake while correctly understanding what it does and does not simulate.
