# Changelog

All notable changes to Blue Drake are documented here. The project uses
semantic package versions while separately versioning scenarios, run artifacts,
and analytical benchmark output.

## Unreleased

No changes yet.

## 1.0.1 - 2026-07-23

First final 1.x release. There was no separate `v1.0.0` final tag;
`v1.0.0rc1` was the public-contract release candidate.

- Add a bounded geometric station-keeping controller with explicit Drake state,
  desired-pose, and body-wrench ports.
- Demonstrate outer `DiagramBuilder` composition through the existing actuator
  allocation and lag path.
- Add an optional articulated ROV parallel gripper with a welded palm, two
  prismatic jaws, joint limits, actuators, collision geometry, and bounded
  joint-space control.
- Add executable station-keeping and gripper trajectory examples.
- Extend the analytical benchmark suite from 11 to 13 checks.
- Document controller frames, equations, tuning limits, gripper port semantics,
  assumed parameters, and the manipulation fidelity boundary.
- Streamline the project README while preserving the project disclaimer and
  scope boundaries.

## 1.0.0rc1 - 2026-07-21

First public-contract release candidate. It is aimed at affordable, repeatable
sensor and vehicle experiments for students, plus quick plausibility checks for
professionals. It is not a substitute for hardware qualification, controlled
water testing, or field trials.

- Apply one bounded emergence envelope to all USV water-dependent loads,
  eliminating water drag, restoring torque, and added inertia in air.
- Add machine-readable benchmark subjects, evidence types, provenance and
  sources, plus an analytical UUV propulsion reference case.
- Reject physically impossible dry-inertia diagonals, out-of-envelope centers
  of buoyancy, airborne USV starts, and initial seafloor intersections.
- Enforce sonar and acoustic-modem depth ratings, and prevent range error from
  manufacturing a sonar target outside the true geometric envelope.
- Record software versions in artifact schema 2, verify byte-identical repeated
  runs, and report expected runtime failures without Python tracebacks.
- Add supported scenario-to-diagram and context-configuration Python APIs so
  library users do not depend on CLI internals.
- Add a read-only `doctor` command and complete Ubuntu, WSL2, secure NUC, and
  first sensor-experiment instructions.
- Add shared, machine-readable provenance and source URLs to vehicle
  configurations, scenario inspection, and catalogs.
- Add a tested custom-vehicle and mounted-sensor Python example with explicit
  consistency, provenance, and validation guidance.
- Pass the custom-named marine plant and scene graph explicitly to Drake's
  visualization configuration, restoring Meshcat startup on Drake 1.54.
- Scale buoyancy and water-relative loads through an upright-box free-surface
  transition so positive-buoyancy vehicles cannot accelerate into the sky.
- Add a five-minute, trimmed `fleet_showcase.toml` Meshcat scenario without
  permanent horizontal propulsion commands.
- Separate air density and wind from water density and current, and apply an
  exposed-box aerodynamic drag envelope above the free surface.
- Use separate air and water temperatures for pressure measurements, invalidate
  flat-seafloor sonar above water, and report out-of-medium acoustic events.
- Add fixed flat-seafloor collision geometry matching the rendered plane.
- Bind Meshcat to localhost by default and provide explicit host and port flags
  for deliberate trusted-LAN access.
- Reduce subsea and surface propulsor authority as their body leaves the water,
  while retaining unrestricted external-wrench inputs for test loads.

## 0.1.0 - 2026-07-21

Initial student release candidate:

- UUV, ROV, underwater-glider, and USV presets using Drake rigid bodies;
- hydrostatics, body-axis drag, diagonal zero-rate added mass, low-angle glider
  lift, linearized surface restoring forces, and bounded actuator allocation;
- Blue Robotics, Cerulean, Xsens MTi-630R, and Xsens Avior sensor envelopes;
- custom physical sensor profiles and bounded custom-value vector sensors;
- provisional DiveNET Sealink timing and deterministic shared-channel events;
- Meshcat marine context, TOML scenarios, inspection/catalog commands, and
  deterministic CSV/JSON run artifacts;
- pure deterministic 2D/3D axis-connected A* path planning; and
- versioned analytical benchmark evidence and release/package smoke checks.

Known limitations are maintained in [the fidelity matrix](docs/fidelity.md).
This release contains no mission autonomy, ROS application, HIL orchestration,
C2 workflow, operational endpoint, credential, or proprietary mission logic.
