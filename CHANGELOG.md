# Changelog

All notable changes to Blue Drake are documented here. The project uses
semantic package versions while separately versioning scenarios, run artifacts,
and analytical benchmark output.

## Unreleased

- Apply one bounded emergence envelope to all USV water-dependent loads,
  eliminating water drag, restoring torque, and added inertia in air.
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
