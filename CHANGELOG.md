# Changelog

All notable changes to Blue Drake are documented here. The project uses
semantic package versions while separately versioning scenarios, run artifacts,
and analytical benchmark output.

## Unreleased

- Pass the custom-named marine plant and scene graph explicitly to Drake's
  visualization configuration, restoring Meshcat startup on Drake 1.54.

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
