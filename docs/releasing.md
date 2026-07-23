# Release procedure

Blue Drake releases are evidence checkpoints, not claims that the generic
models qualify hardware. A release manager must complete this procedure from a
clean commit on the supported Ubuntu 24.04 and Drake 1.54 configuration.

## Candidate checks

1. Confirm that the version in `src/blue_drake/_version.py`, changelog heading,
   compatibility policy, and intended Git tag agree. Python release candidates
   use PEP 440 form such as `1.0.0rc1`; Git tags use `v1.0.0rc1`.
2. Run formatting, lint, tests, and the analytical benchmark:

   ```bash
   ruff check .
   ruff format --check .
   pytest
   blue-drake benchmark
   ```

3. Validate every checked-in scenario and run the custom composition example:

   ```bash
   blue-drake validate scenarios/mixed_marine.toml
   blue-drake validate scenarios/custom_sensors.toml
   blue-drake validate scenarios/fleet_showcase.toml
   python examples/custom_vehicle.py
   python examples/station_keeping.py
   ```

4. Build both distributions, validate their metadata, and install the wheel in
   a new virtual environment. Run `blue-drake --version`, `doctor`, `validate`,
   and `benchmark` from outside the source tree.
5. Run all short scenarios headlessly and the complete 300-second simulated
   `fleet_showcase.toml` endurance case. Preserve its manifest and CSV outputs
   as CI artifacts.
6. Review the diff from the previous tag for accidental autonomy, ROS
   application, HIL, C2, credentials, operational endpoints, proprietary
   protocol behavior, generated files, or unsupported fidelity claims.
7. Push the candidate commit and require every public CI job to pass. Only then
   create the signed or annotated release tag. Never move a published release
   tag; correct a defect with a new release candidate.

GitHub Actions implements the reproducible portions of this procedure. A local
pass on another platform is useful evidence but does not replace the supported
Ubuntu release gate.

## Post-release smoke check

From a fresh checkout of the tag, follow the
[getting-started guide](getting_started.md) without relying on an existing
development environment. Confirm that the Meshcat URL is locally reachable and
that an SSH-tunneled NUC session remains bound to localhost by default. Record
release defects publicly unless they contain security-sensitive information.
