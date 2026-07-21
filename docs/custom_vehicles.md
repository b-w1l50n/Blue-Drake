# Custom vehicles from Python

Blue Drake's TOML schema intentionally selects reviewed built-in vehicle
presets. A custom physical vehicle is Python model composition, because its
mass, inertia, buoyancy, drag, and actuators deserve explicit code review and
provenance rather than an unstructured bag of scenario numbers.

The runnable [`examples/custom_vehicle.py`](../examples/custom_vehicle.py)
shows the supported lifecycle:

1. start from the closest generic reference case;
2. replace the parameters that are actually known;
3. keep provenance `assumed` unless every behavior-defining value supports a
   stronger aggregate claim;
4. create mounted sensors and a `MarineScenario`;
5. call `build_marine_scenario_diagram()`;
6. call `configure_scenario_context()` on a simulator context; and
7. advance using Drake and read named output ports.

Run it from a repository checkout:

```bash
.venv/bin/python examples/custom_vehicle.py
```

## Parameters that must move together

Changing mass alone is usually inconsistent. Review at least:

- `dry_mass_kg` and `dry_inertia_diagonal_kg_m2`;
- `displaced_volume_m3` and the intended buoyancy margin;
- `dimensions_m`, collision envelope, projected air area, and free-surface
  transition;
- `center_of_buoyancy_B_m` relative to the body-frame mass origin;
- linear, quadratic, and angular drag coefficients;
- translational and rotational added-inertia diagonals;
- actuator positions, directions, signed limits, and time constants; and
- any glider or surface-waterplane terms used by the selected kind.

All vectors use SI units and the body frame documented in
[architecture](architecture.md). Dry inertia diagonals must satisfy the
physical triangle inequalities. The center of buoyancy must lie inside the
declared body envelope.

## Provenance

`parameter_provenance="assumed"` is correct for a learning model, a rough
professional feasibility check, or a configuration assembled from incomplete
information. It is not a mark of poor quality; it is an honest claim boundary.

Use `published`, `measured`, or `fitted` only when the complete configuration
supports that label, and supply HTTPS `parameter_source_urls`. For a fitted
model, retain the dataset version, identification script, held-out validation,
acceptance bounds, and residual evidence. A plausible animation is not a fit.

## What to test before trusting a customization

At minimum, add deterministic tests for:

- gravity and buoyancy equilibrium or the intended buoyancy margin;
- the sign and dissipated power of drag in each axis;
- body-to-world force rotation;
- actuator allocation, saturation, and residual wrench;
- the expected air/water transition;
- seafloor contact at the intended time step; and
- a reference trajectory or equilibrium with explicit tolerances.

These tests make a cheap simulation useful: they catch implementation mistakes
before tank or field time. They do not replace that testing when absolute
performance matters.
