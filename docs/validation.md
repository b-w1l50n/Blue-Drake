# Analytical validation evidence

Blue Drake provides a deterministic analytical benchmark suite:

```bash
blue-drake benchmark
blue-drake benchmark --json
```

The JSON form is intended for CI evidence and uses
`benchmark_schema_version = 2`. Each record includes the governing equation,
subject, evidence type, parameter provenance, source URLs where applicable,
expected value, observed value, units, tolerances, error, and pass status.

## Covered checks

| Check | Independent expectation |
|---|---|
| Submerged buoyancy | `rho * displaced_volume * gravity` |
| Free-surface immersion | box centered on waterline has half buoyancy |
| Air drag | `-0.5 * air_density * coefficient * area * abs(speed) * speed` |
| Surge drag | `-linear_coefficient * u - quadratic_coefficient * abs(u) * u` |
| Surface heave | `mass * gravity - heave_stiffness * displacement` |
| Diagonal added mass | `force / (dry_mass + added_mass)` |
| Glider wing scaling | doubling speed at fixed angle multiplies force by four |
| Pressure sensor | `surface_pressure + rho * gravity * depth` |
| Acoustic latency | preamble + serialization + ideal propagation |
| ROV thruster geometry | four symmetric 45-degree forces sum to pure surge |
| UUV propeller geometry | one axial stern propeller produces pure surge |

The suite calculates expectations directly from the documented equations, then
compares them with the public implementation. It does not import Drake, open a
visualizer, use network services, read scenarios, or depend on wall time or
random state. Diagram-level tests separately verify that the corresponding
Drake systems connect and advance.

## Claim boundary

Passing these checks means the implementation agrees with its declared,
idealized equations for the chosen inputs. It does **not** establish:

- hydrodynamic coefficients for a physical vehicle,
- tow-tank, CFD, pool, or sea-trial agreement,
- manufacturer certification or protocol compatibility,
- sensor calibration, drift, onboard filtering, or timing fidelity,
- acoustic packet error performance in a real channel, or
- oriented wetted-volume, wave, slamming, or air-drag fidelity.

Preset coefficients that are not sourced remain illustrative engineering
assumptions. DiveNET timing remains provisional. Advancing a model beyond this
analytical foundation requires a versioned dataset, provenance, comparison
method, acceptance bounds, and documented results.

The per-vehicle interpretation and complete preset provenance boundary are in
[reference vehicle cases](reference_cases.md).
