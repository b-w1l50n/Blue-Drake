# Reference vehicle cases

Blue Drake's built-in vehicles are reproducible generic reference cases, not
models of named commercial vehicles. They exist so students and professionals
can exercise the complete simulation path with transparent assumptions before
they have identified coefficients for their own platform.

All behavior-defining vehicle preset parameters are currently `assumed`:
dimensions, dry mass and inertia, displaced volume, centers of buoyancy, drag,
added inertia, actuator geometry and limits, glider coefficients, surface
stiffness, and response time constants. They are internally consistent inputs
to documented equations, but they have not been fitted to tow-tank, CFD, pool,
or sea-trial data.

Custom rigid-body inertia diagonals must satisfy the physical triangle
inequalities, and the declared center of buoyancy must lie inside the body
bounding envelope. These are consistency checks, not vehicle validation.

| Reference case | Intended exercise | Release benchmark evidence |
|---|---|---|
| `rov` | Six-axis allocation, pressure/IMU/sonar mounting, buoyancy, drag, and contact | Archimedes support, free-surface transition, drag polynomial, added mass, symmetric thruster geometry |
| `uuv` | Streamlined anisotropic drag and single-axis stern propulsion | Axial stern propeller produces pure body surge |
| `glider` | Low-angle lift, induced drag, buoyancy delta, and pitch-moment surrogate | Wing force scales with speed squared at fixed angle |
| `usv` | Linear displacement support, attitude stiffness, wind, and differential propulsion | Linear heave support plus bounded air/water emergence tests |

The benchmark suite labels these checks `analytical` and their preset parameter
provenance `assumed`. Hardware-informed sensor and modem checks separately list
both `published` envelope inputs and `assumed` environment or abstraction
inputs where applicable.

## Using the cases responsibly

The presets are useful for software integration, sensor geometry, unit and
frame checks, logging, saturation, timing, scenario design, and qualitative
comparisons. Do not use their absolute speed, power, maneuverability, endurance,
or stability as procurement or operational predictions.

To represent a physical vehicle, create a `MarineVehicleConfig` with traceable
parameters and preserve a source record outside the code comments. A future
measured or fitted reference contribution must include the dataset version,
identification method, train/validation split where relevant, acceptance
bounds, and residual plots or equivalent evidence. Replacing one coefficient
with a published value does not validate the whole preset.
