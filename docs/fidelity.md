# Model fidelity

Blue Drake reports capabilities narrowly. A rendered feature is not necessarily
a physical model.

| Capability | Current status | Important omissions |
|---|---|---|
| Rigid-body motion | Implemented with Drake `MultibodyPlant` | Flexible bodies |
| Submerged buoyancy | Implemented | Compressibility, flooding |
| Surface hydrostatics | Linearized heave, roll, and pitch | Waves, slamming, planing |
| Linear/quadratic drag | Implemented per body axis | Cross-coupling, CFD |
| Restoring moment | Center-of-buoyancy offset | Metacentric model |
| Added inertia | Diagonal zero-rate effective-inertia scaling | Coupling, Coriolis, frequency effects |
| Wrench allocation | Bounded weighted least squares | Temporal optimization, failures |
| Actuator response | First-order thrust lag | Motor, shaft, inflow, wake |
| ROV actuation | Generic eight-thruster layout | Vendor curves and interactions |
| UUV propulsion | Generic stern propeller | Propeller curves and fins |
| Glider lift | Low-angle symmetric wing foundation | Stall, sideslip, unsteady flow |
| Glider control | Lagged buoyancy delta and pitch-moment surrogate | Pump and movable-mass mechanics |
| USV actuation | Generic differential propellers | Rudders, ventilation, curves |
| USV dynamics | Linear displacement-mode foundation | Seakeeping, hull form, wind |
| Contact | Primitive collision geometry | Hydroelastic tuning |
| DiveNET Sealink | Published envelope plus provisional deterministic packet schedule | Protocol, channel physics, hardware validation |
| Bar02 / Bar30 | Hydrostatic pressure and depth | Compensation, drift, wet aging |
| Xsens MTi-630R | Mounted raw IMU envelope | Proprietary fusion, magnetics, clock |
| Xsens Avior AHRS | Mounted raw IMU envelope | Proprietary fusion, magnetics, clock |
| Ping Sonar | Flat-seafloor center ray | Beam return, confidence, profile |
| Cerulean sonars | Profiles plus center ray | Point clouds, imagery, onboard processing |
| Acoustic events | Stationary range, timing, collision, and half-duplex diagnostics | BER/PER, multipath, Doppler, queues, moving links |
| Meshcat marine world | Water plane and flat seafloor context | Bathymetry, waves, shoreline, water optics |
| Custom physical profiles | User-defined envelope with existing pressure, raw-IMU, or center-ray physics | Automatic calibration, validation of user sources |
| Custom vector sensors | Explicit bounded value/error ports with channel metadata | Physical generation, unit conversion, timing, device protocols |
| Run logging | Periodic generalized state and sensor measurements in CSV with JSON manifest | Streaming, compression, bounded-memory retention |
| Grid path planning | Optimal deterministic 4/6-connected A* | Dynamic obstacles, vehicle constraints, smoothing, execution |

## Validation rule

A capability may move from "foundation" to "implemented" only when its tests
include a physical invariant, an analytic result, published manufacturer data,
or a clearly identified experimental dataset. Assumed and fitted parameters must
remain distinguishable from published specifications.
