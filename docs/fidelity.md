# Model fidelity

Blue Drake reports capabilities narrowly. A rendered feature is not necessarily
a physical model.

| Capability | Current status | Important omissions |
|---|---|---|
| Rigid-body motion | Implemented with Drake `MultibodyPlant` | Flexible bodies |
| Submerged buoyancy | Archimedes support with upright-box free-surface transition | Oriented wetted volume, waves, compressibility, flooding |
| Surface hydrostatics | Linearized heave, roll, and pitch | Waves, slamming, planing |
| Linear/quadratic drag | Implemented per body axis | Cross-coupling, CFD |
| Air environment | Separate density, steady wind, exposed-box quadratic drag | Gusts, profiles, aerodynamic moments, CFD |
| Restoring moment | Center-of-buoyancy offset | Metacentric model |
| Added inertia | Diagonal zero-rate effective-inertia scaling | Coupling, Coriolis, frequency effects |
| Wrench allocation | Bounded weighted least squares | Temporal optimization, failures |
| Station keeping | Bounded geometric SE(3) PD controller | Integral action, estimator, feedforward, robustness guarantee |
| Actuator response | First-order thrust lag and box-immersion authority envelope | Motor, shaft, inflow, ventilation, wake |
| ROV actuation | Generic eight-thruster layout | Vendor curves and interactions |
| UUV propulsion | Generic stern propeller | Propeller curves and fins |
| Glider lift | Low-angle symmetric wing foundation | Stall, sideslip, unsteady flow |
| Glider control | Lagged buoyancy delta and pitch-moment surrogate | Pump and movable-mass mechanics |
| USV actuation | Generic differential propellers | Rudders, ventilation, curves |
| USV dynamics | Linear displacement-mode foundation with bounded emergence envelope | Seakeeping, wetted hull form, slamming |
| Contact | Flat fixed seafloor and primitive vehicle collision geometry | Terrain, hydroelastic tuning |
| ROV parallel gripper | Welded palm, two prismatic jaws, limits, collision, bounded PD actuation | Link hydrodynamics, compliant pads, transmission, force sensing, calibrated hardware |
| DiveNET Sealink | Range-, depth-, and phase-bounded provisional deterministic packet schedule | Protocol, channel physics, hardware validation |
| Bar02 / Bar30 | Hydrostatic pressure and depth | Compensation, drift, wet aging |
| Xsens MTi-630R | Mounted raw IMU envelope | Proprietary fusion, magnetics, clock |
| Xsens Avior AHRS | Mounted raw IMU envelope | Proprietary fusion, magnetics, clock |
| Ping Sonar | Wet, depth-rated flat-seafloor center ray | Beam return, confidence, profile |
| Cerulean sonars | Profiles plus center ray | Point clouds, imagery, onboard processing |
| Acoustic events | Wet-endpoint stationary range, timing, collision, and half-duplex diagnostics | Mount offsets, BER/PER, multipath, Doppler, queues, moving links |
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
