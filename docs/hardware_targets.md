# Hardware-informed targets

Vendor and product names identify behaviors that Blue Drake intends to model.
They do not imply affiliation, endorsement, or protocol compatibility.

## Selectable 1.0 scope

- DiveNET Sealink is the only selectable acoustic modem family.
- Blue Robotics profiles include Bar02, Bar30, and Ping Sonar.
- Cerulean profiles include Omniscan 450FS 300 m and Surveyor 240-16.
- Xsens targets currently include the MTi-630R and Avior AHRS envelopes.

Popoto modem support is intentionally deferred. Current mounted Xsens systems
expose transparent raw-IMU semantics, not proprietary fused AHRS behavior.
DiveNET support is a generic timing and interference abstraction, not a
Sealink protocol implementation.

Ping360, Celsius, cameras, vehicle-health measurements, Cerulean DVL and
positioning products, side-scan output, and other catalog products are not
selectable 1.0 profiles. They may be considered later when their simulated
measurement, provenance, and validation boundary are designed; a product name
alone is not a feature.

## Parameter provenance

Every hardware profile must label each nontrivial parameter as one of:

- `published`: traceable to manufacturer documentation,
- `measured`: traceable to a versioned experimental dataset,
- `fitted`: derived from a documented calibration procedure, or
- `assumed`: a clearly visible placeholder.

A vendor profile remains `provisional` while any behavior-critical parameter is
assumed. Protocol emulation and physical measurement simulation are separate
features; implementing one does not imply the other.
