# Hardware-informed targets

Vendor and product names identify behaviors that Blue Drake intends to model.
They do not imply affiliation, endorsement, or protocol compatibility.

## Initial scope

- DiveNET Sealink is the only selectable acoustic modem family.
- Blue Robotics targets include Bar depth sensors, Ping sonar, Ping360, Celsius
  temperature, low-light camera, and vehicle-health measurements.
- Cerulean targets include DVL-75, Omniscan 450 forward-looking and side-scan
  variants, Surveyor multibeam echosounder, and ROVL/Omnitrack positioning.
- Xsens targets currently include the MTi-630R and Avior AHRS envelopes.

Popoto modem support is intentionally deferred. Current mounted Xsens systems
expose transparent raw-IMU semantics, not proprietary fused AHRS behavior.
DiveNET support is a generic timing and interference abstraction, not a
Sealink protocol implementation.

## Parameter provenance

Every hardware profile must label each nontrivial parameter as one of:

- `published`: traceable to manufacturer documentation,
- `measured`: traceable to a versioned experimental dataset,
- `fitted`: derived from a documented calibration procedure, or
- `assumed`: a clearly visible placeholder.

A vendor profile remains `provisional` while any behavior-critical parameter is
assumed. Protocol emulation and physical measurement simulation are separate
features; implementing one does not imply the other.
