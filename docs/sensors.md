# Sensors and profile provenance

Blue Drake separates published device envelopes from simulation behavior.
A profile records manufacturer-published limits and geometry. Measurement
functions provide transparent physics. Error inputs provide deterministic,
replayable uncertainty. No profile represents a vendor's proprietary onboard
filter, firmware, target detector, or acoustic image formation.

## Coordinate and output contracts

Every sensor has a rigid transform from sensor frame S to vehicle body frame B:

- `position_B_m` is the sensor origin measured from the body origin in B.
- `rpy_BS_deg` orients S in B using extrinsic roll, pitch, and yaw.
- world and body frames use `x` forward, `y` left, and `z` up.

Fleet ports use `<vehicle>_<sensor>_<suffix>` names. Each sensor exports
`ideal`, `measurement`, and `error` ports. The measured output includes the
configured constant bias plus the error input. The example CLI fixes error
inputs to zero; callers may connect a seeded Drake random source or replayed
error log.

Output vectors are:

| Kind | Output values |
|---|---|
| Pressure | `[pressure_Pa, inferred_depth_m, temperature_C, valid]` |
| Raw IMU | `[gyro_S_radps(3), specific_force_S_mps2(3), gyro_valid, accel_valid]` |
| Sonar | `[center_ray_range_m, valid]` |

Pressure is calculated from the sensor's world `z`, surface pressure, water
density, and gravity. Depth is inferred back from measured pressure. Raw IMU
specific force is evaluated at the mounted sensor origin and includes rigid
body tangential and centripetal acceleration. Sonar intersects only the
center ray (`+x` in S) with a horizontal seafloor plane.

## Hardware-informed profiles

Sources were retrieved on 2026-07-21.

| Profile ID | Published fields represented | Primary source |
|---|---|---|
| `blue-robotics-bar02` | 2 bar range, ~10 m depth, 0.16 mm nominal depth resolution, ±2 °C temperature accuracy | [Blue Robotics Bar sensors](https://bluerobotics.com/store/sensors-cameras/sensors/bar-depth-pressure-sensor/) |
| `blue-robotics-bar30` | 30 bar range, ~300 m depth, 2 mm nominal depth resolution, ±4 °C temperature accuracy | [Blue Robotics Bar sensors](https://bluerobotics.com/store/sensors-cameras/sensors/bar-depth-pressure-sensor/) |
| `blue-robotics-ping-sonar` | 115 kHz, 0.3–100 m, 25° beamwidth, 0.5% waterfall range resolution, 300 m rating | [Blue Robotics Ping Sonar](https://bluerobotics.com/store/sonars/echosounders/ping-sonar-r2-rp/) |
| `cerulean-surveyor-240-16` | 240 kHz, 80° by 4° transmit field, 50 m suggested range, 0.5% range resolution, 300 m rating, up to 20 Hz at short range | [Cerulean Surveyor specifications](https://docs.ceruleansonar.com/c/surveyor-240-16/specifications) |
| `cerulean-omniscan-450fs-300m` | 450 kHz, 120 m maximum range, 0.8° by 50° beam, 1/1200 range resolution, 300 m rating, 20 Hz maximum | [Cerulean Omniscan specifications](https://ceruleansonar.com/product/omniscan-450fs-100m-300m/) |
| `xsens-mti-630r` | raw gyro/accelerometer ranges and noise densities, 2 kHz maximum output rate, headline AHRS attitude accuracy | [Xsens MTi-630R datasheet](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-630R.pdf) |
| `xsens-avior-ahrs` | ±300°/s gyro, ±8 g accelerometer, raw noise densities, 2 kHz high-rate output, headline AHRS attitude accuracy | [Xsens Avior datasheet](https://www.movella.com/hubfs/A-and-M-Avior/A%26M%20-%20Datasheet%20Avior%20LR.pdf) |

Product names identify selectable simulation targets only. Blue Robotics,
Cerulean Sonar, Xsens, and their parent companies do not sponsor or endorse
Blue Drake.

## Error semantics

Biases and error inputs use output-native units. Pressure sensors accept
`[pressure_Pa, temperature_C]`; IMUs accept three gyro errors in rad/s followed
by three accelerometer errors in m/s²; sonar accepts range error in meters.
Values are applied before range clipping and validity evaluation.

Published resolution and noise density are metadata, not automatically treated as Gaussian
standard deviation. Resolution, accuracy, repeatability, bias stability, and
noise density are different quantities. A future sampled-noise adapter may
convert a documented noise density using a declared bandwidth and sampling
model, but Blue Drake does not invent that relationship. The Avior selection
therefore changes the sourced device envelope, not the raw-IMU output contract.

## Deliberate omissions

- Xsens sensor fusion, magnetic field mapping, heading modes, and protocol,
- pressure temperature compensation, drift, and long-term wet exposure,
- sonar beam footprints, multipath, attenuation, confidence, and profiles,
- Cerulean multibeam point detection and Omniscan imagery,
- occlusion and intersection with arbitrary SceneGraph geometry,
- device clocks, sampling events, latency, packet formats, and dropouts, and
- drivers, ROS messages, HIL, and hardware communication.

The Cerulean and Ping systems currently share the same transparent center-ray
geometry adapter. Their profile metadata is selectable, but rendered imagery
or a multibeam point cloud must not be claimed.
