# Custom sensor profiles and supplied values

Blue Drake lets a scenario declare sensor profiles without changing Python
code. A declaration is either a custom envelope for an existing transparent
physical model or an explicitly supplied numeric vector. These are different
semantics and Blue Drake keeps them visibly separate.

## Physical custom profiles

The `pressure`, `imu`, `echosounder`, `multibeam_echosounder`, and
`forward_looking_sonar` kinds reuse the same calculations as built-in profiles:

- pressure uses mounted depth and the configured hydrostatic environment,
- IMU uses mounted rigid-body velocity and acceleration, and
- sonar intersects its mounted center ray with the flat seafloor.

Only the operating envelope is custom. A scenario cannot inject a `value` into
these profiles or override a built-in profile ID.

All parameter names carry SI units. For example:

```toml
[[sensor_profiles]]
id = "student-pressure"
display_name = "Student Pressure Transducer"
kind = "pressure"
provenance = "assumed"
maximum_pressure_Pa = 2000000.0
approximate_depth_rating_m = 190.0
nominal_depth_resolution_m = 0.01
temperature_accuracy_C = 2.0
```

The IMU fields match `ImuSensorProfile`: three-axis ranges in rad/s and m/s²,
noise densities, maximum output rate, and headline attitude accuracies in
radians RMS. Sonar fields match `SonarProfile`: frequency, minimum and maximum
range, horizontal and vertical field of view in radians, range-resolution
fraction, depth rating, and optional maximum ping rate.

## Supplied custom vectors

A `custom_vector` is intentionally not a physics model. It is useful for a
student-built sensor, a temporary laboratory quantity, a replay adapter, or an
application-defined signal whose physics Blue Drake does not yet represent.

```toml
[[sensor_profiles]]
id = "student-water-quality"
display_name = "Student Water Quality Package"
kind = "custom_vector"
provenance = "assumed"
channel_names = ["salinity", "turbidity"]
units = ["PSU", "NTU"]
minimum_values = [0.0, 0.0]
maximum_values = [50.0, 100.0]
default_values = [35.0, 2.0]

[[vehicles.sensors]]
id = "water_quality"
profile = "student-water-quality"
value = [34.8, 1.2]
bias = [0.1, 0.0]
```

For vehicle `rov_1`, the diagram exports these ports:

| Port | Size | Meaning |
|---|---:|---|
| `rov_1_water_quality_value` | 2 | Caller-supplied values |
| `rov_1_water_quality_error` | 2 | Explicit additive runtime error |
| `rov_1_water_quality_ideal` | 3 | Bounded values plus collective validity |
| `rov_1_water_quality_measurement` | 3 | Value + configured bias + error, bounded, plus validity |

The CLI fixes the value input to the sensor instance's `value`, or to the
profile's `default_values` when `value` is omitted. Library callers must connect
or fix the exported value and error ports themselves. Channel names and units
are metadata; Blue Drake does not infer conversions from unit strings.

A result outside any declared bound is clipped and receives a final validity
value of `0`. A result inside every bound receives `1`. Profiles contain 1 to
64 uniquely named channels and require matching units, bounds, and defaults.

## Provenance

Each custom profile declares one of `assumed`, `measured`, `fitted`, or
`published`. `assumed` is the default. Every other value requires a nonempty
`source_url`; `source_retrieved` may record the retrieval date. This declaration
does not independently validate the source—it prevents unattributed values from
being presented as measured, fitted, or published.

Custom definitions remain local scenario data. They do not load plugins,
execute code, read drivers, open sockets, or communicate with hardware.

See [`scenarios/custom_sensors.toml`](../scenarios/custom_sensors.toml) for a
runnable custom-vector and custom-pressure example.
