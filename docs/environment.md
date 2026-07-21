# Air, water, and the free surface

Blue Drake treats air and water as distinct environmental phases. World
`z = 0` is the flat mean water surface; negative `z` is underwater. This is a
deterministic engineering foundation, not a free-surface CFD model.

## Scenario inputs

Global scalar properties are explicit SI values:

```toml
water_density_kg_m3 = 1025.0
air_density_kg_m3 = 1.225
surface_pressure_Pa = 101325.0
water_temperature_C = 12.0
air_temperature_C = 18.0
```

Water current and wind are independent world-frame inputs for each vehicle:

```toml
[[vehicles]]
id = "surface_1"
preset = "usv"
position_W_m = [0.0, 0.0, 0.0]
water_current_W_mps = [0.2, 0.0, 0.0]
wind_velocity_W_mps = [4.0, -1.0, 0.0]
```

Both default to zero. They are steady inputs in schema version 1; gusts,
profiles, and time schedules are not yet modeled.

The phase boundary also applies to environmental sensors and acoustic links.
Pressure sensors above the surface use atmospheric pressure and air
temperature; flat-seafloor sonar is invalid above the surface; and acoustic
events are `out_of_medium` when either vehicle origin is above it. A surface
vehicle origin at exactly `z = 0` remains a wet acoustic endpoint.

## Phase fractions

For nominally submerged presets, an upright bounding-box approximation
calculates the fraction below the water surface. Water buoyancy, drag, glider
lift, and diagonal added inertia use that immersed fraction. Quadratic air drag
uses the complementary exposed fraction.

Surface-piercing USVs retain their independently linearized waterplane and
water-drag model. Their exposed box fraction additionally receives aerodynamic
drag. This prevents atmospheric density or wind from being silently treated as
water properties.

## Aerodynamic envelope

For each body axis, air drag is

```text
F_air,i = -0.5 * rho_air * exposed_fraction * C_d,i * A_i * |v_rel,i| * v_rel,i
v_rel = body_velocity - wind_velocity
```

Projected areas come from the vehicle bounding-box dimensions. The current
dimensionless `air_drag_coefficient_xyz` values are explicit assumed envelope
parameters, not vendor, CFD, or wind-tunnel data.

The model omits aerodynamic moments, lift, rotor or propeller aerodynamics,
spray, ventilation, slamming, wave elevation, and orientation-dependent wetted
geometry. Those omissions must remain visible in fidelity claims.
