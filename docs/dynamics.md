# Marine dynamics foundations

Blue Drake's rigid-body foundation includes three deliberately narrow
models: diagonal effective inertia, low-angle glider lift and buoyancy control,
and linearized surface attitude stiffness. The preset coefficients are
illustrative engineering assumptions. They are not identified system
parameters or validation data for a particular vehicle.

## Flat free-surface transition

Nominally submerged presets use their upright bounding-box height to estimate
the fraction below the flat `z = 0` water surface:

```text
immersed_fraction = clamp((surface_z - box_bottom_z) / box_height, 0, 1)
```

Buoyancy, body drag, angular drag, glider wing force, diagonal added mass, and
added rotational inertia are multiplied by this fraction. A fully emerged body
therefore receives neither displaced-water support nor water-relative loads and
falls back toward the water with dry-body inertia under Drake gravity. This
prevents positive-buoyancy vehicles from continuing into the sky.

The transition assumes an upright rectangular envelope. It does not integrate
oriented wetted geometry, model wave elevation, slamming, ventilation, spray,
or air drag. Surface-piercing USVs use a separate envelope that provides full
water-load authority at the nominal body-origin waterline and below, then
tapers to zero as the upright bounding box becomes fully emerged. It applies
consistently to water drag, angular drag, restoring torque, added inertia, and
propulsion.

## Diagonal effective inertia

Each vehicle already declares dry translational mass `M_D`, dry rotational
inertia `I_D`, and diagonal added terms `M_A` and `I_A`. Drake's
`MultibodyPlant` owns the dry rigid-body inertia and does not provide a Python
force element that augments its mass matrix with an arbitrary directional
fluid inertia. Feeding plant acceleration back into an added-mass force would
create an algebraic loop.

Blue Drake instead applies a zero-rate effective-inertia correction. In body
coordinates, with marine load `f_h` and gravity load `f_g`, the force sent to
the dry plant is

```text
f_plant = M_D (M_D + M_A)^-1 (f_h + f_g) - f_g.
```

The same diagonal scaling is applied to torque using `I_D` and `I_A`.
Consequently, each uncoupled translational axis at zero angular rate obeys

```text
acceleration = total_load / (dry_mass + added_mass_axis).
```

Static force balance is preserved. The approximation does not implement a
coupled six-by-six added-mass matrix, added-mass Coriolis terms, off-diagonal
products, frequency dependence, or free-surface variation. Angular scaling is
only a zero-rate approximation because the plant's dry gyroscopic terms remain.

## Glider wing force

The glider preset has a symmetric lifting-surface approximation in its body
`x-z` plane. For forward planar water-relative velocity `(u, w)`,

```text
alpha = atan2(-w, u)  # z-up convention
alpha_limited = clamp(alpha, -alpha_max, alpha_max)
C_L = lift_curve_slope * alpha_limited
q = 0.5 * water_density * (u^2 + w^2)
```

Lift is perpendicular to planar velocity. Induced drag opposes it and has
coefficient `induced_drag_factor * C_L^2`. The implementation returns zero wing
force for reverse flow and clamps lift rather than attempting a post-stall
model. It omits sideslip, separate control surfaces, wing-body interference,
Reynolds-number effects, dynamic stall, and unsteady hydrodynamics.

## Glider buoyancy and pitch controls

The glider exports a two-element command:

```text
[buoyancy_delta_N, pitch_moment_Nm]
```

Both values are symmetrically bounded and pass through independent first-order
responses. Buoyancy delta is applied in world-up, while pitch moment is applied
about body `+y`. The pitch moment is a transparent surrogate for a movable
internal mass; it does not model mass position, vehicle center-of-mass motion,
or the reaction dynamics of the mechanism. The buoyancy response does not
model pump flow, bladder volume, pressure dependence, energy, or hysteresis.

## Surface hydrostatic attitude stiffness

Surface-piercing vehicles retain the linear heave waterplane model and now add
small-angle roll and pitch moments:

```text
tau_roll = -K_roll * roll
tau_pitch = -K_pitch * pitch
```

Roll and pitch are recovered from world-up expressed in the body frame, so yaw
does not affect the restoring result. This improves basic displacement-USV
behavior near level trim but is not a metacentric, hull-section, wave, slamming,
planing, or seakeeping model.

## Validation evidence

Tests cover these invariants:

- zero added mass leaves the marine wrench unchanged,
- surge acceleration matches force divided by dry-plus-added mass,
- effective inertia preserves static neutral-buoyancy balance,
- free-surface buoyancy is full, half, and zero at the analytical box limits,
- a positive-buoyancy ROV rises toward the surface without escaping above it,
- glider wing force scales with speed squared,
- glider wing force supplies lift while induced drag removes energy,
- surface stiffness opposes positive roll and pitch,
- glider commands saturate and follow their analytic first-order response, and
- a mixed surface/subsea Drake simulation advances with all models connected.

The release-facing subset is reproducible with `blue-drake benchmark`; its
equations, numeric evidence, and claim boundary are documented in
[analytical validation evidence](validation.md).

Moving these models beyond “foundation” requires comparison against published
coefficients, tow-tank data, sea-trial logs, or another clearly identified
reference dataset.
