# Generic grid path planning

`blue_drake.planning` provides deterministic A* search over a bounded uniform
grid. It produces geometric waypoints for experiments; it does not command a
vehicle, schedule a mission, avoid dynamic traffic, or constitute autonomy.

## Surface and subsea grids

`MarineGrid` accepts either two dimensions for surface planning or three
dimensions for volumetric subsea planning. `origin_W_m` is the minimum corner
of cell zero, `resolution_m` is uniform on every axis, and returned waypoints
are cell centers in the world frame.

```python
from blue_drake import MarineGrid, plan_grid_path

grid = MarineGrid(
    shape=(20, 12, 8),
    resolution_m=2.0,
    origin_W_m=(-20.0, -12.0, -18.0),
    blocked_indices=frozenset({(8, 5, 3), (8, 5, 4)}),
)
path = plan_grid_path(grid, (1, 1, 1), (18, 10, 6))
if path is not None:
    print(path.length_m, path.waypoints_W_m)
```

The planner uses 4-connected motion in 2D and 6-connected motion in 3D. With
uniform edge cost and Manhattan heuristic, the returned path has minimum
axis-connected length. Heap tie-breaking and neighbor order are deterministic.
`expanded_nodes` exposes a simple review diagnostic. A disconnected problem
returns `None`; blocked or out-of-bounds endpoints are configuration errors.

## Fidelity boundary

Blocked cells are supplied by the caller. Blue Drake does not yet rasterize
SceneGraph geometry, sonar returns, bathymetry, currents, modem coverage, or
vehicle turning constraints into the grid. Paths are not smoothed and do not
encode time, speed, orientation, energy, clearance, or uncertainty.

Connecting waypoints to a controller is deliberately outside this module. Any
future path follower must have its own design, stability expectations, limits,
and tests; it must not silently turn this geometric utility into mission logic.
