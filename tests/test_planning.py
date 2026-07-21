from __future__ import annotations

import pytest

from blue_drake.planning import MarineGrid, plan_grid_path


def test_two_dimensional_astar_finds_optimal_obstacle_detour() -> None:
    grid = MarineGrid(
        shape=(5, 3),
        resolution_m=2.0,
        origin_W_m=(-5.0, -3.0),
        blocked_indices=frozenset({(2, 1)}),
    )
    path = plan_grid_path(grid, (0, 1), (4, 1))
    assert path is not None
    assert path.indices[0] == (0, 1)
    assert path.indices[-1] == (4, 1)
    assert len(path.indices) == 7
    assert path.length_m == 12.0
    assert (2, 1) not in path.indices
    assert path.waypoints_W_m[0] == pytest.approx((-4.0, 0.0))


def test_three_dimensional_astar_routes_around_blocked_depth_cell() -> None:
    grid = MarineGrid(
        shape=(2, 2, 3),
        resolution_m=1.0,
        origin_W_m=(0.0, 0.0, -3.0),
        blocked_indices=frozenset({(0, 0, 1)}),
    )
    path = plan_grid_path(grid, (0, 0, 0), (0, 0, 2))
    assert path is not None
    assert len(path.indices) == 5
    assert path.length_m == 4.0
    assert all(len(point) == 3 for point in path.waypoints_W_m)


def test_astar_returns_none_for_disconnected_free_cells() -> None:
    grid = MarineGrid(
        shape=(3, 3),
        resolution_m=1.0,
        origin_W_m=(0.0, 0.0),
        blocked_indices=frozenset({(1, 0), (1, 1), (1, 2)}),
    )
    assert plan_grid_path(grid, (0, 1), (2, 1)) is None


def test_grid_coordinate_round_trip_uses_cell_centers() -> None:
    grid = MarineGrid(shape=(3, 2), resolution_m=0.5, origin_W_m=(-1.0, 2.0))
    center = grid.cell_center_W_m((2, 1))
    assert center == pytest.approx((0.25, 2.75))
    assert grid.index_for_position_W_m(center) == (2, 1)


def test_planner_rejects_blocked_or_out_of_bounds_endpoints() -> None:
    grid = MarineGrid(
        shape=(2, 2),
        resolution_m=1.0,
        origin_W_m=(0.0, 0.0),
        blocked_indices=frozenset({(1, 1)}),
    )
    with pytest.raises(ValueError, match="free cells"):
        plan_grid_path(grid, (0, 0), (1, 1))
    with pytest.raises(ValueError, match="inside the grid"):
        plan_grid_path(grid, (0, 0), (2, 0))


def test_astar_tie_breaking_is_deterministic() -> None:
    grid = MarineGrid(shape=(3, 3), resolution_m=1.0, origin_W_m=(0.0, 0.0))
    first = plan_grid_path(grid, (0, 0), (2, 2))
    second = plan_grid_path(grid, (0, 0), (2, 2))
    assert first == second


def test_start_equal_to_goal_returns_zero_length_path() -> None:
    grid = MarineGrid(shape=(2, 2), resolution_m=1.0, origin_W_m=(0.0, 0.0))
    path = plan_grid_path(grid, (1, 1), (1, 1))
    assert path is not None
    assert path.indices == ((1, 1),)
    assert path.length_m == 0.0
    assert path.expanded_nodes == 1


def test_grid_rejects_invalid_dimension_and_blocked_index() -> None:
    with pytest.raises(ValueError, match="dimension must be 2 or 3"):
        MarineGrid(
            shape=(2, 2, 2, 2),
            resolution_m=1.0,
            origin_W_m=(0.0, 0.0, 0.0, 0.0),
        )
    with pytest.raises(ValueError, match="blocked index is outside"):
        MarineGrid(
            shape=(2, 2),
            resolution_m=1.0,
            origin_W_m=(0.0, 0.0),
            blocked_indices=frozenset({(2, 0)}),
        )
