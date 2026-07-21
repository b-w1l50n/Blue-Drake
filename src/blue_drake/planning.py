"""Deterministic generic grid path planning without vehicle execution logic."""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

GridIndex = tuple[int, ...]
WorldPoint = tuple[float, ...]


def _index(name: str, value, dimension: int) -> GridIndex:
    try:
        result = tuple(value)
    except TypeError as exc:
        raise ValueError(f"{name} must contain {dimension} integers") from exc
    if len(result) != dimension or any(
        isinstance(item, bool) or not isinstance(item, int) for item in result
    ):
        raise ValueError(f"{name} must contain {dimension} integers")
    return result


@dataclass(frozen=True)
class MarineGrid:
    """A bounded 2D or 3D uniform grid with occupied cells.

    ``origin_W_m`` is the minimum corner of cell zero. Returned waypoints are
    cell centers. The grid is a planning abstraction and has no simulator-side
    collision or vehicle-control behavior.
    """

    shape: tuple[int, ...]
    resolution_m: float
    origin_W_m: WorldPoint
    blocked_indices: frozenset[GridIndex] = frozenset()

    def __post_init__(self) -> None:
        try:
            shape = tuple(self.shape)
        except TypeError as exc:
            raise ValueError("marine grid dimension must be 2 or 3") from exc
        if len(shape) not in {2, 3}:
            raise ValueError("marine grid dimension must be 2 or 3")
        if any(
            isinstance(item, bool) or not isinstance(item, int) or item <= 0
            for item in shape
        ):
            raise ValueError("marine grid shape must contain positive integers")
        if self.resolution_m <= 0.0 or not math.isfinite(self.resolution_m):
            raise ValueError("resolution_m must be positive and finite")
        try:
            origin = tuple(float(item) for item in self.origin_W_m)
        except (TypeError, ValueError) as exc:
            raise ValueError("origin_W_m must contain finite values") from exc
        if len(origin) != len(shape) or not all(
            math.isfinite(item) for item in origin
        ):
            raise ValueError(
                f"origin_W_m must contain {len(shape)} finite values"
            )
        blocked = frozenset(
            _index("blocked index", item, len(shape))
            for item in self.blocked_indices
        )
        object.__setattr__(self, "shape", shape)
        object.__setattr__(self, "origin_W_m", origin)
        object.__setattr__(self, "blocked_indices", blocked)
        for item in blocked:
            if not self.contains(item):
                raise ValueError(f"blocked index is outside grid: {item}")

    @property
    def dimension(self) -> int:
        """Return 2 for surface grids and 3 for volumetric grids."""

        return len(self.shape)

    def contains(self, index: GridIndex) -> bool:
        """Return whether an index is inside the bounded grid."""

        return (
            isinstance(index, tuple)
            and len(index) == self.dimension
            and all(
                isinstance(item, int) and not isinstance(item, bool)
                for item in index
            )
            and all(
                0 <= item < extent
                for item, extent in zip(index, self.shape, strict=True)
            )
        )

    def is_free(self, index: GridIndex) -> bool:
        """Return whether an in-bounds cell is not occupied."""

        return self.contains(index) and index not in self.blocked_indices

    def cell_center_W_m(self, index: GridIndex) -> WorldPoint:
        """Convert an in-bounds cell index to its world-frame center."""

        index = _index("index", index, self.dimension)
        if not self.contains(index):
            raise ValueError(f"index is outside grid: {index}")
        return tuple(
            origin + (item + 0.5) * self.resolution_m
            for origin, item in zip(self.origin_W_m, index, strict=True)
        )

    def index_for_position_W_m(self, position_W_m) -> GridIndex:
        """Map a world point to its containing grid cell."""

        try:
            position = tuple(float(item) for item in position_W_m)
        except (TypeError, ValueError) as exc:
            raise ValueError("position_W_m must contain finite values") from exc
        if len(position) != self.dimension or not all(
            math.isfinite(item) for item in position
        ):
            raise ValueError(
                f"position_W_m must contain {self.dimension} finite values"
            )
        index = tuple(
            math.floor((item - origin) / self.resolution_m)
            for item, origin in zip(position, self.origin_W_m, strict=True)
        )
        if not self.contains(index):
            raise ValueError(f"position is outside grid: {position}")
        return index

    def free_axis_neighbors(self, index: GridIndex) -> tuple[GridIndex, ...]:
        """Return deterministic 4-connected or 6-connected free neighbors."""

        index = _index("index", index, self.dimension)
        if not self.contains(index):
            raise ValueError(f"index is outside grid: {index}")
        neighbors = []
        for axis in range(self.dimension):
            for direction in (-1, 1):
                candidate = list(index)
                candidate[axis] += direction
                neighbor = tuple(candidate)
                if self.is_free(neighbor):
                    neighbors.append(neighbor)
        return tuple(neighbors)


@dataclass(frozen=True)
class GridPath:
    """An optimal axis-connected grid path and reviewable diagnostics."""

    indices: tuple[GridIndex, ...]
    waypoints_W_m: tuple[WorldPoint, ...]
    length_m: float
    expanded_nodes: int


def _manhattan(first: GridIndex, second: GridIndex) -> int:
    return sum(abs(a - b) for a, b in zip(first, second, strict=True))


def plan_grid_path(
    grid: MarineGrid,
    start: GridIndex,
    goal: GridIndex,
) -> GridPath | None:
    """Return an optimal deterministic A* path or ``None`` when disconnected."""

    start = _index("start", start, grid.dimension)
    goal = _index("goal", goal, grid.dimension)
    if not grid.contains(start) or not grid.contains(goal):
        raise ValueError("start and goal must be inside the grid")
    if not grid.is_free(start) or not grid.is_free(goal):
        raise ValueError("start and goal must be free cells")

    frontier = [(float(_manhattan(start, goal)), 0.0, start)]
    predecessor: dict[GridIndex, GridIndex] = {}
    costs = {start: 0.0}
    closed: set[GridIndex] = set()
    expanded_nodes = 0
    while frontier:
        _, cost, current = heapq.heappop(frontier)
        if current in closed or cost != costs[current]:
            continue
        closed.add(current)
        expanded_nodes += 1
        if current == goal:
            indices = [goal]
            while indices[-1] != start:
                indices.append(predecessor[indices[-1]])
            indices.reverse()
            path_indices = tuple(indices)
            return GridPath(
                indices=path_indices,
                waypoints_W_m=tuple(
                    grid.cell_center_W_m(item) for item in path_indices
                ),
                length_m=(len(path_indices) - 1) * grid.resolution_m,
                expanded_nodes=expanded_nodes,
            )
        for neighbor in grid.free_axis_neighbors(current):
            new_cost = cost + 1.0
            if new_cost >= costs.get(neighbor, math.inf):
                continue
            costs[neighbor] = new_cost
            predecessor[neighbor] = current
            priority = new_cost + _manhattan(neighbor, goal)
            heapq.heappush(frontier, (priority, new_cost, neighbor))
    return None
