import numpy as np

from algorithms.configuration.maps.map import Map
from algorithms.configuration.maps.dense_map import DenseMap
from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.goal import Goal

from structures import Point, Size

class OccupancyGridMap(DenseMap):
    """
    For now, this map is simply DenseMap representation with floats
    representing the obstacles.
    """
    IMPASSABLE_THRESHOLD: float = 0.99
    weight_grid: np.array

    # Assumes that floats passed into the grid between 0 and 1 are obstacles (movement to be decided later)
    def set_grid(self, grid: np.array, transpose) -> None:
        self.grid = np.transpose(grid) if transpose else grid
        self.weight_grid = np.array(self.grid, copy=True)
        self.grid = np.where((self.weight_grid >= OccupancyGridMap.IMPASSABLE_THRESHOLD) & (self.weight_grid <= 1),
                             self.WALL_ID, self.weight_grid)

        self.size = Size(*self.grid.shape)

        for index in np.ndindex(*self.size):
            val: float = self.grid[index]
            if val == self.AGENT_ID:
                self.agent = Agent(Point(*index))
            elif val == self.GOAL_ID:
                self.goal = Goal(Point(*index))

    def at(self, p: Point) -> int:
        return self.grid[p.pos]

    def __repr__(self) -> str:
        return "Occupancy grid map: " + super().__repr__()

    def extend_walls(self, extend_weight_grid: bool = False) -> None:
        super().extend_walls()

        if not extend_weight_grid:
            return

        # TODO: to implement, we want to take the max over each area considered in the loop
        raise NotImplementedError()

        """def extend_obstacle_bound() -> None:
            for b in bounds:
                for index in np.ndindex(*(len(self.size) * [self.agent.radius*2 + 1])):
                    # We want the max of all these indeces
                    p = [elem + b[i] - self.agent.radius for i, elem in enumerate(index)]
                    if not self.is_out_of_bounds_pos(Point(*p)):
                        dist: Union[float, np.ndarray] = np.linalg.norm(np.array(p) - np.array(b))
                        if dist <= self.agent.radius and self.at(Point(*p)) == DenseMap.CLEAR_ID:
                            self.grid[tuple(p)] = DenseMap.EXTENDED_WALL
                            self.obstacles.append(ExtendedWall(Point(*p)))
                            visited.add(Point(*p))


        visited: Set[Point] = set()

        for index in np.ndindex(*self.size):
            if (Point(*index) not in visited) and (self.grid[index] == self.WALL_ID):
                bounds: Set[Point] = self.get_obstacle_bound(Point(*index), visited)
                extend_obstacle_bound()"""

    def convert_to_sparse_map(self) -> None:
        raise NotImplementedError()
