import numpy as np

from algorithms.configuration.maps.map import Map
from algorithms.configuration.maps.dense_map import DenseMap
from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.entities.obstacle import Obstacle

from structures import Point, Size

class OccupancyGridMap(DenseMap):
    """
    For now, this map is simply DenseMap representation with floats
    representing the obstacles.
    """
    IMPASSABLE_THRESHOLD: float = 0.99
    _weight_grid: np.array

    @property
    def weight_grid(self) -> np.array:
        return self._weight_grid

    def set_weight_grid(self, weight_grid: np.array) -> None:
        self._weight_grid = np.array(weight_grid, copy=True)
        self.grid = np.where((self._weight_grid >= OccupancyGridMap.IMPASSABLE_THRESHOLD) & (self._weight_grid <= 1),
                             self.WALL_ID, self.CLEAR_ID)
        self.size = Size(*self.grid.shape)

        self.obstacles.clear()
        for index in np.ndindex(*self.size):
            val: float = self.grid[index]
            if val == self.AGENT_ID:
                self.agent = Agent(Point(*index))
            elif val == self.GOAL_ID:
                self.goal = Goal(Point(*index))
            elif val == self.WALL_ID:
                self.obstacles.append(Obstacle(Point(*index)))

        self.grid[self.agent.position.values] = self.AGENT_ID
        self.grid[self.goal.position.values] = self.GOAL_ID
        self.extend_walls()

    def at(self, p: Point) -> int:
        return self.grid[p.values]

    def __repr__(self) -> str:
        return "Occupancy grid map: " + super().__repr__()

    def extend_walls(self) -> None:
        super().extend_walls()
        # TODO: extend walls in weight grid

    def convert_to_sparse_map(self) -> None:
        raise NotImplementedError()
