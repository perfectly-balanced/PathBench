import numpy as np
import copy

from typing import Dict, List, Any, Optional, Tuple
from numbers import Real

from simulator.services.services import Services

from algorithms.configuration.maps.map import Map
from algorithms.configuration.maps.dense_map import DenseMap
from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.entities.obstacle import Obstacle

from structures import Point, Size

from collections.abc import Iterable

def flatten(l):
    for el in l:
        if isinstance(el, Iterable) and not isinstance(el, (str, bytes)):
            yield from flatten(el)
        else:
            yield el

class OccupancyGridMap(DenseMap):
    """
    For now, this map is simply DenseMap representation with floats
    representing the obstacles.
    """
    weight_grid: np.array
    traversable_threshold: float

    def __init__(self, weight_grid: List[Any], agent: Agent, goal: Goal, weight_bounds: Optional[Tuple[Real, Real]] = None, traversable_threshold: Optional[Real] = None, services: Services = None) -> None:
        super().__init__(None, services)
        self.agent = agent
        self.goal = goal
        self.set_grid(weight_grid, weight_bounds)

    @property
    def weight_bounds(self):
        return (0, 1)

    @staticmethod
    def __get_array_size(a):
        if isinstance(a[0], Iterable):
            return Size(len(a), *OccupancyGridMap.__get_array_size(a[0]))
        else:
            return (len(a),)

    def set_grid(self, weight_grid: List[Any], weight_bounds: Optional[Tuple[Real, Real]] = None, traversable_threshold: Optional[Real] = None) -> None:
        self.size = self.__get_array_size(weight_grid)
        self.grid = np.empty(self.size, dtype=np.uint8)
        self.weight_grid = np.empty(self.size, dtype=np.float32)

        # bounds
        if weight_bounds is None:
            weight_bounds = (min(flatten(weight_grid)), max(flatten(weight_grid)))

        def normalise(x): return (x - weight_bounds[0]) / (weight_bounds[1] - weight_bounds[0])

        # threshold
        if traversable_threshold is None:
            traversable_threshold = min(weight_bounds[0] + (weight_bounds[1] - weight_bounds[0]) * 0.95, weight_bounds[1])
        self.traversable_threshold = normalise(traversable_threshold)

        self.obstacles.clear()
        for idx in np.ndindex(*self.size):
            v = weight_grid[idx[0]]
            for i in range(1, self.size.n_dim):
                v = v[idx[i]]

            self.weight_grid[idx] = normalise(v)
            if v >= traversable_threshold:
                self.grid[idx] = self.WALL_ID
                self.obstacles.append(Obstacle(Point(*idx)))
            else:
                self.grid[idx] = self.CLEAR_ID

        self.grid[self.agent.position.values] = self.AGENT_ID
        self.grid[self.goal.position.values] = self.GOAL_ID

    def at(self, p: Point) -> int:
        return self.grid[p.values]

    def __repr__(self) -> str:
        return "Occupancy Grid: " + super().__repr__()

    def extend_walls(self) -> None:
        super().extend_walls()
        # TODO: extend walls in weight grid

    def convert_to_sparse_map(self) -> None:
        raise NotImplementedError()

    def __deepcopy__(self, memo: Dict) -> 'OccupancyGridMap':
        dense_map = self.__class__(copy.deepcopy(self.weight_grid), copy.deepcopy(self.agent), copy.deepcopy(self.goal))
        dense_map.trace = copy.deepcopy(self.trace)
        dense_map.obstacles = copy.deepcopy(self.obstacles)
        return dense_map
