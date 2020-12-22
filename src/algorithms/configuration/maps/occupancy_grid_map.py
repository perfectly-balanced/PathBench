import numpy as np

from typing import Dict, List, Any, Optional, Tuple
from numbers import Real
import copy

from simulator.services.services import Services

from typing import Dict, List, Any, Optional, Tuple
from numbers import Real
import copy

from simulator.services.services import Services
from algorithms.configuration.maps.map import Map
from algorithms.configuration.maps.dense_map import DenseMap
from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.entities.obstacle import Obstacle
from utility.misc import flatten, array_shape
from utility.compatibility import Final
from structures import Point, Size

class OccupancyGridMap(DenseMap):
    weight_grid: np.array
    traversable_threshold: float
    DEFAULT_TRAVERSABLE_THRESHOLD: Final[float] = 0.95

    def __init__(self,
                 weight_grid: Optional[List[Any]] = None,
                 agent: Optional[Agent] = None,
                 goal: Optional[Goal] = None,
                 weight_bounds: Optional[Tuple[Real, Real]] = None,
                 traversable_threshold: Optional[Real] = None,
                 services: Services = None) -> None:
        super().__init__(None, services)
        self.agent = agent
        self.goal = goal
        self.weight_grid = None
        self.traversable_threshold = self.DEFAULT_TRAVERSABLE_THRESHOLD
        if weight_grid is not None:
            self.set_grid(weight_grid, weight_bounds)

    @property
    def weight_bounds(self):
        return (0, 1)

    def set_grid(self, weight_grid: List[Any], weight_bounds: Optional[Tuple[Real, Real]] = None, traversable_threshold: Optional[Real] = None) -> None:
        self.size = Size(*array_shape(weight_grid))
        self.grid = np.empty(self.size, dtype=np.uint8)
        self.weight_grid = np.empty(self.size, dtype=np.float32)

        # bounds
        if weight_bounds is None:
            weight_bounds = (min(flatten(weight_grid)), max(flatten(weight_grid)))

        def normalise(x): return (x - weight_bounds[0]) / (weight_bounds[1] - weight_bounds[0])

        # threshold
        if traversable_threshold is None:
            traversable_threshold = min(weight_bounds[0] + (weight_bounds[1] - weight_bounds[0]) * self.DEFAULT_TRAVERSABLE_THRESHOLD, weight_bounds[1])
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
        self.extend_walls()

    def at(self, p: Point) -> int:
        return self.grid[p.values]

    def __repr__(self) -> str:
        return "Occupancy Grid: " + super().__repr__()

    def convert_to_sparse_map(self) -> None:
        raise NotImplementedError()

    def __deepcopy__(self, memo: Dict) -> 'OccupancyGridMap':
        mp = self.__class__(copy.deepcopy(self.weight_grid),
                            copy.deepcopy(self.agent), copy.deepcopy(self.goal),
                            traversable_threshold=self.traversable_threshold,
                            services=self._services)
        mp.trace = copy.deepcopy(self.trace)
        mp.obstacles = copy.deepcopy(self.obstacles)
        mp.grid = copy.deepcopy(self.grid)
        return mp
