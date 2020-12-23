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
                 unmapped_value: Optional[Real] = None,
                 services: Services = None) -> None:
        super().__init__(None, services)
        self.agent = agent
        self.goal = goal
        self.weight_grid = None
        self.traversable_threshold = self.DEFAULT_TRAVERSABLE_THRESHOLD
        if weight_grid is not None:
            self.set_grid(weight_grid, weight_bounds, traversable_threshold, unmapped_value)

    @property
    def weight_bounds(self):
        return (0, 1)

    def set_grid(self, weight_grid: List[Any], weight_bounds: Optional[Tuple[Real, Real]] = None, traversable_threshold: Optional[Real] = None, unmapped_value: Optional[Real] = None) -> None:
        if not isinstance(weight_grid, np.ndarray):
            self.set_grid(np.array(weight_grid), weight_bounds, traversable_threshold, unmapped_value)
            return
        
        self.size = Size(*weight_grid.shape)
        self.grid = np.full(self.size, self.CLEAR_ID, dtype=np.uint8)
        self.weight_grid = np.empty(self.size, dtype=np.float32)

        # unmapped value
        if unmapped_value is not None:
            for idx in np.ndindex(*self.size):
                if weight_grid[idx] == unmapped_value:
                    self.grid[idx] = self.UNMAPPED_ID

        # bounds
        if weight_bounds is None:
            ignored = [] if unmapped_value is None else [unmapped_value]
            weight_bounds = (min(flatten(weight_grid, ignored)), max(flatten(weight_grid, ignored)))

        def normalise(x):
            y = (weight_bounds[1] - weight_bounds[0])
            return max(weight_bounds[0], min(weight_bounds[1], ((x - weight_bounds[0]) / y) if y else weight_bounds[0]))

        # threshold
        if traversable_threshold is None:
            traversable_threshold = min(weight_bounds[0] + (weight_bounds[1] - weight_bounds[0]) * self.DEFAULT_TRAVERSABLE_THRESHOLD, weight_bounds[1])
        self.traversable_threshold = normalise(traversable_threshold)

        print(weight_bounds, unmapped_value)
        print("start obstacles")
        self.obstacles.clear()
        for idx in np.ndindex(*self.size):
            v = weight_grid[idx]
            self.weight_grid[idx] = normalise(v)
            if v >= traversable_threshold and self.grid[idx] != self.UNMAPPED_ID:
                self.grid[idx] = self.WALL_ID
                self.obstacles.append(Obstacle(Point(*idx)))
        print("end obstacles")

        self.grid[self.agent.position.values] = self.AGENT_ID
        self.grid[self.goal.position.values] = self.GOAL_ID

        print("start extend walls")
        # self.extend_walls()
        print("end extend walls")

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
