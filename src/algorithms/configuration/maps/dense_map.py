import copy
from typing import List, Set, Union, Dict, TYPE_CHECKING, Optional

import numpy as np

from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.entities.extended_wall import ExtendedWall
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.entities.obstacle import Obstacle
from algorithms.configuration.maps.map import Map
from simulator.services.debug import DebugLevel
from simulator.services.services import Services
from structures import Point, Size


if TYPE_CHECKING:
    from algorithms.configuration.maps.sparse_map import SparseMap


class DenseMap(Map):
    """
    This type of map is not memory efficient as it stores a grid of entities, but it
    is faster than :class:`SparseMap` as the collision detection is more efficient
    """
    grid: np.array

    # The transpose flag is set to true as a default for the initialization, as that is
    # how we are storing internally, but it will be set to false when we are simply translating from sparsemap
    # When we create more map views, we should set it to false
    def __init__(self, grid: Optional[List] = None, services: Services = None, transpose: bool = True, mutable: bool = False, name: Optional[str] = None) -> None:
        self.grid = None

        arr_grid = None
        if grid is not None:
            arr_grid = np.atleast_2d(np.array(grid))
            if arr_grid.dtype == object:
                raise ValueError("Cannot create DenseMap grid from ragged nested sequences (which is a list-or-tuple of lists-or-tuples-or ndarrays with different lengths or shapes)")

            super().__init__(Size(*([0]*arr_grid.ndim)), services, mutable, name)
        else:
            super().__init__(services=services, mutable=mutable, name=name)
            return

        # Doesn't work with non-uniform grids
        self.set_grid(arr_grid, transpose)

    def set_grid(self, grid: np.array, transpose) -> None:
        # We transpose here to not worry about flipping coordinates later on
        # Please take care I'm not sure why everythng works but it does atm
        self.grid = np.transpose(grid) if transpose else grid

        self.size = Size(*self.grid.shape)
        for index in np.ndindex(*self.size):
            val: int = self.grid[index]
            if val == self.AGENT_ID:
                self.agent = Agent(Point(*index))
            elif val == self.GOAL_ID:
                self.goal = Goal(Point(*index))
            elif val == self.WALL_ID:
                self.obstacles.append(Obstacle(Point(*index)))
            elif val == self.EXTENDED_WALL_ID:
                self.obstacles.append(ExtendedWall(Point(*index)))

    def extend_walls(self) -> None:
        """
        Method for extending the walls by agent radius
        """

        def extend_obstacle_bound() -> None:
            for b in bounds:
                for index in np.ndindex(*(len(self.size) * [self.agent.radius*2 + 1])):
                    p = [elem + b[i] - self.agent.radius for i, elem in enumerate(index)]
                    pt = Point(*p)
                    if not self.is_out_of_bounds_pos(pt):
                        dist: Union[float, np.ndarray] = np.linalg.norm(np.array(p) - np.array(b))
                        if dist <= self.agent.radius and self.at(pt) == DenseMap.CLEAR_ID:
                            self.grid[pt.values] = DenseMap.EXTENDED_WALL_ID
                            self.obstacles.append(ExtendedWall(pt))
                            visited.add(pt)

        visited: Set[Point] = set()

        for index in np.ndindex(*self.size):
            pt = Point(*index)
            if (pt not in visited) and (self.grid[index] == self.WALL_ID):
                bounds: Set[Point] = self.get_obstacle_bound(pt, visited)
                extend_obstacle_bound()

    def at(self, p: Point) -> int:
        assert not p.is_float, f"Can't look up a floating point Point in dense_map: {p}"
        return self.grid[p.values]

    def update_point(self, p: Point, val: int) -> bool:
        if self.at(p) == val:
            return False
        else:
            self.grid[p.values] = val
            return True

    def move(self, entity: Entity, to: Point, no_trace: bool = False) -> bool:
        """
        Read super description
        """
        super().move(entity, to, no_trace)

        if isinstance(entity, Agent):
            if not self.is_agent_valid_pos(to):
                return False

            prev_pos = self.agent.position
            self.agent.position = to
        elif isinstance(entity, Goal):
            prev_pos = self.goal.position
            self.grid[self.goal.position.values] = self.CLEAR_ID
            self.goal.position = to
        else:
            raise NotImplementedError()

        self.grid[prev_pos.values] = self.CLEAR_ID
        self.grid[self.goal.position.values] = self.GOAL_ID
        self.grid[self.agent.position.values] = self.AGENT_ID

        for i in range(len(self.obstacles)):
            if self.obstacles[i].position == self.goal.position:
                del self.obstacles[i]
                break
        return True

    def convert_to_sparse_map(self) -> Optional['SparseMap']:
        """
        Method for converting current map into a :class:`SparseMap`
        :return: The converted map
        """
        from algorithms.configuration.maps.sparse_map import SparseMap
        obstacles: List[Obstacle] = list(filter(lambda o: not isinstance(o, ExtendedWall), self.obstacles))
        sparse_map: SparseMap = SparseMap(self.size, copy.deepcopy(self.agent),
                                          obstacles, copy.deepcopy(self.goal), self.services)
        sparse_map.trace = copy.deepcopy(self.trace)
        return sparse_map

    def is_agent_valid_pos(self, pos: Point) -> bool:
        """
        Read super description
        """
        if not super().is_agent_valid_pos(pos):
            return False

        return self.at(pos) in (self.CLEAR_ID, self.AGENT_ID, self.GOAL_ID)

    def __repr__(self) -> str:
        debug_level: DebugLevel = DebugLevel.BASIC
        if self.services is not None:
            debug_level = self.services.settings.simulator_write_debug_level
        # flipping it back for the str repr
        new_grid = np.transpose(self.grid)

        res: str = "DenseMap: {\n\t\tsize: " + str(self.size) + \
                   ", \n\t\tagent: " + str(self.agent) + \
                   ", \n\t\tgoal: " + str(self.goal) + \
                   ", \n\t\tobstacles: " + str(len(self.obstacles))
        if self.size.n_dim == 2 and (debug_level == DebugLevel.HIGH or (self.size.width <= 20 and self.size.height <= 20)):
            res += ", \n\t\tgrid: [\n"
            for i in range(len(new_grid)):
                res += "\t\t\t"
                for j in range(len(new_grid[i])):
                    res += str(new_grid[i][j]) + ", "
                res += "\n"
            res += "\t\t]"
        elif self.size.n_dim == 3 and (debug_level == DebugLevel.HIGH or (self.size.width <= 5 and self.size.height <= 5 and self.size.depth <= 5)):
            res += ", \n\t\tgrid: [\n"
            for i in range(len(new_grid)):
                for j in range(len(new_grid[i])):
                    res += "\t\t\t"
                    for k in range(len(new_grid[i][j])):
                        res += str(new_grid[i][j][k]) + ", "
                    res += "\n"
                res += "\n"
            res = res[:-1] + "\t\t]"
        else:
            res += f", grid: Size({', '.join(str(i) for i in new_grid.shape)})"
        return res + "\n\t}"

    def __copy__(self) -> 'DenseMap':
        return copy.deepcopy(self)

    def __deepcopy__(self, memo: Dict) -> 'DenseMap':
        dense_map = self.__class__(copy.deepcopy(self.grid), services=self.services, transpose=False)
        dense_map.name = copy.deepcopy(self.name)
        dense_map.trace = copy.deepcopy(self.trace)
        dense_map.agent = copy.deepcopy(self.agent)
        dense_map.goal = copy.deepcopy(self.goal)
        dense_map.obstacles = copy.deepcopy(self.obstacles)
        return dense_map

    def __eq__(self, other: object) -> bool:
        from algorithms.configuration.maps.sparse_map import SparseMap
        if isinstance(other, SparseMap):
            other: DenseMap = other.convert_to_dense_map()
        if not isinstance(other, DenseMap):
            return False
        return np.array_equal(self.grid, other.grid) and super().__eq__(other)
