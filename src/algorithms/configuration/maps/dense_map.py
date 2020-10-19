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

    CLEAR_ID: int = 0
    WALL_ID: int = 1
    AGENT_ID: int = 2
    GOAL_ID: int = 3
    EXTENDED_WALL: int = 4

    def __init__(self, grid: Optional[List], services: Services = None) -> None:
        self.grid = None
        #default to creating a size 0 2D map
        super().__init__(Size(0, 0), services)

        if grid is None:
            return

        # Doesn't work with non-uniform grids
        self.set_grid(np.array(grid))

    def set_grid(self, grid: np.array) -> None:

        #We transpose here to not worry about flipping coordinates later on
        self.grid = np.atleast_2d(np.transpose(grid))

        self.size = Size(*self.grid.shape)
        for index in np.ndindex(*self.size):
            val: int = self.grid[index]
            if val == self.AGENT_ID:
                self.agent = Agent(Point(*index))
            elif val == self.GOAL_ID:
                self.goal = Goal(Point(*index))
            elif val == self.WALL_ID:
                self.obstacles.append(Obstacle(Point(*index)))
            elif val == self.EXTENDED_WALL:
                self.obstacles.append(ExtendedWall(Point(*index)))

    def extend_walls(self) -> None:
        """
        Method for extending the walls by agent radius
        """

        #def extend_obstacle_bound() -> None:
        #    for b in bounds:
        #        for x in range(b.x - self.agent.radius, b.x + self.agent.radius + 1):
        #            for y in range(b.y - self.agent.radius, b.y + self.agent.radius + 1):
        #                if not self.is_out_of_bounds_pos(Point(x, y)):
        #                    dist: Union[float, np.ndarray] = np.linalg.norm(np.array([x, y]) - np.array(b))
        #                    if dist <= self.agent.radius and self.grid[y][x] == DenseMap.CLEAR_ID:
        #                        self.grid[y][x] = DenseMap.EXTENDED_WALL
        #                        self.obstacles.append(ExtendedWall(Point(x, y)))
        #                        visited.add(Point(x, y))

        def extend_obstacle_bound() -> None:
            for b in bounds:
                for x in range(b.x - self.agent.radius, b.x + self.agent.radius + 1):
                    for y in range(b.y - self.agent.radius, b.y + self.agent.radius + 1):
                        if not self.is_out_of_bounds_pos(Point(x, y)):
                            dist: Union[float, np.ndarray] = np.linalg.norm(np.array([x, y]) - np.array(b))
                            if dist <= self.agent.radius and self.grid[x][y] == DenseMap.CLEAR_ID:
                                self.grid[x][y] = DenseMap.EXTENDED_WALL
                                self.obstacles.append(ExtendedWall(Point(x, y)))
                                visited.add(Point(x, y))

        visited: Set[Point] = set()
        #visited: List[List[bool]] = [[False for _ in range(len(self.grid[i]))] for i in range(len(self.grid))]

        for index in np.ndindex(*self.size):
            if (Point(index) not in visited) and (self.grid[index] == self.WALL_ID):
                bounds: Set[Point] = self.get_obstacle_bound(Point(index), visited)
                extend_obstacle_bound()
        
        #for i in range(self.grid_dim[0]):
        #    for j in range(self.grid_dim[1]):
        #        #Is there a better way to do this without constructing a point every time
        #        if Point(j, i) not in visited:
        #            if self.grid[i][j] == self.WALL_ID:
        #                bounds: Set[Point] = self.get_obstacle_bound(Point(j, i), visited)
        #                extend_obstacle_bound()

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
            self.grid[self.goal.position.pos] = self.CLEAR_ID
            self.goal.position = to
        else:
            raise NotImplementedError()

        self.grid[prev_pos.pos] = self.CLEAR_ID
        self.grid[self.goal.position.pos] = self.GOAL_ID
        self.grid[self.agent.position.pos] = self.AGENT_ID

        for i in range(len(self.obstacles)):
            if self.obstacles[i].position == self.goal.position:
                del self.obstacles[i]
                break

        return True

    def convert_to_sparse_map(self) -> 'SparseMap':
        """
        Method for converting current map into a :class:`SparseMap`
        :return: The converted map
        """
        from algorithms.configuration.maps.sparse_map import SparseMap
        obstacles: List[Obstacle] = list(filter(lambda o: not isinstance(o, ExtendedWall), self.obstacles))
        sparse_map: SparseMap = SparseMap(self.size, copy.deepcopy(self.agent),
                                          obstacles, copy.deepcopy(self.goal), self._services)
        sparse_map.trace = copy.deepcopy(self.trace)
        return sparse_map

    def is_agent_valid_pos(self, pos: Point) -> bool:
        """
        Read super description
        """
        if not super().is_agent_valid_pos(pos):
            return False

        return self.grid[pos.x][pos.y] != self.WALL_ID and self.grid[pos.x][pos.y] != self.EXTENDED_WALL

    def __str__(self) -> str:
        debug_level: DebugLevel = DebugLevel.BASIC
        if self._services is not None:
            debug_level = self._services.settings.simulator_write_debug_level
        #flipping it back for the str repr
        new_grid = np.transpose(self.grid)
        
        res: str = "DenseMap: {\n\t\tsize: " + str(self.size) + \
                   ", \n\t\tagent: " + str(self.agent) + \
                   ", \n\t\tgoal: " + str(self.goal) + \
                   ", \n\t\tobstacles: " + str(len(self.obstacles))
        if debug_level == DebugLevel.HIGH or (self.size.width <= 20 and self.size.height <= 20):
            res += ", \n\t\tgrid: [\n"
            for i in range(len(new_grid)):
                res += "\t\t\t"
                for j in range(len(new_grid[i])):
                    res += str(new_grid[i][j]) + ", "
                res += "\n"
            res += "\t\t]"
        return res + "\n\t}"

    def __copy__(self) -> 'DenseMap':
        return copy.deepcopy(self)

    def __deepcopy__(self, memo: Dict) -> 'DenseMap':
        dense_map = DenseMap(copy.deepcopy(self.grid))
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
