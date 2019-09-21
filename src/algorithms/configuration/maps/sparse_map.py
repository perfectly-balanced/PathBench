import copy
from typing import List, Union, Dict

import numpy as np

from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.entities.obstacle import Obstacle
from algorithms.configuration.maps.dense_map import DenseMap
from algorithms.configuration.maps.map import Map
from simulator.services.debug import DebugLevel
from simulator.services.services import Services
from structures import Point, Size


class SparseMap(Map):
    """
    This type of map keeps a sparse configuration of the entities, thus being memory efficient. 
    The downside is that the collision system is quite bad and takes a long time.
    """

    DIST_TOLERANCE = 0.0001

    def __init__(self, size: Size, agent: Agent, obstacles: List[Obstacle], goal: Goal, services: Services = None) \
            -> None:
        super().__init__(size, services)
        self.agent = agent
        self.obstacles = obstacles
        self.goal = goal

    def move(self, entity: Entity, to: Point, no_trace: bool = False) -> bool:
        """
        Read super description
        """
        super().move(entity, to, no_trace)


        if isinstance(entity, Agent):
            if not self.is_agent_valid_pos(to):
                return False
            self.agent.position = to
        elif isinstance(entity, Goal):
            self.goal.position = to
        else:
            raise NotImplementedError()

        return True

    def convert_to_dense_map(self) -> DenseMap:
        """
        Converts current map into a :class:`DenseMap`
        :return: The converted map
        """
        grid: List[List[int]] = [[0 for _ in range(self.size.width)] for _ in range(self.size.height)]
        should_optimize: bool = len(self.obstacles) > 20

        for obstacle in self.obstacles:
            total_radius: int = self.agent.radius + obstacle.radius
            if should_optimize:
                total_radius = obstacle.radius
            for x in range(obstacle.position.x - total_radius, obstacle.position.x + total_radius + 1):
                for y in range(obstacle.position.y - total_radius, obstacle.position.y + total_radius + 1):
                    if not self.is_out_of_bounds_pos(Point(x, y)):
                        dist: Union[float, np.ndarray] = np.linalg.norm(np.array([x, y]) - np.array(obstacle.position))
                        if not should_optimize and \
                                dist <= obstacle.radius + self.agent.radius and grid[y][x] == DenseMap.CLEAR_ID:
                            grid[y][x] = DenseMap.EXTENDED_WALL
                        if dist <= obstacle.radius:
                            grid[y][x] = DenseMap.WALL_ID

        grid[self.agent.position.y][self.agent.position.x] = DenseMap.AGENT_ID
        grid[self.goal.position.y][self.goal.position.x] = DenseMap.GOAL_ID
        dense_map: DenseMap = DenseMap(grid, self._services)
        dense_map.agent = copy.deepcopy(self.agent)
        dense_map.goal = copy.deepcopy(self.goal)
        dense_map.trace = copy.deepcopy(self.trace)
        if should_optimize:
            dense_map.extend_walls()
        return dense_map

    def is_agent_valid_pos(self, pos: Point) -> bool:
        """
        Read super description
        """
        if not super().is_agent_valid_pos(pos):
            return False
        for obstacle in self.obstacles:
            dist: Union[float, np.ndarray] = np.linalg.norm(np.array(pos) - np.array(obstacle.position))
            if dist <= (obstacle.radius + self.agent.radius) or dist < self.DIST_TOLERANCE:
                return False
        return True

    def __str__(self) -> str:
        debug_level: DebugLevel = DebugLevel.BASIC
        if self._services is not None:
            debug_level = self._services.settings.simulator_write_debug_level

        obst_str: str = "{\n\t\t\tsize: " + str(len(self.obstacles))
        if debug_level == DebugLevel.HIGH or len(self.obstacles) <= 10:
            obst_str += ", \n\t\t\tentities: [\n"
            for obst in self.obstacles:
                obst_str += "\t\t\t\t" + str(obst) + ", \n"
            obst_str += "\t\t\t]"
        obst_str += "\n\t\t}"

        return "SparseMap: {\n\t\tsize: " + str(self.size) + ", \n\t\tagent: " + str(self.agent) + \
               ", \n\t\tobstacles: " + obst_str + ", \n\t\tgoal: " + str(self.goal) + "\n\t}"

    def __copy__(self) -> 'SparseMap':
        return copy.deepcopy(self)

    def __deepcopy__(self, memo: Dict) -> 'SparseMap':
        dense_map = SparseMap(self.size, copy.deepcopy(self.agent),
                              copy.deepcopy(self.obstacles), copy.deepcopy(self.goal), self._services)
        dense_map.trace = copy.deepcopy(self.trace)
        return dense_map

    def __eq__(self, other: object) -> bool:
        if isinstance(other, DenseMap):
            other: SparseMap = other.convert_to_sparse_map()
        if not isinstance(other, SparseMap):
            return False
        return super().__eq__(other)
