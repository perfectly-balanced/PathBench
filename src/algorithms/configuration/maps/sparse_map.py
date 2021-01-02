import copy
from typing import List, Union, Dict, Optional

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

    def __init__(self, size: Size, agent: Agent, obstacles: List[Obstacle], goal: Goal, services: Services = None, mutable: bool = False, name: Optional[str] = None) \
            -> None:
        super().__init__(size, services, mutable, name)
        self.agent = agent
        self.obstacles = obstacles
        self.goal = goal

    def at(self, p: Point) -> int:
        if self.agent.position == p:
            return self.AGENT_ID
        elif p in [o.position for o in self.obstacles]:
            return self.WALL_ID
        elif self.goal.position == p:
            return self.GOAL_ID
        else:
            return self.CLEAR_ID

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

    def convert_to_dense_map(self) -> Optional[DenseMap]:
        """
        Converts current map into a :class:`DenseMap`
        :return: The converted map
        """
        grid: np.array = np.zeros(self.size, dtype=np.int32)
        should_optimize: bool = len(self.obstacles) > 20

        for obstacle in self.obstacles:
            total_radius: int = self.agent.radius + obstacle.radius
            if should_optimize:
                total_radius = obstacle.radius
            for index in np.ndindex(*(len(self.size) * [total_radius*2 + 1])):
                p = [elem + obstacle.position[i] - total_radius for i, elem in enumerate(index)]
                if not self.is_out_of_bounds_pos(Point(*p)):
                    dist: Union[float, np.ndarray] = np.linalg.norm(np.array(p) - np.array(obstacle.position))
                    if (not should_optimize) and (dist <= obstacle.radius + self.agent.radius) and (grid[tuple(p)] == DenseMap.CLEAR_ID):
                        grid[tuple(p)] = DenseMap.EXTENDED_WALL_ID
                    if dist <= obstacle.radius:
                        grid[tuple(p)] = DenseMap.WALL_ID
        grid[self.agent.position.values] = DenseMap.AGENT_ID
        grid[self.goal.position.values] = DenseMap.GOAL_ID
        dense_map: DenseMap = DenseMap(grid, self.services, transpose=False)
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

    def __repr__(self) -> str:
        debug_level: DebugLevel = DebugLevel.BASIC
        if self.services is not None:
            debug_level = self.services.settings.simulator_write_debug_level

        obst_str: str = "{\n\t\t\tsize: " + str(len(self.obstacles))
        if debug_level == DebugLevel.HIGH or len(self.obstacles) <= 10:
            obst_str += ", \n\t\t\tentities: [\n"
            for obst in sorted(self.obstacles, key=lambda o: o.position[::-1]):
                obst_str += "\t\t\t\t" + str(obst) + ", \n"
            obst_str += "\t\t\t]"
        obst_str += "\n\t\t}"

        return "SparseMap: {\n\t\tsize: " + str(self.size) + ", \n\t\tagent: " + str(self.agent) + \
               ", \n\t\tobstacles: " + obst_str + ", \n\t\tgoal: " + str(self.goal) + "\n\t}"

    def __copy__(self) -> 'SparseMap':
        return copy.deepcopy(self)

    def __deepcopy__(self, memo: Dict) -> 'SparseMap':
        dense_map = SparseMap(self.size,
                              copy.deepcopy(self.agent),
                              copy.deepcopy(self.obstacles),
                              copy.deepcopy(self.goal),
                              self.services,
                              self.mutable,
                              copy.deepcopy(self.name))
        dense_map.trace = copy.deepcopy(self.trace)
        return dense_map

    def __eq__(self, other: object) -> bool:
        if isinstance(other, DenseMap):
            other: SparseMap = other.convert_to_sparse_map()
        if not isinstance(other, SparseMap):
            return False
        return super().__eq__(other)
