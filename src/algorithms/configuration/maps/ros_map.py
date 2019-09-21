import copy
from typing import List, Callable, Dict, Any, Optional

from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.entities.obstacle import Obstacle
from algorithms.configuration.maps.dense_map import DenseMap
from simulator.services.debug import DebugLevel
from simulator.services.services import Services
from structures import Point, Size


class RosMap(DenseMap):
    __update_requested: Optional[Callable[[], None]]
    __get_grid: Callable[[], List[List[int]]]
    __wp_publish: Optional[Callable[[Point], None]]

    def __init__(self, size: Size, agent: Agent, goal: Goal,
                 get_grid: Callable[[], List[List[int]]],
                 wp_publish: Optional[Callable[[Point], None]] = None,
                 update_requested: Optional[Callable[[], None]] = None,
                 services: Services = None, default_update: bool = True) -> None:
        super().__init__(None, services)

        self.__get_grid = get_grid
        self.__wp_publish = wp_publish
        self.__update_requested = update_requested
        self.size = size
        self.agent = agent
        self.goal = goal

        if self._services:
            self.request_update = self._services.debug.debug_func(DebugLevel.LOW)(self.request_update)

        if default_update:
            self.grid = [[self.WALL_ID for _ in range(self.size.width)] for _ in range(self.size.height)]
            self.__update_grid(self.grid)

    def __update_grid(self, occ_grid):
        self.obstacles.clear()

        for i in range(len(occ_grid)):
            for j in range(len(occ_grid[i])):
                pos = Point(j, i)
                if self.agent.position != pos and self.goal.position != pos and occ_grid[i][j] == self.WALL_ID:
                    self.obstacles.append(Obstacle(Point(j, i)))
                if occ_grid[i][j] == self.EXTENDED_WALL:
                    occ_grid[i][j] = self.CLEAR_ID

        occ_grid[self.agent.position.y][self.agent.position.x] = self.AGENT_ID
        occ_grid[self.goal.position.y][self.goal.position.x] = self.GOAL_ID
        self.grid = occ_grid
        self.extend_walls()

    def request_update(self):
        self.__update_grid(copy.deepcopy(self.__get_grid()))

        if self.__update_requested:
            self.__update_requested()

    def publish_wp(self, to: Point):
        self.__wp_publish(to)

    def __copy__(self) -> 'DenseMap':
        return copy.deepcopy(self)

    def __deepcopy__(self, memo: Dict) -> 'DenseMap':
        dense_map = RosMap(self.size, self.agent, self.goal, self.__get_grid, self.__wp_publish,
                           self.__update_requested, self._services, False)
        dense_map.grid = copy.deepcopy(self.grid)
        dense_map.trace = copy.deepcopy(self.trace)
        dense_map.agent = copy.deepcopy(self.agent)
        dense_map.goal = copy.deepcopy(self.goal)
        dense_map.obstacles = copy.deepcopy(self.obstacles)
        return dense_map
