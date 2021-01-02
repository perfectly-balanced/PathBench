import copy
from typing import List, Tuple, Callable, Dict, Any, Optional
from numbers import Real

from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.entities.obstacle import Obstacle
from algorithms.configuration.maps.occupancy_grid_map import OccupancyGridMap
from simulator.services.debug import DebugLevel
from simulator.services.services import Services
from structures import Point, Size

import numpy as np


class RosMap(OccupancyGridMap):
    __update_requested: Optional[Callable[[], None]]
    __get_grid: Callable[[], List[Any]]
    __wp_publish: Optional[Callable[[Point], None]]

    def __init__(self, agent: Agent, goal: Goal,
                 get_grid: Callable[[], List[Any]],
                 weight_bounds: Optional[Tuple[Real, Real]] = None, traversable_threshold: Optional[Real] = None, unmapped_value: Optional[Real] = None,
                 wp_publish: Optional[Callable[[Point], None]] = None,
                 update_requested: Optional[Callable[[], None]] = None,
                 services: Services = None,
                 mutable: bool = True,
                 name: Optional[str] = None) -> None:
        super().__init__(agent=agent, goal=goal, services=services, mutable=mutable, name=name)

        self.__get_grid = get_grid
        self.__wp_publish = wp_publish
        self.__update_requested = update_requested
        self.__weight_bounds = weight_bounds
        self.__traversable_threshold = traversable_threshold
        self.__unmapped_value = unmapped_value

        if self.services:
            self.request_update = self.services.debug.debug_func(DebugLevel.LOW)(self.request_update)

    def request_update(self):
        self.set_grid(self.__get_grid(), self.__weight_bounds, self.__traversable_threshold, self.__unmapped_value)

        if self.__update_requested:
            self.__update_requested()

    def publish_wp(self, to: Point):
        self.__wp_publish(to)

    def __copy__(self) -> 'RosMap':
        return copy.deepcopy(self)

    def __deepcopy__(self, memo: Dict) -> 'RosMap':
        mp = RosMap(copy.deepcopy(self.agent), copy.deepcopy(self.goal),
                    self.__get_grid, self.__weight_bounds, self.__traversable_threshold, self.__unmapped_value,
                    self.__wp_publish, self.__update_requested,
                    self.services, self.mutable, copy.deepcopy(self.name))
        mp.size = copy.deepcopy(self.size)
        mp.weight_grid = copy.deepcopy(self.weight_grid)
        mp.traversable_threshold = copy.deepcopy(self.traversable_threshold)
        mp.trace = copy.deepcopy(self.trace)
        mp.obstacles = copy.deepcopy(self.obstacles)
        mp.grid = copy.deepcopy(self.grid)
        return mp
