from utility.threading import Condition
from typing import Optional, List
from abc import ABC, abstractmethod

from algorithms.basic_testing import BasicTesting
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map.display.entities_map_display import EntitiesMapDisplay
from simulator.views.map.display.map_display import MapDisplay
from simulator.views.map.display.solid_iterable_map_display import SolidIterableMapDisplay
from structures import Point


class Algorithm(ABC):
    """
    Class for defining basic API for algorithms.
    All algorithms must inherit from this class.
    """
    testing: Optional[BasicTesting]
    _services: Services

    def __init__(self, services: Services, testing: BasicTesting = None) -> None:
        self._services = services
        self.testing = testing
        self.__root_key_frame = self._services.algorithm

    def set_condition(self, key_frame_condition: Condition) -> None:
        """
        This method is used to initialise the debugging condition
        :param key_frame_condition: The condition
        """
        if self.testing is not None:
            self.testing.set_condition(key_frame_condition)

    def get_display_info(self) -> List[MapDisplay]:
        """
        Returns the info displays
        :return: A list of info displays
        """
        if self.testing is not None:
            return self.testing.display_info
        return []

    def key_frame(self, *args, **kwargs) -> None:
        """
        Method that marks a key frame
        It is used in animations
        """
        if self.testing is not None:
            self.testing.key_frame(*args, **kwargs, root_key_frame=self.__root_key_frame)

    def set_root_key_frame(self, algo):
        self.__root_key_frame = algo

    def find_path(self) -> None:
        """
        Method for finding a path from agent to goal
        Movement should be done using the map APIs
        """
        if self.testing is not None:
            self.testing.algorithm_start()
        self._find_path_internal()
        if self.testing is not None:
            self.testing.algorithm_done()

    def _get_grid(self) -> Map:
        """
        Shortcut to get the map
        :return: The map
        """
        return self._services.algorithm.map

    def move_agent(self, to: Point) -> None:
        """
        Method used to move the agent on the map
        :param to: the destination
        """
        #method is in map.py. follow param means Instead of teleport, moves in a straight line
        self._get_grid().move_agent(to, follow=True)

    @abstractmethod
    def set_display_info(self) -> List[MapDisplay]:
        """
        Method used for setting the info displays
        All algorithms must override this method
        :return: A list of info displays
        """
        return []

    @abstractmethod
    def _find_path_internal(self) -> None:
        """
        The internal implementation of :ref:`find_path`
        All algorithms must override this method
        """
        pass
