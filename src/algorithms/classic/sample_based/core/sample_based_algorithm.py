from typing import List
from algorithms.algorithm import Algorithm
from simulator.views.map.display.graph_map_display import GraphMapDisplay
from simulator.views.map.display.map_display import MapDisplay


class SampleBasedAlgorithm(Algorithm):
    __map_displays: List[MapDisplay]

    def _init_displays(self) -> None:
        self.__map_displays = [GraphMapDisplay(self._services, self._graph)]

    def set_display_info(self) -> List[MapDisplay]:
        return super().set_display_info() + self.__map_displays