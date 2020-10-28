from typing import List
from algorithms.algorithm import Algorithm
from simulator.views.map.display.graph_map_display import GraphMapDisplay
from simulator.views.map.display.map_display import MapDisplay


class SampleBasedAlgorithm(Algorithm):

    def set_display_info(self) -> List[MapDisplay]:
        return super().set_display_info() + [GraphMapDisplay(self._services, self._graph)]