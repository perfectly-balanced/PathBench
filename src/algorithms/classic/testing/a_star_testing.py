from typing import Dict, Any

from algorithms.basic_testing import BasicTesting
from simulator.services.debug import DebugLevel


class AStarTesting(BasicTesting):
    """
    Testing for AStar
    """

    def get_results(self) -> Dict[str, Any]:
        """
        Read super description
        """
        res: Dict[str, Any] = super().get_results()
        res["fringe"] = BasicTesting.get_occupancy_percentage_size(self._services.algorithm.map.size,
                                                                   len(self._services.algorithm.instance.mem.priority_queue))
        res["search_space"] = BasicTesting.get_occupancy_percentage_size(self._services.algorithm.map.size,
                                                                         len(self._services.algorithm.instance.mem.visited))
        res["total_search_space"] = BasicTesting.get_occupancy_percentage_size(self._services.algorithm.map.size,
                                                                               len(self._services.algorithm.instance.mem.priority_queue) +
                                                                               len(self._services.algorithm.instance.mem.visited))
        return res

    def print_results(self) -> None:
        """
        Read super description
        """
        super().print_results()
        results: Dict[str, Any] = self.get_results()
        self._services.debug.write("Search space percentage (no fringe): {0:.2f}%".format(results["search_space"]), DebugLevel.BASIC)
        self._services.debug.write("Fringe percentage: {0:.2f}%".format(results["fringe"]), DebugLevel.BASIC)
        self._services.debug.write("Total search space percentage: {0:.2f}%".format(results["total_search_space"]), DebugLevel.BASIC)
