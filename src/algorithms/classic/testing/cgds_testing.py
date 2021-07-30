from typing import Dict, Any

from algorithms.basic_testing import BasicTesting
from simulator.services.debug import DebugLevel


class CGDSTesting(BasicTesting):
    """
    Testing for CGDS
    """

    def get_results(self) -> Dict[str, Any]:
        """
        Read super description
        """
        res: Dict[str, Any] = super().get_results()
        res["active_space"] = BasicTesting.get_occupancy_percentage_size(self._services.algorithm.map.size,
                                                                   len(self._services.algorithm.instance.mem.deque))
        res["search_space"] = BasicTesting.get_occupancy_percentage_size(self._services.algorithm.map.size,
                                                                         len(self._services.algorithm.instance.mem.visited))
        return res

    def print_results(self) -> None:
        """
        Read super description
        """
        super().print_results()
        results: Dict[str, Any] = self.get_results()
        self._services.debug.write("Search space percentage: {0:.2f}%".format(results["search_space"]), DebugLevel.BASIC)
        self._services.debug.write("Active search space percentage: {0:.2f}%".format(results["active_space"]), DebugLevel.BASIC)

