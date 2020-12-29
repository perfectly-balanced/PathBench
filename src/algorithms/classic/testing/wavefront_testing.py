from typing import Dict, Any

from algorithms.basic_testing import BasicTesting
from simulator.services.debug import DebugLevel

import numpy as np


class WavefrontTesting(BasicTesting):
    """
    Testing for Wavefront
    """

    def get_results(self) -> Dict[str, Any]:
        """
        Read super description
        """
        res: Dict[str, Any] = super().get_results()

        step_grid = self._services.algorithm.instance.step_grid
        res["search_space"] = 100 - BasicTesting.get_occupancy_percentage_grid(step_grid, 0)
        return res

    def print_results(self) -> None:
        """
        Read super description
        """
        super().print_results()
        results: Dict[str, Any] = self.get_results()
        self._services.debug.write("Search space percentage: {0:.2f}%".format(results["search_space"]),
                                   DebugLevel.BASIC)
