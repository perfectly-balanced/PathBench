from typing import Dict, Any

import numpy as np

from algorithms.basic_testing import BasicTesting
from simulator.services.debug import DebugLevel


class CombinedOnlineLSTMTesting(BasicTesting):
    """
    Testing for AStar
    """

    def get_results(self) -> Dict[str, Any]:
        """
        Read super description
        """
        res: Dict[str, Any] = super().get_results()
        res["kernels"] = self._services.algorithm.instance.kernel_names
        res["best_kernel"] = res["kernels"][self._services.algorithm.instance.kernel_call_idx] if self._services.algorithm.instance.kernel_call_idx is not None else None
        res["best_kernel_idx"] = self._services.algorithm.instance.kernel_call_idx
        return res

    def print_results(self) -> None:
        """
        Read super description
        """
        super().print_results()
        results: Dict[str, Any] = self.get_results()
        self._services.debug.write("Kernels: {}".format(results["kernels"]), DebugLevel.BASIC)
        self._services.debug.write("Best kernel: {}".format(results["best_kernel"]), DebugLevel.BASIC)
