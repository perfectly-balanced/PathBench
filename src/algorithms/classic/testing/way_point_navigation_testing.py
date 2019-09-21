from typing import Dict, Any

import numpy as np

from algorithms.basic_testing import BasicTesting
from algorithms.lstm.combined_online_LSTM import CombinedOnlineLSTM
from simulator.services.debug import DebugLevel


class WayPointNavigationTesting(BasicTesting):
    """
    Testing for AStar
    """

    def get_results(self) -> Dict[str, Any]:
        """
        Read super description
        """
        res: Dict[str, Any] = super().get_results()
        res["nr_of_way_points"] = len(self._services.algorithm.instance.way_points)
        res["global_kernel_steps"] = self._services.algorithm.instance.global_kernel_steps - (res["nr_of_way_points"] - 1)

        local_testing = self._services.algorithm.instance.local_testing_results[:-1]

        res["global_kernel_distance"] = sum(map(lambda t: t["total_distance"], local_testing))

        res["last_way_point_distance_from_goal"] = self.distance_from_agent_to_goal(self._services.algorithm.map, self._services.algorithm.instance.last_way_point_position)
        res["nr_of_global_kernel_calls"] = len(self._services.algorithm.instance.global_testing_results)
        res["global_kernel_improvement"] = float(res["global_kernel_distance"]) / res["total_distance"] * 100 if res["total_distance"] != 0 else 100

        distances = []
        if len(self._services.algorithm.instance.way_points) > 1:
            for i in range(1, len(self._services.algorithm.instance.way_points)):
                prev_way_point = self._services.algorithm.instance.way_points[i - 1]
                next_way_point = self._services.algorithm.instance.way_points[i]
                distances.append(np.linalg.norm(np.array(prev_way_point) - np.array(next_way_point)))

        average_way_point_distance = 0

        if distances:
            average_way_point_distance = float(sum(distances)) / len(distances)

        res["average_way_point_distance"] = average_way_point_distance

        if self._services.algorithm.instance.global_kernel_type == CombinedOnlineLSTM and res["nr_of_global_kernel_calls"] > 0:
            kernels = self._services.algorithm.instance.global_testing_results[0]["kernels"]
            calls = [0 for _ in range(len(kernels))]

            for t_result in self._services.algorithm.instance.global_testing_results:
                calls[t_result["best_kernel_idx"]] += 1

            for i in range(len(calls)):
                calls[i] = float(calls[i]) / res["nr_of_global_kernel_calls"] * 100

            res["global_kernel_kernel_names"] = kernels
            res["global_kernel_best_kernel_calls"] = calls

        res["local_kernel_total_search_space"] = \
            BasicTesting.get_occupancy_percentage_size(self._services.algorithm.map.size, len(self._services.algorithm.instance.display_info_data[0]))
        res["local_kernel_total_fringe"] = \
            BasicTesting.get_occupancy_percentage_size(self._services.algorithm.map.size, len(self._services.algorithm.instance.display_info_data[1]))
        res["local_kernel_total_total"] = \
            BasicTesting.get_occupancy_percentage_size(self._services.algorithm.map.size,
                                                       len(self._services.algorithm.instance.display_info_data[0]) + len(self._services.algorithm.instance.display_info_data[1]))

        average_search_space = self._services.algorithm.instance.display_info_data[2]
        average_search_space = list(map(lambda el: BasicTesting.get_occupancy_percentage_size(self._services.algorithm.map.size, el), average_search_space))
        average_search_space = float(sum(average_search_space)) / len(average_search_space)

        average_fringe = self._services.algorithm.instance.display_info_data[3]
        average_fringe = list(map(lambda el: BasicTesting.get_occupancy_percentage_size(self._services.algorithm.map.size, el), average_fringe))
        average_fringe = float(sum(average_fringe)) / len(average_fringe)

        average_total = self._services.algorithm.instance.display_info_data[4]
        average_total = list(map(lambda el: BasicTesting.get_occupancy_percentage_size(self._services.algorithm.map.size, el), average_total))
        average_total = float(sum(average_total)) / len(average_total)

        res["local_kernel_average_search_space"] = average_search_space
        res["local_kernel_average_fringe"] = average_fringe
        res["local_kernel_average_total"] = average_total

        return res

    def print_results(self) -> None:
        """
        Read super description
        """
        super().print_results()
        results: Dict[str, Any] = self.get_results()
        self._services.debug.write("Last way point distance from goal: {0:.2f}".format(results["last_way_point_distance_from_goal"]), DebugLevel.BASIC)
        self._services.debug.write("Number of way points: {}".format(results["nr_of_way_points"]), DebugLevel.BASIC)
        self._services.debug.write("Average way point in-between distance: {0:.2f}".format(results["average_way_point_distance"]), DebugLevel.BASIC)
        self._services.debug.write("Global kernel steps: {}".format(results["global_kernel_steps"]), DebugLevel.BASIC)
        self._services.debug.write("Global kernel distance: {0:.2f}".format(results["global_kernel_distance"]), DebugLevel.BASIC)
        self._services.debug.write("Global kernel progress: {0:.2f}%".format(results["global_kernel_improvement"]), DebugLevel.BASIC)
        self._services.debug.write("Local kernel all search space (no fringe): {0:.2f}%".format(results["local_kernel_total_search_space"]),DebugLevel.BASIC)
        self._services.debug.write("Local kernel all fringe: {0:.2f}%".format(results["local_kernel_total_fringe"]), DebugLevel.BASIC)
        self._services.debug.write("Local kernel all total search space: {0:.2f}%".format(results["local_kernel_total_total"]), DebugLevel.BASIC)
        self._services.debug.write("Local kernel session average search space (no fringe): {0:.2f}%".format(results["local_kernel_average_search_space"]), DebugLevel.BASIC)
        self._services.debug.write("Local kernel session average fringe: {0:.2f}%".format(results["local_kernel_average_fringe"]), DebugLevel.BASIC)
        self._services.debug.write("Local kernel session average total search space: {0:.2f}%".format(results["local_kernel_average_total"]), DebugLevel.BASIC)

        if "global_kernel_kernel_names" in results:
            self._services.debug.write("Combined Online LSTM kernels: {}".format(results["global_kernel_kernel_names"]), DebugLevel.BASIC)
            self._services.debug.write("Kernel percentages: {}".format(list(map(lambda r: str(round(r, 2)) + "%", results["global_kernel_best_kernel_calls"]))), DebugLevel.BASIC)
