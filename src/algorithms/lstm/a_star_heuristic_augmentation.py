import copy
from typing import Tuple, Type, List, Dict

import numpy as np

from algorithms.algorithm import Algorithm
from algorithms.basic_testing import BasicTesting
from algorithms.classic.graph_based.a_star import AStar
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.entities.trace import Trace
from algorithms.lstm.LSTM_tile_by_tile import OnlineLSTM
from simulator.services.algorithm_runner import AlgorithmRunner
from simulator.services.services import Services
from simulator.views.map.display.gradient_list_map_display import GradientListMapDisplay
from simulator.views.map.display.map_display import MapDisplay
from structures import Point


class AStarHeuristicAugmentation(AStar):
    _heuristic_multiplier: List[List[float]]
    _algorithm: str

    def __init__(self, services: Services, testing: BasicTesting = None,
                 algorithm: Tuple[Type[Algorithm], Tuple[List, Dict]] = (OnlineLSTM, ([200], {}))):
        super().__init__(services, testing)

        self._algorithm = algorithm
        self._heuristic_multiplier = None

    def set_display_info(self) -> List[MapDisplay]:
        ret: List[MapDisplay] = super().set_display_info()
        ret.append(GradientListMapDisplay(self._services, self._heuristic_multiplier, min_color=np.array([100., 0., 0.]),
                               max_color=np.array([250., 250., 250.]), z_index=25))
        return ret

    def _find_path_internal(self) -> None:
        expand_dist = 1

        # modify heuristic
        lstm_algo: AlgorithmRunner = self._services.algorithm.get_new_runner(copy.deepcopy(self._get_grid()),
                                                                             *self._algorithm)
        lstm_algo.find_path()

        self._build_heuristic_grid_from_trace(lstm_algo.map.trace, expand_dist)
        self.key_frame()
        super()._find_path_internal()

    def _build_heuristic_grid_from_trace(self, trace: List[Trace], expand_dist) -> None:
        self._heuristic_multiplier = [[1 for _ in range(self._get_grid().size.width)] for _ in
                                      range(self._get_grid().size.height)]

        for t in trace:
            self._expand_heuristic_node(t.position, True, expand_dist)
            self.key_frame()

    def _expand_heuristic_node(self, pos: Point, all_0: bool, expand_dist) -> None:
        if expand_dist == 0:
            return

        self._heuristic_multiplier[pos.y][pos.x] = 0
        if not all_0:
            self._heuristic_multiplier[pos.y][pos.x] = min(self._heuristic_multiplier[pos.y][pos.x], 1.0 / (expand_dist + 1))

        for n in self._get_grid().get_neighbours(pos):
            self._expand_heuristic_node(n, all_0, expand_dist - 1)

    def _euclidean_distance(self, pos: Point, goal: Goal) -> float:
        return np.linalg.norm(np.array(pos) - np.array(goal.position))

    def _custom_heuristic(self, pos: Point, goal: Goal) -> float:
        return self._heuristic_multiplier[pos.y][pos.x] * self._euclidean_distance(pos, goal)

    def get_heuristic(self, pos: Point) -> float:
        """
        Returns the euclidean distance from the given position to the goal
        It does memoization as well
        :param goal: The goal
        :param pos: The initial position
        :return:
        """
        h_func = min(self._euclidean_distance(pos, self._get_grid().goal), self._custom_heuristic(pos, self._get_grid().goal))
        self.mem.h.setdefault(pos, h_func)
        return self.mem.h[pos]
