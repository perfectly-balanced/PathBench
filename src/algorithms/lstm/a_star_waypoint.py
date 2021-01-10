import copy
from typing import List, Set, Any, Dict, Tuple, Type, Union

from algorithms.algorithm import Algorithm
from algorithms.basic_testing import BasicTesting
from algorithms.classic.graph_based.a_star import AStar
from algorithms.classic.testing.combined_online_lstm_testing import CombinedOnlineLSTMTesting
from algorithms.configuration.maps.map import Map
from algorithms.configuration.maps.ros_map import RosMap
from algorithms.lstm.LSTM_tile_by_tile import OnlineLSTM
from algorithms.lstm.combined_online_LSTM import CombinedOnlineLSTM
from simulator.services.algorithm_runner import AlgorithmRunner
from simulator.services.services import Services
from simulator.views.map.display.map_display import MapDisplay
from simulator.views.map.display.solid_iterable_map_display import SolidIterableMapDisplay
from structures import Point, Colour


class WayPointNavigation(Algorithm):
    SIMPLE_ANIMATIONS: bool = False

    way_points: List[Point]
    display_info_data: List[Union[Set[Any], List]]
    _local_kernel: Tuple[Type[Algorithm], Tuple[list, dict]]
    _global_kernel: Tuple[Type[Algorithm], Tuple[list, dict]]
    _global_kernel_max_it: int

    __local_kernel_anim: AlgorithmRunner
    __global_kernel_anim: AlgorithmRunner
    __is_local_anim: bool

    def __init__(self, services: Services, testing: BasicTesting = None,
                 local_kernel: Tuple[Type[Algorithm], Tuple[List, Dict]] = (AStar, ([], {})),
                 global_kernel: Tuple[Type[OnlineLSTM], Tuple[List, Dict]] = (CombinedOnlineLSTM, ([], {})), global_kernel_max_it: int = 20):
        super().__init__(services, testing)

        if not global_kernel:
            raise NotImplementedError("Parameters local_kernel, global_kernel, global_kernel_max_it are mandatory")

        self.way_points = []

        self.display_info_data = [set(), set(), [], [], []]
        self._local_kernel = local_kernel
        self._global_kernel = global_kernel
        self._global_kernel_max_it = global_kernel_max_it

        # testing fields
        self.total_steps = 0
        self.global_kernel_steps = 0
        self.last_way_point_position = None
        self.global_testing_results = []
        self.local_testing_results = []
        self.global_kernel_type = global_kernel[0]
        self.local_kernel_type = local_kernel[0]

        self.__local_kernel_anim = None
        self.__global_kernel_anim = None
        self.__is_local_anim = False

    def set_display_info(self) -> List[MapDisplay]:
        ret: List[MapDisplay] = super().set_display_info()

        if not self.SIMPLE_ANIMATIONS:
            if self.__is_local_anim and self.__local_kernel_anim:
                ret += self.__local_kernel_anim.instance.set_display_info()

            if not self.__is_local_anim and self.__global_kernel_anim:
                ret += self.__global_kernel_anim.instance.set_display_info()

        visited_colour = self._services.state.views.add_colour("visited", Colour(0.19, 0.19, 0.2, 0.8))
        waypoint_colour = self._services.state.views.add_colour("waypoint", Colour(0, 1, 1))

        ret.append(SolidIterableMapDisplay(self._services, self.display_info_data[0], visited_colour, z_index=70))
        ret.append(SolidIterableMapDisplay(self._services, self.display_info_data[1], visited_colour, z_index=65))
        ret.append(SolidIterableMapDisplay(self._services, self.way_points, waypoint_colour, z_index=200))
        return ret

    def _find_path_internal(self) -> None:
        stuck_threshold = 2
        history_frequency: Dict[Point, int] = {}

        global_kernel_params = self._global_kernel[1]
        global_kernel_params[1]["max_it"] = self._global_kernel_max_it
        global_testing = BasicTesting
        if self._global_kernel[0] == CombinedOnlineLSTM:
            global_testing = CombinedOnlineLSTMTesting
        global_kernel_algo: AlgorithmRunner = self._services.algorithm.get_new_runner(copy.deepcopy(self._get_grid()),
                                                                                      self._global_kernel[0],
                                                                                      global_kernel_params,
                                                                                      algorithm_testing=global_testing,
                                                                                      with_animations=not self.SIMPLE_ANIMATIONS)
        local_kernel_algo: AlgorithmRunner = self._services.algorithm.get_new_runner(copy.deepcopy(self._get_grid()),
                                                                                     *self._local_kernel,
                                                                                     algorithm_testing=BasicTesting,
                                                                                     with_animations=not self.SIMPLE_ANIMATIONS)
        self.way_points.append(self._get_grid().agent.position)

        self.__local_kernel_anim = local_kernel_algo
        self.__global_kernel_anim = global_kernel_algo

        while True:
            self.__update_map(local_kernel_algo, global_kernel_algo)

            self.__is_local_anim = False

            global_kernel_algo.find_path(agent_pos=self.way_points[-1])
            self.way_points.append(global_kernel_algo.map.agent.position)
            res = None
            if global_kernel_algo.instance.testing:
                res = global_kernel_algo.instance.testing.get_results()
            self.global_testing_results.append(res)

            self.__is_local_anim = True

            local_kernel_algo.find_path(agent_pos=self.way_points[-2], goal_pos=self.way_points[-1])
            local_kernel_algo.map.replay_trace(lambda m: self.__replay(m))
            self.__accumulate_space(local_kernel_algo)

            new_freq: int = history_frequency.get(self.way_points[-1], 0) + 1
            history_frequency[self.way_points[-1]] = new_freq

            self.__update_map(local_kernel_algo, global_kernel_algo)
            self.key_frame(True)

            # fail safe
            if new_freq >= stuck_threshold:
                break

            if self._get_grid().is_goal_reached(self.way_points[-1]):
                break

        self.__is_local_anim = True

        self.global_kernel_steps = self.total_steps
        self.last_way_point_position = self.way_points[-1]

        self.__update_map(local_kernel_algo, global_kernel_algo)
        local_kernel_algo.find_path(agent_pos=self.way_points[-1], goal_pos=self._get_grid().goal.position)
        local_kernel_algo.map.replay_trace(lambda m: self.__replay(m))
        self.__accumulate_space(local_kernel_algo)
        self.__local_kernel_anim = None
        self.__global_kernel_anim = None
        self.key_frame(True)

    def __update_map(self, local_kernel_algo, global_kernel_algo):
        if isinstance(self._get_grid(), RosMap):
            self._get_grid().request_update()
            local_kernel_algo.map.request_update()
            global_kernel_algo.map.request_update()

    def __accumulate_space(self, local_kernel_algo: AlgorithmRunner) -> None:
        if not isinstance(local_kernel_algo.instance, AStar):
            return

        self.local_testing_results.append(local_kernel_algo.instance.testing.get_results())
        self.display_info_data[0] = self.display_info_data[0].union(local_kernel_algo.instance.mem.visited)
        self.display_info_data[1] = self.display_info_data[1].union(
            set(map(lambda el: el[1], local_kernel_algo.instance.mem.priority_queue)))
        self.display_info_data[2].append(len(local_kernel_algo.instance.mem.visited))
        self.display_info_data[3].append(len(local_kernel_algo.instance.mem.priority_queue))
        self.display_info_data[4].append(
            len(local_kernel_algo.instance.mem.visited) + len(local_kernel_algo.instance.mem.priority_queue))

    def __replay(self, m: Map) -> None:
        self.move_agent(m.agent.position)
        if isinstance(self._get_grid(), RosMap):
            self._get_grid().publish_wp(m.agent.position)
        self.total_steps += 1
        self.key_frame(True)
