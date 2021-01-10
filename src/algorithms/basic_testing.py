from typing import Dict, Any, List, Callable, Optional
from operator import mul
from functools import reduce
import time

import numpy as np
from structures.heap import Heap

from utility.timer import Timer
from utility.threading import Condition, cond_var_wait_for
from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.trace import Trace
from algorithms.configuration.entities.obstacle import Obstacle
from algorithms.configuration.maps.dense_map import DenseMap
from algorithms.configuration.maps.map import Map
from algorithms.configuration.maps.sparse_map import SparseMap
from simulator.services.debug import DebugLevel
from simulator.services.services import Services
from simulator.views.map.display.map_display import MapDisplay
from structures import Size, Point


class BasicTesting:
    """
    Basic class used to assess the performance of an algorithms
    All tests must inherit from this class
    """

    timer: Timer
    display_info: List[MapDisplay]
    key_frame: Callable[[List, Dict], None]
    cv: Condition
    key_frame_skip_count: int
    total_time: int

    def __init__(self, services: Services) -> None:
        self._services = services
        self.timer = None
        self.display_info = []
        self.key_frame = lambda *args, **kwargs: None
        self.cv = None
        self.key_frame_skip_count = 0
        self.total_time = 0
        self.last_frame_time = 0
        self.requires_key_frame = False
        self.reset()

    def reset(self) -> None:
        """
        This method resets the testing state
        """
        self.display_info = []
        self.cv = None
        self.key_frame_skip_count = 0
        self.total_time = 0
        self.last_frame_time = 0
        if self._services.settings.simulator_key_frame_speed == 0 or not self._services.settings.simulator_graphics:
            self.key_frame = lambda *args, **kwargs: None
        else:
            self.key_frame = self.key_frame_internal

    def set_condition(self, cv: Condition) -> None:
        """
        Setter for condition
        :param cv:  The condition is used to synchronize key frames
        """
        self.cv = cv

    def algorithm_start(self) -> None:
        """
        Marks the start of the algorithms
        """
        self._services.debug.write("Algorithm " + str(self._services.algorithm.algorithm_type) + " started..",
                                   DebugLevel.BASIC)
        self.timer = Timer()

    def is_terminated(self) -> bool:
        return self._services.algorithm.instance.testing != self

    def key_frame_internal(self, ignore_key_frame_skip: bool = False, only_render: List[int] = None, root_key_frame=None) -> None:
        """
        Internal key frame handler
        """
        if self.timer is None:
            self._services.debug.write_error("Algorithm " + str(self._services.algorithm.algorithm_type) + " hasn't started yet!")
            return
        self.timer.pause()

        if root_key_frame:
            root_key_frame.instance.testing.key_frame(ignore_key_frame_skip=ignore_key_frame_skip, only_render=only_render)

        def sleep_pred():
            return not self.requires_key_frame or self.is_terminated()

        # skip render frame
        if ignore_key_frame_skip or self.key_frame_skip_count >= self._services.settings.simulator_key_frame_skip:
            self.display_info = self._services.algorithm.instance.set_display_info()

            if only_render:
                self.display_info = [self.display_info[i] for i in only_render]

            if self.cv is not None:
                with self.cv:
                    self.requires_key_frame = True
                    self.cv.notify()
                    cond_var_wait_for(self.cv, sleep_pred)
            self.key_frame_skip_count = 0
        else:
            self.key_frame_skip_count += 1

        # limit key frame speed
        if self.cv is not None:
            t = time.time()
            sleep_dt = self._services.settings.simulator_key_frame_speed - (t - self.last_frame_time)
            self.last_frame_time = t

            while sleep_dt > 0:
                with self.cv:
                    self.requires_key_frame = True
                    self.cv.notify()
                    cond_var_wait_for(self.cv, sleep_pred)
                nt = time.time()
                sleep_dt = sleep_dt - nt + t
                t = nt

        if self.is_terminated():
            from simulator.models.map_model import AlgorithmTerminated
            raise AlgorithmTerminated()

        self.timer.resume()

    def algorithm_done(self) -> None:
        """
        Marks the end of the algorithms
        """
        if self.timer is None:
            self._services.debug.write_error(
                "Algorithm " + str(self._services.algorithm.algorithm_type) + "did not start!")
            return
        self.total_time = self.timer.stop()

        if self._services.settings.simulator_graphics:
            self.display_info = self._services.algorithm.instance.set_display_info()
        self.print_results()
        self._services.debug.write("", DebugLevel.BASIC, timestamp=False)

    def get_results(self) -> Dict[str, Any]:
        """
        Returns the test results
        :return: The test results
        """
        trace: List[Trace] = self._services.algorithm.map.trace

        grid: Optional[DenseMap] = self._services.algorithm.map
        if isinstance(grid, SparseMap):
            grid: DenseMap = grid.convert_to_dense_map()

        return {
            "map": self._services.algorithm.map,
            "map_obstacles_percentage": self.get_occupancy_percentage_grid(grid.grid, DenseMap.WALL_ID),
            "goal_found": self._services.algorithm.map.is_goal_reached(grid.agent.position),
            "distance_to_goal": self.distance_from_agent_to_goal(grid),
            "original_distance_to_goal": self.get_original_distance(grid),
            "trace": trace,
            "total_steps": len(trace),
            "total_distance": self.get_euclidean_distance_traveled(trace, grid.agent),
            "smoothness_of_trajectory": self.get_smoothness(trace, grid.agent),
            "obstacle_clearance": self.get_average_clearance(trace, self._services.algorithm.map),
            "total_time": self.total_time,
            "algorithm_type": self._services.algorithm.algorithm_type,
        }

    def print_results(self) -> None:
        """
        Prints the test results
        """
        results: Dict[str, Any] = self.get_results()
        self._services.debug.write("Map: " + str(results["map"]), DebugLevel.MEDIUM)
        self._services.debug.write("Original distance: {0:.2f}".format(results["original_distance_to_goal"]), DebugLevel.BASIC)
        self._services.debug.write("Occupancy percentage: {0:.2f}%".format(results["map_obstacles_percentage"]), DebugLevel.BASIC)
        self._services.debug.write("Goal was " + ("FOUND" if results["goal_found"] else "NOT FOUND"), DebugLevel.BASIC)
        if not results["goal_found"]:
            self._services.debug.write("Distance to goal: " + str(results["distance_to_goal"]) + " (Original: {})".format(results["original_distance_to_goal"]))
        self._services.debug.write("Total steps: " + str(results["total_steps"]), DebugLevel.BASIC)
        self._services.debug.write("Total distance: {0:.2f}".format(results["total_distance"]), DebugLevel.BASIC)
        self._services.debug.write("Trajectory Smoothness: {0:.2f}".format(results["smoothness_of_trajectory"]), DebugLevel.BASIC)
        self._services.debug.write("Total time: " + str(results["total_time"]) + " seconds", DebugLevel.BASIC)
        self._services.debug.write("Trace: " + str(results["trace"]), DebugLevel.MEDIUM)

    @staticmethod
    def get_occupancy_percentage_grid(grid: np.array, token: int) -> float:
        """
        Searches for token in grid and returns the occupancy percentage of the token
        :param grid: The grid
        :param token: The search token
        :return: The percentage
        """
        tokens: int = np.sum(grid == token)

        return BasicTesting.get_occupancy_percentage_size(Size(*grid.shape), tokens)

    @staticmethod
    def get_original_distance(mp: Map):
        agent_pos = mp.trace[0].position if mp.trace else None
        return BasicTesting.distance_from_agent_to_goal(mp, agent_pos)

    @staticmethod
    def distance_from_agent_to_goal(mp: Map, agent_pos: Point = None):
        if agent_pos is None:
            agent_pos = mp.agent.position
        dist = np.linalg.norm(np.array(agent_pos) - np.array(mp.goal.position))
        return dist

    @staticmethod
    def get_occupancy_percentage_size(size: Size, items_nr: int) -> float:
        """
        Returns the percentage of present items given the size
        :param size: The size
        :param items_nr:  The number of items
        :return: The percentage
        """
        return (items_nr / reduce(mul, size, 1)) * 100

    @staticmethod
    def get_euclidean_distance_traveled(trace: List[Trace], agent: Agent) -> float:
        if (not trace) or trace[-1].position != agent.position:
            trace.append(Trace(agent.position))
        dist = 0
        for i in range(1, len(trace)):
            dist += np.linalg.norm(np.array(trace[i - 1].position) - np.array(trace[i].position))
        return dist

    @staticmethod
    def get_smoothness(trace: List[Trace], agent: Agent) -> float:
        if (not trace) or trace[-1].position != agent.position:
            trace.append(Trace(agent.position))
        angle = 0
        prev_angle = None

        for i in range(1, len(trace)):

            if(not trace[i - 1].position == trace[i].position):

                unit_vector1 = np.array(trace[i - 1].position - trace[i].position) / np.linalg.norm(np.array(trace[i - 1].position - trace[i].position))
                t = [0] * (len(agent.position) - 1)
                t = tuple([1] + t)
                dot_product = np.dot(unit_vector1, t)
                if prev_angle is None:
                    prev_angle = np.arccos(dot_product)
                    continue

                angle += abs(prev_angle - np.arccos(dot_product))
                prev_angle = np.arccos(dot_product)

        return (angle / len(trace))

    @staticmethod
    def get_average_clearance(trace: List[Trace], grid: Map) -> float:
        if isinstance(grid, DenseMap):
            obstacles = np.transpose(np.where(grid.grid == grid.WALL_ID))
        else:
            obstacles = np.full(grid.size, grid.CLEAR_ID, dtype=np.uint8)
            for o in grid.obstacles:
                if type(o) is Obstacle:  # we don't want extended wall
                    obstacles[o.position.values] = grid.WALL_ID
            obstacles = np.transpose(np.where(obstacles == grid.WALL_ID))

        if len(grid.obstacles) == 0:
            return 0

        obstacle_dists = []
        for t in trace:
            pos = np.array(t.position.values)
            obstacles_shifted = obstacles - pos
            norms = np.linalg.norm(obstacles_shifted, axis=1)
            min_dist = np.min(norms)
            obstacle_dists.append(min_dist)
        return sum(obstacle_dists) / len(obstacle_dists)

    @property
    def map(self) -> str:
        return "map"

    @map.getter
    def map(self) -> Map:
        return self._services.algorithm.map
