from heapq import heappush, heappop
from typing import Set, List, Tuple, Optional, Dict

import numpy as np

from algorithms.algorithm import Algorithm
from algorithms.basic_testing import BasicTesting
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map.display.gradient_map_display import GradientMapDisplay
from simulator.views.map.display.map_display import MapDisplay
from simulator.views.map.display.solid_color_map_display import SolidColorMapDisplay
from structures import Point, Colour, BLUE, DynamicColour

from memory_profiler import profile

"""
Heap duplicates https://www.redblobgames.com/pathfinding/a-star/implementation.html#optimizations
"""


class AStar(Algorithm):
    PQ_COLOUR_HIGH: Colour = BLUE
    PQ_COLOUR_LOW: Colour = Colour(0.27, 0.33, 0.35, 0.2)
    VISITED_COLOUR: Colour = Colour(0.19, 0.19, 0.2, 0.8)

    class InternalMemory:
        priority_queue: List[Tuple[int, Point]]
        visited: Set[Point]
        back_pointer: Dict[Point, Optional[Point]]
        g: Dict[Point, int]
        h: Dict[Point, float]

        def __init__(self):
            self.priority_queue = []
            self.visited = set()
            self.back_pointer = {}
            self.g = {}
            self.h = {}

    mem: InternalMemory

    def __init__(self, services: Services, testing: BasicTesting = None):
        super().__init__(services, testing)

        self.mem = AStar.InternalMemory()

    def set_display_info(self) -> List[MapDisplay]:
        """
        Read super description
        """
        return super().set_display_info() + [
            SolidColorMapDisplay(self._services, self.mem.visited, DynamicColour(self.VISITED_COLOUR), z_index=50),
            GradientMapDisplay(self._services, pts=self.mem.priority_queue,
                               min_color=DynamicColour(self.PQ_COLOUR_LOW), max_color=DynamicColour(self.PQ_COLOUR_HIGH), z_index=49, inverted=True),
        ]

    # noinspection PyUnusedLocal
    #@profile
    def _find_path_internal(self) -> None:
        self._init_mem()

        if self._expand():
            self._follow_back_trace()

    def _init_mem(self) -> None:
        grid: Map = self._get_grid()

        # push agent
        self.mem.g[grid.agent.position] = 0
        item: Tuple[float, Point] = (self.get_heuristic(grid.agent.position), grid.agent.position)
        heappush(self.mem.priority_queue, item)
        self.mem.back_pointer[grid.agent.position] = None

    def _expand(self) -> bool:
        grid: Map = self._get_grid()

        while len(self.mem.priority_queue) > 0:
            total_dist: float
            next_node: Point
            # peek and check if we need to terminate
            total_dist, next_node = heappop(self.mem.priority_queue)

            if grid.is_goal_reached(next_node):
                heappush(self.mem.priority_queue, (total_dist, next_node))
                return True

            self.mem.visited.add(next_node)

            for n, idx in grid.get_next_positions_with_move_index(next_node):
                if n not in self.mem.visited:
                    dist = grid.get_movement_cost_from_index(idx)
                    if n not in self.mem.g or self.mem.g[next_node] + dist < self.mem.g[n]:
                        # it does not matter if we have duplicates as we will not be looking at them
                        # therefore it does not affect the priority
                        self.mem.g[n] = self.mem.g[next_node] + dist
                        item = (self.f(n), n)
                        heappush(self.mem.priority_queue, item)
                        self.mem.back_pointer[n] = next_node

            self.key_frame()
        return False

    def f(self, x: Point) -> float:
        g = self.mem.g[x]
        h = self.get_heuristic(x)
        ret = g + h
        return ret

    def _follow_back_trace(self):
        trace: List[Point] = self.get_back_trace(self._get_grid().goal)
        trace.reverse()
        for t in trace:
            self.move_agent(t)
            self.key_frame(ignore_key_frame_skip=True)

    def get_back_trace(self, goal: Goal) -> List[Point]:
        """
        Follows the back pointer until it gets to the agent position
        :return: The trace
        """
        trace = []
        pos = goal.position
        while self.mem.back_pointer[pos] is not None:
            trace.append(pos)
            pos = self.mem.back_pointer[pos]
        return trace

    def get_heuristic(self, pos: Point) -> float:
        """
        Returns the euclidean distance from the given position to the goal
        It does memoization as well
        :param goal: The goal
        :param pos: The initial position
        :return:
        """
        self.mem.h.setdefault(pos, np.linalg.norm(np.array(pos) - np.array(self._get_grid().goal.position)))
        return self.mem.h[pos]
