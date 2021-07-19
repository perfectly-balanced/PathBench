from typing import Set, List, Tuple, Optional, Dict

import numpy as np

from algorithms.algorithm import Algorithm
from algorithms.basic_testing import BasicTesting
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.maps.map import Map
from algorithms.configuration.maps.ros_map import RosMap
from simulator.services.services import Services
from simulator.views.map.display.gradient_list_map_display import GradientListMapDisplay
from simulator.views.map.display.map_display import MapDisplay
from simulator.views.map.display.solid_iterable_map_display import SolidIterableMapDisplay
from structures import Point, Colour, BLUE, DynamicColour
from structures.factory import gen_set, gen_list


"""
Child-Generator-Deque-Search
"""


class CGDS(Algorithm):
    class InternalMemory:
        deque: List[Tuple[Point, float, Point, List[Tuple[Point, float]]]]
        dequeVisual: List[Tuple[int, Point]]
        visited: Set[Point]
        back_pointer: Dict[Point, Optional[Point]]
        costs: Dict[Point, float]
        h: Dict[Point, float]

        def __init__(self, services: Services):

            self.deque = gen_list(services)
            self.dequeVisual = gen_list(services)
            self.visited = gen_set(services)
            self.back_pointer = {}
            self.costs = {}
            self.h = {}

    mem: InternalMemory

    pq_colour_max: DynamicColour
    pq_colour_min: DynamicColour
    visited_colour: DynamicColour

    __map_displays: List[MapDisplay]

    def __init__(self, services: Services, testing: BasicTesting = None):
        super().__init__(services, testing)

        self.mem = CGDS.InternalMemory(self._services)

        self.pq_colour_max = self._services.state.views.add_colour("explored max", BLUE)
        self.pq_colour_min = self._services.state.views.add_colour("explored min", Colour(0.27, 0.33, 0.35, 0.2))
        self.visited_colour = self._services.state.views.add_colour("visited", Colour(0.19, 0.19, 0.2, 0.8))

        self.__map_displays = [SolidIterableMapDisplay(self._services, self.mem.visited, self.visited_colour, z_index=50),
                               GradientListMapDisplay(self._services, self.mem.dequeVisual, min_colour=self.pq_colour_min,
                                                  max_colour=self.pq_colour_max, z_index=49, inverted=True)
                               ]

    def set_display_info(self) -> List[MapDisplay]:
        """
        Read super description
        """
        return super().set_display_info() + self.__map_displays

    # noinspection PyUnusedLocal
    # @profile
    def _find_path_internal(self) -> None:
        self._init_mem()

        if self._expand():
            self._follow_back_trace()

    def _init_mem(self) -> None:
        grid: Map = self._get_grid()

        # push agent
        self.mem.deque.insert(0, (grid.agent.position, 0.0, None, None))
        self.mem.back_pointer[grid.agent.position] = None

    def _expand(self) -> bool:
        grid: Map = self._get_grid()

        while len(self.mem.deque) > 0:
            parentNode: Point
            childrenNodes: List[Tuple[Point, float]]
            parentNode, cost, prev, childrenNodes = self.mem.deque.pop(0)

            if childrenNodes is None:
                # New parent Node whose children haven't been generated yet

                # Check to see if cost is lower or equal to what has been seen before
                if parentNode in self.mem.costs and self.mem.costs[parentNode] <= cost:
                    continue  # skip since cost isn't lower than previously seen

                self.mem.costs[parentNode] = cost
                self.mem.back_pointer[parentNode] = prev
                self.mem.visited.add(parentNode)

                # Check if we've reached the goal
                if grid.is_goal_reached(parentNode):
                    return True

                # Generate children:
                children = []
                for child, idx in grid.get_next_positions_with_move_index(parentNode):
                    dist = grid.get_movement_cost_from_index(idx)
                    children.append((self.get_heuristic(child), dist, child))
                childrenNodes = [(child, dist) for c, dist, child in sorted(children)]

                # Visualisation:
                listOrder = [node for node, _, _, _ in self.mem.deque]
                self.mem.dequeVisual.clear()
                self.mem.dequeVisual.extend([(i, n) for i, n in enumerate(listOrder)])

                self.key_frame()

            if len(childrenNodes) == 0:
                # No children Nodes so go back and pop a new parent node
                continue
            else:
                # Get the next child and add them to the front of the deque:
                nextNode, dist = childrenNodes.pop(0)
                self.mem.deque.insert(0, (nextNode, cost+dist, parentNode, None))
                # Add parent Node to the back of the queue
                self.mem.deque.append((parentNode, cost, prev, childrenNodes))

        # Reaching this means the deque is empty but no solutions have been found:
        return False

    def _follow_back_trace(self):
        grid: Map = self._get_grid()
        
        trace: List[Point] = self.get_back_trace(grid.goal)
        trace.reverse()
        for t in trace:
            self.move_agent(t)
            if isinstance(grid, RosMap):
                grid.publish_wp(grid.agent.position)
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
