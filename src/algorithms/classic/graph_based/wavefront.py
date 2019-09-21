from typing import List, Tuple

import numpy as np

from algorithms.algorithm import Algorithm
from algorithms.basic_testing import BasicTesting
from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map_displays.gradient_map_display import GradientMapDisplay
from simulator.views.map_displays.map_display import MapDisplay
from simulator.views.map_displays.numbers_map_display import NumbersMapDisplay
from structures import Point


class Wavefront(Algorithm):
    step_grid: List[List[int]]
    STEP_GRID_MIN_COLOR = np.array([255, 255, 255])
    STEP_GRID_MAX_COLOR = np.array([0, 0, 255])

    def __init__(self, services: Services, testing: BasicTesting = None):
        super().__init__(services, testing)
        self.step_grid = []

    def set_display_info(self) -> List[MapDisplay]:
        """
        Read super description
        """
        display_info: List[MapDisplay] = super().set_display_info() + [
            GradientMapDisplay(self._services, self.step_grid,
                                               min_color=Wavefront.STEP_GRID_MIN_COLOR,
                                               max_color=Wavefront.STEP_GRID_MAX_COLOR),
            #NumbersMapDisplay(self._services, copy.deepcopy(self.step_grid))
        ]
        return display_info

    # noinspection PyUnusedLocal
    def _find_path_internal(self) -> None:
        """
        Read super description
        """
        grid: Map = self._get_grid()
        agent: Agent = grid.agent
        goal: Goal = grid.goal
        queue: List[Tuple[Point, int]] = [(goal.position, 2)]
        self.step_grid = [[0 for _ in range(grid.size.width)] for _ in range(grid.size.height)]
        self.step_grid[goal.position.y][goal.position.x] = 1
        agent_reached: bool = self.__equal_pos(goal.position, agent.position)

        while len(queue) > 0 and not agent_reached:
            next_node: Point
            count: int
            next_node, count = queue.pop(0)
            for n in grid.get_next_positions(next_node):
                if self.step_grid[n.y][n.x] == 0:
                    self.step_grid[n.y][n.x] = count
                    queue.append((n, count + 1))
                if self.__equal_pos(n, agent.position):
                    agent_reached = True
                    self.step_grid[n.y][n.x] = count
                    break
            self.key_frame()

        if agent_reached:
            trace: List[Point] = self.gradient_descent(agent.position, grid)
            for t in trace:
                self.move_agent(t)
                self.key_frame(ignore_key_frame_skip=True)

    def __equal_pos(self, pos1: Point, pos2: Point) -> bool:
        return pos1.x == pos2.x and pos1.y == pos2.y

    def gradient_descent(self, current: Point, grid: Map) -> List[Point]:
        """
        Search backward from the given point the next lowest number until we reach the agent position
        :param current: The position from witch to start the descent
        :param grid: the map
        :return: The trace
        """
        trace: List[Point] = [current]

        while self.step_grid[current.y][current.x] != 1:
            for n in grid.get_next_positions(current):
                if self.step_grid[n.y][n.x] == self.step_grid[current.y][current.x] - 1:
                    trace.append(n)
                    current = n
                    break

        return trace[1:]
