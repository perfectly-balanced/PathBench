from typing import List, Set, Tuple, Optional

import numpy as np

from algorithms.algorithm import Algorithm
from simulator.views.map.display.map_display import MapDisplay
from structures import Point

"""
Does not work on map boundary
Can loop infinitely in phase2 first step if initial_position is not reached (add visited set()) 
"""


class Bug2(Algorithm):
    TOLERANCE: float = 1

    def set_display_info(self) -> List[MapDisplay]:
        """
        Read super description
        """
        return super().set_display_info()

    def _find_path_internal(self) -> None:
        """
        Read super description
        """
        self.phase1()

    # move towards goal
    def phase1(self) -> None:
        """
        First phase of the algorithms: go in the direction of the goal
        """
        while True:
            if self._get_grid().is_goal_reached(self._get_grid().agent.position):
                return

            direction: np.array = np.array(self._get_grid().goal.position) - np.array(self._get_grid().agent.position)
            next_move: Point = self._get_grid().get_move_along_dir(direction)
            next_pos: Point = self._get_grid().agent.position + next_move
            if self._get_grid().is_agent_valid_pos(next_pos):
                self.move_agent(next_pos)
            else:
                return self.phase2(self._get_grid().get_move_index(direction), next_pos,
                                   self._get_grid().agent.position)
            self.key_frame()

    # surround object
    # noinspection PyUnusedLocal,PyUnusedLocal,PyUnusedLocal
    def phase2(self, obstacle_dir_idx: int, cur_obstacle_pos: Point, initial_agent_pos: Point) -> None:
        """
        Second phase surround the object and when we get back to the goal direction go in first phase
        :param obstacle_dir_idx: The direction of the obstacle given as an index
        :param cur_obstacle_pos: The position of the obstacle that we hit
        :param initial_agent_pos: The agent position
        """

        def get_obstacle_dir_idx(__agent_dir_idx: int) -> int:
            """
            Returns the obstacle direction index from the agent one
            :param __agent_dir_idx: The agent direction index
            :return: The obstacle index
            """
            return (len(self._get_grid().ALL_POINTS_MOVE_VECTOR) + __agent_dir_idx - 1) % len(
                self._get_grid().ALL_POINTS_MOVE_VECTOR)

        def get_pos_from_dir_idx(dir_idx: int, pos: Point) -> Point:
            """
            Applies the given direction as a move to the given location
            :param dir_idx: The direction index
            :param pos: The initial position
            :return: The final position
            """
            return self._get_grid().apply_move(self._get_grid().ALL_POINTS_MOVE_VECTOR[dir_idx], pos)

        def get_obstacle_neighbours(obst_pos: Point, expand: int = 5) -> Set[Point]:
            """
            Returns the neighbours as 4 point connectivity (applies DFS)
            :param obst_pos: the position of the obstacle
            :param expand: Level of expansions
            :return: The set of neighbours
            """
            if expand == 0:
                return set()
            else:
                obsts = self._get_grid().get_four_point_neighbours(obst_pos)
                obsts = list(filter(lambda __o: not self._get_grid().is_agent_valid_pos(__o), obsts))
                sol: Set[Point] = set(obsts)
                for o in obsts:
                    sol = sol.union(get_obstacle_neighbours(o, expand - 1))
                return sol

        def walk_on_the_edge(__agent_dir_idx: int) -> Tuple[Optional[int], Optional[Point], Optional[Point], bool]:
            """
            Walks around the edge of the object
            :param __agent_dir_idx: The agent direction index
            :return: The next agent direction index, next agent position,
                     new obstacle position and if the algorithms should stop
            """
            visited.add(cur_pos)
            available_pos: Set[Point] = set(self._get_grid().get_next_positions(cur_pos))
            connected_obstacles: Set[Point] = get_obstacle_neighbours(cur_obstacle_pos)
            connected_obstacles.add(cur_obstacle_pos)
            prev_agent_dir_index: int = __agent_dir_idx
            while True:
                agent_pos: Point = get_pos_from_dir_idx(__agent_dir_idx, cur_pos)
                new_obstacle_pos = get_pos_from_dir_idx(get_obstacle_dir_idx(__agent_dir_idx), cur_pos)
                if agent_pos not in visited and agent_pos in available_pos and new_obstacle_pos in connected_obstacles:
                    visited.add(agent_pos)
                    return __agent_dir_idx, agent_pos, new_obstacle_pos, False
                __agent_dir_idx = (__agent_dir_idx + 1) % len(self._get_grid().ALL_POINTS_MOVE_VECTOR)
                if prev_agent_dir_index == __agent_dir_idx:
                    return __agent_dir_idx, agent_pos, new_obstacle_pos, True

        agent_dir_idx: int = (obstacle_dir_idx + 1) % len(self._get_grid().ALL_POINTS_MOVE_VECTOR)
        cur_pos: Point = self._get_grid().agent.position
        visited: Set[Point] = set()
        while True:
            next_pos: Optional[Point]
            should_stop: bool
            agent_dir_idx, next_pos, cur_obstacle_pos, should_stop = walk_on_the_edge(agent_dir_idx)
            if should_stop:
                return
            cur_pos = next_pos
            self.move_agent(cur_pos)

            vec: np.array = np.array(self._get_grid().goal.position) - np.array(cur_pos)
            vec = vec / np.linalg.norm(vec)
            p1: np.array = np.array(initial_agent_pos)
            p2: np.array = np.array(self._get_grid().goal.position)
            p3: np.array = np.array(cur_pos)
            dist: float = np.linalg.norm(np.cross(p2 - p1, p1 - p3)) / np.linalg.norm(p2 - p1)
            if dist < self.TOLERANCE:
                break
            self.key_frame()
        self.phase1()
