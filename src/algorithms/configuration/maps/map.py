import copy
from typing import List, Optional, Set, Dict, Callable, TypeVar, Union, Tuple

import math
import numpy as np

from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.entities.obstacle import Obstacle
from algorithms.configuration.entities.trace import Trace
from simulator.services.services import Services
from structures import Point, Size


class Map:
    """
    This class represents the environment in which the agent can move.
    It contains 1 agent, 1 goal and multiple obstacles.
    All maps must inherit this class.
    """

    size: Size
    agent: Agent
    goal: Goal
    obstacles: List[Obstacle]
    trace: List[Trace]
    _services: Optional[Services]
    __cached_move_costs: List[float] = []

    EIGHT_POINTS_MOVE_VECTOR: List[Point] = [
        Point(1, 0),
        Point(1, -1),
        Point(0, -1),
        Point(-1, -1),
        Point(-1, 0),
        Point(-1, 1),
        Point(0, 1),
        Point(1, 1),
    ]

    FOUR_POINTS_MOVE_VECTOR: List[Point] = [
        Point(1, 0),
        Point(0, -1),
        Point(-1, 0),
        Point(0, 1),
    ]

    def __init__(self, size: Size, services: Services = None) -> None:
        """
        :param size: The map size
        :param services: The simulator services
        """
        self._services = services
        self.size = size
        self.trace = []
        self.agent: None
        self.goal: None
        self.obstacles = []

    def get_obstacle_bound(self, obstacle_start_point: Point, visited: Optional[List[List[bool]]] = None) -> Set[Point]:
        """
        Method for finding the bound of an obstacle by doing a fill
        :param obstacle_start_point: The obstacle location
        :param visited: Can pass own visited set so that we do not visit those nodes again
        :return: Obstacle bound
        """
        if visited is None:
            visited = [[False for _ in range(self.size.width)] for _ in range(self.size.height)]

        if self.is_agent_valid_pos(obstacle_start_point):
            self._services.debug.write_error("NextPos should be invalid")
        st: List[Point] = [obstacle_start_point]
        bounds: Set[Point] = set()
        while len(st) > 0:
            next_node: Point = st.pop()
            visited[next_node.y][next_node.x] = True
            obstacle_count: int = 0
            for n in self.get_neighbours(next_node):
                if self.is_agent_valid_pos(n):
                    continue
                obstacle_count += 1
                if not visited[n.y][n.x]:
                    st.append(n)
            if obstacle_count < len(self.EIGHT_POINTS_MOVE_VECTOR):
                # we are on the edge
                bounds.add(next_node)
        return bounds

    @staticmethod
    def get_move_index(direction: List[float]) -> int:
        """
        Method for getting the move index from a direction
        :param direction: The direction
        :return: The index corresponding to :ref:`EIGHT_POINTS_MOVE_VECTOR`
        """
        # because pygame has inverted y
        true_direction: List[float] = [direction[0], -direction[1]]
        angle: float = np.arctan2(true_direction[1], true_direction[0])
        angle += 2 * np.pi if angle < 0 else 0
        return int(
            np.round(angle / (2 * np.pi / len(Map.EIGHT_POINTS_MOVE_VECTOR))) % len(Map.EIGHT_POINTS_MOVE_VECTOR))

    @staticmethod
    def get_move_along_dir(direction: List[float]) -> Point:
        """
        Method for getting the movement direction from a normal direction
        :param direction: The true direction
        :return: The :ref:`EIGHT_POINTS_MOVE_VECTOR` point
        """
        return Map.EIGHT_POINTS_MOVE_VECTOR[Map.get_move_index(direction)]

    def reset(self) -> None:
        """
        Resets the map by replaying the trace
        """
        if len(self.trace) > 0:
            self.move_agent(self.trace[0].position, True)
        self.trace.clear()

    def move(self, entity: Entity, to: Point, no_trace: bool = False) -> bool:
        """
        All maps should override this method
        :param entity: The entity to move (currently only agent is supported)
        :param to: The destination
        :param no_trace: If we want to add the movement to the map history
        """

        if not no_trace:
            self.trace.append(Trace(self.agent.position))

        return True

    def move_agent(self, to: Point, no_trace: bool = False, follow: bool = False) -> bool:
        """
        Method for moving the agent
        :param to: The destination
        :param no_trace: If we want to add the movement to the map history
        :param follow: Instead of teleport, moves in a straight line
        """

        if not follow:
            return self.move(self.agent, to, no_trace)
        # else:
        #     line: List[Point] = self.get_line_sequence(self.agent.position, to)
        #     for next_pos in line:
        #         if not self.move(self.agent, next_pos, no_trace):
        #             return False
        #     return True        
                    # for pt in EIGHT_POINTS_MOVE_VECTOR:
                    #     inx = int(next_pos.x + point.x)
                    #     iny = int(next_pos.y + point.y)
        else:
            line: List[Point] = self.get_line_sequence(self.agent.position, to)
            print(line)
            n=0
            for next_pos in line:
                if not self.move(self.agent, next_pos, no_trace):
                    if n == 0:
                        continue     
                    if n is not 0: 
                        return False         
            return True        

    def get_line_sequence(self, frm: Point, to: Point) -> List[Point]:
        '''
        Bresenham's line algorithm:
        Given coordinate of two points A(x1, y1) and B(x2, y2). The task to find all the intermediate points required for 
        drawing line AB .
        '''

        x1 = frm.x
        y1 = frm.y
        x2 = to.x
        y2 = to.y

        steep = math.fabs(y2 - y1) > math.fabs(x2 - x1)
        if steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        # takes left to right
        also_steep = x1 > x2
        if also_steep:
            x1, x2 = x2, x1
            y1, y2 = y2, y1

        dx = x2 - x1
        dy = math.fabs(y2 - y1)
        error = 0.0
        delta_error = 0.0
        # Default if dx is zero
        if dx != 0:
            delta_error = math.fabs(dy / dx)

        y_step = 1 if y1 < y2 else -1

        y = y1
        ret = []
        for x in range(x1, x2):
            p = Point(y, x) if steep else Point(x, y)
            if len(ret) != 0:
                if ret[-1].x - p.x != 0 and ret[-1].y - p.y != 0:
                    # make it not directionally conditioned
                    if ret[-1].x > p.x:
                        ret.append(Point(ret[-1].x, p.y))
                    else:
                        ret.append(Point(p.x, ret[-1].y))
            ret.append(p)
            error += delta_error
            if error >= 0.5:
                y += y_step
                error -= 1
        if also_steep:  # because we took the left to right instead
            ret.reverse()
        return ret + [to]



    def is_valid_line_sequence(self, line_sequence: List[Point]) -> bool:
        for pt in line_sequence:
            if not self.is_agent_valid_pos(pt):
                return False
        return True

    def is_goal_reached(self, pos: Point, goal: Goal = None) -> bool:
        """
        Method that checks if the position coincides with the goal
        :param pos: The position
        :param goal: If not default goal is considered
        :return: If the goal has been reached from the given position
        """

        if not goal:
            goal = self.goal

        return pos.x == goal.position.x and pos.y == goal.position.y

    def is_agent_in_goal_radius(self, agent_pos: Point = None, goal: Goal = None) -> bool:
        if agent_pos is None:
            agent_pos = self.agent.position
        if goal is None:
            goal = self.goal
        dist: np.ndarray = np.linalg.norm(np.array(agent_pos) - np.array(goal.position))
        return dist <= (goal.radius + self.agent.radius) or dist < 0.0001

    def is_agent_valid_pos(self, pos: Point) -> bool:
        """
        Checks if the point is a valid position for the agent
        :param pos: The position
        :return: If the point is a valid position for the agent
        """
        return not self.is_out_of_bounds_pos(pos)

    def is_out_of_bounds_pos(self, pos: Point) -> bool:
        """
        Checks if the position is out of bounds
        :param pos: The position
        :return: If the position is out of bounds
        """
        return pos.x < 0 or pos.x >= self.size.width or pos.y < 0 or pos.y >= self.size.height

    def get_next_positions(self, pos: Point) -> List[Point]:
        """
        Returns the next available positions valid from the agent point of view given 8-point connectivity
        :param pos: The position
        :return: A list of positions
        """
        ns: List[Point] = []
        for move in self.EIGHT_POINTS_MOVE_VECTOR:
            n: Point = self.apply_move(move, pos)
            if self.is_agent_valid_pos(n):
                ns.append(n)
        return ns

    def get_next_positions_with_move_index(self, pos: Point) -> List[Tuple[Point, int]]:
        """
        Returns the next available positions valid from the agent point of view given 8-point connectivity
        :param pos: The position
        :return: A list of positions with move index
        """
        ns: List[Tuple[Point, int]] = []
        for idx, move in enumerate(self.EIGHT_POINTS_MOVE_VECTOR):
            n: Point = self.apply_move(move, pos)
            if self.is_agent_valid_pos(n):
                ns.append((n, idx))
        return ns

    def get_neighbours(self, pos: Point) -> List[Point]:
        """
        Returns the neighbours (8-point connectivity)
        :param pos: The position
        :return: The neighbours
        """
        ns: List[Point] = []
        for move in self.EIGHT_POINTS_MOVE_VECTOR:
            n: Point = self.apply_move(move, pos)
            if not self.is_out_of_bounds_pos(n):
                ns.append(n)
        return ns

    def get_neighbours_with_move_index(self, pos: Point) -> List[Tuple[Point, int]]:
        """
        Returns the neighbours (8-point connectivity) with move index
        :param pos: The position
        :return: The neighbours with move index
        """
        ns: List[Tuple[Point, int]] = []
        for idx, move in enumerate(self.EIGHT_POINTS_MOVE_VECTOR):
            n: Point = self.apply_move(move, pos)
            if not self.is_out_of_bounds_pos(n):
                ns.append((n, idx))
        return ns

    def get_four_point_neighbours(self, pos: Point) -> List[Point]:
        """
        Returns the neighbours (4-point connectivity)
        :param pos: the position
        :return: The neighbours
        """
        ns: List[Point] = []
        for move in self.FOUR_POINTS_MOVE_VECTOR:
            n: Point = self.apply_move(move, pos)
            if not self.is_out_of_bounds_pos(n):
                ns.append(n)
        return ns

    T = TypeVar('T')

    def replay_trace(self, f: Callable[['Map'], T]) -> List[T]:
        if not self.trace:
            return []

        rem_trace: List[Point] = list(map(lambda tr: tr.position, self.trace))
        rem_trace.append(self.agent.position)
        ret: List[any] = []
        self.trace.clear()

        self.move_agent(rem_trace.pop(0), True)
        ret.append(f(self))

        while rem_trace:
            self.move_agent(rem_trace.pop(0))
            ret.append(f(self))

        return ret[:-1]

    @staticmethod
    def apply_move(move: Point, pos: Point) -> Point:
        """
        Applies the given move and returns the new destination
        :param move: The move to apply
        :param pos: The source
        :return: The destination after applying the move
        """
        return Point(pos.x + move.x, pos.y + move.y)

    def get_movement_cost(self, frm: Union[Point, Entity] = None, to: Union[Point, Entity] = None) -> float:
        if frm is None:
            frm = self.agent

        if to is None:
            to = self.goal

        if isinstance(frm, Entity):
            frm = frm.position

        if isinstance(to, Entity):
            to = frm.position

        return self.get_distance(frm, to)

    def get_movement_cost_from_index(self, idx: int) -> float:
        if not self.__cached_move_costs:
            self.__cached_move_costs = list(map(lambda p: self.get_movement_cost(Point(0, 0), p), self.EIGHT_POINTS_MOVE_VECTOR))
        return self.__cached_move_costs[idx]

    @staticmethod
    def get_distance(frm: Point, to: Point) -> float:
        return np.linalg.norm(np.array(frm) - np.array(to))

    @property
    def services(self) -> str:
        return 'services'

    @services.setter
    def services(self, new_value: Services) -> None:
        self._services = new_value

    def __copy__(self) -> 'Map':
        return copy.deepcopy(self)

    def __deepcopy__(self, memo: Dict) -> 'Map':
        raise Exception("Override this")

    # noinspection PyPep8
    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Map):
            return False
        return self.size == other.size and \
               self.trace == other.trace and \
               self.agent == other.agent and \
               self.goal == other.goal and \
               self.obstacles == other.obstacles

    def __ne__(self, other: object) -> bool:
        return not self.__eq__(other)
