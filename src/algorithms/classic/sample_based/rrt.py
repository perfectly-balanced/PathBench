from typing import List, Callable, Optional, Set

import torch

from algorithms.algorithm import Algorithm
from algorithms.basic_testing import BasicTesting
from simulator.services.services import Services
from simulator.views.map_displays.graph_map_display import GraphMapDisplay
from simulator.views.map_displays.map_display import MapDisplay
from structures import Point

from algorithms.classic.sample_based.core.vertex import Vertex

class Graph:
    __root_vertex: Vertex

    def __init__(self, agent_pos: Point) -> None:
        self.__root_vertex = Vertex(agent_pos)

    @staticmethod
    def add_edge(parent: Vertex, child: Optional['Vertex']):
        parent.add_child(child)
        child.set_parent(parent)

    def walk_dfs(self, f: Callable[[Vertex], bool]):
        self.__root_vertex.visit_children(f)

    def get_nearest_vertex(self, point: Point) -> Vertex:
        def get_nearest(current: Vertex, __acc) -> bool:
            dist: float = torch.norm(point.to_tensor() - current.position.to_tensor())
            if dist <= __acc[0]:
                __acc[0] = dist
                __acc[1] = current
                return True
            return False

        acc: [float, Vertex] = [float('inf'), self.__root_vertex]
        self.walk_dfs(lambda current: get_nearest(current, acc))
        return acc[1]


class RRT(Algorithm):
    __graph: Graph

    def __init__(self, services: Services, testing: BasicTesting = None) -> None:
        super().__init__(services, testing)
        self.__graph = Graph(self._get_grid().agent.position)

    def set_display_info(self) -> List[MapDisplay]:
        return super().set_display_info() + [GraphMapDisplay(self._services, self.__graph)]

    def _find_path_internal(self) -> None:
        max_dist: float = 10
        iterations: int = 10000
        for i in range(iterations):
        #while True:
            q_sample: Point = self.__get_random_sample()
            q_near: Vertex = self.__get_nearest_vertex(q_sample)
            if q_near.position == q_sample:
                continue
            q_new: Vertex = self.__get_new_vertex(q_near, q_sample, max_dist)

            if not self._get_grid().is_valid_line_sequence(self._get_grid().get_line_sequence(q_near.position, q_new.position)):
                continue

            self.__graph.add_edge(q_near, q_new)

            if self._get_grid().is_agent_in_goal_radius(agent_pos=q_new.position):
                goal_v: Vertex = Vertex(self._get_grid().goal.position)
                self.__graph.add_edge(q_new, goal_v)
                # trace back
                path: List[Vertex] = [goal_v]

                while len(path[-1].parents) != 0:
                    for parent in path[-1].parents:
                        path.append(parent)
                        break

                del path[-1]
                path.reverse()

                for p in path:
                    self.move_agent(p.position)
                    self.key_frame(ignore_key_frame_skip=True)
                break

            self.key_frame()

    def __get_random_sample(self) -> Point:
        while True:
            sample: Point = Point(torch.randint(0, self._get_grid().size.width, (1,)).item(),
                                  torch.randint(0, self._get_grid().size.height, (1,)).item())
            if self._get_grid().is_agent_valid_pos(sample):
                return sample

    def __get_nearest_vertex(self, q_sample: Point) -> Vertex:
        return self.__graph.get_nearest_vertex(q_sample)

    @staticmethod
    def __get_new_vertex(q_near: Vertex, q_sample: Point, max_dist) -> Vertex:
        dir = q_sample.to_tensor() - q_near.position.to_tensor()
        if torch.norm(dir) <= max_dist:
            return Vertex(q_sample)

        dir_normalized = dir / torch.norm(dir)
        q_new = Point.from_tensor(q_near.position.to_tensor() + max_dist * dir_normalized)
        return Vertex(q_new)
