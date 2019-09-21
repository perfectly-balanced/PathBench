from typing import List

import torch

from algorithms.algorithm import Algorithm
from algorithms.basic_testing import BasicTesting
from simulator.services.services import Services
from simulator.views.map_displays.graph_map_display import GraphMapDisplay
from simulator.views.map_displays.map_display import MapDisplay
from structures import Point

from algorithms.classic.sample_based.core.vertex import Vertex
from algorithms.classic.sample_based.core.graph import Graph


class RT(Algorithm):
    __graph: Graph

    def __init__(self, services: Services, testing: BasicTesting = None) -> None:
        super().__init__(services, testing)
        start_vertex = Vertex(self._get_grid().agent.position)
        goal_vertex = Vertex(self._get_grid().goal.position)
        self.__graph = Graph(start_vertex, goal_vertex, [])

    def set_display_info(self) -> List[MapDisplay]:
        return super().set_display_info() + [GraphMapDisplay(self._services, self.__graph)]

    def _find_path_internal(self) -> None:
        max_dist: float = 10
        iterations: int = 10000
        for i in range(iterations):
            q_sample: Point = self.__get_random_sample()
            q_near: Vertex = self.__get_random_vertex()
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

    def __get_random_vertex(self) -> Vertex:
        return self.__graph.get_random_vertex([self.__graph.root_vertex_start])

    @staticmethod
    def __get_new_vertex(q_near: Vertex, q_sample: Point, max_dist) -> Vertex:
        dir = q_sample.to_tensor() - q_near.position.to_tensor()
        if torch.norm(dir) <= max_dist:
            return Vertex(q_sample)

        dir_normalized = dir / torch.norm(dir)
        q_new = Point.from_tensor(q_near.position.to_tensor() + max_dist * dir_normalized)
        return Vertex(q_new)