from typing import List, Callable, Optional, Set, Dict

import torch

from algorithms.algorithm import Algorithm
from algorithms.basic_testing import BasicTesting
from simulator.services.services import Services
from simulator.views.map_displays.graph_map_display import GraphMapDisplay
from simulator.views.map_displays.map_display import MapDisplay
from structures import Point

from algorithms.classic.sample_based.core.vertex import Vertex


class Graph:
    V: List[Vertex]

    def __init__(self, agent_pos: Point, goal_pos: Point, root_vertices: List[Vertex]) -> None:
        self.V = [Vertex(goal_pos, store_connectivity=True)] + [Vertex(agent_pos, store_connectivity=True)] + root_vertices

    @staticmethod
    def add_edge(parent: Vertex, child: Vertex):
        if child is not parent:
            parent.add_child(child)
            child.add_parent(parent)

    def walk_dfs(self, f: Callable[[Vertex], bool]):
        for vertex in self.V:
            if not f(vertex):
                return

    def get_vertices_within_radius(self, vertex: Vertex, radius: float) -> List[Vertex]:

        def get_within_radius(current: Vertex, __acc) -> bool:
            dist: float = torch.norm(vertex.position.to_tensor() - current.position.to_tensor())
            if dist <= radius:
                __acc.append(current)
            return True

        acc: List[Vertex] = list()
        for v in self.V:
            get_within_radius(v, acc)
        return acc


class SPRM(Algorithm):
    __graph: Graph
    __V_size: int
    __max_radius: float

    def __init__(self, services: Services, testing: BasicTesting = None) -> None:
        super().__init__(services, testing)
        self.__V_size = 200
        self.__max_radius = 15
        V: List[Vertex] = list()
        for i in range(self.__V_size):
            q_rand: Point = self.__get_random_sample()
            V.append(Vertex(q_rand, store_connectivity=True))
        self.__graph = Graph(self._get_grid().agent.position, self._get_grid().goal.position, V)

    def set_display_info(self) -> List[MapDisplay]:
        return super().set_display_info() + [GraphMapDisplay(self._services, self.__graph)]

    def __path(self):

        goal: Vertex = self.__graph.V[0]
        agent: Vertex = self.__graph.V[1]

        current_vertex = agent
        path = list()
        while current_vertex is not goal:
            current_vertex = current_vertex.connectivity[goal]
            path.append(current_vertex)

        for p in path:
            self.move_agent(p.position)
            self.key_frame(ignore_key_frame_skip=True)

    def __near(self, vertex: Vertex) -> List[Vertex]:
        return self.__graph.get_vertices_within_radius(vertex, self.__max_radius)


    def _find_path_internal(self) -> None:

        for i, v in enumerate(self.__graph.V):
            U = self.__near(v)
            for u in U:
                if self._get_grid().is_valid_line_sequence(self._get_grid().get_line_sequence(u.position, v.position)):
                    if v is not u:
                        self.__graph.add_edge(v, u)
                        self.__graph.add_edge(u, v)
                    self.key_frame()
                    if self.__graph.V[0] in self.__graph.V[1].connectivity:
                        self.__path()
                        return

    def __get_random_sample(self) -> Point:
        while True:
            sample: Point = Point(torch.randint(0, self._get_grid().size.width, (1,)).item(),
                                  torch.randint(0, self._get_grid().size.height, (1,)).item())
            if self._get_grid().is_agent_valid_pos(sample):
                return sample

    @staticmethod
    def __get_new_vertex(q_near: Vertex, q_sample: Point, max_dist) -> Vertex:
        dir = q_sample.to_tensor() - q_near.position.to_tensor()
        if torch.norm(dir) <= max_dist:
            return Vertex(q_sample)

        dir_normalized = dir / torch.norm(dir)
        q_new = Point.from_tensor(q_near.position.to_tensor() + max_dist * dir_normalized)
        return Vertex(q_new)
