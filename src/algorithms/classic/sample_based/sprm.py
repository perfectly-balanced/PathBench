from typing import List

import torch

from algorithms.classic.sample_based.core.sample_based_algorithm import SampleBasedAlgorithm
from algorithms.basic_testing import BasicTesting
from simulator.services.services import Services
from structures import Point

from algorithms.classic.sample_based.core.vertex import Vertex
from algorithms.classic.sample_based.core.graph import gen_cyclic_graph, CyclicGraph


class SPRM(SampleBasedAlgorithm):
    _graph: CyclicGraph
    _V_size: int
    _max_radius: float

    def __init__(self, services: Services, testing: BasicTesting = None) -> None:
        super().__init__(services, testing)
        self._V_size = 200
        self._max_radius = 15
        V: List[Vertex] = list()
        for i in range(self._V_size):
            q_rand: Point = self._get_random_sample()
            V.append(Vertex(q_rand, store_connectivity=True))
        self._graph = gen_cyclic_graph(self._services,
                                       Vertex(self._get_grid().agent.position, store_connectivity=True),
                                       Vertex(self._get_grid().goal.position, store_connectivity=True),
                                       V)
        self._graph.edges_removable = False
        self._init_displays()

    # Helper Functions #
    # -----------------#

    def _near(self, vertex: Vertex) -> List[Vertex]:
        return self._graph.get_vertices_within_radius(self._graph.root_vertices, vertex.position, self._max_radius)

    def _get_random_sample(self) -> Point:
        while True:
            sample = Point(*[torch.randint(0, self._get_grid().size[i], (1,)).item() for i in range(self._get_grid().size.n_dim)])
            if self._get_grid().is_agent_valid_pos(sample):
                return sample

    def _get_new_vertex(self, q_near: Vertex, q_sample: Point, max_dist) -> Vertex:
        dir = q_sample.to_tensor() - q_near.position.to_tensor()
        if torch.norm(dir) <= max_dist:
            return Vertex(q_sample)

        dir_normalized = dir / torch.norm(dir)
        q_new = Point.from_tensor(q_near.position.to_tensor() + max_dist * dir_normalized)
        return Vertex(q_new)

    def _extract_path(self):

        goal: Vertex = self._graph.root_vertices[1]
        agent: Vertex = self._graph.root_vertices[0]

        current_vertex = agent
        path = list()
        while current_vertex is not goal:
            current_vertex = current_vertex.connectivity[goal]
            path.append(current_vertex)

        for p in path:
            self.move_agent(p.position)
            self.key_frame(ignore_key_frame_skip=True)

    # Overridden Implementation #
    # --------------------------#

    def _find_path_internal(self) -> None:

        for i, v in enumerate(self._graph.root_vertices):
            U = self._near(v)
            for u in U:
                if self._get_grid().is_valid_line_sequence(self._get_grid().get_line_sequence(u.position, v.position)):
                    if v is not u:
                        self._graph.add_edge(v, u)
                        self._graph.add_edge(u, v)
                    self.key_frame()
                    if self._graph.root_vertices[1] in self._graph.root_vertices[0].connectivity:
                        self._extract_path()
                        return
