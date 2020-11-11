from typing import List

import torch
import numpy as np

from memory_profiler import profile

from algorithms.classic.sample_based.core.sample_based_algorithm import SampleBasedAlgorithm
from algorithms.basic_testing import BasicTesting
from simulator.services.services import Services
from structures import Point

from algorithms.classic.sample_based.core.vertex import Vertex
from algorithms.classic.sample_based.core.graph import gen_forest, Forest

n=0

class RRT_Connect(SampleBasedAlgorithm):
    _graph: Forest
    _max_dist: float
    _iterations: int

    def __init__(self, services: Services, testing: BasicTesting = None) -> None:
        super().__init__(services, testing)
        
        self._graph = gen_forest(self._services, Vertex(self._get_grid().agent.position), Vertex(self._get_grid().goal.position), [])
        self._graph.edges_removable = False
        self._init_displays()
        
        self._max_dist = 10
        self._iterations = 10000

    # Helper Functions #
    # -----------------#

    def _extend(self, root_vertex: Vertex, q: Point) -> str:
        self._q_near: Vertex = self._get_nearest_vertex(root_vertex, q)
        self._q_new: Vertex = self._get_new_vertex(self._q_near, q, self._max_dist)
        if self._get_grid().is_valid_line_sequence(self._get_grid().get_line_sequence(self._q_near.position, self._q_new.position)):
            self._graph.add_edge(self._q_near, self._q_new)
            if self._q_new.position == q:
                return 'reached'
            else:
                return 'advanced'
        return 'trapped'

    def _connect(self, root_vertex: Vertex, q: Vertex) -> str:
        S = 'advanced'
        while S == 'advanced':
            S = self._extend(root_vertex, q.position)
        self._mid_vertex = q
        return S

    def _extract_path(self):

        # trace back
        path_mid_to_b: List[Vertex] = [self._q_new]

        while len(path_mid_to_b[-1].parents) != 0:
            for parent in path_mid_to_b[-1].parents:
                path_mid_to_b.append(parent)
                break

        path_a_to_mid: List[Vertex] = [self._extension_target]

        while len(path_a_to_mid[-1].parents) != 0:
            for parent in path_a_to_mid[-1].parents:
                path_a_to_mid.append(parent)
                break

        path_a_to_mid.reverse()
        path = path_a_to_mid + path_mid_to_b

        if self._graph.root_vertices[0] is self._graph.root_vertex_goal:
            path.reverse()

        for p in path:
            self.move_agent(p.position)
            self.key_frame(ignore_key_frame_skip=True)

    def _get_random_sample(self) -> Point:
        while True:
            rand_pos = np.random.randint(0, self._get_grid().size, self._get_grid().size.n_dim)
            sample: Point = Point(*rand_pos)
            if self._get_grid().is_agent_valid_pos(sample):
                return sample

    def _get_nearest_vertex(self, graph_root_vertex: Vertex, q_sample: Point) -> Vertex:
        return self._graph.get_nearest_vertex([graph_root_vertex], q_sample)

    def _get_new_vertex(self, q_near: Vertex, q_sample: Point, max_dist) -> Vertex:
        dir = q_sample.to_tensor() - q_near.position.to_tensor()
        if torch.norm(dir) <= max_dist:
            return Vertex(q_sample)

        dir_normalized = dir / torch.norm(dir)
        q_new = Point.from_tensor(q_near.position.to_tensor() + max_dist * dir_normalized)
        return Vertex(q_new)

    # Overridden Implementation #
    # --------------------------#

    #@profile
    def _find_path_internal(self) -> None:

        for i in range(self._iterations):
            global n 
            n+=1

            if n == 100:
                break

            q_rand: Point = self._get_random_sample()

            if not self._extend(self._graph.root_vertices[0], q_rand) == 'trapped':
                self._extension_target = self._q_new
                if self._connect(self._graph.root_vertices[-1], self._q_new) == 'reached':
                    self._extract_path()
                    break
            self._graph.reverse_root_vertices()

            # visualization code
            self.key_frame()