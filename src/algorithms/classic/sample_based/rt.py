from typing import List

import torch
import numpy as np

from algorithms.basic_testing import BasicTesting
from simulator.services.services import Services
from structures import Point

from algorithms.classic.sample_based.core.vertex import Vertex
from algorithms.classic.sample_based.core.graph import gen_forest, Forest
from algorithms.classic.sample_based.core.sample_based_algorithm import SampleBasedAlgorithm


class RT(SampleBasedAlgorithm):
    _graph: Forest

    def __init__(self, services: Services, testing: BasicTesting = None) -> None:
        super().__init__(services, testing)

        self._graph = gen_forest(self._services, Vertex(self._get_grid().agent.position), Vertex(self._get_grid().goal.position), [])
        self._graph.edges_removable = False
        self._init_displays()

    # Helper Functions #
    # -----------------#

    def _get_random_sample(self) -> Point:
        while True:
            rand_pos = np.random.randint(0, self._get_grid().size, self._get_grid().size.n_dim)
            sample: Point = Point(*rand_pos)
            if self._get_grid().is_agent_valid_pos(sample):
                return sample

    def _get_random_vertex(self) -> Vertex:
        return self._graph.get_random_vertex([self._graph.root_vertex_start])

    def _get_new_vertex(self, q_near: Vertex, q_sample: Point, max_dist) -> Vertex:
        dir = q_sample.to_tensor() - q_near.position.to_tensor()
        if torch.norm(dir) <= max_dist:
            return Vertex(q_sample)

        dir_normalized = dir / torch.norm(dir)
        q_new = Point.from_tensor(q_near.position.to_tensor() + max_dist * dir_normalized)
        return Vertex(q_new)

    def _extract_path(self, q_new):
        goal_v: Vertex = Vertex(self._get_grid().goal.position)
        self._graph.add_edge(q_new, goal_v)
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

    # Overridden Implementation #
    # --------------------------#

    def _find_path_internal(self) -> None:

        max_dist: float = 10
        iterations: int = 10000

        for i in range(iterations):

            q_sample: Point = self._get_random_sample()
            q_near: Vertex = self._get_random_vertex()
            if q_near.position == q_sample:
                continue
            q_new: Vertex = self._get_new_vertex(q_near, q_sample, max_dist)

            if not self._get_grid().is_valid_line_sequence(self._get_grid().get_line_sequence(q_near.position, q_new.position)):
                continue

            self._graph.add_edge(q_near, q_new)

            if self._get_grid().is_agent_in_goal_radius(agent_pos=q_new.position):
                self._extract_path(q_new)
                break

            self.key_frame()
