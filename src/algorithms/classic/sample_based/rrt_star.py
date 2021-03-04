from typing import List

import torch
import numpy as np

from algorithms.classic.sample_based.core.sample_based_algorithm import SampleBasedAlgorithm
from algorithms.basic_testing import BasicTesting
from simulator.services.services import Services
from structures import Point
from algorithms.configuration.maps.ros_map import RosMap
from algorithms.configuration.maps.map import Map
from algorithms.classic.sample_based.core.vertex import Vertex
from algorithms.classic.sample_based.core.graph import gen_forest, Forest


class RRT_Star(SampleBasedAlgorithm):
    _graph: Forest

    def __init__(self, services: Services, testing: BasicTesting = None) -> None:
        super().__init__(services, testing)
        
        start_vertex = Vertex(self._get_grid().agent.position)
        start_vertex.cost = 0
        goal_vertex = Vertex(self._get_grid().goal.position)

        self._graph = gen_forest(self._services, start_vertex, goal_vertex, [])
        self._init_displays()

    # Helper Functions #
    # -----------------#

    def _get_random_sample(self) -> Point:
        while True:
            rand_pos = np.random.randint(0, self._get_grid().size, self._get_grid().size.n_dim)
            sample: Point = Point(*rand_pos)
            if self._get_grid().is_agent_valid_pos(sample):
                return sample

    def _get_nearest_vertex(self, q_sample: Point) -> Vertex:
        return self._graph.get_nearest_vertex([self._graph.root_vertex_start], q_sample)

    def _get_vertices_within_radius(self, vertex: Vertex, radius: float) -> List[Vertex]:
        return self._graph.get_vertices_within_radius([self._graph.root_vertex_start], vertex.position, radius)

    def _get_new_vertex(self, q_near: Vertex, q_sample: Point, max_dist) -> Vertex:
        dir = q_sample.to_tensor() - q_near.position.to_tensor()
        dir_norm = torch.norm(dir)
        if dir_norm <= max_dist:
            return Vertex(q_sample)

        dir_normalized = dir / torch.norm(dir)
        q_new = Point.from_tensor(q_near.position.to_tensor() + max_dist * dir_normalized)
        return Vertex(q_new)

    def _extract_path(self, q_new):

        goal_v: Vertex = Vertex(self._get_grid().goal.position)
        child_parent_dist = torch.norm(q_new.position.to_tensor() - goal_v.position.to_tensor())
        goal_v.cost = q_new.cost + child_parent_dist
        self._graph.add_edge(q_new, goal_v)
        path: List[Vertex] = [goal_v]

        while len(path[-1].parents) != 0:
            for parent in path[-1].parents:
                path.append(parent)
                break

        del path[-1]
        path.reverse()

        for p in path:
            self.move_agent(p.position)
            #sends waypoint for ros extension
            grid: Map = self._get_grid()
            if isinstance(grid, RosMap):
                grid.publish_wp(grid.agent.position)
            self.key_frame(ignore_key_frame_skip=True)

    # Overridden Implementation #
    # --------------------------#

    def _find_path_internal(self) -> None:

        max_dist: float = 10
        iterations: int = 10000
        max_radius: float = 50
        lambda_rrt_star: float = 50
        dimension = 2

        for i in range(iterations):

            q_sample: Point = self._get_random_sample()
            q_nearest: Vertex = self._get_nearest_vertex(q_sample)
            if q_nearest.position == q_sample:
                continue
            q_new: Vertex = self._get_new_vertex(q_nearest, q_sample, max_dist)

            if not self._get_grid().is_valid_line_sequence(self._get_grid().get_line_sequence(q_nearest.position, q_new.position)):
                continue

            card_v = torch.tensor(float(self._graph.size))
            log_card_v = torch.log(card_v)
            radius = min(lambda_rrt_star*((log_card_v/card_v)**(1/dimension)),max_radius)
            Q_near: List[Vertex] = self._get_vertices_within_radius(q_new, radius)
            q_min = q_nearest
            c_min = q_nearest.cost + torch.norm(q_nearest.position.to_tensor() - q_new.position.to_tensor())

            for q_near in Q_near:
                near_new_collision_free = self._get_grid().is_valid_line_sequence(self._get_grid().get_line_sequence(q_near.position, q_new.position))
                cost_near_to_new = q_near.cost + torch.norm(q_near.position.to_tensor() - q_new.position.to_tensor())
                if near_new_collision_free and cost_near_to_new < c_min:
                    q_min = q_near
                    c_min = cost_near_to_new

            child_parent_dist = torch.norm(q_min.position.to_tensor() - q_new.position.to_tensor())
            q_new.cost = q_min.cost + child_parent_dist
            self._graph.add_edge(q_min, q_new)

            for q_near in Q_near:
                near_new_collision_free = self._get_grid().is_valid_line_sequence(self._get_grid().get_line_sequence(q_near.position, q_new.position))
                cost_new_to_near = q_new.cost + torch.norm(q_new.position.to_tensor() - q_near.position.to_tensor())
                if near_new_collision_free and cost_new_to_near < q_near.cost:
                    q_parent = None
                    for parent in q_near.parents:
                        q_parent = parent
                        break
                    q_near.cost = None
                    self._graph.remove_edge(q_parent, q_near)
                    child_parent_dist = torch.norm(q_new.position.to_tensor() - q_near.position.to_tensor())
                    q_near.cost = q_new.cost + child_parent_dist
                    self._graph.add_edge(q_new, q_near)

            if self._get_grid().is_agent_in_goal_radius(agent_pos=q_new.position):
                self._extract_path(q_new)
                break

            self.key_frame()
