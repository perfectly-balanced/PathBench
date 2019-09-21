from typing import Set, List, Callable, Optional

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
        self.__root_vertex.cost: float = 0
        self.__size: int = 1

    def add_edge(self, parent: Vertex, child: Optional['Vertex']):
        child_parent_dist = torch.norm(parent.position.to_tensor() - child.position.to_tensor())
        child.cost = parent.cost + child_parent_dist
        parent.add_child(child)
        child.set_parent(parent)
        self.__size += 1

    def remove_edge(self, parent: Vertex, child: Optional['Vertex']):
        child.cost = None
        parent.remove_child(child)
        self.__size -= 1

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

    def get_vertices_within_radius(self, vertex: Vertex, radius: float) -> List[Vertex]:
        def get_within_radius(current: Vertex, __acc) -> bool:
            dist: float = torch.norm(vertex.position.to_tensor() - current.position.to_tensor())
            if dist <= radius:
                __acc.append(current)
            return True
        acc: List[Vertex] = list()
        self.walk_dfs(lambda current: get_within_radius(current, acc))
        return acc

    @property
    def size(self) -> int:
        return self.__size

class RRT_Star(Algorithm):
    __graph: Graph

    def __init__(self, services: Services, testing: BasicTesting = None) -> None:
        super().__init__(services, testing)
        self.__graph = Graph(self._get_grid().agent.position)

    def set_display_info(self) -> List[MapDisplay]:
        return super().set_display_info() + [GraphMapDisplay(self._services, self.__graph)]

    def _find_path_internal(self) -> None:
        max_dist: float = 10
        iterations: int = 10000
        max_radius: float = 50
        lambda_rrt_star: float = 50
        dimension = 2
        for i in range(iterations):
        #while True:

            q_sample: Point = self.__get_random_sample()
            q_nearest: Vertex = self.__get_nearest_vertex(q_sample)
            if q_nearest.position == q_sample:
                continue
            q_new: Vertex = self.__get_new_vertex(q_nearest, q_sample, max_dist)

            if not self._get_grid().is_valid_line_sequence(self._get_grid().get_line_sequence(q_nearest.position, q_new.position)):
                continue

            card_v = torch.tensor(float(self.__graph.size))
            log_card_v = torch.log(card_v)
            radius = min(lambda_rrt_star*((log_card_v/card_v)**(1/dimension)),max_radius)
            Q_near: List[Vertex] = self.__get_vertices_within_radius(q_new, radius)
            q_min = q_nearest
            c_min = q_nearest.cost + torch.norm(q_nearest.position.to_tensor() - q_new.position.to_tensor())

            for q_near in Q_near:
                near_new_collision_free = self._get_grid().is_valid_line_sequence(self._get_grid().get_line_sequence(q_near.position, q_new.position))
                cost_near_to_new = q_near.cost + torch.norm(q_near.position.to_tensor() - q_new.position.to_tensor())
                if near_new_collision_free and cost_near_to_new < c_min:
                    q_min = q_near
                    c_min = cost_near_to_new

            self.__graph.add_edge(q_min, q_new)

            for q_near in Q_near:
                near_new_collision_free = self._get_grid().is_valid_line_sequence(self._get_grid().get_line_sequence(q_near.position, q_new.position))
                cost_new_to_near = q_new.cost + torch.norm(q_new.position.to_tensor() - q_near.position.to_tensor())
                if near_new_collision_free and cost_new_to_near < q_near.cost:
                    q_parent = None
                    for parent in q_near.parents:
                        q_parent = parent
                        break
                    self.__graph.remove_edge(q_parent, q_near)
                    self.__graph.add_edge(q_new, q_near)

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

    def __get_vertices_within_radius(self, vertex: Vertex, radius: float) -> List[Vertex]:
        return self.__graph.get_vertices_within_radius(vertex, radius)

    @staticmethod
    def __get_new_vertex(q_near: Vertex, q_sample: Point, max_dist) -> Vertex:
        dir = q_sample.to_tensor() - q_near.position.to_tensor()
        dir_norm = torch.norm(dir)
        if dir_norm <= max_dist:
            return Vertex(q_sample)

        dir_normalized = dir / torch.norm(dir)
        q_new = Point.from_tensor(q_near.position.to_tensor() + max_dist * dir_normalized)
        return Vertex(q_new)
