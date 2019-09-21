from typing import Callable, List
from algorithms.classic.sample_based.core.vertex import Vertex
from structures import Point
import torch


class Graph:
    __root_vertex_start__: Vertex
    __root_vertex_goal__: Vertex
    __root_vertices__: List[Vertex]

    def __init__(self, start_pos: Vertex, goal_pos: Vertex, root_vertices: List[Vertex]) -> None:
        self.root_vertex_start = start_pos
        self.root_vertex_goal = goal_pos
        self.root_vertices = [self.root_vertex_start] + [self.root_vertex_goal] + root_vertices
        self.__size: int = 1

    def add_edge(self, parent: Vertex, child: Vertex):
        parent.add_child(child)
        child.set_parent(parent)
        self.__size += 1

    def remove_edge(self, parent: Vertex, child: Vertex):
        parent.remove_child(child)
        child.remove_parent(parent)
        self.__size -= 1

    def reverse_root_vertices(self):
        self.root_vertices.reverse()

    def walk_dfs_subset_of_vertices(self, root_vertices_subset: List[Vertex], f: Callable[[Vertex], bool]):
        for root_vertex in root_vertices_subset:
            root_vertex.visit_children(f)

    def walk_dfs(self, f: Callable[[Vertex], bool]):
        for root_vertex in self.root_vertices:
            root_vertex.visit_children(f)

    def get_random_vertex(self, root_vertices: List[Vertex]) -> Vertex:
        def get_random(current: Vertex, __acc) -> bool:
            random_val: float = float(torch.rand(1).numpy())
            if random_val <= __acc[0]:
                __acc[0] = random_val
                __acc[1] = current
                return True
            return False
        acc: [float, Vertex] = [float('inf'), root_vertices[0]]
        self.walk_dfs_subset_of_vertices(root_vertices, lambda current: get_random(current, acc))
        return acc[1]

    def get_nearest_vertex(self, root_vertices: List[Vertex], point: Point) -> Vertex:
        def get_nearest(current: Vertex, __acc) -> bool:
            dist: float = torch.norm(point.to_tensor() - current.position.to_tensor())
            if dist <= __acc[0]:
                __acc[0] = dist
                __acc[1] = current
                return True
            return False

        acc: [float, Vertex] = [float('inf'), root_vertices[0]]
        self.walk_dfs_subset_of_vertices(root_vertices, lambda current: get_nearest(current, acc))
        return acc[1]

    def get_vertices_within_radius(self, root_vertices: List[Vertex], point: Point, radius: float) -> List[Vertex]:
        def get_within_radius(current: Vertex, __acc) -> bool:
            dist: float = torch.norm(point.to_tensor() - current.position.to_tensor())
            if dist <= radius:
                __acc.append(current)
            return True
        acc: List[Vertex] = list()
        self.walk_dfs_subset_of_vertices(root_vertices, lambda current: get_within_radius(current, acc))
        return acc

    @property
    def size(self) -> int:
        return self.__size
