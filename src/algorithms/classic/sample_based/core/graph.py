from algorithms.classic.sample_based.core.vertex import Vertex
from structures import Point
from structures.tracked import Tracked
from structures.factory import prefer_tracked
from simulator.services.services import Services

import torch
from abc import ABC, abstractmethod
from typing import Callable, List, Tuple, Any, Dict, Union

class Graph(ABC):
    def __init__(self, start_pos: Vertex, goal_pos: Vertex, root_vertices: List[Vertex]) -> None:
        self.root_vertex_start = start_pos
        self.root_vertex_goal = goal_pos
        self.root_vertices = [self.root_vertex_start] + [self.root_vertex_goal] + root_vertices
        self.size = 2 + len(root_vertices)

    def reverse_root_vertices(self):
        self.root_vertices.reverse()

    @abstractmethod
    def walk_dfs_subset_of_vertices(self, root_vertices_subset: List[Vertex], f: Callable[[Vertex], bool]):
        pass

    @abstractmethod
    def walk_dfs(self, f: Callable[[Vertex], bool]):
        pass

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

class TrackedGraph(Tracked):
    added: List[Tuple[Vertex, Vertex]]
    removed: List[Tuple[Vertex, Vertex]]
    edges_removable: bool

    def __init__(self) -> None:
        self.added = []
        self.removed = []
        self.edges_removable = True

    def _edge_added(self, parent: Vertex, child: Vertex) -> None:
        self.added.append((parent, child))

    def _edge_removed(self, parent: Vertex, child: Vertex) -> None:
        assert self.edges_removable
        self.removed.append((parent, child))

    def clear_tracking_data(self) -> None:
        self.added.clear()
        self.removed.clear()


class Forest(Graph):
    def add_edge(self, parent: Vertex, child: Vertex):
        if child is not parent:
            parent.add_child(child)
            child.set_parent(parent)
            self.size += 1

    def remove_edge(self, parent: Vertex, child: Vertex):
        parent.remove_child(child)
        child.remove_parent(parent)
        self.size -= 1

    def walk_dfs_subset_of_vertices(self, root_vertices_subset: List[Vertex], f: Callable[[Vertex], bool]):
        for root_vertex in root_vertices_subset:
            root_vertex.visit_children(f)

    def walk_dfs(self, f: Callable[[Vertex], bool]):
        for root_vertex in self.root_vertices:
            root_vertex.visit_children(f)

class TrackedForest(Forest, TrackedGraph):
    def __init__(self, *args) -> None:
        Forest.__init__(self, *args)
        TrackedGraph.__init__(self)

    def add_edge(self, parent: Vertex, child: Vertex) -> None:
        Forest.add_edge(self, parent, child)
        TrackedGraph._edge_added(self, parent, child)

    def remove_edge(self, parent: Vertex, child: Vertex) -> None:
        Forest.remove_edge(self, parent, child)
        TrackedGraph._edge_removed(self, parent, child)

class CyclicGraph(Graph):
    def add_edge(self, parent: Vertex, child: Vertex):
        if child is not parent:
            parent.add_child(child)
            child.add_parent(parent)
            self.size += 1

    def remove_edge(self, parent: Vertex, child: Vertex):
        parent.remove_child(child)
        child.remove_parent(parent)
        self.size -= 1

    def walk_dfs_subset_of_vertices(self, root_vertices_subset: List[Vertex], f: Callable[[Vertex], bool]):
        for root_vertex in root_vertices_subset:
            if not f(root_vertex):
                return

    def walk_dfs(self, f: Callable[[Vertex], bool]):
        for root_vertex in self.root_vertices:
            if not f(root_vertex):
                return

class TrackedCyclicGraph(CyclicGraph, TrackedGraph):
    def __init__(self, *args) -> None:
        CyclicGraph.__init__(self, *args)
        TrackedGraph.__init__(self)

    def add_edge(self, parent: Vertex, child: Vertex) -> None:
        CyclicGraph.add_edge(self, parent, child)
        if child is not parent:
            TrackedGraph._edge_added(self, parent, child)

    def remove_edge(self, parent: Vertex, child: Vertex) -> None:
        CyclicGraph.remove_edge(self, parent, child)
        TrackedGraph._edge_removed(self, parent, child)

def gen_forest(services: Services, *args, **kwargs) -> Forest:
    if prefer_tracked(services):
        return TrackedForest(*args, **kwargs)
    else:
        return Forest(*args, **kwargs)

def gen_cyclic_graph(services: Services, *args, **kwargs) -> CyclicGraph:
    if prefer_tracked(services):
        return TrackedCyclicGraph(*args, **kwargs)
    else:
        return CyclicGraph(*args, **kwargs)


'''
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
'''
