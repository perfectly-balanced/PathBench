from typing import Set, Callable, Dict, List, Any
from structures import Point


class Vertex:
    __position: Point
    __children: Set['Vertex']
    __parents: Set['Vertex']
    __connectivity: Dict['Vertex', 'Vertex']
    __aux: Dict[Any, Any]

    def __init__(self, position: Point, store_connectivity: bool = False) -> None:
        self.__position = position
        self.__children = set()
        self.__parents = set()
        self.__connectivity = {self: self}
        self.__store_connectivity = store_connectivity
        self.__cost = None
        self.__aux = {}

    def __add_connectivity(self, vertex_added: 'Vertex'):

        if vertex_added is self:
            return

        # update connectivity of this vertex
        self.__connectivity[vertex_added] = vertex_added

        connectivity_keys = self.connectivity.keys()
        vertex_added_connectivity_keys = vertex_added.connectivity.keys()
        new_connection_keys = vertex_added_connectivity_keys - connectivity_keys

        for vertex_key in new_connection_keys:
            if vertex_key:
                self.__connectivity[vertex_key] = vertex_added

        # get connectivity of all connections correct
        connectivity_keys = self.connectivity.keys()
        vertex_added.connectivity[self] = self
        for vertex_to_update in self.__connectivity:
            vertex_connectivity_keys = vertex_to_update.connectivity.keys()
            new_connection_keys = connectivity_keys - vertex_connectivity_keys
            for vertex_key in new_connection_keys:
                if self in vertex_to_update.connectivity:
                    new_vertex_path_target = self
                else:
                    new_vertex_path_target = vertex_added
                new_vertex_path_step = vertex_to_update.connectivity[new_vertex_path_target]
                vertex_to_update.connectivity[vertex_key] = new_vertex_path_step

    # Adding #

    def add_child(self, child: 'Vertex') -> None:
        self.__children.add(child)
        if self.__store_connectivity:
            self.__add_connectivity(child)


    def add_parent(self, parent: 'Vertex') -> None:
        self.__parents.add(parent)
        if self.__store_connectivity:
            self.__add_connectivity(parent)

    # Removing #

    def remove_child(self, child: 'Vertex') -> None:
        self.__children.remove(child)
        # ToDo No remove connecitivty implemented, as it is not trivial

    def remove_parent(self, parent: 'Vertex') -> None:
        self.__parents.remove(parent)
        # No remove connecitivty implemented, as it is not trivial

    # Setting #

    def set_child(self, child: 'Vertex'):
        self.__children.clear()
        # ToDo No remove connecitivty implemented, as it is not trivial
        self.__children.add(child)
        if self.__store_connectivity:
            child.__add_connectivity(self)
            self.__add_connectivity(child)

    def set_parent(self, parent: 'Vertex'):
        self.__parents.clear()
        # ToDo No remove connecitivty implemented, as it is not trivial
        self.__parents.add(parent)
        if self.__store_connectivity:
            parent.__add_connectivity(self)
            self.__add_connectivity(parent)

    # Visiting #

    def visit_children(self, f: Callable[['Vertex'], bool]) -> None:
        for child in self.__children:
            child.visit_children(f)
        if not f(self):
            return

    def visit_parents(self, f: Callable[['Vertex'], bool]) -> None:
        for child in self.__children:
            child.visit_children(f)
        if not f(self):
            return

    # Properties #

    @property
    def cost(self) -> float:
        return self.__cost

    @property
    def position(self) -> Point:
        return self.__position

    @property
    def children(self) -> Set['Vertex']:
        return self.__children

    @property
    def parents(self) -> Set['Vertex']:
        return self.__parents

    @property
    def connectivity(self) -> Dict['Vertex', 'Vertex']:
        return self.__connectivity

    @property
    def aux(self) -> Dict[Any, Any]:
        return self.__aux

    # Setters #

    @cost.setter
    def cost(self, val: float):
        self.__cost = val

