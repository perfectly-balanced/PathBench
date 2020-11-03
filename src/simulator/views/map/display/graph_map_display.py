from typing import List, TYPE_CHECKING

from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map.display.map_display import MapDisplay
from structures import DynamicColour, Colour, RED
from structures.tracked import Tracked
from algorithms.classic.sample_based.core.graph import TrackedGraph, Graph, Vertex


class GraphMapDisplay(MapDisplay):
    __graph: Graph

    edge_colour: DynamicColour
    node_colour: DynamicColour
    __deduced_edge_colour: Colour
    __deduced_node_colour: Colour

    def __init__(self, services: Services, graph: Graph, custom_map: Map = None) -> None:
        super().__init__(services, z_index=250, custom_map=custom_map)
        self.__graph = graph

        self.edge_colour = self._services.state.add_colour("graph edge", RED)
        self.node_colour = self._services.state.add_colour("graph node", RED)

        self.__deduced_edge_colour = self.edge_colour()
        self.__deduced_node_colour = self.node_colour()

    def render(self, refresh: bool) -> bool:
        if not super().render(refresh):
            return False

        c = self.edge_colour()
        refresh = refresh or c != self.__deduced_edge_colour
        self.__deduced_edge_colour = c

        c = self.node_colour()
        refresh = refresh or c != self.__deduced_node_colour
        self.__deduced_node_colour = c

        if not refresh and isinstance(self.__graph, TrackedGraph):
            return self.__render_lazy()
        else:
            return self.__render_eager()

    def __render_lazy(self) -> bool:
        print("todo")
        self.__graph.walk_dfs(lambda child: self.__render(child))
        return True

    def __render_eager(self) -> bool:
        self.__graph.walk_dfs(lambda child: self.__render(child))
        return True

    def __render(self, current: Vertex) -> bool:
        self.__render_node(current)
        if len(current.parents) != 0:
            self.__render_edge(current)
        return True

    def __render_edge(self, current: Vertex):
        rv = self.get_renderer_view()
        for parent in current.parents:
            rv.draw_line(self.__deduced_edge_colour, current.position, parent.position)

    def __render_node(self, current: Vertex) -> None:
        rv = self.get_renderer_view()
        if self._map.size.n_dim == 2:
            rv.draw_circle_filled(current.position, colour=self.__deduced_node_colour)
        else:
            rv.draw_sphere(current.position, colour=self.__deduced_node_colour)

    def get_tracked_data(self) -> List[Tracked]:
        return [self.__graph] if isinstance(self.__graph, Tracked) else []
