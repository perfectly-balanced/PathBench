from typing import TYPE_CHECKING

from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map.display.map_display import MapDisplay
from structures import Colour, RED

if TYPE_CHECKING:
    from algorithms.classic.sample_based.rrt import Graph, Vertex


class GraphMapDisplay(MapDisplay):
    __graph: 'Graph'

    edge_colour: Colour
    node_colour: Colour

    def __init__(self, services: Services, graph: 'Graph', custom_map: Map = None) -> None:
        super().__init__(services, z_index=250, custom_map=custom_map)
        self.__graph = graph

        self.edge_colour = self._services.state.add_colour("graph edge", RED)
        self.node_colour = self._services.state.add_colour("graph node", RED)

    def render(self) -> bool:
        if not super().render():
            return False

        try:
            self.__graph.walk_dfs(lambda child: self.__render(child))
        except: # todo: fixme shouldn't ever throw (something fishy is going on)
            print("GraphMapDisplay FAILURE")

        return True

    def __render(self, current: 'Vertex') -> bool:
        self.__render_node(current)
        if len(current.parents) != 0:
            self.__render_edge(current)
        return True

    def __render_edge(self, current: 'Vertex'):
        for parent in current.parents:
            self.get_renderer_view().draw_line(self.edge_colour(), current.position, parent.position)

    def __render_node(self, current: 'Vertex') -> None:
        if self._map.size.n_dim == 2:
            self.get_renderer_view().draw_circle_filled(current.position, colour=self.node_colour())
        else:
            self.get_renderer_view().draw_sphere(current.position, colour=self.node_colour())

    def __lt__(self, other):
        return super().__lt__(other)
