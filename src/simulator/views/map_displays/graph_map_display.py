import pygame
from typing import TYPE_CHECKING

from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map_displays.map_display import MapDisplay

if TYPE_CHECKING:
    from algorithms.classic.sample_based.rrt import Graph, Vertex


class GraphMapDisplay(MapDisplay):
    __graph: 'Graph'

    def __init__(self, services: Services, graph: 'Graph', custom_map: Map = None) -> None:
        super().__init__(services, z_index=250, custom_map=custom_map)
        self.__graph = graph

    def render(self) -> bool:
        if not super().render():
            return False

        self.__graph.walk_dfs(lambda child: self.__render(child))

        return True

    def __render(self, current: 'Vertex') -> bool:
        self.__render_node(current)
        if len(current.parents) != 0:
            self.__render_edge(current)
        return True

    def __render_edge(self, current: 'Vertex'):
        first_point = self.get_renderer_view().get_center(current.position)
        for parent in current.parents:
            second_point = self.get_renderer_view().get_center(parent.position)
            self.get_renderer_view().draw_line((0, 0, 200), first_point, second_point)

    def __render_node(self, current: 'Vertex') -> None:
        pt = self.get_renderer_view().get_center(current.position)
        self.get_renderer_view().draw_circle(pt)

    def __lt__(self, other):
        return super().__lt__(other)
