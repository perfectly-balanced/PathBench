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

    def render(self, screen: pygame.Surface) -> bool:
        if not super().render(screen):
            return False

        self.__graph.walk_dfs(lambda child: self.__render(child, screen))

        return True

    def __render(self, current: 'Vertex', screen: pygame.Surface) -> bool:
        self.__render_node(current, screen)
        if len(current.parents) != 0:
            self.__render_edge(current, screen)
        return True

    def __render_edge(self, current: 'Vertex', screen: pygame.Surface):
        first_point = self.get_renderer_view().get_screen_rect(current.position).center
        for parent in current.parents:
            second_point = self.get_renderer_view().get_screen_rect(parent.position).center
            self.services.render_engine.draw_line(screen, (0, 0, 200), first_point, second_point, 1)

    def __render_node(self, current: 'Vertex', screen: pygame.Surface) -> None:
        pt = self.get_renderer_view().get_screen_rect(current.position).center
        self.services.render_engine.draw_circle(screen, (0, 0, 255), pt, 2)

    def __lt__(self, other):
        return super().__lt__(other)
