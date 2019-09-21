import pygame

from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map_displays.map_display import MapDisplay
from structures import Point


class EmptyMapDisplay(MapDisplay):
    def __init__(self, services: Services, custom_map: Map = None) -> None:
        super().__init__(services, z_index=0, custom_map=custom_map)

    def render(self, screen: pygame.Surface) -> bool:
        if not super().render(screen):
            return False

        screen.fill((0, 0, 0))

        if self.services.settings.simulator_grid_display:
            self.render_grid_display_cells(screen)
        else:
            self.render_image_map(screen)

        return True

    def render_grid_display_cells(self, screen: pygame.Surface) -> None:
        for x in range(self.services.algorithm.map.size.width):
            for y in range(self.services.algorithm.map.size.height):
                self._root_view.render_pos(screen, Entity(Point(x, y), 1), (255, 255, 255))

    def render_image_map(self, screen: pygame.Surface) -> None:
        self.services.render_engine.draw_rect(screen, (255, 255, 255),
                                              pygame.Rect(0, 0,
                                                          self.services.algorithm.map.size.width,
                                                          self.services.algorithm.map.size.height))

    def __lt__(self, other):
        return super().__lt__(other)
