import pygame
from typing import List

from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map_displays.map_display import MapDisplay
from structures import Point


class NumbersMapDisplay(MapDisplay):
    grid: List[List[float]]
    font: pygame.font.SysFont

    def __init__(self, services: Services, grid: List[List[float]], custom_map: Map = None) -> None:
        super().__init__(services, z_index=150, custom_map=custom_map)
        self.font = None
        self.grid = grid
        if self.services.render_engine.is_display_init():
            self.font = pygame.font.SysFont('Arial', 60)

    def render(self, screen: pygame.Surface) -> bool:
        if not super().render(screen) or not self.grid:
            return False

        if self.services.settings.simulator_grid_display:
            for i in range(len(self.grid)):
                for j in range(len(self.grid[i])):
                    self.render_text(screen, Point(j, i), str(round(self.grid[i][j], 1)))

        return True

    def render_text(self, screen: pygame.Surface, position: Point, text: str) -> None:
        if self.font is None:
            return
        text_surface: pygame.Surface = self.font.render(text, False, (0, 0, 0))
        text_rect: pygame.Rect = text_surface.get_rect()
        screen.blit(text_surface, [self._root_view.get_screen_rect(position).center[0] - text_rect.width / 2 + 2,
                                   self._root_view.get_screen_rect(position).center[1] - text_rect.height / 2 + 3])

    def __lt__(self, other):
        return super().__lt__(other)
