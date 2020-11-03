import pygame
from typing import List

from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map.display.map_display import MapDisplay
from structures import Point


class NumbersMapDisplay(MapDisplay):
    grid: List[List[float]]

    def __init__(self, services: Services, grid: List[List[float]], custom_map: Map = None) -> None:
        super().__init__(services, z_index=150, custom_map=custom_map)
        self.font = None
        self.grid = grid

    def render(self) -> bool:
        if not super().render() or self.grid is None:
            return False

        if self.services.settings.simulator_grid_display:
            for i in range(len(self.grid)):
                for j in range(len(self.grid[i])):
                    self.get_renderer_view().render_text(Point(i, j), str(round(self.grid[i][j], 1)))

        return True

    def __lt__(self, other):
        return super().__lt__(other)
