import pygame
import numpy as np
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
            for index in np.ndindex(*(self.grid.shape)):
                self.get_renderer_view().render_text(Point(*index), str(round(self.grid[index], 1)))

        return True
