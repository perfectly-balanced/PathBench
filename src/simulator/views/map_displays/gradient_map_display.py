import pygame
from typing import List, Tuple, Union

import numpy as np

from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map_displays.map_display import MapDisplay
from structures import Point, Colour

class GradientMapDisplay(MapDisplay):
    inverted: bool
    pts: List[Tuple[Union[int, float], Point]]
    min_color: np.ndarray
    max_color: np.ndarray

    def __init__(self, services: Services, grid: List[List[Union[int, float]]] = None,
                 pts: List[Tuple[Union[int, float], Point]] = None,
                 min_color: np.ndarray = np.array([150., 150., 0.]),
                 max_color=np.array([150., 0., 0.]), z_index=50, inverted: bool = False, custom_map: Map = None) -> None:
        super().__init__(services, z_index=z_index, custom_map=custom_map)

        self.pts = None

        if grid:
            self.pts = self.__transform_to_points(grid)

        if pts:
            self.pts = pts

        self.min_color = min_color
        self.max_color = max_color
        self.inverted = inverted

    @staticmethod
    def __transform_to_points(grid: List[List[Union[int, float]]]) -> List[Tuple[Union[int, float], Point]]:
        ret: List[Tuple[Union[int, float], Point]] = []

        for i in range(len(grid)):
            for j in range(len(grid[i])):
                ret.append((grid[i][j], Point(j, i)))

        return ret

    def get_color(self, val: float, min_val: float, max_val: float) -> np.ndarray:
        proc_dist: float = (val - min_val) / ((max_val - min_val) if (max_val - min_val) != 0 else 1)
        color_vec: np.ndarray = self.max_color - self.min_color
        clr = self.min_color + proc_dist * color_vec
        if self.inverted:
            clr = self.max_color - proc_dist * color_vec
        return np.clip(clr, 0, 255)

    def render(self) -> bool:
        if self._map.size.n_dim == 3:
            return True # hacky: don't render

        if not super().render():
            return False

        if self.pts is None:
            return False

        min_val: float = np.inf
        max_val: float = -np.inf
        for p in self.pts:
            min_val = min(min_val, p[0])
            max_val = max(max_val, p[0])

        for p in self.pts:
            clr = self.get_color(p[0], min_val, max_val)
            c = list(clr)
            for i in range(len(c)):
                c[i] /= 255
                c[i] = min(1, c[i])
            c = Colour(*c)
            self._root_view.render_pos(Entity(p[1]), c)

        return True

    def __lt__(self, other):
        return super().__lt__(other)
