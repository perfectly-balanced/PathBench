from typing import List, Tuple, Union

import numpy as np

from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map.display.map_display import MapDisplay
from structures import Point, Colour, DynamicColour

class GradientMapDisplay(MapDisplay):
    inverted: bool
    pts: List[Tuple[Union[int, float], Point]]
    min_color: DynamicColour
    max_color: DynamicColour

    def __init__(self, services: Services, grid: List[List[Union[int, float]]] = None,
                 pts: List[Tuple[Union[int, float], Point]] = None,
                 min_color: DynamicColour = None,
                 max_color: DynamicColour = None, z_index=50, inverted: bool = False, custom_map: Map = None) -> None:
        super().__init__(services, z_index=z_index, custom_map=custom_map)

        self.pts = None

        if grid is not None:
            self.pts = self.__transform_to_points(grid)
        elif pts is not None:
            self.pts = pts

        assert min_color is not None
        assert max_color is not None
        self.min_color = min_color
        self.max_color = max_color
        self.inverted = inverted

        self.__cube_colours = {}

    @staticmethod
    def __transform_to_points(grid: List[List[Union[int, float]]]) -> List[Tuple[Union[int, float], Point]]:
        ret: List[Tuple[Union[int, float], Point]] = []

        for i in range(len(grid)):
            for j in range(len(grid[i])):
                ret.append((grid[i][j], Point(j, i)))

        return ret

    def get_colour(self, val: float, min_val: float, max_val: float) -> Colour:
        cmin = self.min_color()
        cmax = self.max_color()

        # todo: fixme shouldn't have to clamp 'proc_dist' (something fishy is going on)
        proc_dist: float = max(0, min(1, (val - min_val) / ((max_val - min_val) if (max_val - min_val) != 0 else 1)))
        proc_dist_clr = Colour(proc_dist, proc_dist, proc_dist, proc_dist)
        clr_vec: Colour = cmax - cmin
        clr = cmin + proc_dist_clr * clr_vec
        if self.inverted:
            clr = cmax - proc_dist_clr * clr_vec
        return clr

    def render(self) -> bool:
        if not super().render():
            return False

        if self.pts is None:
            return False

        self.get_renderer_view().display_updates_cube()

        min_val: float = np.inf
        max_val: float = -np.inf
        for p in self.pts:
            print("point[0]" + str(p[0]))
            min_val = min(min_val, p[0])
            max_val = max(max_val, p[0])

        self.__cube_colours.clear()
        for p in self.pts:
            clr = self.get_colour(p[0], min_val, max_val)
            self.__cube_colours[self.get_renderer_view().cube_requires_update(p[1])] = clr
        return True
    
    def update_cube(self, p: Point) -> None:
        if p in self.__cube_colours:
            self._root_view.colour_cube(self.__cube_colours[p])

    def __lt__(self, other):
        return super().__lt__(other)
