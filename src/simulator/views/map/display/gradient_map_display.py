from typing import List, Tuple, Union, Dict

import numpy as np

from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map.display.map_display import MapDisplay
from structures import Point, Colour, DynamicColour
from structures.tracked import Tracked

class GradientMapDisplay(MapDisplay):
    inverted: bool
    pts: List[Tuple[Union[int, float], Point]]
    min_color: DynamicColour
    max_color: DynamicColour
    min_value: float
    max_value: float
    __cube_colours: Dict[Point, Colour]

    def __init__(self, services: Services, grid: List[List[Union[int, float]]] = None,
                 pts: List[Tuple[Union[int, float], Point]] = None,
                 min_color: DynamicColour = None,
                 max_color: DynamicColour = None, z_index=50, inverted: bool = False, custom_map: Map = None) -> None:
        super().__init__(services, z_index=z_index, custom_map=custom_map)

        self.pts = None

        if pts is not None:
            self.pts = pts
        elif grid is not None:
            self.pts = self.__transform_to_points(grid)

        assert min_color is not None
        assert max_color is not None
        self.min_color = min_color
        self.max_color = max_color
        self.inverted = inverted
        self.min_val = np.inf
        self.max_val = -np.inf

        self.__cube_colours = {}

    @staticmethod
    def __transform_to_points(grid: List[List[Union[int, float]]]) -> List[Tuple[Union[int, float], Point]]:
        ret: List[Tuple[Union[int, float], Point]] = []

        for i in range(len(grid)):
            for j in range(len(grid[i])):
                ret.append((grid[i][j], Point(j, i)))

        return ret

    def get_colour(self, val: float) -> Colour:
        d = (self.max_val - self.min_val)
        d = d if d != 0 else 1
        mag = (val - self.min_val) / d

        cmag = Colour(mag, mag, mag, mag)
        cmin = self.min_color()
        cmax = self.max_color()
        cvec: Colour = cmax - cmin
        clr = (cmax - cmag * cvec) if self.inverted else (cmin + cmag * cvec)
        
        return clr

    def render(self, refresh: bool) -> bool:
        if not super().render(refresh):
            return False

        if self.pts is None:
            return False

        self.get_renderer_view().display_updates_cube()

        if not refresh and isinstance(self.pts, Tracked):
            return self.__render_tracked()
        else:
            return self.__render_not_tracked()
    
    def __render_tracked(self) -> bool:
        if self.pts.elems_were_removed:
            return self.__render_not_tracked()
        
        old_min_val = self.min_val
        old_max_val = self.max_val
        for p in self.pts.modified:
            self.min_val = min(self.min_val, p[0])
            self.max_val = max(self.max_val, p[0])

        if old_min_val != self.min_val or old_max_val != self.max_val:
            self.__eager_colour_update_dispatch()
        else:
            rv = self.get_renderer_view()
            self.__cube_colours.clear()
            for p in self.pts.modified:
                clr = self.get_colour(p[0])
                self.__cube_colours[rv.cube_requires_update(p[1])] = clr
        return True

    def __render_not_tracked(self) -> bool:
        self.min_val: float = np.inf
        self.max_val: float = -np.inf
        for p in self.pts:
            self.min_val = min(self.min_val, p[0])
            self.max_val = max(self.max_val, p[0])
        self.__eager_colour_update_dispatch()
        return True

    def __eager_colour_update_dispatch(self) -> None:
        rv = self.get_renderer_view()
        for p in self.__cube_colours:
            rv.cube_requires_update(p)
        self.__cube_colours.clear()
        for p in self.pts:
            clr = self.get_colour(p[0])
            self.__cube_colours[rv.cube_requires_update(p[1])] = clr
    
    def update_cube(self, p: Point) -> None:
        if p in self.__cube_colours:
            self._root_view.colour_cube(self.__cube_colours[p])

    def get_tracked_data(self) -> List[Tracked]:
        return [self.pts] if isinstance(self.pts, Tracked) else []

    def __lt__(self, other):
        return super().__lt__(other)
