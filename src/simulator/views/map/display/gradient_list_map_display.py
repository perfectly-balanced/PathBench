from typing import List, Tuple, Union, Dict, Optional, Any

import numpy as np

from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map.display.map_display import MapDisplay
from structures import Point, Colour, DynamicColour
from structures.tracked import Tracked
from structures.tracked_grid import TrackedGrid

class GradientListMapDisplay(MapDisplay):
    inverted: bool
    pts: List[Tuple[Union[int, float], Point]]
    __tracked_data: List[Tracked]
    min_colour: DynamicColour
    max_colour: DynamicColour
    min_value: float
    max_value: float
    __deduced_min_colour: Colour
    __deduced_max_colour: Colour
    __cube_colours: Dict[Point, Colour]

    def __init__(self, services: Services,
                 pts: List[Tuple[Union[int, float], Point]],
                 min_colour: DynamicColour = None, max_colour: DynamicColour = None,
                 value_bounds: Optional[Tuple[Union[int, float], Union[int, float]]] = None,
                 z_index=50, inverted: bool = False, custom_map: Map = None) -> None:
        super().__init__(services, z_index=z_index, custom_map=custom_map)

        self.pts = pts

        self.__tracked_data = [self.pts] if isinstance(self.pts, Tracked) else []

        assert min_colour is not None
        assert max_colour is not None
        self.min_colour = min_colour
        self.max_colour = max_colour

        self.inverted = inverted

        self.value_bounds = value_bounds
        if self.value_bounds is None:
            self.min_val = np.inf
            self.max_val = -np.inf
        else:
            self.min_val, self.max_val = self.value_bounds

        self.__deduced_min_colour = self.min_colour()
        self.__deduced_max_colour = self.max_colour()

        self.__cube_colours = {}

        self.updates_cubes = True

    def get_colour(self, val: float) -> Optional[Colour]:
        mag = (val - self.min_val)

        d = (self.max_val - self.min_val)
        if d != 0:
            mag /= d

        # catch NaN in addition to out-of-bounds
        if not (mag >= 0 and mag <= 1):
            return None

        cmag = Colour(mag, mag, mag, mag)
        cmin = self.__deduced_min_colour
        cmax = self.__deduced_max_colour
        cvec: Colour = cmax - cmin
        clr = (cmax - cmag * cvec) if self.inverted else (cmin + cmag * cvec)

        return clr

    def render(self, *discarded) -> None:
        c = self.min_colour()
        refresh = c != self.__deduced_min_colour
        self.__deduced_min_colour = c

        c = self.max_colour()
        refresh = refresh or c != self.__deduced_max_colour
        self.__deduced_max_colour = c

        if not refresh and isinstance(self.pts, Tracked):
            self.__render_lazy()
        else:
            self.__render_eager()

    def __render_lazy(self) -> None:
        if self.pts.elems_were_removed:
            # todo: could add some laziness here
            self.__render_eager()
            return

        if self.value_bounds is None:
            old_min_val = self.min_val
            old_max_val = self.max_val
            for p in self.pts.modified:
                self.min_val = min(self.min_val, p[0])
                self.max_val = max(self.max_val, p[0])

            if old_min_val != self.min_val or old_max_val != self.max_val:
                self.__eager_colour_update_dispatch()
                return

        rv = self.get_renderer_view()
        for p in self.pts.modified:
            clr = self.get_colour(p[0])
            self.__cube_colours[rv.cube_requires_update(p[1])] = clr

    def __render_eager(self) -> None:
        if self.value_bounds is None:
            self.min_val: float = np.inf
            self.max_val: float = -np.inf
            for p in self.pts:
                self.min_val = min(self.min_val, p[0])
                self.max_val = max(self.max_val, p[0])
        self.__eager_colour_update_dispatch()

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
            clr = self.__cube_colours[p]
            if clr is not None:
                self._root_view.colour_cube(clr)

    def get_tracked_data(self) -> List[Tracked]:
        return self.__tracked_data

    def request_update_all_cubes(self) -> None:
        rv = self.get_renderer_view()
        for p in self.pts:
            rv.cube_requires_update(p[1])
