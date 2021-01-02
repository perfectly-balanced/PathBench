from typing import List, Tuple, Union, Dict, Optional, Any
from numbers import Real

import numpy as np

from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map.display.map_display import MapDisplay
from structures import Point, Colour, DynamicColour
from structures.tracked import Tracked
from structures.tracked_grid import TrackedGrid

class GradientGridMapDisplay(MapDisplay):
    offset: Point
    inverted: bool
    value_bounds: Optional[Tuple[Real, Real]]
    grid: Union[np.ndarray, TrackedGrid]
    __tracked_data: List[Tracked]
    min_colour: DynamicColour
    max_colour: DynamicColour
    min_value: float
    max_value: float
    __deduced_min_colour: Colour
    __deduced_max_colour: Colour
    cube_colours: np.ndarray

    def __init__(self, services: Services,
                 grid: Union[np.ndarray, TrackedGrid],
                 min_colour: DynamicColour = None, max_colour: DynamicColour = None,
                 value_bounds: Optional[Tuple[Real, Real]] = None,
                 z_index=50, inverted: bool = False, custom_map: Map = None, offset: Optional[Point] = None) -> None:
        super().__init__(services, z_index=z_index, custom_map=custom_map)

        if isinstance(grid, np.ndarray) or isinstance(grid, TrackedGrid):
            self.grid = grid
        else:
            self.grid = np.array(grid)

        self.__tracked_data = [self.grid] if isinstance(self.grid, Tracked) else []

        assert min_colour is not None
        assert max_colour is not None
        self.min_colour = min_colour
        self.max_colour = max_colour

        self.inverted = inverted

        self.value_bounds = value_bounds
        if self.value_bounds is None:
            self.min_val = np.inf
            self.max_val = -np.inf
            for idx in np.ndindex(self.grid.shape):
                v = self.grid[idx]
                self.min_val = min(self.min_val, v)
                self.max_val = max(self.max_val, v)
        else:
            self.min_val, self.max_val = self.value_bounds

        self.__deduced_min_colour = self.min_colour()
        self.__deduced_max_colour = self.max_colour()

        self.cube_colours = np.full(self.grid.shape, None, dtype=Colour)
        for idx in np.ndindex(self.grid.shape):
            self.cube_colours[idx] = self.get_colour(self.grid[idx])

        self.offset = Point(*([0]*self.grid.ndim)) if offset is None else offset

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

        if not refresh and isinstance(self.grid, Tracked):
            self.__render_lazy()
        else:
            self.__render_eager()

    def __render_lazy(self) -> None:
        if self.value_bounds is None:
            old_min_val = self.min_val
            old_max_val = self.max_val
            for idx, _ in self.grid.modified:
                v = self.grid[idx]
                self.min_val = min(self.min_val, v)
                self.max_val = max(self.max_val, v)

            if old_min_val != self.min_val or old_max_val != self.max_val:
                self.__eager_colour_update_dispatch()
                return

        rv = self.get_renderer_view()
        for idx, _ in self.grid.modified:
            self.cube_colours[idx] = self.get_colour(self.grid[idx])
            rv.cube_requires_update(Point(*idx))

    def __render_eager(self) -> None:
        if self.value_bounds is None:
            self.min_val: float = np.inf
            self.max_val: float = -np.inf
            for idx in np.ndindex(self.grid.shape):
                v = self.grid[idx]
                self.min_val = min(self.min_val, v)
                self.max_val = max(self.max_val, v)
        self.__eager_colour_update_dispatch()

    def __eager_colour_update_dispatch(self) -> None:
        rv = self.get_renderer_view()
        for idx in np.ndindex(self.grid.shape):
            clr = self.get_colour(self.grid[idx])
            if clr != self.cube_colours[idx]:
                rv.cube_requires_update(Point(*idx))
                self.cube_colours[idx] = clr

    def update_cube(self, p: Point) -> None:
        p = Point(*p.values[:self.grid.ndim]) - self.offset
        if p.values < self.grid.shape:
            c = self.cube_colours[p.values]
            if c is not None:
                self._root_view.colour_cube(c)

    def get_tracked_data(self) -> List[Tracked]:
        return self.__tracked_data

    def request_update_all_cubes(self) -> None:
        rv = self.get_renderer_view()
        for idx in np.ndindex(self.grid.shape):
            rv.cube_requires_update(Point(*idx))
