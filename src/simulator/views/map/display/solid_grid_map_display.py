from typing import Set, Union, Tuple, List, Optional, Callable
from numbers import Real

import numpy as np

from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map.display.map_display import MapDisplay
from structures import Point, Colour, DynamicColour
from structures.tracked_grid import TrackedGrid
from structures.tracked import Tracked


class SolidGridMapDisplay(MapDisplay):
    grid: Union[np.ndarray, TrackedGrid]
    comparator: Callable[[Real], bool]
    __tracked_data: List[Tracked]

    colour: DynamicColour
    __deduced_colour: Colour

    def __init__(self, services: Services, grid: Union[np.ndarray, TrackedGrid],
                 colour: DynamicColour, z_index=50, comparator: Callable[[Real], bool] = lambda x: bool(x), offset: Optional[Real] = None, custom_map: Map = None) -> None:
        super().__init__(services, z_index=z_index, custom_map=custom_map)

        if isinstance(grid, np.ndarray) or isinstance(grid, TrackedGrid):
            self.grid = grid
        else:
            self.grid = np.array(grid)

        self.comparator = comparator
        self.__tracked_data = [self.grid] if isinstance(self.grid, Tracked) else []

        self.colour = colour
        self.__deduced_colour = self.colour()

        self.offset = Point(*([0]*self.grid.ndim)) if offset is None else offset

        self.updates_cubes = True

    def render(self, *discarded) -> None:
        c = self.colour()
        refresh = c != self.__deduced_colour
        self.__deduced_colour = c

        if not refresh and isinstance(self.grid, Tracked):
            self.__render_lazy()
        else:
            self.__render_eager()

    def __render_lazy(self) -> None:
        for idx, _ in self.grid.modified:
            self._root_view.cube_requires_update(Point(*idx))

    def __render_eager(self) -> None:
        for idx in np.ndindex(self.grid.shape):
            if self.comparator(self.grid[idx]):
                self._root_view.cube_requires_update(Point(*idx))

    def get_tracked_data(self) -> List[Tracked]:
        return self.__tracked_data

    def update_cube(self, p: Point) -> None:
        p = Point(*p.values[:self.grid.ndim]) - self.offset
        if p.values < self.grid.shape and self.comparator(self.grid[p.values]):
            self._root_view.colour_cube(self.__deduced_colour)

    def request_update_all_cubes(self) -> None:
        rv = self.get_renderer_view()
        for idx in np.ndindex(self.grid.shape):
            if self.comparator(self.grid[idx]):
                rv.cube_requires_update(Point(*idx))
