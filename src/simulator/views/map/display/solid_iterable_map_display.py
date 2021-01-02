from typing import Set, Union, Tuple, List, Optional
import copy

from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map.display.map_display import MapDisplay
from structures import Point, Colour, DynamicColour
from structures.tracked import Tracked


class SolidIterableMapDisplay(MapDisplay):
    colour: DynamicColour

    points: Union[Set[Point], List[Entity]]
    __old_pts: Optional[Union[Set[Point], List[Entity]]]
    __point_dim: int
    __tracked_data: List[Tracked]

    __deduced_colour: Colour

    def __init__(self, services: Services, points: Union[Set[Point], List[Entity]],
                 colour: DynamicColour, z_index=50, custom_map: Map = None) -> None:
        super().__init__(services, z_index=z_index, custom_map=custom_map)

        self.colour = colour
        self.__deduced_colour = self.colour()

        self.pts = points
        self.__old_pts = None
        self.__point_dim = None
        self.__tracked_data = [self.pts] if isinstance(self.pts, Tracked) else []

        self.updates_cubes = True

    def render(self, *discarded) -> None:
        if len(self.pts) == 0:
            return

        c = self.colour()
        refresh = c != self.__deduced_colour
        self.__deduced_colour = c

        self.__point_dim = next(iter(self.pts)).n_dim

        if not refresh and isinstance(self.pts, Tracked):
            self.__render_lazy()
        else:
            self.__render_eager()

    def __render_lazy(self) -> None:
        for p in self.pts.modified:
            self._root_view.cube_requires_update(p)

    def __render_eager(self) -> None:
        if self.__old_pts is not None:
            for p in self.__old_pts:
                self._root_view.cube_requires_update(p)

        self.__old_pts = copy.deepcopy(self.pts)
        if self.__old_pts is None:
            return

        for p in self.__old_pts:
            self._root_view.cube_requires_update(p)

    def get_tracked_data(self) -> List[Tracked]:
        return self.__tracked_data

    def update_cube(self, p: Point) -> None:
        if self.__point_dim != 3:
            p = Point(p.x, p.y)

        if p in self.pts:
            self._root_view.colour_cube(self.__deduced_colour)

    def request_update_all_cubes(self) -> None:
        rv = self.get_renderer_view()
        for p in self.pts:
            rv.cube_requires_update(p)
