from typing import Set, Union, Tuple, List, Optional
import copy

from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map.display.map_display import MapDisplay
from structures import Point, Colour, DynamicColour
from structure.tracked import Tracked
from structure.tracked_set import TrackedSet


class SolidColourMapDisplay(MapDisplay):
    radius: int
    colour: DynamicColour

    points: Union[Set[Point], List[Entity]]
    __old_points: Optional[Union[Set[Point], List[Entity]]]

    __deduced_colour: Colour

    def __init__(self, services: Services, points: Union[Set[Point], List[Entity]],
                 colour: DynamicColour, z_index=50, radius=0, custom_map: Map = None) -> None:
        super().__init__(services, z_index=z_index, custom_map=custom_map)

        self.radius = radius
        self.colour = colour

        self.points = points
        self.__old_points = None

        self.__deduced_colour = self.colour()

    def render(self) -> bool:
        if not super().render():
            return False

        if self.points is None:
            return False

        self._root_view.display_updates_cube()
        self.__deduced_colour = self.colour()

        if type(self.points) is TrackedSet:
            return self.__render_tracked_set()
        else:
            return self.__render_not_tracked()

    def __render_tracked_set(self) -> bool:
        td = self.points.get_tracking_data()
        for p in td.added:
            self._root_view.cube_requires_update(p)
        for p in td.removed:
            self._root_view.cube_requires_update(p)
        return True

    def __render_not_tracked(self) -> bool:
        if self.__old_points is not None:
            for p in self.__old_points:
                self._root_view.cube_requires_update(p)

        self.__old_points = copy.deepcopy(self.points)
        if self.__old_points is None:
            return False
        self.__old_points = set(map(self._root_view.to_point3, self.__old_points))

        for p in self.__old_points:
            self._root_view.cube_requires_update(p)

    def get_tracked_data(self) -> List[Tracked]:
        return [self.points] if isinstance(self.points, Tracked) else []

    def update_cube(self, p: Point) -> None:
        if self._map.size.n_dim == 2:
            p = Point(p.x, p.y)
        if p in self.points:
            self._root_view.colour_cube(self.__deduced_colour)

    def __lt__(self, other: 'SolidColourMapDisplay') -> bool:
        return tuple(*self.colour) < tuple(*other.colour)
