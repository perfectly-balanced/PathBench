from typing import Set, Union, Tuple, List
import copy

from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map.display.map_display import MapDisplay
from structures import Point, Colour, DynamicColour


class SolidColourMapDisplay(MapDisplay):
    radius: int
    points: Union[Set[Point], List[Entity]]
    colour: DynamicColour

    def __init__(self, services: Services, points: Union[Set[Point], List[Entity]],
                 colour: DynamicColour, z_index=50, radius=0, custom_map: Map = None) -> None:
        super().__init__(services, z_index=z_index, custom_map=custom_map)
        self.points = points
        self.colour = colour
        self.radius = radius

        self.__cube_colours = {}
        self.__c = self.colour()

    def render(self) -> bool:
        if not super().render():
            return False

        self.__unique_points: Union[Set[Point], List[Entity]] = copy.deepcopy(self.points)
        if self.__unique_points is None:
            return False

        self.__unique_points = set(map(self._root_view.to_point3, self.__unique_points))
        self._root_view.display_updates_cube()
        self.__c = self.colour()
        for point in self.__unique_points:
            self._root_view.cube_requires_update(point)

        return True
    
    def update_cube(self, p: Point) -> None:
        if p in self.__unique_points:
            self._root_view.colour_cube(self.__c)

    def __lt__(self, other: 'SolidColourMapDisplay') -> bool:
        return tuple(*self.colour) < tuple(*other.colour)
