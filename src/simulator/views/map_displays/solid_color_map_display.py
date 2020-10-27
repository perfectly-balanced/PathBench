from typing import Set, Union, Tuple, List
import copy

from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map_displays.map_display import MapDisplay
from structures import Point, Colour, DynamicColour


class SolidColorMapDisplay(MapDisplay):
    radius: int
    points: Union[Set[Point], List[Entity]]
    colour: DynamicColour

    def __init__(self, services: Services, points: Union[Set[Point], List[Entity]],
                 colour: DynamicColour, z_index=50, radius=0, custom_map: Map = None) -> None:
        super().__init__(services, z_index=z_index, custom_map=custom_map)
        self.points = points
        self.colour = colour
        self.radius = radius

    def render(self) -> bool:
        if not super().render():
            return False

        points: Union[Set[Point], List[Entity]] = copy.deepcopy(self.points)
        if points is None:
            return False

        def f(pt):
            if isinstance(pt, Entity):
                pt = pt.position
            return pt

        points = set(map(f, points))

        c = self.colour()
        for point in points:
            self._root_view.render_pos(Entity(point, self.radius), c)

        return True

    def __lt__(self, other: 'SolidColorMapDisplay') -> bool:
        return tuple(*self.colour) < tuple(*other.colour)
