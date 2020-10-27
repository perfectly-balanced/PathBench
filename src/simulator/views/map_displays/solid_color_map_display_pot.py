from typing import Set, Union, Tuple, List
import copy

from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map_displays.map_display import MapDisplay
from structures import Point, Colour

class SolidColorMapDisplaypot(MapDisplay):
    radius: int
    points: Union[Set[Point], List[Entity]]

    def __init__(self, services: Services, points, z_index=50, radius=0, custom_map: Map = None, pmaplst=None) -> None:
        super().__init__(services, z_index=z_index, custom_map=custom_map)
        self.points = points
        self.radius = radius
        self.color1 = Colour(0.0, 0.0, 0.6)
        self.color2 = Colour(0.0, 0.0, 0.8)
        self.color3 = Colour(0.0, 0.0, 1.0)
        self.color4 = Colour(0.2, 0.2, 1.0)
        self.color5 = Colour(0.4, 0.4, 1.0)
        self.color6 = Colour(0.6, 0.6, 1.0)
        self.large = max(pmaplst)
        self.small = min(pmaplst)
        self.range=(self.large-self.small)/6

        assert self._map.size.n_dim == 2, "Unsupported map dimension, expected 2D"

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

        for point in points:
            if point[1] < self.small+(self.range*1):
                self._root_view.render_pos(Entity(point[0], self.radius), self.color6)
            elif point[1] < self.small+(self.range*2):
                self._root_view.render_pos(Entity(point[0], self.radius), self.color5)
            elif point[1] < self.small+(self.range*3):
                self._root_view.render_pos(Entity(point[0], self.radius), self.color4)
            elif point[1] < self.small+(self.range*4):
                self._root_view.render_pos(Entity(point[0], self.radius), self.color3)
            elif point[1] < self.small+(self.range*5):
                self._root_view.render_pos(Entity(point[0], self.radius), self.color2)
            elif point[1] < self.small+(self.range*6):
                self._root_view.render_pos(Entity(point[0], self.radius), self.color1)    
        return True
