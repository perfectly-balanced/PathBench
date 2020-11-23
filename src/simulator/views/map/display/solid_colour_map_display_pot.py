from typing import Set, Union, Tuple, List
import copy

from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map.display.map_display import MapDisplay
from structures import Point, Colour

class SolidColourMapDisplaypot(MapDisplay):
    radius: int
    points: Union[Set[Point], List[Entity]]
    pts: Set[Point]
    point_dim: int

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
        self.range = (self.large-self.small)/6
        self.pts = set()
        self.__point_dim = None

    def render(self, *discarded) -> None:
        points: Union[Set[Point], List[Entity]] = copy.deepcopy(self.points)
        if not points:
            return

        def f(pt):
            if isinstance(pt, Entity):
                pt = pt.position
            return pt

        self.pts = set(map(f, points))
        self.__point_dim = next(iter(self.pts))[0].n_dim

        for p in self.pts:
            self._root_view.cube_requires_update(Entity(p[0], self.radius))
            

    def update_cube(self, p: Point) -> None:
        if self.__point_dim != 3:
            p = Point(p.x, p.y)
        
        if p in self.pts:
            if p[1] < self.small+(self.range*1):
                self._root_view.colour_cube(self.color6)
            elif p[1] < self.small+(self.range*2):
                self._root_view.colour_cube(self.color5)
            elif p[1] < self.small+(self.range*3):
                self._root_view.colour_cube(self.color4)
            elif p[1] < self.small+(self.range*4):
                self._root_view.colour_cube(self.color3)
            elif p[1] < self.small+(self.range*5):
                self._root_view.colour_cube(self.color2)
            elif p[1] < self.small+(self.range*6):
                self._root_view.colour_cube(self.color1)
