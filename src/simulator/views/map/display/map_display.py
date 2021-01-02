from typing import Optional, List

from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.view import View
from structures import Point
from structures.tracked import Tracked


class MapDisplay(View):
    services: Services
    z_index: int
    _map: Map
    _previous_cube_update_displays_idx: int
    updates_cubes: bool

    def __init__(self, services: Services, z_index: int = 0, custom_map: Map = None) -> None:
        super().__init__(services, None, None)
        self.services = services
        self.z_index = z_index

        self._map = self.services.algorithm.map
        if custom_map:
            self._map = custom_map

        self._previous_cube_update_displays_idx = None
        self.updates_cubes = False

    def render(self, refresh: bool) -> None:
        pass
    
    def update_cube(self, p: Point) -> None:
        pass

    def get_tracked_data(self) -> List[Tracked]:
        return []

    def request_update_all_cubes(self) -> None:
        pass
    
    def get_renderer_view(self) -> Optional['MapView']:
        from simulator.views.map.map_view import MapView
        if isinstance(self._root_view, MapView):
            return self._root_view

    def __lt__(self, other):
        return self.z_index < other.z_index
