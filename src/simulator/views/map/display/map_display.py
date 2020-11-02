from typing import Optional, Callable

from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.view import View
from structures import Point


class MapDisplay(View):
    services: Services
    z_index: int
    render_method: Optional[Callable[[], None]]
    _map: Map

    def __init__(self, services: Services, render_method: Optional[Callable[[], None]] = None,
                 z_index: int = 0, custom_map: Map = None) -> None:
        super().__init__(services, None, None)
        self.services = services
        self.render_method = render_method
        self.z_index = z_index

        self._map = self.services.algorithm.map

        if custom_map:
            self._map = custom_map

    def render(self) -> bool:
        if self.render_method is not None:
            self.render_method()

        return True
    
    def update_cube(self, p: Point) -> None:
        pass

    def get_renderer_view(self) -> Optional['MapView']:
        from simulator.views.map.map_view import MapView
        if isinstance(self._root_view, MapView):
            return self._root_view

    def __lt__(self, other):
        return True
