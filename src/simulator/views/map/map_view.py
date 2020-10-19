from time import sleep

from heapq import heappush, heappop
from typing import List, Any, Tuple, Optional, Union

from algorithms.configuration.entities.entity import Entity
from simulator.models.model import Model
from simulator.services.debug import DebugLevel
from simulator.services.services import Services
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.window_loaded_event import WindowLoadedEvent
from simulator.services.event_manager.events.key_frame_event import KeyFrameEvent
from simulator.services.event_manager.events.take_screenshot_event import TakeScreenshotEvent

from simulator.views.map_displays.empty_map_display import EmptyMapDisplay
from simulator.views.map_displays.entities_map_display import EntitiesMapDisplay
from simulator.views.map_displays.map_display import MapDisplay
from simulator.views.map_displays.numbers_map_display import NumbersMapDisplay
from simulator.views.map_displays.online_lstm_map_display import OnlineLSTMMapDisplay
from simulator.views.view import View
from structures import Point

from simulator.views.panda.gui.view_editor import ViewEditor
from simulator.views.map.camera import Camera
from simulator.views.map.voxel_map import VoxelMap

from panda3d.core import NodePath

class MapView(View):
    __displays: List[Any]

    __world: NodePath
    __map: VoxelMap
    __cam: Camera
    __vs: ViewEditor

    def __init__(self, services: Services, model: Model, root_view: Optional[View]) -> None:
        super().__init__(services, model, root_view)        
        self.__displays = []

    def notify(self, event: Event) -> None:
        super().notify(event)
        if isinstance(event, WindowLoadedEvent):
            self.__init()
        elif isinstance(event, KeyFrameEvent):
            self.update_view()
        elif isinstance(event, TakeScreenshotEvent):
            self.take_screenshot()

    def __init(self):
        base = self._services.window

        # disables the default camera behaviour
        base.disable_mouse()

        base.set_background_color(0, 0, 0.2, 1)

        # Creating the world origin as a dummy node
        self.__world = base.render.attach_new_node("world")

        map_data = {}
        for x in range(0, self._services.algorithm.map.size.width):
            map_data[x] = {}
            for y in range(0, self._services.algorithm.map.size.height):
                map_data[x][y] = {}
                map_data[x][y][0] = not self._services.algorithm.map.is_agent_valid_pos(Point(x, y, 0))
        start_pos = Point(*self._services.algorithm.map.agent.position, 0)
        goal_pos = Point(*self._services.algorithm.map.goal.position, 0)

        # MAP #
        self.__map = VoxelMap(map_data, self.__world, start_pos=start_pos, goal_pos=goal_pos, artificial_lighting=True)

        # map centering - todo: use tight bounds
        self.__map.root.set_pos(self.__world.getX() - len(self.map.traversables_data) / 2, self.__world.getY() - len(self.map.traversables_data) / 2, self.__world.getZ() - len(self.map.traversables_data) / 2)

        # CAMERA #
        self.__cam = Camera(base, base.cam, self.__world)

        # GUI #
        self.__vs = ViewEditor(base, self.map)

        self.update_view()

    @property
    def map(self) -> str:
        return 'map'

    @map.getter
    def map(self) -> VoxelMap:
        return self.__map

    def __get_displays(self) -> None:
        self.__displays = []
        displays: List[MapDisplay] = [EmptyMapDisplay(self._services),
                                      EntitiesMapDisplay(self._services)]
        # drop map displays if not compatible with display format
        displays += list(filter(lambda d: self._services.settings.simulator_grid_display or not isinstance(d, NumbersMapDisplay),
                                self._services.algorithm.instance.get_display_info()))
        for display in displays:
            display._model = self._model
            self.add_child(display)
            heappush(self.__displays, (display.z_index, display))

    def __render_displays(self) -> None:
        while len(self.__displays) > 0:
            display: MapDisplay
            _, display = heappop(self.__displays)
            # todo: display.render() # update simulated map data (colours)
        # todo: update actual viewable map data

    def update_view(self) -> None:
        self.__get_displays()
        self.__render_displays()

    def take_screenshot(self) -> None:
        """
        self._model.save_screenshot(image)
        """
        print("TODO: take_screenshot")
