from panda3d.core import NodePath, TransparencyAttrib, LVecBase3f

from structures import DynamicColour, Colour, TRANSPARENT
from simulator.services.services import Services
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.colour_update_event import ColourUpdateEvent
from simulator.services.event_manager.events.key_frame_event import KeyFrameEvent

from typing import Optional, Callable, Dict

class MapData:
    _services: Services
    __name: str
    __root: NodePath
    __colour_callbacks: Dict[str, Callable[[DynamicColour], None]]

    BG: str = "background"

    def __init__(self, services: Services, parent: NodePath, name: str = "map"):
        self._services = services
        self.__name = name
        self.__colour_callbacks = {}

        self._services.ev_manager.register_listener(self)

        self.__root = parent.attach_new_node(self.name)
        self.root.set_transparency(TransparencyAttrib.M_alpha)

        self._add_colour(MapData.BG, Colour(0, 0, 0.2, 1), callback=lambda dc: self._services.graphics.window.set_background_color(*dc()))

    def notify(self, event: Event) -> None:
        if isinstance(event, ColourUpdateEvent):
            if event.colour.name in self.__colour_callbacks:
                self.__colour_callbacks[event.colour.name](event.colour)

    @property
    def root(self) -> str:
        return 'root'

    @root.getter
    def root(self) -> NodePath:
        return self.__root

    @property
    def name(self) -> str:
        return 'name'

    @name.getter
    def name(self) -> NodePath:
        return self.__name

    def _add_colour(self, name: str, default_colour: Colour, default_visible: bool = True, invoke_callback: bool = True, callback: Optional[Callable[[DynamicColour], None]] = None) -> DynamicColour:
        dc = self._services.state.add_colour(name, default_colour, default_visible)
        if callback is None:
            return
        if name not in self.__colour_callbacks:
            self.__colour_callbacks[name] = callback
        if invoke_callback:
            callback(dc)
        return dc
