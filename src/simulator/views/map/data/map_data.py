from panda3d.core import NodePath, TransparencyAttrib, LVecBase3f

from structures import DynamicColour, Colour, TRANSPARENT
from simulator.services.services import Services
from simulator.services.event_manager.events.key_frame_event import KeyFrameEvent

from typing import Optional, Callable, Dict

class MapData:
    _services: Services
    __name: str
    __root: NodePath
    colours: Dict[str, DynamicColour]

    BG: str = "background"

    def __init__(self, services: Services, parent: NodePath, name: str = "map"):
        self._services = services
        self.__name = name

        self.__root = parent.attach_new_node(self.name)
        self.root.set_transparency(TransparencyAttrib.M_alpha)

        self.colours = {}
        self.add_colour(MapData.BG, Colour(0, 0, 0.2, 1), callback=lambda dc: self._services.graphics.window.set_background_color(*dc()))

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

    def _dynamic_colour_callback(self, colour: DynamicColour) -> None:
        self._services.ev_manager.post(KeyFrameEvent(refresh=True))

    def add_colour(self, name: str, default_colour: Colour, default_visible: bool = True, invoke_callback: bool = True, callback: Optional[Callable[[DynamicColour], None]] = None) -> DynamicColour:
        if callback is None:
            callback = self._dynamic_colour_callback
        if name not in self.colours:
            dc = self.colours[name] = DynamicColour(default_colour, name, callback, default_visible)
        else:
            dc = self.colours[name]
        if invoke_callback:
            callback(dc)
        return dc
    
    def set_visibility(self, name: str, visible: bool) -> None:
        self.colours[name].visible = visible

    def set_colour(self, name: str, colour: Colour) -> None:
        self.colours[name].colour = colour