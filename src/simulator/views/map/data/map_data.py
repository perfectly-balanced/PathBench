from panda3d.core import NodePath, TransparencyAttrib, LVecBase3f

from structures import DynamicColour, Colour, TRANSPARENT, WHITE, BLACK
from simulator.services.services import Services
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.colour_update_event import ColourUpdateEvent
from simulator.services.event_manager.events.key_frame_event import KeyFrameEvent
from utility.compatibility import Final

from typing import Optional, Callable, Dict, Any
from abc import ABC, abstractmethod

import numpy as np
from nptyping import NDArray

class MapData(ABC):
    _services: Services
    __name: Final[str]
    __root: Final[NodePath]
    __colour_callbacks: Dict[str, Callable[[DynamicColour], None]]

    TRAVERSABLE_MASK: Final = np.uint8(1 << 0)
    OBSTACLE_MASK: Final = np.uint8(1 << 1)
    UNMAPPED_MASK: Final = np.uint8(1 << 2)
    EXTENDED_WALL_MASK: Final = np.uint8(1 << 3)

    data: Final[NDArray[(Any, Any, Any), np.uint8]]

    # REQUIRED COLOURS / COMPONENTS #
    BG: Final[str] = "background"
    
    # OPTIONAL COLOURS / COMPONENTS #
    TRAVERSABLES: Final[str] = "traversables"
    TRAVERSABLES_WF: Final[str] = "traversables w.f."
    OBSTACLES: Final[str] = "obstacles"
    OBSTACLES_WF: Final[str] = "obstacles w.f."
    AGENT: Final[str] = "agent"
    TRACE: Final[str] = "trace"
    GOAL: Final[str] = "goal"

    _DEFAULTS: Final[Dict[str, Colour]] = {BG: Colour(0, 0, 0.2, 1),
                                           TRAVERSABLES: WHITE,
                                           TRAVERSABLES_WF: BLACK,
                                           OBSTACLES: BLACK,
                                           OBSTACLES_WF: WHITE,
                                           AGENT: Colour(0.8, 0, 0),
                                           TRACE: Colour(0, 0.9, 0),
                                           GOAL: Colour(0, 0.9, 0)}

    logical_w: Final[int] # x-axis (width)
    logical_h: Final[int] # y-axis (height)
    logical_d: Final[int] # z-axis (depth)

    def __init__(self, services: Services, data: NDArray[(Any, Any, Any), bool], parent: NodePath, name: str = "map"):
        self._services = services
        self.__name = name
        self.__colour_callbacks = {}

        self.data = data

        self.logical_w = int(self.data.shape[0])
        self.logical_h = int(self.data.shape[1])
        self.logical_d = int(self.data.shape[2])

        self._services.ev_manager.register_listener(self)

        self.__root = parent.attach_new_node(self.name)
        self.root.set_transparency(TransparencyAttrib.M_alpha)

        self._add_colour(MapData.BG, callback=lambda dc: self._services.graphics.window.set_background_color(*dc()))

    def notify(self, event: Event) -> None:
        if isinstance(event, ColourUpdateEvent):
            if event.colour.name in self.__colour_callbacks:
                self.__colour_callbacks[event.colour.name](event.colour)

    @property
    @abstractmethod
    def dim(self) -> int:
        ...

    @property
    def root(self) -> NodePath:
        return self.__root

    @property
    def name(self) -> NodePath:
        return self.__name

    def _add_colour(self, name: str, default_colour: Optional[Colour] = None, default_visible: bool = True, invoke_callback: bool = True, callback: Optional[Callable[[DynamicColour], None]] = None) -> DynamicColour:
        if default_colour is None:
            default_colour = MapData._DEFAULTS[name]
        dc = self._services.state.views.add_colour(name, default_colour, default_visible)
        if callback is None:
            return dc
        if name not in self.__colour_callbacks:
            self.__colour_callbacks[name] = callback
        if invoke_callback:
            callback(dc)
        return dc
    
    def center(self) -> None:
        world = self.root.get_parent()
        (x1, y1, z1), (x2, y2, z2) = self.root.get_tight_bounds()
        self.root.set_pos(world.getX() - (x2 - x1) / 2,
                          world.getY() - (y2 - y1) / 2,
                          world.getZ() - (z2 - z1) / 2)

    def destroy(self) -> None:
        self.__root.remove_node()
        self._services.ev_manager.unregister_listener(self)
