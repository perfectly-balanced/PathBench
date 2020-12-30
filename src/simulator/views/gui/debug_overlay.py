from direct.showbase.ShowBase import ShowBase
from simulator.services.services import Services
from simulator.services.debug import DebugLevel
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import TextNode
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.colour_update_event import ColourUpdateEvent
from structures import DynamicColour, Colour, TRANSPARENT, WHITE, BLACK
from typing import Optional, Callable


class DebugOverlay():
    __services: Services
    __base: ShowBase

    def __init__(self, services: Services):
        self.__services = services
        self.__base = self.__services.graphics.window
        self.__labels = ["Map:", "Goal:", "Agent:", "Algorithm:", "State:"]

        self.__debug_labels = []
        for i in range(0, 5):
            l = OnscreenText(text=self.__labels[i],
                             parent=self.__base.aspect2d,
                             pos=(-1.45, 0.92 - i * 0.05),
                             mayChange=True,
                             align=TextNode.ALeft,
                             scale=0.03)
            self.__debug_labels.append(l)

        self.__debug_var = []
        for i in range(0, 5):
            v = OnscreenText(text="..",
                             parent=self.__base.aspect2d,
                             pos=(-1.3, 0.92 - i * 0.05),
                             mayChange=True,
                             align=TextNode.ALeft,
                             scale=0.03)
            self.__debug_var.append(v)

        self.__services.ev_manager.register_listener(self)
        self.__text_colour = self.__services.state.add_colour("debug overlay", WHITE)

    def set_text_colour(self, colour):
        for i in range(0, 5):
            self.__debug_labels[i].setFg(colour)
            self.__debug_var[i].setFg(colour)

    # use setText(), getText(), clearText()

    def notify(self, event: Event) -> None:
        if isinstance(event, ColourUpdateEvent):
            if event.colour.name == self.__text_colour.name:
                self.set_text_colour(self.__text_colour())
