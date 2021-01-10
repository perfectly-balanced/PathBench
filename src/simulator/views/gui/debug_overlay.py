from direct.showbase.ShowBase import ShowBase
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import TextNode

from typing import Any

from simulator.services.services import Services
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.colour_update_event import ColourUpdateEvent
from simulator.services.event_manager.events.toggle_debug_overlay_event import ToggleDebugOverlayEvent
from simulator.services.event_manager.events.state_initialising_event import StateInitialisingEvent
from simulator.services.event_manager.events.state_running_event import StateRunningEvent
from simulator.services.event_manager.events.state_initialised_event import StateInitialisedEvent
from simulator.services.event_manager.events.state_done_event import StateDoneEvent
from simulator.services.event_manager.events.state_terminated_event import StateTerminatedEvent
from simulator.services.event_manager.events.state_entity_update_event import StateEntityUpdateEvent
from structures import DynamicColour, Colour, TRANSPARENT, WHITE, BLACK

class DebugOverlay():
    __services: Services
    __base: ShowBase
    __visible: bool

    def __init__(self, services: Services) -> None:
        self.__services = services
        self.__base = self.__services.graphics.window
        self.__labels = ["Map:", "Goal:", "Agent:", "Algorithm:", "State:"]
        self.__visible = True

        self.__debug_labels = []
        for i in range(0, 5):
            l = OnscreenText(text=self.__labels[i],
                             parent=self.__base.render2d,
                             pos=(-0.865, 0.92 - i * 0.05),
                             mayChange=True,
                             align=TextNode.ARight,
                             scale=(0.02, 0.029, 0.73))
            self.__debug_labels.append(l)

        self.__debug_var = []
        for i in range(0, 5):
            v = OnscreenText(text="-",
                             parent=self.__base.render2d,
                             pos=(-0.855, 0.92 - i * 0.05),
                             mayChange=True,
                             align=TextNode.ALeft,
                             scale=(0.02, 0.029, 0.73))
            self.__debug_var.append(v)

        self.__services.ev_manager.register_listener(self)
        self.__text_colour = self.__services.state.views.add_colour("debug overlay", WHITE)

        if self.__services.settings.map_name:
            self.set_text(0, self.__services.settings.map_name)
        if self.__services.settings.algorithm_name:
            self.set_text(3, self.__services.settings.algorithm_name)

    def set_text_colour(self, colour: Colour) -> None:
        for i in range(0, 5):
            self.__debug_labels[i].setFg(colour)
            self.__debug_var[i].setFg(colour)

    def set_text(self, i: int, text: Any) -> None:
        self.__debug_var[i].setText(str(text))

    def toggle_visible(self):
        if self.__visible:
            for i in range(0, 5):
                self.__debug_labels[i].hide()
                self.__debug_var[i].hide()
        else:
            for i in range(0, 5):
                self.__debug_labels[i].show()
                self.__debug_var[i].show()
        self.__visible = not self.__visible

    def notify(self, event: Event) -> None:
        if isinstance(event, StateInitialisedEvent):
            self.set_text(4, "Initialised")
            self.set_text(1, self.__services.algorithm.map.goal.position)
            self.set_text(2, self.__services.algorithm.map.agent.position)
        elif isinstance(event, StateEntityUpdateEvent):
            self.set_text(1, self.__services.algorithm.map.goal.position)
            self.set_text(2, self.__services.algorithm.map.agent.position)
        elif isinstance(event, StateInitialisingEvent):
            self.set_text(4, "Initialising")
            self.set_text(0, self.__services.settings.map_name)
            self.set_text(3, self.__services.settings.algorithm_name)
            self.set_text(1, "-")
            self.set_text(2, "-")
        elif isinstance(event, StateRunningEvent):
            self.set_text(4, "Running")
        elif isinstance(event, StateTerminatedEvent):
            self.set_text(4, "Terminated")
        elif isinstance(event, StateDoneEvent):
            self.set_text(4, "Done")
        elif isinstance(event, ColourUpdateEvent):
            if event.colour.name == self.__text_colour.name:
                self.set_text_colour(self.__text_colour())
        elif isinstance(event, ToggleDebugOverlayEvent):
            self.toggle_visible()
