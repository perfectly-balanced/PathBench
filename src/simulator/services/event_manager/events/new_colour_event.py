from simulator.services.event_manager.events.event import Event
from structures import DynamicColour

class NewColourEvent(Event):
    __colour: DynamicColour

    def __init__(self, colour: DynamicColour) -> None:
        super().__init__("NewColour event")
        self.__colour = colour

    @property
    def colour(self) -> DynamicColour:
        return self.__colour
