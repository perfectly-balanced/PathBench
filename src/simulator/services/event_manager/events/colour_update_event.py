from simulator.services.event_manager.events.event import Event
from structures import DynamicColour
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from simulator.services.persistent_state.persistent_state_view import PersistentStateView

class ColourUpdateEvent(Event):
    __colour: DynamicColour
    __view: 'PersistentStateView'

    def __init__(self, colour: DynamicColour, view: 'PersistentStateView') -> None:
        super().__init__("ColourUpdate event")
        self.__colour = colour
        self.__view = view

    @property
    def colour(self) -> DynamicColour:
        return self.__colour

    @property
    def view(self) -> 'PersistentStateView':
        return self.__view
