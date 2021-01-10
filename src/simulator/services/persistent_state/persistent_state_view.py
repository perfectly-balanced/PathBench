from structures import DynamicColour, Colour
from simulator.services.event_manager.events.colour_update_event import ColourUpdateEvent
from simulator.services.persistent_state.persistent_state_object import PersistentStateObject
from simulator.services.persistent_state.persistent_state import PersistentState
from utility.compatibility import Final

from typing import Dict, Any

class PersistentStateView(PersistentStateObject):
    _state: PersistentState
    index: int
    colours: Dict[str, DynamicColour]

    EFFECTIVE_VIEW: Final[int] = -1

    def __init__(self, state: PersistentState, index: int) -> None:
        super().__init__(state)
        self.index = index

        self.colours = {}

    def __colour_callback(self, colour: DynamicColour) -> None:
        if self.index == self._state.views.view_idx:
            self._state.views.effective_view.colours[colour.name].set_all(colour.colour, colour.visible)
        for s in self._state.all_services:
            s.ev_manager.post(ColourUpdateEvent(colour, self))

    def _add_colour(self, name: str, colour: Colour, visible: bool) -> DynamicColour:
        if name in self.colours:
            return self.colours[name]
        dc = self.colours[name] = DynamicColour(colour, name, self.__colour_callback, visible)
        self.__colour_callback(dc)
        return dc

    def _from_view(self, other: 'PersistentStateView') -> None:
        self.colours = {k: v for k, v in self.colours.items() if k in other.colours}
        for n, c in other.colours.items():
            if n not in self.colours:
                self._add_colour(n, c.colour, c.visible)
            else:
                self.colours[n].set_all(c.colour, c.visible)

    def _from_json(self, data: Dict[str, Any]) -> None:
        for n, c in data["colours"].items():
            colour = Colour(*c["colour"])
            visible = bool(c["visible"])
            self.colours[n] = DynamicColour(colour, n, self.__colour_callback, visible)

    def _to_json(self) -> Dict[str, Any]:
        data = {}
        dc = data["colours"] = {}
        for n, c in self.colours.items():
            dc[n] = {}
            dc[n]["colour"] = tuple(c.colour)
            dc[n]["visible"] = c.visible
        return data

    def is_effective(self) -> bool:
        return self.index == PersistentStateView.EFFECTIVE_VIEW
