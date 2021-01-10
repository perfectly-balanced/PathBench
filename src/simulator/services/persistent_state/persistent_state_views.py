from structures import DynamicColour, Colour
from simulator.services.event_manager.events.new_colour_event import NewColourEvent
from simulator.services.persistent_state.persistent_state_object import PersistentStateObject
from simulator.services.persistent_state.persistent_state_view import PersistentStateView
from simulator.services.debug import DebugLevel
from utility.misc import exclude_from_dict
from utility.compatibility import Final

from weakref import WeakSet
from typing import Dict, Any, List, Optional
import json
import os
import copy
import traceback

class PersistentStateViews(PersistentStateObject):
    MAX_VIEWS: Final[int] = 6
    effective_view: PersistentStateView
    views: List[PersistentStateView]
    __view_idx: int

    def __init__(self, *args):
        super().__init__(*args)
        self.effective_view = PersistentStateView(self._state, PersistentStateView.EFFECTIVE_VIEW)
        self.views = [PersistentStateView(self._state, i) for i in range(self.MAX_VIEWS)]
        self.__view_idx = 0

    def _from_json(self, data: Dict[str, Any]) -> None:
        vs = data["views"]
        for v in range(self.MAX_VIEWS):
            self.views[v]._from_json(vs[v])
        self.view_idx = data["view_index"]

    def _to_json(self) -> Dict[str, Any]:
        data = {}
        data["view_index"] = self.view_idx
        data["views"] = [self.views[v]._to_json() for v in range(self.MAX_VIEWS)]
        return data

    def add_colour(self, name: str, default_colour: Colour, default_visible: bool = True, save_delay: float = 0) -> DynamicColour:
        if name in self.effective_view.colours:
            return self.effective_view.colours[name]

        dc = self.effective_view._add_colour(name, default_colour, default_visible)
        for v in range(self.MAX_VIEWS):
            self.views[v]._add_colour(name, default_colour, default_visible)
        self._state.root_services.ev_manager.post(NewColourEvent(dc))
        self._state.schedule_save(save_delay)
        return dc

    def restore_effective_view(self):
        self.effective_view._from_view(self.view)

    def apply_effective_view(self, save_delay: float = 0):
        self.view._from_view(self.effective_view)
        self._state.schedule_save(save_delay)

    @property
    def view_idx(self) -> str:
        return 'view_idx'

    @view_idx.getter
    def view_idx(self) -> int:
        return self.__view_idx

    @view_idx.setter
    def view_idx(self, value: Any) -> None:
        idx, save_delay = (value, 4.0) if isinstance(value, int) else value
        if idx < 0 or idx >= self.MAX_VIEWS:
            raise IndexError("index '{}' not in bounds, expected 0 < view_idx: int < {}.".format(idx, self.MAX_VIEWS))
        self.__view_idx = idx
        self.effective_view._from_view(self.views[idx])
        self._state.schedule_save(save_delay)

    @property
    def view(self) -> str:
        return 'view'

    @view.getter
    def view(self) -> PersistentStateView:
        return self.views[self.view_idx]

    @view.setter
    def view(self, v: PersistentStateView) -> None:
        assert v == self.views[v.index], "PersistentStateView is not owned by this PersistentStateViews"
        self.view_idx = v.index
