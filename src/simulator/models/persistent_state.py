
from structures import DynamicColour, Colour, TRANSPARENT
from simulator.services.services import Services
from simulator.models.model import Model

from typing import Optional, Callable, Dict
from functools import partial
import json

class View():
    __services: Services
    state: 'PersistentState'
    index: int

    colours: Dict[str, DynamicColour]

    def __init__(self, services: Services, state: 'PersistentState', index: int) -> None:
        self.__services = services
        self.state = state
        self.index = index

        self.colours = {}
    
    def __colour_callback(self, colour: DynamicColour) -> None:
        if self.index == self.state.view_idx:
            self.state.colours[colour.name].set_all(colour.colour, colour.visible)
        self.__services.ev_manager.post(ColourUpdateEvent(colour, view=self))

    def _add_colour(self, name: str, colour: Colour, visible: bool) -> DynamicColour:
        dc = self.colours[name] = DynamicColour(colour, name, self._colour_callback, visible)
        return dc

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
            dc[n]["colour"] = tuple(*c.colour)
            dc[n]["visible"] = tuple(*c.visible)
        return data

class PersistentState(Model):
    __services: Services

    colours: Dict[str, DynamicColour]
    views: List[View]
    __view_idx: int
    
    MAX_VIEWS: int = 6

    FILE_NAME: str = ".pathbench.json"
    
    def __init__(self, services: Services):
        self.__services = services

        self.colours = {}
        self.views = [View(services, self, i) for i in range(self.MAX_VIEWS)]
        self.__view_idx = 0

        self.__load()

    def __load(self) -> None:
        def check(req: bool) -> None:
            if not req:
                raise RuntimeError("{} is corrupted".format(self.FILE_NAME))

        if os.path.isfile(self.FILE_NAME):
            with open(self.FILE_NAME, 'r') as f:
                data = json.load(f)
                
                try:
                    check(isinstance(data, list))
                    check(len(data) == self.MAX_VIEWS)
                    for v in range(len(data)):
                        check(isinstance(v, dict))
                        self.views[v]._from_json(data[v])
                except RuntimeError as e:
                    print(e)
    
    def __save(self) -> None:
        pass

    def __colour_callback(self, colour: DynamicColour) -> None:
        self.__services.ev_manager.post(ColourUpdateEvent(colour))

    def add_colour(self, name: str, default_colour: Colour, default_visible: bool = True) -> DynamicColour:
        if name in self.colours:
            return self.colours[name]
        
        dc = self.colours[name] = DynamicColour(default_colour, name, self.__colour_callback, default_visible)
        for v in range(len(self.views)):
            self.views[v]._add_colour(name, default_colour, default_visible)
        callback(dc)
        return dc

    @property
    def view_idx(self) -> str:
        return 'view_idx'
    
    @view_idx.getter
    def view_idx(self) -> int:
        return self.__view_idx
    
    @view_idx.setter
    def view_idx(self, idx: int) -> None:
        self.__view_idx = idx
        for n, c in self.views[idx].colours.items():
            self.colours[n].set_all(c.colour, c.visible)

    @property
    def view(self) -> str:
        return 'view'

    @view.getter
    def view(self) -> View:
        return self.views[self.view_idx]
    
    @view.setter
    def view(self, v: View) -> None:
        assert v == self.views[v.index], "View is not owned by this PersistentState"

        self.__view_idx = v.index
        for n, c in v.colours.items():
            self.colours[n].set_all(c.colour, c.visible)
