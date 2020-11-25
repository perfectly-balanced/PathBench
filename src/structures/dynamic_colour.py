from typing import Optional, Callable

from structures.colour import Colour, TRANSPARENT
from utility.compatibility import Final

class DynamicColour:
    __colour: Colour
    __callback: Optional[Callable[['DynamicColour'], None]]
    __name: Optional[str]
    __visible: bool

    def __init__(self, colour: Colour, name: Optional[str] = None, callback: Optional[Callable[['DynamicColour'], None]] = None, visible: bool = True):
        self.__colour = colour
        self.__name = name
        self.__visible = visible
        self.__callback = callback

    @property
    def name(self) -> Optional[str]:
        return self.__name

    @property
    def deduced_colour(self) -> Colour:
        return self.__colour if self.visible else TRANSPARENT

    @property
    def colour(self) -> str:
        return 'colour'

    @colour.getter
    def colour(self) -> Colour:
        return self.__colour

    @colour.setter
    def colour(self, colour: Colour) -> None:
        self.__colour = colour
        if self.__callback is not None:
            self.__callback(self)

    @property
    def visible(self) -> str:
        return 'visible'

    @visible.getter
    def visible(self) -> bool:
        return self.__visible

    @visible.setter
    def visible(self, visible: bool) -> None:
        self.__visible = visible
        if self.__callback is not None:
            self.__callback(self)

    def set_all(self, colour: Colour, visible: bool) -> None:
        self.__colour = colour
        self.__visible = visible
        if self.__callback is not None:
            self.__callback(self)

    def __repr__(self) -> str:
        return f"DynamicColour(colour={self.__colour}, visible={self.visible})"

    def __call__(self) -> Colour:
        return self.deduced_colour
