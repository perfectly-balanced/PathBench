from simulator.services.event_manager.events.event import Event
from structures import Point

from typing import List

class MapUpdateEvent(Event):
    __updated_cells: List[Point]

    def __init__(self, updated_cells: List[Point]) -> None:
        super().__init__("MapUpdate event")
        self.__updated_cells = updated_cells

    @property
    def updated_cells(self) -> List[Point]:
        return self.__updated_cells
