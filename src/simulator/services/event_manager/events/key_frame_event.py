from simulator.services.event_manager.events.event import Event
from typing import Optional


class KeyFrameEvent(Event):
    __refresh: bool

    def __init__(self, refresh: bool = False) -> None:
        super().__init__("KeyFrame event", True)
        self.__refresh = refresh

    @property
    def refresh(self) -> bool:
        return self.__refresh

    def _absorb(self, other: 'Event') -> None:
        self.__refresh = self.refresh or other.refresh
