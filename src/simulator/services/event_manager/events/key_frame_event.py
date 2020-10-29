from simulator.services.event_manager.events.event import Event
from typing import Optional


class KeyFrameEvent(Event):
    __is_first: bool
    __refresh: bool

    def __init__(self, is_first: bool = False, refresh: Optional[bool] = None) -> None:
        super().__init__("KeyFrame event", True)
        self.__is_first = is_first
        if refresh is None:
            self.__refresh = self.is_first
        else:
            self.__refresh = refresh

    @property
    def is_first(self) -> bool:
        return self.__is_first

    @property
    def refresh(self) -> bool:
        return self.__refresh
