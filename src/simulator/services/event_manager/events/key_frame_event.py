from simulator.services.event_manager.events.event import Event


class KeyFrameEvent(Event):
    __is_first: bool

    def __init__(self, is_first: bool = False) -> None:
        super().__init__()
        self._name = "KeyFrame event"
        self.__is_first = is_first

    @property
    def is_first(self) -> bool:
        return self.__is_first
