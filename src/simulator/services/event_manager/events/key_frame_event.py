from simulator.services.event_manager.events.event import Event


class KeyFrameEvent(Event):
    def __init__(self) -> None:
        super().__init__()
        self._name = "KeyFrame event"
