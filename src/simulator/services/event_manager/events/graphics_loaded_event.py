from simulator.services.event_manager.events.event import Event


class GraphicsLoadedEvent(Event):
    def __init__(self) -> None:
        super().__init__()
        self._name = "WindowLoaded event"
