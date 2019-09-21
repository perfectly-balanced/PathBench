from simulator.services.event_manager.events.event import Event


class TakeScreenshotEvent(Event):
    def __init__(self) -> None:
        super().__init__()
        self._name = "TakeScreenshot event"
