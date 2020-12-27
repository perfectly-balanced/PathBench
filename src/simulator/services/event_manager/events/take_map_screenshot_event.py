from simulator.services.event_manager.events.event import Event


class TakeMapScreenshotEvent(Event):
    def __init__(self) -> None:
        super().__init__("TakeMapScreenshot event", True)
