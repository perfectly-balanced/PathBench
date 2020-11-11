from simulator.services.event_manager.events.event import Event


class TakeScreenshotTexEvent(Event):
    def __init__(self) -> None:
        super().__init__("TakeScreenshotTex event", True)
