from simulator.services.event_manager.events.event import Event

class ToggleDebugOverlayEvent(Event):
    def __init__(self) -> None:
        super().__init__("ToggleDebugOverlay event", True)
