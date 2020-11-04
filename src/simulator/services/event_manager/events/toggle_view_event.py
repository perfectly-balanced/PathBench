from simulator.services.event_manager.events.event import Event

class ToggleViewEvent(Event):
    def __init__(self) -> None:
        super().__init__("ToggleView event", True)
