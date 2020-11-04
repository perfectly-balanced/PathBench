from simulator.services.event_manager.events.event import Event

class ToggleSimulatorConfigEvent(Event):
    def __init__(self) -> None:
        super().__init__("ToggleSimulatorConfig event", True)
