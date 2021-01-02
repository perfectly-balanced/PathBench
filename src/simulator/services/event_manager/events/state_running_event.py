from simulator.services.event_manager.events.event import Event


class StateRunningEvent(Event):
    def __init__(self) -> None:
        super().__init__("SetRunning event", True)
