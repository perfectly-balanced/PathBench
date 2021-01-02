from simulator.services.event_manager.events.event import Event


class StateInitialisingEvent(Event):
    def __init__(self) -> None:
        super().__init__("SetInitialising event", True)
