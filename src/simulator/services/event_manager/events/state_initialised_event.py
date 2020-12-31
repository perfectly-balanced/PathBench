from simulator.services.event_manager.events.event import Event


class StateInitialisedEvent(Event):
    def __init__(self) -> None:
        super().__init__("SetInitialised event", True)
