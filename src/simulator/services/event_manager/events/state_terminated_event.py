from simulator.services.event_manager.events.event import Event


class StateTerminatedEvent(Event):
    def __init__(self) -> None:
        super().__init__("SetTerminated event", True)
