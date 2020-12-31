from simulator.services.event_manager.events.event import Event


class StateDoneEvent(Event):
    def __init__(self) -> None:
        super().__init__("SetDone event", True)
