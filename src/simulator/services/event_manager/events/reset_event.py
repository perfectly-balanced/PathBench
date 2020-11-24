from simulator.services.event_manager.events.event import Event


class ResetEvent(Event):
    """
    Current running algorithm should be reset.
    """

    def __init__(self) -> None:
        super().__init__("Reset event")
