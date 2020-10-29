from simulator.services.event_manager.events.event import Event


class QuitEvent(Event):
    """
    Quit event.
    """

    def __init__(self) -> None:
        super().__init__("Quit event", True)
