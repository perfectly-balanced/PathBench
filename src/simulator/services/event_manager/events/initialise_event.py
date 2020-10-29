from simulator.services.event_manager.events.event import Event


class InitialiseEvent(Event):
    """
    Tells all listeners to initialise themselves.
    This includes loading libraries and resources.

    Avoid initializing such things within listener __init__ calls
    to minimize snafus (if some rely on others being yet created.)
    """

    def __init__(self) -> None:
        super().__init__("Initialise event")
