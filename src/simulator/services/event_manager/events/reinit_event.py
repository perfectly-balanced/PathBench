from simulator.services.event_manager.events.event import Event


class ReinitEvent(Event):
    """
    Notifies all listeners that there has been
    a call to Services.reset(). In essence, the
    algorithm runner has been re-initialised with
    the newest version of the settings.
    """

    def __init__(self) -> None:
        super().__init__("Reinit event")
