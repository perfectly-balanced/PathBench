from simulator.services.event_manager.events.event import Event
from simulator.services.services import Services


class Model:
    """
    The model base class
    All logic should inherit from this class
    """

    _services: Services

    def __init__(self, services: Services) -> None:
        self._services = services
        self._services.ev_manager.register_listener(self)

    def notify(self, event: Event) -> None:
        pass
