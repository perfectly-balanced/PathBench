from simulator.models.model import Model
from simulator.services.event_manager.events.event import Event
from simulator.services.services import Services


class Controller:
    """
    The controller base class
    All controllers should inherit from this class
    """
    _services: Services
    _model: Model

    def __init__(self, services: Services, model: Model) -> None:
        self._services = services
        self._services.ev_manager.register_listener(self)
        self._model = model

    def notify(self, event: Event) -> None:
        pass

    def destroy(self) -> None:
        pass
