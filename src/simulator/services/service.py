from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from simulator.services.services import Services


class Service:
    _services: 'Services'

    def __init__(self, _services: 'Services') -> None:
        self._services = _services
