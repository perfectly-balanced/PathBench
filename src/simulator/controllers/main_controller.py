from simulator.controllers.controller import Controller
from simulator.services.event_manager.events.event import Event

class MainController(Controller):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        window = self._services.graphics.window
        window.accept("escape", lambda: self._services.ev_manager.post(QuitEvent()))
