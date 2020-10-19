from simulator.controllers.controller import Controller
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.window_loaded_event import WindowLoadedEvent

class MainController(Controller):
    def notify(self, event: Event) -> None:
        """
        Receive events posted to the message queue.
        """

        super().notify(event)
        if isinstance(event, WindowLoadedEvent):
            self._services.window.accept("escape", lambda: self._services.ev_manager.post(QuitEvent()))
