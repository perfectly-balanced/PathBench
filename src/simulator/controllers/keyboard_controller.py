from simulator.controllers.controller import Controller
from simulator.models.model import Model
from simulator.services.debug import DebugLevel
from simulator.services.event_manager.events.keyboard_event import KeyboardEvent
from simulator.services.event_manager.events.mouse_event import MouseEvent
from simulator.services.event_manager.events.quit_event import QuitEvent
from simulator.services.services import Services


class KeyboardController(Controller):
    """
    Handles keyboard input
    """
        
    def notify(self, event: Event) -> None:
        """
        Receive events posted to the message queue.
        """

        super().notify(event)
        if isinstance(event, WindowLoadedEvent):
            self._services.window.accept("escape", lambda: self._services.ev_manager.post(QuitEvent()))
