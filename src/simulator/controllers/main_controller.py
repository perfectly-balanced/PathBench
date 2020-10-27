from simulator.controllers.controller import Controller
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.graphics_loaded_event import GraphicsLoadedEvent

class MainController(Controller):
    def notify(self, event: Event) -> None:
        """
        Receive events posted to the message queue.
        """

        super().notify(event)
        if isinstance(event, GraphicsLoadedEvent):
            window = self._services.graphics.window

            window.accept("escape", lambda: self._services.ev_manager.post(QuitEvent()))

            # disables the default camera behaviour
            window.disable_mouse()
