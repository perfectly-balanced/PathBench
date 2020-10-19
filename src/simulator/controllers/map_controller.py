from simulator.controllers.controller import Controller
from simulator.models.model import Model
from simulator.services.services import Services
from simulator.services.event_manager.events.window_loaded_event import WindowLoadedEvent

class MapController(Controller):
    def notify(self, event: Event) -> None:
        """
        Receive events posted to the message queue.
        """

        super().notify(event)
        if isinstance(event, WindowLoadedEvent):
            self._services.window.accept("escape", lambda: self._services.ev_manager.post(QuitEvent()))
            self._services.window.accept("arrow_up", lambda: self._model.move_up())
            self._services.window.accept("arrow_down", lambda: self._model.move_down())
            self._services.window.accept("arrow_left", lambda: self._model.move_left())
            self._services.window.accept("arrow_right", lambda: self._model.move_right())
            self._services.window.accept("c", lambda: self._model.compute_trace())
            self._services.window.accept("m", lambda: self._model.toggle_convert_map())
            self._services.window.accept("s", lambda: self._model.stop_algorithm())
            self._services.window.accept("r", lambda: self._model.resume_algorithm())
            self._services.window.accept("p", lambda: self._services.ev_manager.post(TakeScreenshotEvent())
