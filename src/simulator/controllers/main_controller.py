from simulator.controllers.controller import Controller
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.quit_event import QuitEvent

from direct.showbase.DirectObject import DirectObject

class MainController(Controller, DirectObject):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        self.accept("escape", lambda: self._services.ev_manager.post(QuitEvent()))
        self.accept("p", lambda: self._services.resources.screenshots_dir.append(lambda fn: self._services.graphics.window.win.save_screenshot(fn)))

    def destroy(self) -> None:
        self.ignore_all()
        self._services.ev_manager.unregister_listener(self)
