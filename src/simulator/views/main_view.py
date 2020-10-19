from typing import Optional

from simulator.models.model import Model
from simulator.services.debug import DebugLevel
from simulator.services.event_manager.events.event import Event
from simulator.services.services import Services
from simulator.services.timer import Timer
from simulator.services.event_manager.events.quit_event import QuitEvent
from simulator.services.event_manager.events.window_loaded_event import WindowLoadedEvent
from simulator.views.view import View


class MainView(View):
    MVG_AVG_SIZE = 3
    __frame_timer: Timer
    __frame_count: int
    __frame_mvg_average: int
    __fps: int

    """
    Draws the model state onto the screen.
    """

    def __init__(self, services: Services, model: Model, root_view: Optional[View]) -> None:
        super().__init__(services, model, root_view)
        self._services.ev_manager.register_tick_listener(self)
        self.__frame_timer = Timer()
        self.__frame_count = 0
        self.__frame_mvg_average = 0
        self.__fps = 0
        self.__screen = None
        self._initialised = False

    def notify(self, event: Event) -> None:
        """
        Receive events posted to the message queue.
        """

        super().notify(event)
        if isinstance(event, QuitEvent):
            self._initialised = False
            self._services.window.quit()

    def initialise(self) -> None:
        """
        Set up the self.services.window graphical display and loads graphical resources.
        """

        self._services.window.init("PathBench", size=self._services.settings.simulator_window_size)
        self._services.ev_manager.post(WindowLoadedEvent())
        self.__frame_timer = Timer()

    def tick(self) -> None:
        if not self._initialised:
            return
        self.update()

    def update(self) -> None:
        if self.__frame_timer.stop() >= 1:
            if self.__frame_mvg_average == 0:
                self.__fps = self.__frame_count
            else:
                self.__fps += (self.__frame_count - self.__fps) / self.__frame_mvg_average
            self.__frame_mvg_average = min(self.__frame_mvg_average + 1, self.MVG_AVG_SIZE)
            self.__frame_count = 0
            self.__frame_timer = Timer()
            self._services.debug.write("FPS: " + str(self.__fps), DebugLevel.MEDIUM)

        for child in self._children:
            child.update()

        # update window
        self._services.window.update()
        self.__frame_count += 1
