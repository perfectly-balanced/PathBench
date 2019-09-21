from typing import Optional

import pygame

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
    __screen: pygame.Surface
    __clock: pygame.time.Clock
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
        self.__clock = None

    def notify(self, event: Event) -> None:
        """
        Receive events posted to the message queue.
        """

        super().notify(event)
        if isinstance(event, QuitEvent):
            # shut down the self.services.render_engine graphics
            self._initialized = False
            self._services.render_engine.quit()

    def initialize(self) -> None:
        """
        Set up the self.services.render_engine graphical display and loads graphical resources.
        """

        self._services.render_engine.init("PathBench")
        self.__clock = pygame.time.Clock()
        self.__screen = self._services.render_engine.set_display(self._services.settings.simulator_window_size)
        self._services.ev_manager.post(WindowLoadedEvent())
        self.__frame_timer = Timer()

    def tick(self) -> None:
        if not self._initialized:
            return
        self.render(self.__screen)
        # limit the redraw speed to 100 frames per second
        self.__clock.tick(100)

    def render(self, screen: pygame.Surface) -> None:
        if screen is None:
            return

        if self.__frame_timer.stop() >= 1:
            if self.__frame_mvg_average == 0:
                self.__fps = self.__frame_count
            else:
                self.__fps += (self.__frame_count - self.__fps) / self.__frame_mvg_average
            self.__frame_mvg_average = min(self.__frame_mvg_average + 1, self.MVG_AVG_SIZE)
            self.__frame_count = 0
            self.__frame_timer = Timer()
            self._services.debug.write("FPS: " + str(self.__fps), DebugLevel.MEDIUM)

        # clear display
        screen.fill((0, 0, 0))

        for child in self._children:
            child.render(screen)

        # flip the display to show whatever we drew
        self._services.render_engine.render()
        self.__frame_count += 1
