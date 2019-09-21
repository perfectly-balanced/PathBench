import pygame

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

    def __init__(self, services: Services, model: Model) -> None:
        super().__init__(services, model)
        self._services.ev_manager.register_tick_listener(self)

    def tick(self) -> None:
        if self._services.render_engine.is_display_init() == 0:
            return

        # Called for each game tick. We check our keyboard presses here.
        for event in self._services.render_engine.get_events():
            # handle window manager closing our window
            if event.type == pygame.QUIT:
                self._services.ev_manager.post(QuitEvent())
            # handle key down events
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self._services.ev_manager.post(QuitEvent())
                else:
                    # post any other keys to the message queue for everyone else to see
                    ev = KeyboardEvent(event.key)
                    self._services.ev_manager.post(ev)
                    self._services.debug.write(ev, DebugLevel.MEDIUM)
            if event.type == pygame.MOUSEMOTION or \
                    event.type == pygame.MOUSEBUTTONDOWN or \
                    event.type == pygame.MOUSEBUTTONUP:
                self._services.ev_manager.post(MouseEvent(event))
