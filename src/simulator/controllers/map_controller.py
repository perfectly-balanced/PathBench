from typing import Optional

import pygame

from simulator.controllers.controller import Controller
from simulator.models.main import Main
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.keyboard_event import KeyboardEvent
from simulator.services.event_manager.events.take_screenshot_event import TakeScreenshotEvent


class MapController(Controller):
    def notify(self, event: Event) -> None:
        model: Optional[Main] = self._model
        
        if isinstance(event, KeyboardEvent):
            if event.key == pygame.K_UP:
                model.move_up()
            if event.key == pygame.K_DOWN:
                model.move_down()
            if event.key == pygame.K_LEFT:
                model.move_left()
            if event.key == pygame.K_RIGHT:
                model.move_right()
            if event.key == pygame.K_c:
                model.compute_trace()
            if event.key == pygame.K_m:
                model.toggle_convert_map()
            if event.key == pygame.K_s:
                model.stop_algorithm()
            if event.key == pygame.K_r:
                model.resume_algorithm()
            if event.key == pygame.K_p:
                self._services.ev_manager.post(TakeScreenshotEvent())
