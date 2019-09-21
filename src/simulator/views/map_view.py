from time import sleep

import pygame
from heapq import heappush, heappop
from typing import List, Any, Tuple, Optional, Union

from algorithms.configuration.entities.entity import Entity
from simulator.models.model import Model
from simulator.services.debug import DebugLevel
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.mouse_event import MouseEvent
from simulator.services.event_manager.events.key_frame_event import KeyFrameEvent
from simulator.services.event_manager.events.take_screenshot_event import TakeScreenshotEvent
from simulator.services.services import Services
from simulator.views.map_displays.empty_map_display import EmptyMapDisplay
from simulator.views.map_displays.entities_map_display import EntitiesMapDisplay
from simulator.views.map_displays.map_display import MapDisplay
from simulator.views.map_displays.numbers_map_display import NumbersMapDisplay
from simulator.views.map_displays.online_lstm_map_display import OnlineLSTMMapDisplay
from simulator.views.view import View
from structures import Point


class MapView(View):
    MIN_AGENT_RADIUS: int = 5
    __cell_size: int
    __padding: Point
    __cell_gap: int
    __prev_seen: Point
    __surface: pygame.Surface
    __render_displays: List[Any]

    def __init__(self, services: Services, model: Model, root_view: Optional[View]) -> None:
        super().__init__(services, model, root_view)
        self.__cell_size = 1
        self.__cell_gap = 1
        self.__padding = Point(0, 0)
        self.__prev_seen = Point(-1, -1)
        self.__surface = pygame.Surface(self._services.settings.simulator_window_size)
        self.__render_displays = []
        if self._services.settings.simulator_grid_display:
            self.__cell_size = min((self._services.settings.simulator_window_size.width + self.__cell_gap) //
                                   (self._services.algorithm.map.size.width + self.__cell_gap),
                                   (self._services.settings.simulator_window_size.height + self.__cell_gap) //
                                   (self._services.algorithm.map.size.height + self.__cell_gap))

            self.__padding = Point((self._services.settings.simulator_window_size.width + self.__cell_gap -
                                    (self.__cell_size + self.__cell_gap) * self._services.algorithm.map.size.width) // 2,
                                   (self._services.settings.simulator_window_size.height + self.__cell_gap -
                                  (self.__cell_size + self.__cell_gap) * self._services.algorithm.map.size.height) // 2)
        self.set_render_displays()

    def notify(self, event: Event) -> None:
        super().notify(event)
        if isinstance(event, MouseEvent):
            grid_pos: Point = self.get_grid_pos(Point(event.pos[0], event.pos[1]))
            if event.type == pygame.MOUSEMOTION:
                if self._services.render_engine.is_mouse_focused() and self.__prev_seen != grid_pos:
                    self.__prev_seen = grid_pos
                    self._services.debug.write(grid_pos, DebugLevel.MEDIUM)
            if event.type == pygame.MOUSEBUTTONDOWN and event.event.button == 1:
                self._services.debug.write("Moved agent to: " + str(grid_pos), DebugLevel.MEDIUM)
                self._model.move(grid_pos)
            if event.type == pygame.MOUSEBUTTONDOWN and event.event.button == 3:
                self._services.debug.write("Moved goal to: " + str(grid_pos), DebugLevel.MEDIUM)
                self._model.move_goal(grid_pos)
        elif isinstance(event, KeyFrameEvent):
            self.set_render_displays()
        elif isinstance(event, TakeScreenshotEvent):
            self.take_screenshot()

    def set_render_displays(self) -> None:
        self.__render_displays = []
        displays: List[MapDisplay] = [EmptyMapDisplay(self._services),
                                      EntitiesMapDisplay(self._services)]
        # drop map displays if not compatible with display format
        displays += list(filter(lambda d: self._services.settings.simulator_grid_display or not isinstance(d, NumbersMapDisplay),
                                self._services.algorithm.instance.get_display_info()))
        for display in displays:
            display._model = self._model
            self.add_child(display)
            heappush(self.__render_displays, (display.z_index, display))

    # noinspection PyUnusedLocal
    def render(self, screen: pygame.Surface) -> None:
        while len(self.__render_displays) > 0:
            display: MapDisplay
            _, display = heappop(self.__render_displays)
            display.render(self.__surface)
        screen.blit(self.__surface, self.__surface.get_rect())

    def render_pos(self, screen: pygame.Surface, entity: Entity,
                   color: Union[pygame.Color, Tuple[int, int, int]]) -> None:
        if self._services.settings.simulator_grid_display:
            self._services.render_engine.draw_rect(screen, color, self.get_screen_rect(entity.position))
        else:
            self._services.render_engine.draw_circle(screen, color, entity.position, entity.radius, 0)

    def get_screen_rect(self, grid_pos: Point) -> pygame.Rect:
        if not self._services.settings.simulator_grid_display:
            return pygame.Rect(grid_pos.x, grid_pos.y, 0, 0)
        else:
            return pygame.Rect(self.__padding.x + grid_pos.x * (self.__cell_size + self.__cell_gap),
                               self.__padding.y + grid_pos.y * (self.__cell_size + self.__cell_gap),
                               self.__cell_size, self.__cell_size)

    def get_grid_pos(self, screen_pos: Point) -> Point:
        if not self._services.settings.simulator_grid_display:
            return screen_pos
        else:
            grid_pos: Point = Point(screen_pos.x - self.__padding.x, screen_pos.y - self.__padding.y)
            grid_pos = Point(grid_pos.x // (self.__cell_size + self.__cell_gap),
                             grid_pos.y // (self.__cell_size + self.__cell_gap))
            return grid_pos

    def get_map_rect(self) -> pygame.Rect:
        if not self._services.settings.simulator_grid_display:
            return pygame.Rect(
                0, 0,
                self._services.algorithm.map.size.width, self._services.algorithm.map.size.height
            )
        else:
            return pygame.Rect(
                self.__padding.x, self.__padding.y,
                (self.__cell_size + self.__cell_gap) * self._services.algorithm.map.size.width -
                self.__cell_gap * int(self._services.algorithm.map.size.width != 0),
                (self.__cell_size + self.__cell_gap) * self._services.algorithm.map.size.height -
                self.__cell_gap * int(self._services.algorithm.map.size.height != 0),
            )

    def take_screenshot(self) -> None:
        map_rect: pygame.Rect = self.get_map_rect()
        surface_size: Tuple[int, int] = (map_rect.width + self.__cell_gap * 2, map_rect.height + self.__cell_gap * 2)
        image: pygame.Surface = pygame.Surface(surface_size)
        image.fill((0, 0, 0))
        image.blit(self.__surface, (self.__cell_gap, self.__cell_gap), area=map_rect)
        self._model.save_screenshot(image)
