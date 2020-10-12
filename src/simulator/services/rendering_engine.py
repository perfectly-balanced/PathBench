from typing import List, Any, Tuple, Union

import pygame

from simulator.services.service import Service
from structures import Size, Point


class RenderingEngine(Service):
    @staticmethod
    def set_display(window_size: Size) -> pygame.Surface:
        return pygame.display.set_mode(window_size)

    @staticmethod
    def draw_rect(screen: pygame.Surface, color: Union[pygame.Color, Tuple[int, int, int]], rect: pygame.Rect) -> None:
        try:
            pygame.draw.rect(screen, color, rect)
        except TypeError:
            print(color)
            raise

    @staticmethod
    def draw_circle(screen: pygame.Surface, color: Union[pygame.Color, Tuple[int, int, int]], pos: Point, radius: int,
                    width: int = 0) -> None:
        pygame.draw.circle(screen, color, pos, radius, width)

    @staticmethod
    def draw_line(screen: pygame.Surface, color: Union[pygame.Color, Tuple[int, int, int]], start_pos: Point,
                  end_pos: Point, width: int = 1):
        pygame.draw.line(screen, color, start_pos, end_pos, width)

    @staticmethod
    def draw_arc(screen: pygame.Surface, color: Union[pygame.Color, Tuple[int, int, int]], area: pygame.Rect,
                 start_angle: float, stop_angle: float, width: int = 1):
        pygame.draw.arc(screen, color, area, start_angle, stop_angle, width)

    @staticmethod
    def is_display_init() -> bool:
        return pygame.display.get_init()

    @staticmethod
    def get_events() -> List[Any]:
        return pygame.event.get()

    @staticmethod
    def quit() -> None:
        pygame.quit()

    # noinspection PyArgumentList
    @staticmethod
    def init(title: str) -> None:
        pygame.init()
        pygame.key.set_repeat(1, 100)
        pygame.display.set_caption(title)

    @staticmethod
    def render() -> None:
        pygame.display.flip()

    @staticmethod
    def is_mouse_focused() -> bool:
        if not RenderingEngine.is_display_init():
            return False
        return pygame.mouse.get_focused()
