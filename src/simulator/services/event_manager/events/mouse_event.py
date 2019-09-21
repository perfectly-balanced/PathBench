import pygame

from simulator.services.event_manager.events.event import Event
from structures import Point


class MouseEvent(Event):
    event: pygame.event

    def __init__(self, event: pygame.event) -> None:
        super().__init__()
        self._name = "Mouse event"
        self.event = event

    @property
    def type(self) -> str:
        return "type"

    @type.getter
    def type(self) -> int:
        return self.event.type

    @property
    def pos(self) -> str:
        return "pos"

    @pos.getter
    def pos(self) -> Point:
        return self.event.pos
