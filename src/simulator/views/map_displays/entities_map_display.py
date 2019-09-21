from typing import Optional, TYPE_CHECKING

import pygame

from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.entities.extended_wall import ExtendedWall
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.entities.obstacle import Obstacle
from algorithms.configuration.entities.trace import Trace
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map_displays.map_display import MapDisplay

if TYPE_CHECKING:
    from simulator.views.map_view import MapView


class EntitiesMapDisplay(MapDisplay):
    animation_step: int

    def __init__(self, services: Services, z_index=100, custom_map: Map = None) -> None:
        super().__init__(services, z_index=z_index, custom_map=custom_map)
        self.animation_step = 0

    def render(self, screen: pygame.Surface) -> bool:
        if not super().render(screen):
            return False

        self.render_entities(screen)
        return True

    def render_entities(self, screen: pygame.Surface) -> None:
        self.render_obstacles(screen)
        self.render_trace(screen)
        self.render_agent(screen, self._map.agent)
        self.render_goal(screen, self._map.goal)

    def render_trace(self, screen: pygame.Surface) -> None:
        for trace_point in self._map.trace:
            self.render_trace_point(screen, trace_point)
        self.render_initial_position(screen, self._map)

    def render_obstacles(self, screen: pygame.Surface) -> None:
        for obstacle in self._map.obstacles:
            self.render_obstacle(screen, obstacle)

    def render_obstacle(self, screen: pygame.Surface, obstacle: Obstacle) -> None:
        color = (200, 200, 200) if isinstance(obstacle, ExtendedWall) else (0, 0, 0)
        self.get_renderer_view().render_pos(screen, obstacle, color)

    def render_agent(self, screen: pygame.Surface, agent: Agent) -> None:
        self.get_renderer_view().render_pos(screen, agent, (255, 0, 0))

    def render_goal(self, screen: pygame.Surface, goal: Goal) -> None:
        self.get_renderer_view().render_pos(screen, goal, (0, 100, 0))

    def render_trace_point(self, screen: pygame.Surface, trace_point: Trace) -> None:
        self.get_renderer_view().render_pos(screen, trace_point, (0, 200, 0))

    def render_initial_position(self, screen: pygame.Surface, grid: Map) -> None:
        if len(grid.trace) >= 1:
            self.get_renderer_view().render_pos(screen, Entity(grid.trace[0].position, grid.agent.radius), (150, 0, 0))

    def __lt__(self, other):
        return super().__lt__(other)
