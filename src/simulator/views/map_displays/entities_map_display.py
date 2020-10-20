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
from simulator.views.map.voxel_map import VoxelMap

if TYPE_CHECKING:
    from simulator.views.map.map_view import MapView


class EntitiesMapDisplay(MapDisplay):
    animation_step: int
    __map: VoxelMap

    def __init__(self, map: VoxelMap, services: Services, z_index=100, custom_map: Map = None) -> None:
        super().__init__(services, z_index=z_index, custom_map=custom_map)
        self.__map = map
        self.animation_step = 0

    def render(self) -> bool:
        if not super().render():
            return False

        self.render_entities()
        return True

    def render_entities(self) -> None:
        self.render_trace()
        self.render_agent(self._map.agent)
        self.render_goal(self._map.goal)

    def render_trace(self) -> None:
        for trace_point in self._map.trace:
            self.render_trace_point(trace_point)
        self.render_initial_position(self._map)

    def render_agent(self, agent: Agent) -> None:
        self.get_renderer_view().render_pos(agent, self.__map.agent_colour)

    def render_goal(self, goal: Goal) -> None:
        self.get_renderer_view().render_pos(goal, self.__map.goal_colour)

    def render_trace_point(self, trace_point: Trace) -> None:
        self.get_renderer_view().render_pos(trace_point, self.__map.trace_colour)

    def render_initial_position(self, grid: Map) -> None:
        if len(grid.trace) >= 1:
            self.get_renderer_view().render_pos(Entity(grid.trace[0].position, grid.agent.radius), self.__map.agent_colour)

    def __lt__(self, other):
        return super().__lt__(other)
