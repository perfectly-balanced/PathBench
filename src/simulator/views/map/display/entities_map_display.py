from typing import Optional, Dict, TYPE_CHECKING

from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.entities.extended_wall import ExtendedWall
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.entities.obstacle import Obstacle
from algorithms.configuration.entities.trace import Trace
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map.display.map_display import MapDisplay
from simulator.views.map.data.voxel_map import VoxelMap

from structures import DynamicColour, Colour, Point

if TYPE_CHECKING:
    from simulator.views.map.map_view import MapView


class EntitiesMapDisplay(MapDisplay):
    __cube_colours: Dict[Point, Colour]
    __agent_colour: DynamicColour
    __trace_colour: DynamicColour
    __goal_colour: DynamicColour

    def __init__(self, services: Services, z_index=100, custom_map: Map = None) -> None:
        super().__init__(services, z_index=z_index, custom_map=custom_map)

        self.__agent_colour = self._services.state.views.effective_view.colours[VoxelMap.AGENT]
        self.__trace_colour = self._services.state.views.effective_view.colours[VoxelMap.TRACE]
        self.__goal_colour = self._services.state.views.effective_view.colours[VoxelMap.GOAL]

        self.__cube_colours = {}

        self.updates_cubes = True

    def render(self, *discarded) -> None:
        rv = self.get_renderer_view()

        for p in self.__cube_colours:
            rv.cube_requires_update(p)
        self.__cube_colours.clear()

        p3 = rv.to_point3

        self.__cube_colours[p3(self._map.agent)] = self.__agent_colour()
        self.__cube_colours[p3(self._map.goal)] = self.__goal_colour()

        tc = self.__trace_colour()
        for trace_point in self._map.trace:
            self.__cube_colours[p3(trace_point)] = tc
        if len(self._map.trace) >= 1:
            self.__cube_colours[p3(Entity(self._map.trace[0].position, self._map.agent.radius))] = self.__agent_colour()

        for p in self.__cube_colours:
            rv.cube_requires_update(p)

    def update_cube(self, p: Point) -> None:
        if p in self.__cube_colours:
            self.get_renderer_view().colour_cube(self.__cube_colours[p])
    
    def request_update_all_cubes(self) -> None:
        rv = self.get_renderer_view()
        for p in self.__cube_colours:
            rv.cube_requires_update(p)
