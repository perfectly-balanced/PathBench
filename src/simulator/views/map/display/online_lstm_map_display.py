import copy
from typing import Union, Tuple, Optional

import torch
import numpy as np
import pandas.tests.extension

from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.maps.map import Map
from algorithms.lstm.map_processing import MapProcessing
from simulator.services.services import Services
from simulator.views.map.display.map_display import MapDisplay
from structures import Point, Colour, DynamicColour, RED


class OnlineLSTMMapDisplay(MapDisplay):
    ray_colour: DynamicColour
    bearing_colour: DynamicColour

    def __init__(self, services: Services, ray_colour: Optional[DynamicColour] = None, bearing_colour: Optional[DynamicColour] = None, custom_map: Map = None):
        super().__init__(services, z_index=250, custom_map=custom_map)

        self.ray_colour = self._services.state.views.add_colour("ray", RED) if ray_colour is None else ray_colour
        self.bearing_colour = self._services.state.views.add_colour("bearing", Colour(1, 0, 1)) if bearing_colour is None else bearing_colour

    def render(self, *discarded) -> None:
        self._services.graphics.renderer.push_line_thickness(50)

        features = MapProcessing.extract_features(self._map, [
            "distance_to_goal_normalized",
            "raycast_8_normalized",
            "direction_to_goal_normalized",
            "agent_goal_angle"]
        )

        rc = self.ray_colour()
        for (ray, dir) in zip(features["raycast_8_normalized"], self._map.ALL_POINTS_MOVE_VECTOR):
            ray = 50.0 * ray - self._map.get_movement_cost(Point(0, 0), dir)
            dir = dir.to_tensor() / torch.norm(dir.to_tensor())
            vec = Point.from_tensor(self._map.agent.position.to_tensor() + ray * dir)
            self.__render_line(self._map.agent.position, vec, rc)

        bc = self.bearing_colour()
        self.__render_line(self._map.agent.position, self._map.goal.position, bc)
        self.__render_arc(self._map.agent.position, self._map.goal.position, bc)

        self._services.graphics.renderer.pop_line_thickness()

    def __render_arc(self, p1: Point, p2: Point, colour: Colour):
        center = p1
        radius = 1

        dir = p2.to_tensor() - p1.to_tensor()
        angle = torch.atan2(dir[1], dir[0])
        angle %= 2 * np.pi

        self.get_renderer_view().draw_arc(center, angle, radius=radius, colour=colour)

    def __render_line(self, p1: Point, p2: Point, colour: Colour):
        self.get_renderer_view().draw_line(colour, p1, p2)
