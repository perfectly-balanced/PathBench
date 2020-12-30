import copy
from typing import Union, Tuple

import torch
import numpy as np
import pandas.tests.extension

from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.maps.map import Map
from algorithms.lstm.map_processing import MapProcessing
from simulator.services.services import Services
from simulator.views.map.display.map_display import MapDisplay
from structures import Point, Colour, RED


class OnlineLSTMMapDisplay(MapDisplay):
    def __init__(self, services: Services, custom_map: Map = None):
        super().__init__(services, z_index=250, custom_map=custom_map)

    def render(self, *discarded) -> None:
        mp = copy.deepcopy(self._map)

        features = MapProcessing.extract_features(mp, [
            "distance_to_goal_normalized",
            "raycast_8_normalized",
            "direction_to_goal_normalized",
            "agent_goal_angle"]
        )

        for (ray, dir) in zip(features["raycast_8_normalized"], mp.ALL_POINTS_MOVE_VECTOR):
            ray = 50.0 * ray - mp.get_movement_cost(Point(0, 0), dir)
            dir = dir.to_tensor() / torch.norm(dir.to_tensor())
            vec = Point.from_tensor(mp.agent.position.to_tensor() + ray * dir)
            self.__render_line(mp.agent.position, vec, RED)

        self.__render_line(mp.agent.position, mp.goal.position, Colour(1, 0, 1))
        self.__render_arc(mp.agent.position, mp.goal.position, Colour(1, 0, 1))

    def __render_arc(self, p1: Point, p2: Point, colour: Colour):
        center = p1
        radius = 1
        
        dir = p2.to_tensor() - p1.to_tensor()
        angle = torch.atan2(dir[1], dir[0])
        angle %= 2 * np.pi

        self.get_renderer_view().draw_arc(center, angle, radius=radius, colour=colour)
        
    def __render_line(self, p1: Point, p2: Point, colour: Colour):
        self.get_renderer_view().draw_line(colour, p1, p2)
