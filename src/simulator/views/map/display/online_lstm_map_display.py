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

    def render(self, refresh: bool) -> None:
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

    """
    'distance_to_goal_normalized'(4323596448) = {Tensor}
    tensor(0.4342)
    'raycast_8_normalized'(4323234992) = {Tensor}
    tensor([0.1200, 0.3677, 0.1200, 0.1697, 0.4800, 0.6223, 0.4400, 0.1697])
    'direction_to_goal_normalized'(4323596368) = {Tensor}
    tensor([0.2534, -0.9674])
    'agent_goal_angle'(4323234920) = {Tensor}
    tensor(-1.3146)
    """

    def __render_arc(self, p1: Point, p2: Point, colour: Colour):
        raise NotImplementedError

    def __render_line(self, p1: Point, p2: Point, colour: Colour):
        self.get_renderer_view().draw_line(colour, p1, p2)
