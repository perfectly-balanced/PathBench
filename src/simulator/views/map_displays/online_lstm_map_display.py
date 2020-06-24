import copy
from typing import Union, Tuple

import torch
import pygame
#from pandas.tests.extension.numpy_.test_numpy_nested import np
import numpy as np
import pandas.tests.extension

from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.maps.map import Map
from algorithms.lstm.map_processing import MapProcessing
from simulator.services.services import Services
from simulator.views.map_displays.map_display import MapDisplay
from structures import Point


class OnlineLSTMMapDisplay(MapDisplay):
    def __init__(self, services: Services, custom_map: Map = None):
        super().__init__(services, z_index=250, custom_map=custom_map)

    def render(self, screen: pygame.Surface) -> bool:
        if not super().render(screen):
            return False

        mp = copy.deepcopy(self._map)

        # self.__render_line(mp.agent.position, mp.goal.position, screen, (255, 0, 255))

        features = MapProcessing.extract_features(mp, [
            "distance_to_goal_normalized",
            "raycast_8_normalized",
            "direction_to_goal_normalized",
            "agent_goal_angle"]
        )

        for (ray, dir) in zip(features["raycast_8_normalized"], mp.EIGHT_POINTS_MOVE_VECTOR):
            ray = 50.0 * ray - mp.get_movement_cost(Point(0, 0), dir)
            dir = dir.to_tensor() / torch.norm(dir.to_tensor())
            vec = Point.from_tensor(mp.agent.position.to_tensor() + ray * dir)
            self.__render_line(mp.agent.position, vec, screen, (255, 0, 0))

        self.__render_line(mp.agent.position, mp.goal.position, screen, (255, 0, 255))
        self.__render_arc(mp.agent.position, mp.goal.position, screen, (255, 0, 255))

        return True

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

    def __render_arc(self, p1: Point, p2: Point, screen: pygame.Surface,
                      color: Union[pygame.Color, Tuple[int, int, int]]):
        point = self.get_renderer_view().get_screen_rect(p1).center
        size = 100
        area = pygame.Rect(point[0] - size / 2, point[1] - size / 2, size, size)
        dir = p2.to_tensor() - p1.to_tensor()
        angle = torch.atan2(-dir[1], dir[0]) # pygame
        angle %= 2 * np.pi
        # print(p1, angle)
        self.services.render_engine.draw_arc(screen, color, area, 0, angle, 2)

    def __render_vector(self, origin: Point, magnitude: float, angle: float, screen: pygame.Surface,
                        color: Union[pygame.Color, Tuple[int, int, int]]):
        rot_mat = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)],
        ])
        unit_vector = np.array([1, 0]) * magnitude
        vec = origin + np.matmul(rot_mat, unit_vector)
        self.__render_line(origin, Point(*vec), screen, color)

    def __render_line(self, p1: Point, p2: Point, screen: pygame.Surface,
                      color: Union[pygame.Color, Tuple[int, int, int]]):
        first_point = self.get_renderer_view().get_screen_rect(p1).center
        second_point = self.get_renderer_view().get_screen_rect(p2).center
        self.services.render_engine.draw_line(screen, color, first_point, second_point, 2)

    def __lt__(self, other):
        return super().__lt__(other)
