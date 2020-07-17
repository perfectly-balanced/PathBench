import copy
import pygame
from typing import Set, Union, Tuple, List

from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.maps.map import Map
from simulator.services.services import Services
from simulator.views.map_displays.map_display import MapDisplay
from structures import Point
import numpy as np


class SolidColorMapDisplaypot(MapDisplay):
    radius: int
    points: Union[Set[Point], List[Entity]]
    color: pygame.Color


    def __init__(self, services: Services, points,
                 color: Union[pygame.Color, Tuple[int, int, int]] = None, z_index=50, radius=0, custom_map: Map = None, pmaplst=None) -> None:
        super().__init__(services, z_index=z_index, custom_map=custom_map)
        self.points = points
        self.radius = radius
        self.color1 = np.array([0, 0, 153])
        self.color2 = np.array([0, 0, 204])
        self.color3 = np.array([0, 0, 255])
        self.color4 = np.array([51, 51, 255])
        self.color5 = np.array([102, 102, 255])
        self.color6 = np.array([153, 153, 255])
        self.large = max(pmaplst)
        self.small = min(pmaplst)
        self.range=(self.large-self.small)/6
        


    def render(self, screen: pygame.Surface) -> bool:
        if not super().render(screen):
            return False

        points: Union[Set[Point], List[Entity]] = copy.deepcopy(self.points)
        if points is None:
            return False




        def f(pt):
            if isinstance(pt, Entity):
                pt = pt.position
            return pt

        points = set(map(f, points))        

        #print ("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&self.large,self.small,self.range",self.large,self.small,self.range)
        for point in points:
            #print ("point[1],self.small+(self.range*1)",point[1],self.small+(self.range*1))
            if point[1] < self.small+(self.range*1):
                self._root_view.render_pos(screen, Entity(point[0], self.radius), self.color6)
                #print("************************************check color 6")   
            elif point[1] < self.small+(self.range*2):
                self._root_view.render_pos(screen, Entity(point[0], self.radius), self.color5)
                #print("************************************check color 5")   
            elif point[1] < self.small+(self.range*3):
                self._root_view.render_pos(screen, Entity(point[0], self.radius), self.color4)
                #print("************************************check color 4")   
            elif point[1] < self.small+(self.range*4):
                self._root_view.render_pos(screen, Entity(point[0], self.radius), self.color3)
                #print("************************************check color 3")   
            elif point[1] < self.small+(self.range*5):
                self._root_view.render_pos(screen, Entity(point[0], self.radius), self.color2)
                #print("************************************check color 2")   
            elif point[1] < self.small+(self.range*6):
                self._root_view.render_pos(screen, Entity(point[0], self.radius), self.color1)    
                #print("************************************check color 1")           

 
        return True

    def __lt__(self, other: 'SolidColorMapDisplaypot') -> bool:
        return tuple(self.color) < tuple(other.color)
