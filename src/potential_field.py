from typing import List, Tuple
import pygame
from typing import Set, List, Tuple, Optional, Dict

import numpy as np

from algorithms.configuration.configuration import Configuration
from algorithms.algorithm import Algorithm
from algorithms.basic_testing import BasicTesting
from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.maps.map import Map
from algorithms.configuration.entities.obstacle import Obstacle
from simulator.services.services import Services
from simulator.views.map.display.gradient_map_display import GradientMapDisplay
from simulator.views.map.display.map_display import MapDisplay
from simulator.views.map.display.numbers_map_display import NumbersMapDisplay
from structures import Point


class PotentialField(Algorithm):
    step_grid: List[List[int]]
    STEP_GRID_MIN_COLOR = np.array([255, 255, 255])
    STEP_GRID_MAX_COLOR = np.array([0, 0, 255])
    PQ_COLOR_HIGH: np.ndarray = (0, 0, 255)

    PQ_COLOR_LOW: np.ndarray = np.array(pygame.Color("#ADD8E6")[:3])  # (0, 0, 255)
    VISITED_COLOR: np.ndarray = np.array(pygame.Color("#777a7f")[:3])
    
    KP = 5.0  # attractive potential gain
    ETA = 100.0  # repulsive potential gain
    AREA_WIDTH = 30.0  # potential area width [m]
    grid_size = 1    #potential grid size [m]

    def __init__(self, services: Services, testing: BasicTesting = None):
        super().__init__(services, testing)
        self.step_grid = []


    def set_display_info(self) -> List[MapDisplay]:
        """
        Read super description
        """
        return super().set_display_info() + [
             SolidColourMapDisplay(self._services, self.mem.visited, self.VISITED_COLOR, z_index=50),
             # SolidColourMapDisplay(self._services, set(map(lambda el: el[1], self.mem.priority_queue)), (0, 0, 255)),
             GradientMapDisplay(self._services, pts=self.mem.priority_queue,
                                min_color=self.PQ_COLOR_LOW, max_color=self.PQ_COLOR_HIGH, z_index=49, inverted=True),
         ]

    def set_display_info(self) -> List[MapDisplay]:
        """
        Read super description
        """
        display_info: List[MapDisplay] = super().set_display_info() + [
            GradientMapDisplay(self._services, self.step_grid,
                                               min_color=Wavefront.STEP_GRID_MIN_COLOR,
                                               max_color=Wavefront.STEP_GRID_MAX_COLOR),
        
            #NumbersMapDisplay(self._services, copy.deepcopy(self.step_grid))
        ]
        return display_info
    
    # for GradientMapDisplay class  
    # __init__(self, services: Services, grid: List[List[Union[int, float]]] = None,
    #             pts: List[Tuple[Union[int, float], Point]] = None,
    #             min_color: np.ndarray = np.array([150., 150., 0.]),
    #             max_color=np.array([150., 0., 0.]), z_index=50, inverted: bool = False, custom_map: Map = None) -> None:



    def calc_potential_field(self,grid: Map):
        minx = min([grid.obstacles.position.x for point in grid.obstacles.position]) - self.AREA_WIDTH / 2.0
        miny = min([grid.obstacles.position.y for point in grid.obstacles.position]) - self.AREA_WIDTH / 2.0
        maxx = max([grid.obstacles.position.x for point in grid.obstacles.position]) + self.AREA_WIDTH / 2.0
        maxy = max([grid.obstacles.position.y for point in grid.obstacles.position]) + self.AREA_WIDTH / 2.0
        #xw = int(round((maxx - minx) / self.grid_size))
        #yw = int(round((maxy - miny) / self.grid_size))
        xw = grid.size.width
        yw = grid.size.height

        # calc each potential
        pmap = [[0.0 for i in range(yw)] for i in range(xw)]

        for ix in range(xw):
            x = ix * self.grid_size + minx

            for iy in range(yw):
                y = iy * self.grid_size + miny
                ug = self.calc_attractive_potential(x, y, (grid.goal.position.x), (grid.goal.position.y))
                uo = self.calc_repulsive_potential(x=x, y=y, ox=([grid.obstacles.position.x for x in grid.obstacles.position]), 
                oy=([grid.obstacles.position.y for y in grid.obstacles.position]), rr=(grid.agent.radius))
                uf = ug + uo    
                pmap[ix][iy] = uf

                if uo!=0:
                    print("x=ix * reso + minx=",x,"ix=",ix)
                    print("y=iy * reso + miny=",y,"iy=",iy)
                    print("ug=",ug)
                    print("uo=",uo)
                    print("uf=",uf)    
                    print("gx,gy=",gx,gy)
                    print("______")
                    print(np.hypot(x - gx, y - gy))                
        
        #print("pmap=",pmap)
        print("len(pmap)=", len(pmap[0])*len(pmap))

        print ("xw=",xw,"yw=",yw)

        print ("minx=",minx,"miny=",miny)
        print ("maxx=",maxx,"maxy=",maxy)
        print("grid_size=",self.grid_size)

        return pmap, minx, miny


    def calc_attractive_potential(self,x, y, gx, gy):
        return 0.5 * KP * np.hypot(x - gx, y - gy)


    def calc_repulsive_potential(self,x, y, ox, oy, rr):
        # search nearest obstacle
        minid = -1
        dmin = float("inf")
        for i, _ in enumerate(ox):
            d = np.hypot(x - ox[i], y - oy[i])
            if dmin >= d:
                dmin = d
                minid = i

        # calc repulsive potential
        dq = np.hypot(x - ox[minid], y - oy[minid])

        if dq <= rr:
            if dq <= 0.1:
                dq = 0.1

            return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
        else:
            return 0.0

    def potential_field_planning(self,grid):

        # calc potential field
        pmap, minx, miny = calc_potential_field(grid)

        # search path
        d = np.hypot(grid.agent.position.x - grid.goal.position.x, grid.agent.position.y - grid.goal.position.x)
        ix = round((grid.agent.position.x - minx) / self.grid_size)    #index starting x position
        iy = round((grid.agent.position.y - miny) / self.grid_size) 
        gix = round((grid.goal.position.x - minx) / self.grid_size)    #index goal x position
        giy = round((grid.goal.position.y - miny) / self.grid_size)
        

        if show_animation:
            draw_heatmap(pmap)
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(ix, iy, "*k")
            plt.plot(gix, giy, "*m")

        rx, ry = [grid.agent.position.x], [grid.agent.position.y]
        motion = grid.EIGHT_POINTS_MOVE_VECTOR
        while d >= self.grid_size:
            minp = float("inf")
            minix, miniy = -1, -1
            for i, _ in enumerate(motion):
                inx = int(ix + motion[i][0])
                iny = int(iy + motion[i][1])
                if inx >= len(pmap) or iny >= len(pmap[0]):
                    p = float("inf")  # outside area
                else:
                    p = pmap[inx][iny]
                if minp > p:
                    minp = p
                    minix = inx
                    miniy = iny
            ix = minix
            iy = miniy
            xp = ix * self.grid_size + minx
            yp = iy * self.grid_size + miny
            d = np.hypot(grid.goal.position.x - xp, grid.goal.position.y - yp)
            rx.append(xp)
            ry.append(yp)
            point=Point(ix,iy)

            # if show_animation:
            #     plt.plot(ix, iy, ".r")
            #     plt.pause(0.01)
            self.move_agent(point)
            self.key_frame()

        print("Goal!!")

        return rx, ry


    # noinspection PyUnusedLocal
    def _find_path_internal(self) -> None:
        """
        Read super description
        The internal implementation of :ref:`find_path`
        """

        #._get_grid() is in Algorithm class and gets the map
        grid: Map = self._get_grid()
        #agent and goal are represented by a point(x,y) and radius
        agent: Agent = grid.agent
        goal: Goal = grid.goal
        obstacles: Obstacle =grid.obstacles
        #put position of goal in tuple with 2 in a list called queue
        #queue: List[Tuple[Point, int]] = [(goal.position, 2)]
        #make all the cells of the stepgrid 0  
        #self.step_grid = [[0 for _ in range(grid.size.width)] for _ in range(grid.size.height)]
        #make the goal cell 1
        #self.step_grid[goal.position.y][goal.position.x] = 1


        # sx =   # start x position [m]
        # sy = 10.0  # start y positon [m]
        # gx = 30.0  # goal x position [m]
        # gy = 30.0  # goal y position [m]
        # grid_size = 0.5  # potential grid size [m]
        # robot_radius = 5.0  # robot radius [m]



        # if show_animation:
        #     plt.grid(True)
        #     plt.axis("equal")

        # path generation
        _, _ = potential_field_planning(grid)

        # if show_animation:
        #     plt.show()


config = Configuration()

serv =Services(config)

pot = PotentialField(serv)
grid=pot._get_grid()
pot.calc_potential_field(grid)