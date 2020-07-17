from typing import List, Tuple
import pygame
from typing import Set, List, Tuple, Optional, Dict

import numpy as np

#from algorithms.configuration.configuration import Configuration
from algorithms.algorithm import Algorithm
from algorithms.basic_testing import BasicTesting
from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.maps.map import Map
from algorithms.configuration.entities.obstacle import Obstacle
from simulator.services.services import Services
from simulator.views.map_displays.gradient_map_display import GradientMapDisplay
from simulator.views.map_displays.map_display import MapDisplay
from simulator.views.map_displays.numbers_map_display import NumbersMapDisplay
from simulator.views.map_displays.solid_color_map_display_pot import SolidColorMapDisplaypot
from structures import Point
import copy


class PotentialField(Algorithm):
    step_grid: List[List[int]]


    # PQ_COLOR_LOW: np.ndarray = np.array(pygame.Color("#ADD8E6")[:3])  # (0, 0, 255)
    # VISITED_COLOR: np.ndarray = np.array(pygame.Color("#777a7f")[:3])
    
    KP = 10.0  # attractive potential gain
    ETA = 50.0  # repulsive potential gain
    AREA_WIDTH = 30.0  # potential area width [m]
    grid_size = 1    #potential grid size [m]
    pmapheat =[]
    pmapnew=[]

    def __init__(self, services: Services, testing: BasicTesting = None):
        super().__init__(services, testing)
        self.step_grid = []



    def set_display_info(self) -> List[MapDisplay]:
        """
        Read super description
        """

        display_info: List[MapDisplay] = super().set_display_info() + [
            # GradientMapDisplay(self._services, self.step_grid,
            #                                    min_color=PotentialField.STEP_GRID_MIN_COLOR,
            #                                    max_color=PotentialField.STEP_GRID_MAX_COLOR),
        
            #NumbersMapDisplay(self._services, copy.deepcopy(self.step_grid)) +
            SolidColorMapDisplaypot(self._services, self.pmapheat, z_index=50, pmaplst=self.pmapnew)
        ]

        # display_info: List[MapDisplay] = super().set_display_info() + [
        #     # GradientMapDisplay(self._services, self.step_grid,
        #     #                                    min_color=PotentialField.STEP_GRID_MIN_COLOR,
        #     #                                    max_color=PotentialField.STEP_GRID_MAX_COLOR),
        
        #SolidColorMapDisplaypot(self._services, self.pmapheat, z_index=50, pmaplst=pmapnew)
        # ]


        return display_info

    # def set_display_info(self) -> List[MapDisplay]:
    #     """
    #     Read super description
    #     """
    #     return super().set_display_info() #+ [
            #  SolidColorMapDisplay(self._services, self.mem.visited, self.VISITED_COLOR, z_index=50),
            #  # SolidColorMapDisplay(self._services, set(map(lambda el: el[1], self.mem.priority_queue)), (0, 0, 255)),
            #  GradientMapDisplay(self._services, pts=self.mem.priority_queue,
            #                     min_color=self.PQ_COLOR_LOW, max_color=self.PQ_COLOR_HIGH, z_index=49, inverted=True),
         #]

    # def set_display_info(self) -> List[MapDisplay]:
    #     """
    #     Read super description
    #     """
    #     display_info: List[MapDisplay] = super().set_display_info() + [
    #         GradientMapDisplay(self._services, self.step_grid,
    #                                            min_color=PotentialField.STEP_GRID_MIN_COLOR,
    #                                            max_color=PotentialField.STEP_GRID_MAX_COLOR),
        
    #         #NumbersMapDisplay(self._services, copy.deepcopy(self.step_grid))
    #     ]
    #     return display_info
    
    # for GradientMapDisplay class  
    # __init__(self, services: Services, grid: List[List[Union[int, float]]] = None,
    #             pts: List[Tuple[Union[int, float], Point]] = None,
    #             min_color: np.ndarray = np.array([150., 150., 0.]),
    #             max_color=np.array([150., 0., 0.]), z_index=50, inverted: bool = False, custom_map: Map = None) -> None:



    def calc_potential_field(self,grid: Map):
        xw = grid.size.width
        yw = grid.size.height

        # calc each potential
        pmap = [[0.0 for i in range(yw)] for i in range(xw)]

        for ix in range(xw):
            #x = ix * self.grid_size + minx
            x = ix

            for iy in range(yw):
                #y = iy * self.grid_size + miny
                y = iy
                ug = self.calc_attractive_potential(x, y, (grid.goal.position.x), (grid.goal.position.y))
                if Point(x,y) not in grid.obstacles:
                    uo = self.calc_repulsive_potential(x=x, y=y, ox=([x.position.x for x in grid.obstacles]), 
                    oy=([y.position.y for y in grid.obstacles]), rr=(grid.agent.radius))
                else:
                    uo = 0

                uf = ug + uo    
                pmap[ix][iy] = uf
                point = Point(ix,iy)
                self.pmapheat.append((point,uf))
                
        #         #print("*****ug,uo",ug,uo)
                #if uo!=0:
                # print("2nd_________________________________")
                # print("ug=",ug)
                # print("uo=",uo)
                # print("uf=",uf)    
                # print('ix,iy',ix,iy)
                # print("*******************************")
        #             print("sx,sy=",grid.agent.position.x,grid.agent.position.y)
        #             print("gx,gy=",grid.goal.position.x,grid.goal.position.y)
        #             print("______")
        #             #print(np.hypot(x - grid.goal.position.x, y - grid.goal.position.y))                
        # #print("pmap=",pmap)
        # print("len(pmap)=", len(pmap[0])*len(pmap)
        # print ("xw=",xw,"yw=",yw)
        # print("grid_size=",self.grid_size)
        # print("ox=",[point.position.x for point in grid.obstacles])
        # print("len(ox)",len([point.position.x for point in grid.obstacles]))
        # print("_____________________________________")
        # print("oy=",[point.position.y for point in grid.obstacles])
        # print("len(oy)",len([point.position.y for point in grid.obstacles]))

        return pmap


    def calc_attractive_potential(self,x, y, gx, gy):
        return 0.5 * self.KP * np.hypot(x - gx, y - gy)


    def calc_repulsive_potential(self,x, y, ox, oy, rr):
        # search nearest obstacle
        minid = -1
        
        dmin = float("inf")
        for i, _ in enumerate(ox):
            d = np.hypot(x - ox[i], y - oy[i])
            #print("x,ox[i],y,oy[i]",x,ox[i], y,oy[i])
            #print("d=",d)
            
            #find closest obstacle
            if dmin >= d:
                dmin = d    
                minid = i
            
            
            # if d > rr:
            #     rep+= 0.0
            # else:
            #     rep+= 0.5 * self.ETA * (1.0 / d-1.0/1) ** 2
                # dmin = d    
                # minid = i
                #print("minid,dmin=",minid,dmin)
        
        #print("_____________________________________")
        #print("minid,dmin=",minid,dmin)
        
        
        #distance from point to closest obstacle
        dq = np.hypot(x - ox[minid], y - oy[minid])

        print('dq,1st',dq)

        if dq <= 0.99:
        #if dq <= 1.85:
            if dq <= 0.1:
                return 0.5 * self.ETA * (1.0 / dq) ** 2
            else:
                #return 0.5 * self.ETA * (1.0 / dq - 1.0 / rr) ** 2
                return 0.5 * self.ETA * (1.0 / dq) ** 2
        else:
            return 0    
            #return 0.5 * self.ETA * (1.0 / dq-1.0/1) ** 2

    def potential_field_planning(self,grid):

        # calc potential field
        pmap = self.calc_potential_field(grid)

        print('pmapheat',self.pmapheat)
        self.step_grid=pmap

        for lst in pmap:
            for num in lst:
                print(str((num))+"  ",end='')
            print("\n")
        
        self.pmapnew= set([num for lst in pmap for num in lst if num < 1000000])
        #print("pmap=",pmapnew)
        #print (pmap)

        # search path
        d = np.hypot(grid.agent.position.x - grid.goal.position.x, grid.agent.position.y - grid.goal.position.x)
        ix = round((grid.agent.position.x ) / self.grid_size)    #index starting x position
        iy = round((grid.agent.position.y ) / self.grid_size) 
        gix = round((grid.goal.position.x ) / self.grid_size)    #index goal x position
        giy = round((grid.goal.position.y ) / self.grid_size)
        
        # print('final________________________________________________')
        # print('ix,iy,gix,giy,d',ix,iy,gix,giy,d)
        # print('len(pmap),len(pmap[0])',len(pmap),len(pmap[0]))

        # if show_animation:
        #     draw_heatmap(pmap)
        #     # for stopping simulation with the esc key.
        #     plt.gcf().canvas.mpl_connect('key_release_event',
        #             lambda event: [exit(0) if event.key == 'escape' else None])
        #     plt.plot(ix, iy, "*k")
        #     plt.plot(gix, giy, "*m")

        rx, ry = [grid.agent.position.x], [grid.agent.position.y]
        print('rx,ry',rx,ry)
        motion = grid.EIGHT_POINTS_MOVE_VECTOR
        visited=[]
        lst = grid.obstacles
        for ob in lst:
            visited.append((ob.position.x,ob.position.y))
        while d >= self.grid_size:
            minp = float("inf")
            minix, miniy = -1, -1
            for point in motion:
                inx = int(ix + point.x)
                iny = int(iy + point.y)
                if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0 :
                    p = float("inf")  # outside area
                else:
                    p = pmap[inx][iny]
                
                point = (inx,iny)
                #find which neighbour has the largest potential value (so can move there)
                if minp > p and point not in visited:
                    minp = p
                    minix = inx
                    miniy = iny

            visited.append((minix,miniy))
            print("minix,miniy",minix,miniy)
            ix = minix
            iy = miniy
            d = np.hypot(grid.goal.position.x - ix, grid.goal.position.y - iy)
            print('d=',d)
            rx.append(iy)
            ry.append(ix)
            point1=Point(ix,iy)

            # if show_animation:
            #     plt.plot(ix, iy, ".r")
            #     plt.pause(0.01)
            self.move_agent(point1)
            self.key_frame()

        print("Goal!!")



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

        # path generation
        self.potential_field_planning(grid)

