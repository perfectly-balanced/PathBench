from panda3d.core import NodePath, TransparencyAttrib, LVecBase4f, LineSegs

from simulator.services.services import Services
from simulator.views.map.data.map_data import MapData
from simulator.views.map.object.cube_mesh import CubeMesh

from structures import Point, DynamicColour, Colour, TRANSPARENT, WHITE, BLACK

from typing import List
import random

import numpy as np

class VoxelMap(MapData):
    traversables: NodePath
    traversables_wf: NodePath
    obstacles: NodePath
    obstacles_wf: NodePath

    traversables_mesh: CubeMesh
    traversables_wf_mesh: CubeMesh
    obstacles_mesh: CubeMesh
    obstacles_wf_mesh: CubeMesh

    TRAVERSABLES: str = "traversables"
    TRAVERSABLES_WF: str = "traversables_wf"
    OBSTACLES: str = "obstacles"
    OBSTACLES_WF: str = "obstacles_wf"
    AGENT: str = "agent"
    TRACE: str = "trace"
    GOAL: str = "goal"

    def __init__(self, services: Services, data: List[List[List[bool]]], parent: NodePath, name: str = "voxel_map", artificial_lighting: bool = False):
        super().__init__(services, parent, name)

        self.obstacles_data = data
        self.traversables_data = {}
        for i in range(len(self.obstacles_data)):
            self.traversables_data[i] = {}
            for j in range(len(self.obstacles_data[i])):
                self.traversables_data[i][j] = {}
                for k in range(len(self.obstacles_data[i][j])):
                    self.traversables_data[i][j][k] = False if self.obstacles_data[i][j][k] else True
        
        self.np_obstacles_data = np.empty((len(self.obstacles_data), len(self.obstacles_data[0]), len(self.obstacles_data[0][0])), dtype=bool)
        self.np_traversables_data = np.empty((len(self.obstacles_data), len(self.obstacles_data[0]), len(self.obstacles_data[0][0])), dtype=bool)
        for i in range(len(self.obstacles_data)):
            for j in range(len(self.obstacles_data[i])):
                for k in range(len(self.obstacles_data[i][j])):
                    self.np_obstacles_data[i, j, k] = self.obstacles_data[i][j][k]
                    self.np_traversables_data[i, j, k] = self.traversables_data[i][j][k]

        self.traversables_mesh = CubeMesh(self.np_traversables_data, self.name + '_traversables', artificial_lighting, hidden_faces=True)
        self.traversables = self.root.attach_new_node(self.traversables_mesh.geom_node)


        def gen_cube_mesh_wireframe():
            ls = LineSegs()
            ls.set_thickness(5)
            for i in range(len(self.traversables_data)):
                for j in range(len(self.traversables_data[i])):
                    for k in range(len(self.traversables_data[i][j])):
                        if self.traversables_data[i][j][k]:
                            self.arr_x = [0, 0, 0, 0, 1, 1, 1, 1]
                            self.arr_y = [0, 0, 1, 1, 1, 1, 0, 0]
                            self.arr_z = [0, -1, -1, 0, 0, -1, -1, 0]
                            for pos1 in range(len(self.arr_x) - 1):
                                for pos2 in range(pos1, len(self.arr_x)):
                                    x = self.arr_x[pos1] + i
                                    y = self.arr_y[pos1] + j
                                    z = self.arr_z[pos1] + k
                                    x1 = self.arr_x[pos2] + i
                                    y1 = self.arr_y[pos2] + j
                                    z1 = self.arr_z[pos2] + k
                                    if (is_connected(x, y, z, x1, y1, z1)):
                                        ls.move_to(x, y, z)
                                        ls.draw_to(x1, y1, z1)
            np = ls.create()
            return np

        def gen_cube_mesh_wireframe_obstacles():
            ls = LineSegs()
            ls.set_thickness(5)
            for i in range(len(self.obstacles_data)):
                for j in range(len(self.obstacles_data[i])):
                    for k in range(len(self.obstacles_data[i][j])):
                        if self.obstacles_data[i][j][k]:
                            self.arr_x = [0, 0, 0, 0, 1, 1, 1, 1]
                            self.arr_y = [0, 0, 1, 1, 1, 1, 0, 0]
                            self.arr_z = [0, -1, -1, 0, 0, -1, -1, 0]
                            for pos1 in range(len(self.arr_x) - 1):
                                for pos2 in range(pos1, len(self.arr_x)):
                                    x = self.arr_x[pos1] + i
                                    y = self.arr_y[pos1] + j
                                    z = self.arr_z[pos1] + k
                                    x1 = self.arr_x[pos2] + i
                                    y1 = self.arr_y[pos2] + j
                                    z1 = self.arr_z[pos2] + k
                                    if (is_connected(x, y, z, x1, y1, z1)):
                                        ls.move_to(x, y, z)
                                        ls.draw_to(x1, y1, z1)
            np = ls.create()
            return np

        def is_connected(x, y, z, x1, y1, z1):
            return (abs(x - x1) == 1 and abs(y - y1) != 1 and abs(z - z1) != 1) or (
                    abs(x - x1) != 1 and abs(y - y1) == 1 and abs(z - z1) != 1) or (
                           abs(x - x1) != 1 and abs(y - y1) != 1 and abs(z - z1) == 1)

        self.traversables_wf = self.root.attach_new_node(gen_cube_mesh_wireframe())
        self.obstacles_wf = self.root.attach_new_node(gen_cube_mesh_wireframe_obstacles())

        self.obstacles_mesh = CubeMesh(self.np_obstacles_data, self.name + '_obstacles', artificial_lighting, hidden_faces=True)
        self.obstacles = self.root.attach_new_node(self.obstacles_mesh.geom_node)

        self._add_colour(VoxelMap.TRAVERSABLES, WHITE, callback=self.__traversables_colour_callback)
        self._add_colour(VoxelMap.TRAVERSABLES_WF, BLACK, callback=lambda dc: self.__mesh_colour_callback(dc, self.traversables_wf))
        self._add_colour(VoxelMap.OBSTACLES, BLACK, callback=lambda dc: self.__mesh_colour_callback(dc, self.obstacles))
        self._add_colour(VoxelMap.OBSTACLES_WF, WHITE, callback=lambda dc: self.__mesh_colour_callback(dc, self.obstacles_wf))

        self._add_colour(VoxelMap.AGENT, Colour(0.8, 0, 0))
        self._add_colour(VoxelMap.TRACE, Colour(0, 0.9, 0))
        self._add_colour(VoxelMap.GOAL, Colour(0, 0.9, 0))

    def __traversables_colour_callback(self, dc: DynamicColour) -> None:
        self.traversables_mesh.default_colour = dc()

    def __mesh_colour_callback(self, dc: DynamicColour, np: NodePath) -> None:
        if dc.visible:
            np.show()
        else:
            np.hide()
        np.set_color(LVecBase4f(*dc()), 1)

    def destroy(self) -> None:
        self.traversables.remove_node()
        self.traversables_wf.remove_node()
        self.obstacles.remove_node()
        self.obstacles_wf.remove_node()
        super().destroy()