from panda3d.core import NodePath, TransparencyAttrib, LVecBase4f, LineSegs

from simulator.services.services import Services
from simulator.views.map.data.map_data import MapData
from simulator.views.map.object.cube_mesh import CubeMesh

from structures import Point, DynamicColour, Colour, TRANSPARENT, WHITE, BLACK

import random
from typing import Any

import numpy as np
from nptyping import NDArray

class VoxelMap(MapData):
    traversables: NodePath
    traversables_wf: NodePath
    obstacles: NodePath
    obstacles_wf: NodePath

    traversables_mesh: CubeMesh
    traversables_wf_mesh: CubeMesh
    obstacles_mesh: CubeMesh
    obstacles_wf_mesh: CubeMesh

    def __init__(self, services: Services, data: NDArray[(Any, Any, Any), bool], parent: NodePath, name: str = "voxel_map", artificial_lighting: bool = False):
        super().__init__(services, data, parent, name)

        self.traversables_mesh = CubeMesh(self.traversables_data, self.name + '_traversables', artificial_lighting, hidden_faces=True)
        self.traversables = self.root.attach_new_node(self.traversables_mesh.geom_node)
        self.traversables_wf = self.root.attach_new_node(self.traversables_mesh.gen_wireframe())

        self.obstacles_mesh = CubeMesh(self.obstacles_data, self.name + '_obstacles', artificial_lighting, hidden_faces=True)
        self.obstacles = self.root.attach_new_node(self.obstacles_mesh.geom_node)
        self.obstacles_wf = self.root.attach_new_node(self.obstacles_mesh.gen_wireframe())

        self._add_colour(MapData.TRAVERSABLES, callback=self.__traversables_colour_callback)
        self._add_colour(MapData.TRAVERSABLES_WF, callback=lambda dc: self.__mesh_colour_callback(dc, self.traversables_wf))
        self._add_colour(MapData.OBSTACLES, callback=lambda dc: self.__mesh_colour_callback(dc, self.obstacles))
        self._add_colour(MapData.OBSTACLES_WF, callback=lambda dc: self.__mesh_colour_callback(dc, self.obstacles_wf))

        self._add_colour(MapData.AGENT)
        self._add_colour(MapData.TRACE)
        self._add_colour(MapData.GOAL)

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