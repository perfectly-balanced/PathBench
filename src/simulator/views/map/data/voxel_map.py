from panda3d.core import NodePath, TransparencyAttrib, LVecBase4f, LineSegs

from simulator.services.services import Services
from simulator.views.map.data.map_data import MapData
from simulator.views.map.geometry.meshes.dynamic_voxel_mesh import DynamicVoxelMesh
from simulator.views.map.geometry.meshes.static_voxel_mesh import StaticVoxelMesh

from structures import Point, DynamicColour, Colour, TRANSPARENT, WHITE, BLACK

import random
from typing import Any, Union

import numpy as np
from nptyping import NDArray

class VoxelMap(MapData):
    traversables: NodePath
    traversables_wf: NodePath
    obstacles: NodePath
    obstacles_wf: NodePath

    traversables_mesh: Union[DynamicVoxelMesh, StaticVoxelMesh]
    obstacles_mesh: Union[DynamicVoxelMesh, StaticVoxelMesh]

    def __init__(self, services: Services, data: NDArray[(Any, Any, Any), bool], parent: NodePath, name: str = "voxel_map", artificial_lighting: bool = False):
        super().__init__(services, data, parent, name)

        # Uses the fastest mesh possible
        Mesh = DynamicVoxelMesh if self._services.algorithm.map.mutable else StaticVoxelMesh

        self.traversables_mesh = Mesh(self.data, MapData.TRAVERSABLE_MASK, self.root, self.name + '_traversables', artificial_lighting=artificial_lighting)
        self.traversables = self.traversables_mesh.body
        self.traversables_wf = self.traversables_mesh.wireframe

        self.obstacles_mesh = Mesh(self.data, MapData.OBSTACLE_MASK, self.root, self.name + '_obstacles', artificial_lighting=artificial_lighting)
        self.obstacles = self.obstacles_mesh.body
        self.obstacles_wf = self.obstacles_mesh.wireframe

        self._add_colour(MapData.TRAVERSABLES, callback=self.__traversables_colour_callback)
        self._add_colour(MapData.TRAVERSABLES_WF, callback=lambda dc: self.__mesh_colour_callback(dc, self.traversables_wf))
        self._add_colour(MapData.OBSTACLES_WF, callback=lambda dc: self.__mesh_colour_callback(dc, self.obstacles_wf))
        if self._services.algorithm.map.mutable:
            self._add_colour(MapData.OBSTACLES, callback=self.__mutable_obstacles_colour_callback)
        else:
            self._add_colour(MapData.OBSTACLES, callback=lambda dc: self.__mesh_colour_callback(dc, self.obstacles))

        self._add_colour(MapData.AGENT)
        self._add_colour(MapData.TRACE)
        self._add_colour(MapData.GOAL)

    @property
    def dim(self) -> int:
        return 3

    def __traversables_colour_callback(self, dc: DynamicColour) -> None:
        self.traversables_mesh.default_colour = dc()

    def __mutable_obstacles_colour_callback(self, dc: DynamicColour) -> None:
        self.obstacles_mesh.default_colour = dc()

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
