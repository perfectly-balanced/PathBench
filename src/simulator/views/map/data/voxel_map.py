from panda3d.core import NodePath, TransparencyAttrib, LVecBase4f

from simulator.services.services import Services
from simulator.views.map.data.map_data import MapData
from simulator.views.map.object.cube_mesh import CubeMesh

from structures import Point, DynamicColour, Colour, TRANSPARENT

from typing import List
import random

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

        self.traversables_mesh = CubeMesh(self.traversables_data, self.name + '_traversables', artificial_lighting, hidden_faces=True)
        self.traversables = self.root.attach_new_node(self.traversables_mesh.geom_node)

        self.traversables_wf_mesh = CubeMesh(self.traversables_data, self.name + '_traversables_wf', artificial_lighting, hidden_faces=True)
        self.traversables_wf = self.root.attach_new_node(self.traversables_wf_mesh.geom_node)
        self.traversables_wf.setRenderModeWireframe()
        self.traversables_wf.setRenderModeThickness(2.2)

        self.obstacles_mesh = CubeMesh(self.obstacles_data, self.name + '_obstacles', artificial_lighting, hidden_faces=True)
        self.obstacles = self.root.attach_new_node(self.obstacles_mesh.geom_node)

        self.obstacles_wf_mesh = CubeMesh(self.obstacles_data, self.name + '_obstacles_wf', artificial_lighting, hidden_faces=True)
        self.obstacles_wf = self.root.attach_new_node(self.obstacles_wf_mesh.geom_node)
        self.obstacles_wf.setRenderModeWireframe()
        self.obstacles_wf.setRenderModeThickness(2.2)

        self.add_colour(VoxelMap.TRAVERSABLES, Colour(1), callback=self.__traversables_colour_callback)
        self.add_colour(VoxelMap.TRAVERSABLES_WF, Colour(0), callback=lambda dc: self.__mesh_colour_callback(dc, self.traversables_wf))
        self.add_colour(VoxelMap.OBSTACLES, Colour(0), callback=lambda dc: self.__mesh_colour_callback(dc, self.obstacles))
        self.add_colour(VoxelMap.OBSTACLES_WF, Colour(1), callback=lambda dc: self.__mesh_colour_callback(dc, self.obstacles_wf))

        self.add_colour(VoxelMap.AGENT, Colour(0.8, 0, 0))
        self.add_colour(VoxelMap.TRACE, Colour(0, 0.9, 0))
        self.add_colour(VoxelMap.GOAL, Colour(0, 0.9, 0))

    def __traversables_colour_callback(self, dc: DynamicColour) -> None:
        self.traversables_mesh.default_colour = dc()
        self._dynamic_colour_callback(dc)

    def __mesh_colour_callback(self, dc: DynamicColour, np: NodePath) -> None:
        if dc.visible:
            np.show()
        else:
            np.hide()
        np.set_color(LVecBase4f(*dc()))
