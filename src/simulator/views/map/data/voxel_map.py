from panda3d.core import NodePath, TransparencyAttrib, LVecBase3f

from simulator.views.map.data.map_data import MapData
from simulator.views.map.object.cube_mesh import CubeMesh

from structures import Point, Colour, TRANSPARENT

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

    agent_visible: bool
    _agent_colour: Colour
    trace_visible: bool
    _trace_colour: Colour
    goal_visible: bool
    _goal_colour: Colour

    def __init__(self, data: List[List[List[bool]]], parent: NodePath, name: str = "voxel_map", artificial_lighting: bool = False):
        super().__init__(parent, name)

        self.obstacles_data = data
        self.traversables_data = {}
        for i in range(len(self.obstacles_data)):
            self.traversables_data[i] = {}
            for j in range(len(self.obstacles_data[i])):
                self.traversables_data[i][j] = {}
                for k in range(len(self.obstacles_data[i][j])):
                    self.traversables_data[i][j][k] = False if self.obstacles_data[i][j][k] else True
                    
        self.traversables_mesh = CubeMesh(self.traversables_data, self.name + '_traversables', artificial_lighting, hidden_faces = True)
        self.traversables = self.root.attach_new_node(self.traversables_mesh.geom_node)

        self.traversables_wf_mesh = CubeMesh(self.traversables_data, self.name + '_traversables_wf', artificial_lighting, hidden_faces = True)
        self.traversables_wf = self.root.attach_new_node(self.traversables_wf_mesh.geom_node)
        self.traversables_wf.setRenderModeWireframe()
        self.traversables_wf.setRenderModeThickness(2.2)

        self.obstacles_mesh = CubeMesh(self.obstacles_data, self.name + '_obstacles', artificial_lighting, hidden_faces = True)
        self.obstacles = self.root.attach_new_node(self.obstacles_mesh.geom_node)

        self.obstacles_wf_mesh = CubeMesh(self.obstacles_data, self.name + '_obstacles_wf', artificial_lighting, hidden_faces = True)
        self.obstacles_wf = self.root.attach_new_node(self.obstacles_wf_mesh.geom_node)
        self.obstacles_wf.setRenderModeWireframe()
        self.obstacles_wf.setRenderModeThickness(2.2)

        self.agent_visible = True
        self._agent_colour = Colour(0.8, 0, 0)
        self.goal_visible = True
        self._goal_colour = Colour(0, 0.9, 0)
        self.trace_visible = True
        self._trace_colour = Colour(0, 0.9, 0)

    @property
    def agent_colour(self) -> str:
        return 'agent_colour'
    
    @agent_colour.getter
    def agent_colour(self):
        return self._agent_colour if self.agent_visible else TRANSPARENT

    @property
    def trace_colour(self) -> str:
        return 'trace_colour'
    
    @trace_colour.getter
    def trace_colour(self):
        return self._trace_colour if self.trace_visible else TRANSPARENT

    @property
    def goal_colour(self) -> str:
        return 'goal_colour'
    
    @goal_colour.getter
    def goal_colour(self):
        return self._goal_colour if self.goal_visible else TRANSPARENT