from panda3d.core import NodePath, TransparencyAttrib
from .cube_mesh import CubeMesh

from typing import List

class VoxelMap():
    root: NodePath
    traversables: NodePath
    traversables_wf: NodePath
    obstacles: NodePath
    obstacles_wf: NodePath

    traversables_mesh: CubeMesh
    traversables_wf_mesh: CubeMesh
    obstacles_mesh: CubeMesh
    obstacles_wf_mesh: CubeMesh

    def __init__(self, data: List[List[List[bool]]], parent: NodePath, name: str = "voxel_map", artificial_lighting: bool = False):
        self.obstacles_data = data
        self.traversables_data = {}
        for i in range(len(self.obstacles_data)):
            self.traversables_data[i] = {}
            for j in range(len(self.obstacles_data[i])):
                self.traversables_data[i][j] = {}
                for k in range(len(self.obstacles_data[i][j])):
                    self.traversables_data[i][j][k] = False if self.obstacles_data[i][j][k] else True
                    
        self.root = parent.attach_new_node(name)

        self.traversables_mesh = CubeMesh(self.traversables_data, name + '_traversables', artificial_lighting, hidden_faces = False)
        self.traversables = self.root.attach_new_node(self.traversables_mesh.geom_node)
        self.traversables.set_transparency(TransparencyAttrib.M_alpha)

        self.traversables_wf_mesh = CubeMesh(self.traversables_data, name + '_traversables_wf', artificial_lighting, hidden_faces = True)
        self.traversables_wf = self.root.attach_new_node(self.traversables_wf_mesh.geom_node)
        self.traversables_wf.set_transparency(TransparencyAttrib.M_alpha)
        self.traversables_wf.setRenderModeWireframe()

        self.obstacles_mesh = CubeMesh(self.obstacles_data, name + '_obstacles', artificial_lighting, hidden_faces = False)
        self.obstacles = self.root.attach_new_node(self.obstacles_mesh.geom_node)
        self.obstacles.set_transparency(TransparencyAttrib.M_alpha)

        self.obstacles_wf_mesh = CubeMesh(self.obstacles_data, name + '_obstacles_wf', artificial_lighting, hidden_faces = True)
        self.obstacles_wf = self.root.attach_new_node(self.obstacles_wf_mesh.geom_node)
        self.obstacles_wf.set_transparency(TransparencyAttrib.M_alpha)
        self.obstacles_wf.setRenderModeWireframe()