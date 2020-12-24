from panda3d.core import NodePath

from typing import List, Any

import numpy as np
from nptyping import NDArray

from structures import Point, Colour, TRANSPARENT
from simulator.views.map.meshes.cube_mesh import CubeMesh
from simulator.views.map.meshes.voxel_mesh import VoxelMesh
from utility.misc import exclude_from_dict


class MutableVoxelMesh(VoxelMesh):
    """ Wireframes are quite laggy, potential room for optimisation by aggregating them. """

    __wireframe_instance: NodePath
    __wireframe_instance_name: str
    __wireframes: NDArray[(Any, Any, Any), NodePath]

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **exclude_from_dict(kwargs, ["wireframe_thickness"]))
        wireframe_thickness: float = kwargs["wireframe_thickness"] if "wireframe_thickness" in kwargs else 5

        self.__wireframe_instance = self.wireframe.attach_new_node(CubeMesh(wireframe_thickness=wireframe_thickness).wireframe_node)
        self.__wireframe_instance.detach_node()
        self.__wireframe_instance_name = self.name + '_wf_inst'

        self.__wireframes = np.empty(self.structure.shape, dtype=NodePath)

        for idx in np.ndindex(self.structure.shape):
            if bool(self.structure[idx] & self.mask):
                self.__add_cube(idx)
            else:
                self.__wireframes[idx] = None

        self._triangles.close_primitive()
        self.mesh.add_primitive(self._triangles)

    def __add_cube(self, idx) -> None:
        self._add_cube_faces(idx)

        self.__wireframes[idx] = path = self.wireframe.attach_new_node(self.__wireframe_instance_name)
        self.__wireframe_instance.instance_to(path)
        path.set_pos(*idx)

    def structural_update(self, cubes_updated: List[Point]) -> None:
        for c in cubes_updated:
            if bool(self.structure[c.values] & self.mask):
                if self.__wireframes[c.values] is None:
                    self.__add_cube(c.values)
                else:
                    self.reset_cube(c)
                    path = self.__wireframes[c.values]
                    self.wireframe.attach_node(path)
                    path.set_pos(*c)
            elif self.__wireframes[c.values] is not None:
                self.set_cube_colour(c, TRANSPARENT)
                self.__wireframes[c.values].detach_node()

        self._triangles.close_primitive()
        self.mesh.add_primitive(self._triangles)
