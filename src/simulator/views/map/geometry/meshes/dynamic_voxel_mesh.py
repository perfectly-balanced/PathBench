from panda3d.core import NodePath

from typing import List, Any

import numpy as np
from nptyping import NDArray

from structures import Point, Colour, TRANSPARENT
from simulator.views.map.geometry.meshes.cube_mesh import CubeMesh
from simulator.views.map.geometry.meshes.voxel_mesh import VoxelMesh
from utility.misc import exclude_from_dict


class DynamicVoxelMesh(VoxelMesh):
    """
    DynamicVoxelMesh is used for maps that are mutable.

    Wireframes are quite laggy when map gets large.
    Potential room for optimisation by aggregating them.
    For example, slice the grid structure into sectors,
    with each sector having a single wireframe node. The 
    wireframe needs to be fully reconstructed every time
    a single cube in the sector changes. A good heuristic
    for sector size is required to see good performance
    since this is used for dynamic environments.
    """

    __wireframe_instance: NodePath
    __wireframe_instance_name: str
    __wireframes: NDArray[(Any, Any, Any), NodePath]

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **exclude_from_dict(kwargs, ["wireframe_thickness"]))
        wireframe_thickness: float = kwargs["wireframe_thickness"] if "wireframe_thickness" in kwargs else 5

        self.__wireframe_instance = self.wireframe.attach_new_node(CubeMesh(wireframe_thickness=wireframe_thickness).wireframe_node)
        self.__wireframe_instance.detach_node()
        self.__wireframe_instance_name = self.name + '_wf_inst'

        self.__wireframes = np.full(self.structure.shape, None, dtype=NodePath)

        for idx in np.ndindex(self.structure.shape):
            if bool(self.structure[idx] & self.mask):
                self.__add_cube(idx)

        self._triangles.close_primitive()
        self.mesh.add_primitive(self._triangles)

    def __add_cube(self, idx) -> None:
        self._add_cube_faces(idx)

        p = self.wireframe.attach_new_node(self.__wireframe_instance_name)
        self.__wireframe_instance.instance_to(p)
        p.set_pos(*idx)
        self.__wireframes[idx] = p

    def structural_update(self, cubes_updated: List[Point]) -> None:
        added_cube = False
        for p in cubes_updated:
            if bool(self.structure[p.values] & self.mask):
                if self.__wireframes[p.values] is None:
                    added_cube = True
                    self.__add_cube(p.values)
                    self.reset_cube(p)
                else:
                    self.reset_cube(p)
                    path = self.__wireframes[p.values]
                    self.wireframe.attach_node(path)
                    path.set_pos(*p)
            elif self.__wireframes[p.values] is not None:
                self.set_cube_colour(p, TRANSPARENT)
                self.__wireframes[p.values].detach_node()

        if added_cube:
            self._triangles.close_primitive()
            self.mesh.add_primitive(self._triangles)
