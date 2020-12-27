from panda3d.core import LineSegs

import numpy as np

from simulator.views.map.geometry.meshes.voxel_mesh import VoxelMesh
from utility.misc import exclude_from_dict

class StaticVoxelMesh(VoxelMesh):
    """
    StaticVoxelMesh is used for maps that are immutable. This
    implementation (specifically the wireframe) is much more
    efficient than using DynamicVoxelMap - difference becomes
    apparent on large maps.
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **exclude_from_dict(kwargs, ["wireframe_thickness"]))
        wireframe_thickness: float = kwargs["wireframe_thickness"] if "wireframe_thickness" in kwargs else 5

        def is_connected(x, y, z, x1, y1, z1):
            return (abs(x - x1) == 1 and abs(y - y1) != 1 and abs(z - z1) != 1) or \
                   (abs(x - x1) != 1 and abs(y - y1) == 1 and abs(z - z1) != 1) or \
                   (abs(x - x1) != 1 and abs(y - y1) != 1 and abs(z - z1) == 1)

        ls = LineSegs()
        ls.set_thickness(wireframe_thickness)
        for i, j, k in np.ndindex(self.structure.shape):
            if bool(self.structure[i, j, k] & self.mask):
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
        self.wireframe.attach_new_node(ls.create())

        for idx in np.ndindex(self.structure.shape):
            if bool(self.structure[idx] & self.mask):
                self._add_cube_faces(idx)

        self._triangles.close_primitive()
        self.mesh.add_primitive(self._triangles)
