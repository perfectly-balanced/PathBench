from panda3d.core import Texture, GeomNode, LineSegs
from panda3d.core import GeomVertexFormat, GeomVertexData
from panda3d.core import Geom, GeomTriangles, GeomVertexWriter, GeomVertexRewriter, GeomVertexArrayData
from panda3d.core import Vec3, Vec4
from panda3d.core import NodePath

from typing import List, Any, Tuple, Optional
from numbers import Real
import math

import numpy as np
from nptyping import NDArray

from structures import Point, Colour, WHITE
from simulator.views.map.geometry.meshes.cube_mesh import CubeMesh

class OldDynamicVoxelMesh():
    """
    This works but it doesn't scale. Could easily add full dynamicness
    to this but due to performance issues this route was abandoned. Remains
    here as an illustration of what not to do.
    """

    name: str

    __structure: NDArray[(Any, Any, Any), np.uint8]
    __mask: np.uint8

    __body: NodePath
    __wireframe: NodePath
    __default_colour: Colour
    __cubes: NDArray[(Any, Any, Any), NodePath]
    __wireframes: NDArray[(Any, Any, Any), NodePath]
    __cube_default_coloured: NDArray[(Any, Any, Any), bool]
    __cube_mesh: CubeMesh
    __cube_instance: NodePath
    __cube_instance_name: str
    __wireframe_instance: NodePath
    __wireframe_instance_name: str

    def __init__(self, structure: NDArray[(Any, Any, Any), np.uint8], mask: np.uint8, parent: NodePath, name: str = 'old_dynamic_voxel_mesh', default_colour: Colour = WHITE) -> None:
        self.name = name
        self.__structure = structure
        self.__mask = mask
        self.__default_colour = default_colour

        self.__body = parent.attach_new_node(self.name)
        self.__wireframe = parent.attach_new_node(self.name + "_wf")

        self.__cube_mesh = CubeMesh()
        self.__wireframe_instance = self.__wireframe.attach_new_node(self.__cube_mesh.wireframe_node)
        self.__wireframe_instance.detach_node()
        self.__wireframe_instance_name = name + '_wf_inst'
        self.__cube_instance = self.__body.attach_new_node(self.__cube_mesh.body_node)
        self.__cube_instance.detach_node()
        self.__cube_instance_name = name + '_cube_inst'

        self.__cube_default_coloured = np.full(self.structure.shape, True, dtype=bool)
        self.__wireframes = np.empty(self.structure.shape, dtype=NodePath)
        self.__cubes = np.empty(self.structure.shape, dtype=NodePath)

        for idx in np.ndindex(self.structure.shape):
            if bool(self.structure[idx] & self.mask):
                self.__cubes[idx] = path = self.__body.attach_new_node(self.__cube_instance_name)
                self.__cube_instance.instance_to(path)
                path.set_color(*self.__default_colour)
                path.set_pos(*idx)

                self.__wireframes[idx] = path = self.__wireframe.attach_new_node(self.__wireframe_instance_name)
                self.__wireframe_instance.instance_to(path)
                path.set_pos(*idx)

    def get_cube_colour(self, p: Point) -> Colour:
        return Colour(*self.__cubes[p.values].get_color())

    def set_cube_colour(self, p: Point, colour: Colour) -> None:
        self.__cube_default_coloured[p.values] = False
        self.__cubes[p.values].set_color(*colour)

    def reset_cube(self, p: Point) -> None:
        self.set_cube_colour(p, self.default_colour)
        self.__cube_default_coloured[p.values] = True

    def reset_all_cubes(self) -> None:
        for idx in np.ndindex(self.structure.shape):
            if bool(self.structure[idx] & self.mask) and not self.__cube_default_coloured[idx]:
                self.reset_cube(Point(*idx))

    @property
    def default_colour(self) -> str:
        return 'default_colour'

    @default_colour.getter
    def default_colour(self) -> Colour:
        return self.__default_colour

    @default_colour.setter
    def default_colour(self, value: Colour) -> None:
        self.__default_colour = value

        # update colour of cubes that have old clear colour
        for p in np.ndindex(self.structure.shape):
            if bool(self.structure[p] & self.mask) and self.__cube_default_coloured[p]:
                self.reset_cube(Point(*p))

    @property
    def structure(self) -> NDArray[(Any, Any, Any), np.uint8]:
        return self.__structure

    @property
    def mask(self) -> np.uint8:
        return self.__mask

    @property
    def body(self) -> NodePath:
        return self.__body

    @property
    def wireframe(self) -> NodePath:
        return self.__wireframe
