from panda3d.core import Texture, GeomNode, LineSegs
from panda3d.core import GeomVertexFormat, GeomVertexData
from panda3d.core import Geom, GeomTriangles, GeomVertexWriter, GeomVertexArrayData
from panda3d.core import Vec3, Vec4
from panda3d.core import NodePath

from typing import List, Any, Tuple, Optional
from numbers import Real
import math

import numpy as np
from nptyping import NDArray

from structures import Point, Colour, WHITE
from simulator.views.map.geometry.meshes.common import normalise, Face


class VoxelMesh():
    name: str
    mesh: Geom

    __structure: NDArray[(Any, Any, Any), np.uint8]
    __mask: np.uint8
    __artificial_lighting: bool
    __default_colour: Colour
    __face_count: int

    # index: <face-index>
    # value: <cube-pos>
    __face_cube_map: List[Tuple[int, int, int]]

    # NDArray[(Any, Any, Any), Tuple[int, int, int, int, int, int]]
    # index: <cube-pos>
    # value: (<face-index-left>, <face-index-right>, <face-index-back>, <face-index-front>, <face-index-bottom>, <face-index-top>)
    __cube_face_map: NDArray[(Any, Any, Any), tuple]

    # index: <cube-pos>
    # value: is cube using the default colour
    __cube_default_coloured: NDArray[(Any, Any, Any), bool]

    __vertex_data_format: GeomVertexFormat
    __vertex_data: GeomVertexData

    _triangles: GeomTriangles
    __triangle_data: GeomVertexArrayData

    __vertex: GeomVertexWriter
    __normal: GeomVertexWriter
    __colour: GeomVertexWriter

    def __init__(self, structure: NDArray[(Any, Any, Any), np.uint8], mask: np.uint8, parent: NodePath, name: str = 'voxel_mesh', artificial_lighting: bool = False, default_colour: Colour = WHITE) -> None:
        self.name = name
        self.__structure = structure
        self.__mask = mask
        self.__artificial_lighting = artificial_lighting
        self.__default_colour = default_colour

        self.__body = parent.attach_new_node(self.name)
        self.__wireframe = parent.attach_new_node(self.name + "_wf")

        self.__vertex_data_format = GeomVertexFormat.getV3n3c4()
        self.__vertex_data = GeomVertexData(name, self.__vertex_data_format, Geom.UHStatic)

        self.mesh = Geom(self.__vertex_data)
        node = GeomNode(self.name)
        node.addGeom(self.mesh)
        self.__body.attach_new_node(node)

        self._triangles = GeomTriangles(Geom.UHStatic)
        self.__triangle_data = self._triangles.modifyVertices()

        self.__vertex = GeomVertexWriter(self.__vertex_data, 'vertex')
        self.__normal = GeomVertexWriter(self.__vertex_data, 'normal')
        self.__colour = GeomVertexWriter(self.__vertex_data, 'color')

        self.__face_count = 0

        self.__face_cube_map = []
        self.__cube_face_map = np.full(self.structure.shape, None, dtype=object)
        self.__cube_default_coloured = np.full(self.structure.shape, True, dtype=bool)

    def _add_cube_faces(self, idx) -> None:
        faces = []
        x, y, z = idx

        def add_face(face: Face, should_add: bool) -> None:
            self.__make_face(face, idx)
            self.__face_cube_map.append(idx)
            faces.append(self.__face_count-1)

        xdim, ydim, zdim = self.structure.shape
        add_face(Face.LEFT, (x-1) < 0 or not bool(self.structure[x-1, y, z] & self.mask))
        add_face(Face.RIGHT, (x+1) >= xdim or not bool(self.structure[x+1, y, z] & self.mask))
        add_face(Face.BACK, (y-1) < 0 or not bool(self.structure[x, y-1, z] & self.mask))
        add_face(Face.FRONT, (y+1) >= ydim or not bool(self.structure[x, y+1, z] & self.mask))
        add_face(Face.BOTTOM, (z-1) < 0 or not bool(self.structure[x, y, z-1] & self.mask))
        add_face(Face.TOP, (z+1) >= zdim or not bool(self.structure[x, y, z+1] & self.mask))

        self.__cube_face_map[idx] = tuple(faces)

    def __make_face(self, face: Face, pos: Point) -> None:
        colour = self._face_colour(self.default_colour, face)

        def make(x1, y1, z1, x2, y2, z2) -> None:
            if x1 == x2:
                self.__vertex.add_data3f(x1, y1, z1)
                self.__vertex.add_data3f(x2, y2, z1)
                self.__vertex.add_data3f(x2, y2, z2)
                self.__vertex.add_data3f(x1, y1, z2)

                self.__normal.add_data3(normalise(2 * x1 - 1, 2 * y1 - 1, 2 * z1 - 1))
                self.__normal.add_data3(normalise(2 * x2 - 1, 2 * y2 - 1, 2 * z1 - 1))
                self.__normal.add_data3(normalise(2 * x2 - 1, 2 * y2 - 1, 2 * z2 - 1))
                self.__normal.add_data3(normalise(2 * x1 - 1, 2 * y1 - 1, 2 * z2 - 1))
            else:
                self.__vertex.add_data3f(x1, y1, z1)
                self.__vertex.add_data3f(x2, y1, z1)
                self.__vertex.add_data3f(x2, y2, z2)
                self.__vertex.add_data3f(x1, y2, z2)

                self.__normal.add_data3(normalise(2 * x1 - 1, 2 * y1 - 1, 2 * z1 - 1))
                self.__normal.add_data3(normalise(2 * x2 - 1, 2 * y1 - 1, 2 * z1 - 1))
                self.__normal.add_data3(normalise(2 * x2 - 1, 2 * y2 - 1, 2 * z2 - 1))
                self.__normal.add_data3(normalise(2 * x1 - 1, 2 * y2 - 1, 2 * z2 - 1))

            self.__colour.set_row(self.__face_count * 4)
            self.__colour.add_data4f(*colour)
            self.__colour.add_data4f(*colour)
            self.__colour.add_data4f(*colour)
            self.__colour.add_data4f(*colour)

            vertex_id = self.__face_count * 4

            self._triangles.addVertices(vertex_id, vertex_id + 1, vertex_id + 3)
            self._triangles.addVertices(vertex_id + 1, vertex_id + 2, vertex_id + 3)

            self.__face_count += 1

        x, y, z = pos
        if face == Face.FRONT:
            make(x + 1, y + 1, z - 1, x, y + 1, z)
        elif face == Face.BACK:
            make(x, y, z - 1, x + 1, y, z)
        elif face == Face.RIGHT:
            make(x + 1, y, z - 1, x + 1, y + 1, z)
        elif face == Face.LEFT:
            make(x, y + 1, z - 1, x, y, z)
        elif face == Face.TOP:
            make(x + 1, y + 1, z, x, y, z)
        elif face == Face.BOTTOM:
            make(x, y + 1, z - 1, x + 1, y, z - 1)
        else:
            raise Exception("unknown face")

    def get_cube_colour(self, p: Point) -> Colour:
        faces = self.__cube_face_map[p.values]
        for i in range(len(faces)):
            self.__colour.set_row(faces[i] * 4)
            r, g, b, a = self.__colour.getData4f()
            if self.artificial_lighting:
                factor = self.__LIGHT_ATTENUATION_FACTOR(Face(i))
                return Colour(r / factor, g / factor, b / factor, a)
            else:
                return Colour(r, g, b, a)

        return self.default_colour

    def set_cube_colour(self, p: Point, colour: Colour) -> None:
        self.__cube_default_coloured[p.values] = False
        faces = self.__cube_face_map[p.values]
        for i in range(len(faces)):
            c = self._face_colour(colour, Face(i))

            self.__colour.set_row(faces[i] * 4)
            self.__colour.add_data4f(*c)
            self.__colour.add_data4f(*c)
            self.__colour.add_data4f(*c)
            self.__colour.add_data4f(*c)

    def reset_cube(self, p: Point) -> None:
        self.set_cube_colour(p, self.default_colour)
        self.__cube_default_coloured[p.values] = True

    def reset_all_cubes(self) -> None:
        for p in np.ndindex(self.structure.shape):
            if bool(self.structure[p] & self.mask) and not self.__cube_default_coloured[p]:
                self.reset_cube(p)

    @staticmethod
    def _attenuate_colour(colour: Colour, factor: Real) -> Colour:
        r, g, b, a = colour
        return Colour(r * factor, g * factor, b * factor, a)

    @staticmethod
    def _LIGHT_ATTENUATION_FACTOR(face: Face) -> Real:
        switcher = {Face.LEFT: 0.9,
                    Face.RIGHT: 1.0,
                    Face.BACK: 0.85,
                    Face.FRONT: 0.6,
                    Face.BOTTOM: 0.7,
                    Face.TOP: 1.0
                    }
        return switcher.get(face.value)

    def _face_colour(self, colour: Colour, face: Face) -> Colour:
        if self.artificial_lighting:
            return self._attenuate_colour(colour, self._LIGHT_ATTENUATION_FACTOR(face))
        else:
            return colour

    @property
    def artificial_lighting(self) -> str:
        return 'artificial_lighting'

    @artificial_lighting.getter
    def artificial_lighting(self) -> bool:
        return self.__artificial_lighting

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

    def cube_visible(self, p: Point) -> bool:
        return self.__cube_face_map[p.values] is not None

    @property
    def body(self) -> NodePath:
        return self.__body

    @property
    def wireframe(self) -> NodePath:
        return self.__wireframe
