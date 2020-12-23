from panda3d.core import Texture, GeomNode, LineSegs
from panda3d.core import GeomVertexFormat, GeomVertexData
from panda3d.core import Geom, GeomTriangles, GeomVertexWriter, GeomVertexRewriter, GeomVertexArrayData
from panda3d.core import Vec3, Vec4

from typing import List, Any, Tuple, Optional
from numbers import Real
import math

import numpy as np
from nptyping import NDArray

from structures import Point, Colour, WHITE
from simulator.views.map.meshes.common import normalise

class DynamicVoxelMesh():
    name: str

    __structure: NDArray[(Any, Any, Any), np.uint8]
    __mask: np.uint8
    __artificial_lighting: bool
    __default_colour: Colour

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

    __triangles: GeomTriangles
    __triangle_data: GeomVertexArrayData

    __vertex: GeomVertexWriter
    __normal: GeomVertexWriter
    __texcoord: GeomVertexWriter
    __colour: GeomVertexRewriter

    def __init__(self, structure: NDArray[(Any, Any, Any), np.uint8], mask: np.uint8, name: str = 'DynamicVoxelMesh', artificial_lighting: bool = False, default_colour: Colour = WHITE) -> None:
        self.name = name
        self.__structure = structure
        self.__mask = mask
        self.__artificial_lighting = artificial_lighting
        self.__default_colour = default_colour

        self.__vertex_data_format = GeomVertexFormat.getV3n3()
        self.__vertex_data = GeomVertexData(name, self.__vertex_data_format, Geom.UHStatic)
        self.__vertex_data.set_num_rows(4)

        self.__cube_mesh = Geom(self.__vertex_data)
        self.__triangles = GeomTriangles(Geom.UHStatic)
        self.__triangle_data = self.__triangles.modifyVertices()

        self.__vertex = GeomVertexWriter(self.__vertex_data, 'vertex')
        self.__normal = GeomVertexWriter(self.__vertex_data, 'normal')

        self.__face_count = 0

        def add_face(face: Face) -> None:
            self.__make_face(face, pos)
            self.__face_cube_map.append(pos)
            faces.append(self.__face_count-1)

        add_face(Face.LEFT)
        add_face(Face.RIGHT)
        add_face(Face.BACK)
        add_face(Face.FRONT)
        add_face(Face.BOTTOM)
        add_face(Face.TOP)

        self.__face_cube_map = []
        self.__cube_face_map = np.empty(self.structure.shape, dtype=object)
        self.__cube_default_coloured = np.full(self.structure.shape, True, dtype=bool)

        for x, y, z in np.ndindex(self.structure.shape):
            faces = []
            pos = (x, y, z)

            # skip if cube doesn't exist
            if not bool(self.structure[pos] & self.mask):
                self.__cube_face_map[pos] = (None, None, None, None, None, None)
                continue

            

            self.__cube_face_map[pos] = tuple(faces)

        self.__triangles.close_primitive()
        self.mesh.add_primitive(self.__triangles)
    
    def __create_cube_mesh(self) -> None:

    
    def __make_face(self, face: Face, pos: Point) -> None:
        colour = self.__face_colour(self.default_colour, face)

        def make(x1, y1, z1, x2, y2, z2) -> None:
            if x1 == x2:
                self.__vertex.addData3f(x1, y1, z1)
                self.__vertex.addData3f(x2, y2, z1)
                self.__vertex.addData3f(x2, y2, z2)
                self.__vertex.addData3f(x1, y1, z2)

                self.__normal.addData3(normalise(2 * x1 - 1, 2 * y1 - 1, 2 * z1 - 1))
                self.__normal.addData3(normalise(2 * x2 - 1, 2 * y2 - 1, 2 * z1 - 1))
                self.__normal.addData3(normalise(2 * x2 - 1, 2 * y2 - 1, 2 * z2 - 1))
                self.__normal.addData3(normalise(2 * x1 - 1, 2 * y1 - 1, 2 * z2 - 1))
            else:
                self.__vertex.addData3f(x1, y1, z1)
                self.__vertex.addData3f(x2, y1, z1)
                self.__vertex.addData3f(x2, y2, z2)
                self.__vertex.addData3f(x1, y2, z2)

                self.__normal.addData3(normalise(2 * x1 - 1, 2 * y1 - 1, 2 * z1 - 1))
                self.__normal.addData3(normalise(2 * x2 - 1, 2 * y1 - 1, 2 * z1 - 1))
                self.__normal.addData3(normalise(2 * x2 - 1, 2 * y2 - 1, 2 * z2 - 1))
                self.__normal.addData3(normalise(2 * x1 - 1, 2 * y2 - 1, 2 * z2 - 1))

            vertex_id = self.__face_count * 4

            self.__triangles.addVertices(vertex_id, vertex_id + 1, vertex_id + 3)
            self.__triangles.addVertices(vertex_id + 1, vertex_id + 2, vertex_id + 3)

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
            if faces[i] != None:
                self.__colour.setRow(faces[i] * 4)
                r, g, b, a = self.__colour.getData4f()
                if self.artificial_lighting:
                    factor = self.__LIGHT_ATTENUATION_FACTOR(Face(i))
                    return Colour(r / factor, g / factor, b / factor, a)
                else:
                    return Colour(r, g, b, a)

        return self.default_colour

    def set_cube_colour(self, p: Point, colour: Colour) -> None:
        pass

    def reset_cube(self, p: Point) -> None:
        self.set_cube_colour(p, self.default_colour)
        self.__cube_default_coloured[p.values] = True

    def reset_all_cubes(self) -> None:
        for p in np.ndindex(self.structure.shape):
            if bool(self.structure[p] & self.mask) and not self.__cube_default_coloured[p]:
                self.reset_cube(p)

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
        for f in self.__cube_face_map[p.values]:
            if f is not None:
                return True
        return False

    @property
    def body_node(self) -> GeomNode:
        node = GeomNode(self.name)
        node.addGeom(self.mesh)
        return node

    @property
    def wireframe_node(self) -> GeomNode:
        thickness: float = 5

        def is_connected(x, y, z, x1, y1, z1):
            return (abs(x - x1) == 1 and abs(y - y1) != 1 and abs(z - z1) != 1) or \
                   (abs(x - x1) != 1 and abs(y - y1) == 1 and abs(z - z1) != 1) or \
                   (abs(x - x1) != 1 and abs(y - y1) != 1 and abs(z - z1) == 1)

        ls = LineSegs()
        ls.set_thickness(thickness)
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
        return ls.create()
