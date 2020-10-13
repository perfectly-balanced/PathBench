from panda3d.core import Texture, GeomNode
from panda3d.core import GeomVertexFormat, GeomVertexData
from panda3d.core import Geom, GeomTriangles, GeomVertexWriter, GeomVertexRewriter
from panda3d.core import LVector3, Vec3, Vec4
from enum import IntEnum, unique, Enum
from typing import List
from numbers import Real
import math

from .types import TColour, Colour, IntPoint3

def normalise(*args):
    v = LVector3(*args)
    v.normalize()
    return v

@unique
class Face(IntEnum):
    LEFT = 0
    RIGHT = 1
    BACK = 2
    FRONT = 3
    BOTTOM = 4
    TOP = 5

class CubeMesh():
    name: str
    mesh: Geom
    structure: List[List[List[bool]]]

    def __init__(self, structure: List[List[List[bool]]], name: str = 'CubeMesh', artificial_lighting: bool = False, default_colour: Colour = 1.0) -> None:
        self.structure = structure
        self.name = name
        self.__artificial_lighting = artificial_lighting
        self.__default_colour = default_colour

        self.__vertex_data_format = GeomVertexFormat.getV3n3c4t2()
        self.__vertex_data = GeomVertexData(name, self.__vertex_data_format, Geom.UHDynamic)

        self.mesh = Geom(self.__vertex_data)
        self.__triangles = GeomTriangles(Geom.UHDynamic)
        self.__triangle_data = self.__triangles.modifyVertices()

        self.__vertex = GeomVertexWriter(self.__vertex_data, 'vertex')
        self.__normal = GeomVertexWriter(self.__vertex_data, 'normal')
        self.__colour = GeomVertexRewriter(self.__vertex_data, 'color')
        self.__texcoord = GeomVertexWriter(self.__vertex_data, 'texcoord')

        self.__face_count = 0

        # List[IntPoint3]
        # key: <face-index>
        # value: <cube-pos>
        self.__face_cube_map = []

        # List[List[List[Tuple[Integral, Integral, Integral, Integral, Integral, Integral]]]]
        # key: <cube-pos>
        # value: (<face-index-left>, <face-index-right>, <face-index-back>, <face-index-front>, <face-index-bottom>, <face-index-top>)
        self.__cube_face_map = []

        # we only make visible faces: if there are two adjacent faces, they aren't added.
        self.__cube_face_map = [None for _ in self.structure]
        for i in self.structure:
            self.__cube_face_map[i] = [None for _ in self.structure[i]]
            for j in self.structure[i]:
                self.__cube_face_map[i][j] = [None for _ in self.structure[i][j]]
                for k in self.structure[i][j]:
                    faces = []
                    pos = (i, j, k)

                    # skip if cube doesn't exist
                    if not self.structure[i][j][k]:
                        self.__cube_face_map[i][j][k] = (None, None, None, None, None, None)
                        continue

                    def add_face(face: Face, should_add: bool) -> None:
                        if should_add:
                            self.__make_face(face, pos)
                            self.__face_cube_map.append(pos)
                            faces.append(self.__face_count-1)
                        else:
                            faces.append(None)

                    # add face if there is not adjacent face
                    add_face(Face.LEFT, i-1 not in self.structure or not self.structure[i-1][j][k])
                    add_face(Face.RIGHT, i+1 not in self.structure or not self.structure[i+1][j][k])
                    add_face(Face.BACK, j-1 not in self.structure[i] or not self.structure[i][j-1][k])
                    add_face(Face.FRONT, j+1 not in self.structure[i] or not self.structure[i][j+1][k])
                    add_face(Face.BOTTOM, k-1 not in self.structure[i][j] or not self.structure[i][j][k-1])
                    add_face(Face.TOP, k+1 not in self.structure[i][j] or not self.structure[i][j][k+1])

                    self.__cube_face_map[i][j][k] = tuple(faces)

        self.__triangles.closePrimitive()
        self.mesh.addPrimitive(self.__triangles)

    def get_cube_colour(self, pos: IntPoint3) -> TColour:
        x, y, z = pos

        faces = self.__cube_face_map[x][y][z]
        for i in range(len(faces)):
            if faces[i] != None:
                self.__colour.setRow(faces[i] * 4)
                r, g, b, _ = self.__colour.getData4f()
                if self.artificial_lighting:
                    factor = self.__LIGHT_ATTENUATION_FACTOR(Face(i))
                    return (r / factor, g / factor, b / factor)
                else:
                    return (r, g, b)

        return self.default_colour

    def set_cube_colour(self, pos: IntPoint3, colour: Colour) -> None:
        x, y, z = pos

        faces = self.__cube_face_map[x][y][z]
        for i in range(len(faces)):
            if faces[i] != None:
                r, g, b = self.__face_colour(colour, Face(i))

                self.__colour.setRow(faces[i] * 4)
                self.__colour.addData4f(r, g, b, 1.0)
                self.__colour.addData4f(r, g, b, 1.0)
                self.__colour.addData4f(r, g, b, 1.0)
                self.__colour.addData4f(r, g, b, 1.0)

    def reset_cube_colour(self, pos: IntPoint3) -> None:
        self.set_cube_colour(pos, self.default_colour)

    @staticmethod
    def __attenuate_colour(colour: Colour, factor: Real) -> Colour:
        if isinstance(colour, Real):
            return colour * factor
        else:
            r, g, b = colour
            return (r * factor, g * factor, b * factor)

    @staticmethod
    def __LIGHT_ATTENUATION_FACTOR(face: Face) -> Real:
        switcher = {Face.LEFT: 0.9,
                    Face.RIGHT: 1.0,
                    Face.BACK: 0.85,
                    Face.FRONT: 0.6,
                    Face.BOTTOM: 0.7,
                    Face.TOP: 1.0
                    }
        return switcher.get(face.value)

    def __face_colour(self, colour: Colour, face: Face) -> TColour:
        c = (colour, colour, colour) if isinstance(colour, Real) else colour
        if self.artificial_lighting:
            return self.__attenuate_colour(c, self.__LIGHT_ATTENUATION_FACTOR(face))
        else:
            return c

    def __make_face(self, face: Face, pos: IntPoint3) -> None:
        r, g, b = self.__face_colour(self.default_colour, face)

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

            self.__colour.addData4f(r, g, b, 1.0)
            self.__colour.addData4f(r, g, b, 1.0)
            self.__colour.addData4f(r, g, b, 1.0)
            self.__colour.addData4f(r, g, b, 1.0)

            self.__texcoord.addData2f(0.0, 1.0)
            self.__texcoord.addData2f(0.0, 0.0)
            self.__texcoord.addData2f(1.0, 0.0)
            self.__texcoord.addData2f(1.0, 1.0)

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

    @property
    def geom_node(self) -> str:
        return 'geom_node'

    @geom_node.getter
    def geom_node(self) -> GeomNode:
        node = GeomNode(self.name)
        node.addGeom(self.mesh)
        return node

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
    def default_colour(self) -> TColour:
        c = self.__default_colour
        return (c, c, c) if isinstance(c, Real) else c

    @default_colour.setter
    def default_colour(self, value: Colour) -> None:
        old_r, old_g, old_b = self.default_colour
        self.__default_colour = value

        # update colour of cubes that have old clear colour
        for i in self.structure:
            for j in self.structure[i]:
                for k in self.structure[i][j]:
                    if not self.structure[i][j][k]:
                        continue
                    p = (i, j, k)

                    r, g, b = self.get_cube_colour(p)

                    def close(a, b):
                        return math.isclose(a, b, rel_tol=1e-3)

                    # only change cube colour if it closely resembles
                    # the previous default colour.
                    if close(old_r, r) and \
                       close(old_g, g) and \
                       close(old_b, b):
                        self.set_cube_colour(p, self.default_colour)
    
    def cube_visible(self, pos: IntPoint3) -> bool:
        x, y, z = pos
        for f in self.__cube_face_map[x][y][z]:
            if f != None:
                return True
        return False
