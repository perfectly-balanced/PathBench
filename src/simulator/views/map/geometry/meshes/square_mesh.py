from panda3d.core import Texture, GeomNode
from panda3d.core import GeomVertexFormat, GeomVertexData
from panda3d.core import Geom, GeomTriangles, GeomVertexWriter, GeomVertexArrayData

from numbers import Real
from typing import Tuple

from simulator.views.map.geometry.meshes.common import normalise
from utility.compatibility import Final

class SquareMesh():
    name: Final[str]
    mesh: Final[Geom]
    depth: Final[float]

    __vertex_data_format: GeomVertexFormat
    __vertex_data: GeomVertexData

    __triangles: GeomTriangles
    __triangle_data: GeomVertexArrayData

    __vertex: GeomVertexWriter
    __normal: GeomVertexWriter
    __texcoord: GeomVertexWriter

    def __init__(self, width: int = 1, height: int = 1, depth: Real = 0.1, name: str = 'SquareMesh') -> None:
        self.name = name
        self.depth = depth

        self.__vertex_data_format = GeomVertexFormat. getV3n3t2()
        self.__vertex_data = GeomVertexData(name, self.__vertex_data_format, Geom.UHStatic)
        self.__vertex_data.set_num_rows(4)

        self.mesh = Geom(self.__vertex_data)
        self.__triangles = GeomTriangles(Geom.UHStatic)
        self.__triangle_data = self.__triangles.modifyVertices()

        self.__vertex = GeomVertexWriter(self.__vertex_data, 'vertex')
        self.__normal = GeomVertexWriter(self.__vertex_data, 'normal')
        self.__texcoord = GeomVertexWriter(self.__vertex_data, 'texcoord')

        self.__face_count = 0

        def make(x1, y1, z1, x2, y2, z2, tex) -> None:
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

            for data in tex:
                self.__texcoord.addData2f(*data)

            vertex_id = self.__face_count * 4

            self.__triangles.addVertices(vertex_id, vertex_id + 1, vertex_id + 3)
            self.__triangles.addVertices(vertex_id + 1, vertex_id + 2, vertex_id + 3)

            self.__face_count += 1

        if self.depth == 0:
            make(0, 0, 0, width, height, 0, [(1.0, 1.0), (0.0, 1.0), (0.0, 0.0), (1.0, 0.0)])
        else:
            make(0, height, -self.depth, 0, 0, 0, ((0.0, 1.0), (0.0, 0.0), (0.0, 0.0), (0.0, 1.0))) # SIDE
            make(0, 0, -self.depth, width, 0, 0, ((0.0, 0.0), (1.0, 0.0), (1.0, 0.0), (0.0, 0.0)))  # SIDE
            make(width, height, -self.depth, 0, height, 0, ((1.0, 1.0), (0.0, 1.0), (0.0, 1.0), (1.0, 1.0))) # SIDE
            make(width, 0, -self.depth, width, height, 0, ((1.0, 0.0), (1.0, 1.0), (1.0, 1.0), (1.0, 0.0))) # SIDE
            make(width, height, 0, 0, 0, 0, ((1.0, 1.0), (0.0, 1.0), (0.0, 0.0), (1.0, 0.0))) # TOP
            make(0, height, -self.depth, width, 0, -self.depth, ((0.0, 1.0), (1.0, 1.0), (1.0, 0.0), (0.0, 0.0))) # BOTTOM

        self.__triangles.close_primitive()
        self.mesh.add_primitive(self.__triangles)

    @property
    def geom_node(self) -> str:
        return 'geom_node'

    @geom_node.getter
    def geom_node(self) -> GeomNode:
        node = GeomNode(self.name)
        node.addGeom(self.mesh)
        return node
