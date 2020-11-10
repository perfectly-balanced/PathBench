from panda3d.core import Texture, GeomNode
from panda3d.core import GeomVertexFormat, GeomVertexData
from panda3d.core import Geom, GeomTriangles, GeomVertexWriter, GeomVertexArrayData

from numbers import Real

from simulator.views.map.object.common import normalise

class SquareMesh():
    name: str
    mesh: Geom

    __vertex_data_format: GeomVertexFormat
    __vertex_data: GeomVertexData

    __triangles: GeomTriangles
    __triangle_data: GeomVertexArrayData

    __vertex: GeomVertexWriter
    __normal: GeomVertexWriter
    __texcoord: GeomVertexWriter

    def __init__(self, width: int = 1, height: int = 1, depth: Real = 0.1, name: str = 'SquareMesh') -> None:
        self.name = name

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

        def make(x1, y1, z1, x2, y2, z2, use_tex: bool = False) -> None:
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

            if use_tex:
                # UVs are flipped here for convenience
                self.__texcoord.addData2f(1.0, 1.0)
                self.__texcoord.addData2f(0.0, 1.0)
                self.__texcoord.addData2f(0.0, 0.0)
                self.__texcoord.addData2f(1.0, 0.0)
            else:  # wrap around colour (typically wireframe)
                self.__texcoord.addData2f(0.0, 0.0)
                self.__texcoord.addData2f(0.0, 0.0)
                self.__texcoord.addData2f(0.0, 0.0)
                self.__texcoord.addData2f(0.0, 0.0)

            vertex_id = self.__face_count * 4

            self.__triangles.addVertices(vertex_id, vertex_id + 1, vertex_id + 3)
            self.__triangles.addVertices(vertex_id + 1, vertex_id + 2, vertex_id + 3)

            self.__face_count += 1

        if depth == 0:
            make(0, 0, 0, width, height, 0)
        else:
            make(width, height, -depth, 0, height, 0)
            make(0, 0, -depth, width, 0, 0)
            make(width, 0, -depth, width, height, 0)
            make(0, height, -depth, 0, 0, 0)
            make(width, height, 0, 0, 0, 0, True)
            make(0, height, -depth, width, 0, -depth)

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
