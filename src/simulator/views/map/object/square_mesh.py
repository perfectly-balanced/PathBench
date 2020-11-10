from panda3d.core import Texture, GeomNode
from panda3d.core import GeomVertexFormat, GeomVertexData
from panda3d.core import Geom, GeomTriangles, GeomVertexWriter, GeomVertexArrayData
from panda3d.core import LVector3

def normalise(*args):
    v = LVector3(*args)
    v.normalize()
    return v

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

    def __init__(self, name: str = 'SquareMesh') -> None:
        self.name = name

        self.__vertex_data_format = GeomVertexFormat. getV3n3t2()
        self.__vertex_data = GeomVertexData(name, self.__vertex_data_format, Geom.UHStatic)
        self.__vertex_data.set_num_rows(4)

        self.mesh = Geom(self.__vertex_data)
        self.__triangles = GeomTriangles(Geom.UHDynamic)
        self.__triangle_data = self.__triangles.modifyVertices()

        self.__vertex = GeomVertexWriter(self.__vertex_data, 'vertex')
        self.__normal = GeomVertexWriter(self.__vertex_data, 'normal')
        self.__texcoord = GeomVertexWriter(self.__vertex_data, 'texcoord')

        x1, y1, z1, x2, y2, z2 = -0.5, -0.5, 0, 0.5, 0.5, 0

        self.__vertex.addData3f(x1, y1, z1)
        self.__vertex.addData3f(x2, y1, z1)
        self.__vertex.addData3f(x2, y2, z2)
        self.__vertex.addData3f(x1, y2, z2)

        self.__normal.addData3(normalise(2 * x1 - 1, 2 * y1 - 1, 2 * z1 - 1))
        self.__normal.addData3(normalise(2 * x2 - 1, 2 * y1 - 1, 2 * z1 - 1))
        self.__normal.addData3(normalise(2 * x2 - 1, 2 * y2 - 1, 2 * z2 - 1))
        self.__normal.addData3(normalise(2 * x1 - 1, 2 * y2 - 1, 2 * z2 - 1))

        self.__texcoord.addData2f(0.0, 1.0)
        self.__texcoord.addData2f(0.0, 0.0)
        self.__texcoord.addData2f(1.0, 0.0)
        self.__texcoord.addData2f(1.0, 1.0)

        self.__triangles.addVertices(0, 1, 3)
        self.__triangles.addVertices(1, 2, 3)

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