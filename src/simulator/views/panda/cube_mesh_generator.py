from panda3d.core import Texture, GeomNode
from panda3d.core import GeomVertexFormat, GeomVertexData
from panda3d.core import Geom, GeomTriangles, GeomVertexWriter
from panda3d.core import LVector3, Vec3, Vec4, Point3
from numbers import Real

from .types import Colour

def normalise(*args):
    v = LVector3(*args)
    v.normalize()
    return v

FRONT_FACE_ATTENUATION = 0.6
BACK_FACE_ATTENUATION = 0.85
RIGHT_FACE_ATENUATION = 1.0
LEFT_FACE_ATTENUATION = 0.9
TOP_FACE_ATTENUATION = 1.0
BOTTOM_FACE_ATTENUATION = 0.7

class CubeMeshGenerator():
    name : str
    mesh : Geom

    def __init__(self, name: str = 'CubeMesh', artificial_lighting: bool = False, clear_colour: Colour = 1.0) -> None:
        self.name = name
        self.__finished = False
        self.__artificial_lighting = artificial_lighting
        self.__clear_colour = clear_colour
 
        self.__format = GeomVertexFormat.getV3n3c4t2()

        self.__vertex_data = GeomVertexData(name, self.__format, Geom.UHDynamic)
        
        self.mesh = Geom(self.__vertex_data)
        self.__triangles = GeomTriangles(Geom.UHDynamic)
        self.__triangle_data = self.__triangles.modifyVertices()
        
        self.__vertex = GeomVertexWriter(self.__vertex_data, 'vertex')
        self.__normal = GeomVertexWriter(self.__vertex_data, 'normal')
        self.__colour = GeomVertexWriter(self.__vertex_data, 'color')
        self.__texcoord = GeomVertexWriter(self.__vertex_data, 'texcoord')
        
        self.__face_count = 0
    
    def make_face(self, x1, y1, z1, x2, y2, z2, colour: Colour) -> None:
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

        r, g, b = (colour, colour, colour) if isinstance(colour, Real) else colour
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

    def get_cube_colour(self, pos: Point3) -> Colour:
        return 0 # todo
    
    def set_cube_colour(self, pos: Point3, colour: Colour) -> None:
        pass # todo

    def clear_cube_colour(self, pos: Point3) -> None:
        self.set_cube_colour(pos, self.clear_colour)

    @staticmethod
    def __attenuate_colour(colour: Colour, factor: Real):
        if isinstance(colour, Real):
            return colour * factor
        else:
            r, g, b = colour
            return (r * factor, g * factor, b * factor)
    
    def __apply_artificial_lighting(self, colour: Colour, factor: Real) -> Colour:
        return self.__attenuate_colour(colour, factor) if self.artificial_lighting else colour
        
    def make_front_face(self, x, y, z) -> None:
        c = self.__apply_artificial_lighting(self.clear_colour, FRONT_FACE_ATTENUATION)
        self.make_face(x + 1, y + 1, z - 1, x, y + 1, z, c)
    
    def make_back_face(self, x, y, z) -> None:
        c = self.__apply_artificial_lighting(self.clear_colour, BACK_FACE_ATTENUATION)
        self.make_face(x, y, z - 1, x + 1, y, z, c)
    
    def make_right_face(self, x, y, z) -> None:
        c = self.__apply_artificial_lighting(self.clear_colour, RIGHT_FACE_ATENUATION)
        self.make_face(x + 1, y, z - 1, x + 1, y + 1, z, c)
    
    def make_left_face(self, x, y, z) -> None:
        c = self.__apply_artificial_lighting(self.clear_colour, LEFT_FACE_ATTENUATION)
        self.make_face(x, y + 1, z - 1, x, y, z, c)
    
    def make_top_face(self, x, y, z) -> None:
        c = self.__apply_artificial_lighting(self.clear_colour, TOP_FACE_ATTENUATION)
        self.make_face(x + 1, y + 1, z, x, y, z, c)
    
    def make_bottom_face(self, x, y, z) -> None:
        c = self.__apply_artificial_lighting(self.clear_colour, BOTTOM_FACE_ATTENUATION)
        self.make_face(x, y + 1, z - 1, x + 1, y, z - 1, c)

    @property
    def geom_node(self) -> str:
        return 'geom_node'

    @geom_node.getter
    def geom_node(self) -> GeomNode:
        if not self.__finished:
            self.__triangles.closePrimitive()
            self.mesh.addPrimitive(self.__triangles)
            self.__finished = True
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
    def clear_colour(self) -> str:
        return 'clear_colour'

    @clear_colour.getter
    def clear_colour(self) -> bool:
        return self.__clear_colour
    
    @clear_colour.setter
    def clear_colour(self, value: Colour) -> None:
        self.__clear_colour = value
        # todo:
        # find all the cubes to recolour (ones that have colour same to old clear colour)
        # clear their colour
