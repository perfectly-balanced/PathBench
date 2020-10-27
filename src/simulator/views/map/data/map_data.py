from panda3d.core import NodePath, TransparencyAttrib, LVecBase3f

from structures import Colour, TRANSPARENT

class MapData:
    __name: str
    __root: NodePath

    _bg_colour: Colour

    def __init__(self, parent: NodePath, name: str = "map"):
        self.__name = name

        self.__root = parent.attach_new_node(self.name)
        self.root.set_transparency(TransparencyAttrib.M_alpha)

        self._bg_colour = Colour(0, 0, 0.2, 1)

    @property
    def root(self) -> str:
        return 'root'

    @root.getter
    def root(self) -> NodePath:
        return self.__root
    
    @property
    def name(self) -> str:
        return 'name'

    @name.getter
    def name(self) -> NodePath:
        return self.__name

    @property
    def bg_colour(self) -> str:
        return 'bg_colour'

    @bg_colour.getter
    def bg_colour(self) -> Colour:
        return self.__bg_colour