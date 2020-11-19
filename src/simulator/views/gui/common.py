from structures import Colour
from utility.compatibility import Final
from panda3d.core import *
from direct.gui.OnscreenImage import OnscreenImage
from direct.gui.DirectGui import *
from direct.showbase.ShowBase import ShowBase

WINDOW_BG_COLOUR: Final = Colour(0.5, 0.5, 0.5, 1.0)
WIDGET_BG_COLOUR: Final = Colour(0.3, 0.3, 0.3, 1.0)

class Window():
    __base: ShowBase
    __name: str

    __frame: DirectFrame
    __drag_start: Point2
    __drag_offset: Vec2

    def __init__(self, base: ShowBase, name: str, *args, **kwargs):
        self.__base = base
        self.__name = name

        if 'frameSize' not in kwargs:
            kwargs['frameSize'] = (-.8, .8, -1., 1.)

        self.__frame = DirectFrame(*args, parent=self.__base.pixel2d, **kwargs)
        self.__frame['state'] = DGG.NORMAL
        self.__frame.bind(DGG.B1PRESS, command=self.__start_drag)
        self.__frame.bind(DGG.B1RELEASE, command=self.__stop_drag)

        self.__drag_start = Point2()
        self.__drag_offset = Vec2()

    def __drag(self, task):
        if not self.__base.mouseWatcherNode.hasMouse():
            return task.cont

        pointer = self.__base.win.getPointer(0)
        pos = Point2(pointer.getX(), -pointer.getY())
        x, z = pos + self.__drag_offset
        self.__frame.setPos(x, 0., z)
        self.__drag_start = Point2(pos)

        return task.cont

    def __start_drag(self, *args):
        pointer = self.__base.win.getPointer(0)
        fx, _, fz = self.__frame.getPos()
        self.focus()

        self.__drag_start = Point2(pointer.getX(), -pointer.getY())
        self.__drag_offset = Point2(fx, fz) - self.__drag_start

        self.__base.taskMgr.add(self.__drag, 'drag_' + self.__name)

    def __stop_drag(self, *args):
        self.__base.taskMgr.remove('drag_' + self.__name)

    @property
    def frame(self) -> DirectFrame:
        return self.__frame

    def focus(self) -> None:
        self.__frame.detach_node()
        self.__frame.reparent_to(self.__base.pixel2d)