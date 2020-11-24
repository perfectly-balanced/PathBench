from panda3d.core import NodePath
from direct.gui.DirectGui import DirectFrame, DGG
from direct.showbase.ShowBase import ShowBase

from utility.compatibility import Final

class Window():
    __base: Final[ShowBase]
    __name: Final[str]

    __frame: Final[DirectFrame]
    __mouse_node: Final[NodePath]

    def __init__(self, base: ShowBase, name: str, *args, **kwargs):
        self.__base = base
        self.__name = name

        if 'frameSize' not in kwargs:
            kwargs['frameSize'] = (-.8, .8, -1., 1.)

        self.__frame = DirectFrame(*args, parent=self.__base.aspect2d, **kwargs)
        self.__frame['state'] = DGG.NORMAL
        self.__frame.bind(DGG.B1PRESS, self.__start_drag)
        self.__frame.bind(DGG.B1RELEASE, self.__stop_drag)

        # set sort value to ensure when window is reparented to mouse node it is on top
        # of everything else.
        self.__mouse_node = self.__base.aspect2d.attach_new_node(name + '_mouse_node', sort=100000000)
        self.__base.taskMgr.add(self.__mouse_node_task, name + '_mouse_node_task')

    def __mouse_node_task(self, task):
        if self.__base.mouseWatcherNode.hasMouse():
            x = self.__base.mouseWatcherNode.getMouseX()
            y = self.__base.mouseWatcherNode.getMouseY()
            self.__mouse_node.setPos(self.__base.render2d, x, 0, y)
        return task.cont

    def __start_drag(self, *discarded):
        self.__frame.wrt_reparent_to(self.__mouse_node)

    def __stop_drag(self, *discarded):
        if self.__frame.get_parent() != self.__base.aspect2d:
            self.__frame.wrt_reparent_to(self.__base.aspect2d)

    @property
    def frame(self) -> DirectFrame:
        return self.__frame

    def focus(self) -> None:
        self.__frame.detach_node()
        self.__frame.reparent_to(self.__base.aspect2d)
