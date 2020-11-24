from panda3d.core import NodePath
from direct.gui.DirectGui import DirectFrame, DGG
from direct.showbase.ShowBase import ShowBase

from utility.compatibility import Final
from typing import Callable, List

class Window():
    __base: Final[ShowBase]
    __name: Final[str]
    __mouse1_press_callbacks: Final[List[Callable[[], None]]]

    __frame: Final[DirectFrame]
    __mouse_node: Final[NodePath]

    __zoom: float
    __visible: bool

    def __init__(self, base: ShowBase, name: str, mouse1_press_callbacks: List[Callable[[], None]], *args, **kwargs):
        self.__base = base
        self.__name = name
        self.__mouse1_press_callbacks = mouse1_press_callbacks

        self.__visible = True
        self.__zoom = 1 / 5

        if 'frameSize' not in kwargs:
            kwargs['frameSize'] = (-.8, .8, -1., 1.)

        self.__frame = DirectFrame(*args, parent=self.__base.aspect2d, **kwargs)
        self.__frame['state'] = DGG.NORMAL
        self.__frame.bind(DGG.B1PRESS, self.__start_drag)
        self.__frame.bind(DGG.B1RELEASE, self.__stop_drag)
        self.__frame.set_scale(self.zoom)

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
        for c in self.__mouse1_press_callbacks:
            c()
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

    def zoom_in(self):
        self.__zoom += 0.05
        self.frame.set_scale(self.__zoom)

    def zoom_out(self):
        self.__zoom -= 0.05
        self.frame.set_scale(self.__zoom)

    @property
    def zoom(self) -> float:
        return self.__zoom

    def toggle_visible(self):
        if self.__visible:
            self.frame.hide()
        else:
            self.frame.show()
            self.focus()
        self.__visible = not self.__visible

    @property
    def visible(self) -> bool:
        return self.__visible
