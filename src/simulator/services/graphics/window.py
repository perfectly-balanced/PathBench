from typing import Optional
import os

from direct.showbase.ShowBase import ShowBase
from panda3d.core import WindowProperties, LVector2i

from structures import Size

class Window(ShowBase):
    def __init__(self, title: str) -> None:
        super().__init__()

        props = WindowProperties()
        props.set_title(title)
        self.win.request_properties(props)

    def quit(self) -> None:
        self.destroy()

    def update(self) -> None:
        self.taskMgr.step()

    @property
    def size(self) -> str:
        return 'size'

    @size.getter
    def size(self) -> Size:
        props = self.win.get_properties()
        return Size(*props.size)

    @size.setter
    def size(self, value: Size) -> None:
        props = WindowProperties()
        props.size = LVector2i(*value)
        self.win.request_properties(props)

    @property
    def fullscreen(self) -> str:
        return 'fullscreen'

    @fullscreen.getter
    def fullscreen(self) -> bool:
        props = self.win.get_properties()
        return props.fullscreen

    @fullscreen.setter
    def fullscreen(self, value: bool) -> None:
        props = WindowProperties()
        props.fullscreen = value
        props.size = LVector2i(self.pipe.getDisplayWidth(), self.pipe.getDisplayHeight())
        self.win.request_properties(props)
