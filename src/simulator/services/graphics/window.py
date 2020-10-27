from typing import Optional

from direct.showbase.ShowBase import ShowBase
from panda3d.core import WindowProperties, LVector2i

from structures import Size

class Window(ShowBase):
    def __init__(self, title: str, size: Optional[Size] = None) -> None:
        super().__init__()

        props = WindowProperties()
        props.set_title(title)
        if size is not None:
            props.set_size(LVector2i(*size))
        self.win.request_properties(props)

    def quit(self) -> None:
        self.destroy()

    def update(self) -> None:
        self.taskMgr.step()
