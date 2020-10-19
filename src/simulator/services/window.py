from simulator.services.service import Service
from structures import Size

from typing import Optional

from direct.showbase.ShowBase import ShowBase
from panda3d.core import WindowProperties, LVector2i

class Window(Service, ShowBase):
    __base_initialised: bool

    def __init__(self, *args, **kwargs):
        Service.__init__(self, *args, **kwargs)
        # delayed initialisation of ShowBase
        self.__base_initialised = False

    @property
    def initialised(self) -> bool:
        return self.__base_initialised

    def init(self, title: str, size: Optional[Size] = None) -> None:
        if not self.initialised:
            ShowBase.__init__(self)
            self.__base_initialised = True
        
        props = WindowProperties()
        props.set_title(title)
        if size is not None:
            props.set_size(LVector2i(*size))
        self.win.request_properties(props)

    def quit(self) -> None:
        if self.initialised:
            self.destroy()

    def update(self) -> None:
        self.taskMgr.step()
