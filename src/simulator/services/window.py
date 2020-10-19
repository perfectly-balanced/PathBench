from simulator.services.service import Service
from pandac.PandaModules import WindowProperties
from structures import Size

from typing import Optional

class RenderEngine(Service, ShowBase):
    __base_initialised: bool

    def __init__(self, *args, **kwargs):
        Service.__init__(*args, **kwargs)
        # delayed initialisation of ShowBase
        __base_initialised = False

    @property
    def initialised(self) -> bool:
        return self.__base_initialised

    def init(title: str, size: Optional[Size] = None) -> None:
        if not self.initialised:
            ShowBase.__init__(self)
            self.__base_initialised = True
        
        props = WindowProperties()
        props.set_title(title)
        if size is not None:
            props.set_size(size)
        self.win.request_properties(props)

    def quit(self) -> None:
        if self.initialised:
            self.destroy()

    def update(self) -> None:
        self.taskMgr.step()
