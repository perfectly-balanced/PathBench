from simulator.services.service import Service
from simulator.services.graphics.window import Window
from simulator.services.event_manager.events.graphics_loaded_event import GraphicsLoadedEvent

class GraphicsManager(Service):
    __initialised: bool
    __window: Window

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.__initialised = False
        self.__window = None
    
    def init(self, *args, **kwargs) -> None:
        assert not self.__initialised

        self.__window = Window(*args, **kwargs)
        self._services.ev_manager.post(GraphicsLoadedEvent())

        self.__initialised = True

    @property
    def initialised(self) -> bool:
        return self.__initialised

    @property
    def window(self) -> Window:
        return self.__window