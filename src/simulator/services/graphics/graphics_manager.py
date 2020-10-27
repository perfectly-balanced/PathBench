from simulator.services.service import Service
from simulator.services.graphics.window import Window
from simulator.services.event_manager.events.graphics_loaded_event import GraphicsLoadedEvent

class GraphicsManager(Service):
    __initialised: bool
    __window: Window

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.__initialised = False
        self.__window = Window("PathBench")
        self._services.ev_manager.post(GraphicsLoadedEvent())

    @property
    def window(self) -> Window:
        return self.__window