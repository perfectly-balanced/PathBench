from simulator.services.service import Service
from simulator.services.graphics.window import Window

class GraphicsManager(Service):
    __initialised: bool
    __window: Window

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.__initialised = False
        self.__window = Window("PathBench")

    @property
    def window(self) -> Window:
        return self.__window
