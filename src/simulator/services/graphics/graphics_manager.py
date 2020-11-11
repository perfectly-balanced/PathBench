from simulator.services.service import Service
from simulator.services.graphics.window import Window
from simulator.services.graphics.renderer import Renderer
from utility.compatibility import Final

class GraphicsManager(Service):
    __window: Final[Window]
    __renderer: Final[Renderer]

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.__window = Window("PathBench")
        self.__renderer = Renderer(self.window.render)

    @property
    def window(self) -> Window:
        return self.__window

    @property
    def renderer(self) -> Renderer:
        return self.__renderer
