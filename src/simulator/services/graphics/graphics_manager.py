from simulator.services.service import Service
from simulator.services.graphics.window import Window
from simulator.services.graphics.renderer import Renderer
from utility.compatibility import Final

_g_window = None

class GraphicsManager(Service):
    __renderer: Final[Renderer]

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        global _g_window
        if _g_window is None:
            _g_window = Window("PathBench")
        self.__renderer = Renderer(self.window.render)

    @property
    def window(self) -> Window:
        return _g_window

    @property
    def renderer(self) -> Renderer:
        return self.__renderer
