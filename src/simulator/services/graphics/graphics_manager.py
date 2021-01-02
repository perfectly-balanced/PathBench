from simulator.services.service import Service
from simulator.services.graphics.window import Window
from simulator.services.graphics.renderer import Renderer
from utility.compatibility import Final

from typing import Callable

from direct.task import Task

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

    def schedule(self, payload: Callable[[], None], skip_next_frame: bool = True) -> Task:
        skip_frame: bool = skip_next_frame

        def exec_payload(task):
            nonlocal skip_frame

            if skip_frame:
                skip_frame = False
                return task.cont

            payload()
            return task.done

        return self.window.taskMgr.do_method_later(0, exec_payload, 'scheduled_payload')

    def force_render_frame(self) -> None:
        self.window.graphicsEngine.render_frame()
