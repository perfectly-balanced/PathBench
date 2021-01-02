from simulator.views.view import View
from simulator.models.model import Model
from simulator.services.services import Services
from simulator.views.gui.view_editor import ViewEditor
from simulator.views.gui.simulator_config import SimulatorConfig
from simulator.views.gui.debug_overlay import DebugOverlay

from typing import Optional, List, Callable


class GuiView(View):
    __vs: ViewEditor
    __sc: SimulatorConfig
    __mouse1_press_callback: List[Callable[[], None]]

    def __init__(self, services: Services, model: Optional[Model], root_view: Optional[View]) -> None:
        super().__init__(services, model, root_view)

        self.__mouse1_press_callback = []
        self.__vs = ViewEditor(self._services, self.__mouse1_press_callback)
        self.__do = DebugOverlay(self._services)

        if self._services.settings.visualiser_simulator_config:
            self.__sc = SimulatorConfig(self._services, self.__mouse1_press_callback)
