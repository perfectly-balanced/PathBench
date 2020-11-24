from simulator.views.view import View
from simulator.models.model import Model
from simulator.services.services import Services
from simulator.views.gui.view_editor import ViewEditor
from simulator.views.gui.simulator_config import SimulatorConfig

from typing import Optional

class GuiView(View):
    __vs: ViewEditor
    __sc: SimulatorConfig

    def __init__(self, services: Services, model: Optional[Model], root_view: Optional[View]) -> None:
        super().__init__(services, model, root_view)

        self.__vs = ViewEditor(self._services)
        self.__sc = SimulatorConfig(self._services)
