from simulator.controllers.controller import Controller
from simulator.views.gui.gui_view import GuiView
from direct.showbase.DirectObject import DirectObject
from simulator.services.event_manager.events.toggle_view_event import ToggleViewEvent
from simulator.services.event_manager.events.toggle_simulator_config_event import ToggleSimulatorConfigEvent



class GuiController(Controller, DirectObject):

    def __init__(self, gui_view: GuiView, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.accept("c", lambda: self._services.ev_manager.post(ToggleSimulatorConfigEvent()))
        self.accept("v", lambda: self._services.ev_manager.post(ToggleViewEvent()))

