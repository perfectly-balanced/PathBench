from simulator.controllers.controller import Controller
from simulator.views.gui.gui_view import GuiView
from simulator.services.event_manager.events.toggle_debug_overlay_event import ToggleDebugOverlayEvent
from simulator.services.event_manager.events.toggle_view_event import ToggleViewEvent
from simulator.services.event_manager.events.toggle_simulator_config_event import ToggleSimulatorConfigEvent

from direct.showbase.DirectObject import DirectObject


class GuiController(Controller, DirectObject):

    def __init__(self, gui_view: GuiView, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.accept("c", lambda: self._services.ev_manager.post(ToggleSimulatorConfigEvent()))
        self.accept("v", lambda: self._services.ev_manager.post(ToggleViewEvent()))
        self.accept("i", lambda: self._services.ev_manager.post(ToggleDebugOverlayEvent()))
