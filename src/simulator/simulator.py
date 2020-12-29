from typing import Optional

from algorithms.basic_testing import BasicTesting
from simulator.controllers.main_controller import MainController
from simulator.controllers.map.map_controller import MapController
from simulator.controllers.gui.gui_controller import GuiController

from simulator.models.main_model import MainModel
from simulator.models.map_model import MapModel
from simulator.services.debug import DebugLevel
from simulator.services.services import Services
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.reinit_event import ReinitEvent
from simulator.views.main_view import MainView
from simulator.views.map.map_view import MapView
from simulator.views.gui.gui_view import GuiView
from structures import Size

"""
Implementation is done after https://github.com/wesleywerner/mvc-game-design
"""


class Simulator:
    """
    The main simulator class
    """
    __services: Services
    __main: MainModel
    __map: MapModel
    __main_controller: MainController
    __map_controller: MapController
    __gui_controller: GuiController
    __main_view: MainView
    __map_view: MapView
    __gui_view: GuiView

    def __init__(self, services: Services) -> None:
        # init services
        self.__services = services
        self.__services.ev_manager.register_listener(self)

        self.__main = None
        self.__map = None
        self.__main_controller = None
        self.__map_controller = None
        self.__gui_controller = None
        self.__main_view = None
        self.__map_view = None

    def start(self) -> Optional[BasicTesting]:
        """
        Starts the simulator
        :return The testing results if any
        """
        if self.__services.settings.simulator_graphics:
            return self.__start_with_graphics()
        else:
            return self.__start_without_graphics()

    def __try_setup_map_graphics(self) -> None:
        if self.__services.algorithm.instance is not None:
            if self.__map_controller is not None:
                self.__map_controller.destroy()
            if self.__map_view is not None:
                self.__map_view.destroy()
            self.__map = MapModel(self.__services)
            self.__map_view = MapView(self.__services, self.__map, self.__main_view)
            self.__map_controller = MapController(self.__map_view, self.__services, self.__map)

    def __start_with_graphics(self) -> None:
        """
        Starts simulator with graphics
        """
        # init models, views, controllers
        self.__main = MainModel(self.__services)

        # init views
        self.__main_view = MainView(self.__services, self.__main, None)
        self.__gui_view = GuiView(self.__services, None, self.__main_view)


        # init controllers
        self.__main_controller = MainController(self.__services, self.__main)
        self.__gui_controller = GuiController(self.__gui_view, self.__services,self.__main)

        self.__try_setup_map_graphics()
        self.__main.run()

    def __start_without_graphics(self) -> Optional[BasicTesting]:
        """
        Starts simulator without graphics
        :return: The test results
        """
        self.__services.algorithm.instance.find_path()
        return self.__services.algorithm.instance.testing

    def notify(self, event: Event) -> None:
        if isinstance(event, ReinitEvent):
            if self.__map:
                """
                self.__map.stop_algorithm()
                if self.__map.last_thread:
                    self.__map.last_thread.join()
                """
                self.__map.reset()
                self.__services.ev_manager.unregister_listener(self.__map)
                self.__services.ev_manager.unregister_tick_listener(self.__map)
            self.__try_setup_map_graphics()

    @property
    def services(self) -> Services:
        return self.__services
