from typing import Optional

from algorithms.basic_testing import BasicTesting
from simulator.controllers.main_controller import MainController
from simulator.controllers.map.map_controller import MapController
from simulator.models.main import Main
from simulator.models.map import Map
from simulator.services.debug import DebugLevel
from simulator.services.services import Services
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.reset_event import ResetEvent
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
    __main: Main
    __map: Map
    __main_controller: MainController
    __map_controller: MapController
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
            self.__map = Map(self.__services)
            self.__map_view = MapView(self.__services, self.__map, self.__main_view)
            self.__map_controller = MapController(self.__map_view, self.__services, self.__map)

    def __start_with_graphics(self) -> None:
        """
        Starts simulator with graphics
        """
        # init models, views, controllers
        self.__main = Main(self.__services)

        # init views
        self.__main_view = MainView(self.__services, self.__main, None)
        self.__gui_view = GuiView(self.__services, None, self.__main_view)

        # init controllers
        self.__main_controller = MainController(self.__services, self.__main)

        self.__try_setup_map_graphics()

        self.__services.debug.write("", DebugLevel.BASIC, timestamp=False)
        self.display_help()
        self.__services.debug.write("", DebugLevel.LOW, timestamp=False)
        self.__main.run()

    def __start_without_graphics(self) -> Optional[BasicTesting]:
        """
        Starts simulator without graphics
        :return: The test results
        """
        self.__services.algorithm.instance.find_path()
        return self.__services.algorithm.instance.testing

    def display_help(self) -> None:
        """
        Displays the help panel at init
        """
        self.__services.debug.write(
            """Important runtime commands:
    * escape - exit the simulator
    * c - find the path between the agent and goal
    * mouse click - moves agent to mouse location 
    * mouse right click - moves goal to mouse location
    
Additional runtime commands:    
    * s - stop trace animation (animations required)
    * r - resume trace animation  (animations required)
    * mouse hover - displays hovered cell coordinates (debug level >= Medium)
    * p - take screenshot (the screenshot is placed in resources directory)
    * up arrow - moves agent up (depends on agent speed) 
    * left arrow - moves agent left (depends on agent speed) 
    * down arrow - moves agent down (depends on agent speed) 
    * right arrow - moves agent right (depends on agent speed)
    * m - toggle map between Sparse and Dense
""", DebugLevel.BASIC, timestamp=False)

    def notify(self, event: Event) -> None:
        if isinstance(event, ResetEvent):
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
