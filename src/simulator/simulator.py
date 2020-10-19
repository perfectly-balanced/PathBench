from typing import Optional

from algorithms.basic_testing import BasicTesting
from simulator.controllers.main_controller import MainController
from simulator.controllers.map_controller import MapController
from simulator.models.main import Main
from simulator.models.map import Map
from simulator.services.debug import DebugLevel
from simulator.services.services import Services
from simulator.views.main_view import MainView
from simulator.views.map_view import MapView
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

    def __init__(self, services: Services) -> None:
        # init services
        self.__services = services

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
            return self.start_with_graphics()
        else:
            return self.start_without_graphics()

    # helper
    def start_with_graphics(self) -> None:
        """
        Starts simulator with graphics
        """

        if self.__services.settings.simulator_grid_display or self.__services.algorithm.map is None:
            self.__services.settings.simulator_window_size = Size(900, 900)
        else:
            self.__services.settings.simulator_window_size = self.__services.algorithm.map.size

        # init models
        self.__main = Main(self.__services)
        self.__map = Map(self.__services)

        # init views
        self.__main_view = MainView(self.__services, self.__main, None)
        self.__map_view = MapView(self.__services, self.__map, self.__main_view)

        # init controllers
        self.__main_controller = MainController(self.__services, self.__main)
        self.__map_controller = MapController(self.__map_view, self.__services, self.__map)

        self.__services.debug.write("", DebugLevel.BASIC, timestamp=False)
        self.display_help()
        self.__services.debug.write("", DebugLevel.LOW, timestamp=False)
        self.__main.run()

    # helper
    def start_without_graphics(self) -> Optional[BasicTesting]:
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
