from simulator.controllers.controller import Controller
from simulator.services.event_manager.events.event import Event

from direct.showbase.DirectObject import DirectObject

from maps import Maps
from simulator.services.debug import DebugLevel
from algorithms.classic.testing.a_star_testing import AStarTesting
from algorithms.classic.graph_based.a_star import AStar

class MainController(Controller, DirectObject):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        def test():
            mp = (Maps.grid_map_complex_obstacle2, True)
            algo = (AStar, AStarTesting, ([], {}))
            ani = (0.00001, 0)
            debug = DebugLevel.HIGH

            config = self._services.settings
            config.simulator_initial_map, config.simulator_grid_display = mp   #Optional[Union[str, Map]], bool
            config.simulator_algorithm_type, config.simulator_testing_type, config.simulator_algorithm_parameters = algo   #Optional[Type[Algorithm]],Optional[Type[BasicTesting]], Tuple[List, Dict]
            config.simulator_key_frame_speed, config.simulator_key_frame_skip = ani     #int, int
            config.simulator_write_debug_level = debug    #DebugLevel
            self._services.reset()

        self.accept("escape", lambda: self._services.ev_manager.post(QuitEvent()))
        self.accept("r", test)

    def destroy(self) -> None:
        self.ignore_all()