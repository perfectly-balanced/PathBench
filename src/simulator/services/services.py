from typing import TYPE_CHECKING, Optional

from simulator.services.debug import Debug
from simulator.services.event_manager.event_manager import EventManager
from simulator.services.event_manager.events.reinit_event import ReinitEvent
from simulator.services.event_manager.events.state_initialising_event import StateInitialisingEvent
from simulator.services.event_manager.events.state_initialised_event import StateInitialisedEvent

if TYPE_CHECKING:
    from algorithms.configuration.configuration import Configuration
    from simulator.services.graphics.graphics_manager import GraphicsManager
    from simulator.services.algorithm_runner import AlgorithmRunner
    from simulator.services.resources.resources import Resources
    from simulator.services.torch import Torch

_g_persistent_state = None

class Services:
    """
    This class is a bag of services available throughout all simulator classes and algorithms
    """
    __settings: 'Configuration'
    __debug: Debug
    __ev_manager: EventManager
    __state: 'PersistentState'
    __algorithm_runner: 'AlgorithmRunner'
    __graphics: Optional['GraphicsManager']
    __resources_dir: 'Resources'
    __torch: 'Torch'

    def __init__(self, config: 'Configuration') -> None:
        self.__settings = None
        self.__debug = None
        self.__ev_manager = None
        self.__state = None
        self.__graphics = None
        self.__algorithm_runner = None
        self.__resources_dir = None
        self.__torch = None
        self.__setup(config)

    def __setup(self, config: 'Configuration') -> None:
        from simulator.services.graphics.graphics_manager import GraphicsManager
        from simulator.services.algorithm_runner import AlgorithmRunner
        from simulator.services.resources.resources import Resources
        from simulator.services.persistent_state.persistent_state import PersistentState
        from simulator.services.torch import Torch

        from simulator.services.persistent_state.persistent_state_views import PersistentStateViews
        from simulator.views.gui.simulator_config_state import SimulatorConfigState

        self.__settings = config
        self.__debug = Debug(self)
        self.__ev_manager = EventManager(self)

        # persistent state is shared across all active services
        global _g_persistent_state
        if _g_persistent_state is None:
            _g_persistent_state = PersistentState(self, types=[(PersistentStateViews, 'views'), (SimulatorConfigState, None)])
            self.__state = _g_persistent_state
        else:
            self.__state = _g_persistent_state
            _g_persistent_state.add_services(self)

        self.__resources_dir = Resources(self)
        self.__torch = Torch(self)
        self.__algorithm_runner = AlgorithmRunner(self)
        self.__algorithm_runner.reset_algorithm()

        if self.__settings.simulator_graphics:
            self.__graphics = GraphicsManager(self)

    def reinit(self, refresh_map: bool = False) -> None:
        self.__ev_manager.broadcast(StateInitialisingEvent())

        if self.graphics is not None:
            self.graphics.force_render_frame()
        
        from simulator.services.algorithm_runner import AlgorithmRunner
        self.__algorithm_runner = AlgorithmRunner(self)
        self.__algorithm_runner.reset_algorithm(refresh_map)
        self.ev_manager.post(ReinitEvent())

    @property
    def state(self) -> str:
        return 'state'

    @state.getter
    def state(self) -> 'PersistentState':
        return self.__state

    @property
    def debug(self) -> str:
        return 'debug'

    @debug.getter
    def debug(self) -> 'Debug':
        return self.__debug

    @property
    def ev_manager(self) -> str:
        return 'ev_manager'

    @ev_manager.getter
    def ev_manager(self) -> 'EventManager':
        return self.__ev_manager

    @property
    def graphics(self) -> str:
        return 'graphics'

    @graphics.getter
    def graphics(self) -> 'GraphicsManager':
        return self.__graphics

    @property
    def settings(self) -> str:
        return 'settings'

    @settings.getter
    def settings(self) -> 'Configuration':
        return self.__settings

    @property
    def algorithm(self) -> str:
        return 'algorithms'

    @algorithm.getter
    def algorithm(self) -> 'AlgorithmRunner':
        return self.__algorithm_runner

    @property
    def resources(self) -> str:
        return 'resources'

    @resources.getter
    def resources(self) -> 'Resources':
        return self.__resources_dir

    @property
    def torch(self) -> str:
        return 'torch'

    @torch.getter
    def torch(self) -> 'Torch':
        return self.__torch


class GenericServices:
    __services: Services

    def __init__(self):
        from algorithms.configuration.configuration import Configuration
        self.__services = Services(Configuration())

    @property
    def resources(self) -> str:
        return 'resources'

    @resources.getter
    def resources(self) -> 'Resources':
        return self.__services.resources
