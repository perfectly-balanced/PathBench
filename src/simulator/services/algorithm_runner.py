import copy
from typing import Optional, Type, TYPE_CHECKING, List, Any, Tuple, Dict

from algorithms.configuration.configuration import Configuration
from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.maps.ros_map import RosMap
from simulator.services.service import Service
from structures import Point

if TYPE_CHECKING:
    from algorithms.algorithm import Algorithm
    from algorithms.configuration.maps.map import Map
    from simulator.services.services import Services
    from algorithms.basic_testing import BasicTesting


class AlgorithmRunner(Service):
    __testing_type: Optional[Type['BasicTesting']]
    __algorithm_type: Optional[Type['Algorithm']]
    __algorithm_parameters: Tuple[List, Dict]
    __map: Optional['Map']
    __algorithm: Optional['Algorithm']
    __buffered_services: 'Services'

    __pass_root_key_frame: 'AlgorithmRunner'

    def __init__(self, _services: 'Services') -> None:
        super().__init__(_services)

        self.__testing_type = self._services.settings.simulator_testing_type
        self.__algorithm_type = self._services.settings.simulator_algorithm_type
        self.__algorithm_parameters = self._services.settings.simulator_algorithm_parameters
        self.__map = None
        self.__algorithm = None
        self.__buffered_services = None
        self.__pass_root_key_frame = None
        
        self.map = copy.deepcopy(self._services.settings.simulator_initial_map)

    def reset_algorithm(self, refresh_map: bool = False) -> None:
        """
        Creates a new instance of the algorithms and setups the algorithms service
        """

        if refresh_map:
            if self.map is not None:
                self.map.reset()
            self.map = copy.deepcopy(self._services.settings.simulator_initial_map)
        elif self.map is None:
            return
        else:
            self.map.reset()

        testing: Optional[Type['BasicTesting']] = None
        if self.__testing_type is not None:
            testing = self.__testing_type(self._services)
        if self.__algorithm_type is not None:
            self.__algorithm = self.__algorithm_type(self._services, testing, *self.__algorithm_parameters[0],
                                                     **self.__algorithm_parameters[1])

    @property
    def instance(self) -> str:
        return 'instance'

    @instance.getter
    def instance(self) -> 'Algorithm':
        return self.__algorithm

    @property
    def map(self) -> str:
        return 'map'

    @map.getter
    def map(self) -> 'Map':
        return self.__map

    @map.setter
    def map(self, new_value: 'Map'):
        self.__map = new_value

        if self.__map and isinstance(self.__map, str):
            self.__map = self._services.resources.maps_dir.load(self.__map)

        if self.__map is not None:
            self.__map.services = self._services

    @property
    def testing_type(self) -> str:
        return 'testing_type'

    @testing_type.getter
    def testing_type(self) -> 'Type[BasicTesting]':
        return self.__testing_type

    @property
    def algorithm_type(self) -> str:
        return 'algorithm_type'

    @algorithm_type.getter
    def algorithm_type(self) -> 'Type[Algorithm]':
        return self.__algorithm_type

    @property
    def algorithm_parameters(self) -> str:
        return 'algorithm_parameters'

    @algorithm_parameters.getter
    def algorithm_parameters(self) -> Tuple[List, Dict]:
        return self.__algorithm_parameters

    def set_root(self):
        self.__pass_root_key_frame = self

    def get_new_runner(self, mp: 'Map', algorithm_type: Type['Algorithm'],
                       algorithm_parameters: Tuple[List, Dict] = None,
                       algorithm_testing: Optional[Type['BasicTesting']] = None,
                       with_animations: bool = False) -> 'AlgorithmRunner':
        if not algorithm_parameters:
            algorithm_parameters = [], {}

        from simulator.services.services import Services
        config = Configuration()
        config.simulator_initial_map = mp
        config.simulator_algorithm_type = algorithm_type
        config.simulator_testing_type = algorithm_testing
        config.simulator_algorithm_parameters = algorithm_parameters

        if with_animations and algorithm_testing:
            config.simulator_graphics = True
            config.simulator_key_frame_speed = self._services.settings.simulator_key_frame_speed
            config.simulator_key_frame_skip = self._services.settings.simulator_key_frame_skip

        s = Services(config)

        if with_animations and algorithm_testing:
            s.algorithm.__pass_root_key_frame = self.__pass_root_key_frame

        return s.algorithm

    def find_path(self, agent_pos: Point = None, goal_pos: Point = None) -> None:
        self.reset_algorithm()
        if self._services.settings.simulator_key_frame_speed > 0:
            self.instance.set_root_key_frame(self.__pass_root_key_frame)
        if agent_pos is not None:
            self.map.move_agent(agent_pos, True)
        if goal_pos is not None:
            self.map.move(self.map.goal, goal_pos, True)
        self.instance.find_path()
