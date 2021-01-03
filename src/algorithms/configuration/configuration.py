from typing import Tuple, Callable, Type, List, Optional, Dict, Any, Union
import copy

from algorithms.algorithm import Algorithm
from algorithms.basic_testing import BasicTesting
from algorithms.configuration.maps.map import Map
from algorithms.lstm.LSTM_tile_by_tile import BasicLSTMModule
from algorithms.lstm.ML_model import MLModel
from simulator.services.debug import DebugLevel

from structures import Point

class Configuration:
    simulator_initial_map: Optional[Union[str, Map]]
    simulator_algorithm_type: Optional[Type[Algorithm]]
    simulator_testing_type: Optional[Type[BasicTesting]]
    simulator_algorithm_parameters: Tuple[List, Dict]
    simulator_graphics: bool
    simulator_key_frame_speed: int
    simulator_key_frame_skip: int
    simulator_write_debug_level: DebugLevel

    # VIN
    simulator_vin_plot: bool
    simulator_vin_weights: int
    simulator_vin_imsize: int
    simulator_vin_l_h: int
    simulator_vin_l_q: int

    # Generator
    generator: bool
    generator_labelling_atlases: List[Any]
    generator_gen_type: str
    generator_nr_of_examples: int
    generator_labelling_features: List[str]
    generator_labelling_labels: List[str]
    generator_single_labelling_features: List[str]
    generator_single_labelling_labels: List[str]
    generator_modify: Callable[[], Tuple[str, Callable[[Map], Map]]]
    generator_show_gen_sample: bool
    generator_house_expo: bool
    generator_size: int
    generator_obstacle_fill_min: float
    generator_obstacle_fill_max: float
    generator_min_room_size: int
    generator_max_room_size: int

    # Trainer
    trainer: bool
    trainer_model: Type[MLModel]
    trainer_custom_config: Optional[Dict[str, Any]]
    trainer_pre_process_data_only: bool

    # Visualiser
    visualiser_simulator_config: bool

    # Misc
    analyzer: bool
    algorithms: Dict[str, Tuple[Type[Algorithm], Type[BasicTesting], Tuple[List[Any], Dict[str, Any]]]]
    get_agent_position: Callable[[], Point] 
    load_simulator: bool
    clear_cache: bool
    num_dim: int
    map_name: Optional[str]
    algorithm_name: Optional[str]

    def __init__(self) -> None:
        # Simulator settings
        self.simulator_initial_map = None
        self.simulator_testing_type = None
        self.simulator_algorithm_type = None
        self.simulator_algorithm_parameters = ([], {})
        self.simulator_graphics = False
        self.simulator_key_frame_speed = 0
        self.simulator_key_frame_skip = 0
        self.simulator_write_debug_level = DebugLevel.LOW

        # Generator
        self.generator = False
        self.generator_labelling_atlases = ['block_map_100']
        self.generator_nr_of_examples = 100
        self.generator_gen_type = "block_map"
        self.generator_labelling_features = ["distance_to_goal_normalized",
                                             "raycast_8_normalized",
                                             "direction_to_goal_normalized",
                                             "agent_goal_angle"]
        self.generator_labelling_labels = ["next_position_index"]
        self.generator_single_labelling_features = []
        self.generator_single_labelling_labels = []
        self.generator_aug_labelling_features = []
        self.generator_aug_labelling_labels = []
        self.generator_aug_single_labelling_features = []
        self.generator_aug_single_labelling_labels = []
        self.generator_modify = None
        self.generator_show_gen_sample = False
        self.generator_house_expo = False
        self.generator_obstacle_fill_min = 0.1
        self.generator_obstacle_fill_max = 0.3
        self.generator_min_room_size = 3
        self.generator_max_room_size = 16
        self.generator_size = 64

        self.num_dim = 2

        # Trainer
        self.trainer = False
        self.trainer_model = BasicLSTMModule
        self.trainer_custom_config = None
        self.trainer_pre_process_data_only = False
        self.trainer_bypass_and_replace_pre_processed_cache = False

        # Analyzer
        self.analyzer = False

        # Common
        from algorithms.algorithm_manager import AlgorithmManager
        self.algorithms = copy.deepcopy(AlgorithmManager.builtins)
        
        # Method that returns an externally determined agent position
        # When not None, this prevents any changes by the UI. This method
        # will be called at every frame when algorithm isn't running and
        # no post-run algorithm graphics is shown (e.g. after moving the
        # goal position or immediately before restarting the algorithm).
        self.get_agent_position = None

        from maps.map_manager import MapManager
        self.maps = copy.deepcopy(MapManager.builtins)

        self.map_name = None
        self.algorithm_name = None

        # Simulator
        self.load_simulator = False

        # Cache
        self.clear_cache = False

        # Visualiser
        self.visualiser_simulator_config = True
