from typing import Tuple, Callable, Type, List, Optional, Dict, Any, Union

from algorithms.algorithm import Algorithm
from algorithms.basic_testing import BasicTesting
from algorithms.configuration.maps.map import Map
from algorithms.lstm.LSTM_tile_by_tile import BasicLSTMModule
from algorithms.lstm.ML_model import MLModel
from simulator.services.debug import DebugLevel
from structures import Size


class Configuration:
    simulator_grid_display: bool
    simulator_initial_map: Optional[Union[str, Map]]
    simulator_algorithm_type: Optional[Type[Algorithm]]
    simulator_testing_type: Optional[Type[BasicTesting]]
    simulator_algorithm_parameters: Tuple[List, Dict]
    simulator_graphics: bool
    simulator_key_frame_speed: int
    simulator_key_frame_skip: int
    simulator_write_debug_level: DebugLevel
    simulator_window_size: Size
    generator: bool
    generator_labelling_atlases: List[Any]
    generator_gen_type: str
    generator_nr_of_examples: int
    generator_labelling_features: List[str]
    generator_labelling_labels: List[str]
    generator_single_labelling_features: List[str]
    generator_single_labelling_labels: List[str]
    generator_modify: Callable[[], Tuple[str, Callable[[Map], Map]]]
    trainer: bool
    trainer_model: Type[MLModel]
    trainer_custom_config: Optional[Dict[str, Any]]
    trainer_pre_process_data_only: bool
    analyzer: bool
    load_simulator: bool
    clear_cache: bool

    def __init__(self) -> None:
        # Simulator settings
        self.simulator_grid_display = True
        self.simulator_initial_map = None
        self.simulator_testing_type = None
        self.simulator_algorithm_type = None
        self.simulator_algorithm_parameters = [], {}
        self.simulator_graphics = False
        self.simulator_key_frame_speed = 0
        self.simulator_key_frame_skip = 0
        self.simulator_write_debug_level = DebugLevel.NONE
        self.simulator_window_size = Size(0, 0)

        # Generator
        self.generator = False
        self.generator_labelling_atlases = []
        self.generator_nr_of_examples = 0
        self.generator_gen_type = ""
        self.generator_labelling_features = []
        self.generator_labelling_labels = []
        self.generator_single_labelling_features = []
        self.generator_single_labelling_labels = []
        self.generator_aug_labelling_features = []
        self.generator_aug_labelling_labels = []
        self.generator_aug_single_labelling_features = []
        self.generator_aug_single_labelling_labels = []
        self.generator_modify = None

        # Trainer
        self.trainer = False
        self.trainer_model = BasicLSTMModule
        self.trainer_custom_config = None
        self.trainer_pre_process_data_only = False
        self.trainer_bypass_and_replace_pre_processed_cache = False

        # Custom behaviour settings
        self.analyzer = False

        # Simulator
        self.load_simulator = False

        # Cache
        self.clear_cache = False
