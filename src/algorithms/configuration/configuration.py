from typing import Tuple, Callable, Type, List, Optional, Dict, Any, Union

from algorithms.algorithm import Algorithm
from algorithms.basic_testing import BasicTesting
from algorithms.configuration.maps.map import Map
from algorithms.lstm.LSTM_tile_by_tile import BasicLSTMModule
from algorithms.lstm.LSTM_CAE_tile_by_tile import LSTMCAEModel
from algorithms.lstm.ML_model import MLModel
from simulator.services.debug import DebugLevel
from structures import Size
from maps import Maps 

#planner implementations
from algorithms.classic.graph_based.a_star import AStar
from algorithms.classic.graph_based.bug1 import Bug1
from algorithms.classic.graph_based.bug2 import Bug2
from algorithms.classic.graph_based.dijkstra import Dijkstra
from algorithms.classic.graph_based.potential_field import PotentialField
from algorithms.classic.sample_based.sprm import SPRM
from algorithms.classic.sample_based.rt import RT
from algorithms.classic.sample_based.rrt import RRT
from algorithms.classic.sample_based.rrt_star import RRT_Star
from algorithms.classic.sample_based.rrt_connect import RRT_Connect
#from algorithms.classic.sample_based.OMPLtesting import OMPL_RRT_Test
from algorithms.classic.graph_based.wavefront import Wavefront
from algorithms.lstm.LSTM_tile_by_tile import OnlineLSTM
#from algorithms.lstm.a_star_waypoint import WayPointNavigation
#from algorithms.lstm.combined_online_LSTM import CombinedOnlineLSTM

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
    #VIN
    simulator_vin_plot: bool
    simulator_vin_weights: int
    simulator_vin_imsize: int
    simulator_vin_l_h: int
    simulator_vin_l_q: int

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
    trainer: bool
    trainer_model: Type[MLModel]
    trainer_custom_config: Optional[Dict[str, Any]]
    trainer_pre_process_data_only: bool
    analyzer: bool
    load_simulator: bool
    clear_cache: bool

    num_dim: int

    def __init__(self) -> None:
        # Simulator settings
        self.simulator_grid_display = False
        self.simulator_initial_map = None #Maps.grid_map_one_obstacle1
        self.simulator_testing_type = None
        self.simulator_algorithm_type = None #AStar
        self.simulator_algorithm_parameters = ([], {})
        self.simulator_graphics = False
        self.simulator_key_frame_speed = 0
        self.simulator_key_frame_skip = 0
        self.simulator_write_debug_level = DebugLevel.NONE


        # Generator
        self.generator = True
        self.generator_labelling_atlases = ['house_100']
        self.generator_nr_of_examples = 100
        self.generator_gen_type = "house"
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
        self.generator_size = 28
        
        self.num_dim = 3
        
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