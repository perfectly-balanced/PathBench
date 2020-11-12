from typing import Dict, Optional, Any, Tuple, List

import torch
from torch import nn
from torch.nn import LSTM
from torch.nn.utils.rnn import pack_padded_sequence, pad_packed_sequence, PackedSequence
from torch.utils.data import Dataset, TensorDataset

from algorithms.algorithm import Algorithm
from algorithms.basic_testing import BasicTesting
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.maps.map import Map
from algorithms.lstm.ML_model import MLModel, SingleTensorDataset, PackedDataset
from algorithms.lstm.map_processing import MapProcessing
from simulator.services.services import Services
from simulator.views.map.display.entities_map_display import EntitiesMapDisplay
from simulator.views.map.display.online_lstm_map_display import OnlineLSTMMapDisplay
from structures import Point


# 1. torch.nn.utils.rnn.pad_packed_sequence(out) - Done
# 2. try to do batch second - No improvement
# 3. add none action - Not needed
# 4. add previous action - Quite hard to implement with packed
class BasicLSTMModule(MLModel):
    _hidden_state: Tuple[torch.Tensor, torch.Tensor]
    _lstm_layer: LSTM

    def __init__(self, services: Services, config: Dict[str, any]):
        super().__init__(services, config)

        self._hidden_state = None
        self._normalisation_layer1 = nn.BatchNorm1d(num_features=self.config["lstm_input_size"])
        self._lstm_layer = nn.LSTM(self.config["lstm_input_size"], self.config["lstm_output_size"],
                                   num_layers=self.config["num_layers"], batch_first=True)
        self._normalisation_layer2 = nn.BatchNorm1d(num_features=self.config["lstm_output_size"])
        self._fc = nn.Linear(in_features=self.config["lstm_output_size"], out_features=self.config["lstm_output_size"])

    def init_hidden(self, batch_size=1) -> None:
        self._hidden_state = (
            torch.zeros((self.config["num_layers"], batch_size, self.config["lstm_output_size"])).to(
                self._services.torch.device),
            torch.zeros((self.config["num_layers"], batch_size, self.config["lstm_output_size"])).to(
                self._services.torch.device)
        )

    def init_running_algorithm(self, mp: Map) -> None:
        self.init_hidden()

    def pre_process_data(self) -> Tuple[Dataset, Dataset]:
        data_features, data_labels = super().pre_process_data()
        data_labels.data = data_labels.data.long()
        return data_features, data_labels

    def _prepare_data_before_forward(self, inputs: Tuple[torch.Tensor, torch.Tensor],
                                     labels: Tuple[torch.Tensor, torch.Tensor]) -> \
            Tuple[torch.Tensor, torch.Tensor, torch.Tensor, List[int]]:
        x, x_len = inputs
        y, _ = labels
        self.init_hidden(len(x))

        perm = self.get_sort_by_lengths_indices(x_len)
        y_sorted, _ = self.pack_data(y, x_len, perm)

        x = x.to(self._services.torch.device)
        x_len = x_len.to(self._services.torch.device)
        y_sorted = y_sorted.data.view(-1).to(self._services.torch.device)

        return x, x_len, y_sorted, perm

    def batch_start(self, inputs: Tuple[torch.Tensor, torch.Tensor], labels: Tuple[torch.Tensor, torch.Tensor]) -> \
            Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        x, x_len, y_sorted, perm = self._prepare_data_before_forward(inputs, labels)
        out = self.forward(x, x_len, perm).view(-1, 8)
        l: torch.Tensor = self.config["loss"](out, y_sorted)
        return l, out, y_sorted

    @staticmethod
    def get_sort_by_lengths_indices(lengths: torch.Tensor) -> List[int]:
        perm = list(range(len(lengths)))
        perm = list(map(lambda el: el[0], sorted(zip(perm, lengths), key=lambda el: el[1], reverse=True)))
        return perm

    @staticmethod
    def pack_data(seq: torch.Tensor, lengths: torch.Tensor, perm: List[int] = None) -> Tuple[PackedSequence, List[int]]:
        if not perm:
            perm = BasicLSTMModule.get_sort_by_lengths_indices(lengths)
        return pack_padded_sequence(seq[perm], lengths[perm], batch_first=True), perm

    def forward(self, x: torch.Tensor, x_len: torch.Tensor, perm: List[int] = None) -> torch.Tensor:
        normalized_x = self._normalisation_layer1(x.view((-1, x.shape[-1]))).view(x.shape)
        packed_x, _ = self.pack_data(normalized_x, x_len, perm)
        packed_out, self._hidden_state = self._lstm_layer.forward(packed_x, self._hidden_state)
        normalized_lstm_out = self._normalisation_layer2(packed_out.data)
        out = self._fc(normalized_lstm_out)
        return out

    def forward_features(self, mp: Map) -> torch.Tensor:
        raw_features: Dict[str, torch.Tensor] = MapProcessing.extract_features(mp, self.config["data_features"])
        transformed_features: torch.Tensor = MapProcessing.combine_features(raw_features)

        inp = transformed_features.view((1, 1, -1))
        inp = inp.to(self._services.torch.device)
        inp_len = torch.Tensor([1])
        res = self.forward(inp, inp_len)
        _, mov_idx = torch.max(res.squeeze(), 0)
        return Map.ALL_POINTS_MOVE_VECTOR[mov_idx].to_tensor()

    @staticmethod
    def get_config() -> Dict[str, Any]:
        return {
            "data_features": [
                "distance_to_goal_normalized",
                "raycast_8_normalized",
                "direction_to_goal_normalized",
                "agent_goal_angle",
            ],
            "data_labels": [
                "next_position_index",
            ],
            "save_name": "tile_by_tile",
            "training_data": [
                'training_house_100'
                # "training__house_expo"
                #"training_house_100", #Impt
            ],
            # training_uniform_random_fill_10000_block_map_10000_house_10000, "training_uniform_random_fill_10000_block_map_10000", "training_house_10000", "training_uniform_random_fill_10000", "training_block_map_10000",
            "epochs": 100,
            "num_layers": 2,
            "lstm_input_size": 12,
            "lstm_output_size": 8,
            "loss": nn.CrossEntropyLoss(),  # nn.MSELoss(),
            "optimizer": lambda model: torch.optim.Adam(model.parameters(), lr=0.01),
        }


class OnlineLSTM(Algorithm):
    _load_name: str
    _max_it: float

    def __init__(self, services: Services, testing: BasicTesting = None, max_it: float = float('inf'),
                 load_name: str = None):
        super().__init__(services, testing)

        if not load_name:
            raise NotImplementedError("load_name needs to be supplied")

        self._load_name = load_name
        self._max_it = max_it

    def set_display_info(self):
        return super().set_display_info() + [
            OnlineLSTMMapDisplay(self._services)
        ]

    # noinspection PyUnusedLocal
    def _find_path_internal(self) -> None:
        model: BasicLSTMModule = self._services.resources.model_dir.load(self._load_name)
        model.init_running_algorithm(self._get_grid())
        history_frequency: Dict[Point, int] = {}
        last_agent_pos: Point = self._get_grid().agent.position
        stuck_threshold = 5

        it = 0
        while it < self._max_it:
            # goal reached if radius intersects
            if self._get_grid().is_agent_in_goal_radius():
                self.move_agent(self._get_grid().goal.position)
                break

            next_move: Point = Point.from_tensor(model.forward_features(self._get_grid()))
            self.move_agent(self._get_grid().apply_move(next_move, self._get_grid().agent.position))

            last_agent_pos = self._get_grid().agent.position
            new_freq: int = history_frequency.get(last_agent_pos, 0) + 1
            history_frequency[last_agent_pos] = new_freq

            # fail safe
            if new_freq >= stuck_threshold:
                break

            it += 1
            self.key_frame()
