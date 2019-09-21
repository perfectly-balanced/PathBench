"""
---------------------------------------OBSOLETE BY CAE LSTM---------------------------------------
---------------------------------------OBSOLETE BY CAE LSTM---------------------------------------
---------------------------------------OBSOLETE BY CAE LSTM---------------------------------------
---------------------------------------OBSOLETE BY CAE LSTM---------------------------------------

from typing import Dict, Any, Tuple, List

import torch
from torch import nn

from algorithms.basic_testing import BasicTesting
from algorithms.lstm.online_lstm import BasicLSTMModule, OnlineLSTM
from simulator.services.services import Services


class LSTMCNN(BasicLSTMModule):
    def __init__(self, services: Services, config: Dict[str, any]):
        super().__init__(services, config)

        self._hidden_state = None
        self._normalisation_layer1 = nn.BatchNorm1d(num_features=9*9+60*60)

        self.__local_pipeline = nn.Sequential(
            nn.Conv2d(in_channels=1, out_channels=3, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(in_channels=3, out_channels=1, kernel_size=3, padding=1),
            nn.ReLU(),
        )

        self.__global_pipeline = nn.Sequential(
            nn.Conv2d(in_channels=1, out_channels=3, kernel_size=3, padding=1), # 60 * 60
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(in_channels=3, out_channels=6, kernel_size=3, padding=1), # 30 * 30
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(in_channels=6, out_channels=3, kernel_size=3, padding=1), # 15 * 15
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(in_channels=3, out_channels=1, kernel_size=3, padding=1), # 7 * 7
            nn.ReLU(),
        )

        self._lstm_layer = nn.LSTM(self.config["lstm_input_size"], self.config["lstm_output_size"],
                                   num_layers=self.config["num_layers"], batch_first=True)
        self._normalisation_layer2 = nn.BatchNorm1d(num_features=self.config["lstm_output_size"])
        self._fc = nn.Linear(in_features=self.config["lstm_output_size"], out_features=self.config["lstm_output_size"])

    def forward(self, x: torch.Tensor, x_len: torch.Tensor, perm: List[int] = None) -> torch.Tensor:
        normalized_data = self._normalisation_layer1(x.view(-1, x.shape[-1])).view(x.shape)
        local_map: torch.Tensor = normalized_data[:, :, :9*9].view((-1, 1, 9, 9))
        global_map: torch.Tensor = normalized_data[:, :, 9*9:].view(-1, 1, 60, 60)

        local_out = self.__local_pipeline(local_map).view((x.shape[0], x.shape[1], -1))
        global_out = self.__global_pipeline(global_map).view((x.shape[0], x.shape[1], -1))
        all_out = torch.cat((local_out, global_out), len(x.shape) - 1)

        packed_data, _ = BasicLSTMModule.pack_data(all_out, x_len, perm)
        packed_out, self._hidden_state = self._lstm_layer(packed_data, self._hidden_state)
        normalized_lstm_out = self._normalisation_layer2(packed_out.data)
        out = self._fc(normalized_lstm_out)
        return out

    @staticmethod
    def get_config() -> Dict[str, Any]:
        return {
            "data_features": [
                "local_map",
                "global_map",
            ],
            "data_labels": [
                "next_position_index",
            ],
            "save_name": "cnn_tile_by_tile",
            "training_data": "training_1000",
            "epochs": 5,
            "num_layers": 2,
            "lstm_input_size": 65,
            "lstm_output_size": 8,
            "loss": nn.CrossEntropyLoss(),
            "optimizer": lambda model: torch.optim.Adam(model.parameters(), lr=0.01),
        }


class LSTMCNNTileByTile(OnlineLSTM):
    def __init__(self, services: Services, testing: BasicTesting = None):
        super().__init__(services, testing)

        self._load_name = "cnn_tile_by_tile_training_1000_model"
"""
