import copy
import math
import os
from typing import List, Any, Tuple, Dict

import torch
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import numpy as np
import pandas
from torch import nn
from torch.nn import functional as F
from torch.utils.data import Dataset, TensorDataset, Subset
from torchvision import transforms, datasets

from algorithms.basic_testing import BasicTesting
from algorithms.configuration.maps.map import Map
from algorithms.lstm.LSTM_tile_by_tile import BasicLSTMModule, OnlineLSTM
from algorithms.lstm.ML_model import MLModel, EvaluationResults
from algorithms.lstm.map_processing import MapProcessing
from simulator.services.services import Services
from utility.constants import DATA_PATH


class CAEEncoder(nn.Module):
    def __init__(self, latent_dim: int):
        super().__init__()
        # The four convolutional layers, and four batch normalization layers in the CAE Encoder.
        self.conv1 = nn.Conv2d(in_channels=1, out_channels=64, kernel_size=3, padding=1)
        self.bn1 = nn.BatchNorm2d(num_features=64)
        self.conv2 = nn.Conv2d(in_channels=64, out_channels=32, kernel_size=3, padding=1)
        self.bn2 = nn.BatchNorm2d(num_features=32)
        self.conv3 = nn.Conv2d(in_channels=32, out_channels=16, kernel_size=3, padding=1)
        self.bn3 = nn.BatchNorm2d(num_features=16)
        self.conv4 = nn.Conv2d(in_channels=16, out_channels=8, kernel_size=3, padding=1)
        self.bn4 = nn.BatchNorm2d(num_features=8)
        self.latent_linear = nn.Linear(in_features=8 * 4 * 4, out_features=latent_dim)
        self.bn_latent = nn.BatchNorm1d(num_features=latent_dim)

        # Define the forward motion of the CAE Encoder, basically applies the Leakey Relu + Max pool activation function -
        # to the Batch Normalized Convolutional Layer,
    def forward(self, input):
        out1 = F.leaky_relu(F.max_pool2d(self.bn1(self.conv1(input)), kernel_size=2, stride=2))
        out2 = F.leaky_relu(F.max_pool2d(self.bn2(self.conv2(out1)), kernel_size=2, stride=2))
        out3 = F.leaky_relu(F.max_pool2d(self.bn3(self.conv3(out2)), kernel_size=2, stride=2))
        out4 = F.leaky_relu(F.max_pool2d(self.bn4(self.conv4(out3)), kernel_size=2, stride=2))
        out5 = self.bn_latent(self.latent_linear(out4.view(-1, 8 * 4 * 4)))
        return [out5, out4, out3, out2, out1, input]


class CAEDecoder(nn.Module):
    def __init__(self, latent_dim: int, with_skip):
        super().__init__()

        self.__with_skip = with_skip
        # Similar to above, but in opposite order
        self.latent_linear = nn.Linear(in_features=latent_dim, out_features=8 * 4 * 4)
        self.bn_latent = nn.BatchNorm1d(num_features=8 * 4 * 4)
        self.deconv1 = nn.ConvTranspose2d(in_channels=8, out_channels=16, kernel_size=2, stride=2)
        self.bn1 = nn.BatchNorm2d(num_features=16)
        self.deconv2 = nn.ConvTranspose2d(in_channels=16, out_channels=32, kernel_size=2, stride=2)
        self.bn2 = nn.BatchNorm2d(num_features=32)
        self.deconv3 = nn.ConvTranspose2d(in_channels=32, out_channels=64, kernel_size=2, stride=2)
        self.bn3 = nn.BatchNorm2d(num_features=64)
        self.deconv4 = nn.ConvTranspose2d(in_channels=64, out_channels=1, kernel_size=2, stride=2)
        self.bn4 = nn.BatchNorm2d(num_features=1)

    def forward(self, x):
        out1 = self.bn_latent(self.latent_linear(x[0])).view(-1, 8, 4, 4)

        if self.__with_skip:
            out2 = F.relu(self.bn1(x[2] + self.deconv1(out1)))
            out3 = F.relu(self.bn2(x[3] + self.deconv2(out2)))
            out4 = F.relu(self.bn3(x[4] + self.deconv3(out3)))
            out5 = torch.tanh(self.bn4(x[5] + self.deconv4(out4)))
        else:
            out2 = F.relu(self.bn1(self.deconv1(out1)))
            out3 = F.relu(self.bn2(self.deconv2(out2)))
            out4 = F.relu(self.bn3(self.deconv3(out3)))
            out5 = torch.tanh(self.bn4(self.deconv4(out4)))
        return out5


# Actual MLModel CAE class
class CAE(MLModel):
    MAP_COLORMAP_FULL = LinearSegmentedColormap.from_list("my_list", [(1, 1, 1), (0, 0, 0), (1, 0, 0), (0, 1, 0)])
    MAP_COLORMAP = "gray_r"
    MNIST_COLORMAP = "gray"

    def __init__(self, services: Services, config: Dict[str, any]):
        super().__init__(services, config)
        # Run encoder then dedoder
        self.encoder = CAEEncoder(self.config["latent_dim"])
        self.decoder = CAEDecoder(self.config["latent_dim"], self.config["with_skip_connections"])

        if self.config["use_mnist_instead"]:
            self.__cm = self.MNIST_COLORMAP
        else:
            self.__cm = self.MAP_COLORMAP

    @staticmethod
    def normalize_data(data: torch.Tensor, mean: float = 0.5, std: float = 0.5) -> torch.Tensor:
        return (data - mean) / std

    def pre_process_data(self) -> Tuple[Dataset, Dataset]:
        if self.config["use_mnist_instead"]:
            return self.get_mnist_dataset()
        else:
            features, labels = super().pre_process_data()
            features = TensorDataset(self.normalize_data(features.tensors[0]))
            labels = TensorDataset(self.normalize_data(labels.tensors[0]))
            return features, labels

    def batch_start(self, inputs: torch.Tensor, labels: torch.Tensor) -> Tuple[
            torch.Tensor, torch.Tensor, torch.Tensor]:

        inp = inputs[0].view((inputs[0].shape[0], -1)).to(self._services.torch.device)
        lbl = labels[0].view((labels[0].shape[0], -1)).to(self._services.torch.device)

        out = self.forward(inp)
        l: torch.Tensor = self.config["loss"](out, lbl)

        return l, out, lbl

    def encode(self, x: torch.Tensor) -> torch.Tensor:
        return self.encoder.forward(x.view((-1, 1, self.config["in_dim"][0], self.config["in_dim"][1])))

    def decode(self, x) -> torch.Tensor:
        return self.decoder.forward(x)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.decode(self.encode(x)).view((-1, self.config["in_dim"][0] * self.config["in_dim"][1]))

    def show_training_results(self, training_results: EvaluationResults, validation_results: EvaluationResults,
                              test_results: EvaluationResults) -> None:
        super().show_training_results(training_results, validation_results, test_results)

        def convert_map_sample(mp_name, idx):
            if any(map(lambda t: mp_name in t, self.config["training_data"])):
                mp = self._services.resources.maps_dir.load(mp_name + "_10/" + str(idx))
                features = MapProcessing.extract_features(mp, self.config["data_single_features"])
                features = MapProcessing.combine_features(features).view(self.config["in_dim"]).to(self._services.torch.device)
                return features

        def convert_sample(img):
            features = CAE.normalize_data(img)
            converted = self.forward(features).view(self.config["in_dim"]).data.tolist()
            feature_maps = self.encode(features)
            latent = feature_maps[0].view((10, 10)).data.tolist()
            feature_maps = list(map(lambda f: f.squeeze(), feature_maps[1:-1]))
            return features.tolist(), converted, latent, feature_maps

        def plot_map(idx, with_f_map):
            maps = ["uniform_random_fill", "block_map", "house"]
            data = list(filter(lambda x: x is not None, map(lambda m: convert_map_sample(m, idx), maps)))
            plot(data, idx, with_f_map)

        def plot_feature_map(img_name, f_map_idx, f_map):
            if f_map.shape[0] == 8:
                size = 2
                size2 = 4

            if f_map.shape[0] == 16:
                size = 4
                size2 = 4

            if f_map.shape[0] == 32:
                size = 4
                size2 = 8

            if f_map.shape[0] == 64:
                size = 8
                size2 = 8

            fig, axes = plt.subplots(size, size2, figsize=(9, 9))

            for i in range(size):
                for j in range(size2):
                    axes[i][j].imshow(f_map[i + j].tolist(), cmap="gray")
                    axes[i][j].axis('off')

            self._services.resources.model_dir.get_subdir(self.save_name()).save_figure("feature_maps_" + str(img_name.lower().replace(" ", "_") + "_" + str(f_map_idx)))
            plt.show()

        def plot(imgs, idx, with_f_map):
            data = list(map(lambda x: convert_sample(x), imgs))

            fig, axes = plt.subplots(len(data), 3, figsize=(9, 3.5 * len(data)))

            if len(data) == 1:
                axes[0].title.set_text('Original')
                axes[0].imshow(data[0][0], cmap=self.__cm)

                axes[1].title.set_text('Converted')
                axes[1].imshow(data[0][1], cmap=self.__cm)

                axes[2].title.set_text('Latent Space')
                axes[2].imshow(data[0][2], cmap="gray")
            else:
                for i in range(len(data)):
                    axes[i][0].title.set_text('Original')
                    axes[i][0].imshow(data[i][0], cmap=self.__cm)

                    axes[i][1].title.set_text('Converted')
                    axes[i][1].imshow(data[i][1], cmap=self.__cm)

                    axes[i][2].title.set_text('Latent Space')
                    axes[i][2].imshow(data[i][2], cmap="gray")
            self._services.resources.model_dir.get_subdir(self.save_name()).save_figure("network_analysis_" + str(idx))
            plt.show()

            if with_f_map:
                for i in range(len(data)):
                    name = "map_" + str(idx) + "_" + str(i)
                    for q in range(4):
                        plot_feature_map(name, 3 - q, data[i][3][3 - q])

        if self.config["use_mnist_instead"]:
            imgs = [
                test_results.data.dataset[0][0][0].view(self.config["in_dim"]).to(self._services.torch.device),
                test_results.data.dataset[1][0][0].view(self.config["in_dim"]).to(self._services.torch.device),
                test_results.data.dataset[2][0][0].view(self.config["in_dim"]).to(self._services.torch.device),
            ]
            plot(imgs, 0, True)
        else:
            plot_map(0, True)
            plot_map(1, True)

    @staticmethod
    def get_config() -> Dict[str, Any]:
        return {
            "data_single_features": [
                "global_map"
            ],
            "data_single_labels": [
                "global_map"
            ],
            "save_name": "caelstm_section_cae",
            "training_data": [
                "training_uniform_random_fill_10000_block_map_10000_house_10000",
            ],
            "use_mnist_instead": False,
            "mnist_size": None,
            "epochs": 100,
            "with_skip_connections": True,
            "in_dim": [64, 64],
            "latent_dim": 100,
            "loss": nn.L1Loss(reduction='mean'),
            "optimizer": lambda model: torch.optim.Adam(model.parameters(), lr=0.01),
        }

    """
    def prefix_name(self) -> str:
        return super().prefix_name() + "_{}_{}".format(self.config["with_skip_connections"], self.config["use_mnist_instead"])
    """

    def get_mnist_dataset(self) -> Tuple[Dataset, Dataset]:
        transform = transforms.Compose([
            transforms.Resize([64, 64]),
            transforms.Grayscale(),
            transforms.ToTensor(),
            transforms.Normalize(mean=(0.5,), std=(0.5,))
        ])
        mnist = datasets.MNIST(os.path.join(DATA_PATH, "datasets"), train=True, download=True, transform=transform)
        if self.config["mnist_size"]:
            return Subset(mnist, range(self.config["mnist_size"])), Subset(copy.deepcopy(mnist), range(self.config["mnist_size"]))
        else:
            return mnist, copy.deepcopy(mnist)


class LSTMCAEModel(BasicLSTMModule):
    __cached_encoded_map: torch.Tensor

    def __init__(self, services: Services, config: Dict[str, any]):
        super().__init__(services, config)
        #Problem lies here
        self.__encoder = self.__get_encoder()
        self.__cached_encoded_map = None

        if "with_init_fn" in self.config and self.config["with_init_fn"]:
            self.fc = nn.Linear(in_features=114, out_features=self.config["lstm_input_size"])
            self.bn = nn.BatchNorm1d(num_features=self.config["lstm_input_size"])
            self.dp = nn.Dropout()

    def pre_process_data(self) -> Tuple[Dataset, Dataset]:
        data_features, data_labels = super().pre_process_data()

        if "agent_position" in self.config["data_features"]:
            data_features.subsets[0].data[:, :, -2:] /= 64

        return data_features, data_labels

    def init_running_algorithm(self, mp: Map) -> None:
        super().init_running_algorithm(mp)

        raw_features_img: Dict[str, torch.Tensor] = MapProcessing.extract_features(mp, self.config["data_single_features"])
        transformed_features_img: torch.Tensor = MapProcessing.combine_features(raw_features_img)
        transformed_features_img = transformed_features_img.to(self._services.torch.device)
        self.__cached_encoded_map = self.__encode_image(transformed_features_img).view((1, 1, -1))

    def __get_encoder(self) -> CAE:
        if "custom_encoder" in self.config and self.config["custom_encoder"]:
            encoder_name: str = self.config["custom_encoder"]
        else:
            #Problem here
            encoder_name: str = "caelstm_section_cae_" + self.training_suffix() + "_model"
        return self._services.resources.model_dir.load(encoder_name)

    def __encode_image(self, img: torch.Tensor, seq_size: int = 1) -> torch.Tensor:
        enc_in = CAE.normalize_data(img)
        enc_out = self.__encoder.encode(enc_in)[0].unsqueeze(1).repeat(1, seq_size, 1)
        return enc_out

    def __combine_features(self, x: torch.Tensor, img: torch.Tensor) -> torch.Tensor:
        x = x.to(self._services.torch.device)
        img = img.to(self._services.torch.device)
        enc_out = self.__encode_image(img, x.shape[1])
        return torch.cat((x, enc_out), 2)

    def batch_start(self, inputs: Tuple[Tuple, Tuple], labels: Tuple[torch.Tensor, torch.Tensor]) -> \
            Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        x, x_len, y_sorted, perm = self._prepare_data_before_forward(inputs[0], labels)

        # augment data with encoder result
        x_concat = self.__combine_features(x, inputs[1][0])
        out = self.forward(x_concat, x_len, perm).view(-1, 8)
        if len(y_sorted) != len(out):
            y_sorted = torch.tensor(y_sorted.tolist() + [0]).to(self._services.torch.device)
        l: torch.Tensor = self.config["loss"](out, y_sorted)
        return l, out, y_sorted

    def forward(self, x: torch.Tensor, x_len: torch.Tensor, perm: List[int] = None) -> torch.Tensor:
        init_shape = x.shape
        if "with_init_fn" in self.config and self.config["with_init_fn"]:
            x = F.relu(self.dp(self.bn(self.fc(x.view(-1, self.config["input_size"]))))).view(init_shape[0], init_shape[1], -1)
        return super().forward(x, x_len, perm)

    # cache obstacle once
    def forward_features(self, mp: Map) -> torch.Tensor:
        raw_features_x: Dict[str, torch.Tensor] = MapProcessing.extract_features(mp, self.config["data_features"])
        transformed_features_x: torch.Tensor = MapProcessing.combine_features(raw_features_x)
        transformed_features_x = transformed_features_x.to(self._services.torch.device)

        if "agent_position" in self.config["data_features"]:
            transformed_features_x[-2:-1] /= mp.size.width
            transformed_features_x[-1:] /= mp.size.height

        inp = torch.cat((transformed_features_x.view((1, 1, -1)), self.__cached_encoded_map), 2)
        inp_len = torch.Tensor([1])
        res = self.forward(inp, inp_len)
        _, mov_idx = torch.max(res.squeeze(), 0)
        return mp.ALL_POINTS_MOVE_VECTOR[mov_idx].to_tensor()

    @staticmethod
    def get_config() -> Dict[str, Any]:
        return {
            "data_features": [
                "raycast_8_normalized",
                "distance_to_goal_normalized",
                "direction_to_goal_normalized",
                "agent_goal_angle",
            ],
            "data_single_features": [
                "global_map",
            ],
            "data_labels": [
                "next_position_index",
            ],
            "custom_encoder": None,  # "caelstm_section_cae_training_uniform_random_fill_10000_block_map_10000_house_10000_model",
            "save_name": "caelstm_section_lstm",
            "training_data": [
                "training_uniform_random_fill_300_block_map_300_house_300"
                # "training_uniform_random_fill_10000",
                # "training_block_map_10000",
                # "training_house_10000",
            ],  # training_uniform_random_fill_10000_block_map_10000_house_10000, "training_uniform_random_fill_10000_block_map_10000", "training_house_10000", "training_uniform_random_fill_10000", "training_block_map_10000",
            "epochs": 25,
            "num_layers": 2,
            # "with_init_fn": False,
            # "input_size": 114,
            "lstm_input_size": 112,
            "lstm_output_size": 8,
            "loss": nn.CrossEntropyLoss(),
            "optimizer": lambda model: torch.optim.Adam(model.parameters(), lr=0.01),
        }
