import numpy as np

from sklearn import metrics
from io import StringIO
import random
from typing import Dict, Tuple, List, Any, Union

import matplotlib.pyplot as plt
import torch as torch
import torch.nn as nn
from torch import Tensor
from torch.nn.utils.rnn import pad_sequence, pack_padded_sequence, pad_packed_sequence, pack_sequence, PackedSequence
from torch.utils import data
from torch.utils.data import DataLoader, TensorDataset, Dataset, Subset
from algorithms.lstm.map_processing import MapProcessing
from simulator.services.debug import DebugLevel
from simulator.services.services import Services


class EvaluationResults:
    name: str

    __model: 'MLModel'
    __services: Services
    __data: DataLoader
    __stream: StringIO

    __epoch_acc: Dict[str, Any]
    __stats_acc: List[Dict[str, Any]]

    def __init__(self, services: Services, model: 'MLModel', results_name: str, data: DataLoader,
                 stream: StringIO) -> None:
        self.__services = services
        self.__model = model
        self.__data = data
        self.__stream = stream
        self.name = results_name

    def start(self, with_print: bool = True) -> None:
        if with_print:
            self.__services.debug.write("{}: {}".format(self.name, str(type(self.__model))), DebugLevel.BASIC,
                                        streams=[self.__stream])

        self.__epoch_acc = {}
        self.__stats_acc = []

    def epoch_start(self) -> None:
        self.__epoch_acc = {}

    def epoch_finish(self, term: str = "\n") -> None:
        stats = self.__get_all_stats()

        if "accuracy" not in stats:
            self.__services.debug.write(
                '{} \t Epoch: {} \t Loss: {:.6f}'.format(self.name, len(self.__stats_acc), stats["loss"]),
                DebugLevel.BASIC, term, streams=[self.__stream])
        else:
            self.__services.debug.write(
                '{} \t Epoch: {} \t Loss: {:.6f} \t Accuracy {:.2f} \t Precision: {:.2f} \t Recall: {:.2f} \t F1: {:.2f}'.format(
                    self.name, len(self.__stats_acc), stats["loss"],
                    stats["accuracy"], stats["precision"], stats["recall"], stats["f1"]),
                DebugLevel.BASIC, term, streams=[self.__stream])

        self.__stats_acc.append(stats)

    def __get_normalised(self, val):
        norm_val = val / len(self.__data) * self.__model.config["batch_size"]
        return norm_val

    def __get_all_stats(self) -> Dict[str, Any]:
        res = {
            "loss": self.__get_normalised(self.__epoch_acc["loss"])
        }

        if len(self.__epoch_acc["predictions"]) > 0:
            actual = self.__epoch_acc["actual"]
            predicted = self.__epoch_acc["predictions"]

            res["accuracy"] = metrics.accuracy_score(actual, predicted)
            res["precision"] = metrics.precision_score(actual, predicted, average="macro")
            res["recall"] = metrics.recall_score(actual, predicted, average="macro")
            res["f1"] = metrics.f1_score(actual, predicted, average="macro")
            res["confusion_matrix"] = metrics.confusion_matrix(actual, predicted)

        return res

    def batch_start(self) -> None:
        self.__epoch_acc = {
            "loss": 0,
            "predictions": [],
            "actual": [],
        }

    def batch_finish(self, loss: torch.Tensor, model_out: torch.Tensor, labels_out: torch.Tensor) -> None:
        self.__epoch_acc["loss"] += loss

        if isinstance(self.__model.config["loss"], torch.nn.modules.loss.CrossEntropyLoss):
            _, model_out_classes = torch.max(model_out, 1)
            self.__epoch_acc["predictions"] += model_out_classes.tolist()
            self.__epoch_acc["actual"] += labels_out.tolist()

    def finish(self, with_print: bool = True) -> None:
        if with_print:
            self.__services.debug.write('', DebugLevel.BASIC, timestamp=False, streams=[self.__stream])

    def get_results(self) -> Dict[str, Any]:
        return {
            "latest_loss": self.__stats_acc[-1]["loss"],
            "epochs": len(self.__stats_acc),
            "loss_log": list(map(lambda s: s["loss"], self.__stats_acc)),
            "latest_results": self.__stats_acc[-1],
            "all_results": self.__stats_acc,
        }

    @property
    def data(self) -> str:
        return "data"

    @data.getter
    def data(self) -> DataLoader:
        return self.__data


class CombinedSubsets(Dataset):
    subsets: List[Subset]

    def __init__(self, *subsets) -> None:
        assert all(len(subsets[0]) == len(subset) for subset in subsets)
        self.subsets = subsets

    def __getitem__(self, index: int) -> Tuple:
        return tuple(subset[index] for subset in self.subsets)

    def __len__(self) -> int:
        return len(self.subsets[0])


class SingleTensorDataset(Dataset):
    tensor: Tensor

    def __init__(self, tensor: torch.Tensor) -> None:
        self.tensor = tensor

    def __getitem__(self, index: int) -> torch.Tensor:
        return self.tensor[index]

    def __len__(self) -> int:
        return len(self.tensor)


class PackedDataset(Dataset):
    perm: List[int]
    data: torch.Tensor
    lengths: torch.Tensor

    def __init__(self, seq: List[torch.Tensor]) -> None:
        from algorithms.lstm.LSTM_tile_by_tile import BasicLSTMModule

        ls = list(map(lambda el: el.shape[0], seq))
        self.perm = BasicLSTMModule.get_sort_by_lengths_indices(ls)
        seq.sort(key=lambda el: el.shape[0], reverse=True)
        packed_sequence: PackedSequence = pack_sequence(seq)
        self.data, self.lengths = pad_packed_sequence(packed_sequence, batch_first=True)

    def __getitem__(self, index: int) -> Tuple[Any, int]:
        return self.data[index], self.lengths[index]

    def __len__(self) -> int:
        return len(self.data)


class MLModel(torch.nn.Module):
    config: Dict[str, any]
    _services: Services
    __training_stream: StringIO

    def __init__(self, services: Services, config: Dict[str, any]):
        super().__init__()

        self._services = services
        self.config = self.get_default_config()
        self.config.update(config)
        self.__training_stream = None

    def batch_start(self, inputs: Any, labels: Any) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        DO NOT FORGET TO .to(device) the input and labels
        """
        raise NotImplementedError()

    def _process(self, data: List[Dict[str, Any]], is_features: bool) -> Dataset:
        data_single: List[str] = self.config["data_single_features"] if is_features else self.config["data_single_labels"]
        data_seq: List[str] = self.config["data_features"] if is_features else self.config["data_labels"]
        data_category_single: str = "single_features" if is_features else "single_labels"
        data_category_seq: str = "features" if is_features else "labels"

        if data_seq:
            data_features_sequence: List[torch.Tensor] = list(map(lambda d: MapProcessing.combine_sequential_features(
                MapProcessing.pick_sequential_features(d[data_category_seq], data_seq)
            ), data))

            data_seq_dataset: Dataset = PackedDataset(data_features_sequence)

        if data_single:
            data_features_single: List[torch.Tensor] = list(map(lambda d: MapProcessing.combine_features(
                MapProcessing.pick_features(d[data_category_single], data_single)
            ), data))

            data_features_single = torch.stack(data_features_single)
            if data_seq:
                data_features_single = data_features_single[data_seq_dataset.perm]

            data_single_dataset: Dataset = TensorDataset(data_features_single)

        if data_seq and data_single:
            return CombinedSubsets(data_seq_dataset, data_single_dataset)
        elif data_seq:
            return data_seq_dataset
        else:
            return data_single_dataset

    def pre_process_data(self) -> Tuple[Dataset, Dataset]:
        data: List[Dict[str, Any]] = []
        # load data
        for tr in self.config["training_data"]:
            self._services.debug.write("Loading data: " + tr, DebugLevel.BASIC, streams=[self.__training_stream])
            data = data + self._services.resources.training_data_dir.load(tr)

        if not data:
            raise Exception("No training data")

        self._services.debug.write("Picking features", DebugLevel.BASIC, streams=[self.__training_stream])
        return self._process(data, True), self._process(data, False)

    def split_data(self, data: Dataset, labels: Dataset) -> \
            Tuple[DataLoader, DataLoader, DataLoader]:
        # shuffle data
        indices: List[int] = torch.randperm(len(data))

        # get number of samples
        test_samples: int = int(self.config["test_ratio"] * len(data))
        validation_samples: int = int(self.config["validation_ratio"] * len(data))
        train_samples: int = len(data) - test_samples - validation_samples

        # create subsets
        train_data = Subset(data, indices[:train_samples])
        train_labels = Subset(labels, indices[:train_samples])
        validation_data = Subset(data, indices[train_samples:train_samples + validation_samples])
        validation_labels = Subset(labels, indices[train_samples:train_samples + validation_samples])
        test_data = Subset(data, indices[train_samples + validation_samples:])
        test_labels = Subset(labels, indices[train_samples + validation_samples:])

        combined_train_data: TensorDataset = CombinedSubsets(train_data, train_labels)
        combined_validation_data: TensorDataset = CombinedSubsets(validation_data, validation_labels)
        combined_test_data: TensorDataset = CombinedSubsets(test_data, test_labels)

        train_loader = DataLoader(combined_train_data, batch_size=self.config["batch_size"],
                                  shuffle=False, num_workers=8)
        validation_loader = DataLoader(combined_validation_data, batch_size=self.config["batch_size"],
                                       shuffle=False, num_workers=8)
        test_loader = DataLoader(combined_test_data, batch_size=self.config["batch_size"],
                                 shuffle=False, num_workers=8)

        return train_loader, validation_loader, test_loader

    def full_train_holdout(self) -> None:
        """
        Make sure to delete the cache files from resources if any pipeline step should be run again
        """
        self.__training_stream = StringIO()
        self._services.debug.write(
            "Starting holdout training: " + self._services.debug.pretty_dic_str(self.config) + "\n", DebugLevel.BASIC,
            streams=[self.__training_stream])
        self._services.debug.write("Starting data pre processing", DebugLevel.BASIC, streams=[self.__training_stream])
        cache_hit: bool = True

        def f() -> Tuple[Dataset, Dataset]:
            global cache_hit
            cache_hit = False
            self._services.debug.write("Cache miss, starting new training data pre processing", DebugLevel.BASIC,
                                       streams=[self.__training_stream])
            return self.pre_process_data()

        if self._services.settings.trainer_bypass_and_replace_pre_processed_cache:
            self._services.debug.write("Deleting " + str(
                self.prefix_name() + "_pre_processed_data") + " (trainer_bypass_and_replace_pre_processed_cache = True)",
                DebugLevel.BASIC,
                streams=[self.__training_stream])
            self._services.resources.cache_dir.delete_entry(self.prefix_name() + "_pre_processed_data")

        data_features, data_labels = self._services.resources.cache_dir.get_or_save(
            self.prefix_name() + "_pre_processed_data",
            f
        )

        if cache_hit:
            self._services.debug.write("Cache hit, training data loaded from cache", DebugLevel.BASIC,
                                       streams=[self.__training_stream])

        if self._services.settings.trainer_pre_process_data_only:
            self._services.debug.write("Saving pre processed data in cache (trainer_pre_process_data_only = True)",
                                       DebugLevel.BASIC,
                                       streams=[self.__training_stream])
            return

        self._services.debug.write("Finished data pre processing \n", DebugLevel.BASIC,
                                   streams=[self.__training_stream])
        train_data, validation_data, test_data = self.split_data(data_features, data_labels)
        train_results: Tuple[EvaluationResults, EvaluationResults] = self.train_model(train_data, validation_data)
        eval_results: EvaluationResults = self.evaluate_model(test_data)
        self.save()
        self.show_training_results(*train_results, eval_results)
        self._services.resources.model_dir.get_subdir(self.save_name()).save_log(self.__training_stream)
        self.__training_stream = None

    def show_training_results(self, training_results: EvaluationResults, validation_results: EvaluationResults,
                              test_results: EvaluationResults) -> None:
        if not self._services.debug.should_debug(DebugLevel.BASIC):
            return

        training_raw: Dict[str, Any] = training_results.get_results()
        validation_raw: Dict[str, Any] = validation_results.get_results()
        evaluation_raw: Dict[str, Any] = test_results.get_results()

        self._services.debug.write("Model: {}".format(self), DebugLevel.BASIC, streams=[self.__training_stream])
        self._services.debug.write("Model loss: {}".format(evaluation_raw["latest_loss"]), DebugLevel.BASIC,
                                   streams=[self.__training_stream])

        if "accuracy" in evaluation_raw["latest_results"]:
            self._services.debug.write("Model accuracy: {}".format(evaluation_raw["latest_results"]["accuracy"]), DebugLevel.BASIC,
                                       streams=[self.__training_stream])
            self._services.debug.write("Model precision: {}".format(evaluation_raw["latest_results"]["precision"]), DebugLevel.BASIC,
                                       streams=[self.__training_stream])
            self._services.debug.write("Model recall: {}".format(evaluation_raw["latest_results"]["recall"]), DebugLevel.BASIC,
                                       streams=[self.__training_stream])
            self._services.debug.write("Model f1: {}".format(evaluation_raw["latest_results"]["f1"]), DebugLevel.BASIC,
                                       streams=[self.__training_stream])
            self._services.debug.write("Model confusion matrix: \n{}".format(evaluation_raw["latest_results"]["confusion_matrix"]), DebugLevel.BASIC,
                                       streams=[self.__training_stream])

        plt.plot(range(training_raw["epochs"]), training_raw["loss_log"], c='r', label='training')
        plt.plot(range(validation_raw["epochs"]), validation_raw["loss_log"], c='b', label='validation')
        plt.legend(loc='upper right')
        plt.title('Loss')
        plt.xlabel('epoch')
        plt.ylabel('loss')
        self._services.resources.model_dir.get_subdir(self.save_name()).save_figure("loss")
        plt.show()

        if "accuracy" in evaluation_raw["latest_results"]:
            def plot(res, name):
                t_accuracy = list(map(lambda s: s["accuracy"], res["all_results"]))
                t_precision = list(map(lambda s: s["precision"], res["all_results"]))
                t_recall = list(map(lambda s: s["recall"], res["all_results"]))
                t_f1 = list(map(lambda s: s["f1"], res["all_results"]))

                plt.plot(range(res["epochs"]), t_accuracy, c='r', label='accuracy')
                plt.plot(range(res["epochs"]), t_precision, c='b', label='precision')
                plt.plot(range(res["epochs"]), t_recall, c='g', label='recall')
                plt.plot(range(res["epochs"]), t_f1, c='y', label='f1')
                plt.legend(loc='upper right')
                plt.title(name + ' Statistics')
                plt.xlabel('epoch')
                plt.ylabel('value')
                self._services.resources.model_dir.get_subdir(self.save_name()).save_figure(name.lower() + "_stats")
                plt.show()

            plot(training_raw, "Training")
            plot(validation_raw, "Validation")

    def train_model(self, data_loader: DataLoader, validation_loader: DataLoader) -> Tuple[
            EvaluationResults, EvaluationResults]:
        train_results = EvaluationResults(self._services, self, "Training", data_loader, self.__training_stream)
        validation_results = EvaluationResults(self._services, self, "Validation", validation_loader,
                                               self.__training_stream)

        optimizer = self.config["optimizer"](self)
        train_results.start()
        validation_results.start(False)

        for epoch in range(self.config['epochs']):
            self.train()
            train_results.epoch_start()
            for _, (inputs, labels) in enumerate(data_loader, 0):
                train_results.batch_start()
                optimizer.zero_grad()
                l, model_out, labels_out = self.batch_start(inputs, labels)
                l.backward()
                optimizer.step()
                train_results.batch_finish(l, model_out, labels_out)
            train_results.epoch_finish()

            self.eval()
            validation_results.epoch_start()
            with torch.no_grad():
                for _, (inputs, labels) in enumerate(validation_loader, 0):
                    validation_results.batch_start()
                    l, model_out, labels_out = self.batch_start(inputs, labels)
                    validation_results.batch_finish(l, model_out, labels_out)
            validation_results.epoch_finish("\n\n")

        train_results.finish()
        validation_results.finish(False)
        return train_results, validation_results

    def evaluate_model(self, data_loader: DataLoader) -> EvaluationResults:
        results: EvaluationResults = EvaluationResults(self._services, self, "Evaluation", data_loader,
                                                       self.__training_stream)

        self.eval()
        results.start()
        results.epoch_start()

        with torch.no_grad():
            for _, (inputs, labels) in enumerate(data_loader, 0):
                results.batch_start()
                l, model_out, labels_out = self.batch_start(inputs, labels)
                results.batch_finish(l, model_out, labels_out)

        results.epoch_finish()
        results.finish()
        return results

    @staticmethod
    def get_default_config() -> Dict[str, any]:
        return {
            "data_features": [],
            "data_labels": [],
            "data_single_features": [],
            "data_single_labels": [],
            "epochs": 10,
            "loss": nn.MSELoss(),  # nn.CrossEntropyLoss()
            "optimizer": lambda model: torch.optim.Adam(model.parameters(), lr=0.001),
            "validation_ratio": 0.2,
            "test_ratio": 0.2,
            "save_name": "placeholder",
            "training_data": [],
            "batch_size": 50,
        }

    def prefix_name(self) -> str:
        return "{}_{}".format(self.config["save_name"], self.training_suffix())

    def training_suffix(self):
        # backwards compatibility
        if isinstance(self.config["training_data"], str):
            return self.config["training_data"]

        tr_name = "training"

        for tr in self.config["training_data"]:
            tr_name += "_" + "_".join(tr.split("_")[1:])

        return tr_name

    def save_name(self):
        return self.prefix_name() + "_model"

    def save(self) -> None:
        self._services.resources.model_dir.save(self.save_name(), self)
        self._services.debug.write("Saved model as " + self.save_name(), DebugLevel.BASIC,
                                   streams=[self.__training_stream])
