import copy
from typing import Dict, Callable, List, Set, Union

import torch
import torchvision
from torchvision import transforms

from algorithms.configuration.entities.entity import Entity
from algorithms.configuration.entities.trace import Trace
from algorithms.configuration.maps.dense_map import DenseMap
from algorithms.configuration.maps.map import Map
from structures import Point


class MapProcessing:
    @staticmethod
    def __feature_extractors() -> Dict[str, Callable[[Map], torch.Tensor]]:
        return {
            "agent_position": MapProcessing.agent_position_feature,
            "direction_to_goal": MapProcessing.direction_to_goal_feature,
            "direction_to_goal_normalized": MapProcessing.direction_to_goal_normalized_feature,
            "distance_to_goal": MapProcessing.distance_to_goal_feature,
            "distance_to_goal_normalized": MapProcessing.distance_to_goal_normalized_feature,
            "raycast_8": MapProcessing.raycast_8_feature,
            "raycast_8_normalized": MapProcessing.raycast_8_normalized_feature,
            "agent_goal_angle": MapProcessing.agent_goal_angle_feature,
            "local_map": MapProcessing.local_map_normalized_feature,
            "global_map": MapProcessing.global_map_normalized_feature,
            "valid_moves": MapProcessing.valid_moves_feature,
        }

    @staticmethod
    def __label_extractors() -> Dict[str, Callable[[Map], List[torch.Tensor]]]:
        return {
            "next_position": MapProcessing.next_position_label,
            "next_position_index": MapProcessing.next_position_index_label,
            "global_map": MapProcessing.global_map_normalized_feature,
        }

    @staticmethod
    def get_sequential_features(mp: Map, feature_extractors: List[str]) -> List[Dict[str, torch.Tensor]]:
        """
        Replays the map trace and extracts the features at each time step
        :return: A list, each cell represents a time step and contains a dictionary of features
        """
        if not mp.trace:
            # fake movement
            mp.move_agent(mp.agent.position)
        return mp.replay_trace(lambda m: MapProcessing.extract_features(m, feature_extractors))

    @staticmethod
    def get_single_features(mp: Map, feature_extractors: List[str]) -> Dict[str, torch.Tensor]:
        """
        Replays the map trace and extracts the features at each time step
        :return: A list, each cell represents a time step and contains a dictionary of features
        """
        return MapProcessing.extract_features(mp, feature_extractors)

    @staticmethod
    def extract_features(mp: Map, feature_extractors: List[str]) -> Dict[str, torch.Tensor]:
        """
        Main function for extracting all passed features at the current map state
        :return: A dictionary of features, feature name to actual feature
        """
        ret: Dict[str, torch.Tensor] = {}
        for extractor_name in feature_extractors:
            ret[extractor_name] = MapProcessing.__feature_extractors()[extractor_name](mp)
        return ret

    @staticmethod
    def pick_sequential_features(sequential_features: List[Dict[str, torch.Tensor]], feature_names: List[str]) -> List[
            Dict[str, torch.Tensor]]:
        """
        Extracts only certain features from the sequence of features
        """
        ret: List[Dict[str, torch.Tensor]] = []
        for features in sequential_features:
            ret.append(MapProcessing.pick_features(features, feature_names))
        return ret

    @staticmethod
    def pick_features(features: Dict[str, torch.Tensor], feature_names: List[str]) -> Dict[str, torch.Tensor]:
        """
        Extracts only certain features from a dictionary of features
        """
        ret: Dict[str, torch.Tensor] = {}
        for name in feature_names:
            ret[name] = features[name]
        return ret

    @staticmethod
    def combine_sequential_features(sequential_features: List[Dict[str, torch.Tensor]]) -> torch.Tensor:
        """
        Transforms a sequence of features in a single tensor of dimension s * n,
        s is sequence length, n is the sum of all feature dimensions
        """
        acc: torch.Tensor = torch.Tensor()
        for fs in sequential_features:
            acc = torch.cat((acc, MapProcessing.combine_features(fs).unsqueeze(0)))
        return acc

    @staticmethod
    def combine_features(features: Dict[str, torch.Tensor]) -> torch.Tensor:
        """
        Transforms all features into a single tensor of size n, where n is the sum of all feature dimensions
        """
        acc: torch.Tensor = torch.Tensor()
        for feature_value in features.values():
            acc = torch.cat((acc, feature_value.view(-1)))
        return acc

    @staticmethod
    def get_sequential_labels(mp: Map, label_extractors: List[str]) -> List[Dict[str, torch.Tensor]]:
        """
        Returns a sequence of labels
        """
        ret: List[Dict[str, torch.Tensor]] = []
        for extractor_name in label_extractors:
            labels: List[torch.Tensor] = MapProcessing.__label_extractors()[extractor_name](mp)
            named_labels: List[Dict[str, torch.Tensor]] = list(map(lambda l: {extractor_name: l}, labels))

            if not ret:
                ret = named_labels
            else:
                ret = list(map(lambda el: {**el[0], **el[1]}, zip(ret, named_labels)))
        return ret

    @staticmethod
    def get_single_labels(mp: Map, label_extractors: List[str]) -> Dict[str, torch.Tensor]:
        ret: Dict[str, torch.Tensor] = {}
        for extractor_name in label_extractors:
            ret[extractor_name] = MapProcessing.__label_extractors()[extractor_name](mp)
        return ret

    """ Features """

    @staticmethod
    def agent_position_feature(mp: Map) -> torch.Tensor:
        return MapProcessing.__get_pos(mp.agent)

    @staticmethod
    def direction_to_goal_feature(mp: Map) -> torch.Tensor:
        return MapProcessing.__get_pos(mp.goal) - MapProcessing.__get_pos(mp.agent)

    @staticmethod
    def direction_to_goal_normalized_feature(mp: Map) -> torch.Tensor:
        dir: torch.Tensor = MapProcessing.direction_to_goal_feature(mp)
        nm = torch.norm(dir)
        if nm == 0:
            return torch.zeros([2])
        else:
            return dir / torch.norm(dir)

    @staticmethod
    def distance_to_goal_feature(mp: Map) -> torch.Tensor:
        return torch.norm(MapProcessing.direction_to_goal_feature(mp))

    @staticmethod
    def distance_to_goal_normalized_feature(mp: Map) -> torch.Tensor:
        norm_factor = 100.0
        return torch.clamp(MapProcessing.distance_to_goal_feature(mp), min=0, max=norm_factor) / norm_factor

    @staticmethod
    def raycast_8_feature(mp: Map) -> torch.Tensor:
        ret: List[torch.Tensor] = []
        for mv in mp.ALL_POINTS_MOVE_VECTOR:
            hit_point: torch.Tensor = MapProcessing.__get_hit_point_along_dir(mp, mv).to_tensor()
            dir: torch.Tensor = MapProcessing.__get_pos(mp.agent) - hit_point
            ret.append(torch.norm(dir))
        return torch.stack(ret)

    @staticmethod
    def raycast_8_normalized_feature(mp: Map) -> torch.Tensor:
        norm_factor = 50.0
        return torch.clamp(MapProcessing.raycast_8_feature(mp), min=0, max=norm_factor) / norm_factor

    @staticmethod
    def agent_goal_angle_feature(mp: Map) -> torch.Tensor:
        v: torch.Tensor = MapProcessing.__get_pos(mp.goal) - MapProcessing.__get_pos(mp.agent)
        return torch.atan2(v[1], v[0])

    @staticmethod
    def valid_moves_feature(mp: Map) -> torch.Tensor:
        neighbours: Set[Point] = set(mp.get_neighbours(mp.agent.position))
        res: torch.Tensor = torch.zeros(8) if mp.size.n_dim == 2 else torch.zeros(26)
        for idx, mv in enumerate(mp.ALL_POINTS_MOVE_VECTOR):
            next_point: Point = Map.apply_move(mv, mp.agent.position)
            if next_point in neighbours:
                res[idx] = 1
        return res

    @staticmethod
    def local_map_normalized_feature(mp: Map) -> torch.Tensor:
        extents: int = 4
        dim = mp.size.n_dim

        if not isinstance(mp, DenseMap):
            raise Exception("mp has to be of type DenseMap")

        local_map: torch.Tensor = torch.zeros((extents * 2 + 1, extents * 2 + 1))
        local_map.fill_(DenseMap.WALL_ID)

        xx: int = 0
        for x in range(mp.agent.position.x - extents, mp.agent.position.x + extents + 1):
            yy: int = 0
            for y in range(mp.agent.position.y - extents, mp.agent.position.y + extents + 1):
                if not mp.is_out_of_bounds_pos(Point(x, y)):
                    local_map[yy][xx] = mp.grid[y][x]
                yy += 1
            xx += 1

        return MapProcessing.__convert_grid(local_map)

    @staticmethod
    def global_map_normalized_feature(mp: Map) -> torch.Tensor:
        if not isinstance(mp, DenseMap):
            raise Exception("mp has to be of type DenseMap")

        transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((64, 64)),
            transforms.ToTensor()
        ])

        return transform(MapProcessing.__convert_grid(copy.deepcopy(mp.grid))).ceil()

    """ End Features """

    """ Labels """

    @staticmethod
    def next_position_label(mp: Map) -> List[torch.Tensor]:
        return list(map(
            lambda el: mp.get_move_along_dir(
                MapProcessing.__get_pos(el[1]) - MapProcessing.__get_pos(el[0])).to_tensor(),
            zip(mp.trace, mp.trace[1:]))
        )

    @staticmethod
    def next_position_index_label(mp: Map) -> List[torch.Tensor]:
        move_indexes: List[int] = list(map(
            lambda el: mp.get_move_index(
                MapProcessing.__get_pos(el[1]) - MapProcessing.__get_pos(el[0])),
            zip(mp.trace, mp.trace[1:]))
        )
        res: List[torch.Tensor] = []
        for move_idx in move_indexes:
            res.append(torch.tensor(float(move_idx)))
        if len(res) == 0:
            res.append(torch.tensor(float(0)))
        return res

    """ End Labels """

    """ Helpers """

    @staticmethod
    def __get_hit_point_along_dir(mp: Map, move: Point) -> Point:
        pos: Point = mp.agent.position
        pos = mp.apply_move(pos, move)
        while mp.is_agent_valid_pos(pos):
            pos = mp.apply_move(pos, move)
        return pos

    @staticmethod
    def __get_pos(entity: Entity) -> torch.Tensor:
        return entity.position.to_tensor()

    @staticmethod
    def __convert_grid(grid: Union[List[List[int]], torch.Tensor]) -> torch.Tensor:
        """
        Converts grid to tensor and normalizes
        """

        max_value = 1.0

        for i in range(len(grid)):
            for j in range(len(grid[i])):
                if grid[i][j] == DenseMap.EXTENDED_WALL_ID:
                    grid[i][j] = DenseMap.WALL_ID

                # this condition is used if only obstacles are taken into consideration
                if grid[i][j] == DenseMap.AGENT_ID or grid[i][j] == DenseMap.GOAL_ID:
                    grid[i][j] = DenseMap.CLEAR_ID

                grid[i][j] = grid[i][j] / max_value

        return torch.Tensor(grid)

    """ End Helpers """
