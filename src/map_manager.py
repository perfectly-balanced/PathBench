from typing import Optional, List, Type, Dict, Any, Tuple
import importlib.util
import inspect
import os
import sys
import copy
import traceback

# builtin base map type
from algorithms.configuration.maps.map import Map

# Builtin maps
from maps import Maps

from simulator.services.services import Services

def static_class(cls):
    if getattr(cls, "_static_init_", None):
        cls._static_init_()
    return cls

@static_class
class MapManager():
    MetaData = Tuple[Type[Map], bool]

    cached_maps: Dict[str, MetaData]
    maps: Dict[str, MetaData]

    services: Optional[Services] = None

    @classmethod
    def _static_init_(cls):
        cls.cached_maps = {
            name: (getattr(Maps, name), True)
                for name in filter(
                    lambda x: not x.startswith("_"),
                    dir(Maps))
        }
        cls.maps = {
            "Uniform Random Fill": ("uniform_random_fill_10/0", True),
            "Uniform Random Fill 3D": ("uniform_random_fill_10_3d/0_3d", True),
            "Block": ("block_map_10/6", True),
            "Block 3D": ("block_map_10_3d/6_3d", True),
            "House": ("house_10/6", True),
            "House 3D": ("house_10_3d/6_3d", True),
            "Long Wall": (Maps.grid_map_one_obstacle1, True),
            "Labyrinth": (Maps.grid_map_labyrinth, True),
            "3D Cube": (Maps.grid_map_3d_example, True),
            "vin test 8x8": (Maps.grid_map_small_one_obstacle2, True),
            "vin test 8x8 -2": (Maps.grid_map_small_one_obstacle, True),
            "vin test 8x8 -3": (Maps.grid_map_small_one_obstacle3, True),
            "vin test 16x16 -1": (Maps.grid_map_complex_obstacle, True),
            "vin test 16x16 -2": (Maps.grid_map_complex_obstacle2, True),
            "vin test 28x28 -1": (Maps.grid_map_28x28vin, True),
            "Small Obstacle": (Maps.grid_map_one_obstacle.convert_to_dense_map(), True),
            "Occupancy Grid 2D": (Maps.ogm_2d, False),
            "Occupancy Grid 3D": (Maps.ogm_3d, False),
            "SLAM Map 1": ("map10", False),
            "SLAM Map 1 (compressed)": ("map11", True),
            "SLAM Map 2": ("map14", False),
            "SLAM Map 3": ("map12", False),
        }

    def __init__(self, services: Services):
        self.services = services

    def load_all(self, ids: List[str]) -> List[List[MetaData]]:
        """
        Returns a list of map classes from a list of names or file paths.

        For each element in `ids`, if string is the display name
        of a built-in map, then we return that map. Otherwise,
        we return the result of MapManager.try_load_from_file().
        """
        return list(map(self.load, ids))

    def load(self, map_: str) -> List[MetaData]:
        """
        Returns a list of map classes from one name or file path

        """

        if map_ in MapManager.cached_maps:
            return [copy.deepcopy(MapManager.cached_maps[map_])]
        elif map_ in MapManager.maps:
            map_data = MapManager.maps[map_]
            if isinstance(map_data[0], str):
                # Need to load
                if self.services is None:
                    print("Cannot load map from file without services", file=sys.stderr)
                    return []
                map_data = (self.services.resources.maps_dir.load(map_data[0]), map_data[1])
                MapManager.maps[map_] = map_data # Update static copy
            return [copy.deepcopy(map_data)]
        else:
            return MapManager.try_load_from_file(map_)

    @staticmethod
    def try_load_from_file(path: str) -> List[MetaData]:
        if not os.path.exists(path):
            msg = "File '{}' does not exist".format(path)
            print(msg, file=sys.stderr)
            return []

        try:
            spec = importlib.util.spec_from_file_location("custom_loaded", path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)

            # return all objects that subclass "Map"
            maps = []
            for name in dir(module):
                if name.startswith("_"):
                    continue

                map_ = getattr(module, name)
                if isinstance(map_, Map):
                    maps.append(map_)
            return maps
        except:
            msg = "Failed to load map from file '{}', reason:\n{}".format(path, traceback.format_exc())
            print(msg, file=sys.stderr)
            return []
