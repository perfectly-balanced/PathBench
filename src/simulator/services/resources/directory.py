from io import StringIO
import os
import shutil
import json

import numpy as np

import dill as pickle  # used to pickle lambdas
from typing import BinaryIO, Callable, Any

from algorithms.configuration.maps.dense_map import DenseMap

from simulator.services.debug import DebugLevel
from simulator.services.service import Service
from simulator.services.services import Services
from simulator.services.resources.smart_unpickle import load as smart_load

class Directory(Service):
    _name: str
    _parent: str

    def __init__(self, services: Services, name: str, parent: str, create: bool = False,
                 overwrite: bool = True) -> None:  # Toggle overwrite: to True or False to enable overwriting of generated maps
        super().__init__(services)

        if name:
            name += "" if name[-1] == "/" else "/"

        self._name = name
        self._parent = parent

        if create:
            if overwrite and os.path.exists(self._full_path()):
                shutil.rmtree(self._full_path())

            if not os.path.exists(self._full_path()):
                os.makedirs(self._full_path())
            else:
                raise Exception("Directory already exists")

    def name_without_trailing_slash(self):
        if self._name:
            return self._name[:-1]
        return self._name

    @staticmethod
    def _default_save(dir: 'Directory', name: str, obj: Any):
        Directory._pickle(obj, name, dir._full_path())

    @staticmethod
    def _default_load(dir: 'Directory', name: str) -> Any:
        fi = Directory._unpickle(name, dir._full_path())
        # Assuming this is a .json
        if fi is None:
            return Directory._unjson(name, dir._full_path())
        return fi

    def save(self, name: str, obj: Any, save_function: Callable[['Directory', str, Any], None] = None) -> None:
        if not save_function:
            save_function = Directory._default_save
        save_function(self, name, obj)
        self._services.debug.write("Saved [{}]".format(self._full_path() + name), DebugLevel.LOW)

    def load(self, name: str, load_function: Callable[['Directory', str], Any] = None) -> Any:
        if not load_function:
            load_function = Directory._default_load
        obj: Any = load_function(self, name)
        if (isinstance(obj, np.ndarray) and np.prod(obj.shape) > 0) or obj: # disambiguates boolean check for numpy arrays
            self._services.debug.write("Loaded [{}]".format(self._full_path() + name), DebugLevel.LOW)
        else:
            msg = "File not found [{}]. Consider deleting and re-generating".format(self._full_path() + name)
            self._services.debug.write(msg, DebugLevel.LOW)
        return obj

    def exists(self, name: str, extension: str) -> bool:
        return os.path.exists(self._full_path() + name + extension)

    @staticmethod
    def _pickle(obj: Any, file_name: str, directory: str) -> None:
        file_name = Directory._add_extension(file_name, "pickle")
        handle: BinaryIO
        with open(directory + file_name, 'wb') as handle:
            pickle.dump(obj, handle)

    @staticmethod
    def _unpickle(file_name: str, directory: str) -> Any:
        file_name = Directory._add_extension(file_name, "pickle")
        if not os.path.isfile(directory + file_name):
            return None

        return smart_load(directory + file_name)

    @staticmethod
    def _unjson(file_name: str, directory: str) -> Any:
        file_name = Directory._add_extension(file_name, "json")
        if not os.path.isfile(directory + file_name):
            return None

        dic = json.load(open(directory + file_name))

        if dic['type'] == "DenseMap" and dic['version'] <= 1:
            return DenseMap(dic['grid'])

        return None

    @staticmethod
    def _add_extension(file_name: str, extension: str) -> str:
        if len(file_name.split(".")) == 1:
            file_name += '.' + extension
        return file_name

    def _full_path(self) -> str:
        return self._parent + self._name

    def save_log(self, log: StringIO, name: str = None) -> None:
        if not name:
            name = self.name_without_trailing_slash() + "_log.txt"
        with open(self._full_path() + name, "w") as f:
            f.write(log.getvalue())
