from structures import DynamicColour, Colour, TRANSPARENT
from simulator.services.service import Service
from simulator.services.services import Services
from simulator.services.event_manager.events.colour_update_event import ColourUpdateEvent
from simulator.services.event_manager.events.new_colour_event import NewColourEvent
from simulator.services.persistent_state.persistent_state_object import PersistentStateObject
from simulator.services.debug import DebugLevel
from utility.misc import exclude_from_dict
from utility.compatibility import Final

from weakref import WeakSet
from typing import Dict, Any, List, Optional, Type, Tuple
import json
import os
import copy
import traceback

class PersistentState():
    all_services: WeakSet
    file_name: str

    __types: Dict[str, Tuple[Type[Any], Optional[str]]]
    __objects: List[PersistentStateObject]
    __save_task: Optional['Task']

    def __init__(self, services: Services, types: List[Tuple[Type[Any], Optional[str]]] = [], file_name: str = ".pathbench.json"):
        self.file_name = file_name
        self.__types = {t[0].__name__: t for t in types}

        self.all_services = WeakSet([services])
        self.__objects = []
        self.__save_task = None

        self.load()
    
    def add_services(self, services: Services):
        self.all_services.add(services)
    
    @property
    def root_services(self) -> Services:
        return next(iter(self.all_services))

    def load(self) -> None:
        if os.path.isfile(self.file_name):
            self.root_services.debug.write("Loading state from '{}'".format(self.file_name), DebugLevel.BASIC)
            with open(self.file_name, 'r') as f:
                try:
                    jdata = json.load(f)
                    jobjs = jdata["objects"]
                    for jo in jobjs:
                        cls, attr = self.__types[jo["type"]]
                        o = cls(self)
                        if attr:
                            setattr(self, attr, o)
                        o._from_json(jo["data"])
                        self.__objects.append(o)
                except:
                    msg = "Failed to load state data from '{}', reason:\n{}".format(self.file_name, traceback.format_exc())
                    self.root_services.debug.write(msg, DebugLevel.NONE)
                    self.save()
        else:
            self.root_services.debug.write("'{}' not found, falling back to default state data".format(self.file_name), DebugLevel.BASIC)
        
        do_save: bool = False
        for name, (cls, attr) in self.__types.items():
            if not attr:
                continue
            
            found: bool = False
            for o in self.objects:
                if type(o).__name__ == name:
                    found = True
                    do_save = True
                    break
            
            if not found:
                o = cls(self)
                setattr(self, attr, o)
                self.__objects.append(o)
        if do_save:
            self.save()

    def save(self) -> None:
        self.root_services.debug.write("Saving state to '{}'".format(self.file_name), DebugLevel.BASIC)
        data = {}
        data["objects"] = [{"type": type(o).__name__, "data": o._to_json()} for o in self.__objects]
        try:
            with open(self.file_name, 'w') as f:
                json.dump(data, f, sort_keys=True, indent=4)
        except:
            msg = "Failed to save state data to '{}', reason:\n{}State data attempted to be save:\n{}".format(self.file_name, traceback.format_exc(), data)
            self.root_services.debug.write(msg, DebugLevel.NONE)

        if self.__save_task is not None:
            self.root_services.graphics.window.taskMgr.remove(self.__save_task)
            self.__save_task = None

    def schedule_save(self, delay: float = 0) -> None:
        if delay < 0:
            return

        if delay > 0 and self.root_services.graphics is not None:
            tm = self.root_services.graphics.window.taskMgr
            if self.__save_task is None:
                self.root_services.debug.write("Scheduling state save to be executed {} seconds from now".format(delay), DebugLevel.BASIC)
                self.__save_task = tm.doMethodLater(delay, lambda _: self.save(), "save_persistent_state")
        else:
            self.save()

    def add(self, obj: PersistentStateObject, save_delay: float = 0) -> None:
        self.__objects.append(obj)
        self.schedule_save(save_delay)

    def remove(self, obj: PersistentStateObject, save_delay: float = 0) -> None:
        self.__objects.remove(obj)
        self.schedule_save(save_delay)

    @property
    def objects(self) -> List[PersistentStateObject]:
        return self.__objects.copy()
