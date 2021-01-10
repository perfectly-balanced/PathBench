from direct.showbase.DirectObject import DirectObject

from structures import Point
from simulator.services.debug import DebugLevel
from simulator.views.map.map_view import MapView
from simulator.controllers.controller import Controller
from simulator.controllers.map.map_picker import MapPicker
from simulator.controllers.map.camera_controller import CameraController
from simulator.services.event_manager.events.take_map_screenshot_event import TakeMapScreenshotEvent
from simulator.services.event_manager.events.state_running_event import StateRunningEvent

import math
from typing import Optional
from functools import partial

class MapController(Controller, DirectObject):
    __picker: MapPicker
    __camera: Optional[CameraController]
    __position_updating_enabled: bool

    def __init__(self, map_view: MapView, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.__picker = MapPicker(self._services, self._services.graphics.window, map_view.map)
        self.__camera = CameraController(self._services, self._model, origin=map_view.world)
        self.__position_updating_enabled = True

        def do_agent_position_update():
            # allow external updates of position when algorithm not running
            if self.__position_updating_enabled and self._services.settings.get_agent_position:
                p = self._services.settings.get_agent_position()
                if p != self._services.algorithm.map.agent.position:
                    self._model.move(p)

        def agent_position_update(task):
            do_agent_position_update()
            return task.cont

        self._services.graphics.window.taskMgr.add(agent_position_update, 'agent_position_update')

        def left_click():
            if self._services.settings.get_agent_position:
                return  # agent is externally set, cannot be set by visualiser

            p = self.__picker.pos
            if p != None:
                self._services.debug.write("Moved agent to: " + str(p), DebugLevel.MEDIUM)
                if self._services.algorithm.map.size.n_dim == 2:
                    p = Point(p[0], p[1])
                self._model.move(p)

        def right_click():
            p = self.__picker.pos
            if p != None:
                self.__position_updating_enabled = True
                self._services.debug.write("Moved goal to: " + str(p), DebugLevel.MEDIUM)
                if self._services.algorithm.map.size.n_dim == 2:
                    p = Point(p[0], p[1])
                self._model.move_goal(p)

        def set_view(i):
            self._services.state.views.view_idx = i

        def compute_trace():
            do_agent_position_update()
            self.__position_updating_enabled = False
            self._services.ev_manager.broadcast(StateRunningEvent())
            self._model.compute_trace()

        self.accept('mouse1', left_click)
        self.accept('mouse3', right_click)
        self.accept("m", lambda: self._model.toggle_convert_map())
        self.accept("x", lambda: self._model.toggle_pause_algorithm())
        self.accept("o", lambda: self._services.ev_manager.post(TakeMapScreenshotEvent()))
        self.accept("t", compute_trace)

        # Moves agent and goal entities with keys
        self.accept("arrow_right", self._model.move_right, [self._services.algorithm.map.agent])
        self.accept("arrow_right-repeat", self._model.move_right, [self._services.algorithm.map.agent])
        self.accept("alt-arrow_right", self._model.move_right, [self._services.algorithm.map.goal])
        self.accept("alt-arrow_right-repeat", self._model.move_right, [self._services.algorithm.map.goal])
        self.accept("arrow_left", self._model.move_left, [self._services.algorithm.map.agent])
        self.accept("arrow_left-repeat", self._model.move_left, [self._services.algorithm.map.agent])
        self.accept("alt-arrow_left", self._model.move_left, [self._services.algorithm.map.goal])
        self.accept("alt-arrow_left-repeat", self._model.move_left, [self._services.algorithm.map.goal])
        self.accept("arrow_up", self._model.move_forward, [self._services.algorithm.map.agent])
        self.accept("arrow_up-repeat", self._model.move_forward, [self._services.algorithm.map.agent])
        self.accept("alt-arrow_up", self._model.move_forward, [self._services.algorithm.map.goal])
        self.accept("alt-arrow_up-repeat", self._model.move_forward, [self._services.algorithm.map.goal])
        self.accept("arrow_down", self._model.move_backwards, [self._services.algorithm.map.agent])
        self.accept("arrow_down-repeat", self._model.move_backwards, [self._services.algorithm.map.agent])
        self.accept("alt-arrow_down", self._model.move_backwards, [self._services.algorithm.map.goal])
        self.accept("alt-arrow_down-repeat", self._model.move_backwards, [self._services.algorithm.map.goal])
        self.accept("page_up", self._model.move_up, [self._services.algorithm.map.agent])
        self.accept("page_up-repeat", self._model.move_up, [self._services.algorithm.map.agent])
        self.accept("alt-page_up", self._model.move_up, [self._services.algorithm.map.goal])
        self.accept("alt-page_up-repeat", self._model.move_up, [self._services.algorithm.map.goal])
        self.accept("page_down", self._model.move_down, [self._services.algorithm.map.agent])
        self.accept("page_down-repeat", self._model.move_down, [self._services.algorithm.map.agent])
        self.accept("alt-page_down", self._model.move_down, [self._services.algorithm.map.goal])
        self.accept("alt-page_down-repeat", self._model.move_down, [self._services.algorithm.map.goal])

        for i in range(6):
            self.accept(str(i+1), partial(set_view, i))

        # The following is for debugging dynamic growth

        self.map_2d = [[0, 0, 0.2, 0.3, 0.4, 0.8, 1, 1, 0.8, 0, 0, 1, 1, 0, 0, 0, 0, 0.9],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5]] + \
            [[-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1] for _ in range(40)]

        self.map_3d = [[[0, 0, 0.2, 0.3, 0.4, 0.8, 1, 1, 0.8, 0, 0, 1, 1, 0, 0, 0, 0, 0.9],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5]]] + \
            [[[-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
                [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]] for _ in range(40)]

        def grow_map():
            try:
                from algorithms.configuration.maps.occupancy_grid_map import OccupancyGridMap
                if not isinstance(self._services.algorithm.map, OccupancyGridMap):
                    return
                if self._services.algorithm.map.size.n_dim == 2:
                    self.map_2d[0][1] = 1  # test traversable -> obstacle
                    self.map_2d[0][6] = 0  # test obstacle -> traversable
                    self.map_2d[0][7] = -1  # test obstacle -> unmapped
                    self.map_2d[0][8] = -1  # test traversable -> unmapped
                    for i in range(len(self.map_2d)):
                        if self.map_2d[i][0] == -1:
                            self.map_2d[i] = self.map_2d[0]
                            self.map_2d[i+1] = self.map_2d[1]
                            self._services.algorithm.map.set_grid(self.map_2d, unmapped_value=-1)
                            return
                else:
                    self.map_3d[0][0][1] = 1  # test traversable -> obstacle
                    self.map_3d[0][0][6] = 0  # test obstacle -> traversable
                    self.map_3d[0][0][7] = -1  # test obstacle -> unmapped
                    self.map_3d[0][0][8] = -1  # test traversable -> unmapped
                    for i in range(len(self.map_2d)):
                        if self.map_3d[i][0][0] == -1:
                            self.map_3d[i] = self.map_3d[0]
                            self.map_3d[i+1] = self.map_3d[1]
                            self._services.algorithm.map.set_grid(self.map_3d, unmapped_value=-1)
                            return
            except:
                pass

        self.accept("h", grow_map)

    def destroy(self) -> None:
        self.ignore_all()
        self.__camera.destroy()
        self.__picker.destroy()
        self._services.ev_manager.unregister_listener(self)
