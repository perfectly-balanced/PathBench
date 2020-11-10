from panda3d.core import CollisionTraverser, CollisionHandlerQueue, CollisionNode, BitMask32, CollisionBox, CollisionRay, Point3
from direct.showbase.ShowBase import ShowBase

from direct.showbase.DirectObject import DirectObject
from structures import Point
from simulator.services.debug import DebugLevel
from simulator.views.map.map_view import MapView
from simulator.controllers.controller import Controller
from simulator.controllers.map.cube_map_picker import CubeMapPicker
from simulator.controllers.map.camera_controller import CameraController
from simulator.services.event_manager.events.take_screenshot_event import TakeScreenshotEvent
from simulator.services.event_manager.events.take_screenshot_tex_event import TakeScreenshotTexEvent

import math
from typing import Optional
from functools import partial

class MapController(Controller, DirectObject):
    __picker: Optional[CubeMapPicker]
    __camera: Optional[CameraController]

    def __init__(self, map_view: MapView, *args, **kwargs):
        super().__init__(*args, **kwargs)

        if map_view.map.dim == 3:
            self.__picker = CubeMapPicker(self._services.graphics.window, map_view.map.traversables, map_view.map.traversables_data)
        else:
            self.__picker = None

        self.__camera = CameraController(self._services, self._model, origin=map_view.world)

        def left_click():
            p = self.__picker.pos
            if p != None:
                self._services.debug.write("Moved agent to: " + str(p), DebugLevel.MEDIUM)
                if self._services.algorithm.map.size.n_dim == 2:
                    p = Point(p[0], p[1])
                self._services.lock.acquire()
                self._model.move(p, refresh=True)
                self._services.lock.release()

        def right_click():
            p = self.__picker.pos
            if p != None:
                self._services.debug.write("Moved goal to: " + str(p), DebugLevel.MEDIUM)
                if self._services.algorithm.map.size.n_dim == 2:
                    p = Point(p[0], p[1])
                self._services.lock.acquire()
                self._model.move_goal(p, refresh=True)
                self._services.lock.release()

        def set_view(i):
            self._services.state.view_idx = i

        self.accept('mouse1', left_click)
        self.accept('mouse3', right_click)
        self.accept("t", lambda: self._model.compute_trace())
        self.accept("m", lambda: self._model.toggle_convert_map())
        self.accept("x", lambda: self._model.toggle_pause_algorithm())
        self.accept("p", lambda: self._services.ev_manager.post(TakeScreenshotEvent()))
        self.accept("o", lambda: self._services.ev_manager.post(TakeScreenshotTexEvent()))


        for i in range(6):
            self.accept(str(i+1), partial(set_view, i))

    def destroy(self) -> None:
        self.ignore_all()
        self.__camera.destroy()
        if self.__picker is not None:
            self.__picker.destroy()
        self._services.ev_manager.unregister_listener(self)
