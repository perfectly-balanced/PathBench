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

import math
from typing import Optional

class MapController(Controller, DirectObject):
    __picker: Optional[CubeMapPicker]
    __camera: Optional[CameraController]

    def __init__(self, map_view: MapView, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.__picker = CubeMapPicker(self._services.graphics.window, map_view.map.traversables, map_view.map.traversables_data)
        self.__camera = CameraController(self._services, self._model, origin=map_view.world)

        def left_click():
            p = self.__picker.pos
            if p != None:
                self._services.debug.write("Moved agent to: " + str(p), DebugLevel.MEDIUM)
                if self._services.algorithm.map.size.n_dim == 2:
                    p = Point(p[0], p[1])
                self._model.move(p)

        def right_click():
            p = self.__picker.pos
            if p != None:
                self._services.debug.write("Moved goal to: " + str(p), DebugLevel.MEDIUM)
                if self._services.algorithm.map.size.n_dim == 2:
                    p = Point(p[0], p[1])
                self._model.move_goal(p)

        self.accept('mouse1', left_click)
        self.accept('mouse3', right_click)
        self.accept("c", lambda: self._model.compute_trace())
        self.accept("m", lambda: self._model.toggle_convert_map())
        self.accept("x", lambda: self._model.toggle_pause_algorithm())
        self.accept("p", lambda: self._services.ev_manager.post(TakeScreenshotEvent()))

    def destroy(self) -> None:
        self.ignore_all()
        self.__camera.destroy()
        self.__picker.destroy()
