from direct.gui.OnscreenText import OnscreenText
from direct.showbase.ShowBaseGlobal import globalClock
from direct.showbase.DirectObject import DirectObject
from direct.showbase.ShowBase import ShowBase
from panda3d.core import NodePath
from panda3d.core import Camera
from math import pi, sin, cos
from utility.misc import exclude_from_dict
from simulator.controllers.controller import Controller


class CameraController(Controller, DirectObject):
    __base: ShowBase

    def __init__(self, *args, **kwargs):
        Controller.__init__(self, *args, **exclude_from_dict(kwargs, ["camera", "origin"]))

        self.__base = self._services.graphics.window
        self.__cam = kwargs["camera"] if "camera" in kwargs else self.__base.cam
        self.__origin = kwargs["origin"]

        self.camNode = Camera('cam')
        self.camNP = NodePath(self.camNode)
        self.camNP.reparentTo(self.__origin)

        self.lens = self.__base.camNode.getLens()
        self.target_size = 6.37800000E+06
        self.render_ratio = 1.0e-6
        self.deg_per_sec = 60.0
        self.min_dist = 3.0
        self.max_dist = 4000.0
        self.zoom_per_sec = 4
        self.longitude_deg = 0.0
        self.latitude_deg = -25
        self.dist = 10.0
        self.key_map = {
            "left": 0,
            "right": 0,
            "up": 0,
            "down": 0,
            "wheelup": 0,
            "wheeldown": 0,
            "top_view": 0
        }

        self.angle = 0

        # EVENTS #

        # disables the default camera behaviour
        self.__base.disable_mouse()

        # Setup down events for arrow keys : rotating camera latitude and longitude
        self.accept("a", self.update_key_map, ["left", 1])
        self.accept("d", self.update_key_map, ["right", 1])
        self.accept("w", self.update_key_map, ["up", 1])
        self.accept("s", self.update_key_map, ["down", 1])
        self.accept("q", self.update_key_map, ["top_view", 1])
        self.accept("a-up", self.update_key_map, ["left", 0])
        self.accept("d-up", self.update_key_map, ["right", 0])
        self.accept("w-up", self.update_key_map, ["up", 0])
        self.accept("s-up", self.update_key_map, ["down", 0])

        # Use the scroll mouse button/touchpad to zoom in and out
        self.accept("wheel_up", self.update_key_map, ["wheelup", 1])
        self.accept("wheel_down", self.update_key_map, ["wheeldown", 1])

        self.__task = self.__base.taskMgr.add(self.move_orbital_camera_task, "move_orbital_camera_task")

    def update_key_map(self, key: str, state: int) -> None:
        self.key_map[key] = state

    # Defines a procedure to move the camera by moving the world origin
    # Always keeps the camera oriented towards the world origin
    def move_orbital_camera_task(self, task):
        # First compute new camera angles and distance
        if self.key_map["left"] != 0:
            self.longitude_deg = self.longitude_deg - self.deg_per_sec * globalClock.getDt()
        if self.key_map["right"] != 0:
            self.longitude_deg = self.longitude_deg + self.deg_per_sec * globalClock.getDt()
        if self.key_map["up"] != 0:
            self.latitude_deg = self.latitude_deg - self.deg_per_sec * globalClock.getDt()
        if self.key_map["down"] != 0:
            self.latitude_deg = self.latitude_deg + self.deg_per_sec * globalClock.getDt()
        if self.key_map["wheelup"] != 0:
            self.dist = self.dist * (1 + (self.zoom_per_sec - 1) * globalClock.getDt())
            self.update_key_map("wheelup", 0)
        if self.key_map["wheeldown"] != 0:
            self.dist = self.dist / (1 + (self.zoom_per_sec - 1) * globalClock.getDt())
            self.update_key_map("wheeldown", 0)
        if self.key_map["top_view"] != 0:
            self.latitude_deg = -90
            self.longitude_deg = 0
            self.update_key_map("top_view", 0)
        if self.longitude_deg > 180.0:
            self.longitude_deg = self.longitude_deg - 360.0
        if self.longitude_deg < -180.0:
            self.longitude_deg = self.longitude_deg + 360.0

        if self.dist < self.min_dist:
            self.dist = self.min_dist
        if self.dist > self.max_dist:
            self.dist = self.max_dist

        # Convert to Radians
        angle_longitude_radians = self.longitude_deg * (pi / 180.0)
        angle_latitude_radians = self.latitude_deg * (pi / 180.0)

        # Compute the target object's position with respect to the camera
        x = -self.dist * self.target_size * sin(angle_longitude_radians) * cos(angle_latitude_radians)
        y = self.dist * self.target_size * cos(angle_longitude_radians) * cos(angle_latitude_radians)
        z = self.dist * self.target_size * sin(angle_latitude_radians)

        # Compute the world origin's position with respect to the camera
        x = (x * self.render_ratio)
        y = (y * self.render_ratio)
        z = (z * self.render_ratio)

        # Apply the position
        self.__origin.set_pos(x, y, z)

        # Rotate the camera
        self.__cam.set_hpr(self.longitude_deg, self.latitude_deg, 0)

        # End task
        return task.cont

    def destroy(self) -> None:
        self.ignore_all()
        self.__task.remove()
        self._services.ev_manager.unregister_listener(self)

    def top_view(self):
        self.__origin.setY(-0.3)
        self.__origin.setZ(-63.7)
        self.__cam.setH(0)
        self.__cam.setP(-88.62)
