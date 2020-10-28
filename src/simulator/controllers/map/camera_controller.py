import math

from direct.showbase.ShowBaseGlobal import globalClock
from direct.showbase.DirectObject import DirectObject
from direct.showbase.ShowBase import ShowBase
from math import pi, sin, cos

from utility.utils import exclude_from_dict
from simulator.controllers.controller import Controller


class CameraController(Controller, DirectObject):
    __base: ShowBase

    def __init__(self, *args, **kwargs):
        Controller.__init__(self, *args, **exclude_from_dict(kwargs, ["camera", "origin"]))

        self.__base = self._services.graphics.window
        self.__cam = kwargs["camera"] if "camera" in kwargs else self.__base.cam
        self.__origin = kwargs["origin"]

        self.target_size = 6.37800000E+06
        self.render_ratio = 1.0e-6
        self.deg_per_sec = 60.0
        self.min_dist = 3.0
        self.max_dist = 4000.0
        self.zoom_per_sec = 4
        self.longitude_deg = 0.0
        self.latitude_deg = 0.0
        self.dist = 10.0

        self.key_map = {
            "up1": 0,
            "down1": 0,
            "left1": 0,
            "right1": 0,
            "left": 0,
            "right": 0,
            "up": 0,
            "down": 0,
            "wheelup": 0,
            "wheeldown": 0,
            "left_rot": 0,
            "right_rot": 0,
            "up_tilt": 0,
            "down_tilt": 0,
        }

        self.speed = 40
        self.angle = 0

        # EVENTS #

        # disables the default camera behaviour
        self.__base.disable_mouse()
        # self.__cam.lookAt(self.__origin)

        # Use the arrow keys to move left, right, up and down
        self.accept('arrow_left', self.update_key_map, ["left1", 1])
        self.accept('arrow_left-up', self.update_key_map, ["left1", 0])
        self.accept('arrow_right', self.update_key_map, ["right1", 1])
        self.accept('arrow_right-up', self.update_key_map, ["right1", 0])
        self.accept('arrow_up', self.update_key_map, ["up1", 1])
        self.accept('arrow_up-up', self.update_key_map, ["up1", 0])
        self.accept('arrow_down', self.update_key_map, ["down1", 1])
        self.accept('arrow_down-up', self.update_key_map, ["down1", 0])

        # Setup down events for arrow keys : rotating camera latitude and longitude
        self.accept("a", self.update_key_map, ["left_rot", 1])
        self.accept("d", self.update_key_map, ["right_rot", 1])
        self.accept("w", self.update_key_map, ["up_tilt", 1])
        self.accept("s", self.update_key_map, ["down_tilt", 1])
        self.accept("a-up", self.update_key_map, ["left_rot", 0])
        self.accept("d-up", self.update_key_map, ["right_rot", 0])
        self.accept("w-up", self.update_key_map, ["up_tilt", 0])
        self.accept("s-up", self.update_key_map, ["down_tilt", 0])

        # # Orbital movement
        # self.accept("f", self.update_key_map, ["left", 1])
        # self.accept("h", self.update_key_map, ["right", 1])
        # self.accept("t", self.update_key_map, ["up", 1])
        # self.accept("g", self.update_key_map, ["down", 1])
        # self.accept("f-up", self.update_key_map, ["left", 0])
        # self.accept("h-up", self.update_key_map, ["right", 0])
        # self.accept("t-up", self.update_key_map, ["up", 0])
        # self.accept("g-up", self.update_key_map, ["down", 0])

        # Use the scroll mouse button/touchpad to zoom in and out
        self.accept("wheel_up", self.update_key_map, ["wheelup", 1])
        self.accept("wheel_down", self.update_key_map, ["wheeldown", 1])

        self.__base.taskMgr.add(self.move_camera_task, "move_orbital_camera_task")

    def update_key_map(self, key: str, state: int) -> None:
        self.key_map[key] = state

    def move_camera_task(self, task):

        if self.is_orbital():
            # First compute new camera angles and distance
            if self.key_map["left"] != 0:
                self.longitude_deg = self.longitude_deg - self.deg_per_sec * globalClock.getDt()
            if self.key_map["right"] != 0:
                self.longitude_deg = self.longitude_deg + self.deg_per_sec * globalClock.getDt()
            if self.key_map["up"] != 0:
                self.latitude_deg = self.latitude_deg - self.deg_per_sec * globalClock.getDt()
            if self.key_map["down"] != 0:
                self.latitude_deg = self.latitude_deg + self.deg_per_sec * globalClock.getDt()
            # if self.key_map["wheelup"] != 0:
            #     self.dist = self.dist * (1 + (self.zoom_per_sec - 1) * globalClock.getDt())
            #    #   pos_y = self.__cam.getY()
            #    #   pos_y -= self.speed * dt
            #    #
            #    #   self.__cam.setY(pos_y)
            #     self.update_key_map("wheelup", 0)
            # if self.key_map["wheeldown"] != 0:
            #     # pos_y = self.__cam.getY()
            #     # pos_y += self.speed * dt
            #     # self.__cam.setY(pos_y)
            #     self.dist = self.dist / (1 + (self.zoom_per_sec - 1) * globalClock.getDt())
            #     self.update_key_map("wheeldown", 0)

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
        else:
            dt = globalClock.getDt()
            pos_h = self.__cam.getH()
            pos_p = self.__cam.getP()

            # alter the angle
            if (self.key_map["left_rot"]):
                pos_h += self.speed * dt
                self.__cam.setH(pos_h)
            if (self.key_map["right_rot"]):
                pos_h -= self.speed * dt
                self.__cam.setH(pos_h)
            if (self.key_map["up_tilt"]):
                pos_p += self.speed * dt
                self.__cam.setP(pos_p)
            if (self.key_map["down_tilt"]):
                pos_p -= self.speed * dt
                self.__cam.setP(pos_p)

            # move horizontally or vertically
            if (self.key_map["up1"]):
                self.__cam.setZ(self.__cam, 1.5)

            if (self.key_map["down1"]):
                self.__cam.setZ(self.__cam, -1.5)

            if (self.key_map["left1"]):
                self.__cam.setX(self.__cam, -1.5)

            if (self.key_map["right1"]):
                self.__cam.setX(self.__cam, 1.5)

            # zoom in direction of the camera
            if (self.key_map["wheelup"]):
                self.__cam.setY(self.__cam, -6)
                self.update_key_map("wheelup", 0)
            if (self.key_map["wheeldown"]):
                self.__cam.setY(self.__cam, 6)
                self.update_key_map("wheeldown", 0)

        return task.cont

    def destroy(self) -> None:
        self.ignore_all()

    def is_orbital(self):
        return self.key_map["left"] != 0 or self.key_map["right"] != 0 or self.key_map["up"] != 0 or self.key_map[
            "down"] != 0
