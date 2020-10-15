from direct.gui.OnscreenText import OnscreenText
from direct.showbase.ShowBaseGlobal import globalClock
from math import pi, sin, cos


key_map = {
    "up1": False,
    "down1": False,
    "left1": False,
    "right1": False,
    "left": 0,
    "right": 0,
    "up": 0,
    "down": 0,
    "wheelup":0,
    "wheeldown": 0
}

def update_key_map(key, state):
    key_map[key] = state

class Camera:
    def __init__(self, base, cam, world_origin):
        self.base = base
        self.cam = cam
        self.world_origin = world_origin

        # disables the default camera behaviour
        self.base.disable_mouse()

        # Pseudo-constants
        self.target_size = 6.37800000E+06
        self.renderRatio = 1.0e-6
        self.degPerSecond = 60.0
        self.minCameraDistance = 3.0
        self.maxCameraDistance = 4000.0
        self.zoomPerSecond = 2.1
        self.angle_longitude_degrees = 0.0
        self.angle_latitude_degrees = 0.0
        self.camera_distance = 10.0

        # EVENTS #

        # Use the arrow keys to move left, right, up and down
        self.base.accept('arrow_left', update_key_map, ["left1", True])
        self.base.accept('arrow_left-up', update_key_map, ["left1", False])
        self.base.accept('arrow_right', update_key_map, ["right1", True])
        self.base.accept('arrow_right-up', update_key_map, ["right1", False])
        self.base.accept('arrow_up', update_key_map, ["up1", True])
        self.base.accept('arrow_up-up', update_key_map, ["up1", False])
        self.base.accept('arrow_down', update_key_map, ["down1", True])
        self.base.accept('arrow_down-up', update_key_map, ["down1", False])

        # Setup down events for arrow keys : rotating camera latitude and longitude
        self.base.accept("a", update_key_map, ["left", 1])
        self.base.accept("d", update_key_map, ["right", 1])
        self.base.accept("w", update_key_map, ["up", 1])
        self.base.accept("s", update_key_map, ["down", 1])
        self.base.accept("a-up", update_key_map, ["left", 0])
        self.base.accept("d-up", update_key_map, ["right", 0])
        self.base.accept("w-up", update_key_map, ["up", 0])
        self.base.accept("s-up", update_key_map, ["down", 0])

        # Use the scroll mouse button/touchpad to zoom in and out
        self.base.accept("wheel_up", update_key_map, ["wheelup", 1])
        self.base.accept("wheel_down", update_key_map, ["wheeldown", 1])

        self.speed_l = 4
        self.speed_r = 4
        self.speed_u = 4
        self.speed_d = 4
        self.angle = 0


        self.base.taskMgr.add(self.move_orbital_camera_task, "moveOrbitalCameraTask")

        #self.base.taskMgr.add(self.update, "update")


    # Defines a procedure to move the camera by moving the world origin
    # Always keeps the camera oriented towards the world origin
    def move_orbital_camera_task(self, task):
        # First compute new camera angles and distance
        if key_map["left"] != 0:
            self.angle_longitude_degrees = self.angle_longitude_degrees - self.degPerSecond * globalClock.getDt()
        if key_map["right"] != 0:
            self.angle_longitude_degrees = self.angle_longitude_degrees + self.degPerSecond * globalClock.getDt()
        if key_map["up"] != 0:
            self.angle_latitude_degrees = self.angle_latitude_degrees - self.degPerSecond * globalClock.getDt()
        if key_map["down"] != 0:
            self.angle_latitude_degrees = self.angle_latitude_degrees + self.degPerSecond * globalClock.getDt()
        if key_map["wheelup"] != 0:
            self.camera_distance = self.camera_distance * (1 + (self.zoomPerSecond - 1) * globalClock.getDt())
            update_key_map("wheelup", 0)
        if key_map["wheeldown"] != 0:
            self.camera_distance = self.camera_distance / (1 + (self.zoomPerSecond - 1) * globalClock.getDt())
            update_key_map("wheeldown", 0)

        if self.angle_longitude_degrees > 180.0:
            self.angle_longitude_degrees = self.angle_longitude_degrees - 360.0
        if self.angle_longitude_degrees < -180.0:
            self.angle_longitude_degrees = self.angle_longitude_degrees + 360.0

        if self.camera_distance < self.minCameraDistance:
            self.camera_distance = self.minCameraDistance
        if self.camera_distance > self.maxCameraDistance:
            self.camera_distance = self.maxCameraDistance

        # Convert to Radians
        angle_longitude_radians = self.angle_longitude_degrees * (pi / 180.0)
        angle_latitude_radians = self.angle_latitude_degrees * (pi / 180.0)

        # Compute the target object's position with respect to the camera
        x = -self.camera_distance * self.target_size * sin(angle_longitude_radians) * cos(angle_latitude_radians)
        y = self.camera_distance * self.target_size * cos(angle_longitude_radians) * cos(angle_latitude_radians)
        z = self.camera_distance * self.target_size * sin(angle_latitude_radians)

        # Compute the world origin's position with respect to the camera
        x = (x * self.renderRatio)
        y = (y * self.renderRatio)
        z = (z * self.renderRatio)

        # Apply the position
        self.world_origin.setPos(x, y, z)

        # Rotate the camera
        self.cam.setHpr(self.angle_longitude_degrees, self.angle_latitude_degrees, 0)

        # End task
        return task.cont

    # Movement of the camera using the arrow keys
    def update(self, task):
        dt = globalClock.getDt()
        pos_h = self.cam.getH()
        pos_p = self.cam.getP()
        pos_y = self.cam.getY()
        if (key_map["left1"]):
            pos_h += self.speed_l * dt
            if self.speed_l < 150:
                self.speed_l += 2
        else:
            self.speed_l = 4

        if (key_map["right1"]):
            pos_h -= self.speed_r * dt
            if self.speed_r < 150:
                self.speed_r += 2
        else:
            self.speed_r = 4

        if (key_map["up1"]):
            pos_p += self.speed_u * dt
            if self.speed_u < 150:
                self.speed_u += 2
        else:
            self.speed_u = 4
        if (key_map["down1"]):
            pos_p -= self.speed_d * dt
            if self.speed_d < 150:
                self.speed_d += 2
        else:
            self.speed_d = 4

        self.cam.setH(pos_h)
        self.cam.setP(pos_p)
        self.cam.setY(pos_y)

        return task.cont

    # Displays position of the camera on the screen
    def show_cam_pos(self):
        x = self.cam.getX()
        y = self.cam.getY()
        z = self.cam.getZ()
        self.title = OnscreenText(text=str(x) + ":" + str(y) + ":" + str(z), style=1, fg=(1, 1, 0, 1), pos=(0, 0), scale=0.07)


