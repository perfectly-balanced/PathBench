from direct.showbase.ShowBaseGlobal import globalClock

key_map = {
    "up": False,
    "down": False,
    "left": False,
    "right": False
}

def update_key_map(key, state):
    key_map[key] = state

class Camera:
    def __init__(self, base, cam):
        self.base = base
        self.cam = cam

        # disables the default camera behaviour
        self.base.disable_mouse()

        # EVENTS #
        # Use the scroll mouse button/touchpad to zoom in and out
        self.base.accept('wheel_up', lambda: self.cam.setY(self.cam.getY() + 90 * globalClock.getDt()))
        self.base.accept('wheel_down', lambda: self.cam.setY(self.cam.getY() - 90 * globalClock.getDt()))
        # Use the arrow/awsd keys to move left, right, up and down
        self.base.accept('a', update_key_map, ["left", True])
        self.base.accept('a-up', update_key_map, ["left", False])
        self.base.accept('d', update_key_map, ["right", True])
        self.base.accept('d-up', update_key_map, ["right", False])
        self.base.accept('arrow_left', update_key_map, ["left", True])
        self.base.accept('arrow_left-up', update_key_map, ["left", False])
        self.base.accept('arrow_right', update_key_map, ["right", True])
        self.base.accept('arrow_right-up', update_key_map, ["right", False])
        self.base.accept('w', update_key_map, ["up", True])
        self.base.accept('w-up', update_key_map, ["up", False])
        self.base.accept('s', update_key_map, ["down", True])
        self.base.accept('s-up', update_key_map, ["down", False])
        self.base.accept('arrow_up', update_key_map, ["up", True])
        self.base.accept('arrow_up-up', update_key_map, ["up", False])
        self.base.accept('arrow_down', update_key_map, ["down", True])
        self.base.accept('arrow_down-up', update_key_map, ["down", False])

        self.speed_l = 4
        self.speed_r = 4
        self.speed_u = 4
        self.speed_d = 4
        self.angle = 0
        self.base.taskMgr.add(self.update, "update")

    def update(self, task):
        dt = globalClock.getDt()
        pos_h = self.cam.getH()
        pos_p = self.cam.getP()
        pos_y = self.cam.getY()
        if (key_map["left"]):
            pos_h += self.speed_l * dt
            if self.speed_l < 150:
                self.speed_l += 2
        else:
            self.speed_l = 4

        if (key_map["right"]):
            pos_h -= self.speed_r * dt
            if self.speed_r < 150:
                self.speed_r += 2
        else:
            self.speed_r = 4

        if (key_map["up"]):
            pos_p += self.speed_u * dt
            if self.speed_u < 150:
                self.speed_u += 2
        else:
            self.speed_u = 4
        if (key_map["down"]):
            pos_p -= self.speed_d * dt
            if self.speed_d < 150:
                self.speed_d += 2
        else:
            self.speed_d = 4

        self.cam.setH(pos_h)
        self.cam.setP(pos_p)
        self.cam.setY(pos_y)

        return task.cont
