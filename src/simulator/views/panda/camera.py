from direct.showbase.ShowBaseGlobal import globalClock

keyMap = {
    "up": False,
    "down": False,
    "left": False,
    "right": False
}


def updateKeyMap(key, state):
    keyMap[key] = state


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
        self.base.accept('a', updateKeyMap, ["left", True])
        self.base.accept('a-up', updateKeyMap, ["left", False])
        self.base.accept('d', lambda: self.cam.setH(self.cam.getH() - 90 * globalClock.getDt()))
        self.base.accept('d-up', lambda: self.cam.setH(self.cam.getH() - 90 * globalClock.getDt()))
        self.base.accept('arrow_left', lambda: self.cam.setH(self.cam.getH() + 90 * globalClock.getDt()))
        self.base.accept('arrow_left-up', lambda: self.cam.setH(self.cam.getH() + 90 * globalClock.getDt()))
        self.base.accept('arrow_right', lambda: self.cam.setH(self.cam.getH() - 90 * globalClock.getDt()))
        self.base.accept('arrow_right-up', lambda: self.cam.setH(self.cam.getH() - 90 * globalClock.getDt()))
        self.base.accept('w', lambda: self.cam.setP(self.cam.getP() + 90 * globalClock.getDt()))
        self.base.accept('w-up', lambda: self.cam.setP(self.cam.getP() + 90 * globalClock.getDt()))
        self.base.accept('s', lambda: self.cam.setP(self.cam.getP() - 90 * globalClock.getDt()))
        self.base.accept('s-up', lambda: self.cam.setP(self.cam.getP() - 90 * globalClock.getDt()))
        self.base.accept('arrow_up', lambda: self.cam.setP(self.cam.getP() + 90 * globalClock.getDt()))
        self.base.accept('arrow_up-up', lambda: self.cam.setP(self.cam.getP() + 90 * globalClock.getDt()))
        self.base.accept('arrow_down', lambda: self.cam.setP(self.cam.getP() - 90 * globalClock.getDt()))
        self.base.accept('arrow_down-up', lambda: self.cam.setP(self.cam.getP() - 90 * globalClock.getDt()))

        self.base.accept('space', lambda: self.cam.setP(self.cam.getP() - 90 * globalClock.getDt()))
        self.base.accept('space-repeat', lambda: self.cam.setP(self.cam.getP() - 90 * globalClock.getDt()))
        self.speed = 4
        self.angle = 0
        self.base.taskMgr.add(self.update, "update")

    # add camera manipulation stuff
    def update(self, task):
        dt = globalClock.getDt()
        pos = self.cam.getP()
        if (keyMap["left"]):
            pos -= self.speed * dt
        self.cam.setP(pos)
        return task.cont
