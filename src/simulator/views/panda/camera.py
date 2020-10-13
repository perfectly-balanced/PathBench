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
        self.base.accept('d', updateKeyMap, ["right", True])
        self.base.accept('d-up', updateKeyMap, ["right", False])
        self.base.accept('arrow_left', updateKeyMap, ["left", True])
        self.base.accept('arrow_left-up', updateKeyMap, ["left", False])
        self.base.accept('arrow_right', updateKeyMap, ["right", True])
        self.base.accept('arrow_right-up', updateKeyMap, ["right", False])
        self.base.accept('w', updateKeyMap, ["up", True])
        self.base.accept('w-up', updateKeyMap, ["up", False])
        self.base.accept('s', updateKeyMap, ["down", True])
        self.base.accept('s-up', updateKeyMap, ["down", False])
        self.base.accept('arrow_up', updateKeyMap, ["up", True])
        self.base.accept('arrow_up-up', updateKeyMap, ["up", False])
        self.base.accept('arrow_down', updateKeyMap, ["down", True])
        self.base.accept('arrow_down-up', updateKeyMap, ["down", False])
        # enabling rotation of the object
        # self.base.accept('space', updateKeyMap, ["rotate", True])
        # self.base.accept('space-up', updateKeyMap, ["rotate", False])
        self.speedL = 3
        self.speedR = 3
        self.speedU = 3
        self.speedD = 3
        self.angle = 0
        self.base.taskMgr.add(self.update, "update")

    def update(self, task):
        dt = globalClock.getDt()
        posH = self.cam.getH()
        posP = self.cam.getP()
        posY = self.cam.getY()
        if (keyMap["left"]):
            posH += self.speedL * dt
            if self.speedL < 120:
                self.speedL += 2
        else:
            self.speedL = 4

        if (keyMap["right"]):
            posH -= self.speedR * dt
            if self.speedR < 120:
                self.speedR += 2
        else:
            self.speedR = 4
        if (keyMap["up"]):
            posP += self.speedU * dt
            if self.speedU < 120:
                self.speedU += 2
        else:
            self.speedU = 4
        if (keyMap["down"]):
            posP -= self.speedD * dt
            if self.speedD < 120:
                self.speedD += 2
        else:
            self.speedD = 4
        # if (keyMap["rotate"]):
        #     self.angle += 1.5
        #     self.base.__map.setH(self.angle)

        self.cam.setH(posH)
        self.cam.setP(posP)
        self.cam.setY(posY)

        return task.cont
