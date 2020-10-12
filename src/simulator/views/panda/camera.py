from direct.showbase.ShowBaseGlobal import globalClock

class Camera:
    def __init__(self, base, cam):
        self.base = base
        self.cam = cam

        # disables the default camera behaviour
        self.base.disable_mouse()

        # EVENTS #
        # Use the scroll mouse button/touchpad to zoom in and out
        self.base.accept('wheel_up', lambda : self.cam.setY(self.cam.getY()+90 * globalClock.getDt()))
        self.base.accept('wheel_down', lambda : self.cam.setY(self.cam.getY()-90 * globalClock.getDt()))
        # Use the arrow/awsd keys to move left, right, up and down
        self.base.accept('a', lambda: self.cam.setH(self.cam.getH() + 90 * globalClock.getDt()))
        self.base.accept('a-repeat', lambda: self.cam.setH(self.cam.getH() + 90 * globalClock.getDt()))
        self.base.accept('d', lambda: self.cam.setH(self.cam.getH() - 90 * globalClock.getDt()))
        self.base.accept('d-repeat', lambda: self.cam.setH(self.cam.getH() - 90 * globalClock.getDt()))
        self.base.accept('arrow_left', lambda: self.cam.setH(self.cam.getH() + 90 * globalClock.getDt()))
        self.base.accept('arrow_left-repeat', lambda: self.cam.setH(self.cam.getH() + 90 * globalClock.getDt()))
        self.base.accept('arrow_right', lambda: self.cam.setH(self.cam.getH() - 90 * globalClock.getDt()))
        self.base.accept('arrow_right-repeat', lambda: self.cam.setH(self.cam.getH() - 90 * globalClock.getDt()))
        self.base.accept('w', lambda: self.cam.setP(self.cam.getP() + 90 * globalClock.getDt()))
        self.base.accept('w-repeat', lambda: self.cam.setP(self.cam.getP() + 90 * globalClock.getDt()))
        self.base.accept('s', lambda: self.cam.setP(self.cam.getP() - 90 * globalClock.getDt()))
        self.base.accept('s-repeat', lambda: self.cam.setP(self.cam.getP() - 90 * globalClock.getDt()))
        self.base.accept('arrow_up', lambda: self.cam.setP(self.cam.getP() + 90 * globalClock.getDt()))
        self.base.accept('arrow_up-repeat', lambda: self.cam.setP(self.cam.getP() + 90 * globalClock.getDt()))
        self.base.accept('arrow_down', lambda: self.cam.setP(self.cam.getP() - 90 * globalClock.getDt()))
        self.base.accept('arrow_down-repeat', lambda: self.cam.setP(self.cam.getP() - 90 * globalClock.getDt()))

    # add camera manipulation stuff