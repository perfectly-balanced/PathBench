from direct.gui.DirectGui import DirectFrame, DirectLabel, DirectEntry, DGG
from direct.showbase.ShowBase import ShowBase
from simulator.services.services import Services
from simulator.services.debug import DebugLevel
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import TextNode


class DebugOverlay():
    __services: Services
    __base: ShowBase

    def __init__(self, services: Services):
        self.__services = services
        self.__base = self.__services.graphics.window
        self.__labels = ["Map:", "Goal:", "Agent:", "Algorithm:", "State:"]

        self.__debug_labels = []
        for i in range(0, 5):
            l = OnscreenText(text=self.__labels[i],
                             parent=self.__base.aspect2d,
                             pos=(-1.45, 0.92 - i * 0.05),
                             mayChange=True,
                             align=TextNode.ALeft,
                             scale=0.03)
            self.set_text_colour(l, [255, 255, 255, 1])
            self.__debug_labels.append(l)

        self.__debug_var = []
        for i in range(0, 5):
            v = OnscreenText(text="..",
                             parent=self.__base.aspect2d,
                             pos=(-1.3, 0.92 - i * 0.05),
                             mayChange=True,
                             align=TextNode.ALeft,
                             scale=0.03)
            self.set_text_colour(v, [255, 255, 255, 1])
            self.__debug_var.append(v)

    def set_text_colour(self, txt, colour):
        txt.setFg(colour)

    # use setText(), getText(), clearText()
