from panda3d.core import *
from direct.gui.OnscreenImage import OnscreenImage
from direct.gui.DirectGui import *
from direct.showbase.ShowBase import ShowBase
from simulator.views.gui.common import WINDOW_BG_COLOUR, WIDGET_BG_COLOUR, Window
from simulator.services.services import Services
from structures import Colour, WHITE, BLACK, TRANSPARENT
import os
from constants import DATA_PATH

class SimulatorConfig():
    __services: Services
    __base: ShowBase
    __window: Window

    def __init__(self, services: Services):
        self.__services = services
        self.__services.ev_manager.register_listener(self)
        self.__base = self.__services.graphics.window
        self.hidden_config = False

        self.maps = ["Labyrinth", "Uniform Random Fill", "Block", "House", "Long Wall",
                     "Labyrinth","vin test 8x8","vin test 8x8 -2","vin test 8x8 -3",
                     "vin test 16x16 -1", "vin test 16x16 -2", "vin test 28x28 -1",
                     "Small Obstacle", "SLAM Map 1", "SLAM Map 1 (compressed)", "SLAM Map 2", "SLAM Map 3"]

        self.algorithms = ["OMPL", "A*", "Global Way-point LSTM", "LSTM Bagging", "CAE Online LSTM",
                           "Online LSTM", "SPRM", "RT", "RRT", "RRT*", "RRT-Connect", "Wave-front", "Dijkstra",
                           "Bug1", "Bug2", "Potential Field", "VIN"]

        self.animations = ["None","Normal", "Slow", "Fast"]

        self.__window_config = Window(self.__base, "simulator_config", parent=self.__base.pixel2d,
                               relief=DGG.RAISED,
                               borderWidth=(0.0, 0.0),
                               frameColor=WINDOW_BG_COLOUR,
                               pos=(190, 200, -350),
                               scale=(150, 1., 150),
                               frameSize=(-1.2, 1.2, -4, 1.1))

        DirectFrame(parent=self.__window_config.frame,
                    borderWidth=(.0, .0),
                    frameColor=WIDGET_BG_COLOUR,
                    frameSize=(-1., 1., -0.01, 0.01),
                    pos=(0.0, 0.0, 0.4))

        self.heading_config = DirectLabel(parent=self.__window_config.frame,
                                          text="PathBench",
                                          text_fg=WHITE,
                                          text_bg=WINDOW_BG_COLOUR,
                                          borderWidth=(.0, .0),
                                          pos=(0.0, 0.0, 0.8),
                                          scale=(0.2, 3, 0.2))


        self.heading_config = DirectLabel(parent=self.__window_config.frame,
                                   text="Simulator Configuration",
                                   text_fg=WHITE,
                                   text_bg=WINDOW_BG_COLOUR,
                                   borderWidth=(.0, .0),
                                   pos=(0.0, 0.0, 0.56),
                                   scale=(0.2, 3, 0.18))

        # Quit button
        self.btn = DirectButton(image=os.path.join(DATA_PATH, "quit.png"),
                                # command=self.__toggle_config(),
                                pos=(1., 0.4, 0.86),
                                parent=self.__window_config.frame,
                                scale=0.1,
                                pressEffect=1,
                                frameColor=TRANSPARENT)

        self.options = DirectOptionMenu(text="options",
                                        scale=0.14,
                                        parent=self.__window_config.frame,
                                        initialitem=1,
                                        items=self.maps,
                                        pos=(-0.65, 0.4, 0.1),
                                        highlightColor=(0.65, 0.65, 0.65, 1),
                                        textMayChange=1)

        self.options = DirectOptionMenu(text="options",
                                        scale=0.14,
                                        parent=self.__window_config.frame,
                                        initialitem=1,
                                        items=self.algorithms,
                                        pos=(-0.65, 0.4, -1),
                                        highlightColor=(0.65, 0.65, 0.65, 1),
                                        textMayChange=1)

        self.options = DirectOptionMenu(text="options",
                                        scale=0.14,
                                        parent=self.__window_config.frame,
                                        initialitem=0,
                                        items=self.animations,
                                        pos=(-0.65, 0.4, -2),
                                        highlightColor=(0.65, 0.65, 0.65, 1),
                                        textMayChange=1)

        self.__start_simulator = DirectFrame(parent=self.__window_config.frame,
                                          frameColor=WHITE,
                                          pos=(-0.34, 0.4, -3),
                                          borderWidth=(0.25, 0.15),
                                          frameSize=(-0.69, 2.1, -0.54, 0.54),
                                          scale=(0.50, 3.1, 0.25))
        self.btn_s = DirectButton(
            text="Start Simulator",
            text_fg=(0.3, 0.3, 0.3, 1.0),
            pressEffect=1,
            # command=,
            pos=(0, 0.4, -3.03),
            parent=self.__window_config.frame,
            scale=(0.20, 2.1, 0.15),
            frameColor=TRANSPARENT)

    def __toggle_config(self):
        if not self.hidden_config:
            self.__window_config.frame.hide()
            self.hidden_config = True
        else:
            self.__window_config.frame.show()
            self.hidden_config = False


    def notify(self, event: Event) -> None:
        print("notify")
