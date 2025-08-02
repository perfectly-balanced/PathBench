from panda3d.core import PNMImage, TextNode
from direct.gui.DirectGui import DirectFrame, DirectButton, DirectLabel, DirectEntry, DGG, DirectOptionMenu
from direct.showbase.ShowBase import ShowBase
from direct.showbase.DirectObject import DirectObject

import numpy as np

from typing import Tuple, Union, List, Any, Dict, Callable, Type
import traceback
from direct.gui.OnscreenText import OnscreenText

from structures import Point, WHITE, TRANSPARENT

from simulator.services.services import Services
from simulator.services.debug import DebugLevel
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.reset_event import ResetEvent
from simulator.services.event_manager.events.toggle_simulator_config_event import ToggleSimulatorConfigEvent

from simulator.views.gui.common import WINDOW_BG_COLOUR, WIDGET_BG_COLOUR
from simulator.views.gui.window import Window
from simulator.views.gui.simulator_config_state import SimulatorConfigState

from algorithms.configuration.configuration import Configuration
from algorithms.algorithm_manager import AlgorithmManager
from maps.map_manager import MapManager
from algorithms.configuration.maps.map import Map

class SimulatorConfig(DirectObject):
    __services: Services
    __base: ShowBase
    __window: Window

    __animations = {
        "None": (0, 0),
        "Normal": (np.finfo(float).eps, 0),
        "Slow": (0.5, 0),
        "Fast": (np.finfo(float).eps, 20)
    }

    def __init__(self, services: Services, mouse1_press_callbacks: List[Callable[[], None]]):
        self.__services = services
        self.__services.ev_manager.register_listener(self)
        self.__base = self.__services.graphics.window
        self.__text = " t - find the path between the agent and goal\n\n" \
                      " mouse click - moves agent to mouse location\n\n mouse right click - moves goal to mouse location\n\n arrow keys, PgUp, PgDn - move agent / goal (Alt down)\n\n x - toggle trace animation (animations required)\n\n m - toggle map between Sparse and Dense\n\n o - take a default screenshot of the map\n\n p - take a custom screenshot of the scene\n\n w, a, s, d - orbit around the map\n\n q - top view of the map\n\n c, v - toggle Simulator Configuration / View Editor\n\n i - toggle Debug Overlay"

        self.__algorithms = self.__services.settings.algorithms
        self.__maps = self.__services.settings.maps

        self.__map_keys = list(self.__maps.keys())
        self.__algorithm_keys = list(self.__algorithms.keys())
        self.__animation_keys = list(self.__animations.keys())

        self.__window = Window(self.__base, "simulator_config", mouse1_press_callbacks,
                               borderWidth=(0.0, 0.0),
                               frameColor=WINDOW_BG_COLOUR,
                               pos=(-1, 0.5, 0.5),
                               frameSize=(-1.7, 1.3, -6.2, 0.85)
                               )
        # spacer #
        DirectFrame(parent=self.__window.frame,
                    borderWidth=(.0, .0),
                    frameColor=WIDGET_BG_COLOUR,
                    frameSize=(-1.4, 1.4, -0.011, 0.011),
                    pos=(-0.2, 0.0, 0.4))

        DirectFrame(parent=self.__window.frame,
                    borderWidth=(.0, .0),
                    frameColor=WIDGET_BG_COLOUR,
                    frameSize=(-1.4, 1.4, -0.01, 0.01),
                    pos=(-0.2, 0.0, -2.96))

        self.sim_config = DirectLabel(parent=self.__window.frame,
                                      text="Simulator Configuration",
                                      text_fg=WHITE,
                                      text_bg=WINDOW_BG_COLOUR,
                                      frameColor=WINDOW_BG_COLOUR,
                                      borderWidth=(.0, .0),
                                      pos=(-0.53, 0.0, 0.56),
                                      scale=(0.2, 3, 0.2))
        # Zoom buttons
        self.btn_zoom_out = DirectButton(
            text="-",
            text_fg=WHITE,
            pressEffect=1,
            command=self.__window.zoom_out,
            pos=(0.71, 0., 0.55),
            parent=self.__window.frame,
            scale=(0.3, 4.15, 0.35),
            frameColor=TRANSPARENT)

        self.btn_zoom_in = DirectButton(
            text="+",
            text_fg=WHITE,
            pressEffect=1,
            command=self.__window.zoom_in,
            pos=(0.92, 0., 0.56),
            parent=self.__window.frame,
            scale=(0.3, 4.15, 0.35),
            frameColor=TRANSPARENT)

        # Quit button
        self.btn = DirectButton(text='x',
                                text_fg=WHITE,
                                command=self.__window.toggle_visible,
                                pos=(1.12, 0., 0.576),
                                parent=self.__window.frame,
                                scale=(0.3, 2.9, 0.2),
                                pressEffect=1,
                                frameColor=TRANSPARENT)

        self.user_information = DirectLabel(parent=self.__window.frame,
                                            text=self.__text,
                                            text_fg=WHITE,
                                            text_bg=WINDOW_BG_COLOUR,
                                            frameColor=WINDOW_BG_COLOUR,
                                            text_align=TextNode.ALeft,
                                            borderWidth=(.0, .0),
                                            pos=(-1.55, 0.0, -3.2),
                                            scale=(0.14, 1.4, 0.14))

        self.map_label = DirectLabel(parent=self.__window.frame,
                                     text="Map:",
                                     text_fg=WHITE,
                                     text_bg=WINDOW_BG_COLOUR,
                                     text_align=TextNode.ALeft,
                                     frameColor=WINDOW_BG_COLOUR,
                                     borderWidth=(.0, .0),
                                     pos=(-1.52, 0.4, 0.),
                                     scale=(0.17, 1.09, 0.13))

        self.algo_label = DirectLabel(parent=self.__window.frame,
                                      text="Algorithm:",
                                      text_fg=WHITE,
                                      text_bg=WINDOW_BG_COLOUR,
                                      frameColor=WINDOW_BG_COLOUR,
                                      text_align=TextNode.ALeft,
                                      borderWidth=(.0, .0),
                                      pos=(-1.52, 0.4, -0.5),
                                      scale=(0.17, 1.09, 0.13))

        self.animation_label = DirectLabel(parent=self.__window.frame,
                                           text="Animation:",
                                           text_fg=WHITE,
                                           text_bg=WINDOW_BG_COLOUR,
                                           frameColor=WINDOW_BG_COLOUR,
                                           text_align=TextNode.ALeft,
                                           borderWidth=(.0, .0),
                                           pos=(-1.52, 0.4, -1),
                                           scale=(0.17, 1.09, 0.13))

        self.agent_label = DirectLabel(parent=self.__window.frame,
                                       text="Agent:",
                                       text_fg=WHITE,
                                       text_bg=WINDOW_BG_COLOUR,
                                       frameColor=WINDOW_BG_COLOUR,
                                       text_align=TextNode.ALeft,
                                       borderWidth=(.0, .0),
                                       pos=(-1.52, 0.4, -1.5),
                                       scale=(0.17, 1.09, 0.13))

        self.goal_label = DirectLabel(parent=self.__window.frame,
                                      text="Goal:",
                                      text_fg=WHITE,
                                      text_bg=WINDOW_BG_COLOUR,
                                      frameColor=WINDOW_BG_COLOUR,
                                      text_align=TextNode.ALeft,
                                      borderWidth=(.0, .0),
                                      pos=(-1.52, 0.4, -2),
                                      scale=(0.17, 1.09, 0.13))

        # Creating goal and agent's entry fields
        self.__entries = []
        self.__entry_hovered = False
        mouse1_press_callbacks.append(self.__entry_mouse_click_callback)
        for i in range(0, 6):
            e = DirectEntry(parent=self.__window.frame,
                            scale=0.12,
                            pos=(-0.24 + (i % 3) * 0.57, 0.4, -1.5 - 0.5 * (i // 3)),
                            numLines=1,
                            width=3,
                            suppressKeys=True,
                            text_align=TextNode.ACenter,
                            focusInCommand=self.clear_text,
                            focusInExtraArgs=[i])
            self.__entries.append(e)
            e.bind(DGG.EXIT, self.__entry_exit_callback)
            e.bind(DGG.ENTER, self.__entry_enter_callback)
            e.bind(DGG.B1PRESS, self.__entry_mouse_click_callback)
            self.accept("mouse1", self.__entry_mouse_click_callback)

        self.__agent_disable_overlay = DirectButton(parent=self.__window.frame,
                                                    frameColor=TRANSPARENT,
                                                    borderWidth=(0.0, 0.0),
                                                    frameSize=(-0.6, 1.4, -0.2, 0.2),
                                                    pos=(-0.24, 0.4, -1.5),
                                                    suppressMouse=True)
        self.__agent_disable_overlay.hide()

        self.__maps_option = DirectOptionMenu(text="options",
                                              scale=0.14,
                                              parent=self.__window.frame,
                                              initialitem=self.__map_keys.index("Labyrinth") if "Labyrinth" in self.__map_keys else 0,
                                              items=self.__map_keys,
                                              pos=(-0.65, 0.4, 0.),
                                              highlightColor=(0.65, 0.65, 0.65, 1),
                                              textMayChange=1,
                                              command=self.__use_default_map_positions)

        self.__algorithms_option = DirectOptionMenu(text="options",
                                                    scale=0.14,
                                                    parent=self.__window.frame,
                                                    initialitem=self.__algorithm_keys.index("A*") if "A*" in self.__algorithm_keys else 0,
                                                    items=self.__algorithm_keys,
                                                    pos=(-0.46, 0.4, -0.5),
                                                    highlightColor=(0.65, 0.65, 0.65, 1),
                                                    textMayChange=1)

        self.__animations_option = DirectOptionMenu(text="options",
                                                    scale=0.14,
                                                    parent=self.__window.frame,
                                                    initialitem=self.__animation_keys.index("Fast"),
                                                    items=self.__animation_keys,
                                                    pos=(-0.45, 0.4, -1),
                                                    highlightColor=(0.65, 0.65, 0.65, 1),
                                                    textMayChange=1)

        self._update_frame = DirectFrame(parent=self.__window.frame,
                                         frameColor=WHITE,
                                         pos=(-1, 0.4, -2.6),
                                         borderWidth=(0.25, 0.15),
                                         frameSize=(-0.5, 0.95, -0.54, 0.54),
                                         scale=(0.50, 3.1, 0.25))

        self._reset_frame = DirectFrame(parent=self.__window.frame,
                                        frameColor=WHITE,
                                        pos=(0.412, 0.4, -2.6),
                                        borderWidth=(0.25, 0.15),
                                        frameSize=(-0.5, 0.92, -0.54, 0.54),
                                        scale=(0.50, 3.1, 0.25))

        self.btn_update = DirectButton(
            text="Update",
            text_fg=(0.3, 0.3, 0.3, 1.0),
            pressEffect=1,
            command=self.__update_simulator_callback,
            pos=(-0.9, 0.4, -2.65),
            parent=self.__window.frame,
            scale=(0.20, 2.1, 0.15),
            frameColor=TRANSPARENT)

        self.btn_reset = DirectButton(
            text="Reset",
            text_fg=(0.4, 0.3, 0.3, 1.0),
            pressEffect=1,
            command=self.__reset_simulator_callback,
            pos=(0.51, 0.4, -2.65),
            parent=self.__window.frame,
            scale=(0.20, 2.1, 0.15),
            frameColor=TRANSPARENT)

        # setup state & use saved state if possible
        self.__state = None
        for so in self.__services.state.objects:
            if isinstance(so, SimulatorConfigState):
                self.__state = so
                cmd = self.__maps_option['command']
                try:
                    self.__maps_option.set(self.__map_keys.index(so.mp))
                    self.__algorithms_option.set(self.__algorithm_keys.index(so.algo))
                    self.__animations_option.set(self.__animation_keys.index(so.ani))
                    self.__update_position_entries()
                except:
                    msg = "Failed to load Simulator Config state:\n{}".format(traceback.format_exc())
                    self.__services.debug.write(msg, DebugLevel.NONE)
                    break
                finally:
                    self.__maps_option['command'] = cmd
                return
        new_state = self.__state is None
        if new_state:
            self.__state = SimulatorConfigState(self.__services.state)
        self.__state.mp = self.__maps_option.get()
        self.__state.algo = self.__algorithms_option.get()
        self.__state.ani = self.__animations_option.get()
        self.__use_default_map_positions()
        if new_state:
            self.__services.state.add(self.__state)
        else:
            self.__services.state.save()

    def __entry_exit_callback(self, *discard) -> None:
        self.__entry_hovered = False

    def __entry_enter_callback(self, *discard) -> None:
        self.__entry_hovered = True

    def __entry_mouse_click_callback(self, *discard) -> None:
        if self.__entry_hovered:
            self.__window.focus()
        else:
            for e in self.__entries:
                e['focus'] = False

    def __get_map(self) -> Map:
        name = self.__maps_option.get()
        data = self.__maps[name]

        if isinstance(data, str):
            data = self.__services.resources.maps_dir.load(data)
            self.__maps[name] = data

        assert isinstance(data, Map), "Map failed to load"
        return data

    def __update_simulator_callback(self) -> None:
        agent_mutable = (not self.__services.settings.get_agent_position)

        mp = self.__get_map()
        algo = self.__algorithms[self.__algorithms_option.get()]
        ani = self.__animations[self.__animations_option.get()]

        # update state
        self.__state.mp = self.__maps_option.get()
        self.__state.algo = self.__algorithms_option.get()
        self.__state.ani = self.__animations_option.get()

        def deduce_pos(default, entries) -> Point:
            nonlocal mp
            vs = []
            for i in range(default.n_dim):
                try:
                    vs.append(int(entries[i].get()))
                except:
                    vs.append(default[i])
            p = Point(*vs)
            return p if mp.is_agent_valid_pos(p) else default

        if agent_mutable:
            self.__state.agent = deduce_pos(mp.agent.position, self.__entries[:3])
        self.__state.goal = deduce_pos(mp.goal.position, self.__entries[3:])
        self.__update_position_entries()  # update if user-provided point was invalid

        # save state
        self.__services.state.save()

        # launch simulation
        config = self.__services.settings
        config.algorithm_name = self.__algorithms_option.get()
        old_map_name = config.map_name
        config.map_name = self.__maps_option.get()

        refresh_map = (old_map_name != config.map_name) or \
                      (mp != config.simulator_initial_map) or \
                      (agent_mutable and self.__state.agent != mp.agent.position) or \
                      (self.__state.goal != mp.goal.position)

        if refresh_map:
            if agent_mutable:
                mp.move(mp.agent, self.__state.agent, True)
            mp.move(mp.goal, self.__state.goal, True)

        config.simulator_initial_map = mp
        config.simulator_algorithm_type, config.simulator_testing_type, config.simulator_algorithm_parameters = algo
        config.simulator_key_frame_speed, config.simulator_key_frame_skip = ani
        self.__services.reinit(refresh_map=refresh_map)

    def __reset_simulator_callback(self) -> None:
        self.__maps_option.set(self.__map_keys.index(self.__state.mp))
        self.__algorithms_option.set(self.__algorithm_keys.index(self.__state.algo))
        self.__animations_option.set(self.__animation_keys.index(self.__state.ani))
        self.__services.ev_manager.post(ResetEvent())

    def __use_default_map_positions(self, *discard) -> None:
        m = self.__get_map()

        self.__state.agent = m.agent.position
        self.__state.goal = m.goal.position
        self.__update_position_entries()

    def __update_position_entries(self) -> None:
        def update_entries(entries, pos, mutable=True):
            dim = pos.n_dim
            if not mutable:
                pos = ['-' for _ in range(dim)]

            entries[0].enterText(str(pos[0]))
            entries[1].enterText(str(pos[1]))
            if dim == 3:
                entries[2].enterText(str(pos[2]))
                entries[2].show()
            else:
                entries[2].hide()

        # agent may be externally set
        agent_mutable = (not self.__services.settings.get_agent_position)
        update_entries(self.__entries[:3], self.__state.agent, mutable=agent_mutable)
        update_entries(self.__entries[3:], self.__state.goal)

        if agent_mutable:
            self.__agent_disable_overlay.hide()
        else:
            self.__agent_disable_overlay.show()

        # user has performed an action such as pressing a
        # button, therefore all entries should lose focus
        for e in self.__entries:
            e['focus'] = False

    def notify(self, event: Event) -> None:
        if isinstance(event, ToggleSimulatorConfigEvent):
            self.__window.toggle_visible()

    def clear_text(self, i):
        self.__entries[i].enterText('')
