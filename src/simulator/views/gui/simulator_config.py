from panda3d.core import PNMImage, TextNode
from direct.gui.DirectGui import DirectFrame, DirectButton, DirectLabel, DirectEntry, DGG, DirectOptionMenu
from direct.showbase.ShowBase import ShowBase
from direct.showbase.DirectObject import DirectObject

from typing import Tuple, Union, List, Any, Dict, Callable

from structures import Point, WHITE, TRANSPARENT
from constants import DATA_PATH

from simulator.services.services import Services
from simulator.services.event_manager.events.event import Event
from simulator.services.event_manager.events.reset_event import ResetEvent
from simulator.services.event_manager.events.toggle_simulator_config_event import ToggleSimulatorConfigEvent

from simulator.views.gui.common import WINDOW_BG_COLOUR, WIDGET_BG_COLOUR
from simulator.views.gui.window import Window
from simulator.views.gui.simulator_config_state import SimulatorConfigState

from maps import Maps

# planner testing
from algorithms.basic_testing import BasicTesting
from algorithms.classic.testing.a_star_testing import AStarTesting
from algorithms.classic.testing.combined_online_lstm_testing import CombinedOnlineLSTMTesting
from algorithms.classic.testing.dijkstra_testing import DijkstraTesting
from algorithms.classic.testing.wavefront_testing import WavefrontTesting
from algorithms.classic.testing.way_point_navigation_testing import WayPointNavigationTesting

# planner implementations
from algorithms.classic.graph_based.a_star import AStar
from algorithms.classic.graph_based.bug1 import Bug1
from algorithms.classic.graph_based.bug2 import Bug2
from algorithms.classic.graph_based.dijkstra import Dijkstra
from algorithms.classic.graph_based.potential_field import PotentialField
from algorithms.classic.sample_based.sprm import SPRM
from algorithms.classic.sample_based.rt import RT
from algorithms.classic.sample_based.rrt import RRT
from algorithms.classic.sample_based.rrt_star import RRT_Star
from algorithms.classic.sample_based.rrt_connect import RRT_Connect

from utility.compatibility import HAS_OMPL

if HAS_OMPL:
    from algorithms.classic.sample_based.ompl_rrt import OMPL_RRT
    from algorithms.classic.sample_based.ompl_prmstar import OMPL_PRMstar
    from algorithms.classic.sample_based.ompl_lazyprmstar import OMPL_LazyPRMstar
    from algorithms.classic.sample_based.ompl_rrtstar import OMPL_RRTstar
    from algorithms.classic.sample_based.ompl_rrtsharp import OMPL_RRTsharp
    from algorithms.classic.sample_based.ompl_rrtx import OMPL_RRTXstatic
    from algorithms.classic.sample_based.ompl_informedrrt import OMPL_InformedRRT
    from algorithms.classic.sample_based.ompl_kpiece1 import OMPL_KPIECE1
    from algorithms.classic.sample_based.ompl_ltlplanner import OMPL_LTLPlanner
    from algorithms.classic.sample_based.ompl_pdst import OMPL_PDST
    from algorithms.classic.sample_based.ompl_sst import OMPL_SST
    from algorithms.classic.sample_based.ompl_aitstar import OMPL_AITstar
    from algorithms.classic.sample_based.ompl_anytimepathshortening import OMPL_AnytimePathShortening
    from algorithms.classic.sample_based.ompl_bfmt import OMPL_BFMT
    from algorithms.classic.sample_based.ompl_biest import OMPL_BiEST
    from algorithms.classic.sample_based.ompl_rrtconnect import OMPL_RRTConnect
    from algorithms.classic.sample_based.ompl_trrt import OMPL_TRRT
    from algorithms.classic.sample_based.ompl_birlrt import OMPL_BiRLRT
    from algorithms.classic.sample_based.ompl_bitrrt import OMPL_BiTRRT
    from algorithms.classic.sample_based.ompl_bitstar import OMPL_BITstar
    from algorithms.classic.sample_based.ompl_bkpiece1 import OMPL_BKPIECE1
    from algorithms.classic.sample_based.ompl_syclop import OMPL_Syclop
    from algorithms.classic.sample_based.ompl_cforest import OMPL_CForest
    from algorithms.classic.sample_based.ompl_est import OMPL_EST
    from algorithms.classic.sample_based.ompl_fmt import OMPL_FMT
    from algorithms.classic.sample_based.ompl_lazylbtrrt import OMPL_LazyLBTRRT
    from algorithms.classic.sample_based.ompl_lazyprm import OMPL_LazyPRM
    from algorithms.classic.sample_based.ompl_lazyrrt import OMPL_LazyRRT
    from algorithms.classic.sample_based.ompl_lbkpiece1 import OMPL_LBKPIECE1
    from algorithms.classic.sample_based.ompl_lbtrrt import OMPL_LBTRRT
    from algorithms.classic.sample_based.ompl_prm import OMPL_PRM
    from algorithms.classic.sample_based.ompl_spars import OMPL_SPARS
    from algorithms.classic.sample_based.ompl_spars2 import OMPL_SPARS2
    from algorithms.classic.sample_based.ompl_vfrrt import OMPL_VFRRT
    from algorithms.classic.sample_based.ompl_prrt import OMPL_pRRT
    from algorithms.classic.sample_based.ompl_tsrrt import OMPL_TSRRT
    from algorithms.classic.sample_based.ompl_psbl import OMPL_pSBL
    from algorithms.classic.sample_based.ompl_sbl import OMPL_SBL
    from algorithms.classic.sample_based.ompl_stride import OMPL_STRIDE
    from algorithms.classic.sample_based.ompl_qrrt import OMPL_QRRT

from algorithms.classic.graph_based.wavefront import Wavefront
from algorithms.configuration.configuration import Configuration
from algorithms.lstm.LSTM_tile_by_tile import OnlineLSTM
from algorithms.lstm.a_star_waypoint import WayPointNavigation
from algorithms.lstm.combined_online_LSTM import CombinedOnlineLSTM


class SimulatorConfig(DirectObject):
    __services: Services
    __base: ShowBase
    __window: Window

    __maps = {
        "Uniform Random Fill": ("uniform_random_fill_10/0", True),
        "Block": ("block_map_10/6", True),
        "House": ("house_10/6", True),
        "Long Wall": (Maps.grid_map_one_obstacle1, True),
        "Labyrinth": (Maps.grid_map_labyrinth, True),
        "3D Cube": (Maps.grid_map_3d_example, True),
        "vin test 8x8": (Maps.grid_map_small_one_obstacle2, True),
        "vin test 8x8 -2": (Maps.grid_map_small_one_obstacle, True),
        "vin test 8x8 -3": (Maps.grid_map_small_one_obstacle3, True),
        "vin test 16x16 -1": (Maps.grid_map_complex_obstacle, True),
        "vin test 16x16 -2": (Maps.grid_map_complex_obstacle2, True),
        "vin test 28x28 -1": (Maps.grid_map_28x28vin, True),
        "Small Obstacle": (Maps.grid_map_one_obstacle.convert_to_dense_map(), True),
        "2D Occupancy Grid Map": (Maps.ogm_2d, True),
        "3D Occupancy Grid Map": (Maps.ogm_3d, True),
        "SLAM Map 1": ("map10", False),
        "SLAM Map 1 (compressed)": ("map11", True),
        "SLAM Map 2": ("map14", False),
        "SLAM Map 3": ("map12", False),
    }

    __algorithms: Dict[str, Any]

    __animations = {
        "None": (0, 0),
        "Normal": (0.00001, 0),
        "Slow": (0.5, 0),
        "Fast": (0.16, 20)
    }

    def __init__(self, services: Services, mouse1_press_callbacks: List[Callable[[], None]]):
        self.__services = services
        self.__services.ev_manager.register_listener(self)
        self.__base = self.__services.graphics.window
        self.__text = " t - find the path between the agent and goal\n\n" \
                      " mouse click - moves agent to mouse location \n\n mouse right click - moves goal to" \
                      " mouse location\n\n x - toggle trace animation (animations required)\n\n" \
                      " m - toggle map between Sparse and Dense\n\n o - take a default screenshot of " \
                      "the map\n\n p - take a custom screenshot of the scene\n\n w, a, s, d " \
                      "- orbit around the map\n\n c, v - toggle Simulator Configuration / View Editor"

        self.__algorithms = {
            "A*": (AStar, AStarTesting, ([], {})),
            "Global Way-point LSTM": (WayPointNavigation, WayPointNavigationTesting, (
                [], {"global_kernel": (CombinedOnlineLSTM, ([], {})), "global_kernel_max_it": 100})),
            "LSTM Bagging": (CombinedOnlineLSTM, CombinedOnlineLSTMTesting, ([], {})),
            "CAE Online LSTM": (
                OnlineLSTM, BasicTesting, ([], {"load_name": "caelstm_section_lstm_training_block_map_10000_model"})),
            "Online LSTM": (OnlineLSTM, BasicTesting, (
                [],
                {"load_name": "tile_by_tile_training_uniform_random_fill_10000_block_map_10000_house_10000_model"})),
            "SPRM": (SPRM, BasicTesting, ([], {})),
            "RT": (RT, BasicTesting, ([], {})),
            "RRT": (RRT, BasicTesting, ([], {})),
            "RRT*": (RRT_Star, BasicTesting, ([], {})),
            "RRT-Connect": (RRT_Connect, BasicTesting, ([], {})),
            "Wave-front": (Wavefront, WavefrontTesting, ([], {})),
            "Dijkstra": (Dijkstra, DijkstraTesting, ([], {})),
            "Bug1": (Bug1, BasicTesting, ([], {})),
            "Bug2": (Bug2, BasicTesting, ([], {})),
            "Potential Field": (PotentialField, BasicTesting, ([], {}))
        }
        if HAS_OMPL:
            self.__algorithms.update({
                "OMPL RRT": (OMPL_RRT, BasicTesting, ([], {})),
                "OMPL PRM*": (OMPL_PRMstar, BasicTesting, ([], {})),
                "OMPL Lazy PRM*": (OMPL_LazyPRMstar, BasicTesting, ([], {})),
                "OMPL RRT*": (OMPL_RRTstar, BasicTesting, ([], {})),
                "OMPL RRT#": (OMPL_RRTsharp, BasicTesting, ([], {})),
                "OMPL RRTX": (OMPL_RRTXstatic, BasicTesting, ([], {})),
                "OMPL KPIECE1": (OMPL_KPIECE1, BasicTesting, ([], {})),
                "OMPL LazyLBTRRT": (OMPL_LazyLBTRRT, BasicTesting, ([], {})),
                "OMPL LazyPRM": (OMPL_LazyPRM, BasicTesting, ([], {})),
                "OMPL LazyRRT": (OMPL_LazyRRT, BasicTesting, ([], {})),
                "OMPL LBKPIECE1": (OMPL_LBKPIECE1, BasicTesting, ([], {})),
                "OMPL LBTRRT": (OMPL_LBTRRT, BasicTesting, ([], {})),
                "OMPL PRM": (OMPL_PRM, BasicTesting, ([], {})),
                "OMPL SBL": (OMPL_SBL, BasicTesting, ([], {})),
                "OMPL STRIDE": (OMPL_STRIDE, BasicTesting, ([], {})),
                "OMPL PDST": (OMPL_PDST, BasicTesting, ([], {})),
                "OMPL SST": (OMPL_SST, BasicTesting, ([], {})),
                "OMPL BiEst": (OMPL_BiEST, BasicTesting, ([], {})),
                "OMPL TRRT": (OMPL_TRRT, BasicTesting, ([], {})),
                "OMPL RRTConnect": (OMPL_RRTConnect, BasicTesting, ([], {})),
                "OMPL BITstar": (OMPL_BITstar, BasicTesting, ([], {})),
                "OMPL BKPIECE1": (OMPL_BKPIECE1, BasicTesting, ([], {})),
                "OMPL EST": (OMPL_EST, BasicTesting, ([], {})),
                # "OMPL LTLPlanner": (OMPL_LTLPlanner, BasicTesting, ([], {})),
                # "OMPL AITstar": (OMPL_AITstar, BasicTesting, ([], {})),
                # "OMPL AnytimePathShortening": (OMPL_AnytimePathShortening, BasicTesting, ([], {})),
                # "OMPL BFMT": (OMPL_BFMT, BasicTesting, ([], {})),
                # "OMPL BiRLRT": (OMPL_BiRLRT, BasicTesting, ([], {})),
                # "OMPL BiTRRT": (OMPL_BiTRRT, BasicTesting, ([], {})),
                # "OMPL Syclop ": (OMPL_Syclop, BasicTesting, ([], {})),
                # "OMPL CForest": (OMPL_CForest, BasicTesting, ([], {})),
                # "OMPL FMT": (OMPL_FMT, BasicTesting, ([], {})),
                # "OMPL SPARS": (OMPL_SPARS, BasicTesting, ([], {})),
                # "OMPL SPARS2": (OMPL_SPARS2, BasicTesting, ([], {})),
                # "OMPL VFRRT": (OMPL_VFRRT, BasicTesting, ([], {})),
                # "OMPL pRRT": (OMPL_pRRT, BasicTesting, ([], {})),
                # "OMPL TSRRT": (OMPL_TSRRT, BasicTesting, ([], {})),
                # "OMPL pSBL": (OMPL_pSBL, BasicTesting, ([], {})),
                # "OMPL QRRT": (OMPL_QRRT, BasicTesting, ([], {})),
            })

        self.__map_keys = list(self.__maps.keys())
        self.__algorithm_keys = list(self.__algorithms.keys())
        self.__animation_keys = list(self.__animations.keys())

        self.__window = Window(self.__base, "simulator_config", mouse1_press_callbacks,
                               borderWidth=(0.0, 0.0),
                               frameColor=WINDOW_BG_COLOUR,
                               pos=(-1, 0.5, 0.5),
                               frameSize=(-1.7, 1.3, -5.15, 0.85)
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
                                            scale=(0.11, 1.1, 0.11))
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
                            text_align=TextNode.ACenter)
            self.__entries.append(e)
            e.bind(DGG.EXIT, self.__entry_exit_callback)
            e.bind(DGG.ENTER, self.__entry_enter_callback)
            self.accept("mouse1", self.__entry_mouse_click_callback)

        self.__maps_option = DirectOptionMenu(text="options",
                                              scale=0.14,
                                              parent=self.__window.frame,
                                              initialitem=self.__map_keys.index("Labyrinth"),
                                              items=self.__map_keys,
                                              pos=(-0.65, 0.4, 0.),
                                              highlightColor=(0.65, 0.65, 0.65, 1),
                                              textMayChange=1,
                                              command=self.__use_default_map_positions)

        self.__algorithms_option = DirectOptionMenu(text="options",
                                                    scale=0.14,
                                                    parent=self.__window.frame,
                                                    initialitem=self.__algorithm_keys.index("A*"),
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
                    break
                finally:
                    self.__maps_option['command'] = cmd
                return
        new_state = self.__state is None
        if new_state:
            self.__state = SimulatorConfigState()
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
        if not self.__entry_hovered:
            for e in self.__entries:
                e['focus'] = False

    def __update_simulator_callback(self) -> None:
        mp = self.__maps[self.__maps_option.get()]
        algo = self.__algorithms[self.__algorithms_option.get()]
        ani = self.__animations[self.__animations_option.get()]

        # load up map if necessary
        # occurs when initial map
        # is a string.
        if isinstance(mp[0], str):
            i = self.__maps_option.get()
            self.__maps[i] = (self.__services.resources.maps_dir.load(mp[0]), self.__maps[i][1])
            mp[0] = self.__maps[i][0]

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
            return p if mp[0].is_agent_valid_pos(p) else default

        self.__state.agent = deduce_pos(mp[0].agent.position, self.__entries[:3])
        self.__state.goal = deduce_pos(mp[0].goal.position, self.__entries[3:])
        self.__update_position_entries()  # update if user-provided point was invalid

        # save state
        self.__services.state.save()

        # launch simulation
        config = self.__services.settings

        refresh_map = (self.__state.agent != mp[0].agent.position) or \
                      (self.__state.goal != mp[0].goal.position) or \
                      (mp[0] != config.simulator_initial_map)

        if refresh_map:
            mp[0].move(mp[0].agent, self.__state.agent, True)
            mp[0].move(mp[0].goal, self.__state.goal, True)

        config.simulator_initial_map, config.simulator_grid_display = mp
        config.simulator_algorithm_type, config.simulator_testing_type, config.simulator_algorithm_parameters = algo
        config.simulator_key_frame_speed, config.simulator_key_frame_skip = ani
        self.__services.reinit(refresh_map=refresh_map)

    def __reset_simulator_callback(self) -> None:
        self.__maps_option.set(self.__map_keys.index(self.__state.mp))
        self.__algorithms_option.set(self.__algorithm_keys.index(self.__state.algo))
        self.__animations_option.set(self.__animation_keys.index(self.__state.ani))
        self.__services.ev_manager.post(ResetEvent())

    def __use_default_map_positions(self, *discard) -> None:
        m = self.__maps[self.__maps_option.get()][0]

        # load up map if necessary
        if isinstance(m, str):
            i = self.__maps_option.get()
            self.__maps[i] = (self.__services.resources.maps_dir.load(m), self.__maps[i][1])
            m = self.__maps[i][0]

        self.__state.agent = m.agent.position
        self.__state.goal = m.goal.position
        self.__update_position_entries()

    def __update_position_entries(self) -> None:
        def update_entries(entries, pos):
            entries[0].enterText(str(pos[0]))
            entries[1].enterText(str(pos[1]))
            if pos.n_dim == 3:
                entries[2].enterText(str(pos[2]))
                entries[2].show()
            else:
                entries[2].hide()

        update_entries(self.__entries[:3], self.__state.agent)
        update_entries(self.__entries[3:], self.__state.goal)

        # user has performed an action such as pressing a
        # button, therefore all entries should lose focus
        for e in self.__entries:
            e['focus'] = False

    def notify(self, event: Event) -> None:
        if isinstance(event, ToggleSimulatorConfigEvent):
            self.__window.toggle_visible()
