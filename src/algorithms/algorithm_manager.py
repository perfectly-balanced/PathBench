from algorithms.algorithm import Algorithm
from algorithms.classic.graph_based.cgds import CGDS
from algorithms.classic.testing.cgds_testing import CGDSTesting
from utility.compatibility import HAS_OMPL
from utility.misc import static_class

from typing import Optional, List, Type, Dict, Any, Tuple
import importlib.util
import inspect
import os
import sys
import copy
import traceback

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
from algorithms.classic.graph_based.wavefront import Wavefront
from algorithms.lstm.LSTM_tile_by_tile import OnlineLSTM
from algorithms.lstm.a_star_waypoint import WayPointNavigation
from algorithms.lstm.combined_online_LSTM import CombinedOnlineLSTM

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

@static_class
class AlgorithmManager():
    MetaData = Tuple[Type[Algorithm], Type[BasicTesting], Tuple[List[Any], Dict[str, Any]]]

    builtins: Dict[str, MetaData]

    @classmethod
    def _static_init_(cls):
        cls.builtins = {
            "A*": (AStar, AStarTesting, ([], {})),
            "WPN":(WayPointNavigation, WayPointNavigationTesting, ([], {"global_kernel_max_it": 10, "global_kernel": (CombinedOnlineLSTM, ([], {}))})),
            "WPN-view":(WayPointNavigation, WayPointNavigationTesting, ([], {"global_kernel_max_it": 20, "global_kernel": (OnlineLSTM, ([], {"load_name": "tile_by_tile_training_uniform_random_fill_10000_block_map_10000_house_10000_model"}))})),
            "WPN-map": (WayPointNavigation, WayPointNavigationTesting, ([], {"global_kernel_max_it": 20, "global_kernel": (OnlineLSTM, ([], {"load_name": "caelstm_section_lstm_training_uniform_random_fill_10000_block_map_10000_house_10000_model"}))})),
            "LSTM Bagging": (CombinedOnlineLSTM, CombinedOnlineLSTMTesting, ([], {})),
            "Map Module (CAE) ": (OnlineLSTM, BasicTesting, ([], {"load_name": "caelstm_section_lstm_training_uniform_random_fill_10000_block_map_10000_house_10000_model"})),
            "View Module (Online LSTM)": (OnlineLSTM, BasicTesting, ([], {"load_name": "tile_by_tile_training_uniform_random_fill_10000_block_map_10000_house_10000_model"})),
            "SPRM": (SPRM, BasicTesting, ([], {})),
            "RT": (RT, BasicTesting, ([], {})),
            "RRT": (RRT, BasicTesting, ([], {})),
            "RRT*": (RRT_Star, BasicTesting, ([], {})),
            "RRT-Connect": (RRT_Connect, BasicTesting, ([], {})),
            "Wave-front": (Wavefront, WavefrontTesting, ([], {})),
            "Dijkstra": (Dijkstra, DijkstraTesting, ([], {})),
            "Bug1": (Bug1, BasicTesting, ([], {})),
            "Bug2": (Bug2, BasicTesting, ([], {})),
            "Child-Generator-Deque-Search": (CGDS, CGDSTesting, ([], {})),
            "Potential Field": (PotentialField, BasicTesting, ([], {}))
        }

        if HAS_OMPL:
            cls.builtins.update({
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

    @staticmethod
    def load_all(ids: List[str]) -> List[List[Tuple[str, MetaData]]]:
        """
        Returns a list of algorithms from a list of names or file paths.

        For each element in `ids`, if string is the display name
        of a built-in algorithm, then we return that algorithm. Otherwise,
        we return the result of AlgorithmManager.try_load_from_file().
        """

        algs: List[List[Tuple[str, MetaData]]] = []
        for alg in ids:
            if alg in AlgorithmManager.builtins:
                algs.append([copy.deepcopy((alg, AlgorithmManager.builtins[alg]))])
            else:
                algs.append(AlgorithmManager.try_load_from_file(alg))
        return algs

    @staticmethod
    def try_load_from_file(path: str) -> List[Tuple[str, MetaData]]:
        if not os.path.exists(path):
            msg = "File '{}' does not exist".format(path)
            print(msg, file=sys.stderr)
            return []

        try:
            spec = importlib.util.spec_from_file_location("custom_loaded", path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)

            # return all classes that inherit from "Algorithm"
            algs = []
            for name in dir(module):
                if name.startswith("_"):
                    continue

                cls = getattr(module, name)
                if inspect.isclass(cls) and cls is not Algorithm and issubclass(cls, Algorithm):
                    name = cls.name if "name" in cls.__dict__ else os.path.basename(path) + " ({})".format(name)
                    testing = cls.testing if "testing" in cls.__dict__ else BasicTesting
                    algs.append((name, (cls, testing, ([], {}))))
            return algs
        except:
            msg = "Failed to load algorithms from file '{}', reason:\n{}".format(path, traceback.format_exc())
            print(msg, file=sys.stderr)
            return []
