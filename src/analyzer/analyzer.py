import resource
import psutil
import copy
import random
import tracemalloc
from typing import TYPE_CHECKING, List, Tuple, Type, Any, Dict, Union, Optional
import csv
import pandas as pd
pd.plotting.register_matplotlib_converters()
import matplotlib.pyplot as plt
import seaborn as sns

from io import StringIO

from memory_profiler import profile
from algorithms.algorithm import Algorithm
from algorithms.basic_testing import BasicTesting
from algorithms.classic.testing.a_star_testing import AStarTesting
from algorithms.classic.testing.dijkstra_testing import DijkstraTesting
from algorithms.classic.testing.wavefront_testing import WavefrontTesting
from algorithms.classic.testing.combined_online_lstm_testing import CombinedOnlineLSTMTesting
from algorithms.classic.testing.way_point_navigation_testing import WayPointNavigationTesting
from algorithms.configuration.configuration import Configuration
from algorithms.configuration.maps.dense_map import DenseMap
from algorithms.configuration.maps.map import Map
from algorithms.lstm.LSTM_tile_by_tile import OnlineLSTM
from algorithms.lstm.a_star_waypoint import WayPointNavigation
from algorithms.lstm.combined_online_LSTM import CombinedOnlineLSTM
from maps import Maps
from simulator.services.debug import DebugLevel, Debug
from simulator.services.services import Services
from simulator.simulator import Simulator
from structures import Point

from algorithms.classic.graph_based.a_star import AStar
from algorithms.classic.graph_based.dijkstra import Dijkstra
from algorithms.classic.sample_based.rrt import RRT
from algorithms.classic.sample_based.rrt_star import RRT_Star
from algorithms.classic.sample_based.rrt_connect import RRT_Connect
from algorithms.classic.graph_based.wavefront import Wavefront
from algorithms.classic.graph_based.potential_field import PotentialField
from algorithms.classic.graph_based.bug1 import Bug1
from algorithms.classic.graph_based.bug2 import Bug2
from algorithms.classic.sample_based.rt import RT

# OMPL algorithms
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

if TYPE_CHECKING:
    from main import MainRunner

global algostring
algostring = []

class Analyzer:
    __services: Services
    __analysis_stream: StringIO

    def __init__(self, services: Services) -> None:
        self.__services = services

        self.analyze_algorithms = self.__services.debug.debug_func(DebugLevel.BASIC)(self.analyze_algorithms)
        self.__analysis_stream = None

    @staticmethod
    def __get_average_value(results: List[Dict[str, Any]], attribute: str, decimal_places: int = 2) -> float:
        if len(results) == 0:
            return 0
        #list of data for each result type
        #lstdata =list(map(lambda r: r[attribute], results)))
        val: float = sum(list(map(lambda r: r[attribute], results)))
        val = round(float(val) / len(results), decimal_places)
        return val

    @staticmethod
    def __get_values(results: List[Dict[str, Any]], attribute: str, decimal_places: int = 2) -> float:
        if len(results) == 0:
            return 0
        
        #list of data for each result type
        lstdata =list(map(lambda r: r[attribute], results))

        return lstdata

    @staticmethod
    def __get_results(results: List[Dict[str, Any]], custom_pick: List[bool] = None) -> Dict[str, Any]:
        goal_found: float = round(Analyzer.__get_average_value(results, "goal_found", 4) * 100, 2)

        if custom_pick:
            filtered_results = list(map(lambda el: el[0], filter(lambda el: el[1], zip(results, custom_pick))))
        else:
            filtered_results = list(filter(lambda r: r["goal_found"], results))

        average_steps: float = Analyzer.__get_average_value(filtered_results, "total_steps")
        average_distance: float = Analyzer.__get_average_value(filtered_results, "total_distance")
        average_smoothness: float = Analyzer.__get_average_value(filtered_results, "smoothness_of_trajectory")
        average_time: float = Analyzer.__get_average_value(filtered_results, "total_time", 4)
        average_distance_from_goal: float = Analyzer.__get_average_value(results, "distance_to_goal")
        average_original_distance_from_goal: float = Analyzer.__get_average_value(results, "original_distance_to_goal")

        steps_alldata: List[Any]  = Analyzer.__get_values(filtered_results, "total_steps")
        distance_alldata: List[Any]  = Analyzer.__get_values(filtered_results, "total_distance")
        smoothness_alldata: List[Any]  = Analyzer.__get_values(filtered_results, "smoothness_of_trajectory")
        time_alldata: List[Any]  = Analyzer.__get_values(filtered_results, "total_time", 4)
        distance_from_goal_alldata: List[Any] = Analyzer.__get_values(results, "distance_to_goal")
        original_distance_from_goal_alldata: List[Any] = Analyzer.__get_values(results, "original_distance_to_goal")
        path_deviation_alldata: List[Any]  = Analyzer.__get_values(filtered_results, "total_distance")
        memory_alldata = Analyzer.__get_values(filtered_results, "memory")
        average_memory = Analyzer.__get_average_value(results, "memory")
  
        ret = {
            "goal_found_perc": goal_found,
            "average_steps": average_steps,
            "average_distance": average_distance,
            "average_smoothness": average_smoothness,
            "average_time": average_time,
            "average_distance_from_goal": average_distance_from_goal,
            "average_original_distance_from_goal": average_original_distance_from_goal,
            "steps_alldata": steps_alldata,
            "distance_alldata": distance_alldata,
            "smoothness_alldata": smoothness_alldata,
            "time_alldata": time_alldata,
            "distance_from_goal_alldata": distance_from_goal_alldata,
            "original_distance_from_goal_alldata": original_distance_from_goal_alldata,
            'path_deviation_alldata': distance_alldata,
            'memory_alldata': memory_alldata,
            'average memory': average_memory
            }

        if results:
            if "fringe" in results[0]:
                ret["average_fringe"] = Analyzer.__get_average_value(filtered_results, "fringe")
                ret["average_search_space"] = Analyzer.__get_average_value(filtered_results, "search_space")
                ret["average_total_search_space"] = Analyzer.__get_average_value(filtered_results, "total_search_space")
                ret['search_space_alldata']= Analyzer.__get_values(filtered_results, "total_search_space")


            # combined lstm
            if "kernels" in results[0]:
                ret["kernels"] = results[0]["kernels"]
                ret["kernels_pick_perc"] = [0 for _ in range(len(ret["kernels"]))]

                for r in results:
                    ret["kernels_pick_perc"][r["best_kernel_idx"]] += 1
                ret["kernels_pick_perc"] = list(map(lambda x: round(float(x) / len(results) * 100, 2), ret["kernels_pick_perc"]))

            # way point navigation
            if "global_kernel_steps" in results[0]:
                ret["average_global_kernel_nr_of_way_points"] = Analyzer.__get_average_value(results, "nr_of_way_points")
                ret["average_global_kernel_steps"] = Analyzer.__get_average_value(results, "global_kernel_steps")
                ret["average_global_kernel_distance"] = Analyzer.__get_average_value(results, "global_kernel_distance")
                ret["average_global_kernel_last_way_point_distance_from_goal"] = Analyzer.__get_average_value(results, "last_way_point_distance_from_goal")
                ret["average_global_nr_of_global_kernel_calls"] = Analyzer.__get_average_value(results, "nr_of_global_kernel_calls")
                ret["average_global_kernel_improvement"] = Analyzer.__get_average_value(results, "global_kernel_improvement")
                ret["average_global_average_way_point_distance"] = Analyzer.__get_average_value(results, "average_way_point_distance")

                # local kernel
                ret["average_local_kernel_total_search_space"] = Analyzer.__get_average_value(filtered_results, "local_kernel_total_search_space")
                ret["average_local_kernel_total_fringe"] = Analyzer.__get_average_value(filtered_results, "local_kernel_total_fringe")
                ret["average_local_kernel_total_total"] = Analyzer.__get_average_value(filtered_results, "local_kernel_total_total")
                ret["average_local_kernel_average_search_space"] = Analyzer.__get_average_value(filtered_results, "local_kernel_average_search_space")
                ret["average_local_kernel_average_fringe"] = Analyzer.__get_average_value(filtered_results, "local_kernel_average_fringe")
                ret["average_local_kernel_average_total"] = Analyzer.__get_average_value(filtered_results, "local_kernel_average_total")
                ret["average_total_search_space"] = Analyzer.__get_average_value(filtered_results, "local_kernel_total_search_space")
                ret['search_space_alldata']= Analyzer.__get_values(filtered_results, "local_kernel_total_search_space") #
                ret["average_total_search_space"] = Analyzer.__get_average_value(filtered_results, "local_kernel_total_search_space") #
                
                # with combined lstm
                if "global_kernel_kernel_names" in results[0]:
                    ret["global_kernel_kernel_names"] = results[0]["global_kernel_kernel_names"]
                    ret["kernels_pick_perc"] = [0 for _ in range(len(ret["global_kernel_kernel_names"]))]

                    for r in results:
                        for idx, perc in enumerate(r["global_kernel_best_kernel_calls"]):
                            ret["kernels_pick_perc"][idx] += perc

                    div = sum(ret["kernels_pick_perc"])

                    for idx in range(len(ret["kernels_pick_perc"])):
                        ret["kernels_pick_perc"][idx] = ret["kernels_pick_perc"][idx] / float(div) * 100
                        ret["kernels_pick_perc"][idx] = round(ret["kernels_pick_perc"][idx], 2)
        return ret

    #@profile (precision=4)
    def __run_simulation(self, grid: Map, algorithm_type: Type[Algorithm], testing_type: Type[BasicTesting],
                         algo_params: Tuple[list, dict], agent_pos: Point = None) -> Dict[str, Any]:
        config = Configuration()
        config.simulator_initial_map = copy.deepcopy(grid)
        config.simulator_algorithm_type = algorithm_type
        config.simulator_testing_type = testing_type
        config.simulator_algorithm_parameters = algo_params

        if agent_pos:
            config.simulator_initial_map.move_agent(agent_pos, True, False)

        sim: Simulator = Simulator(Services(config))
        
        tracemalloc.start()
        # print("_*************((((((((((((((((()))))))))))))))))))))%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
        # print ('Memory usage initially:')

        resu = sim.start().get_results()
        
        current, peak = tracemalloc.get_traced_memory()
        
        tracemalloc.stop()
        
        #print("smoothness result!!!!!!!", resu['smoothness_of_trajectory'])
        
        # print ('resu===============================================',resu)
        # print ('Memory usage finally:')

        print('############################################################Memory used =====', peak/1000,'KB')
        #resu ['memory'] = 0
        resu ['memory'] = (peak/1000)
        
        #print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> resu = ", resu)
        
        return resu

    def __convert_maps(self, maps: List[Map]) -> List[Map]:
        def f(m: Union[str, Map]) -> Map:
            if isinstance(m, str):
                return self.__services.resources.maps_dir.load(m)
            return m

        return list(map(f, maps))

    def __report_results(self, results: List[Dict[str, Any]], a_star_res: Optional[List[Dict[str, Any]]],
                         algorithm_type: Type[Algorithm], algorithms) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
        def __get_improvement(val, against, low_is_better=True):
            if against == 0:
                return 0
            res = float(val - against) / float(against) * 100
            res = round(res if not low_is_better else -res, 2)
            return res

        # analyze results and print
        # returns a_star_results if modified
        res_proc = self.__get_results(results)
        
        #print("average smoothness!!!!!!!!!!!!!!!!!!!!!!!", res_proc["average_smoothness"])

        if algorithm_type == AStar:
            a_star_res = results

        a_star_res_proc = self.__get_results(a_star_res, list(map(lambda r: r["goal_found"], results)))

        res_proc["a_star_res_proc"] = a_star_res_proc

        goal_found_perc_improvement = __get_improvement(res_proc["goal_found_perc"], a_star_res_proc["goal_found_perc"], False)
        average_steps_improvement = __get_improvement(res_proc["average_steps"], a_star_res_proc["average_steps"])
        average_distance_improvement = __get_improvement(res_proc["average_distance"], a_star_res_proc["average_distance"])
        average_smoothness_improvement = __get_improvement(res_proc["average_smoothness"], a_star_res_proc["average_smoothness"])
        average_time_improvement = __get_improvement(res_proc["average_time"], a_star_res_proc["average_time"])
        average_distance_from_goal_improvement = __get_improvement(res_proc["average_distance_from_goal"], a_star_res_proc["average_distance_from_goal"])

        res_proc["goal_found_perc_improvement"] = goal_found_perc_improvement
        res_proc["average_steps_improvement"] = average_steps_improvement
        res_proc["average_distance_improvement"] = average_distance_improvement
        res_proc["average_smoothness_improvement"] = average_smoothness_improvement
        res_proc["average_time_improvement"] = average_time_improvement
        res_proc["average_distance_from_goal_improvement"] = average_distance_from_goal_improvement
        res_proc["average_path_deviation"] = res_proc["average_distance"] - a_star_res_proc["average_distance"] 
        res_proc['path_deviation_alldata'] = [ y-x for x,y in zip(res_proc['path_deviation_alldata'], a_star_res_proc["distance_alldata"])]


        self.__services.debug.write("Rate of success: {}%, (A*: {}%), (Improvement: {}%)".format(res_proc["goal_found_perc"], a_star_res_proc["goal_found_perc"], goal_found_perc_improvement), streams=[self.__analysis_stream])
        self.__services.debug.write("Average total steps: {}, (A*: {}), (Improvement: {}%)".format(res_proc["average_steps"], a_star_res_proc["average_steps"], average_steps_improvement), streams=[self.__analysis_stream])
        self.__services.debug.write("Average total distance: {}, (A*: {}), (Improvement: {}%)".format(res_proc["average_distance"], a_star_res_proc["average_distance"], average_distance_improvement), streams=[self.__analysis_stream])
        self.__services.debug.write("Average trajectory smoothness: {}, (A*: {}), (Improvement: {}%)".format(res_proc["average_smoothness"], a_star_res_proc["average_smoothness"], average_smoothness_improvement), streams=[self.__analysis_stream])
        self.__services.debug.write("Average time: {} seconds, (A*: {} seconds), (Improvement: {}%)".format(res_proc["average_time"], a_star_res_proc["average_time"], average_time_improvement), streams=[self.__analysis_stream])
        self.__services.debug.write("Average distance from goal: {}, (A*: {}), (Improvement: {}%) (Average original distance from goal: {})".format(res_proc["average_distance_from_goal"], a_star_res_proc["average_distance_from_goal"], average_distance_from_goal_improvement, res_proc["average_original_distance_from_goal"]), streams=[self.__analysis_stream])

        if "average_fringe" in res_proc:
            self.__services.debug.write("Average search space (no fringe): {}%".format(res_proc["average_search_space"]), streams=[self.__analysis_stream])
            self.__services.debug.write("Average fringe: {}%".format(res_proc["average_fringe"]), streams=[self.__analysis_stream])
            self.__services.debug.write("Average total search space: {}%".format(res_proc["average_total_search_space"]), streams=[self.__analysis_stream])
        # else:
        #     res_proc["average_total_search_space"]= float('nan')
        #     res_proc['search_space_alldata']= [float('nan') for x in range(len(res_proc['path_deviation_alldata']))]

        if "kernels" in res_proc:
            self.__services.debug.write("Kernels: {}".format(res_proc["kernels"]), streams=[self.__analysis_stream])
            self.__services.debug.write("Kernels pick percentages: {}".format(list(map(lambda r: str(r) + "%", res_proc["kernels_pick_perc"]))), streams=[self.__analysis_stream])

        if "average_global_kernel_nr_of_way_points" in res_proc:
            self.__services.debug.write("Average number of way points: {}".format(res_proc["average_global_kernel_nr_of_way_points"]), streams=[self.__analysis_stream])
            self.__services.debug.write("Average number of global kernel steps: {}".format(res_proc["average_global_kernel_steps"]), streams=[self.__analysis_stream])
            self.__services.debug.write("Average global kernel distance: {}".format(res_proc["average_global_kernel_distance"]),streams=[self.__analysis_stream])
            self.__services.debug.write("Average last way point distance to goal: {}".format(res_proc["average_global_kernel_last_way_point_distance_from_goal"]), streams=[self.__analysis_stream])
            self.__services.debug.write("Average number of global kernel calls: {}".format(res_proc["average_global_nr_of_global_kernel_calls"]), streams=[self.__analysis_stream])
            self.__services.debug.write("Average global kernel improvement: {}%".format(res_proc["average_global_kernel_improvement"]), streams=[self.__analysis_stream])
            self.__services.debug.write("Average way point in-between distance: {}".format(res_proc["average_global_average_way_point_distance"]), streams=[self.__analysis_stream])

            average_local_kernel_total_search_space_improvement = __get_improvement(res_proc["average_local_kernel_total_search_space"], a_star_res_proc["average_search_space"])
            average_local_kernel_total_fringe_improvement = __get_improvement(res_proc["average_local_kernel_total_fringe"], a_star_res_proc["average_fringe"])
            average_local_kernel_total_total_improvement = __get_improvement(res_proc["average_local_kernel_total_total"], a_star_res_proc["average_total_search_space"])
            average_local_kernel_session_search_space_improvement = __get_improvement(res_proc["average_local_kernel_average_search_space"], a_star_res_proc["average_search_space"])
            average_local_kernel_session_fringe_improvement = __get_improvement(res_proc["average_local_kernel_average_fringe"], a_star_res_proc["average_fringe"])
            average_local_kernel_session_total_improvement = __get_improvement(res_proc["average_local_kernel_average_total"], a_star_res_proc["average_total_search_space"])

            res_proc["average_local_kernel_total_search_space_improvement"] = average_local_kernel_total_search_space_improvement
            res_proc["average_local_kernel_total_fringe_improvement"] = average_local_kernel_total_fringe_improvement
            res_proc["average_local_kernel_total_total_improvement"] = average_local_kernel_total_total_improvement
            res_proc["average_local_kernel_average_search_space_improvement"] = average_local_kernel_session_search_space_improvement
            res_proc["average_local_kernel_average_fringe_improvement"] = average_local_kernel_session_fringe_improvement
            res_proc["average_local_kernel_average_total_improvement"] = average_local_kernel_session_total_improvement

            res_proc["a_star_res_proc"]["average_local_kernel_total_search_space"] = a_star_res_proc["average_search_space"]
            res_proc["a_star_res_proc"]["average_local_kernel_total_fringe"] = a_star_res_proc["average_fringe"]
            res_proc["a_star_res_proc"]["average_local_kernel_total_total"] = a_star_res_proc["average_total_search_space"]
            res_proc["a_star_res_proc"]["average_local_kernel_average_search_space"] = a_star_res_proc["average_search_space"]
            res_proc["a_star_res_proc"]["average_local_kernel_average_fringe"] = a_star_res_proc["average_fringe"]
            res_proc["a_star_res_proc"]["average_local_kernel_average_total"] = a_star_res_proc["average_total_search_space"]

            self.__services.debug.write("Average local kernel all search space (no fringe): {}%, (A*: {}%), (Improvement: {}%)".format(res_proc["average_local_kernel_total_search_space"], a_star_res_proc["average_search_space"], average_local_kernel_total_search_space_improvement), streams=[self.__analysis_stream])
            self.__services.debug.write("Average local kernel all fringe: {}%, (A*: {}%), (Improvement: {}%)".format(res_proc["average_local_kernel_total_fringe"], a_star_res_proc["average_fringe"], average_local_kernel_total_fringe_improvement), streams=[self.__analysis_stream])
            self.__services.debug.write("Average local kernel all total search space: {}%, (A*: {}%), (Improvement: {}%)".format(res_proc["average_local_kernel_total_total"], a_star_res_proc["average_total_search_space"], average_local_kernel_total_total_improvement), streams=[self.__analysis_stream])
            self.__services.debug.write("Average local kernel session search space (no fringe): {}%, (A*: {}%), (Improvement: {}%)".format(res_proc["average_local_kernel_average_search_space"], a_star_res_proc["average_search_space"], average_local_kernel_session_search_space_improvement), streams=[self.__analysis_stream])
            self.__services.debug.write("Average local kernel session fringe: {}%, (A*: {}%), (Improvement: {}%)".format(res_proc["average_local_kernel_average_fringe"], a_star_res_proc["average_fringe"], average_local_kernel_session_fringe_improvement), streams=[self.__analysis_stream])
            self.__services.debug.write("Average local kernel session total search space: {}%, (A*: {}%), (Improvement: {}%)".format(res_proc["average_local_kernel_average_total"], a_star_res_proc["average_total_search_space"], average_local_kernel_session_total_improvement), streams=[self.__analysis_stream])

            if "global_kernel_kernel_names" in res_proc:
                self.__services.debug.write("Global kernel kernels: {}%".format(res_proc["global_kernel_kernel_names"]), streams=[self.__analysis_stream])
                self.__services.debug.write("Global kernel kernel calls percentages: {}%".format(list(map(lambda r: str(r) + "%", res_proc["kernels_pick_perc"]))), streams=[self.__analysis_stream])

        try:
            if res_proc["average_total_search_space"]:
                pass
        except:
            res_proc["average_total_search_space"] = float('nan')
            res_proc['search_space_alldata'] = [float('nan') for x in range(len(res_proc['path_deviation_alldata']))]

        self.__services.debug.write("\n", timestamp=False, streams=[self.__analysis_stream])

        # writing average data to csv file
        with open('pbtest.csv', 'a+', newline='') as file:
            fieldnames = ['Algorithm', 'Average Path Deviation', 'Success Rate', 'Average Time','Average Steps', 'Average Distance', 'Average Distance from Goal','Average Original Distance from Goal', 'Average Trajectory Smoothness', 'Average Search Space', 'Average Memory']
            writer = csv.DictWriter(file, fieldnames=fieldnames)

            if file.tell() == 0:
                writer.writeheader()
                 
            for lst in algorithms:
                if lst[0] == algorithm_type and lst[3] not in algostring:
                    # lst[3] is name of the current algorithm
                    algoname1 = lst[3]
                    algostring.append(lst[3])
                    break

            writer.writerow({'Algorithm': algoname1, 'Success Rate':res_proc["goal_found_perc"], 'Average Time': res_proc["average_time"],'Average Steps': res_proc["average_steps"]\
                ,'Average Distance': res_proc["average_distance"],'Average Distance from Goal': res_proc["average_distance_from_goal"]\
                    ,'Average Original Distance from Goal': res_proc["average_original_distance_from_goal"], 'Average Path Deviation': res_proc["average_path_deviation"], 'Average Trajectory Smoothness': res_proc["average_smoothness"]\
                    , 'Average Search Space': res_proc["average_total_search_space"], 'Average Memory': res_proc["average memory"] })

        # writing all data points to csv file

        with open('pbtestfull.csv', 'a+', newline='') as file:
            fieldnames2 = ['Algorithm','Time', 'Distance', 'Distance from Goal','Path Deviation','Original Distance from Goal','Trajectory Smoothness', 'Search Space','Memory']
            writer1 = csv.DictWriter(file, fieldnames=fieldnames2)

            if file.tell() == 0:
                # Only write header if file is empty
                writer1.writeheader() 

            for n in range(len(res_proc['time_alldata'])):
                writer1.writerow({'Algorithm': algoname1,  'Time': res_proc["time_alldata"][n], 'Distance': res_proc['distance_alldata'][n],'Path Deviation':res_proc['path_deviation_alldata'][n],'Distance from Goal': res_proc['distance_from_goal_alldata'][n]\
                    , 'Original Distance from Goal': res_proc['original_distance_from_goal_alldata'][n], 'Trajectory Smoothness': res_proc['smoothness_alldata'][n], 'Search Space': res_proc['search_space_alldata'][n]\
                    , 'Memory': res_proc['memory_alldata'][n]})

            '''writer.writerow({'Algorithm': algoname1, 'Time': res_proc["time_alldata"], 'Distance': res_proc["distance_alldata"]\
                ,'Distance from Goal': res_proc["distance_from_goal_alldata"],})'''

        return a_star_res, res_proc

    def __table_to_latex(self, headings, rows) -> str:
        headings = list(map(lambda h: "\\textbf{" + str(h) + "}", headings))
        cols = "c".join(["|" for _ in range(len(headings) + 2)])
        latex_table = "\\begin{tabular}{" + cols + "}" + "\n"
        latex_table += "\\hline" + "\n"
        latex_table += " & ".join(headings) + "\\\\" + "\n"
        latex_table += "\\hline" + "\n"
        for row in rows:
            for i in range(len(row)):
                row[i] = str(row[i])
            latex_table += " & ".join(row) + "\\\\" + "\n"
            latex_table += "\\hline" + "\n"
        latex_table += "\\end{tabular}" + "\n"

        latex_table = latex_table.replace("_", "\\_")
        latex_table = latex_table.replace("%", "\\%")
        latex_table = latex_table.replace("\'", "")

        return latex_table

    def __create_table(self, algorithm_kernels: List[Tuple[Type[Algorithm], Type[BasicTesting], Tuple[list, dict]]],
                       algorithm_results: List[Dict[str, Any]], pick: Dict[str, Tuple[str, str]], exclusive_rows: List[int] = None,
                       show_a_star_res: List[str] = None, show_improvement: List[str] = None, custom_map: Dict[str, List[str]] = None) -> str:
        if not show_a_star_res:
            show_a_star_res = []

        if not show_improvement:
            show_improvement = []

        if not custom_map:
            custom_map = {}

        def __convert_training_name(training_name):
            tokens = training_name.split("_")
            from_idx = 0

            for q in range(len(tokens)):
                if tokens[q] == "training":
                    from_idx = q + 1
                    break

            tokens = tokens[from_idx:-1]
            return "_".join(tokens)

        if not exclusive_rows:
            exclusive_rows = list(range(len(algorithm_results)))

        headings = list(map(lambda v: v[0], pick.values()))
        headings_search = {}
        not_found = "n/a"

        q = 0
        for h in pick.keys():
            headings_search[h] = q
            q += 1

        rows = []
        for i in range(len(algorithm_kernels)):
            if i not in exclusive_rows:
                continue
            rows.append([not_found for _ in range(len(headings))])

        cnt = 0
        for i in range(len(algorithm_kernels)):
            if i not in exclusive_rows:
                continue

            if "nr" in headings_search:
                rows[cnt][headings_search["nr"]] = str(i)

            if "name" in headings_search:
                rows[cnt][headings_search["name"]] = algorithm_kernels[i][0].__name__

                if algorithm_kernels[i][0] == WayPointNavigation:
                    rows[cnt][headings_search["name"]] += " GK: " + algorithm_kernels[i][2][1]["global_kernel"][0].__name__

            if "tr_name" in headings_search:
                if "load_name" in algorithm_kernels[i][2][1]:
                    training_name = __convert_training_name(algorithm_kernels[i][2][1]["load_name"])
                elif algorithm_kernels[i][0] == WayPointNavigation and algorithm_kernels[i][2][1]["global_kernel"][0] == OnlineLSTM:
                    training_name = __convert_training_name(algorithm_kernels[i][2][1]["global_kernel"][1][1]["load_name"])
                else:
                    training_name = not_found

                rows[cnt][headings_search["tr_name"]] = training_name

            for key, value in algorithm_results[i].items():
                l_keys = [key]
                if key in custom_map:
                    l_keys = custom_map[key]

                for k in l_keys:
                    if k not in pick:
                        continue

                    val = str(value) + str(pick[k][1])

                    if key in show_a_star_res:
                        val += " (A*: {}{})".format(str(algorithm_results[i]["a_star_res_proc"][key]), str(pick[k][1]))

                    if key in show_improvement:
                        val += " (I: {}%)".format(str(algorithm_results[i][str(key) + "_improvement"]))

                    rows[cnt][headings_search[k]] = val
            cnt += 1

        return self.__table_to_latex(headings, rows)

    def __tabulate_results(self, algorithm_kernels: List[Tuple[Type[Algorithm], Type[BasicTesting], Tuple[list, dict]]],
                           algorithm_results: List[Dict[str, Any]], with_indexing=False) -> None:
        latex_table0 = self.__create_table(algorithm_kernels, algorithm_results,
                                           {
                                               "nr": ("Nr.", ""),
                                               "name": ("Algorithm Type", ""),
                                               "tr_name": ("Training Data", ""),
                                           })

        latex_table1 = self.__create_table(algorithm_kernels, algorithm_results,
                                          {
                                              "nr": ("Nr.", ""),
                                              "goal_found_perc": ("Success Rate", "%"),
                                              "average_distance": ("Distance", ""),
                                              "average_time": ("Time", "s"),
                                              "average_distance_from_goal": ("Distance Left", "")
                                          }, show_a_star_res=["average_distance"], show_improvement=["goal_found_perc", "average_distance"])

        latex_table2 = self.__create_table(algorithm_kernels, algorithm_results,
                                           {
                                               "nr": ("Nr.", ""),
                                               "kernels_pick_perc": ("Pick Ratio", "%")
                                           }, [11, 14])

        latex_table3 = self.__create_table(algorithm_kernels, algorithm_results,
                                           {
                                               "nr": ("Nr.", ""),
                                               "average_global_kernel_improvement": ("GK Improvement", "%"),
                                               "average_global_kernel_distance": ("GK Distance", ""),
                                               "average_global_kernel_last_way_point_distance_from_goal": ("GK Distance Left", ""),
                                               "average_global_kernel_nr_of_way_points": ("WP", ""),
                                               "average_global_average_way_point_distance": ("WP In-Between Distance", ""),
                                           }, [12, 13, 14])

        latex_table4 = self.__create_table(algorithm_kernels, algorithm_results,
                                           {
                                               "nr": ("Nr.", ""),
                                               "average_local_kernel_total_total": ("Total Search", "%"),
                                               "average_local_kernel_total_fringe": ("Total Fringe", "%"),
                                               "average_local_kernel_average_total": ("Session Search", "%"),
                                               "average_local_kernel_average_fringe": ("Session Fringe", "%"),
                                           }, [0, 12, 13, 14],
                                           show_improvement=["average_local_kernel_total_total",
                                                             "average_local_kernel_total_fringe",
                                                             "average_local_kernel_average_total",
                                                             "average_local_kernel_average_fringe"],
                                           custom_map={
                                               "average_total_search_space": ["average_local_kernel_total_total", "average_local_kernel_average_total"],
                                               "average_fringe": ["average_local_kernel_total_fringe", "average_local_kernel_average_fringe"],
                                           })

        self.__services.debug.write("\\begin{table}[] \n\\footnotesize \n\\centering\n", end="\n\n", timestamp=False, streams=[self.__analysis_stream])
        if with_indexing:
            self.__services.debug.write(latex_table0, end="\n\n", timestamp=False, streams=[self.__analysis_stream])
            self.__services.debug.write("\\bigskip", end="\n\n", timestamp=False, streams=[self.__analysis_stream])
        self.__services.debug.write(latex_table1, end="\n\n", timestamp=False, streams=[self.__analysis_stream])
        self.__services.debug.write("\\bigskip", end="\n\n", timestamp=False, streams=[self.__analysis_stream])
        self.__services.debug.write(latex_table2, end="\n\n", timestamp=False, streams=[self.__analysis_stream])
        self.__services.debug.write("\\bigskip", end="\n\n", timestamp=False, streams=[self.__analysis_stream])
        self.__services.debug.write(latex_table3, end="\n\n", timestamp=False, streams=[self.__analysis_stream])
        self.__services.debug.write("\\bigskip", end="\n\n", timestamp=False, streams=[self.__analysis_stream])
        self.__services.debug.write(latex_table4, end="\n\n", timestamp=False, streams=[self.__analysis_stream])
        self.__services.debug.write("\\caption{} \n\\label{tab:my_label} \n\\end{table}", end="\n\n", timestamp=False, streams=[self.__analysis_stream])

    def analyze_algorithms(self) -> None:
        self.__analysis_stream = StringIO()
        maps: List[Map] = []

        for i in range(45):
            maps.append("testing_maps_pickles/block_map_1000/" + str(i))
            # maps.append("uniform_random_fill_10000/" + str(i*4))
            # maps.append("block_map_10000/" + str(i*4))
            # maps.append("house_10000/" + str(i*4))

        # maps = self.__convert_maps(maps)
        # maps = [Maps.grid_map_small_one_obstacle2]#], Maps.grid_map_labyrinth2]

        '''algorithms: List[Tuple[Type[Algorithm], Type[BasicTesting], Tuple[list, dict]]] = [
            (AStar, AStarTesting, ([], {})),
            (Wavefront, WavefrontTesting, ([], {})),
            (Dijkstra, DijkstraTesting, ([], {})),            
            (OnlineLSTM, BasicTesting, ([], {"load_name": "tile_by_tile_training_uniform_random_fill_10000_model"})),
            (OnlineLSTM, BasicTesting, ([], {"load_name": "tile_by_tile_training_block_map_10000_model"})),
            (OnlineLSTM, BasicTesting, ([], {"load_name": "tile_by_tile_training_house_10000_model"})),
            (OnlineLSTM, BasicTesting, ([], {"load_name": "tile_by_tile_training_uniform_random_fill_10000_block_map_10000_model"})),
            (OnlineLSTM, BasicTesting, ([], {"load_name": "tile_by_tile_training_uniform_random_fill_10000_block_map_10000_house_10000_model"})),
            (OnlineLSTM, BasicTesting, ([], {"load_name": "caelstm_section_lstm_training_uniform_random_fill_10000_model"})),
            (OnlineLSTM, BasicTesting, ([], {"load_name": "caelstm_section_lstm_training_block_map_10000_model"})),
            (OnlineLSTM, BasicTesting, ([], {"load_name": "caelstm_section_lstm_training_house_10000_model"})),
            (OnlineLSTM, BasicTesting, ([], {"load_name": "caelstm_section_lstm_training_uniform_random_fill_10000_block_map_10000_model"})),
            (OnlineLSTM, BasicTesting, ([], {"load_name": "caelstm_section_lstm_training_uniform_random_fill_10000_block_map_10000_house_10000_model"})),
            (CombinedOnlineLSTM, CombinedOnlineLSTMTesting, ([], {})),
            (WayPointNavigation, WayPointNavigationTesting, ([], {"global_kernel_max_it": 20, "global_kernel": (OnlineLSTM, ([], {"load_name": "caelstm_section_lstm_training_block_map_10000_model"}))})),
            (WayPointNavigation, WayPointNavigationTesting, ([], {"global_kernel_max_it": 20, "global_kernel": (OnlineLSTM, ([], {"load_name": "tile_by_tile_training_uniform_random_fill_10000_block_map_10000_house_10000_model"}))})),
            (WayPointNavigation, WayPointNavigationTesting, ([], {"global_kernel_max_it": 20, "global_kernel": (CombinedOnlineLSTM, ([], {}))})),
        ]'''

        algorithms: List[Tuple[Type[Algorithm], Type[BasicTesting], Tuple[list, dict]]] = [
            (AStar, AStarTesting, ([], {}), "A*"),
            #(Wavefront, WavefrontTesting, ([], {}), "Wave-front" ),
            #(Dijkstra, DijkstraTesting, ([], {}), "Dijkstra"),
            #(OnlineLSTM, BasicTesting, ([], {"load_name": "tile_by_tile_training_uniform_random_fill_10000_model"}), "Online LSTM"),
            #(OnlineLSTM, BasicTesting, ([], {"load_name": "tile_by_tile_training_uniform_random_fill_10000_block_map_10000_house_10000_model"}), "Online LSTM (ubh 10000 training)"),
            #(OnlineLSTM, BasicTesting, ([], {"load_name": "caelstm_section_lstm_training_uniform_random_fill_10000_model"}), "CAE Online LSTM"),
            #(OnlineLSTM, BasicTesting, ([], {"load_name": "caelstm_section_lstm_training_uniform_random_fill_10000_block_map_10000_house_10000_model"}), "CAE Online LSTM (ubh 10000 training)"),
            #(CombinedOnlineLSTM, CombinedOnlineLSTMTesting, ([], {}), "Combined Online LSTM"),
            #(WayPointNavigation, WayPointNavigationTesting, ([], {"global_kernel_max_it": 20, "global_kernel": (CombinedOnlineLSTM, ([], {}))}), "WayPointNavigation (Bagging)"),
            #(WayPointNavigation, WayPointNavigationTesting, ([], {"global_kernel_max_it": 20, "global_kernel": (OnlineLSTM, ([], {"load_name": "caelstm_section_lstm_training_block_map_10000_model"}))}), "WayPointNavigation (Map -block training)"),
            #(WayPointNavigation, WayPointNavigationTesting, ([], {"global_kernel_max_it": 20, "global_kernel": (OnlineLSTM, ([], {"load_name": "tile_by_tile_training_uniform_random_fill_10000_block_map_10000_house_10000_model"}))}), "WayPointNavigation (Map -urf training)"),
            #(RT, BasicTesting, ([], {})),
            #(RRT, BasicTesting, ([], {})),
            #(RRT_Star, BasicTesting, ([], {}), "RRT*"),
            #(Bug1, BasicTesting, ([], {}), "Bug 1"),
            #(Bug2, BasicTesting, ([], {}), "Bug 2"),
            #(PotentialField, BasicTesting, ([], {}), "Potential Field"),
            #(OMPL_RRT, BasicTesting, ([], {}), "OMPL RRT"),
            # (OMPL_PRMstar, BasicTesting, ([], {}), "OMPL PRM*"),
            #(RRT_Connect, BasicTesting, ([], {}), "RRT Connect"),
            # (OMPL_LazyPRMstar, BasicTesting, ([], {}), "OMPL Lazy PRM*"),
            # (OMPL_RRTXstatic, BasicTesting, ([], {}), "OMPL RRTX"),
            # (OMPL_RRTstar, BasicTesting, ([], {}), "OMPL RRT*"),
            # (OMPL_RRTsharp, BasicTesting, ([], {}), "OMPL RRT#"),
            # (OMPL_KPIECE1, BasicTesting, ([], {}), "OMPL KPIECE1"),
            # (OMPL_PDST, BasicTesting, ([], {}), "OMPL PDST"),
            # (OMPL_SST, BasicTesting, ([], {}), "OMPL SST"),
            # (OMPL_BiEST, BasicTesting, ([], {}), "OMPL BiEST"),
            # (OMPL_TRRT, BasicTesting, ([], {}), "OMPL TRRT"),
            # (OMPL_RRTConnect, BasicTesting, ([], {}), "OMPL RRT Connect"),
            # (OMPL_BITstar, BasicTesting, ([], {}), "OMPL BIT*"),
            # (OMPL_BKPIECE1, BasicTesting, ([], {}), "OMPL BKPIECE1"),
            # (OMPL_EST, BasicTesting, ([], {}), "OMPL EST"),
            # (OMPL_LazyLBTRRT, BasicTesting, ([], {}), "OMPL LazyLBTRRT"),
            # (OMPL_LazyPRM, BasicTesting, ([], {}), "OMPL LazyPRM"),
            # (OMPL_LazyRRT, BasicTesting, ([], {}), "OMPL LazyRRT"),
            # (OMPL_LBKPIECE1, BasicTesting, ([], {}), "OMPL LBKPIECE1"),
            # (OMPL_LBTRRT, BasicTesting, ([], {}), "OMPL LBTRRT"),
            # (OMPL_PRM, BasicTesting, ([], {}), "OMPL PRM"),
            # (OMPL_STRIDE, BasicTesting, ([], {}), "OMPL STRIDE"), 
            # (OMPL_SBL, BasicTesting, ([], {}), "OMPL SBL"),

        ]

        '''algorithm_names: List[str] = [
            "A*",
            "Wave-front",
            "Dijkstra",               
            "Online LSTM on uniform_random_fill_10000 (paper solution)", # View Module
            "Online LSTM on block_map_10000",
            "Online LSTM on house_10000",
            "Online LSTM on uniform_random_fill_10000_block_map_10000",
            "Online LSTM on uniform_random_fill_10000_block_map_10000_house_10000",
            "CAE Online LSTM on uniform_random_fill_10000", # Map Module
            "CAE Online LSTM on block_map_10000 (paper solution)",
            "CAE Online LSTM on house_10000",
            "CAE Online LSTM on uniform_random_fill_10000_block_map_10000",
            "CAE Online LSTM on uniform_random_fill_10000_block_map_10000_house_10000",
            "Combined Online LSTM (proposed solution)", # Bagging Module
            "WayPointNavigation with local kernel: A* and global kernel: CAE Online LSTM on block_map_10000 (paper solution)",
            "WayPointNavigation with local kernel: A* and global kernel: Online LSTM on uniform_random_fill_10000_block_map_10000_house_10000 (paper solution)",
            "WayPointNavigation with local kernel: A* and global kernel: Combined Online LSTM (proposed solution)",
        ]'''

        algorithm_names: List[str] = [
            "A*",
            #"Wave-front",
            #"Dijkstra",
            #"Online LSTM on uniform_random_fill_10000 (paper solution)",
            #"Online LSTM on uniform_random_fill_10000_block_map_10000_house_10000",
            #"CAE Online LSTM on uniform_random_fill_10000",
            #"CAE Online LSTM on uniform_random_fill_10000_block_map_10000_house_10000",
            #"Combined Online LSTM (proposed solution)",
            #"WayPointNavigation with local kernel: A* and global kernel: Combined Online LSTM (proposed solution)",
            #"WayPointNavigation with local kernel: A* and global kernel: CAE Online LSTM on block_map_10000 (paper solution)",
            #"WayPointNavigation with local kernel: A* and global kernel: Online LSTM on uniform_random_fill_10000_block_map_10000_house_10000 (paper solution)",                                           
            #"RT",
            #"RRT",
            #"RRT*",
            #"Bug1",
            #"Bug2",
            #"PotentialField",
            "VIN",
            #"OMPL RRT",
            # "OMPL PRM*",
            #"RRT-Connect",
            # "OMPL Lazy PRM*",
            # "OMPL RRT*",
            # "OMPL RRT#",
            # "OMPL RRTX",
            # "OMPL KPIECE1",
            # "OMPL PDST",
            # "OMPL SST",
            # "OMPL BiEst",
            # "OMPL TRRT",
            # "OMPL RRTConnect",
            # "OMPL BITstar",
            # "OMPL BKPIECE1",
            # "OMPL EST",
            # "OMPL LazyLBTRRT",
            # "OMPL LazyPRM",
            # "OMPL LazyRRT",
            # "OMPL LBKPIECE1",
            # "OMPL LBTRRT",
            # "OMPL PRM",
            # "OMPL STRIDE",
            # "OMPL SBL",
        ]

        self.__services.debug.write("", timestamp=False, streams=[self.__analysis_stream])
        self.__services.debug.write("Starting basic analysis: number of maps = {}, number of algorithms = {}".format(len(maps), len(algorithms)), end="\n\n", streams=[self.__analysis_stream])

        a_star_res: List[Dict[str, Any]] = None
        algorithm_results: List[Dict[str, Any]] = [None for _ in range(len(algorithms))]

        for idx, (algorithm_type, testing_type, algo_params , algoname) in enumerate(algorithms):
            self.__services.debug.write(str(idx) + ". " + algorithm_names[idx], DebugLevel.BASIC, streams=[self.__analysis_stream])
            
            results: List[Dict[str, Any]] = []

            for _, grid in enumerate(maps):
                results.append(self.__run_simulation(grid, algorithm_type, testing_type, algo_params))  #gets data for the result on map 

            a_star_res, res = self.__report_results(results, a_star_res, algorithm_type, algorithms)
            algorithm_results[idx] = res
        self.__tabulate_results(algorithms, algorithm_results, with_indexing=True)

        df = pd.read_csv('pbtest.csv')
        dffull = pd.read_csv('pbtestfull.csv')

        #df_t = df.T

        print(df)
        print('********************************')
        print(dffull)

        #fig, ax = plt.subplots()
        #fig.set_size_inches(11, 9)




        plot2, axs1 = plt.subplots(ncols=5)
        plot2.set_size_inches(18, 12)

        # 'Algorithm': algostring, 'Time': res_proc["time_alldata"][n], 'Distance': res_proc['distance_alldata'][n],'Distance from Goal': 
        # res_proc['distance_from_goal_alldata'][n], 'Original Distance from Goal': res_proc['original_distance_from_goal_alldata'][n]}

        p1f = sns.violinplot(x="Algorithm", y="Time", data=dffull, ax=axs1[0])
        p1f.set_xticklabels(p1f.get_xticklabels(), rotation=40, ha="right", fontsize=9)
        p1f.set_title('Time vs. Algorithm ')
        p1f.set_ylabel('Time (s)')

        p2f = sns.violinplot(x="Algorithm", y='Distance', data=dffull, ax=axs1[1])
        p2f.set_xticklabels(p1f.get_xticklabels(), rotation=40, ha="right", fontsize=9)
        p2f.set_title('Distance vs. Algorithm ')
        p2f.set_ylabel('Distance')

        p3f = sns.violinplot(x="Algorithm", y='Distance from Goal', data=dffull, ax=axs1[2])
        p3f.set_xticklabels(p1f.get_xticklabels(), rotation=40, ha="right", fontsize=9)
        p3f.set_title('Distance from Goal vs. Algorithm ')
        p3f.set_ylabel('Distance from Goal')

        p4f = sns.violinplot(x="Algorithm", y='Original Distance from Goal', data=dffull, ax=axs1[3])
        p4f.set_xticklabels(p1f.get_xticklabels(), rotation=40, ha="right", fontsize=9)
        p4f.set_title('Original Distance from Goal vs. Algorithm ')
        p4f.set_ylabel('Original Distance from Goal')

        p5f = sns.violinplot(x="Algorithm", y='Path Deviation', data=dffull, ax=axs1[4])
        p5f.set_xticklabels(p1f.get_xticklabels(), rotation=40, ha="right", fontsize=9)
        p5f.set_title('Path Deviation vs. Algorithm ')
        p5f.set_ylabel('Path Deviation')

        # p1f = sns.violinplot(x="Algorithm", y="Time", data=dffull, ax=axs1[4])
        # p1f.set_xticklabels(p1f.get_xticklabels(), rotation=40, ha="right", fontsize=9)
        # p1f.set_title('Time vs. Algorithm ')
        # p1f.set_ylabel('Time (s)')

     
        plot1, axs = plt.subplots(ncols=5)
        plot1.set_size_inches(18, 11)

        #bar plots
        p1 = sns.barplot(x="Algorithm", y="Average Time", data=df, ax=axs[0])
        p1.set_xticklabels(p1.get_xticklabels(), rotation=40, ha="right", fontsize=9)
        p1.set_title('Average Time vs. Algorithm ')
        p1.set_ylabel('Average Time (s)')


        p2 = sns.barplot(x="Algorithm", y="Average Distance", data=df, ax=axs[1])
        p2.set_xticklabels(p1.get_xticklabels(), rotation=40, ha="right", fontsize=9)
        p2.set_title('Average Distance vs. Algorithm ')
        p2.set_ylabel('Average Distance (m)')

        p3 = sns.barplot(x="Algorithm", y="Average Steps", data=df, ax=axs[2])
        p3.set_xticklabels(p1.get_xticklabels(), rotation=40, ha="right", fontsize=9)
        p3.set_title('Average Steps vs. Algorithm ')
        p3.set_ylabel('Average Steps')      

        p4 = sns.barplot(x="Algorithm", y='Success Rate', data=df, ax=axs[3])
        p4.set_xticklabels(p1.get_xticklabels(), rotation=40, ha="right", fontsize=9)
        p4.set_title('Success Rate vs. Algorithm ')
        p4.set_ylabel('Success Rate') 

        # p5 = sns.barplot(x="Algorithm", y="Average Distance from Goal", data=df, ax=axs[4])
        # p5.set_xticklabels(p1.get_xticklabels(), rotation=40, ha="right", fontsize=9)
        # p5.set_title('Average Distance from Goal vs. Algorithm ')
        # p5.set_ylabel('Average Distance from Goal')  

        p6 = sns.barplot(x="Algorithm", y='Average Path Deviation', data=df, ax=axs[4])
        p6.set_xticklabels(p1f.get_xticklabels(), rotation=40, ha="right", fontsize=9)
        p6.set_title('Average Path Deviation vs. Algorithm ')
        p6.set_ylabel('Average Path Deviation') 
        
        #ax = sns.barplot(x="Algorithm", y="Average Time", data=df)
        #plt.xticks(rotation=45)
        plt.tight_layout(pad=2.5, w_pad=1.5, h_pad=1.5)
        plt.show()
        


        # Complex Analysis begins here
            
        # # 6 maps for complex analysis
        # maps: List[Map] = [
        #     # Maps.pixel_map_one_obstacle.convert_to_dense_map(),
        #     "uniform_random_fill_10/4",
        #     "block_map_10/6",
        #     "house_10/6",
        #     Maps.grid_map_labyrinth,
        #     Maps.grid_map_labyrinth2,
        #     Maps.grid_map_one_obstacle.convert_to_dense_map(),
        # ]

       
        # map_names: List[str] = [
        #     "uniform_random_fill_10/4",
        #     "block_map_10/6",
        #     "house_10/6",
        #     "Maps.grid_map_labyrinth",
        #     "Maps.grid_map_labyrinth2",
        #     "Maps.grid_map_one_obstacle.convert_to_dense_map()",
        # ]

        # maps = self.__convert_maps(maps)

        # self.__services.debug.write("\n", timestamp=False, streams=[self.__analysis_stream])
        # self.__services.debug.write("Starting complex analysis: number of maps = {}, number of algorithms = {}".format(len(maps), len(algorithms)), end="\n\n", streams=[self.__analysis_stream])

        # def random_sample_pts(m: Map, nr_of_pts) -> List[Point]:
        #     all_valid_pts = []
        #     for y in range(m.size.height):
        #         for x in range(m.size.width):
        #             new_agent_pos = Point(x, y)
        #             if m.is_agent_valid_pos(new_agent_pos) and \
        #                         not (m.goal.position.x == new_agent_pos.x and m.goal.position.y == new_agent_pos.y):
        #                     all_valid_pts.append(new_agent_pos)

        #     random.shuffle(all_valid_pts)
        #     return all_valid_pts[:nr_of_pts]

        # overall_algo_res = []

        # for _ in range(len(algorithms)):
        #     overall_algo_res.append([])

        # for map_idx, m in enumerate(maps):
        #     # nr_of_samples = 2
        #     nr_of_samples = 50
        #     valid_pts = random_sample_pts(m, nr_of_samples)
        #     self.__services.debug.write("Analysis on map: {}. {} with random samples: {}".format(map_idx, map_names[map_idx], len(valid_pts)), streams=[self.__analysis_stream])
        #     self.__print_ascii_map(m)
        #     self.__services.debug.write("", streams=[self.__analysis_stream], timestamp=False)

        #     for algo_idx, (algorithm_type, testing_type, algo_params) in enumerate(algorithms):
        #         self.__services.debug.write(str(algo_idx) + ". " + algorithm_names[algo_idx], DebugLevel.BASIC, streams=[self.__analysis_stream])
        #         results = []
        #         for valid_pt in valid_pts:
        #             results.append(self.__run_simulation(m, algorithm_type, testing_type, algo_params, valid_pt))

        #         a_star_res, res = self.__report_results(results, a_star_res, algorithm_type)
        #         overall_algo_res[algo_idx] = overall_algo_res[algo_idx] + results
        #         algorithm_results[algo_idx] = res
        #     self.__tabulate_results(algorithms, algorithm_results)

        # self.__services.debug.write("\n", timestamp=False, streams=[self.__analysis_stream])
        # self.__services.debug.write("Complex analysis overall results: \n", streams=[self.__analysis_stream])

        # for idx, overall_res in enumerate(overall_algo_res):
        #     self.__services.debug.write(str(idx) + ". " + algorithm_names[idx], DebugLevel.BASIC, streams=[self.__analysis_stream])
        #     a_star_res, res = self.__report_results(overall_res, a_star_res, algorithms[idx][0])
        #     algorithm_results[idx] = res
        # self.__tabulate_results(algorithms, algorithm_results)

        self.__save_stream("algo")
        self.__analysis_stream = None

    def analyze_training_sets(self) -> None:
        self.__analysis_stream = StringIO()

        training_set_names: List[str] = [
            "training_uniform_random_fill_10000",
            "training_block_map_10000",
            "training_house_10000",
        ]

        self.__services.debug.write("\n", timestamp=False, streams=[self.__analysis_stream])
        self.__services.debug.write("Starting training set analysis: number of training sets = {}".format(len(training_set_names)), end="\n\n", streams=[self.__analysis_stream])

        rows = []

        for t_name in training_set_names:
            self.__services.debug.write("Loading training set: {}".format(t_name), streams=[self.__analysis_stream])
            t = self.__services.resources.training_data_dir.load(t_name)
            self.__services.debug.write("Finished loading\n", streams=[self.__analysis_stream])

            new_dict = {}

            for key, value in t[0].items():
                if key == "features" or key == "labels":
                    value = value[0].keys()
                elif key == "single_features" or key == "single_labels":
                    value = value.keys()
                elif isinstance(value, float) or isinstance(value, int):
                    value = sum(map(lambda d: d[key], t)) / len(t)

                if key == "goal_found":
                    value = str(round(value * 100, 2)) + "%"

                if key == "fringe" or key == "search_space" or key == "map_obstacles_percentage":
                    value = str(round(value, 2)) + "%"

                if isinstance(value, float) or isinstance(value, int):
                    value = round(value, 2)

                new_dict[key] = value

            self.__services.debug.write("Training set: {}\n{}".format(t_name.replace("training_", ""), Debug.pretty_dic_str(new_dict)), streams=[self.__analysis_stream])
            rows.append([t_name, new_dict["goal_found"], new_dict["map_obstacles_percentage"], new_dict["original_distance_to_goal"], new_dict["total_distance"]])

        headings = ["Training Dataset Name", "Path Available", "Obstacles", "Original Distance", "Optimal Travel Distance"]
        latex_table = self.__table_to_latex(headings, rows)
        self.__services.debug.write("", end="\n\n", timestamp=False, streams=[self.__analysis_stream])
        self.__services.debug.write("\\begin{table}[] \n\\footnotesize \n\\centering\n", end="\n\n", timestamp=False, streams=[self.__analysis_stream])
        self.__services.debug.write(latex_table, end="\n\n", timestamp=False, streams=[self.__analysis_stream])
        self.__services.debug.write("\\caption{} \n\\label{tab:my_label} \n\\end{table}", end="\n\n", timestamp=False, streams=[self.__analysis_stream])

        self.__save_stream("map")
        self.__analysis_stream = None

    def __print_ascii_map(self, mp: Map, obstacle_only: bool = True) -> None:
        if not isinstance(mp, DenseMap):
            raise NotImplementedError("Map has to be dense")

        CLEAR_CHAR = "  "
        WALL_CHAR = "██"
        EXTENDED_WALL_CHAR = "██"
        AGENT_CHAR = "AA" if not obstacle_only else CLEAR_CHAR
        GOAL_CHAR = "GG" if not obstacle_only else CLEAR_CHAR

        grid = mp.grid
        grid_rep = [[CLEAR_CHAR for _ in range(mp.size.width + 2)] for _ in range(mp.size.height + 2)]

        for i in range(len(grid_rep)):
            grid_rep[i][0] = WALL_CHAR
            grid_rep[i][-1] = WALL_CHAR

        for j in range(len(grid_rep[0])):
            grid_rep[0][j] = WALL_CHAR
            grid_rep[-1][j] = WALL_CHAR

        for i in range(len(grid)):
            for j in range(len(grid[i])):
                if grid[i][j] == DenseMap.WALL_ID:
                    grid_rep[i + 1][j + 1] = WALL_CHAR

                if grid[i][j] == DenseMap.EXTENDED_WALL:
                    grid_rep[i + 1][j + 1] = EXTENDED_WALL_CHAR

                if grid[i][j] == DenseMap.AGENT_ID:
                    grid_rep[i + 1][j + 1] = AGENT_CHAR

                if grid[i][j] == DenseMap.GOAL_ID:
                    grid_rep[i + 1][j + 1] = GOAL_CHAR

        for i in range(len(grid_rep)):
            for j in range(len(grid_rep[i])):
                self.__services.debug.write(grid_rep[i][j], timestamp=False, end="", streams=[self.__analysis_stream])
            self.__services.debug.write("", timestamp=False, streams=[self.__analysis_stream])

    def __save_stream(self, prefix: str) -> None:
        self.__services.resources.save_log(self.__analysis_stream, prefix + "_analysis_results_log_classical")

    @staticmethod
    def main(m: 'MainRunner') -> None:
        analyzer = Analyzer(m.main_services)

        # analyzer.analyze_training_sets()
        analyzer.analyze_algorithms()
        # analyzer._Analyzer__analysis_stream = StringIO()
        # analyzer._Analyzer__print_ascii_map(Maps.grid_map_labyrinth)



