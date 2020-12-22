import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped

import threading
from threading import Lock, Condition

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from algorithms.classic.graph_based.a_star import AStar
from algorithms.classic.testing.a_star_testing import AStarTesting
from algorithms.configuration.configuration import Configuration
from algorithms.configuration.entities.agent import Agent
from algorithms.configuration.entities.goal import Goal
from algorithms.configuration.maps.dense_map import DenseMap
from algorithms.configuration.maps.ros_map import RosMap
from simulator.services.debug import DebugLevel
from simulator.services.services import Services
from simulator.simulator import Simulator
from structures import Size, Point
from maps import Maps

class Ros:
    _resolution: float
    _origin: Point
    _size: Size
    _grid: np.ndarray

    def __init__(self):
        self._resolution = None
        self._origin = None
        self._size = None
        self._grid = None

        rospy.init_node("algo")
        rospy.Subscriber("/map", OccupancyGrid, self._set_slam)
        self.agent = None
        self.grid = None
        rospy.sleep(2)

    def _set_slam(self, msg):
        minfo = msg.info
        rgrid = np.array(msg.data, dtype=np.int8).astype(np.uint8)

        self._resolution = minfo.resolution
        self._origin = Point(minfo.origin.position.x, minfo.origin.position.y)
        self._size = Size(minfo.width, minfo.height)
        self._grid = np.empty(self._size[::-1], dtype=np.float32)

        for j in range(self._size.height):
            for i in range(self._size.width):
                v = rgrid[j * self._size.width + i]
                self._grid[j, i] = min(1, max(0, 1 - v / 255))

    def _get_grid(self):
        grid = self._grid
        return (grid, (0, 1), 0.95)

    def _set_agent_pos(self, odom_msg):
        self.agent = odom_msg

    def _get_agent_pos(self):
        ret = self.agent
        return ret

    def _update_requested(self):
        pass  # request slam

    def _setup_sim(self) -> Simulator:
        config = Configuration()

        config.simulator_graphics = True
        config.simulator_write_debug_level = DebugLevel.LOW
        config.simulator_key_frame_speed = 0.16
        config.simulator_key_frame_skip = 20
        config.simulator_algorithm_type = AStar
        config.simulator_algorithm_parameters = ([], {})
        config.simulator_testing_type = AStarTesting
        config.simulator_initial_map = RosMap(self._size,
                                              Agent(Point(0, 0)),
                                              Goal(Point(0, 1)),
                                              self._get_grid,
                                              self._update_requested)
        s = Services(config)
        s.algorithm.map.request_update()
        sim = Simulator(s)
        return sim

    def _find_goal(self):
        rospy.loginfo("Starting Simulator")
        sim = self._setup_sim()
        sim.start()

    def start(self):
        self._find_goal()


if __name__ == "__main__":
    ros = Ros()
    ros.start()
