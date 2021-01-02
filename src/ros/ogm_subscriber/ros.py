import os
import sys
from typing import Optional

import numpy as np
import cv2 as cv

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped

# Add PathBench/src to system path for module imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from algorithms.classic.graph_based.a_star import AStar  # noqa: E402
from algorithms.classic.testing.a_star_testing import AStarTesting  # noqa: E402
from algorithms.configuration.configuration import Configuration  # noqa: E402
from algorithms.configuration.entities.agent import Agent  # noqa: E402
from algorithms.configuration.entities.goal import Goal  # noqa: E402
from algorithms.configuration.maps.dense_map import DenseMap  # noqa: E402
from algorithms.configuration.maps.ros_map import RosMap  # noqa: E402
from simulator.services.debug import DebugLevel  # noqa: E402
from simulator.services.services import Services  # noqa: E402
from simulator.simulator import Simulator  # noqa: E402
from structures import Size, Point  # noqa: E402

class Ros:
    _resolution: Optional[float]
    _origin: Optional[Point]
    _size: Optional[Size]
    _grid: Optional[np.ndarray]
    _sim: Optional[Simulator]

    def __init__(self, fake: bool = False):
        self._resolution = None
        self._origin = None
        self._size = None
        self._grid = None
        self._sim = None

        if fake:
            import ogm_publisher
            self._set_grid(ogm_publisher.load_grid())
        else:
            rospy.init_node("algo")
            rospy.Subscriber("/map", OccupancyGrid, self._set_grid)

    def _set_grid(self, msg):
        minfo = msg.info
        rgrid = np.array(msg.data, dtype=np.int8).astype(np.uint8)

        self._resolution = minfo.resolution
        self._origin = Point(minfo.origin.position.x, minfo.origin.position.y)
        self._size = Size(minfo.width, minfo.height)

        grid = np.empty(self._size[::-1], dtype=np.float32)
        for j in range(self._size.height):
            for i in range(self._size.width):
                grid[j, i] = rgrid[j * self._size.width + i]

        MAX_SIZE = 128
        if MAX_SIZE < self._size.height or MAX_SIZE < self._size.width:
            scale = MAX_SIZE / self._size.height
            if (MAX_SIZE / self._size.width) < scale:
                scale = MAX_SIZE / self._size.width
            grid = cv.resize(grid, None, fx=scale, fy=scale, interpolation=cv.INTER_AREA)

        for idx in np.ndindex(grid.shape):
            grid[idx] = min(1, max(0, 1.0 - grid[idx] / 255.0))

        self._grid = grid

        if self._sim is not None:
            self._sim.services.algorithm.map.request_update()

    def _get_grid(self):
        grid = self._grid
        return grid

    def _setup_sim(self) -> Simulator:
        config = Configuration()

        config.simulator_graphics = True
        config.simulator_write_debug_level = DebugLevel.LOW
        config.simulator_key_frame_speed = 0.16
        config.simulator_key_frame_skip = 20
        config.simulator_algorithm_type = AStar
        config.simulator_algorithm_parameters = ([], {})
        config.simulator_testing_type = AStarTesting
        config.simulator_initial_map = RosMap(Agent(Point(40,40)),
                                              Goal(Point(30, 20)),
                                              self._get_grid)
        s = Services(config)

        while self._grid is None:
            rospy.sleep(0.5)
        s.algorithm.map.request_update()

        sim = Simulator(s)
        return sim

    def _find_goal(self):
        rospy.loginfo("Starting Simulator")
        self._sim = self._setup_sim()
        self._sim.start()

    def start(self):
        self._find_goal()


if __name__ == "__main__":
    ros = Ros(fake=True)
    ros.start()
