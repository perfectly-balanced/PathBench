import os
import sys
import argparse
from typing import Optional, Any

from nptyping import NDArray
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
from utility.threading import Lock  # noqa: E402
from utility.argparse import add_configuration_flags  # noqa: E402

class Ros:
    _sim: Optional[Simulator]
    _resolution: Optional[float]
    _origin: Optional[Point]
    _size: Optional[Size]
    _grid: Optional[NDArray[(Any, Any), np.float32]]
    _grid_lock: Lock

    def __init__(self, fake: bool = False):
        self._resolution = None
        self._origin = None
        self._size = None
        self._grid = None
        self._sim = None
        self._grid_lock = Lock()

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

        self.grid = grid

    @property
    def grid(self) -> str:
        return 'grid'

    @grid.setter
    def grid(self, value: NDArray[(Any, Any), np.float32]) -> None:
        self._grid_lock.acquire()
        self._grid = value
        self._grid_lock.release()

    @grid.getter
    def grid(self) -> Optional[NDArray[(Any, Any), np.float32]]:
        self._grid_lock.acquire()
        grid = self._grid
        self._grid_lock.release()
        return grid

    def _setup_sim(self, config: Optional[Configuration] = None) -> Simulator:
        """
        Sets up the simulator (e.g. algorithm and map configuration).
        """

        while self.grid is None:
            rospy.loginfo("Waiting for grid to initialise...")
            rospy.sleep(0.5)

        if config is None:
            config = Configuration()

        # general
        config.simulator_graphics = True
        config.simulator_key_frame_speed = 0.16
        config.simulator_key_frame_skip = 20
        config.visualiser_simulator_config = False  # hide the simulator config window

        # algorithm
        if config.algorithm_name is None:
            config.algorithm_name = "A*"
            config.simulator_algorithm_type, config.simulator_testing_type, config.simulator_algorithm_parameters = config.algorithms[config.algorithm_name]

        # map
        mp = RosMap(Agent(Point(40, 40)),
                    Goal(Point(30, 20)),
                    lambda: self.grid,
                    traversable_threshold=0.4,
                    name="ROS Map")

        config.maps = {mp.name: mp}
        config.simulator_initial_map = list(config.maps.values())[0]
        config.map_name = list(config.maps.keys())[0]

        # create the simulator
        s = Services(config)
        s.algorithm.map.request_update()
        sim = Simulator(s)
        return sim

    def start(self, config: Optional[Configuration] = None) -> None:
        """
        Start the simulator.
        """

        rospy.loginfo("Starting simulator")
        self._sim = self._setup_sim(config)
        self._sim.start()

def main() -> bool:
    parser = argparse.ArgumentParser(prog="ros.py",
                                     description="ROS 2D Occupancy Grid Publish-Subscriber Demo",
                                     formatter_class=argparse.RawTextHelpFormatter)

    configurers: List[Callable[[Configuration, argparse.Namespace], bool]] = []
    configurers.append(add_configuration_flags(parser, visualiser_flags=True, algorithms_flags=True, multiple_algorithms_specifiable=False))

    parser.add_argument("-f", "--fake", action="store_true", help="fake the ROS publisher")

    args = parser.parse_args()
    print("args:{}".format(args))

    config = Configuration()

    for c in configurers:
        if not c(config, args):
            return False

    if args.algorithm:
        config.algorithm_name = list(config.algorithms.keys())[0]
        config.simulator_algorithm_type, config.simulator_testing_type, config.simulator_algorithm_parameters = config.algorithms[config.algorithm_name]

    ros = Ros(fake=args.fake)
    ros.start(config)

    return True


if __name__ == "__main__":
    ret = main()
    exit_code = 0 if ret else 1
    sys.exit(exit_code)
