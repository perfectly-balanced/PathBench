import os
import sys
import argparse
from typing import Optional

from nptyping import NDArray
import numpy as np
import cv2 as cv

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, PoseWithCovariance

# Add PathBench/src to system path for module imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from algorithms.algorithm_manager import AlgorithmManager  # noqa: E402
from algorithms.configuration.configuration import Configuration  # noqa: E402
from algorithms.configuration.entities.agent import Agent  # noqa: E402
from algorithms.configuration.entities.goal import Goal  # noqa: E402
from algorithms.configuration.maps.dense_map import DenseMap  # noqa: E402
from algorithms.configuration.maps.ros_map import RosMap  # noqa: E402

from simulator.services.debug import DebugLevel  # noqa: E402
from simulator.services.services import Services  # noqa: E402
from simulator.simulator import Simulator  # noqa: E402
from structures import Size, Point  # noqa: E402

import utility.math as m  # noqa: E402
from utility.misc import flatten  # noqa: E402
from utility.argparse import add_configuration_flags  # noqa: E402
from utility.threading import Lock  # noqa: E402

class Ros:
    INFLATE: int = 2  # radius of agent for extended walls.
    INIT_MAP_SIZE: int = 128  # maximum map size for the first received map (this determines the scaling factor for all subsequent map updates).
    MAP_SIZE: int = 512  # the overall map size, can be as big as you like. Note, the initial map fragment will be located at the center of this 'big' map.
    TRAVERSABLE_THRESHOLD: int = 30  # weight grid values above this value are considered to be obstacles

    _sim: Optional[Simulator]  # simulator
    _grid: Optional[NDArray[(MAP_SIZE, MAP_SIZE), np.float32]]  # weight grid, shape (width, height), weight bounds: (0, 100), unmapped: -1
    _size: Optional[Size]  # size of the 'big' map (MAP_SIZE, MAP_SIZE).
    _res: Optional[float]  # map resolution (determined by initial map fragment).
    _scale: Optional[float]  # scale factor from raw map fragment to PathBench OGM grid (same factor for both x and y-axis)
    _agent: Optional[PoseWithCovariance]  # latest agent pose data
    _agent_lock: Lock
    _grid_lock: Lock

    def __init__(self) -> None:
        self._grid = None
        self._sim = None
        self._origin = None
        self._size = None
        self._res = None
        self._scale = None
        self._agent = None
        self._agent_lock = Lock()
        self._grid_lock = Lock()

        # initialise ROS node
        rospy.init_node("path_bench", log_level=rospy.INFO)
        rospy.Subscriber("/map", OccupancyGrid, self._set_slam)
        rospy.Subscriber('/odom', Odometry, self._update_agent)
        self.pubs = {
            "vel": rospy.Publisher("/cmd_vel", Twist, queue_size=10),  # velocity
        }

    def _set_slam(self, msg: OccupancyGrid) -> None:
        map_info = msg.info
        grid_data = msg.data

        if self._size is None:  # init #
            self._size = Size(self.MAP_SIZE, self.MAP_SIZE)
            self._res = map_info.resolution

            self._scale = self.INIT_MAP_SIZE / map_info.height
            if (self.INIT_MAP_SIZE / map_info.width) < self._scale:
                self._scale = self.INIT_MAP_SIZE / map_info.width

        # convert raw grid data into a matrix for future processing (compression)
        raw_grid = np.empty((map_info.height, map_info.width), dtype=np.float32)
        for i in range(len(grid_data)):
            col = i % map_info.width
            row = int((i - col) / map_info.width)
            raw_grid[row, col] = grid_data[i]

        # compress the map to a suitable size (maintains aspect ratio)
        raw_grid = cv.resize(raw_grid, None, fx=self._scale, fy=self._scale, interpolation=cv.INTER_AREA)

        if self._origin is None:  # init #
            # set the origin to the big map origin, use the raw grid origin with
            # negative grid position to retrieve the big map's origin in world coordinates
            self._origin = Point(map_info.origin.position.x, map_info.origin.position.y)
            init_map_size = Size(*raw_grid.shape[::-1])
            map_origin_pos = Point((init_map_size.width - self._size.width) // 2, (init_map_size.height - self._size.height) // 2)
            self._origin = self._grid_to_world(map_origin_pos)

        # get position of big map origin in current raw map
        start = self._world_to_grid(self._origin, origin=Point(map_info.origin.position.x, map_info.origin.position.y))

        # take an offsetted, potentially cropped view of the current raw map
        grid = np.full(self._size, -1)
        for i in range(start[0], start[0] + self._size.width):
            for j in range(start[1], start[1] + self._size.height):
                if i >= 0 and j >= 0 and i < raw_grid.shape[1] and j < raw_grid.shape[0]:
                    grid[i - start[0]][j - start[1]] = raw_grid[j][i]

        # hacky work-around for not having extended walls implemented. Here we manually
        # extend the walls, by placing obstacles (works just as well, but should still
        # ideally implement extended walls).
        INVALID_VALUE = -2
        grid_walls_extended = np.full(self._size, INVALID_VALUE)
        for idx in np.ndindex(grid_walls_extended.shape):
            if grid_walls_extended[idx] == INVALID_VALUE:
                grid_walls_extended[idx] = grid[idx]
                if grid[idx] > self.TRAVERSABLE_THRESHOLD:
                    for i in range(-self.INFLATE, self.INFLATE+1):
                        for j in range(-self.INFLATE, self.INFLATE+1):
                            grid_walls_extended[(idx[0]+i, idx[1]+j)] = grid[idx]

        # make new grid accessible.
        # Note, it's up to the user / algorithm to request a map update for thread
        # safety, e.g. via `self._sim.services.algorithm.map.request_update()`.
        self.grid = grid_walls_extended

    @property
    def grid(self) -> str:
        return 'grid'

    @grid.setter
    def grid(self, value: NDArray[(MAP_SIZE, MAP_SIZE), np.float32]) -> None:
        self._grid_lock.acquire()
        self._grid = value
        self._grid_lock.release()

    @grid.getter
    def grid(self) -> Optional[NDArray[(MAP_SIZE, MAP_SIZE), np.float32]]:
        self._grid_lock.acquire()
        grid = self._grid
        self._grid_lock.release()
        return grid

    def _update_agent(self, msg: Odometry) -> None:
        self.agent = msg.pose

    @property
    def agent(self) -> str:
        return 'agent'

    @agent.setter
    def agent(self, value: PoseWithCovariance) -> None:
        self._agent_lock.acquire()
        self._agent = value
        self._agent_lock.release()

    @agent.getter
    def agent(self) -> Optional[PoseWithCovariance]:
        self._agent_lock.acquire()
        agent = self._agent
        self._agent_lock.release()
        return agent

    @staticmethod
    def unit_vector(v):
        return v / np.linalg.norm(v)

    @staticmethod
    def angle(v1, v2):
        v1 = unit_vector(v1)
        v2 = unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))

    def _send_way_point(self, wp: Point) -> None:
        """
        Sends velocity commands to get the robot to move to the given way point.
        """

        goal_tresh = 0.1
        angle_tresh = 0.1
        sleep = 0.01
        max_it = 1000
        rot_multiplier = 5
        forward_multiplier = 0.5
        found = False

        wp = np.array(self._grid_to_world(wp))
        rospy.loginfo("Sending waypoint: {}".format(wp))

        for _ in range(max_it):
            agent_pos = np.array([self.agent.pose.position.x, self.agent.pose.position.y])
            q_agent_orientation = self.agent.pose.orientation
            q_agent_orientation = [q_agent_orientation.x, q_agent_orientation.y,
                                   q_agent_orientation.z, q_agent_orientation.w]
            agent_rot = m.euler_from_quaternion(q_agent_orientation, axes='sxyz')[0]

            goal_dir = wp - agent_pos
            goal_rot = np.arctan2(goal_dir[1], goal_dir[0])
            angle_left = np.sign(goal_rot - agent_rot) * (np.abs(goal_rot - agent_rot) % np.pi)
            dist_left = np.linalg.norm(goal_dir)

            # rotate
            if not np.abs(angle_left) < angle_tresh:
                rot_speed = np.clip(angle_left * rot_multiplier, -1, 1)
                self._send_vel_msg(rot=rot_speed)
                rospy.sleep(sleep)
                continue

            # go forward
            if not dist_left < goal_tresh:
                forward_speed = np.clip(dist_left * forward_multiplier, 0, 0.5)
                self._send_vel_msg(vel=forward_speed)
                rospy.sleep(sleep)
                continue
            else:
                found = True
                break

        # stop
        self._send_vel_msg()

        rospy.loginfo("Waypoint found: {}".format(found))

    def _send_vel_msg(self, vel=None, rot=None) -> None:
        """
        Send velocity.
        """

        if not vel:
            vel = 0
        if not rot:
            rot = 0

        vel = [vel, 0, 0]
        rot = [0, 0, rot]

        vel_msg = Twist()
        vel_msg.linear.x, vel_msg.linear.y, vel_msg.linear.z = vel
        vel_msg.angular.x, vel_msg.angular.y, vel_msg.angular.z = rot
        self.pubs["vel"].publish(vel_msg)

    def _map_update_requested(self) -> None:
        """
        Map update was requested.
        """
        pass

    def _world_to_grid(self, world_pos: Point, origin: Optional[Point] = None) -> Point:
        """
        Converts from meters coordinates to PathBench's grid coordinates (`self.grid`).
        """

        # bottom-left corner of the grid to convert to
        if origin is None:
            origin = self._origin

        grid_pos = world_pos
        grid_pos = grid_pos - origin
        grid_pos = grid_pos / self._res
        grid_pos = grid_pos * self._scale
        grid_pos = Point(*np.rint(grid_pos.values))

        return grid_pos

    def _grid_to_world(self, grid_pos: Point) -> Point:
        """
        Converts PathBench's grid coordinates (`self.grid`) to meters coordinates.
        """

        world_pos = grid_pos
        world_pos = world_pos / self._scale
        world_pos = world_pos * self._res
        world_pos = world_pos + self._origin

        return world_pos

    def _setup_sim(self, config: Optional[Configuration] = None, goal: Optional[Point] = None) -> Simulator:
        """
        Sets up the simulator (e.g. algorithm and map configuration).
        """

        while self.grid is None or self.agent is None:
            rospy.loginfo("Waiting for grid and agent to initialise...")
            rospy.sleep(0.5)

        if config is None:
            config = Configuration()

        # general
        config.simulator_graphics = True
        config.simulator_key_frame_speed = 0.16
        config.simulator_key_frame_skip = 20
        config.get_agent_position = lambda: self._world_to_grid(Point(self.agent.pose.position.x, self.agent.pose.position.y))
        config.visualiser_simulator_config = False  # hide the simulator config window

        # algorithm
        if config.algorithm_name is None:
            config.algorithm_name = "Global Way-point LSTM"
            config.simulator_algorithm_type, config.simulator_testing_type, config.simulator_algorithm_parameters = config.algorithms[config.algorithm_name]

        # map
        goal = Goal(Point(0, 0) if goal is None else goal)
        agent = Agent(self._world_to_grid(Point(self.agent.pose.position.x, self.agent.pose.position.y)),
                      radius=self.INFLATE)

        mp = RosMap(agent,
                    goal,
                    lambda: self.grid,
                    traversable_threshold=self.TRAVERSABLE_THRESHOLD,
                    unmapped_value=-1,
                    wp_publish=self._send_way_point,
                    update_requested=self._map_update_requested,
                    name="ROS Map")

        config.maps = {mp.name: mp}
        config.simulator_initial_map = list(config.maps.values())[0]
        config.map_name = list(config.maps.keys())[0]

        # create the simulator
        s = Services(config)
        s.algorithm.map.request_update()
        sim = Simulator(s)
        return sim

    def start(self, config: Optional[Configuration] = None, goal: Optional[Point] = None) -> None:
        """
        Start the simulator.
        """

        rospy.loginfo("Starting simulator")
        self._sim = self._setup_sim(config, goal)
        self._sim.start()


def main() -> bool:
    parser = argparse.ArgumentParser(prog="ros.py",
                                     description="PathBench 2D ROS extension runner",
                                     formatter_class=argparse.RawTextHelpFormatter)

    configurers: List[Callable[[Configuration, argparse.Namespace], bool]] = []
    configurers.append(add_configuration_flags(parser, visualiser_flags=True, algorithms_flags=True, multiple_algorithms_specifiable=False))

    parser.add_argument("-g", "--goal", nargs=2, type=int, help="goal position \"x y\"")

    args = parser.parse_args()
    print("args:{}".format(args))

    config = Configuration()

    for c in configurers:
        if not c(config, args):
            return False

    if args.algorithm:
        config.algorithm_name = list(config.algorithms.keys())[0]
        config.simulator_algorithm_type, config.simulator_testing_type, config.simulator_algorithm_parameters = config.algorithms[config.algorithm_name]

    goal = Point(*args.goal) if args.goal else None

    ros = Ros()
    ros.start(config, goal)

    return True


if __name__ == "__main__":
    ret = main()
    exit_code = 0 if ret else 1
    sys.exit(exit_code)
