import os
import sys

import numpy as np
import cv2 as cv

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist

# Add PathBench/src to system path for module imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from algorithms.classic.graph_based.a_star import AStar  # noqa: E402
from algorithms.classic.testing.a_star_testing import AStarTesting  # noqa: E402
from algorithms.classic.testing.way_point_navigation_testing import WayPointNavigationTesting  # noqa: E402
from algorithms.lstm.a_star_waypoint import WayPointNavigation  # noqa: E402
from algorithms.lstm.combined_online_LSTM import CombinedOnlineLSTM  # noqa: E402
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

class Ros:
    INFLATE = 3
    START_MAP_MAX_SIZE = 128
    MAP_SIZE = 256

    def __init__(self):
        self._grid = None
        self._sim = None
        self._origin = None
        self._size = None
        self._res = None
        self._agent = None
        self._scale = None

        rospy.init_node("pb3d", log_level=rospy.INFO)
        rospy.Subscriber("/map", OccupancyGrid, self._set_slam)
        rospy.Subscriber('/odom', Odometry, self._set_agent_pos)
        self.pubs = {
            "vel": rospy.Publisher("/cmd_vel", Twist, queue_size=10),  # velocity
        }

    def _set_slam(self, msg):
        map_info = msg.info
        grid_data = msg.data

        if self._size is None:
            self._size = Size(self.MAP_SIZE, self.MAP_SIZE)
            self._res = map_info.resolution
            self._scale = self.START_MAP_MAX_SIZE / map_info.height
            if (self.START_MAP_MAX_SIZE / map_info.width) < self._scale:
                self._scale = self.START_MAP_MAX_SIZE / map_info.width

            # set the origin to the big map origin, use the current map origin with
            # negative grid position to retrieve the big map's origin in world coordinates
            self._origin = [map_info.origin.position.x, map_info.origin.position.y]
            start_map_size_scaled = Size(int(round(self._scale * map_info.width)), int(round(self._scale * map_info.height)))
            map_origin = Point(-(self.MAP_SIZE - start_map_size_scaled.width) // 2, -(self.MAP_SIZE - start_map_size_scaled.height) // 2)
            self._origin = self._grid_to_world(map_origin)

        raw_grid = np.empty((map_info.height, map_info.width), dtype=np.float32)

        for i in range(len(grid_data)):
            col = i % map_info.width
            row = int((i - col) / map_info.width)
            raw_grid[row, col] = grid_data[i]

        raw_grid = cv.resize(raw_grid, None, fx=self._scale, fy=self._scale, interpolation=cv.INTER_AREA)

        # get position of original origin in new map
        start = self._world_to_grid(self._origin, origin=[map_info.origin.position.x, map_info.origin.position.y])

        # take an offsetted, cropped view of the raw grid
        grid = np.full(self._size, -1)
        for i in range(start[0], start[0] + self._size.width):
            for j in range(start[1], start[1] + self._size.height):
                if i >= 0 and j >= 0 and i < raw_grid.shape[1] and j < raw_grid.shape[0]:
                    grid[i - start[0]][j - start[1]] = raw_grid[j][i]

        # make new grid accessible. Note, it's up to the algorithm to request a map update for thread safety.
        self._grid = grid

    def _get_grid(self):
        return self._grid

    def _set_agent_pos(self, msg):
        self._agent = msg.pose

    @staticmethod
    def unit_vector(v):
        return v / np.linalg.norm(v)

    @staticmethod
    def angle(v1, v2):
        v1 = unit_vector(v1)
        v2 = unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))

    def _send_way_point(self, wp):
        goal_tresh = 0.1
        angle_tresh = 0.1
        sleep = 0.01
        max_it = 1000
        rot_multiplier = 5
        forward_multiplier = 0.5
        found = False

        grid_wp = wp
        wp = np.array(self._grid_to_world(wp))
        rospy.loginfo("Sending waypoint: {}".format(wp))

        for _ in range(max_it):
            agent_pos = np.array([self._agent.pose.position.x, self._agent.pose.position.y])
            q_agent_orientation = self._agent.pose.orientation
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

    def _send_vel_msg(self, vel=None, rot=None):
        '''
        send velocity 
        '''
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

    def _setup_sim(self) -> Simulator:
        while self._grid is None or self._agent is None:
            print("Sleep...")
            rospy.sleep(0.5)

        agent_pos = self._world_to_grid([self._agent.pose.position.x, self._agent.pose.position.y])

        config = Configuration()

        # general
        config.simulator_graphics = True
        config.simulator_write_debug_level = DebugLevel.LOW
        config.simulator_key_frame_speed = 0.16
        config.simulator_key_frame_skip = 20
        config.get_agent_position = lambda: self._world_to_grid([self._agent.pose.position.x, self._agent.pose.position.y])
        config.visualiser_simulator_config = False

        # algorithm
        config.algorithms = {"Global Way-point LSTM": (WayPointNavigation,
                                                       WayPointNavigationTesting,
                                                       ([],
                                                        {"global_kernel": (CombinedOnlineLSTM, ([], {})),
                                                         "global_kernel_max_it": 30}))}
        config.simulator_algorithm_type, config.simulator_testing_type, config.simulator_algorithm_parameters = list(config.algorithms.values())[0]
        config.algorithm_name = list(config.algorithms.keys())[0]

        # map
        mp = RosMap(Agent(agent_pos, radius=self.INFLATE),
                    Goal(Point(0, 0)),
                    self._get_grid,
                    traversable_threshold=50,
                    unmapped_value=-1,
                    wp_publish=self._send_way_point,
                    name="ROS Map")

        config.maps = {mp.name: mp}
        config.simulator_initial_map = list(config.maps.values())[0]
        config.map_name = list(config.maps.keys())[0]

        s = Services(config)
        s.algorithm.map.request_update()
        sim = Simulator(s)
        return sim

    def _world_to_grid(self, pos, origin=None):
        '''
        converts from meters coordinates to grid coordinates.
        If `scaled`, convert to PathBench's OGM size, otherwise use ROS's size.
        '''

        if origin is None:
            origin = self._origin

        grid_pos = [pos[0] - origin[0], pos[1] - origin[1]]
        grid_pos = [x / self._res for x in grid_pos]
        grid_pos[0] = int(round(self._scale * grid_pos[0]))
        grid_pos[1] = int(round(self._scale * grid_pos[1]))

        return Point(*grid_pos)

    def _grid_to_world(self, pos):
        '''
        converts grid coordinates to meters coordinates.
        If `scaled`, assume grid coordinates are relative to 
        PathBench's OGM size, otherwise assume ROS's size.
        '''

        world_pos = pos
        world_pos = [p / self._scale for p in world_pos]
        world_pos = [p * self._res for p in world_pos]
        world_pos = [world_pos[0] + self._origin[0],
                     world_pos[1] + self._origin[1]]

        return world_pos

    def _find_goal(self):
        rospy.loginfo("Starting Simulator")
        self._sim = self._setup_sim()
        self._sim.start()

    def start(self):
        self._find_goal()


if __name__ == "__main__":
    ros = Ros()
    ros.start()
