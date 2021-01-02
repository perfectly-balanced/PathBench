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
    MAX_SIZE = 128

    def __init__(self):
        self._grid = None
        self._sim = None
        self._ros_origin = None
        self._ros_size = None
        self._ros_res = None
        self._agent = None
        self._scale_width = 1
        self._scale_height = 1

        rospy.init_node("pb3d", log_level=rospy.INFO)
        rospy.Subscriber("/map", OccupancyGrid, self._set_slam)
        rospy.Subscriber('/odom', Odometry, self._set_agent_pos)
        self.pubs = {
            "vel": rospy.Publisher("/cmd_vel", Twist, queue_size=10),  # velocity
        }

    def _set_slam(self, msg):
        map_info = msg.info
        raw_grid = msg.data

        self._ros_size = Size(map_info.width, map_info.height)
        self._ros_res = map_info.resolution
        self._ros_origin = [map_info.origin.position.x, map_info.origin.position.y]

        grid = np.empty(self._ros_size[::-1], dtype=np.float32)

        for i in range(len(raw_grid)):
            col = i % self._ros_size.width
            row = int((i - col) / self._ros_size.width)
            grid[row, col] = raw_grid[i]

        if self.MAX_SIZE is not None:
            self._scale_width = self.MAX_SIZE / self._ros_size.width
            self._scale_height = self.MAX_SIZE / self._ros_size.height
            grid = cv.resize(grid, None, fx=self._scale_width, fy=self._scale_height, interpolation=cv.INTER_AREA)

        grid.resize((self.MAX_SIZE, self.MAX_SIZE))
        if flatten(grid, -1):
            self._grid = grid

        if self._sim is not None:
            self._sim.services.algorithm.map.request_update()

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

            # print()
            # print("Position: {}, Agent Angle: {}".format(agent_pos, agent_rot))
            # print("Angle Left: {}, Dist Left: {}".format(angle_left, dist_left))

            # rotate
            if not np.abs(angle_left) < angle_tresh:
                rot_speed = np.clip(angle_left * rot_multiplier, -1, 1)
                # print("Rotating with speed: {}".format(rot_speed))
                self._send_vel_msg(rot=rot_speed)
                rospy.sleep(sleep)
                continue

            # go forward
            if not dist_left < goal_tresh:
                forward_speed = np.clip(dist_left * forward_multiplier, 0, 0.5)
                #print("Moving with speed: {}".format(forward_speed))
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

        print("Agent Position: {}".format(agent_pos))
        print(self._ros_origin, [self._agent.pose.position.x, self._agent.pose.position.y])
        print(self._ros_size, Size(*self._grid.shape))
        print(self._ros_res)

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

    def _world_to_grid(self, pos, scaled: bool = True):
        '''
        converts from meters coordinates to grid coordinates.
        If `scaled`, convert to PathBench's OGM size, otherwise use ROS's size.
        '''
        
        scale_width = self._scale_width if scaled else 1
        scale_height = self._scale_height if scaled else 1

        grid_pos = [pos[0] - self._ros_origin[0], pos[1] - self._ros_origin[1]]
        grid_pos = [grid_pos[0] / self._ros_res,
                    grid_pos[1] / self._ros_res]
        grid_pos[0] = int(round(scale_width * (grid_pos[0])))
        grid_pos[1] = int(round(scale_height * (grid_pos[1])))

        return Point(*reversed(grid_pos))

    def _grid_to_world(self, pos, scaled: bool = True):
        '''
        converts grid coordinates to meters coordinates.
        If `scaled`, assume grid coordinates are relative to 
        PathBench's OGM size, otherwise assume ROS's size.
        '''

        if scaled:
            pos = (pos[0] / self._scale_height, pos[1] / self._scale_width)

        world_pos = [pos[1], pos[0]]
        world_pos = [world_pos[0] * self._ros_res + self._ros_res * 0.5,
                     world_pos[1] * self._ros_res + self._ros_res * 0.5]
        world_pos = [world_pos[0] + self._ros_origin[0],
                     world_pos[1] + self._ros_origin[1]]

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
