import os
import sys
import threading
from threading import Lock, Condition

import numpy as np

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped

# Add PathBench/src to system path for module imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from algorithms.classic.testing.way_point_navigation_testing import WayPointNavigationTesting  # noqa: E402
from algorithms.classic.testing.a_star_testing import AStarTesting  # noqa: E402
from algorithms.configuration.configuration import Configuration  # noqa: E402
from algorithms.configuration.entities.agent import Agent  # noqa: E402
from algorithms.configuration.entities.goal import Goal  # noqa: E402
from algorithms.configuration.maps.dense_map import DenseMap  # noqa: E402
from algorithms.configuration.maps.ros_map import RosMap  # noqa: E402
from algorithms.lstm.a_star_waypoint import WayPointNavigation  # noqa: E402
from algorithms.lstm.combined_online_LSTM import CombinedOnlineLSTM  # noqa: E402
from algorithms.lstm.LSTM_tile_by_tile import OnlineLSTM  # noqa: E402

from maps import Maps  # noqa: E402
from simulator.services.debug import DebugLevel  # noqa: E402
from simulator.services.services import Services  # noqa: E402
from simulator.simulator import Simulator  # noqa: E402
from structures import Size, Point  # noqa: E402

import utility.math as m  # noqa: E402


class Ros:
    MAP_SIZE = None
    REZ = None
    ORIGIN = None
    INFLATE = 3
    SIZE = 128
    SLAM_MAP = None

    def __init__(self):
        self._grid = None
        self._grid_lock = Lock()
        self._agent_lock = Lock()
        # self._wp_cond = Condition()
        self._current_wp = None
        self._cur_wp = None
        self.goal = Point(70, 70)

        rospy.init_node("lstm1", log_level=rospy.INFO)
        rospy.Subscriber("/map", OccupancyGrid, self._set_slam)
        rospy.Subscriber('/odom', Odometry, self.__odometryCb)
        # rospy.Subscriber("/odom",Odometry, self._set_agent_pos)
        # rospy.Subscriber("/robot_pose", PoseStamped, self._set_agent_pos) #this sets self.agent
        self.pubs = {
            "vel": rospy.Publisher("/cmd_vel", Twist, queue_size=10),  # velocity
        }
        self.agent = None
        self.grid = None
        rospy.sleep(2)

    def _set_slam(self, msg):
        self._grid_lock.acquire()

        map_info = msg.info
        raw_grid = msg.data

        if not self.MAP_SIZE:
            self.MAP_SIZE = Size(self.SIZE, self.SIZE)  # 128x128
            self.REZ = map_info.resolution
            self.ORIGIN = [map_info.origin.position.x, map_info.origin.position.y]

        self.SLAM_MAP = Size(map_info.width, map_info.height)  # 4000x4000

        #self.MAP_SIZE = Size(map_info.width, map_info.height)
        #self.REZ = map_info.resolution
        #self.ORIGIN = map_info.origin

        # rospy.loginfo("w: {}, h: {}, res: {}".format(self.MAP_SIZE.width,
        #                                        self.MAP_SIZE.height, self.REZ))
        #rospy.loginfo("Origin: {}".format(self.ORIGIN))

        """
        cnt = 0
        for x in grid:
            if x != -1:
                x += 1
        rospy.loginfo("cnt: {}".format(cnt))
        """
        # raw_grid = list(map(lambda el: 1 if el == -1 or el > 50 else 0, raw_grid))

        """
        raw_grid = np.array(raw_grid).reshape(map_info.height, map_info.width)
        self._grid = raw_grid[:self.MAP_SIZE.height, :self.MAP_SIZE.width].tolist()
        """

        grid = [[0 for _ in range(map_info.width)] for _ in range(map_info.height)]

        for i in range(len(raw_grid)):
            col = i % map_info.width
            row = int((i - col) / map_info.width)
            grid[map_info.height - row - 1][col] = raw_grid[i]

        start = self._world_to_grid(self.ORIGIN, Size(map_info.width, map_info.height), [map_info.origin.position.x, map_info.origin.position.y])
        start = Point(start.x, map_info.height - start.y - 1)

        grid2 = [[100 for _ in range(self.MAP_SIZE.width)] for _ in range(self.MAP_SIZE.height)]

        for i in range(start[1], start[1] + self.MAP_SIZE.height):
            for j in range(start[0], start[0] + self.MAP_SIZE.width):
                if i >= 0 and j >= 0 and j < map_info.width and i < map_info.height:
                    grid2[i - start[1]][j - start[0]] = grid[i][j]
        grid = grid2

        available = []

        for i in range(len(grid)):
            for j in range(len(grid[i])):
                if 0 <= grid[i][j] < 50:
                    available.append((i, j))

        for (i, j) in available:
            for (r, c) in zip([1, 1, 0, -1, -1, -1, 0, 1], [0, -1, -1, -1, 0, 1, 1, 1]):
                r = i + r
                c = j + c
                if r >= 0 and r < len(grid) and c >= 0 and c < len(grid[i]):
                    if grid[r][c] == -1:
                        grid[r][c] = 0

        for i in range(len(grid)):
            for j in range(len(grid[i])):
                grid[i][j] = 1 if grid[i][j] == -1 or grid[i][j] > 50 else 0

        # print('Grid$$$$$$$$$$$$$$$$$$$', grid)
        self._grid = grid
        self._grid_lock.release()

    def _get_grid(self):
        self._grid_lock.acquire()
        grid = self._grid
        self._grid_lock.release()
        return grid

    def __odometryCb(self, msg):
        # print('******************************************', msg.pose.pose)
        self.agent = msg.pose

    def _set_agent_pos(self, odom_msg):  # The functions here don't get evaluated.
        # print('agent', self.agent)
        self._agent_lock.acquire()
        self.agent = odom_msg
        self._agent_lock.release()

    def _get_agent_pos(self):
        self._agent_lock.acquire()
        ret = self.agent
        self._agent_lock.release()
        return ret

    @staticmethod
    def unit_vector(v):
        return v / np.linalg.norm(v)

    @staticmethod
    def angle(v1, v2):
        v1 = unit_vector(v1)
        v2 = unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))

    def _send_way_point(self, wp):
        """
        self._wp_cond.acquire()
        self._wp_cond.wait()
        self._wp_cond.release()
        """

        self._cur_wp = wp

        goal_tresh = 0.1
        angle_tresh = 0.1
        sleep = 0.01
        max_it = 1000
        rot_multiplier = 5
        forward_multiplier = 0.5
        found = False

        self._current_wp = wp = self._grid_to_world(wp)
        wp = np.array(wp)
        rospy.loginfo("Sending waypoint: {}".format(wp))

        for _ in range(max_it):
            agent = self._get_agent_pos()
            agent_pos = np.array([agent.pose.position.x, agent.pose.position.y])
            q_agent_orientation = agent.pose.orientation
            q_agent_orientation = [q_agent_orientation.x, q_agent_orientation.y,
                                   q_agent_orientation.z, q_agent_orientation.w]
            agent_rot = m.euler_from_quaternion(q_agent_orientation, axes='sxyz')[0]

            goal_dir = wp - agent_pos
            goal_rot = np.arctan2(goal_dir[1], goal_dir[0])
            angle_left = np.sign(goal_rot - agent_rot) * (np.abs(goal_rot - agent_rot) % np.pi)
            dist_left = np.linalg.norm(goal_dir)

            # print()
            #print("Position: {}, Agent Angle: {}".format(agent_pos, agent_rot))
            #print("Angle Left: {}, Dist Left: {}".format(angle_left, dist_left))

            # rotate
            if not np.abs(angle_left) < angle_tresh:
                rot_speed = np.clip(angle_left * rot_multiplier, -1, 1)
                #print("Rotating with speed: {}".format(rot_speed))
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

        self._current_wp = None

        rospy.loginfo("Waypoint found: {}".format(found))

        # self._has_reached_way_point()

    """
    def _has_reached_way_point(self):
        self._wp_cond.acquire()
        self._wp_cond.notify()
        self._wp_cond.release()
    """

    def _send_vel_msg(self, vel=None, rot=None):
        '''
        send veloicty 
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

    def _update_requested(self):
        pass  # request slam

    def _setup_sim(self, agent_pos) -> Simulator:
        agent_pos = self._world_to_grid(agent_pos)  # converts the slam map to a gridworld, i.e 40000 ->128

        print("Agent Position: {}".format(agent_pos))

        config = Configuration()

        config.simulator_graphics = True
        config.simulator_write_debug_level = DebugLevel.LOW
        config.simulator_key_frame_speed = 0.1
        config.simulator_algorithm_type = WayPointNavigation  # AStarTesting # WayPointNavigation
        # config.simulator_algorithm_parameters = ([],
        #          {"global_kernel": (CombinedOnlineLSTM, ([], {})),
        #           "global_kernel_max_it": 30})
        config.simulator_algorithm_parameters = ([], {"global_kernel_max_it": 20, "global_kernel": (
            OnlineLSTM, ([], {"load_name": "caelstm_section_lstm_training_block_map_10000_model"}))})

        # config.simulator_algorithm_parameters = ([],
        #         {"global_kernel_max_it": 20,
        #         "global_kernel": (OnlineLSTM, ([], {"load_name": "tile_by_tile_training_uniform_random_fill_10000_block_map_10000_house_10000_model"}))})

        config.simulator_testing_type = WayPointNavigationTesting  # AStar # WayPointNavigationTesting  # BasicTesting
        # print('gets to 289 before attempting to make ros map')
        config.simulator_initial_map = RosMap(self.MAP_SIZE,
                                              Agent(agent_pos, radius=self.INFLATE),
                                              Goal(self.goal),
                                              self._get_grid,
                                              self._send_way_point,
                                              self._update_requested)

        s = Services(config)
        # print(s.algorithm.map)
        s.algorithm.map.request_update()
        sim = Simulator(s)
        return sim

    def _world_to_grid(self, pos, map_size=None, origin=None):
        '''
        converts from meters coordinates to grid coordinates (4000x4000)
        '''
        if not map_size:
            map_size = self.MAP_SIZE

        if not origin:
            origin = self.ORIGIN
        #map_size = 128
        #res = 0.05
        # print("ascsascsasacsacascsa", pos, map_size, origin)

        grid_pos = [pos[0] - origin[0], pos[1] - origin[1]]
        # print(grid_pos)
        grid_pos = [int(round(grid_pos[0] / self.REZ)),
                    int(round(grid_pos[1] / self.REZ))]
        # print(grid_pos)
        grid_pos[1] = self.SLAM_MAP.height - grid_pos[1] - 1
        # print(grid_pos)
        # print("FIN")
        return Point(*grid_pos)

    def _grid_to_world(self, pos, map_size: Size = None, origin=None):
        if not map_size:
            map_size = self.MAP_SIZE

        if not origin:
            origin = self.ORIGIN

        world_pos = [pos.x, self.SLAM_MAP.height - pos.y - 1]
        world_pos = [world_pos[0] * self.REZ + self.REZ * 0.5,
                     world_pos[1] * self.REZ + self.REZ * 0.5]
        world_pos = [world_pos[0] + origin[0],
                     world_pos[1] + origin[1]]
        return world_pos

    def _find_goal(self):
        rospy.loginfo("Starting Simulator")
        # self._send_way_point(goal)
        # This issue might have somethign to do with the mpa not being set,
        agent = self._get_agent_pos()
        agent_pos = [agent.pose.position.x, agent.pose.position.y]  # works now might need to fix syntax since I changed to use odom function instead
        print('agent pos', agent_pos)

        # This is returning empty (above line) for some reason
        sim = self._setup_sim(agent_pos)
        # This should return a list of lists

        # signal waypoint
        # self._has_reached_way_point()

        sim.start()

    def start(self):
        rospy.loginfo("Starting LSTM")

        self._find_goal()

        # rospy.spin()
