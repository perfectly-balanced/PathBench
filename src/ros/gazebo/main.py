import sys
import os
import argparse
import atexit
import subprocess
import time

# add 'PathBench/src' to system path for module imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from utility.process import launch_process, kill_processes  # noqa: E402

def run() -> bool:
    parser = argparse.ArgumentParser(prog="main.py",
                                     description="PathBench ROS 2D Real-time Extension",
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--run-path-bench", action="store_true", help="run PathBench exclusively, unless '--run-external-ros-nodes' is also specified")
    parser.add_argument("--run-external-ros-nodes", action="store_true", help="run external ros nodes exclusively, unless '--path-bench' is also specified")
    parser.add_argument("--headless", action="store_true", help="run the gazebo world headlessly")
    parser.add_argument("--rviz", action="store_true", help="run rviz")
    parser.add_argument("-m", "--model", choices=['burger', 'waffle', 'waffle_pi'], default="burger", help="turtlebot3 model")

    args = parser.parse_args()
    print("args:{}".format(args))

    atexit.register(kill_processes)

    this_dir = os.path.dirname(__file__)

    os.environ["TURTLEBOT3_MODEL"] = args.model

    if args.run_path_bench == args.run_external_ros_nodes or args.run_external_ros_nodes:
        launch_process(['roscore'])
        time.sleep(2)

        if args.headless:
            launch_process(['roslaunch', '{}/turtlebot3_gmapping.launch'.format(this_dir), 'param_file:={}/turtlebot3_gmapping.yaml'.format(this_dir)])
            launch_process(['roslaunch', '{}/turtlebot3_house_no_x.launch'.format(this_dir)])
        else:
            launch_process(['roslaunch', 'turtlebot3_slam', 'turtlebot3_gmapping.launch'.format(this_dir), 'param_file:={}/turtlebot3_gmapping.yaml'.format(this_dir)])
            launch_process(['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_house.launch'])
        
        if args.rviz:
            # alternative -> launch_process(['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_gazebo_rviz.launch'])
            launch_process(['roslaunch', 'turtlebot3_navigation', 'turtlebot3_navigation.launch'])
        else:
            launch_process(['roslaunch', '{}/turtlebot3_agent.launch'.format(this_dir)])

        if args.run_path_bench != args.run_external_ros_nodes:
            while True:
                time.sleep(1)

    if args.run_path_bench == args.run_external_ros_nodes or args.run_path_bench:
        subprocess.check_call([sys.executable, "{}/ros.py".format(this_dir)])
    return True

if __name__ == '__main__':
    res = run()
    sys.exit(int(not res))
