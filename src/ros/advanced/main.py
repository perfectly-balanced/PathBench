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
    atexit.register(kill_processes)
    this_dir = os.path.dirname(os.path.abspath(__file__))

    parser = argparse.ArgumentParser(prog="main.py",
                                     description="PathBench ROS 2D Real-time Extension",
                                     formatter_class=argparse.RawTextHelpFormatter)

    # run mode
    parser.add_argument("--run-path-bench", action="store_true", help="run PathBench exclusively, unless '--run-ros-nodes' is also specified")
    parser.add_argument("--run-ros-nodes", action="store_true", help="run external ros nodes exclusively, unless '--path-bench' is also specified")
    parser.add_argument("--headless", action="store_true", help="run the gazebo world headlessly")
    parser.add_argument("--rviz", action="store_true", help="run rviz")

    # configuration
    parser.add_argument("-m", "--model", choices=['burger', 'waffle', 'waffle_pi'], default="burger", help="turtlebot3 model")
    parser.add_argument("--rviz-file", default='{}/rviz/turtlebot3_model.rviz'.format(this_dir), help="configuration file for rviz")
    parser.add_argument("--gmapping-param-file", default='{}/param/turtlebot3_gmapping.yaml'.format(this_dir), help="parameter file for gmapping")

    args, rem_args = parser.parse_known_args()
    print("args:{}".format(args))

    os.environ["TURTLEBOT3_MODEL"] = args.model

    if args.run_path_bench == args.run_ros_nodes or args.run_ros_nodes:
        launch_process(['roscore'])
        time.sleep(2)

        if args.headless:
            launch_process(['roslaunch', '{}/launch/turtlebot3_gmapping.launch'.format(this_dir), 'param_file:={}'.format(args.gmapping_param_file)])
            launch_process(['roslaunch', '{}/launch/turtlebot3_house_no_x.launch'.format(this_dir)])
        else:
            launch_process(['roslaunch', 'turtlebot3_slam', 'turtlebot3_gmapping.launch'.format(this_dir),
                            'param_file:={}'.format(args.gmapping_param_file)])
            launch_process(['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_house.launch'])

        if args.rviz:
            # alternative methods of loading rviz up
            # launch_process(['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_gazebo_rviz.launch'])
            # launch_process(['roslaunch', 'turtlebot3_navigation', 'turtlebot3_navigation.launch'])
            launch_process(['roslaunch', '{}/launch/turtlebot3_agent.launch'.format(this_dir), 'open_rviz:=true', 'rviz_file:={}'.format(args.rviz_file)])
        else:
            launch_process(['roslaunch', '{}/launch/turtlebot3_agent.launch'.format(this_dir), 'open_rviz:=false'])

        if args.run_path_bench != args.run_ros_nodes:
            if rem_args:
                print("Unknown arguments specified", file=sys.stderr)
                return False

            while True:
                time.sleep(1)

    if args.run_path_bench == args.run_ros_nodes or args.run_path_bench:
        subprocess.check_call([sys.executable, "{}/ros.py".format(this_dir)] + rem_args)
    elif rem_args:
        print("Unknown arguments specified", file=sys.stderr)
        return False
    return True


if __name__ == '__main__':
    res = run()
    sys.exit(int(not res))
