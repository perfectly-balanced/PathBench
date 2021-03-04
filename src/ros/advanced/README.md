# ROS Real-time Extension

This extension provides real-time support for visualisation, coordination and interaction with a physical robot.

## Quick Start

To demonstrate the usage of this extension we will use `turtlebot3`.

Required ROS packages:

```
turtlebot3_gazebo
turtlebot3_bringup
turtlebot3_slam
turtlebot3_description
gazebo_ros
```

The following will launch all necessary ROS nodes with `rviz` for visualising the robot, and will subsequently launch PathBench with the ROS Map.

```bash
python3 src/ros/advanced/main.py --headless --rviz
```

For more information, run the following, noting that unrecognised arguments in `main.py` will be forwarded to `ros.py`.

Example
```bash
python3 src/ros/advanced/main.py  --rviz --algorithm "A*" -m waffle
```

```bash
python3 src/ros/advanced/main.py --help
python3 src/ros/advanced/ros.py --help
```

There seems to be a common issue where Gazebo will hang then crash
when opened through PathBench. A current fix is to perform
```bash
killall gzserver
```

