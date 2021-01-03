## Quick Start

The following will launch all necessary ROS nodes with `rviz` for visualising the robot, and will subsequently launch PathBench with the ROS Map.

```bash
python3 src/ros/gazebo/main.py --headless --rviz
```

For more information, run the following, noting that unrecognised arguments in `main.py` will be forwarded to `ros.py`.

```bash
python3 src/ros/gazebo/main.py --help
python3 src/ros/gazebo/ros.py --help
```
