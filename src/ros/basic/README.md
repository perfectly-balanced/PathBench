# ROS 2D Occupancy Grid Publish-Subscriber Demo

Basic demonstration showing how to use `RosMap` with a static occupancy grid map.

### Installing dependencies

```bash
sudo apt-get install python3-roslaunch python3-rospy python3-nav-msgs python3-geometry-msgs
```

### Quick Start

In terminal (1), execute the following command to start the ROS master node:

```bash
roscore
```

In terminal (2), execute the following command to start the 2D `OccupancyGrid` publisher:

```bash
python3 ogm_publisher.py
```

In terminal (3), execute the following command to start PathBench with a 2D `OccupancyGrid` subscriber:

```bash
python3 ros.py
```

Note, wait a few seconds for the master node and publisher to start before executing the command for terminal (3).

For more information about how to use `ros.py`, execute the following:

```bash
python3 ros.py --help
```
