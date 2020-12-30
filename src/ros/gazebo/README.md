The following is a simple method of launching all the necessary nodes. However, this will launch a bunch of unnecessary (and comptuationally expensive) things such as the Gazebo simulator GUI.

```bash
roslaunch turtlebot3_slam turtlebot3_gmapping.launch
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
roslaunch turtlebot3_gazebo turtlebot3_house.launch
```

The customised method, which doesn't launch any extra graphical applications is

```bash
export WS=src/ros/gazebo
roslaunch ${WS}/turtlebot3_gmapping.launch param_file:=${WS}/turtlebot3_gmapping.yaml
roslaunch ${WS}/turtlebot3_house_no_x.launch
roslaunch ${WS}/turtlebot3_agent.launch
```

`param_file` is optional, and defaults to the one found in the `turtlebot3_slam` package. In the above example we use our own configuration for gmapping.
